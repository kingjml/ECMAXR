# ECCC Multi-application X-band Radar (ECMAXR)
# J. King 2020
# Controller: Adafruit Feather M4
# Radar: SiversIMA RS3400X FMCW
# GPS: Adafruit Ultimate GPS
# IMU: Adafruit BNO055

import time
import board
import busio
from digitalio import DigitalInOut, Direction, Pull
import adafruit_bno055
import neopixel
import adafruit_sdcard
import storage

# SD Card Init
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = DigitalInOut(board.D10)
sdcard = adafruit_sdcard.SDCard(spi, cs)
vfs = storage.VfsFat(sdcard)
storage.mount(vfs, "/sd")
sd_error = 0

# Status LEDs
pixel = neopixel.NeoPixel(board.NEOPIXEL, 1, brightness=0.5, auto_write=True)
pixel.fill((0, 0, 255)) # Blue status while booting

# This function constructs a JSON-like object
def write_data(file_name, data):
    #gc.collect()
    error_code = update_status(True)
    out_str = str({'Id': radar.count,
                   'Error': error_code,
                   'Time': gps.utc_time,
                   'Date': gps.utc_date,
                   'Latitude': gps.lat,
                   'Longitude': gps.lon,
                   'Quaternion': list(imu.quaternion),
                   'Temperature': imu.temperature,
                   'Data': data}).replace("'", "\"")

    with open(file_name, 'a') as output:
        output.write(out_str)
        output.write(',')
    return 0

def update_status(verbose=False):
    error_code = int(str(1) + str(radar.status) + str(imu.status)+ str(gps.status)+str(gps.fix))

    if verbose:
        print('TRIG %i, RADAR %i, IMU %i, GPS %i, FIX %i' % (radar.count,
                radar.status, imu.status, gps.status, gps.fix))

    if error_code > 10001:
        pixel.fill((255, 0, 0))
        print('ERROR {}'.format(error_code))
    elif error_code == 10001:
        pixel.fill((255, 255, 0))
        print('NO GPS FIX')
    else:
        pixel.fill((0, 255, 0))

    return error_code

# Radar class
class RADAR():
    def __init__(self, tx, rx):
        # Change settings here
        self.cmd = {
            'init': b'INIT',
            'measure': b'SWEEP:MEASURE ON',
            'sweep': b'SWEEP:NUMBERS 1',
            'trig': b'TRIG:ARM',
            'data': b'TRACE:DATA ?'
        }
        self.uart = busio.UART(tx,
                               rx,
                               baudrate=115200,
                               receiver_buffer_size=64)
        self.count = -1
        self.trig = 0
        self.last_trig = -9999

        self.check()
        self.write(self.cmd['init'], True, False)
        self.write(self.cmd['measure'], True, False)
        self.write(self.cmd['sweep'], True, False)

    # Serial write to radar
    def write(self, cmd, read_now=False, debug=False):
        if debug: print(cmd)
        self.uart.reset_input_buffer()
        self.uart.write(cmd+b'\r')
        if read_now:
            return self.read(debug)

    # Read seial return from radar
    def read(self, debug=False):
        data = self.uart.readline()
        if debug: print(data)
        if ('OK' in data):
            self.status = 0
            return 0
        elif data is None:
            self.status = 1
            return 1
        else:
            self.status = 0
            return data

    # Radar heartbeat
    def check(self):
        self.write(b'')
        data = self.read()
        if '?' in data:
            self.status = 0
            return 0
        else:
            self.status = 1
            return 1

    # Command the radar to fire and
    # read the data back over UART
    def fire(self):
        #pixel.fill((255, 255, 0))
        self.uart.reset_input_buffer()
        counter = 0
        buf_len = 13
        buf = bytearray(buf_len)
        mv = memoryview(buf)
        idx = 0
        dat_out = []

        # Send the trigger command to the radar
        # Note radar.status gets updated by doing this
        self.write(self.cmd['trig'], True, False)

        # Takes ~42 ms for the FMCW ramp to finish
        time.sleep(0.5)

        self.uart.reset_input_buffer()
        if radar.status == 0:
            self.write(self.cmd['data'], False, False)
            while True:
                while idx < len(buf):
                    if self.uart.in_waiting > 0:
                        bytes_read = self.uart.readinto(mv[idx:])
                        idx += bytes_read
                idx = 0
                counter += 1
                radar_amplitude = float(''.join([chr(b) for b in buf]))
                dat_out.append(radar_amplitude)

                if counter == 1501:
                    self.count += 1
                    self.trig = 0
                    break
            return dat_out
        else:
            self.trig = 0
            return 1

# GPS Class
class GPS():
    def __init__(self, tx, rx):
        self.uart = busio.UART(tx, rx, baudrate=9600, timeout=3)
        self.fix = 1
        self.status = 1
        self.last = None
        self.update(debug = False)

    # Get GPS sentance and parse
    # Will throw a 1 with timeout of 3000 ms
    def update(self, timeout=3000, debug=False):
        c_time = time.monotonic()
        while True:
            l_time = time.monotonic()
            sentence = self.uart.readline()
            if '$GPRMC' in sentence:
                self.status = 0
                data = (str(sentence, 'ascii').strip()).split(',')
                if data[3] == '':
                    self.fix = 1  # No GPS fix
                else:
                    self.fix = 0
                    self.last = l_time
                    self.utc_time = data[1].split('.')[0]
                    self.utc_date = data[9]
                    self.lat = float(data[3])
                    self.lon = float(data[5])
                    if debug: print(data)
                    return 0
            elif l_time-c_time > timeout:
                self.status = 1
                self.fix = 1
                if debug: print('GPS Timeout')
                return 1

    def print(self):
        if self.fix == 0:
            print('Timestamp: %s,%s' % (self.utc_date,self.utc_time))
            print('Coordinates: %f,%f' %(self.lat, self.lon))
        else:
            print('NO FIX')

class IMU():
    def __init__(self, i2c):
        try:
            self.sensor = adafruit_bno055.BNO055(i2c)
        except:
            self.status = 1
        self.quaternion = None
        self.temperature = None
        time.sleep(0.5) #Needs a few ticks to wake
        self.update()

    def update(self, verbose=False):
        try:
            self.quaternion = self.sensor.quaternion
            self.temperature = self.sensor.temperature
            self.status = 0
            if verbose: self.print()
        except:
            self.status = 1
            if verbose: print('IMU FAILED')

    def print(self):
        print('Temperature: {}'.format(self.temperature))
        print('Quaternion: {}'.format(self.quaternion))

# System init
print('ECMAXR V0.1\n')
gps = GPS(board.TX, board.RX)
i2c = busio.I2C(board.SCL, board.SDA)
imu = IMU(i2c)
radar = RADAR(board.A2, board.A3)

file_name = '/sd/%s_%s.dat' % (gps.utc_date, gps.utc_time)
print('Writing to file: {}'.format(file_name))

last_trig = time.monotonic()
waiting_flag = False
while True:
    current = time.monotonic()
    if (current-last_trig > 1.0):
        gps.update(debug = False)
        imu.update(verbose = False)
        write_data(file_name,radar.fire())

        last_trig = time.monotonic()
        waiting_flag = False

    elif not waiting_flag:
        print('Waiting...')
        waiting_flag = True