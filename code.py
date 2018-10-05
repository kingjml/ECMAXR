# ECCC Multi-application X-band Radar (ECMAXR)
# J. King 2018

import time
import board
import busio
import adafruit_sdcard
import storage
from digitalio import DigitalInOut, Direction, Pull

# GPS Class
class GPS():
    def __init__(self, tx, rx):
        self.uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=3000)
        self.fix = 1
        self.status = 1
        self.update()

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
                    break
            elif l_time-c_time > timeout:
                print('GPS Timeout')
                break
                
# Radar class
class RADAR():
    def __init__(self, tx, rx):
        # Change settings here
        self.cmd = {
            "init": "INIT",
            "measure": "SWEEP:MEASURE ON",
            "sweep": "SWEEP:NUMBERS 1",
            "trig": "TRIG:ARM",
            "data": "TRACE:DATA ?"
        }
        self.uart = busio.UART(tx, 
                               rx, 
                               baudrate=115200, 
                               receiver_buffer_size=64)
        self.count = -1
        self.trig = 0
        self.last_trig = time.monotonic()
           
        self.check()
        self.write(self.cmd['init'], True, False)
        self.write(self.cmd['measure'], True, False)
        self.write(self.cmd['sweep'], True, False)

    def write(self, cmd, read_now=False, debug=False):
        if debug: print(cmd)
        self.uart.reset_input_buffer()
        self.uart.write(cmd+'\r')
        if read_now:
            return self.read(debug)
        
    def read(self, debug=False):
        data = self.uart.readline()
        if debug: print(data)
        if ('OK' in data):
            return 0
        elif data is None:
            return 1
        else:
            return data
        
    def check(self):
        self.write('')
        data = self.read()
        if '?' in data:
            self.status = 0
            return True
        else:
            self.status = 1
            return False
            
    def fire(self):
        self.uart.reset_input_buffer()
        counter = 0
        buf_len = 13 
        buf = bytearray(buf_len)
        mv = memoryview(buf)
        idx = 0
        dat_out = []
        
        self.write(self.cmd['trig'], True, False)
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
                dat_out.append(float(''.join([chr(b) for b in buf])))
    
                if counter == 1501:
                    self.count += 1
                    self.trig = 0
                    break
            return dat_out
        else:
            self.trig = 0
            return 1
  
# SD Card Init
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = DigitalInOut(board.D10)
sdcard = adafruit_sdcard.SDCard(spi, cs)
vfs = storage.VfsFat(sdcard)
storage.mount(vfs, "/sd")

# Init buttons
radar_switch = DigitalInOut(board.D6)
radar_switch.direction = Direction.INPUT
radar_switch.pull = Pull.UP

# Write to SD
def write_data(data):
    out_str = str({'Id': radar.count, 
                   'Time': gps.utc_time,
                   'Date': gps.utc_date,
                   'Latitude': gps.lat,
                   'Longitude': gps.lon,
                   'Data': data}).replace("'", "\"")
    
    w_max = 1000
    w_remaining  = len(out_str)
            
    with open(file_name, 'a') as output:
        if radar.count>0:
            bytes_out = output.write(',')
        while w_remaining >= w_max:
            s_index =  len(out_str) - w_remaining
            bytes_out = output.write(out_str[s_index: s_index+w_max])
            w_remaining -= w_max
        bytes_out = output.write(out_str[-w_remaining:])
    return 0


# Main
gps = GPS(board.TX, board.RX)
gps.update(debug=False)
if (gps.status == 0): print('GPS READY')

file_name = '/sd/%s_%s.rad' % (gps.utc_date, gps.utc_time)

radar = RADAR(board.A2, board.A3)
if (radar.status == 0): print('RADAR READY')

while True:
    current = time.monotonic()
    if (not radar_switch.value) & (current-radar.last_trig > 1.0):
        radar.last_trig = time.monotonic()
        print('RADAR TRIG %i' % radar.count)
        gps.update(debug=False)
        bytes_out = write_data(radar.fire())        
        print('WROTE %i OK' % bytes_out)