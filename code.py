# Imports
import time
import board
import busio
import adafruit_ssd1306
import adafruit_sdcard
import storage
from digitalio import DigitalInOut, Direction, Pull

# State vars
radar_trigger = False
radar_counter = 0

# GPS Initallize
gps_uart = busio.UART(board.TX, board.RX, baudrate = 9600, timeout=3000)

# SD Card Initialize
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = DigitalInOut(board.D10)
sdcard = adafruit_sdcard.SDCard(spi, cs)
vfs = storage.VfsFat(sdcard)
storage.mount(vfs, "/sd")

# Define display
i2c = busio.I2C(board.SCL, board.SDA)
reset_pin = DigitalInOut(board.D5)
oled = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c, reset=reset_pin)

def oled_write(text_out):
    oled.fill(0)
    oled.text(text_out, 0, 0)
    oled.show()

oled_write('ECCC X-Band FMCW')


# Define buttons
radar_switch = DigitalInOut(board.D6)
radar_switch.direction = Direction.INPUT
radar_switch.pull = Pull.UP
last_trigger = time.monotonic()

# Setup radar UART and clear read buffer
radar_tx = board.A2
radar_rx = board.A3
radar_uart = busio.UART(radar_tx, 
                        radar_rx, 
                        baudrate=115200, 
                        receiver_buffer_size=64)
                        
radar_uart.reset_input_buffer()

def check_radar(verbose=False):
    radar_uart.write('\r')
    uart_data = radar_uart.readline()
    radar_uart.reset_input_buffer()
    if verbose:
        print(uart_data)
    if '?' in uart_data:
        return 0
    else:
        return 1
        
def radar_write(cmd, uart, buf_len=4, verbose=False):
    uart.write(cmd+'\r')
    uart_data = uart.read(buf_len)
    uart.reset_input_buffer()
    if verbose:
        print(uart_data)
    if 'OK' in uart_data:
        return 0
    else:
        return 1
        
def radar_fire():
    radar_uart.reset_input_buffer()
    counter = 0
    buf_len = 13 
    buf = bytearray(buf_len)
    mv = memoryview(buf)
    idx = 0
    dat_out = []

    radar_status = radar_write('TRIG:ARM', radar_uart, 4, False)
    print(radar_status)
    time.sleep(0.5)
    #radar_status = radar_write('TRACE:DATA ?', radar_uart, 4, False)
    radar_uart.write('TRACE:DATA ?\r')
    
    while True:
        while idx < len(buf):
            if radar_uart.in_waiting > 0:
                bytes_read = radar_uart.readinto(mv[idx:])
                idx += bytes_read
        idx = 0
        counter +=1
        dat_out.append(float(''.join([chr(b) for b in buf])))
    
        if counter == 1501:
            break
    print('RADAR READY')
    return dat_out
        
# Initialize the radar
print("RADAR INIT")
radar_write('INIT', radar_uart, 4, False)
radar_status = check_radar(False)
radar_status += radar_write('SWEEP:MEASURE ON', radar_uart, 4, False)
radar_status += radar_write('SWEEP:NUMBERS 1', radar_uart, 4, False)
if radar_status > 0:
    oled_write('RADAR ERROR')
    print('RADAR ERROR')
else:
    oled_write('RADAR READY')
    print('RADAR READY')

while True:
    current = time.monotonic()

    if radar_trigger:
        # Wait for GPS data TODO: Process $GPGGA
        while True: # TODO write timeout
            sentence = gps_uart.readline()
            if '$GPRMC' in sentence:
                sentence = str(sentence, 'ascii').strip()
                data = sentence.split(',')
                utc_time = int(float(data[1]))
                utc_date = int(float(data[9]))
                lat = float(data[3])
                lon = float(data[5])
                #print(data)
                #print(utc_date)
                break
        oled_write('RADAR TRIG')
        print('RADAR TRIG')
        radar_trigger = False
        with open("/sd/test_file.txt", "a") as f:
            if radar_counter > 0:
                f.write(',')
            out_str = str({'ID' : radar_counter, 
                           'Time' : utc_time,
                           'Date' : utc_date,
                           'Latitude' : lat,
                           'Longitude' : lon,
                           'Data' : radar_fire()}).replace("'", "\"")
            f.write(out_str)
        radar_counter += 1
        oled_write('RADAR READY')

    if (not radar_switch.value) & (current-last_trigger>1.0):
        last_trigger = time.monotonic()
        oled_write('RADAR READY')
        radar_trigger = True
