import serial
from xbee import ZigBee

PORT = '/dev/tty.usbserial-000013FA'
BAUD_RATE = 115200

xbee_serial = serial.Serial(PORT, BAUD_RATE)

xbee_serial.write('b')

while True:
    print xbee_serial.read()
