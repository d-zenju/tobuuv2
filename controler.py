# -*- coding: utf-8 -*-

import serial
import os
from time import sleep
import sys

ser = serial.Serial()

# baud rate
ser.baudrate = 38400

# mac OSX only
for file in os.listdir('/dev'):
    if "tty.usbmodem" in file:
        ser.port = '/dev/' + file
        ser.open()


while True:
    ser.write("RXT1500,1600")
    print ser.readline()

#while True:
#    ser.write('RXT')
#    print ser.readline()
#    sleep(1)

    # read line
    #line = sys.stdin.readline()

    # Quit
    #if line == ".q\n":
    #    break

    #sendline = "RXT" + line
    #ser.write(sendline)

    #print ser.readline()

# serial close
#er.close()