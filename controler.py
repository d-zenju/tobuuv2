# -*- coding: utf-8 -*-

import pygame
from pygame.locals import *
import serial
import signal
import os
import sys
import time
import threading
import math


class SerialThread():
    
    def __init__(self):
        self.stop_event = threading.Event()
        self.message = ''

        # SERIAL(MAC ONLY)
        self.ser = serial.Serial()   # serial init
        self.ser.baudrate = 38400    # baud rate
        self.ser.timeout = 0.5       # timeout
        for self.file in os.listdir('/dev'):
            if "tty.usbmodem" in self.file:
                self.ser.port = '/dev/' + self.file
                self.ser.open()
        
        self.t_read = threading.Thread(target = self.read)
        self.t_read.start()

    def stop(self):
        self.write('15001500')
        self.stop_event.set()
        self.t_read.join()
        self.ser.close()

    def read(self):
        while not self.stop_event.is_set():
            self.message = self.ser.readline()
            print self.message
    
    def write(self, message):
        self.ser.write(message)

    def read_message(self):
        return self.message


def main():
    # PYGAME
    pygame.init()   # pygame init
    screen = pygame.display.set_mode((400, 300))    # display size
    pygame.display.set_caption("TOBUUV Controler")  # title
    font = pygame.font.Font(None, 20)   # font size

    t_serial = SerialThread()   # serial thread

    # pulse parameter
    right_pulse = 1500
    left_pulse = 1500
    pulse = [1180, 1500, 1820]  # min, midship, max pulse

    start = time.time()

    while(1):
        pressed_key = pygame.key.get_pressed()
        if pressed_key[K_UP] and pressed_key[K_DOWN]:   # UP & DONW: Right Midship
            right_pulse = pulse[1]
        elif pressed_key[K_UP]:   # UP: Right UP
            right_pulse += 1
        elif pressed_key[K_DOWN]: # DOWN: Right Down
            right_pulse -= 1
        
        if pressed_key[K_w] and pressed_key[K_s]:   # W & S: Left Midship
            left_pulse = pulse[1]
        elif pressed_key[K_w]:    # W: Left UP
            left_pulse += 1
        elif pressed_key[K_s]:    # S: Left Down
            left_pulse -= 1
        
        if pressed_key[K_m]:    # M: Right & Left Midship
            right_pulse = pulse[1]
            left_pulse = pulse[1]

        if pressed_key[K_ESCAPE]:   # ESCAPE: QUIT
            t_serial.stop()
            pygame.quit()
            sys.exit()

        # thruster max, min
        if right_pulse > pulse[2]:
            right_pulse = pulse[2]
        if right_pulse < pulse[0]:
            right_pulse = pulse[0]
        if left_pulse > pulse[2]:
            left_pulse = pulse[2]
        if left_pulse < pulse[0]:
            left_pulse = pulse[0]


        pygame.display.update() # update
        pygame.time.wait(30)    # update time
        screen.fill((0, 0, 0))  # background color
        
        # read message to TEXT
        message = t_serial.read_message()   # read serial message
        agtm = message.split(',')   # split
        
        text = font.render('Accel[x]: ', True, (255, 255, 255))
        screen.blit(text, [20, 80])
        text = font.render('Accel[y]: ', True, (255, 255, 255))
        screen.blit(text, [20, 100])
        text = font.render('Accel[z]: ', True, (255, 255, 255))
        screen.blit(text, [20, 120])
        text = font.render('Gyro[x]: ', True, (255, 255, 255))
        screen.blit(text, [20, 140])
        text = font.render('Gyro[y]: ', True, (255, 255, 255))
        screen.blit(text, [20, 160])
        text = font.render('Gyro[z]: ', True, (255, 255, 255))
        screen.blit(text, [20, 180])
        text = font.render('Magnet[x]: ', True, (255, 255, 255))
        screen.blit(text, [20, 200])
        text = font.render('Magnet[y]: ', True, (255, 255, 255))
        screen.blit(text, [20, 220])
        text = font.render('Magnet[z]: ', True, (255, 255, 255))
        screen.blit(text, [20, 240])
        text = font.render('Temp: ', True, (255, 255, 255))
        screen.blit(text, [20, 260])
        text = font.render('Yaw: ', True, (255, 255, 255))
        screen.blit(text, [200, 80])
        text = font.render('Pitch: ', True, (255, 255, 255))
        screen.blit(text, [200, 100])
        text = font.render('Roll: ', True, (255, 255, 255))
        screen.blit(text, [200, 120])
        if (len(agtm) == 13):
            text = font.render(str(agtm[0]), True, (255, 255, 255))
            screen.blit(text, [100, 80])
            text = font.render(str(agtm[1]), True, (255, 255, 255))
            screen.blit(text, [100, 100])
            text = font.render(str(agtm[2]), True, (255, 255, 255))
            screen.blit(text, [100, 120])
            text = font.render(str(agtm[3]), True, (255, 255, 255))
            screen.blit(text, [100, 140])
            text = font.render(str(agtm[4]), True, (255, 255, 255))
            screen.blit(text, [100, 160])
            text = font.render(str(agtm[5]), True, (255, 255, 255))
            screen.blit(text, [100, 180])
            text = font.render(str(agtm[6]), True, (255, 255, 255))
            screen.blit(text, [100, 200])
            text = font.render(str(agtm[7]), True, (255, 255, 255))
            screen.blit(text, [100, 220])
            text = font.render(str(agtm[8]), True, (255, 255, 255))
            screen.blit(text, [100, 240])
            text = font.render(str(agtm[9]), True, (255, 255, 255))
            screen.blit(text, [280, 80])
            text = font.render(str(agtm[10]), True, (255, 255, 255))
            screen.blit(text, [280, 100])
            text = font.render(str(agtm[11]), True, (255, 255, 255))
            screen.blit(text, [280, 120])
            text = font.render(str(agtm[12]), True, (255, 255, 255))
            screen.blit(text, [100, 260])

        # TEXT
        text = font.render('Thruster', True, (255, 255, 255))
        screen.blit(text, [20, 20])
        text = font.render('Left: ', True, (255, 255, 255))
        screen.blit(text, [20, 40])
        text = font.render('Right: ', True, (255, 255, 255))
        screen.blit(text, [200, 40])
        text = font.render(str(right_pulse), True, (255, 0, 0))
        screen.blit(text, [250, 40])
        text = font.render(str(left_pulse), True, (0, 255, 0))
        screen.blit(text, [70, 40])

        # serial write
        elapsed_time = time.time() - start
        if elapsed_time > 0.2:
            t_serial.write(str(right_pulse) + str(left_pulse))
            start = time.time()

        # event action
        for event in pygame.event.get():
            if event.type == QUIT:  # Quit
                t_serial.stop()
                pygame.quit()
                sys.exit()


if __name__ == '__main__':
    main()