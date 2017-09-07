# -*- coding: utf-8 -*-

import pygame
from pygame.locals import *
import serial
import signal
import os
import sys
import time


def main():
    # PYGAME
    pygame.init()   # pygame init
    screen = pygame.display.set_mode((400, 300))    # display size
    pygame.display.set_caption("TOBUUV Controler")  # title
    font = pygame.font.Font(None, 20)   # font size

    # SERIAL
    ser = serial.Serial()   # serial init
    ser.baudrate = 38400    # baud rate
    ser.timeout = 0.5       # timeout
    for file in os.listdir('/dev'):
        if "tty.usbmodem" in file:
            ser.port = '/dev/' + file
            ser.open()

    right_pulse = 1500
    left_pulse = 1500
    pulse = [1180, 1500, 1820]

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
            pygame.quit()
            ser.close()
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
            ser.write('RXT')
            ser.write(str(right_pulse) + str(left_pulse))
            start = time.time()


        # event action
        for event in pygame.event.get():
            if event.type == QUIT:  # Quit
                pygame.quit()
                ser.close()
                sys.exit()


if __name__ == '__main__':
    main()
