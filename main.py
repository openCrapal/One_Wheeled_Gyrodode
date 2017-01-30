#!/usr/bin/python3.4
# -*-coding:utf-8 -*

from constants import *
import time
from i2cDev import i2c_devices
import localisation
import sys
import signal
import math
from go_robot import Robot
import pygame
import pygame.camera
  
from pygame.locals import *

pygame.init()
pygame.camera.init()
clock = pygame.time.Clock()

screen = pygame.display.set_mode((640, 480))
screen.fill((255,0,0))

cam = pygame.camera.Camera("/dev/video0",(640,480))
cam.start()


myI2cDev = i2c_devices()
myI2cDev.save_gyro_offset(500)
myLoc = localisation.loc()
myI2cDev.start()
myLoc.start()

def fermer_pgrm():
    print("fermer proprement")
    global myLoc, myI2cDev
    import RPi.GPIO as GPIO
    GPIO.cleanup()
    myLoc.finish()
    myI2cDev.finish()
    cam.stop()
    time.sleep(0.1)
    del myLoc
    del myI2cDev
    sys.exit(0)
def fermer_pgrm_sign(signal, frame):
	fermer_pgrm()
signal.signal(signal.SIGINT, fermer_pgrm_sign)


time.sleep(0.2)
myRobot = Robot(myI2cDev, myLoc)

screen = pygame.display.set_mode((640,480))

t0 = time.time()
done = False
active = False
while not done:
    screen.blit(cam.get_image(), (0,0))
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
            
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                done = True
            if event.key == pygame.K_F1 and not active:
                myRobot.start()
                active = True
            
            if event.key == pygame.K_F2 and active:
                myRobot.finish()
                active = False

        if event.type == pygame.KEYUP:
            if event.key == pygame.K_LEFT or event.key == pygame.K_RIGHT:
                x_change = 0
                
    pygame.display.update()
    clock.tick(5)

fermer_pgrm()

