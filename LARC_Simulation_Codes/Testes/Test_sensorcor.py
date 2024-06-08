import math
import struct
from time import sleep
import cv2
import easyocr as ocr
import numpy as np
from controller import Robot

#variaveis
cor_buraco = b'...\xff'
tem_buraco = False
robot = Robot() # Create robot object
timeStep = 32   # timeStep = number of milliseconds between world updates


sensor_de_cor = robot.getDevice("colour_sensor")
sensor_de_cor.enable(timeStep)

def tem_buraco():
    if sensor_de_cor == cor_buraco:
        return True
    return False

while robot.step(timeStep) != -1:

    image = sensor_de_cor.getImage() # Step 4: Retrieve the image frame.
    # Get the individual RGB color channels from the pixel (0,0)
    # Note that these functions require you to pass the width of the overall image in pixels.
    # Since this is a 1px by 1px color sensor, the width of the image is just 1. 
    r = sensor_de_cor.imageGetRed(image, 2, 0, 0)
    g = sensor_de_cor.imageGetGreen(image, 2, 0, 0)
    b = sensor_de_cor.imageGetBlue(image, 2, 0, 0)
    print("r: " + str(r) + " g: " + str(g) + " b: " + str(b))
    