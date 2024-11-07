import math
import struct
import numpy as np
import cv2
from controller import Robot

# cameraE
cameraE = robot.getDevice("cameraE")
cameraE.enable(timeStep)
cameraD = robot.getDevice("cameraD")
cameraD.enable(timeStep)

colorSensor = robot.getDevice("colour_sensor")
colorSensor.enable(timeStep)

def transition_tile():
    #blue:room1<-->room2
    #purple:room2<-->room3
    #red:room3<-->room4
    #green:room1<-->room4



def verifica_ambiente():
    transition_tile()
    tem_buraco()
    
while robot.step(timeStep) != -1:
    verifica_ambiente()
