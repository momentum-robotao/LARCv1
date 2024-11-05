import math
import struct
import numpy as np
import cv2
from controller import Robot



timeStep = 32

robot = Robot()
motorEsquerdo = robot.getDevice("left motor")
motorDireito = robot.getDevice("right motor")
encoders = motorEsquerdo.getPositionSensor()

# cameraE
cameraE = robot.getDevice("cameraE")
cameraE.enable(timeStep)
cameraD = robot.getDevice("cameraD")
cameraD.enable(timeStep)

lidar = robot.getDevice("lidar")
lidar.enable(timeStep)

emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
receiver.enable(timeStep)

gps = robot.getDevice("gps")
gps.enable(timeStep)
position = gps.getValues()
victimTypeH = bytes("H", "utf-8")  # Victim type 'H' for harmed victim

imu = robot.getDevice("inertial_unit")
imu.enable(timeStep)

ds1 = robot.getDevice("ds1")
ds1.enable(timeStep)
ds2 = robot.getDevice("ds2")
ds2.enable(timeStep)

colorSensor = robot.getDevice("colour_sensor")
colorSensor.enable(timeStep)

# Inicializar motores e sensores
motorEsquerdo.setPosition(float("inf"))
motorDireito.setPosition(float("inf"))
motorEsquerdo.setVelocity(0.0)
motorDireito.setVelocity(0.0)
encoders.enable(timeStep)

def tem_buraco():
    distance_left = float(ds1.getValue())
    distance_right = float(ds2.getValue())

    if distance_left > 0.2 and distance_right > 0.2:
        print('buraco frente')
    elif distance_left > 0.2:
        print('buraco direita')
    elif distance_right > 0.2:
        print('buraco esquerda')
    print()


def transition_tile():
    #blue:room1<-->room2
    #purple:room2<-->room3
    #red:room3<-->room4
    #green:room1<-->room4
    image = colorSensor.getImage()
    r = colorSensor.imageGetRed(image, 1, 0, 0)
    g = colorSensor.imageGetGreen(image, 1, 0, 0)
    b = colorSensor.imageGetBlue(image, 1, 0, 0)
    #print(cor)
    if r==88 and g==88 and b==255:
        print("blue") 
    elif r==189 and g==88 and b==250:
        print("purple")
    elif r==255 and g==88 and b==88:
        print("red")
    elif r==47 and g==255 and b==47:
        print("green")
    elif r==255 and g==250 and b==88:
        print("orange")
    elif r==255 and g==255 and b==88:
        print("yellow")
    elif r==242 and g==218 and b==138:
        print("swamp")
    elif r==255 and g==255 and b==255:
        print("checkpoint")


def verifica_ambiente():
    transition_tile()
    tem_buraco()
    
while robot.step(timeStep) != -1:
    verifica_ambiente()
