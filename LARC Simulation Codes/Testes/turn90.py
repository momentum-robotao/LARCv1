from controller import Robot
from time import sleep
import math
import struct
import cv2
import numpy as np


erro_anterior = 0
soma_erros = 0
cor_buraco = b'...\xff'
PI = 3.141592653589793
timeStep = 32
maxVelocity = 6.28
encoder_inicial = 0.0
tem_buraco = False
andou = 0.0
rad = 4.29552
mensagem_enviada = False
posicao_anterior = (0,0,0)
posicaoX_anterior = 0
posicaoY_anterior = 0
tamanho_mapa = 501
coordenada_centro_mapa = tamanho_mapa//2
initial_angle = 0

n = 1
k = 2
direcao = ""

robot = Robot()
motorEsquerdo = robot.getDevice('left motor')
motorDireito = robot.getDevice('right motor')
encoders = motorEsquerdo.getPositionSensor()

# cameraE
cameraE = robot.getDevice("cameraE")
cameraE.enable(timeStep)

'''cameraE = robot.getDevice('cameraD')
cameraE.enable(timeStep)    

cameraE = robot.getDevice('cameraE')
cameraE.enable(timeStep)'''

sensor_de_cor = robot.getDevice("colour_sensor")
sensor_de_cor.enable(timeStep)

emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
receiver.enable(timeStep)

gps = robot.getDevice("gps")
gps.enable(timeStep)
position = gps.getValues()
victimTypeH = bytes('H', "utf-8")# The victim type being sent is the letter 'H' for harmed victim

imu = robot.getDevice("inertial_unit")
imu.enable(timeStep)



# Inicializar motores e sensores
motorEsquerdo.setPosition(float("inf"))
motorDireito.setPosition(float("inf"))
motorEsquerdo.setVelocity(0.0)
motorDireito.setVelocity(0.0)
encoders.enable(timeStep)

def encoder_antes():
    # Retorna o valor do encoder antes de mover
    global encoder_inicial
    encoder_inicial = float(encoders.getValue())
    return encoder_inicial


def mover_para_tras(dist):
    while robot.step(timeStep) != -1:
        global andou
        motorEsquerdo.setVelocity(-maxVelocity)
        motorDireito.setVelocity(-maxVelocity)
        if encoders.getValue() - encoder_inicial < -dist:
            parar()
            break


def mover_para_frente(dist=6):
    while robot.step(timeStep) != -1:
        motorEsquerdo.setVelocity(maxVelocity)
        motorDireito.setVelocity(maxVelocity)
        if tem_buraco():
            andou = encoders.getValue() - encoder_inicial
            mover_para_tras(andou - 0.35)
            encoder_antes()
            virar_180()
            break
        if encoders.getValue() - encoder_inicial > dist:
            parar()
            break


def parar():
    motorEsquerdo.setVelocity(0.0)
    motorDireito.setVelocity(0.0)

def imu_antes():
    # Retorna o valor do encoder antes de mover
    global initial_angle
    initial_angle = imu.getRollPitchYaw()[2]
    return initial_angle

def virar_180():
    # Vira 180 graus
    while robot.step(timeStep) != -1:
        motorEsquerdo.setVelocity(maxVelocity)
        motorDireito.setVelocity(-maxVelocity)
        if abs(initial_angle - imu.getRollPitchYaw()[2]) >= math.pi:
            parar()
            break


# Função para virar à esquerda por 90 graus
def virar_esquerda():
    while robot.step(timeStep) != -1:
        motorEsquerdo.setVelocity(-maxVelocity)
        motorDireito.setVelocity(maxVelocity)
        if abs(initial_angle - imu.getRollPitchYaw()[2]) >= 1.5692036781036944:
            parar()
            break


# Função para virar à direita por 90 graus
def virar_direita():
    while robot.step(timeStep) != -1:
        motorEsquerdo.setVelocity(maxVelocity)
        motorDireito.setVelocity(-maxVelocity)
        if abs(initial_angle - imu.getRollPitchYaw()[2]) >= 1.5692036781036944:
            parar()
            break

start = True

while robot.step(timeStep) != -1:
    imu_antes()
    ''' 
    if start:
        print(f"antes: {imu.getRollPitchYaw()[2]}")
        virar_direita()
        print(f"depois: {imu.getRollPitchYaw()[2]}")
        start = False
    parar()
    '''
    print(imu.getRollPitchYaw()[2])

