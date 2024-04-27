from controller import Robot
from time import sleep
import math
import struct
import cv2
import numpy as np


erro_anterior = 0
soma_erros = 0
cor_buraco = b'...\xff'
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
raio_da_roda = 0.0205
tamanho_tile = 0.12
comprimento_da_roda = 2*math.pi*raio_da_roda
x = tamanho_tile/comprimento_da_roda
n = 1
k = 2
direcao = ""
tile = 5.81343

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

sensoresFrente = [robot.getDevice("ps_frente"), robot.getDevice("ps_tras")]
sensoresEsquerda = [robot.getDevice("ps_esquerda"), robot.getDevice("ps_diagonal_esquerda")]
sensoresDireita = [robot.getDevice("ps_direita"), robot.getDevice("ps_diagonal_direita")]

# Ativar sensores
for sensor in sensoresFrente + sensoresEsquerda + sensoresDireita:
    sensor.enable(timeStep)

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


def mover_para_frente(dist):
    while robot.step(timeStep) != -1:
        motorEsquerdo.setVelocity(maxVelocity)
        motorDireito.setVelocity(maxVelocity)
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
    global posicao_atual, posicaoX_atual, posicaoY_atual
    global deltaX,deltaY
    global coordenada_coluna_atual, coordenada_linha_atual
    posicaoX_anterior = 0
    posicao_atual = gps.getValues()
    posicaoX_atual = posicao_atual[0]
    posicaoY_atual = posicao_atual[2]
    
    deltaX = posicaoX_atual - posicaoX_anterior
    deltaY = posicaoY_atual - posicaoY_anterior
    if start:
        print(f"antes: {round(deltaX, 2)}")
        posicaoX_anterior = gps.getValues()[0]
        mover_para_frente(tile)
        print(f"depois: {round(gps.getValues()[0] - posicaoX_anterior,2)}")
        start = False
    parar()
    



