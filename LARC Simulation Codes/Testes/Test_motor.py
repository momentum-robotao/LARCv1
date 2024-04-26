from controller import *
from time import sleep

#variaveiss
maxVelocity = 6.28
andou = 0.0
robot = Robot() # Create robot object
timeStep = 32   # timeStep = number of milliseconds between world updates
encoder_inicial = 0.0


#Iniciar motores
motorEsquerdo = robot.getDevice('left motor')
motorDireito = robot.getDevice('right motor')
motorEsquerdo.setPosition(float('inf'))
motorDireito.setPosition(float('inf'))

#Iniciar os encoders
encoders = motorEsquerdo.getPositionSensor()
encoders.enable(timeStep)


#Definir um encoder 
def encoder_antes():
    # Retorna o valor do encoder antes de mover
    global encoder_inicial
    encoder_inicial = float(encoders.getValue())
    return encoder_inicial

# Mover para trás uma certa distância(em rads)
def mover_para_tras(dist):
    while robot.step(timeStep) != -1:
        global andou
        motorEsquerdo.setVelocity(-maxVelocity)
        motorDireito.setVelocity(-maxVelocity)
        if encoders.getValue() - encoder_inicial < -dist:
            parar()
            break
# Mover para frente uma certa distância(em rads)
def mover_para_frente(dist):
    while robot.step(timeStep) != -1:
        motorEsquerdo.setVelocity(maxVelocity)
        motorDireito.setVelocity(maxVelocity)

        if encoders.getValue() - encoder_inicial > dist:
            parar()
            break

# Função para virar à esquerda por 90 graus
def virar_esquerda():
    while robot.step(timeStep) != -1:
        motorEsquerdo.setVelocity(-maxVelocity)
        motorDireito.setVelocity(maxVelocity)
        if encoders.getValue() - encoder_inicial < -rad / 2:
            parar()
            break

# Função para virar à direita por 90 graus
def virar_direita():
    while robot.step(timeStep) != -1:
        motorEsquerdo.setVelocity(maxVelocity)
        motorDireito.setVelocity(-maxVelocity)
        if encoders.getValue() - encoder_inicial > rad / 2:
            parar()
            break

# Parar
def parar():
    motorEsquerdo.setVelocity(0.0)
    motorDireito.setVelocity(0.0)


mover_para_frente(0.1)
sleep(0.1)  # gambiarra essencial para o robô não bugar :) NÃO TIRAR

mover_para_frente(50)

while robot.step(timeStep) != -1:
    print(encoder_inicial)