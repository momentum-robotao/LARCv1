from controller import *
from time import sleep


#variaveis
cor_buraco = b'...\xff'
tem_buraco = False
maxVelocity = 6.28
andou = 0.0
robot = Robot() # Create robot object
timeStep = 32   # timeStep = number of milliseconds between world updates
encoder_inicial = 0.0

#Sensor de distancia e de cor 
sensoresFrente = [robot.getDevice("ps7"), robot.getDevice("ps0")]
sensoresEsquerda = [robot.getDevice("ps5"), robot.getDevice("ps6")]
sensoresDireita = [robot.getDevice("ps1"), robot.getDevice("ps2")]
sensor_de_cor = robot.getDevice("colour_sensor")

# Ativar sensores
for sensor in sensoresFrente + sensoresEsquerda + sensoresDireita:
    sensor.enable(timeStep)
sensor_de_cor.enable(timeStep)

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

# Vira 180 graus
def virar_180():
    while robot.step(timeStep) != -1:
        motorEsquerdo.setVelocity(maxVelocity)
        motorDireito.setVelocity(-maxVelocity)
        if encoders.getValue() - encoder_inicial > rad:
            parar()
            break

def parede_direita(sensoresDireita):
    # Retorna True se há parede à direita, senão, retorna False
    if sensoresDireita[1].getValue() < 0.11:
        return True
    return False


def parede_esquerda(sensoresEsquerda):
    # Retorna True se há parede à esquerda, senão, retorna False
    if sensoresEsquerda[1].getValue() < 0.14: #0.14
        return True
    return False


def parede_frente(sensoresFrente):
    # Retorna True se há parede à frente, senão, retorna False
    if sensoresFrente[0].getValue() < 0.155 or sensoresFrente[1].getValue() < 0.155:
        return True
    return False

def ajustar_distancia():
    # Ajusta a distância do robô para a parede
    dist_antes = sensoresFrente[1].getValue()
    compensar = dist_antes - 0.06
    while robot.step(timeStep) != -1:
        if compensar > 0:
            encoder_antes()
            mover_para_frente(compensar)
            break
        if compensar < 0:
            encoder_antes()
            mover_para_tras(compensar)
            break

def tem_buraco():
    if sensor_de_cor == cor_buraco:
        return True
    return False

mover_para_frente(0.01)
sleep(0.1)  # gambiarra essencial para o robô não bugar :) NÃO TIRAR


while robot.step(timeStep) != -1:
    ajustar_distancia()
    







'''

    # Função para seguir a parede
def seguir_parede():
    if not parede_esquerda(sensoresEsquerda):
        # Se não há parede à esquerda, vire à esquerda e mova-se para frente
        print(f'esquerda livre, {sensoresEsquerda[1].getValue()}')
        encoder_antes()
        virar_esquerda()
        encoder_antes()
        mover_para_frente(6)
    elif parede_esquerda(sensoresEsquerda) and not parede_frente(sensoresFrente):
        # Se há parede à esquerda, mas não à frente, mova-se para frente
        print('esquerda ocupada, frente livre')
        encoder_antes()
        mover_para_frente(6)
    elif parede_esquerda(sensoresEsquerda) and parede_frente(sensoresFrente):
        # Se há parede à esquerda e à frente, vire à direita e mova-se para frente
        print('esquerda e frente ocupadas')
        encoder_antes()
        virar_direita()
        encoder_antes()
        mover_para_frente(6)
    elif parede_direita(sensoresDireita):
        print('beco')
        encoder_antes()
        virar_180()
        
'''