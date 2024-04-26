from controller import Robot
from time import sleep
from math import sqrt
#import struct

cor_buraco = b'...\xff'
PI = 3.141592653589
timeStep = 32
maxVelocity = 6.28
encoder_inicial = 0.0
tem_buraco = False
andou = 0.0
rad = 4.32

robot = Robot()
motorEsquerdo = robot.getDevice('left motor')
motorDireito = robot.getDevice('right motor')
encoders = motorEsquerdo.getPositionSensor()

#camera
camera = robot.getDevice("camera1")
camera.enable(timeStep)
#image = camera.getImage()


sensor_de_cor = robot.getDevice("colour_sensor")
sensor_de_cor.enable(timeStep)

emitter = robot.getDevice("emitter")

gps = robot.getDevice("gps")
gps.enable(timeStep)

sensoresFrente = [robot.getDevice("ps7"), robot.getDevice("ps0")]
sensoresEsquerda = [robot.getDevice("ps5"), robot.getDevice("ps6")]
sensoresDireita = [robot.getDevice("ps1"), robot.getDevice("ps2")]

# Ativar sensores
for sensor in sensoresFrente + sensoresEsquerda + sensoresDireita:
    sensor.enable(timeStep)

# Inicializar motores e sensores
motorEsquerdo.setPosition(float("inf"))
motorDireito.setPosition(float("inf"))
motorEsquerdo.setVelocity(0.0)
motorDireito.setVelocity(0.0)
encoders.enable(timeStep)

def tem_buraco():
    cor = sensor_de_cor.getImage()
    if cor == cor_buraco:
        return True
    return False
'''def enviar_mensagem():
    global mensagem_enviada
    # Envia uma mensagem para o supervisor
    posicao = gps.getValues()
    if not mensagem_enviada:
        mensagem = struct.pack('i i i c', 0 , int(posicao[0] * 100), int(posicao[2] * 100), b'H')
        emitter.send(mensagem)
        mensagem_enviada = True'''

def objeto_proximo(posicao):
    # Retorna True se o objeto está próximo, senão, retorna False
    return sqrt(posicao[0] ** 2 + posicao[1] ** 2) < 0.1

'''def   pegar_vitima():
    # Retorna uma lista com as posições das vítimas
    #objeto = camera.getRecognitionObjects()
    vitimas = []
    for i in objeto:
        if i.get_colors() == [1, 1, 1]:
            vitimas.append(i.get_position_on_image()[0])
    return vitimas'''

'''def parar_na_vitima():
    global mensagem_enviada
    # Para o robô quando ele encontra uma vítima
    vitimas = pegar_vitima()
    vitima_encontrada = False
    for vitima in vitimas:
        if objeto_proximo(vitima):
            # Se a vítima está próxima, pare e manda mensagem p o supervisor
            parar()
            enviar_mensagem()
            vitima_encontrada = True
        if not vitima_encontrada:
            mensagem_enviada = False '''
def parede_direita(sensoresDireita):
    # Retorna True se há parede à direita, senão, retorna False
    if sensoresDireita[1].getValue() < 0.11:
        return True
    return False

def parede_esquerda(sensoresEsquerda):
    # Retorna True se há parede à esquerda, senão, retorna False
    if sensoresEsquerda[1].getValue() < 0.14:
        return True
    return False

def parede_frente(sensoresFrente):
    # Retorna True se há parede à frente, senão, retorna False
    if sensoresFrente[0].getValue() < 0.155 or sensoresFrente[1].getValue() < 0.155:
        return True
    return False


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
        if tem_buraco():
            andou = encoders.getValue() - encoder_inicial
            mover_para_tras(andou-0.35)
            encoder_antes()
            virar_180()
            break
        if encoders.getValue() - encoder_inicial > dist:
            parar()
            break
def parar():
    motorEsquerdo.setVelocity(0.0)
    motorDireito.setVelocity(0.0)


def virar_180():

    # Vira 180 graus
    while robot.step(timeStep) != -1:
        motorEsquerdo.setVelocity(maxVelocity)
        motorDireito.setVelocity(-maxVelocity)
        print(encoders.getValue() - encoder_inicial)
        if encoders.getValue() - encoder_inicial > rad:
            parar()
            break
# Função para virar à esquerda por 90 graus
def virar_esquerda():

    while robot.step(timeStep) != -1:
        motorEsquerdo.setVelocity(-maxVelocity)
        motorDireito.setVelocity(maxVelocity)
        print('girando')
        if encoders.getValue() - encoder_inicial < -rad/ 2:
            print('parou')
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

# Loop principal 

mover_para_frente(0.01)
sleep(0.1)# gambiarra essencial para o robô não bugar :) NÃO TIRAR
while robot.step(timeStep) != -1:
    seguir_parede()

    