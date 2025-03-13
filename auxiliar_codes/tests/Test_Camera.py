from controller import *
from time import sleep
from math import sqrt
import struct
import numpy as np
import cv2

#variaveis
cor_buraco = b'...\xff'
tem_buraco = False
maxVelocity = 6.28
andou = 0.0
robot = Robot() # Create robot object
timeStep = 32   # timeStep = number of milliseconds between world updates
encoder_inicial = 0.0
PI = 3.141592653589
rad = 4.2955
victimType = bytes('H', "utf-8") # The victim type being sent is the letter 'H' for harmed victim

#Sensor de distancia, de cor e camera
sensoresFrente = [robot.getDevice("ps7"), robot.getDevice("ps0")]
sensoresEsquerda = [robot.getDevice("ps5"), robot.getDevice("ps6")]
sensoresDireita = [robot.getDevice("ps1"), robot.getDevice("ps2")]
sensor_de_cor = robot.getDevice("colour_sensor")
camera = robot.getDevice("camera1")

# Ativar sensores (sensores de distância, cor e câmera)
for sensor in sensoresFrente + sensoresEsquerda + sensoresDireita:
    sensor.enable(timeStep)
sensor_de_cor.enable(timeStep)
camera.enable(timeStep)

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

def seguir_parede():

    # Se não há parede à esquerda, vire à esquerda e mova-se para frente
    if not parede_esquerda(sensoresEsquerda):
        #print(f'esquerda livre, {sensoresEsquerda[1].getValue()}')
        encoder_antes()
        virar_esquerda()
        encoder_antes()
        mover_para_frente(6)

    # Se há parede à esquerda, mas não à frente, mova-se para frente
    elif parede_esquerda(sensoresEsquerda) and not parede_frente(sensoresFrente):
        #print('esquerda ocupada, frente livre')
        encoder_antes()
        mover_para_frente(6)

    # Se há parede à esquerda e à frente, vire à direita e mova-se para frente
    elif parede_esquerda(sensoresEsquerda) and parede_frente(sensoresFrente) and not parede_direita(sensoresDireita):
        #print('esquerda e frente ocupadas, direita livre')
        encoder_antes()
        virar_direita()
        encoder_antes()
        mover_para_frente(6)
    
    # Se há parede à esquerda, à frente e à direita(beco), vire 180 graus e mova-se para frente
    elif parede_esquerda(sensoresEsquerda) and parede_frente(sensoresFrente) and parede_direita(sensoresDireita):
        print('todas os lados ocupados, é um beco')
        encoder_antes()
        virar_180()
        encoder_antes()
        mover_para_frente(6)


def tem_buraco():
    if sensor_de_cor == cor_buraco:
        return True
    return False

def tem_preto():
    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    # Convert the image to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper threshold values for black in HSV
    lower_black = np.array([0, 0, 0], dtype=np.uint8)
    upper_black = np.array([180, 255, 30], dtype=np.uint8)

    # Create a mask that identifies black pixels
    mask = cv2.inRange(hsv_image, lower_black, upper_black)

    # Bitwise-AND mask and original image
    result = cv2.bitwise_and(image, image, mask=mask)

    # Display the result
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    #print(np.any(result))
    return np.any(result)

def tem_amarelo():
    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))  
    # Convert the image to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper threshold values for yellow in HSV
    lower_yellow = np.array([20, 100, 100], dtype=np.uint8)
    upper_yellow = np.array([30, 255, 255], dtype=np.uint8)

    # Create a mask that identifies yellow pixels
    mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

    # Bitwise-AND mask and original image
    result = cv2.bitwise_and(image, image, mask=mask)

    # Display the result
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return np.any(result)

def tem_vermelho():
    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    # Convert the image to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper threshold values for red in HSV
    lower_red1 = np.array([0, 100, 100], dtype=np.uint8)
    upper_red1 = np.array([10, 255, 255], dtype=np.uint8)
    lower_red2 = np.array([160, 100, 100], dtype=np.uint8)
    upper_red2 = np.array([180, 255, 255], dtype=np.uint8)

    # Create masks that identify red pixels
    mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

    # Combine the masks
    mask = cv2.bitwise_or(mask1, mask2)

    # Bitwise-AND mask and original image
    result = cv2.bitwise_and(image, image, mask=mask)

    # Display the result
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return np.any(result)

victimType = bytes('H', "utf-8") # The victim type being sent is the letter 'H' for harmed victim

def codificar_tipo(tipo):
    x, y = valores_gps()
    victimType = bytes(tipo, "utf-8")
    message = struct.pack("j j x", x, y, victimType) # Pack the message.
    delay(1300) # Delay for 1.3 seconds
    emitter.send(message) # Send out the message
    #print("enviou mensagem")

def reconhecer_vitima():
    if parede_frente(sensoresFrente):
        #ajustar_distancia()
        cv2.waitKey(1)
        image = camera.getImage() # Pega a imagem da camera
        image = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)) # Configurando a tela de exibição da camera
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # Defini uma nova imagem com tom 'hsv'

        # Define um intervalo dos possíveis valores de preto
        lower_black = np.array([0, 0, 0], dtype=np.uint8)
        upper_black = np.array([180, 255, 185], dtype=np.uint8)

        mask = cv2.inRange(hsv_image, lower_black, upper_black) # aplica a máscara , ele percorre um 'for' na imagem_hsv, analisando cada pixel pra ver se ele se encontra no intervalo do preto. Daí td oq n é preto fica branco.
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, 
                                    cv2.CHAIN_APPROX_SIMPLE)   

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,
                                    0.002 * cv2.arcLength(cnt, True), True)
            if len(approx) >= 13 and len(approx) <= 26 and not tem_amarelo() and not tem_vermelho():
                delay(1300)
                codificar_tipo('H')
                
            elif len(approx) >= 27 and len(approx) <= 33 and not tem_amarelo() and not tem_vermelho():
                print('U')
                delay(1300)
                codificar_tipo('U')
            elif len(approx) >= 60 and len(approx) <= 82 and not tem_amarelo() and not tem_vermelho():
                print('S')
                delay(1300)
                codificar_tipo('S')
            elif tem_amarelo():
                print('O')
                delay(1300)
                codificar_tipo('O')
            elif tem_vermelho():
                print('F')
                delay(1300)
                codificar_tipo('F')
            elif tem_preto():
                print('C')
                delay(1300)
                codificar_tipo('C')

        # Bitwise-AND mask and original image
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)

mover_para_frente(0.01)
sleep(0.1)  # gambiarra essencial para o robô não bugar :) NÃO TIRAR

# Dentro desse while fica o "while true" da programação
while robot.step(timeStep) != -1:
    reconhecer_vitima()
    seguir_parede()

