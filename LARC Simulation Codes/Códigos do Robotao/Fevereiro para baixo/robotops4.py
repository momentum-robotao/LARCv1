from controller import Robot
from time import sleep
from math import sqrt
import struct
import cv2
import numpy as np

cor_buraco = b'...\xff'
PI = 3.141592653589
timeStep = 32
maxVelocity = 6.28
encoder_inicial = 0.0
tem_buraco = False
andou = 0.0
rad = 4.29552
mensagem_enviada = False

robot = Robot()
motorEsquerdo = robot.getDevice('left motor')
motorDireito = robot.getDevice('right motor')
encoders = motorEsquerdo.getPositionSensor()

# cameraF
cameraF = robot.getDevice("cameraF")
cameraF.enable(timeStep)

'''cameraF = robot.getDevice('cameraD')
cameraF.enable(timeStep)    

cameraE = robot.getDevice('cameraE')
cameraE.enable(timeStep)
'''
sensor_de_cor = robot.getDevice("colour_sensor")
sensor_de_cor.enable(timeStep)

emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
receiver.enable(timeStep)

gps = robot.getDevice("gps")
gps.enable(timeStep)
position = gps.getValues()
victimTypeH = bytes('H', "utf-8") # The victim type being sent is the letter 'H' for harmed victim

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

victimType = bytes('H', "utf-8") # The victim type being sent is the letter 'H' for harmed victim


def tem_preto():
    image = cameraF.getImage()
    image = np.frombuffer(image, np.uint8).reshape((cameraF.getHeight(), cameraF.getWidth(), 4))
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
    print(np.any(result))
    return np.any(result)

def tem_amarelo():
    image = cameraF.getImage()
    image = np.frombuffer(image, np.uint8).reshape((cameraF.getHeight(), cameraF.getWidth(), 4))  
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
    image = cameraF.getImage()
    image = np.frombuffer(image, np.uint8).reshape((cameraF.getHeight(), cameraF.getWidth(), 4))
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


def codificar_tipo(tipo):
    x, y = valores_gps()
    victimType = bytes(tipo, "utf-8")
    message = struct.pack("i i c", x, y, victimType) # Pack the message.
    delay(1300) # Delay for 1.3 seconds
    emitter.send(message) # Send out the message
    print("enviou mensagem")


def delay(ms):
    initTime = robot.getTime()      # Store starting time (in seconds)
    while robot.step(timeStep) != -1:
        if (robot.getTime() - initTime) * 1000.0 > ms: # If time elapsed (converted into ms) is greater than value passed in
            break

def valores_gps():
    position = gps.getValues()
    x = int(position[0] * 100) # Get the xy coordinates, multiplying by 100 to convert from meters to cm 
    y = int(position[2] * 100) # We will use these coordinates as an estimate for the victim's position
    return x, y
def ajustar_distancia():
    # Ajusta a distância do robô para a parede
    if sensoresFrente[0].getValue() < 0.15:
        falta = 0.15 - sensoresFrente[0].getValue()
        encoder_antes()
        mover_para_tras(falta)
    elif sensoresFrente[0].getValue() < 0.20 and sensoresFrente[0].getValue() > 0.15:
        falta = 0.15 - sensoresFrente[0].getValue()
        encoder_antes()
        mover_para_frente(falta)
         
def reconhecer_vitima():
    if parede_frente(sensoresFrente):
        ajustar_distancia()
        cv2.waitKey(1)
        image = cameraF.getImage()
        image = np.frombuffer(image, np.uint8).reshape((cameraF.getHeight(), cameraF.getWidth(), 4))
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper threshold values for black in HSV
        lower_black = np.array([0, 0, 0], dtype=np.uint8)
        upper_black = np.array([180, 255, 185], dtype=np.uint8)
        mask = cv2.inRange(hsv_image, lower_black, upper_black)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE,
                                    cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,
                                    0.002 * cv2.arcLength(cnt, True), True)
            if len(approx) >= 8 and len(approx) <= 26 and not tem_amarelo() and not tem_vermelho():
                delay(1300)
                codificar_tipo('H')
                print('H')  
            elif len(approx) >= 27 and len(approx) <= 33 and not tem_amarelo() and not tem_vermelho():
                print('U')
                delay(1300)
                codificar_tipo('U')
            elif len(approx) >= 60 and len(approx) <= 82 and not tem_amarelo() and not tem_vermelho():
                print('S')
                delay(1300)
                codificar_tipo('S')
            elif tem_amarelo() and len(approx) != 4:
                print('O')
                delay(1300)
                codificar_tipo('O')
            elif tem_vermelho() and len(approx) !=4:
                print('F')
                delay(1300)
                codificar_tipo('F')
            elif tem_preto() and len(approx) != 4:
                print('C')
                delay(1300)
                codificar_tipo('C')

        # Bitwise-AND mask and original image
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)


def tem_buraco():
    cor = sensor_de_cor.getImage()
    if cor == cor_buraco:
        return True
    return False


def objeto_proximo(posicao):
    # Retorna True se o objeto está próximo, senão, retorna False
    return sqrt(posicao[0] ** 2 + posicao[1] ** 2) < 0.1


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


def virar_180():
    # Vira 180 graus
    while robot.step(timeStep) != -1:
        motorEsquerdo.setVelocity(maxVelocity)
        motorDireito.setVelocity(-maxVelocity)
        if encoders.getValue() - encoder_inicial > rad:
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


# Função para seguir a parede
def seguir_parede():
    if not parede_esquerda(sensoresEsquerda):
        # Se não há parede à esquerda, vire à esquerda e mova-se para frente
        print('esquerda livre,')
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
    elif parede_direita(sensoresDireita) and parede_esquerda(sensoresEsquerda) and parede_frente(sensoresFrente):
        print('beco')
        encoder_antes()
        virar_180()


# Loop principal
mover_para_frente(0.01)
sleep(0.1)  # gambiarra essencial para o robô não bugar :) NÃO TIRAR
while robot.step(timeStep) != -1:
    seguir_parede()
    delay(100)
    ajustar_distancia()
    reconhecer_vitima()