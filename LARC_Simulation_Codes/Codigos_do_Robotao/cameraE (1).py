import easyocr as ocr
from controller import Robot
from time import sleep
from math import sqrt
import struct
import cv2
import numpy as np

vermelho_antes = 0.0
preto_antes = 0.0
cor_buraco = b'...\xff'
PI = 3.141592653589
timeStep = 32
maxVelocity = 6.28
encoder_inicial = 0.0
tem_buraco = False
andou = 0.0
rad = 4.29552
mensagem_enviada = False

erro_anterior = 0
soma_erros = 0

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
victimTypeH = bytes('H', "utf-8") # The victim type being sent is the letter 'H' for harmed victim

sensoresFrente = [robot.getDevice("ps_frente"), robot.getDevice("ps_tras")]
sensoresEsquerda = [robot.getDevice("ps_esquerda"), robot.getDevice("ps_diagonal_esquerda")]
sensoresDireita = [robot.getDevice("ps_direita"), robot.getDevice("ps_diagonal_direita")]

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

def ajustar_preto():
    global preto_antes
    preto = quant_preto()
    vermelho = quant_vermelho()
    if preto > 20 and preto > preto_antes and vermelho == 0:
        print('boraaa,', preto)
        while True:
            print(f'ta aumentando mlk {preto}, {preto_antes}')
            preto_antes = preto
            encoder_antes()
            mover_para_frente(0.1)
            preto = quant_preto()  
            if preto_antes > preto:
                reconhecer_vitima()
                preto_antes = 0
                break
        parar()
        print('parou')
def quant_preto():
    image = cameraE.getImage()
    image = np.frombuffer(image, np.uint8).reshape((cameraE.getHeight(), cameraE.getWidth(), 4))
    # Convert the image to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Count the number of non-black pixels
    non_black_pixels = cv2.countNonZero(gray_image)

    # Subtract the number of non-black pixels from the total number of pixels
    total_pixels = gray_image.size
    black_pixels = total_pixels - non_black_pixels

    return black_pixels
def tem_preto():
    image = cameraE.getImage()
    image = np.frombuffer(image, np.uint8).reshape((cameraE.getHeight(), cameraE.getWidth(), 4))
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
    return np.any(result)

#tem amarelo trava o programa
def tem_amarelo():
    image = cameraE.getImage()
    image = np.frombuffer(image, np.uint8).reshape((cameraE.getHeight(), cameraE.getWidth(), 4))  
    # Convert the image to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper threshold values for yellow in HSV
    lower_yellow = np.array([20, 100, 100], dtype=np.uint8)
    upper_yellow = np.array([30, 255, 255], dtype=np.uint8)

    # Create a mask that identifies yellow pixels
    mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

    # Bitwise-AND mask and original image
    result = cv2.bitwise_and(image, image, mask=mask)

    return np.any(result)

# tem vemrelho trava o programa
def ajustar_vermelho():
    global vermelho_antes
    preto = quant_preto()
    vermelho = quant_vermelho()
    if vermelho > 20 and vermelho > vermelho_antes and preto == 0:
        print(f'sisior,{vermelho}')
        while True:
            print(f'ta aumentando mlk {vermelho}, {vermelho_antes}')
            vermelho_antes = vermelho
            encoder_antes()
            mover_para_frente(0.1)
            vermelho = quant_vermelho()
            if vermelho_antes > vermelho:
                delay(1000)
                reconhecer_vitima()
                vermelho_antes = 0
                break
        parar()
        print('parou')


def quant_vermelho():
    image = cameraE.getImage()
    image = np.frombuffer(image, np.uint8).reshape((cameraE.getHeight(), cameraE.getWidth(), 4))
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
    non_black_pixels = cv2.countNonZero(mask)
    return non_black_pixels
def tem_vermelho():
    image = cameraE.getImage()
    image = np.frombuffer(image, np.uint8).reshape((cameraE.getHeight(), cameraE.getWidth(), 4))
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

def PID():
    global erro_anterior
    global soma_erros
    Kp = 38.5
    Ki= 0.4
    Kd = 1.0
    erro = sensoresEsquerda[1].getValue() - 0.13
    


    proporcional = Kp*erro

    parar()
    reconhecer_vitima()
    if tem_preto():
        ajustar_preto() #se tiver preto na esq, ajusta o centro e reconhece
        delay(200)
    elif tem_vermelho():
        ajustar_vermelho()
        delay(200)
    if proporcional > 12.28:
        proporcional = 12.28

    if tem_buraco():
        andou = encoders.getValue() - encoder_inicial
        mover_para_tras(1.0)
        encoder_antes()
        girar()
        encoder_antes()
        virar_direita()

    if erro - erro_anterior > 0.2:
        parar()
        print('desvio')
        delay(20)
        encoder_antes()
        mover_para_frente(6.0)
        ajustar_distancia()
        reconhecer_vitima()
        print('frente')
        delay(20)
        seguir_parede()
        delay(20)
    
    if erro > 0:
        #subtrai do motor esq e mantem o direito
        motorEsquerdo.setVelocity(maxVelocity - proporcional)
        motorDireito.setVelocity(maxVelocity)
        
        erro_anterior = erro
        soma_erros += erro
    else:
        #subtrai do motor direito e mantem o esquerdo
        motorEsquerdo.setVelocity(maxVelocity)
        motorDireito.setVelocity(maxVelocity + proporcional)
        erro_anterior = erro
        soma_erros += erro
    
    if parede_frente(sensoresFrente):
        ajustar_distancia()
        delay(20)
        ajustar_distancia()
        seguir_parede()
        delay(20)
        print('parede a frente')
'''    print(f'dist{sensoresEsquerda[0].getValue()}')
    print(f'erro: {erro}')
    print(f'proporcional: {proporcional}')'''



def reconhecer_vitima():
    preto = quant_preto()
    vermelho = quant_vermelho()
    if preto > 20 and parede_esquerda(sensoresEsquerda) or vermelho > 20:
        if tem_preto(): ajustar_preto()
        elif tem_vermelho(): ajustar_vermelho()
        print('reconhecendo...')
        possivel_H = ['H', 'HI', 'IH', 'IHI', 'F', 'FI', 'IF', 'IFI', 'A', 'AI', 'IA', 'IAI', 'HH']
        possivel_U = ['U', 'UI', 'IU', 'IUI', 'UU', '(U)', 'U)', '(U']
        possivel_S = ['S', 'SI', 'IS', 'ISI', 'SIS', 'SS', '5', 'I5', '5I', 'I5I', '75']
        ajustar_distancia()
        image = cameraE.getImage()
        image = np.frombuffer(image, np.uint8).reshape((cameraE.getHeight(), cameraE.getWidth(), 4))
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #padrao opencv	
        rgb_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2RGB)
        # Define the lower and upper threshold values for black in HSV

        reader = ocr.Reader(['en'])
        result = reader.readtext(rgb_image)
        for item in result:
            text = item[1].upper()
            print(text)
            if text in possivel_H:
                print('H')
                codificar_tipo('H')
                return 0
            elif text in possivel_U:
                print('U')
                codificar_tipo('U')
                return 0
            elif text in possivel_S:
                print('S')
                codificar_tipo('S')
                return 0
        if tem_amarelo():
            print('O')
            codificar_tipo('O')
        elif tem_vermelho():
            print('F')
            codificar_tipo('F')
        elif tem_preto():
            print('C')
            codificar_tipo('C')


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
    if sensoresDireita[0].getValue() < 0.07:
        return True
    return False


def parede_esquerda(sensoresEsquerda):
    # Retorna True se há parede à esquerda, senão, retorna False
    if sensoresEsquerda[0].getValue() < 0.083:
        return True
    return False


def parede_frente(sensoresFrente):
    # Retorna True se há parede à frente, senão, retorna False
    if sensoresFrente[0].getValue() < 0.16:
        return True
    return False
def parede_tras(sensoresFrente):
    # Retorna True se há parede à tras, senão, retorna False
    if sensoresFrente[1].getValue() < 0.16:
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
        if tem_preto(): print('preto')
        elif tem_vermelho(): print('vermelho')
        if encoders.getValue() - encoder_inicial < -rad / 2:
            parar()
            break


# Função para virar à direita por 90 graus
def virar_direita():
    while robot.step(timeStep) != -1:
        motorEsquerdo.setVelocity(maxVelocity)
        motorDireito.setVelocity(-maxVelocity)
        if tem_preto(): print('preto')
        elif tem_vermelho(): print('vermelho')
        if encoders.getValue() - encoder_inicial > rad / 2:
            parar()
            break

def girar():
    #analisa o ambiente
    while robot.step(timeStep) != -1:
        motorEsquerdo.setVelocity(maxVelocity)
        motorDireito.setVelocity(-maxVelocity)
        if parede_frente(sensoresFrente):
            break
        if encoders.getValue() - encoder_inicial > 2*rad:
            parar()
            break


# Função para seguir a parede
def seguir_parede():
    parar()
    reconhecer_vitima()
    if not parede_esquerda(sensoresEsquerda):
        while not parede_esquerda(sensoresEsquerda):
            # Se não há parede à esquerda, vire à esquerda e mova-se para frente
            print('esquerda livre,')
            encoder_antes()
            virar_esquerda()
            parar()
            reconhecer_vitima()            
            encoder_antes()
            mover_para_frente(6)
            
    elif parede_esquerda(sensoresEsquerda) and not parede_frente(sensoresFrente):
        # Se há parede à esquerda, mas não à frente, mova-se para frente
        print('esquerda ocupada, frente livre')
        parar()
        reconhecer_vitima()
        encoder_antes()
        mover_para_frente(6)
        if not parede_esquerda(sensoresEsquerda):
            seguir_parede()
    elif parede_esquerda(sensoresEsquerda) and parede_frente(sensoresFrente) and not parede_direita(sensoresDireita):
        # Se há parede à esquerda e à frente, vire à direita e mova-se para frente
        print('esquerda e frente ocupadas')
        encoder_antes()
        virar_direita()
        parar()
        reconhecer_vitima()
        encoder_antes()
        mover_para_frente(6)
        if not parede_esquerda(sensoresEsquerda):
            seguir_parede()
    else:
        print('beco')
        encoder_antes()
        virar_180()
    parar()
    reconhecer_vitima()

# Loop principal
mover_para_frente(0.01)
sleep(0.1)  # gambiarra essencial para o robô não bugar :) NÃO TIRAR
while robot.step(timeStep) != -1:
    PID()

