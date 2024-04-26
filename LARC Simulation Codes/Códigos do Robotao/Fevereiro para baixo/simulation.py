from controller import Robot
from time import sleep

TILE = 0.8888
PI = 3.141592653589
timeStep = 32
angle = 0.0
initAngle = 0.0
maxVelocity = 6.28

robot = Robot()
motorEsquerdo = robot.getDevice('left motor')
motorDireito = robot.getDevice('right motor')
encoders = motorEsquerdo.getPositionSensor()
giroscopio = robot.getDevice('gyro')

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
giroscopio.enable(timeStep)
encoders.enable(timeStep)

# Função para atualizar o ângulo do giroscópio
def parede_direita(sensoresDireita):
    # Retorna True se há parede à direita, senão, retorna False
    if sensoresDireita[1].getValue() < 0.35:
        return True
    return False

def parede_esquerda(sensoresEsquerda):
    # Retorna True se há parede à esquerda, senão, retorna False
    if sensoresEsquerda[1].getValue() < 0.35:
        return True
    return False

def parede_frente(sensoresFrente):
    # Retorna True se há parede à frente, senão, retorna False
    if sensoresFrente[0].getValue() < 0.35 or sensoresFrente[1].getValue() < 0.35:
        return True
    return False

def atualizarGiroscopio():
    global angle
    angle += (float(timeStep) / 1000.0) * float(giroscopio.getValues()[1])

# Função para obter o ângulo em graus
def obterAngulo():
    return angle * 180 / PI

# Função para verificar se o robô virou 90 graus 

# Função para mover para frente por uma determinada distância
def mover_para_frente(dist):
    encoder_inicial = encoders.getValue()
    while robot.step(timeStep) != -1:
        motorEsquerdo.setVelocity(maxVelocity)
        motorDireito.setVelocity(maxVelocity)
        print(encoders.getValue())
        print('Movendo para frente...')
        if encoders.getValue() - encoder_inicial > dist:
            print('Parou de ftrente')
            motorEsquerdo.setVelocity(0)
            motorDireito.setVelocity(0)
            break

# Função para virar à esquerda por 90 graus
def virar_esquerda():
    initAngle = obterAngulo()
    while robot.step(timeStep) != -1:
        atualizarGiroscopio()
        motorEsquerdo.setVelocity(-maxVelocity)
        motorDireito.setVelocity(maxVelocity)
        print(f'Virando à esquerda, initAngle {initAngle}, angle {obterAngulo()}')
        if not abs(obterAngulo() - initAngle) < 86.0:
            print('Parou de virar')
            motorEsquerdo.setVelocity(0.0)
            motorDireito.setVelocity(0.0)
            break

# Função para virar à direita por 90 graus
def virar_direita():
    while robot.step(timeStep) != -1:
        atualizarGiroscopio()
        motorEsquerdo.setVelocity(maxVelocity)
        motorDireito.setVelocity(-maxVelocity)
        print(f'Virando à direita, initAngle {initAngle}, angle {obterAngulo()}')
        if not abs(obterAngulo() - initAngle) < 86.0:
            print('Parou de virar')
            motorEsquerdo.setVelocity(0.0)
            motorDireito.setVelocity(0.0)
            break

# Função para seguir a parede
def seguir_parede():
    if not parede_esquerda(sensoresEsquerda):
        # Se não há parede à esquerda, vire à esquerda e mova-se para frente
        print('caso 1')
        mover_para_frente(4)
        virar_esquerda()
        mover_para_frente(4)

    elif parede_esquerda(sensoresEsquerda) and not parede_frente(sensoresFrente):
        # Se há parede à esquerda, mas não à frente, mova-se para frente
        print('caso 2')
        motorEsquerdo.setVelocity(maxVelocity)
        motorDireito.setVelocity(maxVelocity)
    elif parede_esquerda(sensoresEsquerda) and parede_frente(sensoresFrente):
        # Se há parede à esquerda e à frente, vire à direita
        mover_para_frente(4)
        print('caso 3')
        virar_direita()
        mover_para_frente(4)
    elif parede_esquerda(sensoresEsquerda) and parede_frente(sensoresFrente) and parede_direita(sensoresDireita):
        print('caso 4')
        virar_direita()
        sleep(0.1)
        virar_direita()

# Loop principal
virar_direita()
while robot.step(timeStep) != -1:
    atualizarGiroscopio()
    seguir_parede()
