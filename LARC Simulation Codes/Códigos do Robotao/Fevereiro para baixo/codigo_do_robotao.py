from controller import Robot
from time import sleep

TILE = 0.3
PI = 3.141592653589
timeStep = 32
angle = 0.0
initAngle = 0.0
maxVelocity = 6.28
encoder_inicial = 0.0
initAngle = 0.0

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
    if sensoresDireita[1].getValue() < 0.1:
        return True
    return False

def parede_esquerda(sensoresEsquerda):
    # Retorna True se há parede à esquerda, senão, retorna False
    if sensoresEsquerda[1].getValue() < 0.1:
        return True
    return False

def parede_frente(sensoresFrente):
    # Retorna True se há parede à frente, senão, retorna False
    if sensoresFrente[0].getValue() < 0.1 or sensoresFrente[1].getValue() < 0.1:
        return True
    return False

def atualizarGiroscopio():
    global angle
    angle += (float(timeStep) / 1000.0) * float(giroscopio.getValues()[1])

# Função para obter o ângulo em graus
def obterAngulo():
    return angle * 180 / PI


def encoder_antes():
    # Retorna o valor do encoder antes de mover
    global encoder_inicial
    encoder_inicial = float(encoders.getValue())
    return encoder_inicial
# Função para mover para frente por uma determinada distância
def mover_para_frente(dist):
    while robot.step(timeStep) != -1:
        motorEsquerdo.setVelocity(maxVelocity)
        motorDireito.setVelocity(maxVelocity)
        if encoders.getValue() - encoder_inicial > dist:
            motorEsquerdo.setVelocity(0.0)
            motorDireito.setVelocity(0.0)
            break
def angulo_antes():
    # Retorna o valor do ângulo antes de virar
    global initAngle
    initAngle = float(obterAngulo())
    return initAngle
# Função para virar à esquerda por 90 graus
def virar_esquerda():
    while robot.step(timeStep) != -1:
        atualizarGiroscopio()
        motorEsquerdo.setVelocity(-maxVelocity)
        motorDireito.setVelocity(maxVelocity)
        if abs(obterAngulo() - initAngle) > 86.34:
            motorEsquerdo.setVelocity(0.0)
            motorDireito.setVelocity(0.0)
            break

# Função para virar à direita por 90 graus
def virar_direita():
 
    while robot.step(timeStep) != -1:
        atualizarGiroscopio()
        motorEsquerdo.setVelocity(maxVelocity)
        motorDireito.setVelocity(-maxVelocity)
        if abs(obterAngulo() - initAngle) > 86.0:
            motorEsquerdo.setVelocity(0.0)
            motorDireito.setVelocity(0.0)
            break
# Função para seguir a parede
def seguir_parede():
    if not parede_esquerda(sensoresEsquerda):
        # Se não há parede à esquerda, vire à esquerda e mova-se para frente
        print(f'esquerda livre, {sensoresEsquerda[1].getValue()}')
        angulo_antes()
        virar_esquerda()
        encoder_antes()
        mover_para_frente(6)
    elif parede_esquerda(sensoresEsquerda) and not parede_frente(sensoresFrente):
        # Se há parede à esquerda, mas não à frente, mova-se para frente
        print('esquerda ocupada, frente livre')
        encoder_antes()
        mover_para_frente(6)
        sleep(0.1)
    elif parede_esquerda(sensoresEsquerda) and parede_frente(sensoresFrente):
        # Se há parede à esquerda e à frente, vire à direita e mova-se para frente
        print('esquerda e frente ocupadas')
        angulo_antes()
        virar_direita()
        encoder_antes()
        mover_para_frente(6)
    elif parede_esquerda(sensoresEsquerda) and parede_frente(sensoresFrente) and parede_direita(sensoresDireita):
        print('beco')
        angulo_antes()
        virar_direita()
        sleep(0.1)
        angulo_antes()
        virar_direita()

# Loop principal 

mover_para_frente(0.01)
sleep(0.1)# gambiarra essencial para o robô não bugar :) NÃO TIRAR
while robot.step(timeStep) != -1:
    atualizarGiroscopio()
    seguir_parede()
    