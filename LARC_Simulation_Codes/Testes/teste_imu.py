from controller import Robot, Lidar # Step 1: Import Lidar
import numpy as np
from time import sleep
import matplotlib.pyplot as plt
import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())
maxVelocity = 6.28
imu = robot.getDevice("inertial_unit")
imu.enable(timestep)
motorEsquerdo = robot.getDevice("left motor")
motorDireito = robot.getDevice("right motor")
encoders = motorEsquerdo.getPositionSensor()
motorEsquerdo.setPosition(float("inf"))
motorDireito.setPosition(float("inf"))
motorEsquerdo.setVelocity(0.0)
motorDireito.setVelocity(0.0)
encoders.enable(timestep)


def get_orientation(imu):
    orientation = {'north' : 0, 'east' : round(-math.pi/2,2), 'south' : round(math.pi,2), 'west': round(math.pi/2,2)}
    for key in orientation : 
        #print(round(imu.getRollPitchYaw()[2], 2))
        if round(imu.getRollPitchYaw()[2], 2) == orientation[key] : 
            return key
        pass

''' 
north -> 0
east -> -pi/2
south -> -pi
west -> pi/2

colocar um delay de 0.5s a cada giro
'''


def parar():
    sleep(0.1)
    motorEsquerdo.setVelocity(0.0)
    motorDireito.setVelocity(0.0)


def get_delta_rotation(ang, new_ang):
    if new_ang * ang <= 0:
        if round(ang) == 0:
            return abs(ang) + abs(new_ang)
        else:
            return abs(abs(ang) - math.pi) + abs(abs(new_ang) - math.pi)
    return max(ang, new_ang) - min(ang, new_ang)

def virar(direcao, degrees):
    """
    :param: direcao: should be 'left' or 'right'
    """
    parar()
    robot_to_turn_rad = (degrees / 180) * math.pi

    # print(f"======== Começando girada de {degrees}° e {robot_to_turn_rad} rad ========")

    accumulator = 0
    ang = imu.getRollPitchYaw()[2]

    set_vel = lambda faltante: (maxVelocity if faltante > 0.1 else maxVelocity / 100)

    while robot.step(timestep) != -1:
        new_ang = imu.getRollPitchYaw()[2]
        accumulator += get_delta_rotation(ang, new_ang)
        ang = imu.getRollPitchYaw()[2]

        falta = robot_to_turn_rad - accumulator
        if direcao == "left":
            motorEsquerdo.setVelocity(-set_vel(falta))
            motorDireito.setVelocity(set_vel(falta))
        elif direcao == "right":
            motorEsquerdo.setVelocity(set_vel(falta))
            motorDireito.setVelocity(-set_vel(falta))

        if accumulator >= robot_to_turn_rad:
            break

    parar()

def virar_180(initial_angle=None):
    # Vira 180 graus
    virar("right", 180)


# Função para virar à esquerda por 90 graus
def virar_esquerda(initial_angle=None):
    virar("left", 90)


# Função para virar à direita por 90 graus
def virar_direita(initial_angle=None):
    virar("right", 90)


while robot.step(timestep) != -1: # Test on the "TESTE_DISTANCIA2"
    print(f"IMU's Orientation : {get_orientation(imu)}")



