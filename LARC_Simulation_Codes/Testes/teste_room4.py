from controller import Robot, Lidar # Step 1: Import Lidar
import numpy as np
from time import sleep
import matplotlib.pyplot as plt
import math

IMU_THRESHOLD=np.pi/6

robot = Robot()
timestep = int(robot.getBasicTimeStep())
maxVelocity = 6.28

motorEsquerdo = robot.getDevice("left motor")
motorDireito = robot.getDevice("right motor")
encoders = motorEsquerdo.getPositionSensor()
motorEsquerdo.setPosition(float("inf"))
motorDireito.setPosition(float("inf"))
motorEsquerdo.setVelocity(0.0)
motorDireito.setVelocity(0.0)
encoders.enable(timestep)

radius = 0.071/2

imu = robot.getDevice("inertial_unit")
imu.enable(timestep)

lidar = robot.getDevice("lidar") # Step 2: Retrieve the sensor, named "lidar", from the robot. Note that the sensor name may differ between robots.
lidar.enable(timestep) # Step 3: Enable the sensor, using the timestep as the update rate

gps = robot.getDevice("gps")
gps.enable(timestep)

contador_imagem = 0

x_total = []
y_total = []

def get_distance(angle, lidar) -> float:
    rangeImage = lidar.getRangeImage() # Step 4: Retrieve the range image
    n = (angle*512)//360
    print()
    dist = 0
    count = 0
    for i in range(4):
        if rangeImage[n+ i*512] != float('inf') : 
            dist += rangeImage[n+ i*512]
            count +=1

    dist = dist/count

    return dist
    
def get_posicao_atual(gps):
        posicao_atual = gps.getValues()
        posicaoX_atual = posicao_atual[0]
        posicaoY_atual = posicao_atual[2]

        return [posicaoX_atual, posicaoY_atual]

def get_orientation(imu):
    orientation = {'north' : 0, 'east' : round(-math.pi/2,1), 'south' : round(-math.pi,1), 'west': round(math.pi/2,1)}
    for key in orientation : 
        imu_value = imu.getRollPitchYaw()[2]
        if round(imu_value + IMU_THRESHOLD, 5) > orientation[key] and round(imu_value - IMU_THRESHOLD, 5) < orientation[key] : 
            return key
        pass
    print(f"imu_value: {imu_value}")

def get_mapa(lidar, gps, imu):
    global contador_imagem
    x = []
    y = []
    ori = get_orientation(imu)
    soma  = 0
    if ori == 'north' :
        soma = -90
    elif ori == 'east':
        soma = 0
    elif ori == 'south':
        soma = 90
    elif ori == 'west':
        soma = 180
    temp = 0
    for i in range(360):
        if (get_distance(i,lidar) > (1.5)*0.12):
            temp += 1
        else:
            x.append(get_distance(i,lidar)*math.cos((soma + i)*np.pi/180) + get_posicao_atual(gps)[0])
            y.append(get_distance(i,lidar)*math.sin((soma + i)*np.pi/180) + get_posicao_atual(gps)[1])
    for u in range(temp):
        x.append(x[0])
        y.append(y[0])
    fig, ax = plt.subplots()
    ax.yaxis.set_inverted(True)
    ax.scatter(x, y)
    ax.plot(get_posicao_atual(gps)[0], get_posicao_atual(gps)[1], 'x')
    achar_node(lidar, gps, imu)
    fig.savefig(f"C:\\Users\\goten\\Desktop\\LARCv1\\LARC_Simulation_Codes\\images\\mapa_{contador_imagem}.png")
    plt.close(fig)
    contador_imagem +=1
    return np.array(x),np.array(y)

def conferir_circulo(point1, point2):
    if ((point1 >= point2 - radius) and (point1<= point2+radius)) : 
        return True
    
def achar_node(lidar,gps,imu):
    ori = get_orientation(imu)
    get_posicao_atual(gps)
    Xi = get_posicao_atual(gps)[0]
    Yi = get_posicao_atual(gps)[1]
    d = 0.04
    x, y = get_mapa(lidar,gps,imu)
    soma  = 0
    if ori == 'north' :
        soma = -90
    elif ori == 'east':
        soma = 0
    elif ori == 'south':
        soma = 90
    elif ori == 'west':
        soma = 180
     
    # Sempre pensar em posições
    #Fazer as Bolas
    for i in range(180): # Tomar cuidado com o intervalo dos angulos da "frente"
        if(ori == 'north'):
            while((Yi - d*math.sin((soma+i))*np.pi/180) > y[i]):
                if(conferir_circulo(Yi - d*math.sin((soma+i))*np.pi/180, y[i])) or (conferir_circulo(Xi - d*math.cos((soma+i))*np.pi/180, x[i])):
                    d -= 0.04
                    fig, ax = plt.subplots()
                    circle = plt.Circle((Xi - d*math.cos((soma+i))*np.pi/180 , Yi - d*math.sin((soma+i))*np.pi/180), radius, fill = False)
                    ax.add_patch(circle)
                    ax.set_aspect('equal')
                    plt.show()
                    plt.close(fig)
                    break
                else:
                    d += 0.04
        elif(ori == 'south') : 
            while((Yi + d*math.sin((soma+i))*np.pi/180) < y[i]):
                if(conferir_circulo(Yi + d*math.sin((soma+i))*np.pi/180, y[i])) or (conferir_circulo(Xi + d*math.cos((soma+i))*np.pi/180, x[i])):
                    d -= 0.04
                    fig, ax = plt.subplots()
                    circle = plt.Circle((Xi + d*math.cos((soma+i))*np.pi/180, Yi + d*math.sin((soma+i))*np.pi/180), radius, fill = False)
                    ax.set_aspect('equal')
                    ax.add_patch(circle)
                    plt.show()
                    plt.close(fig)
                    break
                else:
                    d += 0.04
        elif(ori == 'east'):
            while((Xi + d*math.cos((soma+i))*np.pi/180) < x[i]):
                if(conferir_circulo(Yi + d*math.sin((soma+i))*np.pi/180, y[i])) or (conferir_circulo(Xi + d*math.cos((soma+i))*np.pi/180, x[i])):
                    d -= 0.04
                    fig, ax = plt.subplots()
                    circle = plt.Circle((Xi + d*math.cos((soma+i))*np.pi/180, Yi + d*math.sin((soma+i))*np.pi/180), radius, fill = False)
                    ax.set_aspect('equal')
                    ax.add_patch(circle)
                    plt.show()
                    plt.close(fig)
                    break
                else:
                    d += 0.04
        elif(ori == 'west'):
            while((Xi - d*math.cos((soma+i))*np.pi/180) > x[i]):
                if(conferir_circulo(Yi - d*math.sin((soma+i))*np.pi/180, y[i])) or (conferir_circulo(Xi - d*math.cos((soma+i))*np.pi/180, x[i])):
                    d -= 0.04
                    fig, ax = plt.subplots()
                    circle = plt.Circle((Xi - d*math.cos((soma+i))*np.pi/180, Yi - d*math.sin((soma+i))*np.pi/180), radius, fill = False)
                    ax.set_aspect('equal')
                    ax.add_patch(circle)
                    plt.show()
                    plt.close(fig)
                    break
                else:
                    d += 0.04


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

def delay(ms):
    initTime = robot.getTime()  # Store starting time (in seconds)
    while robot.step(timestep) != -1:
        if (
            robot.getTime() - initTime
        ) * 1000.0 > ms:  # If time elapsed (converted into ms) is greater than value passed in
            break

def mover_para_frente(dist, gps, posicao_atual, lidar):
    posicaoX_anterior = posicao_atual[0]
    posicaoY_anterior = posicao_atual[1]

    while robot.step(timestep) != -1:
        posicao_atual = get_posicao_atual(gps)
        posicaoX_atual = posicao_atual[0]
        posicaoY_atual = posicao_atual[1]

        round_func = lambda x: (x if round(x, 2) != 0 else 0)
        set_vel = lambda delta: (
            maxVelocity / 100 if delta >= dist - 0.001 else maxVelocity
        )

        tot_delta = round_func(abs(abs(posicaoX_atual) - abs(posicaoX_anterior))) + round_func(
            abs(abs(posicaoY_atual) - abs(posicaoY_anterior))
        )

        motorEsquerdo.setVelocity(set_vel(tot_delta))
        motorDireito.setVelocity(set_vel(tot_delta))
        delay(25)
        #print(set_vel(tot_delta))

        # print(f"Deveria andar {dist} e andou {tot_delta}")
        
        if get_distance(0,lidar) < 0.040:
            print("BOSTACOLOSSAL")
            parar()
            break

        if tot_delta >= dist:
            parar()
            #print(f"A posição X atual é {posicaoX_atual}")
            #print(f"A posição Y atual é {posicaoY_atual}")
            #print(f" O Delta X é {posicaoX_atual- posicaoX_anterior}")
            #print(f" O Delta Y é {posicaoY_atual- posicaoY_anterior}")
            break

def mover_para_tras(dist, gps, posicao_atual):
    posicaoX_anterior = posicao_atual[0]
    posicaoY_anterior = posicao_atual[1]

    while robot.step(timestep) != -1:
        posicao_atual = get_posicao_atual(gps)
        posicaoX_atual = posicao_atual[0]
        posicaoY_atual = posicao_atual[1]

        round_func = lambda x: (x if round(x, 2) != 0 else 0)
        set_vel = lambda delta: (
            -maxVelocity / 100 if delta >= dist - 0.001 else -maxVelocity
        )

        tot_delta = round_func(abs(abs(posicaoX_atual) - abs(posicaoX_anterior))) + round_func(
            abs(abs(posicaoY_atual) - abs(posicaoY_anterior))
        )

        motorEsquerdo.setVelocity(set_vel(tot_delta))
        motorDireito.setVelocity(set_vel(tot_delta))
        delay(25)
        #print(set_vel(tot_delta))

        # print(f"Deveria andar {dist} e andou {tot_delta}")

        if tot_delta >= dist:
            parar()
            #print(f"A posição X atual é {posicaoX_atual}")
            #print(f"A posição Y atual é {posicaoY_atual}")
            #print(f" O Delta X é {posicaoX_atual- posicaoX_anterior}")
            #print(f" O Delta Y é {posicaoY_atual- posicaoY_anterior}")
            break

def parede_direita():
    # Retorna True se há parede à direita, senão, retorna False
    if get_distance(90,lidar) < 0.06:
        return True
    return False


def parede_esquerda():
    # Retorna True se há parede à esquerda, senão, retorna False
    if get_distance(270,lidar) < 0.06:
        return True
    return False


def parede_frente():
    # Retorna True se há parede à frente, senão, retorna False
    if get_distance(0,lidar) < 0.06:
        return True
    return False

def parede_tras():
    # Retorna True se há parede à tras, senão, retorna False
    if get_distance(180, lidar) < 0.08:
        return True
    return False

def ajustar_distancia():
    # Ajusta a distância do robô para a parede
    if get_distance(0,lidar) < 0.055:
        falta = 0.06 - get_distance(0,lidar)
        print("Ajustando Distancia 1")
        mover_para_tras(falta,gps, get_posicao_atual(gps))
    elif get_distance(0,lidar) < 0.11 and get_distance(0,lidar) > 0.55:
        falta = 0.06 - get_distance(0,lidar)
        print("Ajustando Distancia 2")
        mover_para_frente(falta,gps, get_posicao_atual(gps), lidar)

def seguir_parede():
    if not parede_esquerda() : 
        virar_esquerda()
        mover_para_frente(0.12,gps, get_posicao_atual(gps), lidar)
    elif parede_esquerda() and not parede_frente():
        mover_para_frente(0.12,gps, get_posicao_atual(gps), lidar)
    elif parede_esquerda() and parede_frente() and not parede_direita():
        virar_direita()
        mover_para_frente(0.12,gps, get_posicao_atual(gps), lidar)
    else:
        virar_180()
        delay(100)
        mover_para_frente(0.12,gps, get_posicao_atual(gps), lidar)



while robot.step(timestep) != -1:
    
    x, y = get_mapa(lidar,gps,imu)
    #print([len(x), len(y)])
    x_total.append(x)
    y_total.append(y)
    seguir_parede()
    #ajustar_distancia()
    fig, ax = plt.subplots()
    ax.yaxis.set_inverted(True)
    ax.scatter(x_total,y_total)
    fig.savefig(f"C:\\Users\\goten\\Desktop\\LARCv1\\LARC_Simulation_Codes\\images\\mapa_total.png")
    plt.close(fig)
    delay(100)
    
    '''print(get_posicao_atual(gps))
    delay(5000)
    '''