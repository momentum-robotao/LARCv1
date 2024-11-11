import numpy as np
from time import sleep
from devices import Motor
import logging
import os
from datetime import datetime

from controller import Robot as WebotsRobot  # type: ignore

from debugging import ALL_SYSTEMS, DebugInfo, HttpHandler, System
from devices import GPS, IMU, ColorSensor, Communicator, Lidar, Motor
from dfs import dfs
from maze import Maze
from robot import Robot
from types_and_constants import DEBUG, NGROK_URL, ON_DOCKER, Coordinate

if ON_DOCKER:
    import requests  # type: ignore



try:
    log_dir = os.path.dirname(os.getenv("LOG_PATH", ""))
    os.makedirs(log_dir, exist_ok=True)
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(levelname)s %(message)s",
        filename=os.getenv("LOG_PATH"),
    )
    logger = logging.getLogger("Robo LARC v1")
    logger.info(f"Criado log com sucesso em: {os.getenv('LOG_PATH')}")
    if ON_DOCKER:
        print(f"Url do ngrok recuperada: {NGROK_URL}")

        requests.post(f"{NGROK_URL}/start_simulation")

        http_handler = HttpHandler(f"{NGROK_URL}/send")
        http_handler.setLevel(logging.DEBUG)
        formatter = logging.Formatter("%(levelname)s %(message)s")
        http_handler.setFormatter(formatter)
        logger.addHandler(http_handler)

        print("Adicionado handler com ngrok")

except Exception:
    if DEBUG:
        logging.error("Erro ao inicializar o logger", exc_info=True)
        raise

    try:
        # debug_info = DebugInfo(
        #     systems_to_debug=[
        #         e for e in ALL_SYSTEMS if str(e) not in [str(System.lidar_measures)]
        #     ],
        #     systems_to_ignore=[System.lidar_measures],
        # )
        want = [
            # System.dfs_state,
            # System.dfs_decision,
            # System.dfs_verification,
            # System.maze_visited,
            System.unknown_error,
            System.maze_snapshot,
        ]
        want = ALL_SYSTEMS
        debug_info = DebugInfo(
            logger,
            systems_to_debug=want,
            systems_to_ignore=[
                e for e in ALL_SYSTEMS if str(e) not in [str(w) for w in want]
            ],
        )
    except Exception:
        if DEBUG:
            logger.error("Erro ao inicializar o debug info", exc_info=True)
            raise


    # Initialize robot and devices
    try:
        webots_robot = WebotsRobot()
        webots_robot.step(int(os.getenv("TIME_STEP", 32)))

        motor = Motor(webots_robot, debug_info)
        lidar = Lidar(webots_robot, debug_info)
        gps = GPS(webots_robot, debug_info)
        imu = IMU(webots_robot, debug_info)
        color_sensor = ColorSensor(webots_robot, debug_info)
        communicator = Communicator(webots_robot, debug_info)
    except Exception:
        if DEBUG:
            debug_info.send(
                "Erro durante inicialização dos devices", System.initialization, "error"
            )
            raise

    if DEBUG:
        logger.info(f"Começando nova execução: {datetime.now()}")

    try:
        robot = Robot(
            webots_robot, motor, lidar, gps, imu, color_sensor, communicator, debug_info
        )
        robot.step()
    except Exception:
        if DEBUG:
            debug_info.send(
                "Erro durante inicialização do robô", System.initialization, "error"
            )
            raise

IMU_THRESHOLD=np.pi/6

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

radius = (0.071/2) 

imu = robot.getDevice("inertial_unit")
imu.enable(timestep)

lidar = robot.getDevice("lidar") # Step 2: Retrieve the sensor, named "lidar", from the robot. Note that the sensor name may differ between robots.
lidar.enable(timestep) # Step 3: Enable the sensor, using the timestep as the update rate

gps = robot.getDevice("gps")
gps.enable(timestep)

sensor_de_cor = robot.getDevice("colour_sensor")
sensor_de_cor.enable(timestep)

contador_imagem = 0
contador_imagem1 = 0

x_total = []
y_total = []

cor_buraco = b"...\xff"

def radians(degrees: int)->float:
    return degrees*(np.pi/180)


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
    imu_value = round(imu.getRollPitchYaw()[2], 5)
    return (imu_value*(180/np.pi) + 90)*(-1)



def conferir_circulo(centro: list, ponto: list, raio: float) -> bool:
    """
    confere se o ponto está dentro da circunferencia de centro e raio especificado
    """
    if pow(ponto[0] - centro[0], 2) + pow(ponto[1] - centro[1], 2) < pow(raio,2): return True
    return False

''' 
def achar_node(gps,imu, x , y):
    global contador_imagem1
    ori = get_orientation(imu)
    P =  get_posicao_atual(gps)
    nodeX = []
    nodeY = []
    d = 0.03
    soma  = 0
    if ori == 'north':
        soma = -90
    elif ori == 'east':
        soma = 0
    elif ori == 'south':
        soma = 90
    elif ori == 'west':
        soma = 180

    fig_nodes, ax_nodes = plt.subplots()
    for i in range(90): # Tomar cuidado com o intervalo dos angulos da "frente"
        Xl = x[i] + d*np.sin(radians(i))*np.sin(radians(soma)) - d*np.cos(radians(i))*np.cos(radians(soma))
        Yl = y[i] - d*np.cos(radians(i))*np.sin(radians(soma)) - d*np.sin(radians(i))*np.cos(radians(soma))
        condicao = False
        for j in range(360):
            if conferir_circulo([Xl, Yl], [x[j], y[j]], radius) and conferir_circulo([Xl, Yl], [P[0], P[1]], radius):
                condicao = True
                break
        if not condicao: 
            print(f"tentativa par vertice: {[Xl, Yl]}, ponto dentro: {[x[j], y[j]]}, angle {i}, x[{i}] = {x[i]}, y[{i} = {y[i]}]")
            nodeX.append(Xl)
            nodeY.append(Yl)
    for i in range(270,360):
        theta = 360 - i
        Xl = x[i] - d*np.sin(radians(theta))*np.sin(radians(soma)) - d*np.cos(radians(theta))*np.cos(radians(soma))
        Yl = y[i] - d*np.cos(radians(theta))*np.sin(radians(soma)) + d*np.sin(radians(theta))*np.cos(radians(soma))
        condicao = False
        for j in range(360):
            if conferir_circulo([Xl, Yl], [x[j], y[j]], radius) and conferir_circulo([Xl, Yl], [P[0], P[1]], radius):
                condicao = True
                break
        if not condicao: 
            print(f"tentativa par vertice: {[Xl, Yl]}, ponto dentro: {[x[j], y[j]]}, angle {i}, x[{i}] = {x[i]}, y[{i} = {y[i]}]")
            nodeX.append(Xl)
            nodeY.append(Yl)
            ax_nodes.yaxis.set_inverted(True)
            ax_nodes.plot()

    print (f"list size {len(nodeX)} and {len(nodeY)}")
    ax_nodes.scatter(nodeX, nodeY)
    fig_nodes.savefig(f"C:\\Users\\goten\\Desktop\\LARCv1\\LARC_Simulation_Codes\\images\\nodes_{contador_imagem1}.png")
    plt.close(fig_nodes)
    contador_imagem1 +=1
    return nodeX,nodeY
'''


def get_mapa(lidar, gps, imu):
    global contador_imagem
    x = []
    y = []
    soma = get_orientation(imu)

    for i in range(360):
        if (get_distance(i,lidar) > (1.5)*0.12):
            x.append(get_posicao_atual(gps)[0])
            y.append(get_posicao_atual(gps)[1])
        else:
            x.append(get_distance(i,lidar)*np.cos((soma + i)*np.pi/180) + get_posicao_atual(gps)[0])
            y.append(get_distance(i,lidar)*np.sin((soma + i)*np.pi/180) + get_posicao_atual(gps)[1])

    #fig, ax = plt.subplots()
    #ax.yaxis.set_inverted(True)
    #ax.scatter(x, y)
    #ax.plot(get_posicao_atual(gps)[0], get_posicao_atual(gps)[1], 'x')
    #fig.savefig(f"C:\\Users\\goten\\Desktop\\LARCv1\\LARC_Simulation_Codes\\images\\mapa_{contador_imagem}.png")
    #plt.close(fig)
    #contador_imagem +=1
    return np.array(x),np.array(y)
    

def andar_sala4():
    mover_para_frente(0.03, gps, get_posicao_atual(), lidar)


def parar():
    sleep(0.1)
    motorEsquerdo.setVelocity(0.0)
    motorDireito.setVelocity(0.0)


def get_delta_rotation(ang, new_ang):
    if new_ang * ang <= 0:
        if round(ang) == 0:
            return abs(ang) + abs(new_ang)
        else:
            return abs(abs(ang) - np.pi) + abs(abs(new_ang) - np.pi)
    return max(ang, new_ang) - min(ang, new_ang)

def virar(direcao, degrees):
    """
    :param: direcao: should be 'left' or 'right'
    """
    parar()
    robot_to_turn_rad = (degrees / 180) * np.pi

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
        
        if get_distance(0,lidar) < 0.045:
            print("BOSTACOLOSSAL")
            parar()
            break

        if tem_buraco():
            while robot.step(timestep) != -1:
                posicao_atual = gps.getValues()
                posicaoX_atual = posicao_atual[0]
                posicaoY_atual = posicao_atual[2]

                round_func = lambda x: (x if round(x, 2) != 0 else 0)
                set_vel = lambda delta: (
                    maxVelocity / 100 if delta >= dist - 0.001 else maxVelocity
                )

                tot_delta = round_func(abs(abs(posicaoX_atual) - abs(posicaoX_anterior))) + round_func(
                    abs(abs(posicaoY_atual) - abs(posicaoY_anterior))
                )

                motorEsquerdo.setVelocity(-set_vel(tot_delta))
                motorDireito.setVelocity(-set_vel(tot_delta))

                if tot_delta < 0.001:
                    parar()
                    break
            virar_180()
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

def tem_buraco():
    cor = sensor_de_cor.getImage()
    #print(cor)
    if cor == cor_buraco:
        return True
    return False

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
        motor.move('forward', gps, lidar,sensor_de_cor, 0.03)
    elif parede_esquerda() and not parede_frente():
        motor.move('forward', gps, lidar,sensor_de_cor, 0.03)
    elif parede_esquerda() and parede_frente() and not parede_direita():
        virar_direita()
        motor.move('forward', gps, lidar,sensor_de_cor, 0.03)
    else:
        virar_180()
        delay(100)
        motor.move('forward', gps, lidar,sensor_de_cor, 0.03)
    delay(100)

contador_imagem2 = 0
while robot.step(timestep) != -1:


    
    x, y = get_mapa(lidar,gps,imu)
    x_total.append(x)
    y_total.append(y)
    seguir_parede()
    delay(100)
    #ajustar_distancia()
    #fig, ax = plt.subplots()
    #ax.yaxis.set_inverted(True)
    #ax.scatter(x_total,y_total)
    #fig.savefig(f"C:\\Users\\goten\\Desktop\\LARCv1\\LARC_Simulation_Codes\\images\\mapa_total.png")
    #plt.close(fig)
    delay(100)
    