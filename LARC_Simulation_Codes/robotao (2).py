import math
import struct
from time import sleep

import cv2
import easyocr as ocr
import numpy as np
from controller import Robot

branco_antes = 0.0
vermelho_antes = 0.0
preto_antes = 0.0
tamanho_tile = 0.12
erro_anterior = 0
soma_erros = 0
cor_buraco = b"...\xff"
PI = 3.141592653589
timeStep = 32
maxVelocity = 6.28
encoder_inicial = 0.0
andou = 0.0
rad = 4.29552
mensagem_enviada = False
posicao_anterior = (0, 0, 0)
posicaoX_anterior = 0
posicaoY_anterior = 0
posicao_atual = (0, 0, 0)
posicaoX_atual = 0
posicaoY_atual = 0
tamanho_mapa = 501
coordenada_centro_mapa = tamanho_mapa // 2
n = 1
k = 2
direcao = ""
tile = 5.85069

robot = Robot()
motorEsquerdo = robot.getDevice("left motor")
motorDireito = robot.getDevice("right motor")
encoders = motorEsquerdo.getPositionSensor()

# cameraE
cameraE = robot.getDevice("cameraE")
cameraE.enable(timeStep)
cameraD = robot.getDevice("cameraD")
cameraD.enable(timeStep)

lidar = robot.getDevice("lidar") # Step 2: Retrieve the sensor, named "lidar", from the robot. Note that the sensor name may differ between robots.
lidar.enable(timeStep) # Step 3: Enable the sensor, using the timestep as the update rate

''' 
rangeImage = [robot.getDevice("ps_frente"), robot.getDevice("ps_tras")]
rangeImage = [
    robot.getDevice("ps_esquerda"),
    robot.getDevice("ps_diagonal_esquerda"),
]
rangeImage = [
    robot.getDevice("ps_direita"),
    robot.getDevice("ps_diagonal_direita"),
]

# Ativar sensores
for sensor in rangeImage + rangeImage + rangeImage:
    sensor.enable(timeStep)
'''
sensor_de_cor = robot.getDevice("colour_sensor")
sensor_de_cor.enable(timeStep)

emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
receiver.enable(timeStep)

gps = robot.getDevice("gps")
gps.enable(timeStep)
position = gps.getValues()
victimTypeH = bytes(
    "H", "utf-8"
)  # The victim type being sent is the letter 'H' for harmed victim

imu = robot.getDevice("inertial_unit")
imu.enable(timeStep)


# Inicializar motores e sensores
motorEsquerdo.setPosition(float("inf"))
motorDireito.setPosition(float("inf"))
motorEsquerdo.setVelocity(0.0)
motorDireito.setVelocity(0.0)
encoders.enable(timeStep)


sleep(0.2)

victimType = bytes(
    "H", "utf-8"
)  # The victim type being sent is the letter 'H' for harmed victim

# Definir o mapa (Nesse Caso eu fiz um mapa 201x201, o centro é o mapa[100][100])
mapa = np.array([[-1] * tamanho_mapa for _ in range(tamanho_mapa)])
mapa[coordenada_centro_mapa][coordenada_centro_mapa] = 2
coordenada_linha_atual = coordenada_centro_mapa
coordenada_coluna_atual = coordenada_centro_mapa
deltaX = 0
deltaY = 0


def get_delta_rotation(ang, new_ang):
    if new_ang * ang <= 0:
        if round(ang) == 0:
            return abs(ang) + abs(new_ang)
        else:
            return abs(abs(ang) - PI) + abs(abs(new_ang) - PI)
    return max(ang, new_ang) - min(ang, new_ang)





def delay(ms):
    initTime = robot.getTime()  # Store starting time (in seconds)
    while robot.step(timeStep) != -1:
        if (
            robot.getTime() - initTime
        ) * 1000.0 > ms:  # If time elapsed (converted into ms) is greater than value passed in
            break


def valores_gps():
    position = gps.getValues()
    x = int(
        position[0] * 100
    )  # Get the xy coordinates, multiplying by 100 to convert from meters to cm
    y = int(
        position[2] * 100
    )  # We will use these coordinates as an estimate for the victim's position
    return x, y


def ajustar_distancia():
    # Ajusta a distância do robô para a parede
    if get_distance(0,lidar) < 0.06:
        falta = 0.06 - get_distance(0,lidar)
        print("Ajustando Distancia 1")
        mover_para_frente(-falta)
    elif get_distance(0,lidar) < 0.12 and get_distance(0,lidar) > 0.6:
        falta = 0.06 - get_distance(0,lidar)
        print("Ajustando Distancia 2")
        mover_para_frente(falta)


def tem_buraco():
    cor = sensor_de_cor.getImage()
    #print(cor)
    if cor == cor_buraco:
        return True
    return False


def objeto_proximo(posicao):
    # Retorna True se o objeto está próximo, senão, retorna False
    return math.sqrt(posicao[0] ** 2 + posicao[1] ** 2) < 0.1

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

def parede_direita(rangeImage = None):
    # Retorna True se há parede à direita, senão, retorna False
    if get_distance(90,lidar) < 0.06:
        return True
    return False


def parede_esquerda(rangeImage = None):
    # Retorna True se há parede à esquerda, senão, retorna False
    if get_distance(270,lidar) < 0.06:
        return True
    return False


def parede_frente(rangeImage = None):
    # Retorna True se há parede à frente, senão, retorna False
    if get_distance(0,lidar) < 0.06:
        return True
    return False


def parede_tras(rangeImage= None):
    # Retorna True se há parede à tras, senão, retorna False
    if get_distance(180, lidar) < 0.08:
        return True
    return False


def encoder_antes():
    # Retorna o valor do encoder antes de mover
    global encoder_inicial
    encoder_inicial = float(encoders.getValue())
    return encoder_inicial

def mover_para_tras(dist):
    global posicao_atual, posicaoX_atual, posicaoY_atual
    global posicao_anterior, posicaoX_anterior, posicaoY_anterior
    posicaoX_anterior = posicao_atual[0]
    posicaoY_anterior = posicao_atual[2]

    dist = -dist

    while robot.step(timeStep) != -1:
        posicao_atual = gps.getValues()
        posicaoX_atual = posicao_atual[0]
        posicaoY_atual = posicao_atual[2]

        round_func = lambda x: (x if round(x, 2) != 0 else 0)
        set_vel = lambda delta: (
            maxVelocity / 100 if delta >= dist - 0.001 else maxVelocity
        )

        tot_delta = round_func(abs(abs(posicaoX_atual) - abs(posicaoX_anterior))) + round_func(
            abs((posicaoY_atual) - abs(posicaoY_anterior))
        )

        motorEsquerdo.setVelocity(-set_vel(tot_delta))
        motorDireito.setVelocity(-set_vel(tot_delta))
        #print(set_vel(tot_delta))

        # print(f"Deveria andar {dist} e andou {tot_delta}")

        if tot_delta <= dist:
            parar()
            #print(f"A posição X atual é {posicaoX_atual}")
            #print(f"A posição Y atual é {posicaoY_atual}")
            #print(f" O Delta X é {posicaoX_atual- posicaoX_anterior}")
            #print(f" O Delta Y é {posicaoY_atual- posicaoY_anterior}")
            break
        delay(25)

def mover_para_frente(dist):
    global posicao_atual, posicaoX_atual, posicaoY_atual
    global posicao_anterior, posicaoX_anterior, posicaoY_anterior
    posicaoX_anterior = posicao_atual[0]
    posicaoY_anterior = posicao_atual[2]

    while robot.step(timeStep) != -1:
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

        motorEsquerdo.setVelocity(set_vel(tot_delta))
        motorDireito.setVelocity(set_vel(tot_delta))
        delay(25)
        #print(set_vel(tot_delta))

        # print(f"Deveria andar {dist} e andou {tot_delta}")

        if tot_delta > dist:
            parar()
            #print(f"A posição X atual é {posicaoX_atual}")
            #print(f"A posição Y atual é {posicaoY_atual}")
            #print(f" O Delta X é {posicaoX_atual- posicaoX_anterior}")
            #print(f" O Delta Y é {posicaoY_atual- posicaoY_anterior}")
            break


        if tem_buraco():
            while robot.step(timeStep) != -1:
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

def mover_para_frente(dist=tamanho_tile, should_recognize_victim=True):
    global posicao_atual, posicaoX_atual, posicaoY_atual
    global posicao_anterior, posicaoX_anterior, posicaoY_anterior
    posicaoX_anterior = posicao_atual[0]
    posicaoY_anterior = posicao_atual[2]

    while robot.step(timeStep) != -1:
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

        motorEsquerdo.setVelocity(set_vel(tot_delta))
        motorDireito.setVelocity(set_vel(tot_delta))

        #print(set_vel(tot_delta))

        # print(f"Deveria andar {dist} e andou {tot_delta}")

        if tot_delta > dist:
            parar()
            #print(f"A posição X atual é {posicaoX_atual}")
            #print(f"A posição Y atual é {posicaoY_atual}")
            #print(f" O Delta X é {posicaoX_atual- posicaoX_anterior}")
            #print(f" O Delta Y é {posicaoY_atual- posicaoY_anterior}")
            break


        if tem_buraco():
            while robot.step(timeStep) != -1:
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
        if should_recognize_victim:
            # Reconhecer vítima
            mover_para_frente(dist, should_recognize_victim=False)


def parar():
    motorEsquerdo.setVelocity(0.0)
    motorDireito.setVelocity(0.0)


def virar(direcao, degrees):
    """
    :param: direcao: should be 'left' or 'right'
    """
    parar()
    robot_to_turn_rad = (degrees / 180) * PI

    # print(f"======== Começando girada de {degrees}° e {robot_to_turn_rad} rad ========")

    accumulator = 0
    ang = imu.getRollPitchYaw()[2]

    set_vel = lambda faltante: (maxVelocity if faltante > 0.1 else maxVelocity / 100)

    while robot.step(timeStep) != -1:
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


def mudar_direcao(imu):
    orientation = {'north' : 0, 'east' : round(-math.pi/2,2), 'south' : round(math.pi,2), 'west': round(math.pi/2,2)}
    for key in orientation : 
        #print(round(imu.getRollPitchYaw()[2], 2))
        if round(imu.getRollPitchYaw()[2], 2) == orientation[key] : 
            return key
        pass


def foi_visitado(dir, sentido):
    if dir == "direita":
        if (
            sentido == "direita"
            and mapa[coordenada_linha_atual + k][coordenada_coluna_atual] == 2
        ):
            return 1
        if (
            sentido == "esquerda"
            and mapa[coordenada_linha_atual - k][coordenada_coluna_atual] == 2
        ):
            return 1
        if (
            sentido == "atras"
            and mapa[coordenada_linha_atual][coordenada_coluna_atual - k] == 2
        ):
            return 1
        if (
            sentido == "frente"
            and mapa[coordenada_linha_atual][coordenada_coluna_atual + k] == 2
        ):
            return 1

    if dir == "esquerda":
        if (
            sentido == "direita"
            and mapa[coordenada_linha_atual - k][coordenada_coluna_atual] == 2
        ):
            return 1
        if (
            sentido == "esquerda"
            and mapa[coordenada_linha_atual + k][coordenada_coluna_atual] == 2
        ):
            return 1
        if (
            sentido == "atras"
            and mapa[coordenada_linha_atual][coordenada_coluna_atual + k] == 2
        ):
            return 1
        if (
            sentido == "frente"
            and mapa[coordenada_linha_atual][coordenada_coluna_atual - k] == 2
        ):
            return 1

    if dir == "baixo":
        if (
            sentido == "direita"
            and mapa[coordenada_linha_atual][coordenada_coluna_atual - k] == 2
        ):
            return 1
        if (
            sentido == "esquerda"
            and mapa[coordenada_linha_atual][coordenada_coluna_atual + k] == 2
        ):
            return 1
        if (
            sentido == "atras"
            and mapa[coordenada_linha_atual - k][coordenada_coluna_atual] == 2
        ):
            return 1
        if (
            sentido == "frente"
            and mapa[coordenada_linha_atual + k][coordenada_coluna_atual] == 2
        ):
            return 1

    if dir == "cima":
        if (
            sentido == "direita"
            and mapa[coordenada_linha_atual][coordenada_coluna_atual + k] == 2
        ):
            return 1
        if (
            sentido == "esquerda"
            and mapa[coordenada_linha_atual][coordenada_coluna_atual - k] == 2
        ):
            return 1
        if (
            sentido == "atras"
            and mapa[coordenada_linha_atual + k][coordenada_coluna_atual] == 2
        ):
            return 1
        if (
            sentido == "frente"
            and mapa[coordenada_linha_atual - k][coordenada_coluna_atual] == 2
        ):
            return 1
    else:
        #print(" A dir é {}".format(dir))
       # print(" O sentido é {}".format(sentido))
        return 0


# Função para seguir a parede
def seguir_parede():
    global direcao
    print("direcao ", direcao)
    rangeImage = lidar.getLayerRangeImage(0).copy()
    print(rangeImage[0])
    if not parede_esquerda(rangeImage) and not foi_visitado(direcao, "esquerda"):
        # Se não há parede à esquerda, vire à esquerda e mova-se para frente
        # print('esquerda livre,')
        encoder_antes()
        virar_esquerda(imu.getRollPitchYaw()[2])
        encoder_antes()
        mover_para_frente()
        delay(5)
       # print("estou indo para esquerda")

    elif (
        parede_esquerda(rangeImage)
        and not parede_frente(rangeImage)
        and not foi_visitado(direcao, "frente")
    ):
        # Se há parede à esquerda, mas não à frente, mova-se para frente
        # print('esquerda ocupada, frente livre')
        encoder_antes()
        mover_para_frente()
        delay(5)
       # print("estou indo para frente")

    elif (
        parede_esquerda(rangeImage)
        and parede_frente(rangeImage)
        and not parede_direita(rangeImage)
        and not foi_visitado(direcao, "direita")
    ):
        # Se há parede à esquerda e à frente, vire à direita e mova-se para frente
        # print('esquerda e frente ocupadas')
        encoder_antes()
        virar_direita()
        encoder_antes()
        mover_para_frente()
        delay(5)
        #print("estou indo para direita")

    elif (
        parede_esquerda(rangeImage)
        and parede_frente(rangeImage)
        and parede_direita(rangeImage)
        and not foi_visitado(direcao, "atras")
    ):
        encoder_antes()
        virar_180()
        delay(5)

    else:
        # BFS
        #print("BFS!!")
        """ 
        for li in mapa:
            print(li)
        """
        cl = coordenada_linha_atual
        cc = coordenada_coluna_atual
        atual = (cl, cc)
        fila = []
        marc = []
        dist = {}
        path = {}
        fila.append((cl, cc))
        marc.append((cl, cc))
        dist[atual] = 0
        # print("listas_vistos[-1] é {}".format(listas_vistos[-2]))
        found_path = False
        while len(fila) != 0 and not found_path:
            atual = fila.pop(0)
            cl = atual[0]
            cc = atual[1]
            if cl < 0 or cc < 0:
                continue
            try:
                hngrgigrwuirh = mapa[cl][cc]
            except:
                continue
            # print("estou no ", atual)
            if atual == listas_vistos[-2]:
                # print("Achei!!!!!!!!!")
                caminho = [listas_vistos[-2]]
                found_path = True
                aux = listas_vistos[-2]
                while aux != (coordenada_linha_atual, coordenada_coluna_atual):
                    caminho.append(path[aux])
                    aux = path[aux]
                caminho.reverse()
                #print("O caminho é {}".format(caminho))
                continue

            # print(mapa[cl][cc + n])

            if not mapa[cl][cc + n] == 1 and (cl, cc + k) not in marc:
                fila.append((cl, cc + k))
                marc.append((cl, cc + k))
                dist[(cl, cc + k)] = dist[atual] + 1
                path[(cl, cc + k)] = atual

            if not mapa[cl][cc - n] == 1 and (cl, cc - k) not in marc:
                fila.append((cl, cc - k))
                marc.append((cl, cc - k))
                dist[(cl, cc - k)] = dist[atual] + 1
                path[(cl, cc - k)] = atual

            if not mapa[cl + n][cc] == 1 and (cl + k, cc) not in marc:
                fila.append((cl + k, cc))
                marc.append((cl + k, cc))
                dist[(cl + k, cc)] = dist[atual] + 1
                path[(cl + k, cc)] = atual

            if not mapa[cl - n][cc] == 1 and (cl - k, cc) not in marc:
                fila.append((cl - k, cc))
                marc.append((cl - k, cc))
                dist[(cl - k, cc)] = dist[atual] + 1
                path[(cl - k, cc)] = atual

        # print(dist)
        # print(atual)
        # Andar percorrendo o caminho
        for i in range(len(caminho) - 1):
            if caminho[i + 1][0] - caminho[i][0] == 2:
                # DeltaY positivo, andar para baixo
                print("1036DeltaY : {}".format(caminho[i + 1][0] - caminho[i][0]))
                print(direcao)
                if direcao == "direita":
                    encoder_antes()
                    virar_direita()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                elif direcao == "esquerda":
                    encoder_antes()
                    virar_esquerda()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                elif direcao == "cima":
                    encoder_antes()
                    virar_180()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                elif direcao == "baixo":
                    encoder_antes()
                    #mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                direcao = "baixo"

            elif caminho[i + 1][0] - caminho[i][0] == -2:
                # DeltaY negativo, andar para cima
                print("1071DeltaY : {}".format(caminho[i + 1][0] - caminho[i][0]))
                if direcao == "direita":
                    encoder_antes()
                    virar_esquerda()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                elif direcao == "esquerda":
                    encoder_antes()
                    virar_direita()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                elif direcao == "cima":
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                elif direcao == "baixo":
                    encoder_antes()
                    virar_180()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                direcao = "cima"

            elif caminho[i + 1][1] - caminho[i][1] == 2:
                # DeltaX positivo
                print("1106DeltaX : {}".format(caminho[i + 1][1] - caminho[i][1]))
                if direcao == "direita":
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()

                elif direcao == "esquerda":
                    encoder_antes()
                    virar_180()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                elif direcao == "cima":

                    encoder_antes()
                    virar_direita()

                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                elif direcao == "baixo":
                    encoder_antes()
                    virar_esquerda()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                direcao = "direita"

            elif caminho[i + 1][1] - caminho[i][1] == -2:
                # DeltaX negativo
                print("1144DeltaX : {}".format(caminho[i + 1][1] - caminho[i][1]))
                print(direcao)
                if direcao == "direita":
                    encoder_antes()
                    virar_180()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                elif direcao == "esquerda":

                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                elif direcao == "cima":
                    encoder_antes()
                    virar_esquerda()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                elif direcao == "baixo":
                    encoder_antes()
                    virar_direita()
                    encoder_antes()
                    # mover_para_frente()
                    # delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                direcao = "esquerda"
        try:
            if caminho[-1][0] - caminho[-2][0] == 2:
                # DeltaY positivo, andar para baixo
                print("1180DeltaY : {}".format(caminho[i + 1][0] - caminho[i][0]))
                print(direcao)
                if direcao == "direita":
                    encoder_antes()
                    virar_direita()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()

                elif direcao == "esquerda":
                    encoder_antes()
                    virar_esquerda()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                elif direcao == "cima":
                    encoder_antes()
                    virar_180()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                elif direcao == "baixo":
                    encoder_antes()
                    #mover_para_frente()
                    #delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()

            elif caminho[-1][0] - caminho[-2][0] == -2:
                # DeltaY negativo, andar para cima
                print("1215DeltaY : {}".format(caminho[i + 1][0] - caminho[i][0]))
                if direcao == "direita":
                    encoder_antes()
                    virar_esquerda()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                elif direcao == "esquerda":
                    encoder_antes()
                    virar_direita()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                elif direcao == "cima":
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                elif direcao == "baixo":
                    encoder_antes()
                    virar_180()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
            elif caminho[-1][1] - caminho[-2][1] == 2:
                # DeltaX positivo
                print("1248DeltaX : {}".format(caminho[i + 1][1] - caminho[i][1]))
                if direcao == "direita":
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()

                elif direcao == "esquerda":
                    encoder_antes()
                    virar_180()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                elif direcao == "cima":
                    encoder_antes()
                    virar_direita()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                elif direcao == "baixo":
                    encoder_antes()
                    virar_esquerda()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
            elif caminho[-1][1] - caminho[-2][1] == -2:
                # DeltaX negativo
                print("1282DeltaX : {}".format(caminho[i + 1][1] - caminho[i][1]))
                if direcao == "direita":
                    encoder_antes()
                    virar_180()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                elif direcao == "esquerda":
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                elif direcao == "cima":
                    encoder_antes()
                    virar_esquerda()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
                elif direcao == "baixo":
                    encoder_antes()
                    virar_direita()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(rangeImage):
                        ajustar_distancia()
        except Exception as erro:
            print(erro)

def PID():
    rangeImage = lidar.getLayerRangeImage(0).copy()
    global erro_anterior
    global soma_erros
    Kp = 38.5
    Ki = 0.4
    Kd = 1.0
    erro = rangeImage[1].getValue() - 0.07

    proporcional = Kp * erro

    parar()
    """ 
    reconhecer_vitima()
    if tem_preto():
        ajustar_preto() #se tiver preto na esq, ajusta o centro e reconhece
        delay(200)
    elif tem_vermelho():
        ajustar_vermelho()
        delay(200)
    """

    """ 
    if tem_buraco():
        andou = encoders.getValue() - initial_angle
        mover_para_tras(1.0)
        
encoder_antes()
        girar()
        
encoder_antes()
        virar_direita()
    """
    if proporcional > 12.28:
        proporcional = 12.28

    if erro - erro_anterior > 0.2:
        parar()
        #print("desvio")
        delay(20)
        encoder_antes()
        mover_para_frente(tamanho_tile / 3 * 2)
        ajustar_distancia()
        # reconhecer_vitima()
        #print("frente")
        # delay(20)
        # seguir_parede()
        # delay(20)

    if erro > 0:
        # subtrai do motor esq e mantem o direito
        motorEsquerdo.setVelocity(maxVelocity - proporcional)
        motorDireito.setVelocity(maxVelocity)

        erro_anterior = erro
        soma_erros += erro
    else:
        # subtrai do motor direito e mantem o esquerdo
        motorEsquerdo.setVelocity(maxVelocity)
        motorDireito.setVelocity(maxVelocity + proporcional)
        erro_anterior = erro
        soma_erros += erro

    if parede_frente(rangeImage):
        ajustar_distancia()
        # delay(20)
        # ajustar_distancia()
        # seguir_parede()
        # delay(20)
       # print("parede a frente")


"""    print(f'dist{rangeImage[0].getValue()}')
    print(f'erro: {erro}')
    print(f'proporcional: {proporcional}')"""


# Mapeamento
listas_vistos = []
lista_tiles_marcados = [(coordenada_centro_mapa, coordenada_centro_mapa)]


def mapeamento():
    global posicao_atual, posicaoX_atual, posicaoY_atual
    global posicao_anterior, posicaoX_anterior, posicaoY_anterior
    global listas_vistos, lista_tiles_marcados
    global deltaX, deltaY
    global coordenada_coluna_atual, coordenada_linha_atual

    posicao_atual = gps.getValues()
    posicaoX_atual = posicao_atual[0]
    posicaoY_atual = posicao_atual[2]

    deltaX = posicaoX_atual - posicaoX_anterior
    deltaY = posicaoY_atual - posicaoY_anterior
    
    rangeImage = lidar.getLayerRangeImage(0).copy()
    """
    Legenda
    0 -> Sem parede
    1 -> Parde
    2 -> Tile Visitado
    3 -> Tile Visto
    -1 -> Tile Indefinido
    """

    # Início
    if round(deltaX, 2) == round(posicaoX_atual, 2) and round(deltaY, 2) == round(
        posicaoY_atual, 2
    ):
        #print("Eu estou no início")
        if not parede_frente(rangeImage):
            mapa[coordenada_centro_mapa][coordenada_centro_mapa + n] = 0
            mapa[coordenada_centro_mapa][coordenada_centro_mapa + k] = 3
            listas_vistos.append((coordenada_centro_mapa, coordenada_centro_mapa + k))
        else:
            mapa[coordenada_centro_mapa][coordenada_centro_mapa + n] = 1

        if not parede_esquerda(rangeImage):
            mapa[coordenada_centro_mapa - n][coordenada_centro_mapa] = 0
            mapa[coordenada_centro_mapa - k][coordenada_centro_mapa] = 3
            listas_vistos.append((coordenada_centro_mapa - k, coordenada_centro_mapa))
        else:
            mapa[coordenada_centro_mapa - n][coordenada_centro_mapa] = 1

        if not parede_direita(rangeImage):
            mapa[coordenada_centro_mapa + n][coordenada_centro_mapa] = 0
            mapa[coordenada_centro_mapa + k][coordenada_centro_mapa] = 3
            listas_vistos.append((coordenada_centro_mapa + k, coordenada_centro_mapa))
        else:
            mapa[coordenada_centro_mapa + n][coordenada_centro_mapa] = 1

        mapa[coordenada_centro_mapa][
            coordenada_centro_mapa - n
        ] = 1  # Assumindo que tenha uma parede atrás

    # DeltaX positivo
    if round(deltaX, 2) == 0.12 or round(deltaX, 2) == 0.11 or round(deltaX, 2) == 0.13:
        #print("O Delta X é : {}".format(round(deltaX, 2)))
        coordenada_coluna_atual = coordenada_coluna_atual + k
        coordenada_linha_atual = coordenada_linha_atual

        # Processo de Remover Tiles Visitados das lista de tiles vistos e marcando eles (adicionando na lista de tiles_marcados)
        if (coordenada_linha_atual, coordenada_coluna_atual) in listas_vistos:
            listas_vistos.pop(
                listas_vistos.index((coordenada_linha_atual, coordenada_coluna_atual))
            )
            lista_tiles_marcados.append(
                (coordenada_linha_atual, coordenada_coluna_atual)
            )

        # Trocar o estado do tile de visto para visitado
        if mapa[coordenada_linha_atual][coordenada_coluna_atual] == 3:
            mapa[coordenada_linha_atual][coordenada_coluna_atual] = 2

        # Frente
        if (
            not parede_frente(rangeImage)
            and mapa[coordenada_linha_atual][coordenada_coluna_atual + k] != 2
        ):
            mapa[coordenada_linha_atual][coordenada_coluna_atual + n] = 0
            mapa[coordenada_linha_atual][coordenada_coluna_atual + k] = 3

            # Processo de Adicionar o tile na lista de tiles vistos
            if (
                not (coordenada_linha_atual, coordenada_coluna_atual + k)
                in lista_tiles_marcados
            ):
                listas_vistos.append(
                    (coordenada_linha_atual, coordenada_coluna_atual + k)
                )
        elif parede_frente(rangeImage):
            mapa[coordenada_linha_atual][coordenada_coluna_atual + n] = 1

        # Esquerda
        if (
            not parede_esquerda(rangeImage)
            and mapa[coordenada_linha_atual - k][coordenada_coluna_atual] != 2
        ):
            mapa[coordenada_linha_atual - n][coordenada_coluna_atual] = 0
            mapa[coordenada_linha_atual - k][coordenada_coluna_atual] = 3
            # Processo de Adicionar o tile na lista de tiles vistos
            if (
                not (coordenada_linha_atual - k, coordenada_coluna_atual)
                in lista_tiles_marcados
            ):
                listas_vistos.append(
                    (coordenada_linha_atual - k, coordenada_coluna_atual)
                )

        elif parede_esquerda(rangeImage):
            mapa[coordenada_linha_atual - n][coordenada_coluna_atual] = 1

        # Direita
        if (
            not parede_direita(rangeImage)
            and mapa[coordenada_linha_atual + k][coordenada_coluna_atual] != 2
        ):
            mapa[coordenada_linha_atual + n][coordenada_coluna_atual] = 0
            mapa[coordenada_linha_atual + k][coordenada_coluna_atual] = 3
            # Processo de Adicionar o tile na lista de tiles vistos
            if (
                not (coordenada_linha_atual + k, coordenada_coluna_atual)
                in lista_tiles_marcados
            ):
                listas_vistos.append(
                    (coordenada_linha_atual + k, coordenada_coluna_atual)
                )
        elif parede_direita(rangeImage):
            mapa[coordenada_linha_atual + n][coordenada_coluna_atual] = 1

    # DeltaX negativo
    elif (
        round(deltaX, 2) == -0.12
        or round(deltaX, 2) == -0.11
        or round(deltaX, 2) == -0.13
    ):
        #print("O Delta X é : {}".format(round(deltaX, 2)))
        coordenada_coluna_atual = coordenada_coluna_atual - k
        coordenada_linha_atual = coordenada_linha_atual

        # Processo de Remover Tiles Visitados das lista de tiles vistos e marcando eles (adicionando na lista de tiles_marcados)
        if (coordenada_linha_atual, coordenada_coluna_atual) in listas_vistos:
            listas_vistos.pop(
                listas_vistos.index((coordenada_linha_atual, coordenada_coluna_atual))
            )
            lista_tiles_marcados.append(
                (coordenada_linha_atual, coordenada_coluna_atual)
            )

        # Trocar o estado do tile de visto para visitado
        if mapa[coordenada_linha_atual][coordenada_coluna_atual] == 3:
            mapa[coordenada_linha_atual][coordenada_coluna_atual] = 2

        # Frente
        if (
            not parede_frente(rangeImage)
            and mapa[coordenada_linha_atual][coordenada_coluna_atual - k] != 2
        ):
            mapa[coordenada_linha_atual][coordenada_coluna_atual - n] = 0
            mapa[coordenada_linha_atual][coordenada_coluna_atual - k] = 3
            # Processo de Adicionar o tile na lista de tiles vistos
            if (
                not (coordenada_linha_atual, coordenada_coluna_atual - k)
                in lista_tiles_marcados
            ):
                listas_vistos.append(
                    (coordenada_linha_atual, coordenada_coluna_atual - k)
                )
        elif parede_frente(rangeImage):
            mapa[coordenada_linha_atual][coordenada_coluna_atual - n] = 1

        # Esquerda
        if (
            not parede_esquerda(rangeImage)
            and mapa[coordenada_linha_atual + k][coordenada_coluna_atual] != 2
        ):
            mapa[coordenada_linha_atual + n][coordenada_coluna_atual] = 0
            mapa[coordenada_linha_atual + k][coordenada_coluna_atual] = 3
            # Processo de Adicionar o tile na lista de tiles vistos
            if (
                not (coordenada_linha_atual + k, coordenada_coluna_atual)
                in lista_tiles_marcados
            ):
                listas_vistos.append(
                    (coordenada_linha_atual + k, coordenada_coluna_atual)
                )
        elif parede_esquerda(rangeImage):
            mapa[coordenada_linha_atual + n][coordenada_coluna_atual] = 1

        # Direita
        if (
            not parede_direita(rangeImage)
            and mapa[coordenada_linha_atual - k][coordenada_coluna_atual] != 2
        ):
            mapa[coordenada_linha_atual - n][coordenada_coluna_atual] = 0
            mapa[coordenada_linha_atual - k][coordenada_coluna_atual] = 3
            # Processo de Adicionar o tile na lista de tiles vistos
            if (
                not (coordenada_linha_atual - k, coordenada_coluna_atual)
                in lista_tiles_marcados
            ):
                listas_vistos.append(
                    (coordenada_linha_atual - k, coordenada_coluna_atual)
                )
        elif parede_direita(rangeImage):
            mapa[coordenada_linha_atual - n][coordenada_coluna_atual] = 1

    # DeltaY positivo
    if round(deltaY, 2) == 0.12 or round(deltaY, 2) == 0.11 or round(deltaY, 2) == 0.13:
        #print("O Delta Y é : {}".format(round(deltaY, 2)))
        coordenada_linha_atual = coordenada_linha_atual + k
        coordenada_coluna_atual = coordenada_coluna_atual
        #print((coordenada_linha_atual, coordenada_coluna_atual))
        # Processo de Remover Tiles Visitados das lista de tiles vistos e marcando eles (adicionando na lista de tiles_marcados)
        if (coordenada_linha_atual, coordenada_coluna_atual) in listas_vistos:
            listas_vistos.pop(
                listas_vistos.index((coordenada_linha_atual, coordenada_coluna_atual))
            )
            lista_tiles_marcados.append(
                (coordenada_linha_atual, coordenada_coluna_atual)
            )

        # Trocar o tile visto por visitado
        if mapa[coordenada_linha_atual][coordenada_coluna_atual] == 3:
            mapa[coordenada_linha_atual][coordenada_coluna_atual] = 2

        # Frente
        if (
            not parede_frente(rangeImage)
            and mapa[coordenada_linha_atual + k][coordenada_coluna_atual] != 2
        ):
            mapa[coordenada_linha_atual + n][coordenada_coluna_atual] = 0
            mapa[coordenada_linha_atual + k][coordenada_coluna_atual] = 3
            # Processo de Adicionar o tile na lista de tiles vistos
            if (
                not (coordenada_linha_atual + k, coordenada_coluna_atual)
                in lista_tiles_marcados
            ):
                listas_vistos.append(
                    (coordenada_linha_atual + k, coordenada_coluna_atual)
                )

        elif parede_frente(rangeImage):
            mapa[coordenada_linha_atual + n][coordenada_coluna_atual] = 1

        # Direita
        if (
            not parede_direita(rangeImage)
            and mapa[coordenada_linha_atual][coordenada_coluna_atual - k] != 2
        ):
            mapa[coordenada_linha_atual][coordenada_coluna_atual - n] = 0
            mapa[coordenada_linha_atual][coordenada_coluna_atual - k] = 3
            # Processo de Adicionar o tile na lista de tiles vistos
            if (
                not (coordenada_linha_atual, coordenada_coluna_atual - k)
                in lista_tiles_marcados
            ):
                listas_vistos.append(
                    (coordenada_linha_atual, coordenada_coluna_atual - k)
                )
        elif parede_direita(rangeImage):
            mapa[coordenada_linha_atual][coordenada_coluna_atual - n] = 1

        # Esquerda
        if (
            not parede_esquerda(rangeImage)
            and mapa[coordenada_linha_atual][coordenada_coluna_atual + k] != 2
        ):
            mapa[coordenada_linha_atual][coordenada_coluna_atual + n] = 0
            mapa[coordenada_linha_atual][coordenada_coluna_atual + k] = 3
            # Processo de Adicionar o tile na lista de tiles vistos
            if (
                not (coordenada_linha_atual, coordenada_coluna_atual + k)
                in lista_tiles_marcados
            ):
                listas_vistos.append(
                    (coordenada_linha_atual, coordenada_coluna_atual + k)
                )
        elif parede_esquerda(rangeImage):
            mapa[coordenada_linha_atual][coordenada_coluna_atual + n] = 1

    # DeltaY negativo
    elif (
        round(deltaY, 2) == -0.12
        or round(deltaY, 2) == -0.11
        or round(deltaY, 2) == -0.13
    ):
        #print("O Delta Y é : {}".format(round(deltaY, 2)))
        coordenada_linha_atual = coordenada_linha_atual - k
        coordenada_coluna_atual = coordenada_coluna_atual

        # Processo de Remover Tiles Visitados das lista de tiles vistos e marcando eles (adicionando na lista de tiles_marcados)
        if (coordenada_linha_atual, coordenada_coluna_atual) in listas_vistos:
            listas_vistos.pop(
                listas_vistos.index((coordenada_linha_atual, coordenada_coluna_atual))
            )
            lista_tiles_marcados.append(
                (coordenada_linha_atual, coordenada_coluna_atual)
            )

        # Trocando o tile visto por visitado
        if mapa[coordenada_linha_atual][coordenada_coluna_atual] == 3:
            mapa[coordenada_linha_atual][coordenada_coluna_atual] = 2

        # Frente
        if (
            not parede_frente(rangeImage)
            and mapa[coordenada_linha_atual - k][coordenada_coluna_atual] != 2
        ):
            mapa[coordenada_linha_atual - n][coordenada_coluna_atual] = 0
            mapa[coordenada_linha_atual - k][coordenada_coluna_atual] = 3
            # Processo de Adicionar o tile na lista de tiles vistos
            if (
                not (coordenada_linha_atual - k, coordenada_coluna_atual)
                in lista_tiles_marcados
            ):
                listas_vistos.append(
                    (coordenada_linha_atual - k, coordenada_coluna_atual)
                )
        elif parede_frente(rangeImage):
            mapa[coordenada_linha_atual - n][coordenada_coluna_atual] = 1

        # Direita
        if (
            not parede_direita(rangeImage)
            and mapa[coordenada_linha_atual][coordenada_coluna_atual + k] != 2
        ):
            mapa[coordenada_linha_atual][coordenada_coluna_atual + n] = 0
            mapa[coordenada_linha_atual][coordenada_coluna_atual + k] = 3
            # Processo de Adicionar o tile na lista de tiles vistos
            if (
                not (coordenada_linha_atual, coordenada_coluna_atual + k)
                in lista_tiles_marcados
            ):
                listas_vistos.append(
                    (coordenada_linha_atual, coordenada_coluna_atual + k)
                )
        elif parede_direita(rangeImage):
            mapa[coordenada_linha_atual][coordenada_coluna_atual + n] = 1

        # Esquerda
        if (
            not parede_esquerda(rangeImage)
            and mapa[coordenada_linha_atual][coordenada_coluna_atual - k] != 2
        ):
            mapa[coordenada_linha_atual][coordenada_coluna_atual - n] = 0
            mapa[coordenada_linha_atual][coordenada_coluna_atual - k] = 3
            # Processo de Adicionar o tile na lista de tiles vistos
            if (
                not (coordenada_linha_atual, coordenada_coluna_atual - k)
                in lista_tiles_marcados
            ):
                listas_vistos.append(
                    (coordenada_linha_atual, coordenada_coluna_atual - k)
                )
        elif parede_esquerda(rangeImage):
            mapa[coordenada_linha_atual][coordenada_coluna_atual - n] = 1

    """
    #Robo nao se mexe no eixo Y
    else :
        print("O robo nao está andando no eixo Y. O Delta Y é : {}".format(round(deltaY,2)))

    """

    posicao_anterior = posicao_atual
    posicaoX_anterior = posicaoX_atual
    posicaoY_anterior = posicaoY_atual
    mudar_direcao(imu)

while robot.step(timeStep) != -1:
    seguir_parede()
    mapeamento()
    ajustar_distancia()
    #PID()
    parar()

    # print("A lista de tiles vistos é : {}".format(listas_vistos))
    # print("A lista de tiles marcados é : {}".format(lista_tiles_marcados))
    # sleep(1)'''
    pass