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


sensoresFrente = [robot.getDevice("ps_frente"), robot.getDevice("ps_tras")]
sensoresEsquerda = [
    robot.getDevice("ps_esquerda"),
    robot.getDevice("ps_diagonal_esquerda"),
]
sensoresDireita = [
    robot.getDevice("ps_direita"),
    robot.getDevice("ps_diagonal_direita"),
]

# Ativar sensores
for sensor in sensoresFrente + sensoresEsquerda + sensoresDireita:
    sensor.enable(timeStep)

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


def ajustar_branco(camera):
    global branco_antes
    branco = quant_branco(camera)
    if branco > 600:
        print(f"andar{branco}")
        while True:
            print(f"ta aumentando mlk {branco}, {branco_antes}")
            branco_antes = branco
            encoder_antes()
            mover_para_frente(0.0068)
            branco = quant_branco(camera)
            if branco_antes > branco:
                delay(10)
                branco_antes = 0
                break
        parar()
        print("parou")


def ajustar_vermelho(camera):
    global vermelho_antes
    preto = quant_preto(camera)
    vermelho = quant_vermelho(camera)
    if vermelho > 20 and vermelho > vermelho_antes and preto == 0:
        print(f"sisior,{vermelho}")
        while True:
            print(f"ta aumentando mlk {vermelho}, {vermelho_antes}")
            vermelho_antes = vermelho
            encoder_antes()
            mover_para_frente(0.0068)
            vermelho = quant_vermelho(camera)
            if vermelho_antes > vermelho:
                delay(10)
                vermelho_antes = 0
                break
        parar()
        print("parou")


def ajustar_preto(camera):
    global preto_antes
    preto = quant_preto(camera)
    vermelho = quant_vermelho(camera)
    print(preto)
    if preto > 20 and preto > preto_antes and vermelho == 0:
        print("boraaa,", preto)
        while True:
            print(f"ta aumentando mlk {preto}, {preto_antes}")
            preto_antes = preto
            encoder_antes()
            mover_para_frente(0.0068)
            preto = quant_preto(camera)
            if preto_antes > preto:
                preto_antes = 0
                break
        parar()
        print("parou")


def quant_preto(camera):
    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )
    # Convert the image to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Count the number of non-black pixels
    non_black_pixels = cv2.countNonZero(gray_image)

    # Subtract the number of non-black pixels from the total number of pixels
    total_pixels = gray_image.size
    black_pixels = total_pixels - non_black_pixels

    return black_pixels


# Tem preto
def tem_preto(camera):
    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )

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


# tem amarelo trava o programa
def tem_amarelo(camera):
    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )

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


def quant_vermelho(camera):
    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )
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


def tem_vermelho(camera):
    image = camera.getImage()
    # image = image[30:, 0:]
    image = np.frombuffer(image, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )

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


def quant_branco(camera):
    quant = 0
    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    for linhas in gray_image:
        for pixels in linhas:
            if pixels > 150:
                quant += 1
    return quant


def codificar_tipo(tipo):
    x, y = valores_gps()
    victimType = bytes(tipo, "utf-8")
    message = struct.pack("i i c", x, y, victimType)  # Pack the message.
    delay(1300)  # Delay for 1.3 seconds
    emitter.send(message)  # Send out the message
    print("enviou mensagem")


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
    if sensoresFrente[0].getValue() < 0.15:
        falta = 0.15 - sensoresFrente[0].getValue()

        encoder_antes()
        mover_para_tras(falta)

        encoder_antes()
    elif sensoresFrente[0].getValue() < 0.20 and sensoresFrente[0].getValue() > 0.15:
        falta = 0.15 - sensoresFrente[0].getValue()

        encoder_antes()
        mover_para_frente(falta)

        encoder_antes()


def reconhecer_vitima():
    if parede_frente(sensoresFrente):
        ajustar_distancia()
        cv2.waitKey(1)
        image = cameraE.getImage()
        image = np.frombuffer(image, np.uint8).reshape(
            (cameraE.getHeight(), cameraE.getWidth(), 4)
        )
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper threshold values for black in HSV
        lower_black = np.array([0, 0, 0], dtype=np.uint8)
        upper_black = np.array([180, 255, 185], dtype=np.uint8)
        mask = cv2.inRange(hsv_image, lower_black, upper_black)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.002 * cv2.arcLength(cnt, True), True)
            # print(len(approx))
            if (
                len(approx) >= 8 and len(approx) <= 26
            ):  # and not tem_amarelo() and not tem_vermelho():
                delay(1300)
                codificar_tipo("H")
                print("H")
            elif (
                len(approx) >= 27 and len(approx) <= 33
            ):  # and not tem_amarelo() and not tem_vermelho():
                print("U")
                delay(1300)
                codificar_tipo("U")
            elif (
                len(approx) >= 60 and len(approx) <= 82
            ):  # and not tem_amarelo() and not tem_vermelho():
                print("S")
                delay(1300)
                codificar_tipo("S")
            elif tem_amarelo():  # and len(approx) != 4:
                print("O")
                delay(1300)
                codificar_tipo("O")
            elif tem_vermelho() and len(approx) != 4:
                print("F")
                delay(1300)
                codificar_tipo("F")
            elif tem_preto() and len(approx) != 4:
                print("C")
                delay(1300)
                codificar_tipo("C")

        # Bitwise-AND mask and original image
        # cv2.imshow("Mask", mask)
        cv2.waitKey(1)


def verificar_vitima(camera):
    preto = quant_preto(camera)
    branco = quant_branco(camera)
    vermelho = quant_vermelho(camera)

    parar()
    delay(10)
    print(preto, branco, vermelho)
    if tem_preto(camera) and preto <= 100 and 1 <= branco <= 420:
        print("ajustou")
        ajustar_preto(camera)
        verificar_vitima(camera)
    elif tem_vermelho(camera) and vermelho <= 55:
        print("ajustou")
        ajustar_vermelho(camera)
        verificar_vitima(camera)
    # nos dois if, se a vitima estiver cortada, ele ajusta a distancia e chama novamente a func
    if (preto > 100 and branco > 420) and (
        parede_esquerda(sensoresEsquerda) or parede_direita(sensoresDireita)
    ):
        print("vitima!")
        sleep(0.01)
        return True
    elif (
        vermelho > 55
    ):  # and (parede_esquerda(sensoresEsquerda) or parede_direita(sensoresDireita)):
        print("vítima!")
        sleep(0.01)
        return True
    else:
        print("sem vítima!")
        return False
    # aqui ele retorna True se a vitima inteira estiver encaixada na imagem


def reconhecer_vitima(camera):
    preto = quant_preto(camera)
    parar()
    delay(10)
    if verificar_vitima(camera):
        sleep(0.01)
        print("reconhecendo...")
        possivel_H = [
            "H",
            "HI",
            "IH",
            "IHI",
            "F",
            "FI",
            "IF",
            "IFI",
            "A",
            "AI",
            "IA",
            "IAI",
            "HH",
            "4",
        ]
        possivel_U = [
            "U",
            "UI",
            "IU",
            "IUI",
            "UU",
            "(U)",
            "U)",
            "(U",
            "L",
            "IL",
            "LI",
            "ILI",
            "@",
        ]
        possivel_S = [
            "S",
            "SI",
            "IS",
            "ISI",
            "SIS",
            "SS",
            "5",
            "I5",
            "5I",
            "I5I",
            "75",
            "5L",
            "L5",
            "I5L",
            "5LI",
            "I5LI",
            "7C",
            "9",
            "I9",
            "9I",
            "I9I",
            "28",
            "SE",
            "37",
            "AS",
            "M",
        ]
        possivel_P = [
            "POISO",
            "POIS",
            "POION",
            "POIS0N",
            "P0IS0N",
            "POI50N",
            "P0I50N",
            "P0ISON",
            "P0IS0",
            "P0I50",
            "POISDN",
            "POI5ON",
            "P0I5ON",
            "P0I5O",
            "POI50M",
            "P0I50M",
            "POI5OM",
            "P0I5OM",
            "POIS0M",
            "P0IS0M",
            "PO1S0N",
            "P01S0N",
            "PO1S0",
            "P01S0",
            "POI5UN",
            "P0I5UN",
            "POISUN",
            "P0ISUN",
            "POI50UN",
            "P0I50UN",
            "POISOUN",
            "P0ISOUN",
            "POI5OUN",
            "P0I5OUN",
            "POIS0ON",
            "P0IS0ON",
            "POI5OIN",
            "P0I5OIN",
            "POI5D",
            "P0I5D",
            "POI50D",
            "P0I50D",
            "POIS0N5",
            "P0IS0N5",
            "POI50N5",
            "P0I50N5",
            "POI5ONI",
            "P0I5ONI",
            "POIS0NI",
            "P0IS0NI",
            "POI5ON1",
            "P0I5ON1",
            "POIS0N1",
            "P0IS0N1",
            "POI50N1",
            "P0I50N1",
            "POIS0IN",
            "P0IS0IN",
            "POI5O1N",
            "P0I5O1N",
            "POIS01N",
            "P0IS01N",
            "POI50IN",
            "P0I50IN",
            "OINSON",
            "OISO",
            "SON",
        ]
        possivel_F = [
            "FLAMABLE GAS",
            "FLAMABLE GA5",
            "FLAMABLE GA$",
            "FLAMABLE GAZ",
            "FLAMABLE G4S",
            "FLAMABLE GA",
            "FLAMABLE G4",
            "FLAMABLE G",
            "FLAMABLE",
            "FLAMABLE AS",
            "FLAMABLE 5",
            "FLAMABLE $",
            "FLAMABLE Z",
            "FLAMABLE S",
            "FLAMABLE BLE",
            "FLAMABLE LAMABLE",
            "FLAMABLE A",
            "FLAMABLE B",
            "FLAMABLE E",
            "FLAMABLE M",
            "FLAMABLE N",
            "FLAMABLE L",
            "FLAMABLE G1S",
            "FLAMABLE G3S",
            "FLAMABLE GBS",
            "FLAMABLE GAS1",
            "FLAMABLE GAS3",
            "FLAMABLE GAG",
            "FLAMABLE GAY",
            "FLAMABLE GAZZ",
            "FLAMABLE G4Z",
            "FLAMABLE GA4S",
            "FLAMABLE GA$$",
            "FLAMABLE GAF$",
            "FLAMABLE GA",
            "LAMABLE GAS",
            "AMABLE",
            "GAS",
            "LAMABLE GA",
        ]
        possivel_O = [
            "ORGANIC PEROXIDE",
            "ORG4NIC PEROXIDE",
            "0RGANIC PEROXIDE",
            "0RG4NIC PEROXIDE",
            "ORG4N1C PEROXIDE",
            "0RG4N1C PEROXIDE",
            "ORGANIC PER0XIDE",
            "ORG4NIC PER0XIDE",
            "0RGANIC PER0XIDE",
            "0RG4NIC PER0XIDE",
            "ORG4N1C PER0XIDE",
            "0RG4N1C PER0XIDE",
            "ORGANIC PEROXI9E",
            "ORG4NIC PEROXI9E",
            "0RGANIC PEROXI9E",
            "0RG4NIC PEROXI9E",
            "ORG4N1C PEROXI9E",
            "0RG4N1C PEROXI9E",
            "ORGANIC PEROX1DE",
            "ORG4NIC PEROX1DE",
            "0RGANIC PEROX1DE",
            "0RG4NIC PEROX1DE",
            "ORG4N1C PEROX1DE",
            "0RG4N1C PEROX1DE",
            "ORGANIC PEROXID3",
            "ORG4NIC PEROXID3",
            "0RGANIC PEROXID3",
            "0RG4NIC PEROXID3",
            "ORG4N1C PEROXID3",
            "0RG4N1C PEROXID3",
            "0RGANIC PEROX1D3",
            "0RG4NIC PEROX1D3",
            "0RG4N1C PEROX1D3",
            "RGANIC PEROXIDE",
            "RGANIC",
            "ORGANIC",
            "RXIDE",
            "GANIC PER",
            "ANIC PER",
        ]
        possivel_C = [
            "CORROSIVE",
            "CORR0SIVE",
            "CORROS1VE",
            "C0RR0S1VE",
            "C0RR0SIVE",
            "CORR0S1VE",
            "CORROSIV3",
            "C0RR0SIV3",
            "CORR0S1V3",
            "C0RR0S1V3",
            "CORR0S1V",
            "C0RR0S1V",
            "C0RR0SIVE",
            "CORROSI",
            "CORROS",
            "CORROSIV",
        ]

        ajustar_distancia()
        image = camera.getImage()
        image = np.frombuffer(image, np.uint8).reshape(
            (camera.getHeight(), camera.getWidth(), 4)
        )
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # padrao opencv
        rgb_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2RGB)
        # Define the lower and upper threshold values for black in HSV

        reader = ocr.Reader(["en"])
        result = reader.readtext(rgb_image)
        for item in result:
            text = item[1].upper()
            print(text)
            if text in possivel_H:
                print("H")
                codificar_tipo("H")
                return 0
            elif text in possivel_U:
                print("U")
                codificar_tipo("U")
                return 0
            elif text in possivel_S:
                print("S")
                codificar_tipo("S")
                return 0
            elif text in possivel_C:
                print("C")
                codificar_tipo("C")
                return 0
            elif text in possivel_P:
                print("P")
                codificar_tipo("P")
                return 0
            elif text in possivel_F:
                print("F")
                codificar_tipo("F")
                return 0
            elif text in possivel_O:
                print("O")
                codificar_tipo("O")
                return 0

        if tem_amarelo(camera):
            print("O")
            codificar_tipo("O")
        elif tem_vermelho(camera):
            print("F")
            codificar_tipo("F")
        elif preto > 150:
            print("C")
            codificar_tipo("C")
        elif preto < 150:
            print("P")
            codificar_tipo("P")


def tem_buraco():
    cor = sensor_de_cor.getImage()
    if cor == cor_buraco:
        return True
    return False


def objeto_proximo(posicao):
    # Retorna True se o objeto está próximo, senão, retorna False
    return math.sqrt(posicao[0] ** 2 + posicao[1] ** 2) < 0.1


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
    if sensoresFrente[0].getValue() < 0.20:
        return True
    return False


def parede_tras(sensoresFrente):
    # Retorna True se há parede à tras, senão, retorna False
    if sensoresFrente[1].getValue() < 0.20:
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


def mover_para_frente(dist=tamanho_tile):
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

        tot_delta = round_func(abs(posicaoX_atual - posicaoX_anterior)) + round_func(
            abs(posicaoY_atual - posicaoY_anterior)
        )

        motorEsquerdo.setVelocity(set_vel(tot_delta))
        motorDireito.setVelocity(set_vel(tot_delta))
        print(set_vel(tot_delta))

        # print(f"Deveria andar {dist} e andou {tot_delta}")

        if tot_delta > dist:
            parar()
            print(f"A posição X atual é {posicaoX_atual}")
            print(f"A posição Y atual é {posicaoY_atual}")
            print(f" O Delta X é {posicaoX_atual- posicaoX_anterior}")
            print(f" O Delta Y é {posicaoY_atual- posicaoY_anterior}")
            break


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


direcao = "direita"


def mudar_direcao():
    global direcao
    if round(deltaX, 2) == 0.12 or round(deltaX, 2) == 0.11 or round(deltaX, 2) == 0.13:
        direcao = "direita"
        print(direcao)
    elif (
        round(deltaX, 2) == -0.12
        or round(deltaX, 2) == -0.11
        or round(deltaX, 2) == -0.13
    ):
        direcao = "esquerda"
        print(direcao)
    if round(deltaY, 2) == 0.12 or round(deltaY, 2) == 0.11 or round(deltaY, 2) == 0.13:
        direcao = "baixo"
        print(direcao)
    elif (
        round(deltaY, 2) == -0.12
        or round(deltaY, 2) == -0.11
        or round(deltaY, 2) == -0.13
    ):
        direcao = "cima"
        print(direcao)


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
        print(" A dir é {}".format(dir))
        print(" O sentido é {}".format(sentido))
        return 0


# Função para seguir a parede
def seguir_parede():
    global direcao
    if not parede_esquerda(sensoresEsquerda) and not foi_visitado(direcao, "esquerda"):
        # Se não há parede à esquerda, vire à esquerda e mova-se para frente
        # print('esquerda livre,')
        encoder_antes()
        virar_esquerda(imu.getRollPitchYaw()[2])
        encoder_antes()
        mover_para_frente()
        delay(5)
        print("estou indo para esquerda")

    elif (
        parede_esquerda(sensoresEsquerda)
        and not parede_frente(sensoresFrente)
        and not foi_visitado(direcao, "frente")
    ):
        # Se há parede à esquerda, mas não à frente, mova-se para frente
        # print('esquerda ocupada, frente livre')
        encoder_antes()
        mover_para_frente()
        delay(5)
        print("estou indo para frente")

    elif (
        parede_esquerda(sensoresEsquerda)
        and parede_frente(sensoresFrente)
        and not parede_direita(sensoresDireita)
        and not foi_visitado(direcao, "direita")
    ):
        # Se há parede à esquerda e à frente, vire à direita e mova-se para frente
        # print('esquerda e frente ocupadas')
        encoder_antes()
        virar_direita()
        encoder_antes()
        mover_para_frente()
        delay(5)
        print("estou indo para direita")

    elif (
        parede_esquerda(sensoresEsquerda)
        and parede_frente(sensoresFrente)
        and parede_direita(sensoresDireita)
        and not foi_visitado(direcao, "atras")
    ):
        encoder_antes()
        virar_180()
        delay(5)

    else:
        # BFS
        print("BFS!!")
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
                print("O caminho é {}".format(caminho))
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
                print("DeltaY : {}".format(caminho[i + 1][0] - caminho[i][0]))
                if direcao == "direita":
                    encoder_antes()
                    virar_direita()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()
                if direcao == "esquerda":
                    encoder_antes()
                    virar_esquerda()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()
                if direcao == "cima":
                    encoder_antes()
                    virar_180()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()
                if direcao == "baixo":
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()
                direcao = "baixo"

            elif caminho[i + 1][0] - caminho[i][0] == -2:
                # DeltaY negativo, andar para cima
                print("DeltaY : {}".format(caminho[i + 1][0] - caminho[i][0]))
                if direcao == "direita":
                    encoder_antes()
                    virar_esquerda()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()
                if direcao == "esquerda":
                    encoder_antes()
                    virar_direita()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()
                if direcao == "cima":
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()
                if direcao == "baixo":
                    encoder_antes()
                    virar_180()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()
                direcao = "cima"

            elif caminho[i + 1][1] - caminho[i][1] == 2:
                # DeltaX positivo
                print("DeltaX : {}".format(caminho[i + 1][1] - caminho[i][1]))
                if direcao == "direita":
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()

                if direcao == "esquerda":
                    encoder_antes()
                    virar_180()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()
                if direcao == "cima":

                    encoder_antes()
                    virar_direita()

                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()
                if direcao == "baixo":
                    encoder_antes()
                    virar_esquerda()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()
                direcao = "direita"

            elif caminho[i + 1][1] - caminho[i][1] == -2:
                # DeltaX negativo
                print("DeltaX : {}".format(caminho[i + 1][1] - caminho[i][1]))
                if direcao == "direita":
                    encoder_antes()
                    virar_180()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()
                if direcao == "esquerda":

                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()
                if direcao == "cima":
                    encoder_antes()
                    virar_esquerda()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()
                if direcao == "baixo":
                    encoder_antes()
                    virar_direita()
                    encoder_antes()
                    mover_para_frente()
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()
                direcao = "esquerda"

        if caminho[-1][0] - caminho[-2][0] == 2:
            # DeltaY positivo, andar para baixo
            print("DeltaY : {}".format(caminho[i + 1][0] - caminho[i][0]))
            if direcao == "direita":
                encoder_antes()
                virar_direita()
                encoder_antes()
                mover_para_frente()
                delay(5)
                if parede_frente(sensoresFrente):
                    ajustar_distancia()

            if direcao == "esquerda":
                encoder_antes()
                virar_esquerda()
                encoder_antes()
                mover_para_frente()
                delay(5)
                if parede_frente(sensoresFrente):
                    ajustar_distancia()
            if direcao == "cima":
                encoder_antes()
                virar_180()
                encoder_antes()
                mover_para_frente()
                delay(5)
                if parede_frente(sensoresFrente):
                    ajustar_distancia()
            if direcao == "baixo":
                encoder_antes()
                mover_para_frente()
                delay(5)
                if parede_frente(sensoresFrente):
                    ajustar_distancia()

        elif caminho[-1][0] - caminho[-2][0] == -2:
            # DeltaY negativo, andar para cima
            print("DeltaY : {}".format(caminho[i + 1][0] - caminho[i][0]))
            if direcao == "direita":
                encoder_antes()
                virar_esquerda()
                encoder_antes()
                mover_para_frente()
                delay(5)
                if parede_frente(sensoresFrente):
                    ajustar_distancia()
            if direcao == "esquerda":
                encoder_antes()
                virar_direita()
                encoder_antes()
                mover_para_frente()
                delay(5)
                if parede_frente(sensoresFrente):
                    ajustar_distancia()
            if direcao == "cima":
                encoder_antes()
                mover_para_frente()
                delay(5)
                if parede_frente(sensoresFrente):
                    ajustar_distancia()
            if direcao == "baixo":
                encoder_antes()
                virar_180()
                encoder_antes()
                mover_para_frente()
                delay(5)
                if parede_frente(sensoresFrente):
                    ajustar_distancia()
        elif caminho[-1][1] - caminho[-2][1] == 2:
            # DeltaX positivo
            print("DeltaX : {}".format(caminho[i + 1][1] - caminho[i][1]))
            if direcao == "direita":
                encoder_antes()
                mover_para_frente()
                delay(5)
                if parede_frente(sensoresFrente):
                    ajustar_distancia()

            if direcao == "esquerda":
                encoder_antes()
                virar_180()
                encoder_antes()
                mover_para_frente()
                delay(5)
                if parede_frente(sensoresFrente):
                    ajustar_distancia()
            if direcao == "cima":
                encoder_antes()
                virar_direita()
                encoder_antes()
                mover_para_frente()
                delay(5)
                if parede_frente(sensoresFrente):
                    ajustar_distancia()
            if direcao == "baixo":
                encoder_antes()
                virar_esquerda()
                encoder_antes()
                mover_para_frente()
                delay(5)
                if parede_frente(sensoresFrente):
                    ajustar_distancia()
        elif caminho[-1][1] - caminho[-2][1] == -2:
            # DeltaX negativo
            print("DeltaX : {}".format(caminho[i + 1][1] - caminho[i][1]))
            if direcao == "direita":
                encoder_antes()
                virar_180()
                encoder_antes()
                mover_para_frente()
                delay(5)
                if parede_frente(sensoresFrente):
                    ajustar_distancia()
            if direcao == "esquerda":
                encoder_antes()
                mover_para_frente()
                delay(5)
                if parede_frente(sensoresFrente):
                    ajustar_distancia()
            if direcao == "cima":
                encoder_antes()
                virar_esquerda()
                encoder_antes()
                mover_para_frente()
                delay(5)
                if parede_frente(sensoresFrente):
                    ajustar_distancia()
            if direcao == "baixo":
                encoder_antes()
                virar_direita()
                encoder_antes()
                mover_para_frente()
                delay(5)
                if parede_frente(sensoresFrente):
                    ajustar_distancia()


def PID():
    global erro_anterior
    global soma_erros
    Kp = 38.5
    Ki = 0.4
    Kd = 1.0
    erro = sensoresEsquerda[1].getValue() - 0.07

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
        print("desvio")
        delay(20)
        encoder_antes()
        mover_para_frente(tamanho_tile / 3 * 2)
        ajustar_distancia()
        # reconhecer_vitima()
        print("frente")
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

    if parede_frente(sensoresFrente):
        ajustar_distancia()
        # delay(20)
        # ajustar_distancia()
        # seguir_parede()
        # delay(20)
        print("parede a frente")


"""    print(f'dist{sensoresEsquerda[0].getValue()}')
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
        print("Eu estou no início")
        if not parede_frente(sensoresFrente):
            mapa[coordenada_centro_mapa][coordenada_centro_mapa + n] = 0
            mapa[coordenada_centro_mapa][coordenada_centro_mapa + k] = 3
            listas_vistos.append((coordenada_centro_mapa, coordenada_centro_mapa + k))
        else:
            mapa[coordenada_centro_mapa][coordenada_centro_mapa + n] = 1

        if not parede_esquerda(sensoresEsquerda):
            mapa[coordenada_centro_mapa - n][coordenada_centro_mapa] = 0
            mapa[coordenada_centro_mapa - k][coordenada_centro_mapa] = 3
            listas_vistos.append((coordenada_centro_mapa - k, coordenada_centro_mapa))
        else:
            mapa[coordenada_centro_mapa - n][coordenada_centro_mapa] = 1

        if not parede_direita(sensoresDireita):
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
        print("O Delta X é : {}".format(round(deltaX, 2)))
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
            not parede_frente(sensoresFrente)
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
        elif parede_frente(sensoresFrente):
            mapa[coordenada_linha_atual][coordenada_coluna_atual + n] = 1

        # Esquerda
        if (
            not parede_esquerda(sensoresEsquerda)
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

        elif parede_esquerda(sensoresEsquerda):
            mapa[coordenada_linha_atual - n][coordenada_coluna_atual] = 1

        # Direita
        if (
            not parede_direita(sensoresDireita)
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
        elif parede_direita(sensoresDireita):
            mapa[coordenada_linha_atual + n][coordenada_coluna_atual] = 1

    # DeltaX negativo
    elif (
        round(deltaX, 2) == -0.12
        or round(deltaX, 2) == -0.11
        or round(deltaX, 2) == -0.13
    ):
        print("O Delta X é : {}".format(round(deltaX, 2)))
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
            not parede_frente(sensoresFrente)
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
        elif parede_frente(sensoresFrente):
            mapa[coordenada_linha_atual][coordenada_coluna_atual - n] = 1

        # Esquerda
        if (
            not parede_esquerda(sensoresEsquerda)
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
        elif parede_esquerda(sensoresEsquerda):
            mapa[coordenada_linha_atual + n][coordenada_coluna_atual] = 1

        # Direita
        if (
            not parede_direita(sensoresDireita)
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
        elif parede_direita(sensoresDireita):
            mapa[coordenada_linha_atual - n][coordenada_coluna_atual] = 1

    # DeltaY positivo
    if round(deltaY, 2) == 0.12 or round(deltaY, 2) == 0.11 or round(deltaY, 2) == 0.13:
        print("O Delta Y é : {}".format(round(deltaY, 2)))
        coordenada_linha_atual = coordenada_linha_atual + k
        coordenada_coluna_atual = coordenada_coluna_atual
        print((coordenada_linha_atual, coordenada_coluna_atual))
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
            not parede_frente(sensoresFrente)
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

        elif parede_frente(sensoresFrente):
            mapa[coordenada_linha_atual + n][coordenada_coluna_atual] = 1

        # Direita
        if (
            not parede_direita(sensoresDireita)
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
        elif parede_direita(sensoresDireita):
            mapa[coordenada_linha_atual][coordenada_coluna_atual - n] = 1

        # Esquerda
        if (
            not parede_esquerda(sensoresEsquerda)
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
        elif parede_esquerda(sensoresEsquerda):
            mapa[coordenada_linha_atual][coordenada_coluna_atual + n] = 1

    # DeltaY negativo
    elif (
        round(deltaY, 2) == -0.12
        or round(deltaY, 2) == -0.11
        or round(deltaY, 2) == -0.13
    ):
        print("O Delta Y é : {}".format(round(deltaY, 2)))
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
            not parede_frente(sensoresFrente)
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
        elif parede_frente(sensoresFrente):
            mapa[coordenada_linha_atual - n][coordenada_coluna_atual] = 1

        # Direita
        if (
            not parede_direita(sensoresDireita)
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
        elif parede_direita(sensoresDireita):
            mapa[coordenada_linha_atual][coordenada_coluna_atual + n] = 1

        # Esquerda
        if (
            not parede_esquerda(sensoresEsquerda)
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
        elif parede_esquerda(sensoresEsquerda):
            mapa[coordenada_linha_atual][coordenada_coluna_atual - n] = 1

    """
    #Robo nao se mexe no eixo Y
    else :
        print("O robo nao está andando no eixo Y. O Delta Y é : {}".format(round(deltaY,2)))

    """

    posicao_anterior = posicao_atual
    posicaoX_anterior = posicaoX_atual
    posicaoY_anterior = posicaoY_atual
    mudar_direcao()


# Loop principal
mover_para_frente(0.005)
sleep(0.1)  # gambiarra essencial para o robô não bugar :) NÃO TIRAR

while robot.step(timeStep) != -1:
    seguir_parede()
    mapeamento()
    ajustar_distancia()
    # PID()
    parar()
    # reconhecer_vitima()

    print("A lista de tiles vistos é : {}".format(listas_vistos))
    print("A lista de tiles marcados é : {}".format(lista_tiles_marcados))
    reconhecer_vitima(cameraE)
