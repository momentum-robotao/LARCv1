import math
import struct
import numpy as np
import cv2
from controller import Robot

img_height, img_width = 256, 320

timeStep = 32

robot = Robot()
motorEsquerdo = robot.getDevice("left motor")
motorDireito = robot.getDevice("right motor")
encoders = motorEsquerdo.getPositionSensor()

cameraE = robot.getDevice("cameraE")
cameraE.enable(timeStep)
cameraD = robot.getDevice("cameraD")
cameraD.enable(timeStep)

lidar = robot.getDevice("lidar") 
lidar.enable(timeStep) 

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
) 

imu = robot.getDevice("inertial_unit")
imu.enable(timeStep)



motorEsquerdo.setPosition(float("inf"))
motorDireito.setPosition(float("inf"))
motorEsquerdo.setVelocity(0.0)
motorDireito.setVelocity(0.0)
encoders.enable(timeStep)


victimType = bytes(
    "H", "utf-8"
) 



def H_S_U(camera, target_width=320, target_height=256):

    if camera == 'cameraE':
        dist = get_distance(270,lidar)   
    else:
        dist = get_distance(90,lidar)

    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)

        cropped_image = image[y:y+h, x:x+w]
        shape = cropped_image.shape
        altura = shape[0]
        largura = shape[1]
        quadrado = altura / largura
        margem = 0.45


        resized_image = cv2.resize(cropped_image, (target_width, target_height))

        gray_resized = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)

        _, black_white_image = cv2.threshold(gray_resized, 127, 255, cv2.THRESH_BINARY)

        lower_half = black_white_image[190:210, :]  
        non_black_pixels_lower = cv2.countNonZero(lower_half)
        total_pixels_lower = lower_half.size
        preto_baixo = total_pixels_lower - non_black_pixels_lower 

        mid_half = black_white_image[110:130, :] 
        non_black_pixels_mid = cv2.countNonZero(mid_half)
        total_pixels_mid = mid_half.size
        preto_meio = total_pixels_mid - non_black_pixels_mid

        upper_half = black_white_image[55:75, :] 
        non_black_pixels_upper = cv2.countNonZero(upper_half)
        total_pixels_upper = upper_half.size
        preto_cima = total_pixels_upper - non_black_pixels_upper

        vertical_slice = black_white_image[20:230, 118:138]
        non_black_pixels_vertical = cv2.countNonZero(vertical_slice)
        total_pixels_vertical = vertical_slice.size
        preto_vertical = (total_pixels_vertical - non_black_pixels_vertical) * 1.52
        preto_vertical = int(preto_vertical)

        


        if (1 - margem) <= quadrado <= (1 + margem):
            if preto_vertical == min(preto_cima, preto_meio, preto_baixo, preto_vertical) and preto_baixo == max(preto_cima, preto_meio, preto_baixo, preto_vertical):
                return 'U'
            elif preto_meio == max(preto_cima, preto_baixo, preto_meio) and preto_vertical == min(preto_cima, preto_baixo, preto_vertical):
                return 'H'
            elif preto_vertical == max(preto_cima, preto_baixo, preto_meio, preto_vertical):
                return 'S'
            else:
                return 0

def valores_gps():
    position = gps.getValues()
    x = int(
        position[0] * 100
    )  
    y = int(
        position[2] * 100
    )  
    return x, y


def delay(ms):
    initTime = robot.getTime()  
    while robot.step(timeStep) != -1:
        if (
            robot.getTime() - initTime
        ) * 1000.0 > ms:  
            break


def codificar_tipo(tipo):
    x, y = valores_gps()
    victimType = bytes(tipo, "utf-8")
    message = struct.pack("i i c", x, y, victimType)  
    delay(1300)  
    emitter.send(message)  



def get_distance(angle, lidar) -> float:
    rangeImage = lidar.getRangeImage() 
    n = (angle*512)//360
 
    dist = 0
    count = 0
    for i in range(4):
        if rangeImage[n+ i*512] != float('inf') : 
            dist += rangeImage[n+ i*512]
            count +=1

    dist = dist/count

    return dist

def parede_esquerda():

    if get_distance(270,lidar) < 0.08:
        return True
    return False

def parede_direita():

    if get_distance(90,lidar) < 0.08:
        return True
    return False

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



def quant_preto(camera):
    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    non_black_pixels = cv2.countNonZero(gray_image)

    total_pixels = gray_image.size
    black_pixels = total_pixels - non_black_pixels

    return black_pixels

def verificar_HSU_ou_hazmat(camera):
    
    media = 0
    polygon_count = 0
    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    res = cv2.resize(image, (img_width, img_height))
    hsv_image = cv2.cvtColor(res, cv2.COLOR_BGR2HSV)

    # Define the lower and upper threshold values for black in HSV
    lower_black = np.array([0, 0, 0], dtype=np.uint8)
    upper_black = np.array([180, 255, 185], dtype=np.uint8)
    mask = cv2.inRange(hsv_image, lower_black, upper_black)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE,
                                    cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,
                                0.002 * cv2.arcLength(cnt, True), True)
        if len(approx) >= 3:
            polygon_count += 1
            media += len(approx)
        
        
    return polygon_count  
  
def reconhece_esquerda():
    delay(10)
    dist = get_distance(270,lidar)
    preto = quant_preto(cameraE)
    branco = quant_branco(cameraE)
    polygon_count = 0
    hazmat = verificar_HSU_ou_hazmat(cameraE)

    image = cameraE.getImage()
    image = np.frombuffer(image, dtype=np.uint8).reshape((cameraE.getHeight(), cameraE.getWidth(), 4))
    image = image[:, :, :3]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    res = cv2.resize(gray, (img_width, img_height))
    blurred_image = cv2.GaussianBlur(res, (5, 5), 0)


    edges = cv2.Canny(blurred_image, 50, 150)

    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if len(approx) >= 3:
            polygon_count += 1
            

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    if parede_esquerda():
        if np.any(mask_yellow > 0):
            print('O')
            delay(1300)
            #codificar_tipo("O")
        elif np.any(mask_red > 0):
            print('F')
            delay(1300)
            #codificar_tipo("F")
        elif hazmat >= 4:
            if preto < 10:
                print('P')
                delay(1300)
                #codificar_tipo('P')
            else:
                print('C')
                delay(1300)
                #codificar_tipo('C')
        elif branco > 0 and preto > 0 and dist > 0.043 and dist < 0.068:
            if H_S_U(cameraE) == 'H':
                print('H')
                delay(1300)
                #codificar_tipo('H')
            elif H_S_U(cameraE) == 'S':
                print('S')
                delay(1300)
                #codificar_tipo('S')
            elif H_S_U(cameraE) == 'U':
                print('U')
                delay(1300)
                #codificar_tipo('U')  
            else: 
                #tem vitima, mas para reconhecer corretamente ela deve estar inteira na imagem  
                #faria uma estrategia para "encaixar" a vitima inteira
                pass
        elif branco > 0 and preto > 0 and dist < 0.043:
            print('perto')  
        else: 
            #nao tem vitima nenhuma na imagem
            pass


def reconhece_direita():
    delay(10)
    dist = get_distance(90,lidar)
    preto = quant_preto(cameraD)
    branco = quant_branco(cameraD)
    polygon_count = 0
    hazmat = verificar_HSU_ou_hazmat(cameraD)

    image = cameraD.getImage()
    image = np.frombuffer(image, dtype=np.uint8).reshape((cameraD.getHeight(), cameraD.getWidth(), 4))
    image = image[:, :, :3]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    res = cv2.resize(gray, (img_width, img_height))
    blurred_image = cv2.GaussianBlur(res, (5, 5), 0)


    edges = cv2.Canny(blurred_image, 50, 150)

    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if len(approx) >= 3:
            polygon_count += 1
            

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    if parede_direita():
        if np.any(mask_yellow > 0):
            print('O')
            delay(1300)
            #codificar_tipo("O")
        elif np.any(mask_red > 0):
            print('F')
            delay(1300)
            #codificar_tipo("F")
        elif hazmat >= 4:
            if preto < 10:
                print('P')
                delay(1300)
                #codificar_tipo('P')
            else:
                print('C')
                delay(1300)
                #codificar_tipo('C')
        elif branco > 0 and preto > 0 and dist > 0.047 and dist < 0.068:
            if H_S_U(cameraD) == 'H':
                print('H')
                delay(1300)
                #codificar_tipo('H')
            elif H_S_U(cameraD) == 'S':
                print('S')
                delay(1300)
                #codificar_tipo('S')
            elif H_S_U(cameraD) == 'U':
                print('U')
                delay(1300)
                #codificar_tipo('U')  
            else: 
                #tem vitima, mas para reconhecer corretamente ela deve estar inteira na imagem  
                #faria uma estratefia para ajustar o robo ate a vitima ficar inteira
                pass
        elif branco > 0 and preto > 0 and dist < 0.047:
            print('perto')  




while robot.step(timeStep) != -1:
    reconhece_esquerda()
    reconhece_direita()
