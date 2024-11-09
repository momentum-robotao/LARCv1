import math
import struct
import numpy as np
import cv2
from controller import Robot
from controller import Camera

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
#fov_atual = cameraE.getFov()
#fov_atual = math.degrees(fov_atual)

#isso aqui não funciona ^^, ele sempre printa que tem 1 radiano
#de fov mesmo que seja uma camera muito grande ou pequena
#eu testei na mao e descobri q o FOV da camera no robo q eu vou colocar no
#git eh 20 graus, entao eu vou usar isso pra fazer o calculo de distancia ate a vitima


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

def dist_branco(camera):
    
    camera_name = camera.getName()
    if camera_name == 'cameraE':
        lidar_angle_base = 270
    else:
        lidar_angle_base = 90

    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0, 0, 200], dtype=np.uint8)
    upper_white = np.array([180, 55, 255], dtype=np.uint8)

    mask = cv2.inRange(hsv_image, lower_white, upper_white)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)

        central_x = x + w // 2

        # calculo do angulo relativo ao ponto central da mascara, nta extamente cert, mas ta bom osuficiente
        fov = 20  
        angle_offset = (central_x / 320.0) * fov - (fov / 2)
        lidar_angle = lidar_angle_base + angle_offset

        lidar_index = int(round(lidar_angle))

        distance_to_central_point = get_distance(lidar_index, lidar)

        return distance_to_central_point
    else :
        return 0

def organic_peroxide(camera):
    #essencialmente a mesma coisa da outra função, só muda a cor

    camera_name = camera.getName()
    if camera_name == 'cameraE':
        lidar_angle_base = 270
    else:
        lidar_angle_base = 90

    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_yellow = np.array([20, 100, 100], dtype=np.uint8)
    upper_yellow = np.array([30, 255, 255], dtype=np.uint8)

    mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)

        central_x = x + w // 2


        fov = 20  
        angle_offset = (central_x / 320.0) * fov - (fov / 2)
        lidar_angle = lidar_angle_base + angle_offset

        lidar_index = int(round(lidar_angle))

        distance_to_central_point = get_distance(lidar_index, lidar)

        if distance_to_central_point < 0.07:
            return 1

    else:    
        return 0


def flamable_gas(camera):
    #essesncialemnte a mesma coisa das outras funções, só muda a cor
    camera_name = camera.getName()
    if camera_name == 'cameraE':
        lidar_angle_base = 270
    else:
        lidar_angle_base = 90
    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 70, 50], dtype=np.uint8)
    upper_red1 = np.array([10, 255, 255], dtype=np.uint8)
    lower_red2 = np.array([170, 70, 50], dtype=np.uint8)
    upper_red2 = np.array([180, 255, 255], dtype=np.uint8)

    mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)

        central_x = x + w // 2
        fov = 20  
        angle_offset = (central_x / 320.0) * fov - (fov / 2)
        lidar_angle = lidar_angle_base + angle_offset

        lidar_index = int(round(lidar_angle))

        distance_to_central_point = get_distance(lidar_index, lidar)
        if distance_to_central_point < 0.07:
            return 1


def H_S_U(camera, target_width=320, target_height=256):

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
    # mudei isso aqui pra pegar o quarter tile, ent so de ter parede na camera ele ja vai reconhecer
    for angle in range(260, 281):
        if get_distance(angle, lidar) < 0.08:
            return True
    return False


def parede_direita():
    # mesma coisa do de cima, so mudei o range (+-10)
    for angle in range(80, 101):
        if get_distance(angle, lidar) < 0.08:
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

    preto = quant_preto(cameraE)

    hazmat = verificar_HSU_ou_hazmat(cameraE)

    branco = dist_branco(cameraE)


    if parede_esquerda():
        if organic_peroxide(cameraE):
            #esse range pode ser mais suave, pq a cor eh facil de reconhecer
            print('O')
            delay(1300)
            #codificar_tipo("O")
        elif flamable_gas(cameraE):
            print('F')
            delay(1300)
            #codificar_tipo("F")
        elif hazmat >= 4 and branco < 0.07:
            if preto < 50:
                print('P')
                delay(1300)
                #codificar_tipo('P')
            else:
                print('C')
                delay(1300)
                #codificar_tipo('C')
        elif branco < 0.068 and branco > 0.05:
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
        elif branco < 0.05:
            #nota importanto, com a nova camera com FOV mais alto, menor do que 0,5 nao pega a parte de baixo e de cima da vitima
            #entao eu vou fazer um metodo pra ele reconhecer de perto tbm
            print('perto')  
        else: 
            #nao tem vitima nenhuma na imagem
            print('N')
            pass


def reconhece_direita():
    delay(10)

    preto = quant_preto(cameraD)

    hazmat = verificar_HSU_ou_hazmat(cameraD)

    branco = dist_branco(cameraD)


    if parede_esquerda():
        if organic_peroxide(cameraD):
            #esse range pode ser mais suave, pq a cor eh facil de reconhecer
            print('O')
            delay(1300)
            #codificar_tipo("O")
        elif flamable_gas(cameraD):
            print('F')
            delay(1300)
            #codificar_tipo("F")
        elif hazmat >= 4 and branco < 0.07:
            if preto < 50:
                print('P')
                delay(1300)
                #codificar_tipo('P')
            else:
                print('C')
                delay(1300)
                #codificar_tipo('C')
        elif branco < 0.068 and branco > 0.05:
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
                #faria uma estrategia para "encaixar" a vitima inteira
                pass
        elif branco < 0.05:
            #nota importanto, com a nova camera com FOV mais alto, menor do que 0,5 nao pega a parte de baixo e de cima da vitima
            #entao eu vou fazer um metodo pra ele reconhecer de perto tbm
            print('perto')  
        else: 
            #nao tem vitima nenhuma na imagem
            print('N')
            pass



while robot.step(timeStep) != -1:
    reconhece_esquerda()
    reconhece_direita()


