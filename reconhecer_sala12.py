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

# cameraE
cameraE = robot.getDevice("cameraE")
cameraE.enable(timeStep)
cameraD = robot.getDevice("cameraD")
cameraD.enable(timeStep)

lidar = robot.getDevice("lidar") # Step 2: Retrieve the sensor, named "lidar", from the robot. Note that the sensor name may differ between robots.
lidar.enable(timeStep) # Step 3: Enable the sensor, using the timestep as the update rate

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


victimType = bytes(
    "H", "utf-8"
) 

#recado de aviso para o gustavo: se vc reclamar q o codigo ta feio, eu sei que ta feio
#horroroso, ruim, e eh dificil de ler
#pelo menos EU TERMINEI A MINHA PARTE E o reconhecimento e ele funciona
#mas provavelmente vc vai reclamar antes de ler isso

def valores_gps():
    position = gps.getValues()
    x = int(
        position[0] * 100
    )  # Get the xy coordinates, multiplying by 100 to convert from meters to cm
    y = int(
        position[2] * 100
    )  # We will use these coordinates as an estimate for the victim's position
    return x, y


def delay(ms):
    initTime = robot.getTime()  # Store starting time (in seconds)
    while robot.step(timeStep) != -1:
        if (
            robot.getTime() - initTime
        ) * 1000.0 > ms:  # If time elapsed (converted into ms) is greater than value passed in
            break


def codificar_tipo(tipo):
    x, y = valores_gps()
    victimType = bytes(tipo, "utf-8")
    message = struct.pack("i i c", x, y, victimType)  # Pack the message.
    delay(1300)  # Delay for 1.3 seconds
    emitter.send(message)  # Send out the message
    #print("enviou mensagem")


def get_distance(angle, lidar) -> float:
    rangeImage = lidar.getRangeImage() # Step 4: Retrieve the range image
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
    # Retorna True se há parede à esquerda, senão, retorna False
    if get_distance(270,lidar) < 0.06:
        return True
    return False

def parede_direita():
    # Retorna True se há parede à esquerda, senão, retorna False
    if get_distance(90,lidar) < 0.06:
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

def quant_preto_cropped(camera):
    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )
    # Convert the image to grayscale
    image = image[:, :, :3]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    res = cv2.resize(gray, (img_width, img_height))
    blurred_image = cv2.GaussianBlur(res, (5, 5), 0)
    cropped_image = blurred_image[70:140, 60:300] #height; :widght

    # Count the number of non-black pixels
    non_black_pixels = cv2.countNonZero(cropped_image)

    # Subtract the number of non-black pixels from the total number of pixels
    total_pixels = cropped_image.size
    black_pixels = total_pixels - non_black_pixels

    return black_pixels


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
        
        if polygon_count > 0:
            media /= polygon_count
        
    return(polygon_count, media)  
  
def reconhece_esquerda():
    delay(10)
    preto = quant_preto(cameraE)
    preto_cropped_meio = quant_preto_cropped(cameraE)
    branco = quant_branco(cameraE)
    rangeImage = lidar.getLayerRangeImage(0).copy()
    polygon_count = 0
    hazmat, media = verificar_HSU_ou_hazmat(cameraE)

    pixel_branco = 0
    pixel_preto = 0
    total = 0

    proporção_branco = 0  
    proporção_preto = 0  

    image = cameraE.getImage()
    image = np.frombuffer(image, dtype=np.uint8).reshape((cameraE.getHeight(), cameraE.getWidth(), 4))
    image = image[:, :, :3]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    res = cv2.resize(gray, (img_width, img_height))
    blurred_image = cv2.GaussianBlur(res, (5, 5), 0)
    
    _, binary_image = cv2.threshold(res, 200, 255, cv2.THRESH_BINARY)  
    pixel_branco = np.sum(image == 255)
    pixel_preto = np.sum(image == 0)
    total = pixel_branco + pixel_preto

    proporção_branco = pixel_branco / total if total > 0 else 0
    proporção_preto = pixel_preto / total if total > 0 else 0

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
            codificar_tipo("O")
        elif np.any(mask_red > 0):
            print('F')
            delay(1300)
            codificar_tipo("F")
        elif hazmat >= 4:
            if preto < 10:
                print('P')
                delay(1300)
                codificar_tipo('P')
            else:
                print('C')
                delay(1300)
                codificar_tipo('C')
        elif branco > 500 and preto > 0:
            if polygon_count <= 24:
                # diferenciar entre H ou U
                #U menor de 2000 pretos no meio
                #H media de 2700 no meio
                if preto_cropped_meio <= 2300:
                    print('U')
                    delay(1300)
                    codificar_tipo('U')
                else:
                    print('H')
                    delay(1300)
                    codificar_tipo('H')
            else:
                print('S')
                delay(1300)
                codificar_tipo("S")  
        else: print(f'nenhuma vitima encontrada!')  
    else: print('nenhuma vítima encontrada!')


def reconhece_direita():
    delay(10)
    preto = quant_preto(cameraD)
    preto_cropped_meio = quant_preto_cropped(cameraD)
    branco = quant_branco(cameraD)
    rangeImage = lidar.getLayerRangeImage(0).copy()
    polygon_count = 0
    hazmat, media = verificar_HSU_ou_hazmat(cameraD)

    pixel_branco = 0
    pixel_preto = 0
    total = 0

    proporção_branco = 0  
    proporção_preto = 0  

    image = cameraD.getImage()
    image = np.frombuffer(image, dtype=np.uint8).reshape((cameraD.getHeight(), cameraD.getWidth(), 4))
    image = image[:, :, :3]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    res = cv2.resize(gray, (img_width, img_height))
    blurred_image = cv2.GaussianBlur(res, (5, 5), 0)
    
    _, binary_image = cv2.threshold(res, 200, 255, cv2.THRESH_BINARY)  
    pixel_branco = np.sum(image == 255)
    pixel_preto = np.sum(image == 0)
    total = pixel_branco + pixel_preto

    proporção_branco = pixel_branco / total if total > 0 else 0
    proporção_preto = pixel_preto / total if total > 0 else 0

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
            codificar_tipo("O")
        elif np.any(mask_red > 0):
            print('F')
            delay(1300)
            codificar_tipo("F")
        elif hazmat >= 4:
            if preto < 10:
                print('P')
                delay(1300)
                codificar_tipo('P')
            else:
                print('C')
                delay(1300)
                codificar_tipo('C')
        elif branco > 500 and preto > 0:
            if polygon_count <= 24:
                # diferenciar entre H ou U
                #U menor de 2000 pretos no meio
                #H media de 2700 no meio
                if preto_cropped_meio <= 2300:
                    print('U')
                    delay(1300)
                    codificar_tipo('U')
                else:
                    print('H')
                    delay(1300)
                    codificar_tipo('H')
            else:
                print('S')
                delay(1300)
                codificar_tipo("S")  
        else: print(f'nenhuma vitima encontrada!')  
    else: print('nenhuma vítima encontrada!')



# Main loop
while robot.step(timeStep) != -1:
    reconhece_esquerda()
    reconhece_direita()
