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

def get_distance(angle, lidar) -> float:
    rangeImage = lidar.getRangeImage()
    if rangeImage is None:
        raise ValueError("LIDAR range image is None")

    n = (angle * 512) // 360

    dist = 0
    count = 0
    for i in range(4):
        index = n + i * 512
        if index < len(rangeImage) and rangeImage[index] != float('inf'):
            dist += rangeImage[index]
            count += 1

    dist = dist / count if count > 0 else float('inf')
    return dist

def dist_branco(camera):
    #logica: pega o maior contorno branco da imagem, e divide a imagem no meio
    #se maior parte dos pixels brancos estiverem para a direita, por exemplo
    #ele envia um sinal do lidar para o ponto DENTRO contorno mais distante do centro da imagem
    #e retorna a distancia, no caso das outras 2 cores, se for menor que 0.08, ele retorna true
    # se tiver qualquer duvida em como integrar essas funções no seu codigo manda msg

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
        center_x = image.shape[1] // 2 #centro da imagem ('linha imaginaria')

        max_distance = 0
        farthest_point = None
        for point in largest_contour:
            x, y = point[0]
            distance = abs(x - center_x)
            if distance > max_distance:
                max_distance = distance
                farthest_point = (x, y)

        if farthest_point:
            farthest_x, farthest_y = farthest_point


            angle_offset = (farthest_x / image.shape[1]) * 20 - 10 #angulo relativo da camera
            lidar_angle = lidar_angle_base + angle_offset# angulo para o lidar

            lidar_index = int(round(lidar_angle))

            distance_to_farthest_point = get_distance(lidar_index, lidar)


            return distance_to_farthest_point

    return 0

def organic_peroxide(camera):
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

    # Contar a quantidade de pixels na máscara
    non_black_pixels = cv2.countNonZero(mask)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        center_x = image.shape[1] // 2

        # Encontrar o ponto mais distante do centro no eixo x
        max_distance = 0
        farthest_point = None
        for point in largest_contour:
            x, y = point[0]
            distance = abs(x - center_x)
            if distance > max_distance:
                max_distance = distance
                farthest_point = (x, y)

        if farthest_point:
            farthest_x, farthest_y = farthest_point

            # Calcular o ângulo relativo ao ponto mais distante do centro
            angle_offset = (farthest_x / image.shape[1]) * 20 - 10
            lidar_angle = lidar_angle_base + angle_offset

            lidar_index = int(round(lidar_angle))

            distance_to_farthest_point = get_distance(lidar_index, lidar)


            if distance_to_farthest_point < 0.08 and non_black_pixels > 60:
                return 1

    return 0

def flamable_gas(camera):
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

    # Contar a quantidade de pixels na máscara
    non_black_pixels = cv2.countNonZero(mask)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        center_x = image.shape[1] // 2

        # Encontrar o ponto mais distante do centro no eixo x
        max_distance = 0
        farthest_point = None
        for point in largest_contour:
            x, y = point[0]
            distance = abs(x - center_x)
            if distance > max_distance:
                max_distance = distance
                farthest_point = (x, y)

        if farthest_point:
            farthest_x, farthest_y = farthest_point
            # Calcular o ângulo relativo ao ponto mais distante do centro
            angle_offset = (farthest_x / image.shape[1]) * 20 - 10
            lidar_angle = lidar_angle_base + angle_offset

            lidar_index = int(round(lidar_angle))

            distance_to_farthest_point = get_distance(lidar_index, lidar)

            if distance_to_farthest_point < 0.08 and non_black_pixels > 60:
                return 1

    return 0


def robo_torto(lidar, camera):
    #coloquei camera so pra conseguir pegar o lado certo de maneira facil
    step = 20

    camera_name = camera.getName()
    if camera_name == 'cameraE':
        start_angle = 250
        end_angle = 290
    else:
        start_angle=70
        end_angle=110

    distancias = []
    min_distance = float('inf')
    min_angle = -1

    for angle in range(start_angle, end_angle + 1, step):
        distance = get_distance(angle, lidar)
        distancias.append(distance) 

        if distance < min_distance:
            min_distance = distance
            min_angle = angle
    print(min_angle)
    print(min_distance)
    if min_angle == 90 or min_angle == 270:
        #robo ta certinho, mantem a margem
        return 0
    else:
        #aumenta a margem do quadrado
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
        if robo_torto(lidar, camera) == 1:
            margem = 0.5
            print('margem aumentada')
        else:
            print('margem normal')
            margem = 0.2

        # se a vitima (cropped image) estiver inteira na imagem, a imagem vai ser um quadrado, mas
        # quando o robo ta angulado em relação a parede, a imagem da vitima vai ser tipo um trapezio (ponto de fuga no fundo
        # da imagem), entao eu fiz um metodo pra aumentar a margem do quadrado, se o robo estiver angulado
        # como eu não tenho o docker pra rodar o codigo, tem que ir mudando essa marge, mas deve ta meio bom


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

def H_S_U_perto(camera, target_width=320, target_height=256):
#basicamente igal ao outro reconhecedor de HSU,unica diferença eh e ele não usa o contorno branco, pega direto a imagem
#e cria um quadrado, de modo a encaixar a maior quantidade de pixels brancos na imagem (ai isso tira as bordas pretas, que nao sao vitima)

    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0, 0, 200], dtype=np.uint8)
    upper_white = np.array([180, 55, 255], dtype=np.uint8)

    mask = cv2.inRange(hsv_image, lower_white, upper_white)


    # Encontrar a caixa ao redor da região branca
    x, y, w, h = cv2.boundingRect(mask)

    # Ajustar a caixa para ser um quadrado
    if w > h:
        y = max(0, y - (w - h) // 2)
        h = w
    else:
        x = max(0, x - (h - w) // 2)
        w = h

    # Verificar se a caixa delimitadora tem dimensões válidas
    if w > 0 and h > 0:
        # Recortar a imagem para a caixa ajustada
        cropped_image = image[y:y+h, x:x+w]

        # Mostrar a imagem recortada para verificação


        # Redimensionar a imagem recortada
        resized_image = cv2.resize(cropped_image, (target_width, target_height))

        gray_resized = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)

        _, black_white_image = cv2.threshold(gray_resized, 127, 255, cv2.THRESH_BINARY)

        branco = cv2.countNonZero(black_white_image)
        print(branco)

        lower_half = black_white_image[210:230, :]  
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
        if branco > 50000:
            #garantir que a vitima ta inteira na image, sujeit a mudanças
            if preto_vertical == min(preto_cima, preto_baixo, preto_vertical) and preto_baixo == max(preto_cima, preto_meio, preto_baixo):
                return 'U'
            elif preto_meio == max(preto_cima, preto_baixo, preto_meio) and preto_vertical == min(preto_cima, preto_baixo, preto_vertical):
                return 'H'
            elif preto_vertical == max(preto_cima, preto_baixo, preto_vertical):
                return 'S'
            else:
                return 0
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
            #distancia ideal para reconhecimento + ou - 0.06
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
            if H_S_U_perto(cameraE) == 'H':
                print('H')
                delay(1300)
                #codificar_tipo('H')
            if H_S_U_perto(cameraE) == 'S':
                print('S')
                delay(1300)
                #codificar_tipo('S')
            if H_S_U_perto(cameraE) == 'U':
                print('U')
                delay(1300)
                #codificar_tipo('U')  
        else: 
            #nao tem vitima nenhuma na imagem
            print('N')
            pass


def reconhece_direita():
    delay(10)

    preto = quant_preto(cameraD)

    hazmat = verificar_HSU_ou_hazmat(cameraD)

    branco = dist_branco(cameraD)


    if parede_direita():
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
            if H_S_U_perto(cameraE) == 'H':
                print('H')
                delay(1300)
                #codificar_tipo('H')
            if H_S_U_perto(cameraE) == 'S':
                print('S')
                delay(1300)
                #codificar_tipo('S')
            if H_S_U_perto(cameraE) == 'U':
                print('U')
                delay(1300)
                #codificar_tipo('U')   
        else: 
            #nao tem vitima nenhuma na imagem
            print('N')
            pass



#qualquer duvida me chama no whats, eu fiz o codigo meio correndo, ent pode ter uns bugs
#qualquer duvida no reconehcimento de HSU, me chama q eu te explico como funciona

while robot.step(timeStep) != -1:
    reconhece_esquerda()
    reconhece_direita()