from controller import Robot
from time import sleep
from math import sqrt
import struct
import cv2
import numpy as np

cor_buraco = b'...\xff'
PI = 3.141592653589
timeStep = 32
maxVelocity = 6.28
encoder_inicial = 0.0
tem_buraco = False
andou = 0.0
rad = 4.29552
mensagem_enviada = False
posicao_anterior = (0,0,0)
posicaoX_anterior = 0
posicaoY_anterior = 0
tamanho_mapa = 501
coordenada_centro_mapa = tamanho_mapa//2
n = 1
k = 2
direcao = ""



robot = Robot()
motorEsquerdo = robot.getDevice('left motor')
motorDireito = robot.getDevice('right motor')
encoders = motorEsquerdo.getPositionSensor()

# cameraF
cameraF = robot.getDevice("cameraF")
cameraF.enable(timeStep)

'''cameraF = robot.getDevice('cameraD')
cameraF.enable(timeStep)    

cameraE = robot.getDevice('cameraE')
cameraE.enable(timeStep)
'''
sensor_de_cor = robot.getDevice("colour_sensor")
sensor_de_cor.enable(timeStep)

emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
receiver.enable(timeStep)

gps = robot.getDevice("gps")
gps.enable(timeStep)
position = gps.getValues()
victimTypeH = bytes('H', "utf-8") # The victim type being sent is the letter 'H' for harmed victim

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
encoders.enable(timeStep)

sleep(0.2)

victimType = bytes('H', "utf-8") # The victim type being sent is the letter 'H' for harmed victim

#Definir o mapa (Nesse Caso eu fiz um mapa 201x201, o centro é o mapa[100][100])
mapa = np.array([[-1]*tamanho_mapa for _ in range(tamanho_mapa)])
mapa[coordenada_centro_mapa][coordenada_centro_mapa] = 2
coordenada_linha_atual = coordenada_centro_mapa
coordenada_coluna_atual = coordenada_centro_mapa
 

# Tem preto
def tem_preto():
    image = cameraF.getImage()
    image = np.frombuffer(image, np.uint8).reshape((cameraF.getHeight(), cameraF.getWidth(), 4))
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
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    #print(np.any(result))
    return np.any(result)

#tem amarelo trava o programa
def tem_amarelo():
    image = cameraF.getImage()
    image = np.frombuffer(image, np.uint8).reshape((cameraF.getHeight(), cameraF.getWidth(), 4))  
    # Convert the image to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper threshold values for yellow in HSV
    lower_yellow = np.array([20, 100, 100], dtype=np.uint8)
    upper_yellow = np.array([30, 255, 255], dtype=np.uint8)

    # Create a mask that identifies yellow pixels
    mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

    # Bitwise-AND mask and original image
    result = cv2.bitwise_and(image, image, mask=mask)

    # Display the result
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return np.any(result)

# tem vemrelho trava o programa
def tem_vermelho():
    image = cameraF.getImage()
    image = np.frombuffer(image, np.uint8).reshape((cameraF.getHeight(), cameraF.getWidth(), 4))
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

    # Display the result
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return np.any(result)


def codificar_tipo(tipo):
    x, y = valores_gps()
    victimType = bytes(tipo, "utf-8")
    message = struct.pack("i i c", x, y, victimType) # Pack the message.
    delay(1300) # Delay for 1.3 seconds
    emitter.send(message) # Send out the message
    print("enviou mensagem")


def delay(ms):
    initTime = robot.getTime()      # Store starting time (in seconds)
    while robot.step(timeStep) != -1:
        if (robot.getTime() - initTime) * 1000.0 > ms: # If time elapsed (converted into ms) is greater than value passed in
            break

def valores_gps():
    position = gps.getValues()
    x = int(position[0] * 100) # Get the xy coordinates, multiplying by 100 to convert from meters to cm 
    y = int(position[2] * 100) # We will use these coordinates as an estimate for the victim's position
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
        image = cameraF.getImage()
        image = np.frombuffer(image, np.uint8).reshape((cameraF.getHeight(), cameraF.getWidth(), 4))
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper threshold values for black in HSV
        lower_black = np.array([0, 0, 0], dtype=np.uint8)
        upper_black = np.array([180, 255, 185], dtype=np.uint8)
        mask = cv2.inRange(hsv_image, lower_black, upper_black)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE,
                                    cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,
                                    0.002 * cv2.arcLength(cnt, True), True)
            #print(len(approx))
            if len(approx) >= 8 and len(approx) <= 26: #and not tem_amarelo() and not tem_vermelho():
                delay(1300)
                codificar_tipo('H')
                print('H')  
            elif len(approx) >= 27 and len(approx) <= 33: #and not tem_amarelo() and not tem_vermelho():
                print('U')
                delay(1300)
                codificar_tipo('U')
            elif len(approx) >= 60 and len(approx) <= 82: #and not tem_amarelo() and not tem_vermelho():
                print('S')
                delay(1300)
                codificar_tipo('S')
            elif tem_amarelo(): #and len(approx) != 4:
                print('O')
                delay(1300)
                codificar_tipo('O')
            elif tem_vermelho() and len(approx) !=4:
                print('F')
                delay(1300)
                codificar_tipo('F')
            elif tem_preto() and len(approx) != 4:
                print('C')
                delay(1300)
                codificar_tipo('C')

        # Bitwise-AND mask and original image
        #cv2.imshow("Mask", mask)
        cv2.waitKey(1)


def tem_buraco():
    cor = sensor_de_cor.getImage()
    if cor == cor_buraco:
        return True
    return False


def objeto_proximo(posicao):
    # Retorna True se o objeto está próximo, senão, retorna False
    return sqrt(posicao[0] ** 2 + posicao[1] ** 2) < 0.1


def parede_direita(sensoresDireita):
    # Retorna True se há parede à direita, senão, retorna False
    if sensoresDireita[1].getValue() < 0.14:
        return True
    return False


def parede_esquerda(sensoresEsquerda):
    # Retorna True se há parede à esquerda, senão, retorna False
    if sensoresEsquerda[1].getValue() < 0.14:
        return True
    return False


def parede_frente(sensoresFrente):
    # Retorna True se há parede à frente, senão, retorna False
    if sensoresFrente[0].getValue() < 0.155 or sensoresFrente[1].getValue() < 0.155:
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


def mover_para_frente(dist):
    while robot.step(timeStep) != -1:
        motorEsquerdo.setVelocity(maxVelocity)
        motorDireito.setVelocity(maxVelocity)
        if tem_buraco():
            andou = encoders.getValue() - encoder_inicial
            mover_para_tras(andou - 0.35)
            encoder_antes()
            virar_180()
            break
        if encoders.getValue() - encoder_inicial > dist:
            parar()
            break


def parar():
    motorEsquerdo.setVelocity(0.0)
    motorDireito.setVelocity(0.0)


def virar_180():
    # Vira 180 graus
    while robot.step(timeStep) != -1:
        motorEsquerdo.setVelocity(maxVelocity)
        motorDireito.setVelocity(-maxVelocity)
        if encoders.getValue() - encoder_inicial > rad:
            parar()
            break


# Função para virar à esquerda por 90 graus
def virar_esquerda():
    while robot.step(timeStep) != -1:
        motorEsquerdo.setVelocity(-maxVelocity)
        motorDireito.setVelocity(maxVelocity)
        if encoders.getValue() - encoder_inicial < -rad / 2:
            parar()
            break


# Função para virar à direita por 90 graus
def virar_direita():
    while robot.step(timeStep) != -1:
        motorEsquerdo.setVelocity(maxVelocity)
        motorDireito.setVelocity(-maxVelocity)
        if encoders.getValue() - encoder_inicial > rad / 2:
            parar()
            break


direcao = "direita"
def mudar_direcao():
    global direcao
    if round(deltaX, 2) == 0.12 or round(deltaX, 2) == 0.11 or round(deltaX, 2) == 0.13:
        direcao = "direita"
        print(direcao)
    elif round(deltaX, 2) == -0.12 or round(deltaX, 2) == -0.11 or round(deltaX, 2) == -0.13:
        direcao = "esquerda"
        print(direcao)
    if round(deltaY, 2) == 0.12 or round(deltaY, 2) == 0.11 or round(deltaY, 2) == 0.13 :
        direcao = "baixo"
        print(direcao)
    elif round(deltaY,2) == -0.12 or round(deltaY, 2) == -0.11 or round(deltaY, 2) == -0.13:
        direcao = "cima"
        print(direcao)
         

def foi_visitado(dir,sentido):
    if dir == "direita":
        if sentido == "direita" and mapa[coordenada_linha_atual + k][coordenada_coluna_atual] == 2:
            return 1
        if sentido == "esquerda" and mapa[coordenada_linha_atual - k][coordenada_coluna_atual] == 2:
            return 1
        if sentido == "atras" and mapa[coordenada_linha_atual][coordenada_coluna_atual - k] == 2:
            return 1
        if sentido == "frente" and mapa[coordenada_linha_atual][coordenada_coluna_atual + k] == 2:
            return 1

    if dir == "esquerda":
        if sentido == "direita" and mapa[coordenada_linha_atual - k][coordenada_coluna_atual] == 2:
            return 1
        if sentido == "esquerda" and mapa[coordenada_linha_atual + k][coordenada_coluna_atual] == 2:
            return 1
        if sentido == "atras" and mapa[coordenada_linha_atual][coordenada_coluna_atual + k] == 2:
            return 1
        if sentido == "frente" and mapa[coordenada_linha_atual][coordenada_coluna_atual - k] == 2:
            return 1
        
    if dir == "baixo" :
        if sentido == "direita" and mapa[coordenada_linha_atual][coordenada_coluna_atual - k] == 2:
            return 1
        if sentido == "esquerda" and mapa[coordenada_linha_atual][coordenada_coluna_atual + k] == 2:
            return 1
        if sentido == "atras" and mapa[coordenada_linha_atual - k][coordenada_coluna_atual] == 2:
            return 1
        if sentido == "frente" and mapa[coordenada_linha_atual + k][coordenada_coluna_atual] == 2:
            return 1
    
    if dir == "cima" :
        if sentido == "direita" and mapa[coordenada_linha_atual][coordenada_coluna_atual + k] == 2:
            return 1
        if sentido == "esquerda" and mapa[coordenada_linha_atual][coordenada_coluna_atual - k] == 2:
            return 1
        if sentido == "atras" and mapa[coordenada_linha_atual + k][coordenada_coluna_atual] == 2:
            return 1
        if sentido == "frente" and mapa[coordenada_linha_atual - k][coordenada_coluna_atual] == 2:
            return 1
    else :
        print(" A dir é {}".format(dir))
        print(" O sentido é {}".format(sentido))
        return 0


# Função para seguir a parede
def seguir_parede():
    global direcao
    if not parede_esquerda(sensoresEsquerda) and not foi_visitado(direcao, "esquerda"):
        # Se não há parede à esquerda, vire à esquerda e mova-se para frente
        #print('esquerda livre,')
        encoder_antes()
        virar_esquerda()
        encoder_antes()
        mover_para_frente(6)
        delay(5)
        print("estou indo para esquerda")

    elif parede_esquerda(sensoresEsquerda) and not parede_frente(sensoresFrente) and not foi_visitado(direcao, "frente"):
        # Se há parede à esquerda, mas não à frente, mova-se para frente
        #print('esquerda ocupada, frente livre')
        encoder_antes()
        mover_para_frente(6)
        delay(5)
        print("estou indo para frente")

    elif parede_esquerda(sensoresEsquerda) and parede_frente(sensoresFrente) and not parede_direita(sensoresDireita) and not foi_visitado(direcao, "direita"):
        # Se há parede à esquerda e à frente, vire à direita e mova-se para frente
        #print('esquerda e frente ocupadas')
        encoder_antes()
        virar_direita()
        encoder_antes()
        mover_para_frente(6)
        delay(5)
        print("estou indo para direita")

    elif parede_esquerda(sensoresEsquerda) and parede_frente(sensoresFrente) and parede_direita(sensoresDireita) and not foi_visitado(direcao, "atras"):
        encoder_antes()
        virar_180()
        delay(5)

    else:
        # BFS
        print("BFS!!")
        ''' 
        for li in mapa:
            print(li)
        '''
        cl = coordenada_linha_atual
        cc = coordenada_coluna_atual
        atual = (cl, cc)
        fila = []
        marc = []
        dist = {}
        path = {}
        fila.append((cl,cc))
        marc.append((cl,cc))
        dist[atual] = 0
        #print("listas_vistos[-1] é {}".format(listas_vistos[-2]))
        found_path = False
        while len(fila) != 0 and not found_path:
            atual = fila.pop(0)
            cl = atual[0]
            cc = atual[1]
            if(cl < 0 or cc < 0):
                continue
            try:
                hngrgigrwuirh = mapa[cl][cc]
            except:
                continue
            #print("estou no ", atual)
            if atual == listas_vistos[-2]:
                #print("Achei!!!!!!!!!")
                caminho = [listas_vistos[-2]]
                found_path = True
                aux = listas_vistos[-2]
                while aux != (coordenada_linha_atual, coordenada_coluna_atual):
                    caminho.append(path[aux])
                    aux = path[aux]
                caminho.reverse()
                print("O caminho é {}".format(caminho))
                continue
                
            #print(mapa[cl][cc + n])
            
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
            
        #print(dist)
        #print(atual)
        #Andar percorrendo o caminho
        for i in range(len(caminho) - 1):
            if caminho[i+1][0] - caminho[i][0] == 2:
                #DeltaY positivo, andar para baixo
                print("DeltaY : {}".format(caminho[i+1][0] - caminho[i][0]))
                if direcao == "direita":
                    encoder_antes()
                    virar_direita()
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()                    
                if direcao == "esquerda":
                    encoder_antes()
                    virar_esquerda()
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()                       
                if direcao == "cima":
                    encoder_antes()
                    virar_180()
                    encoder_antes()
                    mover_para_frente(6)
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

            elif caminho[i+1][0] - caminho[i][0] == -2:
                #DeltaY negativo, andar para cima
                print("DeltaY : {}".format(caminho[i+1][0] - caminho[i][0]))
                if direcao == "direita":
                    encoder_antes()
                    virar_esquerda()
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()   
                if direcao == "esquerda":
                    encoder_antes()
                    virar_direita()
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()   
                if direcao == "cima":
                    encoder_antes()
                    mover_para_frente(6)
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

            elif caminho[i+1][1] - caminho[i][1] == 2:
                #DeltaX positivo
                print("DeltaX : {}".format(caminho[i+1][1] - caminho[i][1]))
                if direcao == "direita":
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()   

                if direcao == "esquerda":
                    encoder_antes()
                    virar_180()
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()   
                if direcao == "cima":
                    encoder_antes()
                    virar_direita()
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()   
                if direcao == "baixo":
                    encoder_antes()
                    virar_esquerda()
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()   
                direcao = "direita"

            elif caminho[i+1][1] - caminho[i][1] == -2:
                #DeltaX negativo
                print("DeltaX : {}".format(caminho[i+1][1] - caminho[i][1]))
                if direcao == "direita":
                    encoder_antes()
                    virar_180()
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()   
                if direcao == "esquerda":
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()   
                if direcao == "cima":
                    encoder_antes()
                    virar_esquerda()
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()   
                if direcao == "baixo":
                    encoder_antes()
                    virar_direita()
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()   
                direcao = "esquerda"
                
        if caminho[-1][0] - caminho[-2][0] == 2:
                #DeltaY positivo, andar para baixo
                print("DeltaY : {}".format(caminho[i+1][0] - caminho[i][0]))
                if direcao == "direita":
                    encoder_antes()
                    virar_direita()
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()                    


                if direcao == "esquerda":
                    encoder_antes()
                    virar_esquerda()
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()                       
                if direcao == "cima":
                    encoder_antes()
                    virar_180()
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()   
                if direcao == "baixo":
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()  
                     
        elif caminho[-1][0] - caminho[-2][0] == -2:
                #DeltaY negativo, andar para cima
                print("DeltaY : {}".format(caminho[i+1][0] - caminho[i][0]))
                if direcao == "direita":
                    encoder_antes()
                    virar_esquerda()
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()   
                if direcao == "esquerda":
                    encoder_antes()
                    virar_direita()
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()   
                if direcao == "cima":
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()   
                if direcao == "baixo":
                    encoder_antes()
                    virar_180()
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()   
        elif caminho[-1][1] - caminho[-2][1] == 2:
                #DeltaX positivo
                print("DeltaX : {}".format(caminho[i+1][1] - caminho[i][1]))
                if direcao == "direita":
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()   

                if direcao == "esquerda":
                    encoder_antes()
                    virar_180()
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()   
                if direcao == "cima":
                    encoder_antes()
                    virar_direita()
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()   
                if direcao == "baixo":
                    encoder_antes()
                    virar_esquerda()
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()   
        elif caminho[-1][1] - caminho[-2][1] == -2:
                #DeltaX negativo
                print("DeltaX : {}".format(caminho[i+1][1] - caminho[i][1]))
                if direcao == "direita":
                    encoder_antes()
                    virar_180()
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()   
                if direcao == "esquerda":
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()   
                if direcao == "cima":
                    encoder_antes()
                    virar_esquerda()
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()   
                if direcao == "baixo":
                    encoder_antes()
                    virar_direita()
                    encoder_antes()
                    mover_para_frente(6)
                    delay(5)
                    if parede_frente(sensoresFrente):
                        ajustar_distancia()   



listas_vistos = []
lista_tiles_marcados = [(coordenada_centro_mapa, coordenada_centro_mapa)]

# Loop principal
mover_para_frente(0.01)
sleep(0.1)  # gambiarra essencial para o robô não bugar :) NÃO TIRAR

while robot.step(timeStep) != -1:
    
    posicao_atual = gps.getValues()
    posicaoX_atual = posicao_atual[0]
    posicaoY_atual = posicao_atual[2]
    
    deltaX = posicaoX_atual - posicaoX_anterior
    deltaY = posicaoY_atual - posicaoY_anterior

    '''
    Legenda
    0 -> Sem parede
    1 -> Parde
    2 -> Tile Visitado
    3 -> Tile Visto
    -1 -> Tile Indefinido
    '''

    #Início
    if round(deltaX, 2) == round(posicaoX_atual, 2) and round(deltaY, 2) == round(posicaoY_atual, 2):
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
        
        mapa[coordenada_centro_mapa][coordenada_centro_mapa-n] = 1 #Assumindo que tenha uma parede atrás

    #DeltaX positivo 
    if round(deltaX, 2) == 0.12 or round(deltaX, 2) == 0.11 or round(deltaX, 2) == 0.13:
        print("O Delta X é : {}".format(round(deltaX, 2)))
        coordenada_coluna_atual = coordenada_coluna_atual + k
        coordenada_linha_atual = coordenada_linha_atual

        #Processo de Remover Tiles Visitados das lista de tiles vistos e marcando eles (adicionando na lista de tiles_marcados)
        if (coordenada_linha_atual, coordenada_coluna_atual) in listas_vistos:
            listas_vistos.pop(listas_vistos.index((coordenada_linha_atual, coordenada_coluna_atual)))
            lista_tiles_marcados.append((coordenada_linha_atual, coordenada_coluna_atual))
            

        # Trocar o estado do tile de visto para visitado
        if mapa[coordenada_linha_atual][coordenada_coluna_atual] == 3:
            mapa[coordenada_linha_atual][coordenada_coluna_atual] = 2
            

        #Frente
        if not parede_frente(sensoresFrente) and mapa[coordenada_linha_atual][coordenada_coluna_atual + k] != 2 :
            mapa[coordenada_linha_atual][coordenada_coluna_atual + n] = 0
            mapa[coordenada_linha_atual][coordenada_coluna_atual + k] = 3

            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (coordenada_linha_atual, coordenada_coluna_atual + k) in lista_tiles_marcados:
                listas_vistos.append((coordenada_linha_atual, coordenada_coluna_atual + k))
        elif parede_frente(sensoresFrente):
            mapa[coordenada_linha_atual][coordenada_coluna_atual + n] = 1

        #Esquerda
        if not parede_esquerda(sensoresEsquerda) and mapa[coordenada_linha_atual - k][coordenada_coluna_atual] != 2 : 
            mapa[coordenada_linha_atual - n][coordenada_coluna_atual] = 0
            mapa[coordenada_linha_atual - k][coordenada_coluna_atual] = 3
            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (coordenada_linha_atual - k, coordenada_coluna_atual) in lista_tiles_marcados:
                listas_vistos.append((coordenada_linha_atual - k, coordenada_coluna_atual))

        elif parede_esquerda(sensoresEsquerda):
            mapa[coordenada_linha_atual - n][coordenada_coluna_atual] = 1

        # Direita
        if not parede_direita(sensoresDireita) and mapa[coordenada_linha_atual + k][coordenada_coluna_atual] != 2 :
            mapa[coordenada_linha_atual + n][coordenada_coluna_atual] = 0
            mapa[coordenada_linha_atual + k][coordenada_coluna_atual] = 3
            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (coordenada_linha_atual + k, coordenada_coluna_atual) in lista_tiles_marcados:
                listas_vistos.append((coordenada_linha_atual + k, coordenada_coluna_atual))
        elif parede_direita(sensoresDireita):
            mapa[coordenada_linha_atual + n][coordenada_coluna_atual] = 1

    #DeltaX negativo 
    elif round(deltaX, 2) == -0.12 or round(deltaX, 2) == -0.11 or round(deltaX, 2) == -0.13:
        print("O Delta X é : {}".format(round(deltaX,2)))
        coordenada_coluna_atual = coordenada_coluna_atual - k
        coordenada_linha_atual = coordenada_linha_atual

        #Processo de Remover Tiles Visitados das lista de tiles vistos e marcando eles (adicionando na lista de tiles_marcados)
        if (coordenada_linha_atual, coordenada_coluna_atual) in listas_vistos:
            listas_vistos.pop(listas_vistos.index((coordenada_linha_atual, coordenada_coluna_atual)))
            lista_tiles_marcados.append((coordenada_linha_atual, coordenada_coluna_atual))

        #Trocar o estado do tile de visto para visitado
        if mapa[coordenada_linha_atual][coordenada_coluna_atual] == 3:
            mapa[coordenada_linha_atual][coordenada_coluna_atual] = 2

        #Frente
        if not parede_frente(sensoresFrente) and mapa[coordenada_linha_atual][coordenada_coluna_atual - k] != 2:
            mapa[coordenada_linha_atual][coordenada_coluna_atual - n] = 0
            mapa[coordenada_linha_atual][coordenada_coluna_atual - k] = 3
            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (coordenada_linha_atual, coordenada_coluna_atual - k) in lista_tiles_marcados:
                listas_vistos.append((coordenada_linha_atual, coordenada_coluna_atual - k))
        elif parede_frente(sensoresFrente):
            mapa[coordenada_linha_atual][coordenada_coluna_atual - n] = 1

        #Esquerda
        if not parede_esquerda(sensoresEsquerda) and mapa[coordenada_linha_atual + k][coordenada_coluna_atual] != 2:
            mapa[coordenada_linha_atual + n][coordenada_coluna_atual] = 0
            mapa[coordenada_linha_atual + k][coordenada_coluna_atual] = 3
            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (coordenada_linha_atual + k, coordenada_coluna_atual) in lista_tiles_marcados:
                listas_vistos.append((coordenada_linha_atual + k, coordenada_coluna_atual))
        elif parede_esquerda(sensoresEsquerda):
            mapa[coordenada_linha_atual + n][coordenada_coluna_atual] = 1

        #Direita
        if not parede_direita(sensoresDireita) and mapa[coordenada_linha_atual - k][coordenada_coluna_atual] != 2:
            mapa[coordenada_linha_atual - n][coordenada_coluna_atual] = 0
            mapa[coordenada_linha_atual - k][coordenada_coluna_atual] = 3
            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (coordenada_linha_atual - k, coordenada_coluna_atual) in lista_tiles_marcados:
                listas_vistos.append((coordenada_linha_atual - k, coordenada_coluna_atual))
        elif parede_direita(sensoresDireita):
            mapa[coordenada_linha_atual - n][coordenada_coluna_atual] = 1

    #DeltaY positivo 
    if round(deltaY, 2) == 0.12 or round(deltaY, 2) == 0.11 or round(deltaY, 2) == 0.13 :
        print("O Delta Y é : {}".format(round(deltaY,2)))
        coordenada_linha_atual = coordenada_linha_atual + k
        coordenada_coluna_atual = coordenada_coluna_atual
        print((coordenada_linha_atual, coordenada_coluna_atual), "; ")
        #Processo de Remover Tiles Visitados das lista de tiles vistos e marcando eles (adicionando na lista de tiles_marcados)
        if (coordenada_linha_atual, coordenada_coluna_atual) in listas_vistos:
            listas_vistos.pop(listas_vistos.index((coordenada_linha_atual, coordenada_coluna_atual)))
            lista_tiles_marcados.append((coordenada_linha_atual, coordenada_coluna_atual))

        #Trocar o tile visto por visitado
        if mapa[coordenada_linha_atual][coordenada_coluna_atual] == 3:
            mapa[coordenada_linha_atual][coordenada_coluna_atual] = 2

        #Frente
        if not parede_frente(sensoresFrente) and mapa[coordenada_linha_atual + k ][coordenada_coluna_atual] != 2:
            mapa[coordenada_linha_atual + n ][coordenada_coluna_atual] = 0
            mapa[coordenada_linha_atual + k ][coordenada_coluna_atual] = 3
            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (coordenada_linha_atual + k, coordenada_coluna_atual) in lista_tiles_marcados:
                listas_vistos.append((coordenada_linha_atual + k, coordenada_coluna_atual))

        elif parede_frente(sensoresFrente):
            mapa[coordenada_linha_atual + n ][coordenada_coluna_atual] = 1

        #Direita
        if not parede_direita(sensoresDireita) and mapa[coordenada_linha_atual][coordenada_coluna_atual - k] != 2:
            mapa[coordenada_linha_atual][coordenada_coluna_atual - n] = 0
            mapa[coordenada_linha_atual][coordenada_coluna_atual - k] = 3
            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (coordenada_linha_atual, coordenada_coluna_atual - k) in lista_tiles_marcados:
                listas_vistos.append((coordenada_linha_atual, coordenada_coluna_atual - k))
        elif parede_direita(sensoresDireita):
            mapa[coordenada_linha_atual][coordenada_coluna_atual - n] = 1

        #Esquerda
        if not parede_esquerda(sensoresEsquerda) and mapa[coordenada_linha_atual][coordenada_coluna_atual + k] != 2:
            mapa[coordenada_linha_atual][coordenada_coluna_atual + n] = 0
            mapa[coordenada_linha_atual][coordenada_coluna_atual + k] = 3
            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (coordenada_linha_atual, coordenada_coluna_atual + k) in lista_tiles_marcados:
                listas_vistos.append((coordenada_linha_atual, coordenada_coluna_atual + k))
        elif parede_esquerda(sensoresEsquerda):
            mapa[coordenada_linha_atual][coordenada_coluna_atual + n] =1

    #DeltaY negativo 
    elif round(deltaY,2) == -0.12 or round(deltaY, 2) == -0.11 or round(deltaY, 2) == -0.13:
        print("O Delta Y é : {}".format(round(deltaY,2)))
        coordenada_linha_atual = coordenada_linha_atual - k
        coordenada_coluna_atual = coordenada_coluna_atual

        #Processo de Remover Tiles Visitados das lista de tiles vistos e marcando eles (adicionando na lista de tiles_marcados)
        if (coordenada_linha_atual, coordenada_coluna_atual) in listas_vistos:
            listas_vistos.pop(listas_vistos.index((coordenada_linha_atual, coordenada_coluna_atual)))
            lista_tiles_marcados.append((coordenada_linha_atual, coordenada_coluna_atual))

        #Trocando o tile visto por visitado
        if mapa[coordenada_linha_atual][coordenada_coluna_atual] == 3:
            mapa[coordenada_linha_atual][coordenada_coluna_atual] = 2

        #Frente
        if not parede_frente(sensoresFrente) and mapa[coordenada_linha_atual - k][coordenada_coluna_atual] != 2:
            mapa[coordenada_linha_atual - n][coordenada_coluna_atual] = 0
            mapa[coordenada_linha_atual - k][coordenada_coluna_atual] = 3
            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (coordenada_linha_atual - k, coordenada_coluna_atual) in lista_tiles_marcados:
                listas_vistos.append((coordenada_linha_atual - k, coordenada_coluna_atual))
        elif parede_frente(sensoresFrente):
            mapa[coordenada_linha_atual - n][coordenada_coluna_atual] = 1
        
        #Direita
        if not parede_direita(sensoresDireita) and mapa[coordenada_linha_atual][coordenada_coluna_atual + k] != 2:
            mapa[coordenada_linha_atual][coordenada_coluna_atual + n] = 0
            mapa[coordenada_linha_atual][coordenada_coluna_atual + k] = 3
            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (coordenada_linha_atual, coordenada_coluna_atual + k) in lista_tiles_marcados:
                listas_vistos.append((coordenada_linha_atual, coordenada_coluna_atual + k))
        elif parede_direita(sensoresDireita):
            mapa[coordenada_linha_atual][coordenada_coluna_atual + n] = 1

        #Esquerda
        if not parede_esquerda(sensoresEsquerda) and mapa[coordenada_linha_atual][coordenada_coluna_atual - k] != 2:
            mapa[coordenada_linha_atual][coordenada_coluna_atual - n] = 0
            mapa[coordenada_linha_atual][coordenada_coluna_atual - k] = 3
            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (coordenada_linha_atual, coordenada_coluna_atual - k) in lista_tiles_marcados:
                listas_vistos.append((coordenada_linha_atual, coordenada_coluna_atual - k))
        elif parede_esquerda(sensoresEsquerda):
            mapa[coordenada_linha_atual][coordenada_coluna_atual - n] = 1

    '''
    #Robo nao se mexe no eixo Y
    else :
        print("O robo nao está andando no eixo Y. O Delta Y é : {}".format(round(deltaY,2)))
 
    '''
    
    posicao_anterior = posicao_atual
    posicaoX_anterior = posicaoX_atual
    posicaoY_anterior = posicaoY_atual
    mudar_direcao()

    seguir_parede()
    delay(20)
    reconhecer_vitima()
    delay(10)
    ajustar_distancia()
    delay(10)


    print("A lista de tiles vistos é : {}".format(listas_vistos))
    print("A lista de tiles marcados é : {}".format(lista_tiles_marcados))    

    
    