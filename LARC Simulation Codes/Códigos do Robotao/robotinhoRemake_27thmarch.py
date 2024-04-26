# import of libraries
from controller import Robot
from time import sleep
import math
import struct
import cv2
import numpy as np
import easyocr as ocr

#Constants
HOLE_COLOR = b'...\xff'
PI = math.pi
TIMESTEP = 32
MAXVELOCITY = 6.28
INITIAL_ENCODER = 0.0
RAD = 4.29552
MAP_SIZE = 501
MAPCENTER_COORDINATE = MAP_SIZE//2
ONE_UNIT = 1
TWO_UNITS = 2

#Global Variables
hasHole = False
walked = 0.0
sentMessage = False
previousPosition = (0,0,0)
previousPositionX = 0
previousPositionY = 0
currentPosition = 0
currentPositionX = 0
currentPostionY = 0
orientation = "right"
previousRed = 0.0
previousBlack = 0.0
victimTypeH = bytes('H', "utf-8")
seenTileList = []
checkedTileList = [(MAPCENTER_COORDINATE, MAPCENTER_COORDINATE)]
deltaX = currentPositionX - previousPositionX
deltaY = currentPostionY - previousPositionY

#Sensors and Motors' Definition 
robot = Robot()
leftMotor = robot.getDevice('left motor')
rightMotor = robot.getDevice('right motor')
encoders = leftMotor.getPositionSensor()
leftMotor.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
encoders.enable(TIMESTEP)

cameraE = robot.getDevice("cameraE")
cameraE.enable(TIMESTEP)

colorSensor = robot.getDevice("colour_sensor")
colorSensor.enable(TIMESTEP)

emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
receiver.enable(TIMESTEP)

gps = robot.getDevice("gps")
gps.enable(TIMESTEP)
robotGPSPosition = gps.getValues()

frontSensors = [robot.getDevice("ps_frente"), robot.getDevice("ps_tras")]
leftSensors = [robot.getDevice("ps_esquerda"), robot.getDevice("ps_diagonal_esquerda")]
rightSensors = [robot.getDevice("ps_direita"), robot.getDevice("ps_diagonal_direita")]
for sensor in frontSensors + leftSensors + rightSensors: 
    sensor.enable(TIMESTEP)

# Array Definition
map = np.array([[-1]*MAP_SIZE for _ in range(MAP_SIZE)])
map[MAPCENTER_COORDINATE][MAPCENTER_COORDINATE] = 2
currentLineCoordinate = MAPCENTER_COORDINATE
currentColumnCoordinate = MAPCENTER_COORDINATE


##Victim Detection's Functions
def adjustBlack():
    global previousBlack
    black = blackQuantity()
    red = redQuantity()
    if black > 20 and black > previousBlack and red == 0:
        print('boraaa,', black)
        while True:
            print(f'ta aumentando mlk {black}, {previousBlack}')
            previousBlack = black
            previousEncoder()
            goAhead(0.1)
            black = blackQuantity()  
            if previousBlack > black:
                victimDetection()
                previousBlack = 0
                break
        stop()
        print('parou')
def adjustRed():
    global previousRed
    black = blackQuantity()
    red = redQuantity()
    if red > 20 and red > previousRed and black == 0:
        print(f'sisior,{red}')
        while True:
            print(f'ta aumentando mlk {red}, {previousRed}')
            previousRed = red
            previousEncoder()
            goAhead(0.1)
            red = redQuantity()
            if previousRed > red:
                delay(1000)
                victimDetection()
                previousRed = 0
                break
        stop()
        print('parou')
def blackQuantity():
    image = cameraE.getImage()
    image = np.frombuffer(image, np.uint8).reshape((cameraE.getHeight(), cameraE.getWidth(), 4))
    # Convert the image to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Count the number of non-black pixels
    non_black_pixels = cv2.countNonZero(gray_image)

    # Subtract the number of non-black pixels from the total number of pixels
    total_pixels = gray_image.size
    black_pixels = total_pixels - non_black_pixels

    return black_pixels
def redQuantity():
    image = cameraE.getImage()
    image = np.frombuffer(image, np.uint8).reshape((cameraE.getHeight(), cameraE.getWidth(), 4))
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
def hasBlack():
    image = cameraE.getImage()
    image = np.frombuffer(image, np.uint8).reshape((cameraE.getHeight(), cameraE.getWidth(), 4))
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
def hasYellow():
    image = cameraE.getImage()
    image = np.frombuffer(image, np.uint8).reshape((cameraE.getHeight(), cameraE.getWidth(), 4))  
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
def hasRed():
    image = cameraE.getImage()
    image = np.frombuffer(image, np.uint8).reshape((cameraE.getHeight(), cameraE.getWidth(), 4))
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
def codifyType(type):
    x, y = gpsValue()
    victimType = bytes(type, "utf-8")
    message = struct.pack("i i c", x, y, victimType) # Pack the message.
    delay(1300) # Delay for 1.3 seconds
    emitter.send(message) # Send out the message
    print("enviou mensagem")
def gpsValue():
    robotGPSPosition = gps.getValues()
    x = int(robotGPSPosition[0] * 100) # Get the xy coordinates, multiplying by 100 to convert from meters to cm 
    y = int(robotGPSPosition[2] * 100) # We will use these coordinates as an estimate for the victim's position
    return x, y
def victimDetection():
    black = blackQuantity()
    vermelho = redQuantity()
    if black > 20 and leftWall(leftSensors) or vermelho > 20:
        if hasBlack(): adjustBlack()
        elif hasRed(): adjustRed()
        print('reconhecendo...')
        possivel_H = ['H', 'HI', 'IH', 'IHI', 'F', 'FI', 'IF', 'IFI', 'A', 'AI', 'IA', 'IAI', 'HH']
        possivel_U = ['U', 'UI', 'IU', 'IUI', 'UU', '(U)', 'U)', '(U']
        possivel_S = ['S', 'SI', 'IS', 'ISI', 'SIS', 'SS', '5', 'I5', '5I', 'I5I', '75']
        adjustDistance()
        image = cameraE.getImage()
        image = np.frombuffer(image, np.uint8).reshape((cameraE.getHeight(), cameraE.getWidth(), 4))
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #padrao opencv	
        rgb_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2RGB)
        # Define the lower and upper threshold values for black in HSV

        reader = ocr.Reader(['en'])
        result = reader.readtext(rgb_image)
        for item in result:
            text = item[1].upper()
            print(text)
            if text in possivel_H:
                print('H')
                codifyType('H')
                return 0
            elif text in possivel_U:
                print('U')
                codifyType('U')
                return 0
            elif text in possivel_S:
                print('S')
                codifyType('S')
                return 0
        if hasYellow():
            print('O')
            codifyType('O')
        elif hasRed():
            print('F')
            codifyType('F')
        elif hasBlack():
            print('C')
            codifyType('C')

# Wall's Functions
def rightWall(rightSensors):
    # Retorna True se há parede à direita, senão, retorna False
    if rightSensors[0].getValue() < 0.07:
        return True
    return False
def leftWall(leftSensors):
    # Retorna True se há parede à esquerda, senão, retorna False
    if leftSensors[0].getValue() < 0.083:
        return True
    return False
def frontWall(frontSensors):
    # Retorna True se há parede à frente, senão, retorna False
    if frontSensors[0].getValue() < 0.16:
        return True
    return False
def backWall(frontSensors):
    # Retorna True se há parede à tras, senão, retorna False
    if frontSensors[1].getValue() < 0.16:
        return True
    return False

#Movimentation Functions
def previousEncoder():
    # Retorna o valor do encoder antes de mover
    global INITIAL_ENCODER
    INITIAL_ENCODER = float(encoders.getValue())
    return INITIAL_ENCODER
def stop():
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)
def turn180():
    # Vira 180 graus
    while robot.step(TIMESTEP) != -1:
        leftMotor.setVelocity(MAXVELOCITY)
        rightMotor.setVelocity(-MAXVELOCITY)
        if encoders.getValue() - INITIAL_ENCODER > RAD:
            stop()
            break
def turnLeft():
    while robot.step(TIMESTEP) != -1:
        leftMotor.setVelocity(-MAXVELOCITY)
        rightMotor.setVelocity(MAXVELOCITY)
        if hasBlack(): print('preto')
        elif hasRed(): print('vermelho')
        if encoders.getValue() - INITIAL_ENCODER < -RAD / 2:
            stop()
            break
def turnRight():
    while robot.step(TIMESTEP) != -1:
        leftMotor.setVelocity(MAXVELOCITY)
        rightMotor.setVelocity(-MAXVELOCITY)
        if hasBlack(): print('preto')
        elif hasRed(): print('vermelho')
        if encoders.getValue() - INITIAL_ENCODER > RAD / 2:
            stop()
            break
def turn():
    #analisa o ambiente
    while robot.step(TIMESTEP) != -1:
        leftMotor.setVelocity(MAXVELOCITY)
        rightMotor.setVelocity(-MAXVELOCITY)
        if frontWall(frontSensors):
            break
        if encoders.getValue() - INITIAL_ENCODER > 2*RAD:
            stop()
            break
def goBack(dist):
    while robot.step(TIMESTEP) != -1:
        global walked
        leftMotor.setVelocity(-MAXVELOCITY)
        rightMotor.setVelocity(-MAXVELOCITY)
        if encoders.getValue() - INITIAL_ENCODER < -dist:
            stop()
            break
def goAhead(dist):
    while robot.step(TIMESTEP) != -1:
        global walked
        leftMotor.setVelocity(MAXVELOCITY)
        rightMotor.setVelocity(MAXVELOCITY)
        if hasHole():
            walked = encoders.getValue() - INITIAL_ENCODER
            goBack(walked - 0.35)
            previousEncoder()
            turn180()
            break
        if encoders.getValue() - INITIAL_ENCODER > dist:
            stop()
            break
def adjustDistance():
    # Ajusta a distância do robô para a parede
    if frontSensors[0].getValue() < 0.15:
        difference = 0.15 - frontSensors[0].getValue()
        previousEncoder()
        goBack(difference)
    elif frontSensors[0].getValue() < 0.20 and frontSensors[0].getValue() > 0.15:
        difference = 0.15 - frontSensors[0].getValue()
        previousEncoder()
        goAhead(difference)
def wallFollower():
    stop()
    mapping()
    victimDetection()
    if not leftWall(leftSensors) and not was_visited(orientation, "left"):
        # Se não há parede à esquerda, vire à esquerda e mova-se para frente
        print('esquerda livre,')
        previousEncoder()
        turnLeft()
        stop()
        victimDetection()            
        previousEncoder()
        goAhead(6)
    elif leftWall(leftSensors) and not frontWall(frontSensors) and not was_visited(orientation, "front"):
        # Se há parede à esquerda, mas não à frente, mova-se para frente
        print('esquerda ocupada, frente livre')
        stop()
        victimDetection()
        previousEncoder()
        goAhead(6)
        if not leftWall(leftSensors):
            wallFollower()
    elif leftWall(leftSensors) and frontWall(frontSensors) and not rightWall(rightSensors) and not was_visited(orientation, "right"):
        # Se há parede à esquerda e à frente, vire à direita e mova-se para frente
        print('esquerda e frente ocupadas')
        previousEncoder()
        turnRight()
        stop()
        victimDetection()
        previousEncoder()
        goAhead(6)
        if not leftWall(leftSensors):
            wallFollower()
    elif leftWall(leftSensors) and frontWall(frontSensors) and rightWall(rightSensors) and not was_visited(orientation, "behind"):
        previousEncoder()
        turn180()
        delay(5)
    else:
        # BFS
        stop()
        print("BFS!!")
    stop()
    victimDetection()
def PID():
    Kp = 38.5
    Ki= 0.4
    Kd = 1.0
    error = leftSensors[1].getValue() - 0.13
    previousError = 0
    errorSum = 0

    proporcional = Kp*error

    stop()
    victimDetection()
    if hasBlack():
        adjustBlack() #se tiver preto na esq, ajusta o centro e reconhece
        delay(200)
    elif hasRed():
        adjustRed()
        delay(200)
    if proporcional > 12.28:
        proporcional = 12.28

    if hasHole():
        walked = encoders.getValue() - INITIAL_ENCODER
        goBack(1.0)
        previousEncoder()
        turn()
        previousEncoder()
        turnRight()

    if error - previousError > 0.2:
        stop()
        print('desvio')
        delay(20)
        previousEncoder()
        goAhead(6.0)
        adjustDistance()
        victimDetection()
        print('frente')
        delay(20)
        wallFollower()
        delay(20)
    
    if error > 0:
        #subtrai do motor esq e mantem o direito
        leftMotor.setVelocity(MAXVELOCITY - proporcional)
        rightMotor.setVelocity(MAXVELOCITY)
        previousError = error
        errorSum += error
    else:
        #subtrai do motor direito e mantem o esquerdo
        leftMotor.setVelocity(MAXVELOCITY)
        rightMotor.setVelocity(MAXVELOCITY + proporcional)
        previousError = error
        errorSum += error
 
    if frontWall(frontSensors):
        adjustDistance()
        delay(20)
        adjustDistance()
        wallFollower()
        delay(20)
        print('parede a frente')

#Mapping's Functions
def changeOrientation():
    global orientation
    if round(deltaX, 2) == 0.12 or round(deltaX, 2) == 0.11 or round(deltaX, 2) == 0.13:
        orientation = "right"
        print(orientation)
    elif round(deltaX, 2) == -0.12 or round(deltaX, 2) == -0.11 or round(deltaX, 2) == -0.13:
        orientation = "left"
        print(orientation)
    if round(deltaY, 2) == 0.12 or round(deltaY, 2) == 0.11 or round(deltaY, 2) == 0.13 :
        orientation = "down"
        print(orientation)
    elif round(deltaY,2) == -0.12 or round(deltaY, 2) == -0.11 or round(deltaY, 2) == -0.13:
        orientation = "up"
        print(orientation)
def was_visited(dir,guidance):
    if dir == "right":
        if guidance == "right" and map[currentLineCoordinate + TWO_UNITS][currentColumnCoordinate] == 2:
            return 1
        if guidance == "left" and map[currentLineCoordinate - TWO_UNITS][currentColumnCoordinate] == 2:
            return 1
        if guidance == "behind" and map[currentLineCoordinate][currentColumnCoordinate - TWO_UNITS] == 2:
            return 1
        if guidance == "front" and map[currentLineCoordinate][currentColumnCoordinate + TWO_UNITS] == 2:
            return 1

    if dir == "left":
        if guidance == "right" and map[currentLineCoordinate - TWO_UNITS][currentColumnCoordinate] == 2:
            return 1
        if guidance == "left" and map[currentLineCoordinate + TWO_UNITS][currentColumnCoordinate] == 2:
            return 1
        if guidance == "behind" and map[currentLineCoordinate][currentColumnCoordinate + TWO_UNITS] == 2:
            return 1
        if guidance == "front" and map[currentLineCoordinate][currentColumnCoordinate - TWO_UNITS] == 2:
            return 1
        
    if dir == "down" :
        if guidance == "right" and map[currentLineCoordinate][currentColumnCoordinate - TWO_UNITS] == 2:
            return 1
        if guidance == "left" and map[currentLineCoordinate][currentColumnCoordinate + TWO_UNITS] == 2:
            return 1
        if guidance == "behind" and map[currentLineCoordinate - TWO_UNITS][currentColumnCoordinate] == 2:
            return 1
        if guidance == "front" and map[currentLineCoordinate + TWO_UNITS][currentColumnCoordinate] == 2:
            return 1
    
    if dir == "up" :
        if guidance == "right" and map[currentLineCoordinate][currentColumnCoordinate + TWO_UNITS] == 2:
            return 1
        if guidance == "left" and map[currentLineCoordinate][currentColumnCoordinate - TWO_UNITS] == 2:
            return 1
        if guidance == "behind" and map[currentLineCoordinate + TWO_UNITS][currentColumnCoordinate] == 2:
            return 1
        if guidance == "front" and map[currentLineCoordinate - TWO_UNITS][currentColumnCoordinate] == 2:
            return 1
    else :
        return 0
def mapping():
    global currentPosition, currentPositionX, currentPostionY
    global previousPosition, previousPositionX, previousPositionY
    global seenTileList, checkedTileList
    global deltaX, deltaY
    global currentColumnCoordinate, currentLineCoordinate

    currentPosition = gps.getValues()
    currentPositionX = currentPosition[0]
    currentPostionY = currentPosition[2]
    
    deltaX = currentPositionX - previousPositionX
    deltaY = currentPostionY - previousPositionY

    '''
    Legenda
    0 -> Sem parede
    1 -> Parde
    2 -> Tile Visitado
    3 -> Tile Visto
    -1 -> Tile Indefinido
    '''

    #Início
    if round(deltaX, 2) == round(currentPositionX, 2) and round(deltaY, 2) == round(currentPostionY, 2):
        print("Eu estou no início")
        if not frontWall(frontSensors):
            map[MAPCENTER_COORDINATE][MAPCENTER_COORDINATE + ONE_UNIT] = 0
            map[MAPCENTER_COORDINATE][MAPCENTER_COORDINATE + TWO_UNITS] = 3
            seenTileList.append((MAPCENTER_COORDINATE, MAPCENTER_COORDINATE + TWO_UNITS))
        else:
            map[MAPCENTER_COORDINATE][MAPCENTER_COORDINATE + ONE_UNIT] = 1

        if not leftWall(leftSensors):
            map[MAPCENTER_COORDINATE - ONE_UNIT][MAPCENTER_COORDINATE] = 0
            map[MAPCENTER_COORDINATE - TWO_UNITS][MAPCENTER_COORDINATE] = 3
            seenTileList.append((MAPCENTER_COORDINATE - TWO_UNITS, MAPCENTER_COORDINATE))
        else:
            map[MAPCENTER_COORDINATE - ONE_UNIT][MAPCENTER_COORDINATE] = 1
            

        if not rightWall(rightSensors):
            map[MAPCENTER_COORDINATE + ONE_UNIT][MAPCENTER_COORDINATE] = 0
            map[MAPCENTER_COORDINATE + TWO_UNITS][MAPCENTER_COORDINATE] = 3
            seenTileList.append((MAPCENTER_COORDINATE + TWO_UNITS, MAPCENTER_COORDINATE))
        else:
            map[MAPCENTER_COORDINATE + ONE_UNIT][MAPCENTER_COORDINATE] = 1
        
        map[MAPCENTER_COORDINATE][MAPCENTER_COORDINATE- ONE_UNIT] = 1 #Assumindo que tenha uma parede atrás

    #DeltaX positivo 
    if round(deltaX, 2) == 0.12 or round(deltaX, 2) == 0.11 or round(deltaX, 2) == 0.13:
        print("O Delta X é : {}".format(round(deltaX, 2)))
        currentColumnCoordinate = currentColumnCoordinate + TWO_UNITS
        currentLineCoordinate = currentLineCoordinate

        #Processo de Remover Tiles Visitados das lista de tiles vistos e marcando eles (adicionando na lista de tiles_marcados)
        if (currentLineCoordinate, currentColumnCoordinate) in seenTileList:
            seenTileList.pop(seenTileList.index((currentLineCoordinate, currentColumnCoordinate)))
            checkedTileList.append((currentLineCoordinate, currentColumnCoordinate))
            

        # Trocar o estado do tile de visto para visitado
        if map[currentLineCoordinate][currentColumnCoordinate] == 3:
            map[currentLineCoordinate][currentColumnCoordinate] = 2
            

        #Frente
        if not frontWall(frontSensors) and map[currentLineCoordinate][currentColumnCoordinate + TWO_UNITS] != 2 :
            map[currentLineCoordinate][currentColumnCoordinate + ONE_UNIT] = 0
            map[currentLineCoordinate][currentColumnCoordinate + TWO_UNITS] = 3

            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (currentLineCoordinate, currentColumnCoordinate + TWO_UNITS) in checkedTileList:
                seenTileList.append((currentLineCoordinate, currentColumnCoordinate + TWO_UNITS))
        elif frontWall(frontSensors):
            map[currentLineCoordinate][currentColumnCoordinate + ONE_UNIT] = 1

        #Esquerda
        if not leftWall(leftSensors) and map[currentLineCoordinate - TWO_UNITS][currentColumnCoordinate] != 2 : 
            map[currentLineCoordinate - ONE_UNIT][currentColumnCoordinate] = 0
            map[currentLineCoordinate - TWO_UNITS][currentColumnCoordinate] = 3
            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (currentLineCoordinate - TWO_UNITS, currentColumnCoordinate) in checkedTileList:
                seenTileList.append((currentLineCoordinate - TWO_UNITS, currentColumnCoordinate))

        elif leftWall(leftSensors):
            map[currentLineCoordinate - ONE_UNIT][currentColumnCoordinate] = 1

        # Direita
        if not rightWall(rightSensors) and map[currentLineCoordinate + TWO_UNITS][currentColumnCoordinate] != 2 :
            map[currentLineCoordinate + ONE_UNIT][currentColumnCoordinate] = 0
            map[currentLineCoordinate + TWO_UNITS][currentColumnCoordinate] = 3
            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (currentLineCoordinate + TWO_UNITS, currentColumnCoordinate) in checkedTileList:
                seenTileList.append((currentLineCoordinate + TWO_UNITS, currentColumnCoordinate))
        elif rightWall(rightSensors):
            map[currentLineCoordinate + ONE_UNIT][currentColumnCoordinate] = 1

    #DeltaX negativo 
    elif round(deltaX, 2) == -0.12 or round(deltaX, 2) == -0.11 or round(deltaX, 2) == -0.13:
        print("O Delta X é : {}".format(round(deltaX,2)))
        currentColumnCoordinate = currentColumnCoordinate - TWO_UNITS
        currentLineCoordinate = currentLineCoordinate

        #Processo de Remover Tiles Visitados das lista de tiles vistos e marcando eles (adicionando na lista de tiles_marcados)
        if (currentLineCoordinate, currentColumnCoordinate) in seenTileList:
            seenTileList.pop(seenTileList.index((currentLineCoordinate, currentColumnCoordinate)))
            checkedTileList.append((currentLineCoordinate, currentColumnCoordinate))

        #Trocar o estado do tile de visto para visitado
        if map[currentLineCoordinate][currentColumnCoordinate] == 3:
            map[currentLineCoordinate][currentColumnCoordinate] = 2

        #Frente
        if not frontWall(frontSensors) and map[currentLineCoordinate][currentColumnCoordinate - TWO_UNITS] != 2:
            map[currentLineCoordinate][currentColumnCoordinate - ONE_UNIT] = 0
            map[currentLineCoordinate][currentColumnCoordinate - TWO_UNITS] = 3
            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (currentLineCoordinate, currentColumnCoordinate - TWO_UNITS) in checkedTileList:
                seenTileList.append((currentLineCoordinate, currentColumnCoordinate - TWO_UNITS))
        elif frontWall(frontSensors):
            map[currentLineCoordinate][currentColumnCoordinate - ONE_UNIT] = 1

        #Esquerda
        if not leftWall(leftSensors) and map[currentLineCoordinate + TWO_UNITS][currentColumnCoordinate] != 2:
            map[currentLineCoordinate + ONE_UNIT][currentColumnCoordinate] = 0
            map[currentLineCoordinate + TWO_UNITS][currentColumnCoordinate] = 3
            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (currentLineCoordinate + TWO_UNITS, currentColumnCoordinate) in checkedTileList:
                seenTileList.append((currentLineCoordinate + TWO_UNITS, currentColumnCoordinate))
        elif leftWall(leftSensors):
            map[currentLineCoordinate + ONE_UNIT][currentColumnCoordinate] = 1

        #Direita
        if not rightWall(rightSensors) and map[currentLineCoordinate - TWO_UNITS][currentColumnCoordinate] != 2:
            map[currentLineCoordinate - ONE_UNIT][currentColumnCoordinate] = 0
            map[currentLineCoordinate - TWO_UNITS][currentColumnCoordinate] = 3
            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (currentLineCoordinate - TWO_UNITS, currentColumnCoordinate) in checkedTileList:
                seenTileList.append((currentLineCoordinate - TWO_UNITS, currentColumnCoordinate))
        elif rightWall(rightSensors):
            map[currentLineCoordinate - ONE_UNIT][currentColumnCoordinate] = 1
    #DeltaY positivo 
    if round(deltaY, 2) == 0.12 or round(deltaY, 2) == 0.11 or round(deltaY, 2) == 0.13 :
        print("O Delta Y é : {}".format(round(deltaY,2)))
        currentLineCoordinate = currentLineCoordinate + TWO_UNITS
        currentColumnCoordinate = currentColumnCoordinate
        print((currentLineCoordinate, currentColumnCoordinate), "; ")
        #Processo de Remover Tiles Visitados das lista de tiles vistos e marcando eles (adicionando na lista de tiles_marcados)
        if (currentLineCoordinate, currentColumnCoordinate) in seenTileList:
            seenTileList.pop(seenTileList.index((currentLineCoordinate, currentColumnCoordinate)))
            checkedTileList.append((currentLineCoordinate, currentColumnCoordinate))

        #Trocar o tile visto por visitado
        if map[currentLineCoordinate][currentColumnCoordinate] == 3:
            map[currentLineCoordinate][currentColumnCoordinate] = 2

        #Frente
        if not frontWall(frontSensors) and map[currentLineCoordinate + TWO_UNITS ][currentColumnCoordinate] != 2:
            map[currentLineCoordinate + ONE_UNIT ][currentColumnCoordinate] = 0
            map[currentLineCoordinate + TWO_UNITS ][currentColumnCoordinate] = 3
            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (currentLineCoordinate + TWO_UNITS, currentColumnCoordinate) in checkedTileList:
                seenTileList.append((currentLineCoordinate + TWO_UNITS, currentColumnCoordinate))

        elif frontWall(frontSensors):
            map[currentLineCoordinate + ONE_UNIT ][currentColumnCoordinate] = 1

        #Direita
        if not rightWall(rightSensors) and map[currentLineCoordinate][currentColumnCoordinate - TWO_UNITS] != 2:
            map[currentLineCoordinate][currentColumnCoordinate - ONE_UNIT] = 0
            map[currentLineCoordinate][currentColumnCoordinate - TWO_UNITS] = 3
            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (currentLineCoordinate, currentColumnCoordinate - TWO_UNITS) in checkedTileList:
                seenTileList.append((currentLineCoordinate, currentColumnCoordinate - TWO_UNITS))
        elif rightWall(rightSensors):
            map[currentLineCoordinate][currentColumnCoordinate - ONE_UNIT] = 1

        #Esquerda
        if not leftWall(leftSensors) and map[currentLineCoordinate][currentColumnCoordinate + TWO_UNITS] != 2:
            map[currentLineCoordinate][currentColumnCoordinate + ONE_UNIT] = 0
            map[currentLineCoordinate][currentColumnCoordinate + TWO_UNITS] = 3
            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (currentLineCoordinate, currentColumnCoordinate + TWO_UNITS) in checkedTileList:
                seenTileList.append((currentLineCoordinate, currentColumnCoordinate + TWO_UNITS))
        elif leftWall(leftSensors):
            map[currentLineCoordinate][currentColumnCoordinate + ONE_UNIT] =1
    #DeltaY negativo 
    elif round(deltaY,2) == -0.12 or round(deltaY, 2) == -0.11 or round(deltaY, 2) == -0.13:
        print("O Delta Y é : {}".format(round(deltaY,2)))
        currentLineCoordinate = currentLineCoordinate - TWO_UNITS
        currentColumnCoordinate = currentColumnCoordinate

        #Processo de Remover Tiles Visitados das lista de tiles vistos e marcando eles (adicionando na lista de tiles_marcados)
        if (currentLineCoordinate, currentColumnCoordinate) in seenTileList:
            seenTileList.pop(seenTileList.index((currentLineCoordinate, currentColumnCoordinate)))
            checkedTileList.append((currentLineCoordinate, currentColumnCoordinate))

        #Trocando o tile visto por visitado
        if map[currentLineCoordinate][currentColumnCoordinate] == 3:
            map[currentLineCoordinate][currentColumnCoordinate] = 2

        #Frente
        if not frontWall(frontSensors) and map[currentLineCoordinate - TWO_UNITS][currentColumnCoordinate] != 2:
            map[currentLineCoordinate - ONE_UNIT][currentColumnCoordinate] = 0
            map[currentLineCoordinate - TWO_UNITS][currentColumnCoordinate] = 3
            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (currentLineCoordinate - TWO_UNITS, currentColumnCoordinate) in checkedTileList:
                seenTileList.append((currentLineCoordinate - TWO_UNITS, currentColumnCoordinate))
        elif frontWall(frontSensors):
            map[currentLineCoordinate - ONE_UNIT][currentColumnCoordinate] = 1
        
        #Direita
        if not rightWall(rightSensors) and map[currentLineCoordinate][currentColumnCoordinate + TWO_UNITS] != 2:
            map[currentLineCoordinate][currentColumnCoordinate + ONE_UNIT] = 0
            map[currentLineCoordinate][currentColumnCoordinate + TWO_UNITS] = 3
            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (currentLineCoordinate, currentColumnCoordinate + TWO_UNITS) in checkedTileList:
                seenTileList.append((currentLineCoordinate, currentColumnCoordinate + TWO_UNITS))
        elif rightWall(rightSensors):
            map[currentLineCoordinate][currentColumnCoordinate + ONE_UNIT] = 1

        #Esquerda
        if not leftWall(leftSensors) and map[currentLineCoordinate][currentColumnCoordinate - TWO_UNITS] != 2:
            map[currentLineCoordinate][currentColumnCoordinate - ONE_UNIT] = 0
            map[currentLineCoordinate][currentColumnCoordinate - TWO_UNITS] = 3
            #Processo de Adicionar o tile na lista de tiles vistos 
            if not (currentLineCoordinate, currentColumnCoordinate - TWO_UNITS) in checkedTileList:
                seenTileList.append((currentLineCoordinate, currentColumnCoordinate - TWO_UNITS))
        elif leftWall(leftSensors):
            map[currentLineCoordinate][currentColumnCoordinate - ONE_UNIT] = 1
    
    previousPosition = currentPosition
    previousPositionX = currentPositionX
    previousPositionY = currentPostionY

    changeOrientation()

def hasHole():
    color = colorSensor.getImage()
    if color == colorSensor:
        return True
    return False
def delay(ms):
    initTime = robot.getTime()      # Store starting time (in seconds)
    while robot.step(TIMESTEP) != -1:
        if (robot.getTime() - initTime) * 1000.0 > ms: # If time elapsed (converted into ms) is greater than value passed in
            break

# Loop principal
goAhead(0.01)
sleep(0.1)  # gambiarra essencial para o robô não bugar :) NÃO TIRAR
while robot.step(TIMESTEP) != -1:
    PID()  
    print("A lista de tiles vistos é : {}".format(seenTileList))
    print("A lista de tiles marcados é : {}".format(checkedTileList))  