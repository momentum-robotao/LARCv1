try:
    import logging
    import os
    from datetime import datetime
    import math

    from controller import Robot as WebotsRobot  # type: ignore
    from helpers import delay
    from debugging import ALL_SYSTEMS, DebugInfo, HttpHandler, System
    from devices import GPS, IMU, ColorSensor, Communicator, Lidar, Motor, DistanceSensor, Camera
    from dfs import dfs
    from maze import Maze
    from robot import Robot
    from types_and_constants import DEBUG, NGROK_URL, ON_DOCKER, Coordinate, MAX_SPEED, SpecialTileType
    #import matplotlib.pyplot as plt
    from recognize_wall_token import reconhece_lado

    radius = (0.071/2) + 0.005

    x_total = []
    y_total = []
    contador_imagem = 0
    contador_imagem1 = 0
    # Variáveis derivada e Integral
    integral = 0
    derivada = 0
    last_error = 0
    erro = 0
    erro2 = 0
    last_error2 = 0
    Pot0 = MAX_SPEED - 3.5
    condicao_sala4 = False
    desejada = 0.045
    IMU_THRESHOLD = IMU_THRESHOLD=math.pi/6

    if ON_DOCKER:
        import requests  # type: ignore

    def radians(degrees: int)->float:
                return degrees*(math.pi/180)
    
    def conferir_circulo(centro: list, ponto: list, raio: float) -> bool:
        """
        confere se o ponto está dentro da circunferencia de centro e raio especificado
        """
        if (math.pow(ponto[0] - centro[0], 2) + math.pow(ponto[1] - centro[1], 2)) < (math.pow(raio,2)) : return True
        else : return False
    

    def get_distance(angle) -> float:
                
                rangeImage = lidar._get_range_image_() # Step 4: Retrieve the range image
                n = (angle*512)//360
                print()
                dist = 0
                count = 0
                for i in range(4):
                    if rangeImage[n+ i*512] != float('inf') : 
                        dist += rangeImage[n+ i*512]
                        count +=1
                if count != 0:
                    dist = dist/count
                else : dist = 0
                return dist

    def get_posicao_atual():
                ''' 
                    posicao_atual = gps.getValues()
                    posicaoX_atual = posicao_atual[0]
                    posicaoY_atual = posicao_atual[2]

                    return [posicaoX_atual, posicaoY_atual]
                '''
                P = gps.get_position()
                x = P.__getattribute__('x')
                y = P.__getattribute__('y')
                coordinates = [x,y]
                #print(f"coordinates é : {coordinates}")
                return coordinates

    

    # Initialize logger
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


    def main() -> None:
        # Initialize DebugInfo instance
        if DEBUG:
            logger.info(f"Começando nova execução: {datetime.now()}")

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
                #System.wall_token_recognition,
                #System.wall_token_classification,
                System.unknown_error,
                #System.maze_snapshot,
            ]
            global debug_info
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
            
            global motor, lidar, gps, imu, color_sensor, communicator, distancesensor
            gps = GPS(webots_robot, debug_info)
            motor = Motor(webots_robot, debug_info)
            lidar = Lidar(webots_robot, debug_info)
            imu = IMU(webots_robot, debug_info)
            color_sensor = ColorSensor(webots_robot, debug_info)
            communicator = Communicator(webots_robot, debug_info)
            distance_sensor = DistanceSensor(webots_robot, debug_info)
            camera = Camera(webots_robot, debug_info)

        except Exception:
            if DEBUG:
                debug_info.send(
                    "Erro durante inicialização dos devices", System.initialization, "error"
                )
                raise

        try:
            global robot
            robot = Robot(
                webots_robot, motor, lidar, gps, imu, color_sensor, communicator, camera, distance_sensor, debug_info
            )
            robot.step()
        except Exception:
            if DEBUG:
                debug_info.send(
                    "Erro durante inicialização do robô", System.initialization, "error"
                )
                raise

        # Solve map
        try:

            def get_mapa():
                global contador_imagem
                x = []
                y = []
                soma = imu.get_roll_pitch_yaw()
                #print(f"SOMA : {soma}" )

                
                for i in range(360):
                    #print(f"dist {i} : {get_distance(i)}")
                    if (get_distance(i) > (1.5)*0.12) or (get_distance(i) < 0.005):
                        x.append(get_posicao_atual()[0])
                        y.append(get_posicao_atual()[1])
                    else:
                        theta = soma + i
                        x.append(get_distance(i)*math.cos(radians(theta)) + get_posicao_atual()[0])
                        y.append(get_distance(i)*math.sin(radians(theta)) + get_posicao_atual()[1])


                #fig, ax = plt.subplots()
                #ax.yaxis.set_inverted(True)
                #ax.scatter(x, y)
                #ax.plot(get_posicao_atual(gps)[0], get_posicao_atual(gps)[1], 'x')
                #fig.savefig(f"C:\\Users\\goten\\Desktop\\LARCv1\\LARC_Simulation_Codes\\images\\mapa_{contador_imagem}.png")
                #plt.close(fig)
                contador_imagem +=1
                return x,y


            class Point():
                def __init__(self, dist, x, y):
                    self.dist = dist
                    self.x = x
                    self.y = y
                
                def get_dist(self):
                    return self.dist
                
                def get_x(self):
                    return self.x
                
                def get_y(self):
                    return self.y
            
            def tem_parede_frente():
                for i in range(5):
                    if get_distance(i) < 0.043: 
                        print(f"Vi parede no angulo {i}; distância : {get_distance(i)}")
                        return True
                for i in range(359, 356, -1) :
                    if get_distance(i) < 0.043: 
                        print(f"Vi parede no angulo {i}; distância : {get_distance(i)}")
                        return True
                for i in range(6, 45): 
                    if (get_distance(i)) < 0.038: 
                        print(f"Vi parede no angulo {i}; distância : {get_distance(i)}")
                        return True
                for i in range(356, 316): 
                    if (get_distance(i)) < 0.038: 
                        print(f"Vi parede no angulo {i}; distância : {get_distance(i)}")
                        return True
                for i in range(45, 60): 
                    if (get_distance(i)) < 0.038: 
                        print(f"Vi parede no angulo {i}; distância : {get_distance(i)}")
                        return True
                for i in range(316, 300, -1) : 
                    if (get_distance(i)) < 0.038: 
                        print(f"Vi parede no angulo {i}; distância : {get_distance(i)}")
                        return True
                return False

            def tem_parede_direita():
                for i in range(75, 105):
                    if get_distance(i) < 0.038 : return True
                return False
            def tem_parede_esquerda():
                for i in range(255, 285):
                    if get_distance(i) < 0.038 : return True
                return False
            # Função para calcular a média das distâncias entre 0 e 180 graus

            def calcular_media_distancias():
                dist1 = get_distance(70)
                dist2 = get_distance(110)
                soma = 0
                soma2 = 0
                soma3 = 0
                for i in range(87,93):
                    soma += get_distance(i)
                dist3 = soma/5
                for i in range(35, 55):
                    soma2 += get_distance(i)
                dist4 = soma2/20
                for i in range(135, 155):
                    soma3 += get_distance(i)
                dist5 = soma3/20

                for i in range(267,273):
                    soma += get_distance(i)
                dist6 = soma/5
                
                
                return dist1,dist2, dist3, dist4, dist5, dist6
            
            def parede_tras():
                menor_valor = 10000000000
                for i in range(100, 260):
                    if get_distance(i)<menor_valor: menor_valor = get_distance(i)
                if menor_valor < desejada : return menor_valor
                
                else : 
                    menor_valor = 0
                    return menor_valor


            def voltar_tras(pot):
                tras = parede_tras()
                motor.set_velocity(-Pot0, -Pot0)
                if tras != 0 :
                    tempo = (tras)/(pot*radius)
                    delay(webots_robot, debug_info, tempo)
                    motor.stop()
                else : 
                    delay(webots_robot,debug_info,120)
                    motor.stop()
            
            def virar_max():
                global desejada
                angle = 60
                for i in range(359, 285, -1):
                    if(get_distance(i)<= desejada+0.01) : continue
                    else : angle = 360 - i
                return angle
            
            def color_sala4():
                
                if color_sensor.check_colored_special_tile() in [SpecialTileType.PASSAGE_2_4,SpecialTileType.PASSAGE_1_4, SpecialTileType.PASSAGE_3_4]: 
                    print(color_sensor.check_colored_special_tile())
                    return True
                else : return False
            
            def move_until():
                while robot.step() != -1:
                    if not tem_parede_frente():
                        motor.set_velocity(Pot0, Pot0)
                    else : break

            def ver_obstaculo():
                frente = distance_sensor.pegar_distancia(distance_sensor._front)
                tras_esquerda = distance_sensor.pegar_distancia(distance_sensor._backleft)
                tras_direita = distance_sensor.pegar_distancia(distance_sensor._backright)
                distancia = 0.037
                condicao = False
                onde = []
                print(tras_direita)
                if frente<distancia:
                    condicao = True
                    onde.append('frente')
                if tras_esquerda<distancia:
                    condicao = True
                    onde.append('tras_esquerda')
                if tras_direita<distancia:
                    condicao = True
                    onde.append('tras_direita')
                return condicao, onde
                    
                
                    

            def seguir_parede_sala4():
                global desejada, Pot0, erro, last_error, erro2, last_error2
                #kp = 35 
                #kp2 = 40
                #kd = 3
                #kd2 = 2.5
                kp = 35
                kp2 = 45
                kd = 5
                kd2 = 3.5

                PotD = Pot0
                PotE = Pot0

                
                
                # Obter a distância média usando múltiplas leituras do LiDAR
                d1,d2, d3,d4,d5, d6 = calcular_media_distancias()
                for i in [d1,d2,d3,d4,d5]:
                    if i > desejada: i = desejada
                    else : continue

                erro = d1-d2
                erro2 = d1-desejada

                #soma += erro
                PotD = Pot0 - (kp * erro) - last_error*kd - kp2*(erro2) - kd2*last_error2
                PotE = Pot0 + (kp * erro) + last_error*kd + kp2*(erro2) + kd2*last_error2

                
                # Limitar as velocidades para evitar ultrapassar o limite dos motores
                PotD = max(min(PotD, MAX_SPEED), -MAX_SPEED)
                PotE = max(min(PotE, MAX_SPEED), -MAX_SPEED)

                if distance_sensor.detect_hole(webots_robot): 
                    voltar_tras(Pot0)
                    robot.rotate_180()
                    motor.stop()
                
                #print(f"PotD : {PotD} ; PotE : {PotE} ; Erro = {erro}; last_error : {last_error}")
                print(f"direita : {d3} ; esquerda : {d6}" )
                
                if tem_parede_frente():
                    #print("bostacolossal")
                    print("VI PAREDE FRENTE!")
                    voltar_tras(Pot0)
                    delay(webots_robot, debug_info, 100)
                    angle = virar_max()
                    if not tem_parede_esquerda():
                        robot.rotate('left', radians(angle))
                    elif not tem_parede_direita():
                        robot.rotate('right', radians(angle))
                    else:
                        robot.rotate_180()
                
                else:
                    # Seguir em frente quando possível
                    motor.set_velocity(PotE, PotD)
                    #print(f" DIREITA (90 graus) : {get_distance(90)} ; ESQUERDA (270 graus)  : {get_distance(270)}")
                    
                    robot.recognize_wall_token()

                    #motor.set_velocity(Pot0, Pot0)
                    #delay(webots_robot, debug_info, 100)
                    #motor.stop()    



                    '''    
                    motor.set_left_motor_power(Pot0)
                    motor.set_right_motor_power(Pot0)
                    delay(webots_robot, debug_info, 50)
                    motor.stop()
                    ''' 
                    #print(f"d1 = {d1} ; d2 = {d2} ; d3 : {d3} ; d4: {d4}; d5:{d5}")
                    last_error = erro - last_error
                    last2 = erro2 - last_error2
                    #print(f"POT E : {Pot0}")
                    #print(f"POT D : {Pot0}")
                    #print(f"tem parede frente : {tem_parede_frente() or lidar.has_wall('front')}")
                    #print(f"tem parede direita : {tem_parede_direita() or lidar.has_wall('right')}")
                    #print(f"tem parede de esquerda: {tem_parede_esquerda() or lidar.has_wall('left')}")
                #dt = robot.time_step/1000
                # Cálculo da integral e derivada
                #integral += erro * dt  # dt é o intervalo de tempo entre as leituras
                #derivada = (erro - last_error) / dt

                # Ajusta as velocidades dos motores usando o erro e o fator proporcional
                
                
                

                # Definir as velocidades dos motores
                
                #motor._right_motor.setVelocity(PotD)
                #motor._left_motor.setVelocity(PotE)

                
                
            


            def sala4(): 
                global condicao_sala4
                if color_sala4():
                    motor.set_velocity(Pot0, Pot0)
                    delay(webots_robot, debug_info, 750)
                    motor.stop()
                    delay(webots_robot, debug_info, 2000)

                    if not tem_parede_frente() or not lidar.has_wall('front') : 
                        print(f"NAO TEM NADA NA MINHA FRENTE ; d[80] : {get_distance(80)})")
                        move_until()
                        condicao_sala4 = True
                    elif not tem_parede_esquerda() or not lidar.has_wall('left'):
                        print(" TEM FRENTE E NAO TEM NADA NA MINHA ESQUERDA")
                        robot.rotate_90_left()
                        move_until()
                        condicao_sala4 = True
                    elif not tem_parede_direita() or not lidar.has_wall('right'): 
                        (" TEM FRENTE E ESQUERDA E NAO TEM NADA NA MINHA DIREITA")
                        robot.rotate_90_right()
                        move_until()
                        condicao_sala4 = True
                    else : 
                        pass
                if condicao_sala4: 
                    seguir_parede_sala4()
                    if color_sala4():
                        condicao_sala4 = False
                        print("Sai da sala 4")
                else : 
                    robot.rotate_180()
                    condicao_sala4 = False
                
            def cercado():
                for i in range(360) : 
                    if get_distance(i) < 0.038:
                        return True
                    else : return False
                


            
            while robot.step() != -1:
                
                #x, y = get_mapa()
                #seguir_parede_sala4()
                condicao, onde = ver_obstaculo()
                print(f"Tem obstáculo : {print(condicao)} ; onde : {print(onde)}")
                
                #sala4()
                
                #fig, ax = plt.subplots()
                #ax.yaxis.set_inverted(True)
                #ax.scatter(x_total,y_total)
                #fig.savefig(f"C:\\Users\\goten\\Desktop\\LARCv1\\LARC_Simulation_Codes\\images\\mapa_total.png")
                #plt.close(fig)
                delay(webots_robot, debug_info, 100)

        except Exception:
            if DEBUG:
                debug_info.send(
                    "Erro inesperado enquanto resolvia o mapa",
                    System.unknown_error,
                    "critical",
                )
                raise


    try:
        main()
    except Exception:
        pass
    if ON_DOCKER:
        http_handler.send_queue_data()
except Exception as err:
    from controller import Robot as WebotsRobot  # type: ignore

    webots_robot = WebotsRobot()
    webots_robot.step(int(os.getenv("TIME_STEP", 32)))

    if (os.getenv("ON_DOCKER", "") + " ").upper()[0] in ["T", "1"]:
        import logging

        import requests  # type: ignore

        class HttpHandler(logging.Handler):  # type: ignore
            def __init__(
                self,
                url: str,
            ):
                self.url = url
                logging.Handler.__init__(self=self)

            def emit(self, record: logging.LogRecord) -> None:
                requests.post(
                    self.url, json={"new_entries": f"{self.format(record)}\n"}
                )

        logging.basicConfig()
        logger = logging.getLogger("Robo LARC v1")

        NGROK_URL = ""
        with open("./ngrok.txt", "r") as file:
            NGROK_URL = file.readlines()[0]
        print(f"Url do ngrok recuperada: {NGROK_URL}")

        try:
            resp = requests.post(f"{NGROK_URL}/start_simulation")
            print(resp.text)
        except Exception as e:
            print(f"Erro: {e}")

        http_handler = HttpHandler(f"{NGROK_URL}/send")
        http_handler.setLevel(logging.DEBUG)
        formatter = logging.Formatter("%(levelname)s %(message)s")
        http_handler.setFormatter(formatter)
        logger.addHandler(http_handler)

        logger.critical("erro geral", exc_info=True)
    print(err)
