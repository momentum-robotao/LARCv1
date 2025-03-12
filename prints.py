from controller import Robot
import cv2
import numpy as np
import os
import time

# Inicializa o robô
robot = Robot()

# Tempo de passo do simulador
timestep = int(robot.getBasicTimeStep())

# Configuração da câmera
camera2 = robot.getDevice("camera2")
camera2.enable(timestep)

# Declara os motores
left_motor = robot.getDevice("wheel1")
right_motor = robot.getDevice("wheel2")

# Cria a pasta para salvar os frames, se não existir
save_path = r"C:\Users\nicol\Downloads\frames"
if not os.path.exists(save_path):
    os.makedirs(save_path)

frame_counter = 0
capture_interval = 0.5  # Tempo entre capturas (2 frames por segundo)
last_capture_time = time.time()

# Loop principal
while robot.step(timestep) != -1:
    current_time = time.time()
    
    # Captura a cada 0.5 segundos (2 FPS)
    if current_time - last_capture_time >= capture_interval:
        image = camera2.getImageArray()
        if image is not None:
            frame = np.array(image, dtype=np.uint8)  # Converte a imagem para array NumPy
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)  # Converte RGB para BGR

            # Salva o frame como imagem
            filename = os.path.join(save_path, f"frame_{frame_counter:04d}.png")
            cv2.imwrite(filename, frame)
            print(f"Frame {frame_counter} salvo em {filename}")
            frame_counter += 200
            last_capture_time = current_time
