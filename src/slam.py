import numpy as np
from types_and_constants import Coordinate, TILE_SIZE
from figure import fig
import time
import robot

LIDAR_LIMIT =  TILE_SIZE * 8

condicao = True
class Slam: 

    def __init__(self) -> None:
        self.list_x_total = []
        self.list_y_total = []
        self.list_x_atual = []
        self.list_y_atual = []
        self.start_angle = 0

        pass
    
    def atualizar_start_angle(self, robot_orientation :float):
        possible_angles = {(1, 0): 0, (0, 1): np.pi/2, (-1, 0): np.pi, (0, -1): 3*np.pi/2}
        self.start_angle = possible_angles[robot.DIST_CHANGE_MAPPER[robot.round_angle(robot_orientation)]]
        print(f"START_ANGLE : {self.start_angle}")
        
    def take_snapshot(self, gps_position: list, side_angle_to_distance_mapper: dict[float, float], robot_orientation : float) -> None:
        global LIDAR_LIMIT
        global condicao
        x0 = gps_position[0]
        y0 = gps_position[1]

        orientation = robot_orientation

        self.list_x_atual = []
        self.list_y_atual = []
        if condicao : 
            self.atualizar_start_angle(robot_orientation)
            LIDAR_LIMIT = LIDAR_LIMIT/2 # Atualiza o LIDAR_LIMIT PARA MENORAR O CAMPO DE VISÃO DO ROBÔ APOS O PRIMEIRO MOVIMENTO
            condicao = False
        
        
        for side_angle, distance in side_angle_to_distance_mapper.items():  
            if distance == float("inf"):
                continue
            if distance < LIDAR_LIMIT : 
                corrected_angle = side_angle + self.start_angle + orientation

                distance_x = distance*np.cos(corrected_angle) + x0
                distance_y = distance*np.sin(corrected_angle) +  y0
                self.list_x_total.append(distance_x)
                self.list_y_total.append(distance_y)
                self.list_x_atual.append(distance_x)
                self.list_y_atual.append(distance_y)
        #fig.plott(self.list_x, self.list_y, "SLAM", 'b') # Comment This when running in competition
    
    def get_list_total(self) -> list:
        return self.list_x_total, self.list_y_total
        
        
        
    def get_list_atual(self) -> list:
        return self.list_x_atual, self.list_y_atual
    
    def get_start_angle(self) -> float:
        return self.start_angle

slam = Slam()
