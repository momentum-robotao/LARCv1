import numpy as np
from types_and_constants import Coordinate
from figure import fig
import time
import robot


condicao = True
class Slam: 

    def __init__(self) -> None:
        self.list_x_total = []
        self.list_y_total = []
        self.list_x_atual = []
        self.list_y_atual = []
        self.start_angle = 0
        pass
        
    def take_snapshot(self, gps_position: list, side_angle_to_distance_mapper: dict[float, float], robot_orientation : float) -> None:
        global condicao
        x0 = gps_position[0]
        y0 = gps_position[1]

        orientation = robot_orientation

        self.list_x_atual = []
        self.list_y_atual = []

        possible_angles = {(1, 0): 0, (0, 1): np.pi/2, (-1, 0): np.pi, (0, -1): 3*np.pi/2}
        if condicao : 
            self.start_angle = possible_angles[robot.DIST_CHANGE_MAPPER[robot.round_angle(orientation)]]
            condicao = False
        
        
        print(f"Angulo TRATADO: {self.start_angle}")
        print(f"X : {x0} Y : {y0}")
        #time.sleep(2)
        
        
        for side_angle, distance in side_angle_to_distance_mapper.items():
            if distance == float("inf"):
                continue
            if distance < 0.18 : 
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
        ''' 
        for side_angle, distance in side_angle_to_distance_mapper.items():
            print(f"side angle {side_angle} : {distance} ")
        time.sleep(5)
        '''
        
    def get_list_atual(self) -> list:
        return self.list_x_atual, self.list_y_atual

slam = Slam()
