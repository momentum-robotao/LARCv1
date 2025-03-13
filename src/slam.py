import numpy as np
from types_and_constants import Coordinate
from figure import fig
import time
import robot

start_angle = 0
class Slam: 

    def __init__(self) -> None:
        self.list_x_total = []
        self.list_y_total = []
        self.list_x_atual = []
        self.list_y_atual = []
        pass
        
    def take_snapshot(self, gps_position: list, side_angle_to_distance_mapper: dict[float, float], robot_orientation : float) -> None:
        x0 = gps_position[0]
        y0 = gps_position[1]

        orientation = robot_orientation

        self.list_x_atual = []
        self.list_y_atual = []

        
        for side_angle, distance in side_angle_to_distance_mapper.items():
            if distance == float("inf"):
                continue
            if distance < 0.18 : 
                corrected_angle = side_angle + orientation

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
