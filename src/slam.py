import numpy as np
from types_and_constants import Coordinate
from figure import fig
import time
import robot

contador = -1000000
start_angle = robot.expected_angle
class Slam: 

    def __init__(self) -> None:
        self.list_x = []
        self.list_y = []
        pass
        
    def take_snapshot(self, gps_position: list, side_angle_to_distance_mapper: dict[float, float]) -> None:
        global contador 
        if contador <3: 
            x0 = gps_position[0]
            y0 = gps_position[1]

            
            for side_angle, distance in side_angle_to_distance_mapper.items():
                if distance == float ("inf"):
                    continue
                if distance < 0.18 : 
                    distance_x = distance*np.cos(side_angle) + x0
                    distance_y = distance*np.sin(side_angle) + y0
                    self.list_x.append(distance_x)
                    self.list_y.append(distance_y)
            fig.plott(self.list_x, self.list_y, "SLAM", 'b')
            contador+=1
        else : 
            print("Hello World")

        ''' 
        for side_angle, distance in side_angle_to_distance_mapper.items():
            print(f"side angle {side_angle} : {distance} ")
        time.sleep(5)
        '''
        
        

slam = Slam()
