import numpy as np
from types_and_constants import Coordinate
from figure import fig



class Slam: 
    def __init__(self) -> None:
        self.list_x = []
        self.list_y = []
        
    def take_snapshot(self, gps_position: Coordinate, side_angle_to_distance_mapper: dict[float, float]) -> None:
        x0 = gps_position.x
        y0 = gps_position.y


        for side_angle, distance in side_angle_to_distance_mapper.items():
            if distance == float ("inf"):
                continue
            distance_x = distance*np.cos(side_angle) + x0
            distance_y = distance*np.sin(side_angle) + y0
            self.list_x.append(distance_x)
            self.list_y.append(distance_y)
        fig.plott(self.list_x, self.list_y, "SLAM", 'b')
        
        
        

slam = Slam()
