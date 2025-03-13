from slam import slam
from types_and_constants import ROBOT_RADIUS, THRESHOLD_SLAM
from figure import fig
import numpy as np

contador = 0

def slam_snapshot(gps_position: list, side_angle_to_distance_mapper: dict[float, float], robot_orientation: float) -> None:
    slam.take_snapshot(gps_position, side_angle_to_distance_mapper, robot_orientation)

def get_slam_list_atual() -> list:
    return slam.get_list_atual()

def get_slam_list() -> list:
    return slam.get_list()

def calculate_min_node(nodeX: list, nodeY: list, robot_position: list) -> list:
    # Utilizando a lista de nodes, calcula o node mais próximo do robo
    distance = float("inf")
    for nx, ny in zip(nodeX, nodeY):
        if (nx - robot_position[0]) ** 2 + (ny - robot_position[1]) ** 2 < distance:
            distance = (nx - robot_position[0]) ** 2 + (ny - robot_position[1]) ** 2
            min_node = [nx, ny]
    return min_node
    

class Navigation:
    def __init__(self) -> None:
        self.list_x_total = []
        self.list_y_total = []

        self.existing_nodes = set()
    
    def get_node(self, listx: list, listy: list, robot_position: list, ) -> None:
        x0, y0 = robot_position
        radius_offset = ROBOT_RADIUS + THRESHOLD_SLAM
        contador2 =0
        for lx, ly in zip(listx, listy):
            
            theta = np.arctan2(y0 - ly, lx - x0)
            px = lx - radius_offset * np.cos(theta)
            py = ly + radius_offset * np.sin(theta)
            
            contador2+=1
            # Adiciona somente se não estiver muito próximo de outro node e não sobreponha pontos existentes
            if all(
                (px - nx) ** 2 + (py - ny) ** 2 > radius_offset ** 2 for nx, ny in self.existing_nodes
            ) and all(
                (px - wx) ** 2 + (py - wy) ** 2 > ROBOT_RADIUS ** 2 for wx, wy in zip(listx, listy)
            ):
                self.existing_nodes.add((px, py))
        #print(f"Contador2 : {contador2}")

    def navigate(self, gps_position: list, side_angle_to_distance_mapper: dict[float, float], robot_orientation: float) -> None:
        global contador
        slam_snapshot(gps_position, side_angle_to_distance_mapper, robot_orientation)
        new_x, new_y = get_slam_list_atual()

        self.get_node(new_x, new_y, gps_position)
        self.list_x_total.extend(new_x)
        self.list_y_total.extend(new_y)
        

        if contador % 25 == 0:
            #Quero que plote os node_x e node_y 
            print(f"LIST_X : {len(self.list_x_total)}, LIST_Y : {len(self.list_y_total)}, EXISTING_NODES : {len(self.existing_nodes)}") 
            fig.multi_plott(self.list_x_total, self.list_y_total, "SLAM", 'b', [node[0] for node in self.existing_nodes], [node[1] for node in self.existing_nodes], "Node", 'r')
            pass

        

        contador += 1

navigation = Navigation()
