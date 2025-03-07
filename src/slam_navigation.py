from slam import slam
from types_and_constants import ROBOT_RADIUS, THRESHOLD_SLAM
from figure import fig
import numpy as np

contador = 0

def slam_snapshot(gps_position: list, side_angle_to_distance_mapper: dict[float, float], robot_orientation: float) -> None:
    slam.take_snapshot(gps_position, side_angle_to_distance_mapper, robot_orientation)

def get_slam_list() -> list:
    return slam.get_list()

def get_node(listx: list, listy: list, robot_position: list, existing_nodes: set) -> tuple[list, list]:
    nodeX, nodeY = [], []
    x0, y0 = robot_position
    radius_offset = ROBOT_RADIUS + THRESHOLD_SLAM

    for lx, ly in zip(listx, listy):
        theta = np.arctan2(y0 - ly, lx - x0)
        px = lx - radius_offset * np.cos(theta)
        py = ly + radius_offset * np.sin(theta)
        
        # Adiciona somente se não estiver muito próximo de outro node e não sobreponha pontos existentes
        if all((px - nx) ** 2 + (py - ny) ** 2 > radius_offset ** 2 for nx, ny in existing_nodes) and all((px - wx) ** 2 + (py - wy) ** 2 > ROBOT_RADIUS ** 2 for wx, wy in zip(listx, listy)):
            existing_nodes.add((px, py))
            nodeX.append(px)
            nodeY.append(py)
    
    return nodeX, nodeY

class Navigation:
    def __init__(self) -> None:
        self.list_x_antigo = []
        self.list_y_antigo = []
        self.node_X = []
        self.node_Y = []
        self.existing_nodes = set()

    def navigate(self, gps_position: list, side_angle_to_distance_mapper: dict[float, float], robot_orientation: float) -> None:
        global contador
        slam_snapshot(gps_position, side_angle_to_distance_mapper, robot_orientation)
        new_x, new_y = get_slam_list()
        
        if new_x and new_y:
            nodeX, nodeY = get_node(new_x, new_y, gps_position, self.existing_nodes)
            self.node_X.extend(nodeX)
            self.node_Y.extend(nodeY)
            self.list_x_antigo.extend(new_x)
            self.list_y_antigo.extend(new_y)
        
        if contador % 25 == 0:
            fig.multi_plott(self.list_x_antigo, self.list_y_antigo, "SLAM", 'b', self.node_X, self.node_Y, "Node", 'r')
        
        contador += 1

navigation = Navigation()
