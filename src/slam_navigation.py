from slam import slam
from types_and_constants import ROBOT_RADIUS

def slam_snapshot(gps_position: list, side_angle_to_distance_mapper: dict[float, float], robot_orientation : float) -> None:
    slam.take_snapshot(gps_position, side_angle_to_distance_mapper, robot_orientation)
    pass

def get_slam_list() -> list:
    return slam.get_list()

def In_Circle(pointX: float, pointY: float, centerX : float, centerY : float, radius: float) -> bool:
    return (pointX - centerX)**2 + (pointY - centerY)**2 <= radius**2


''' 
A ideia é pegar os nodes, ou seja os pontos próximos a parede que ele pegar e fazer a navegação entre eles. Isso inclui optimizar o caminho, ou seja, pegar o caminho mais curto entre 
os nodes, não repetindo a verificação de um node já visitado e para pegar esse node. Adicionar esse node a uma lista de nodes visitados e com isso fazer um algortimo de busca para pegar o
caminho mais curto entre os nodes. 

'''
def get_node() :
    pass
class Navigation:
    def __init__(self) -> None:
        self.listx = []
        self.listy = []

    def navigate(self, gps_position: list, side_angle_to_distance_mapper: dict[float, float], robot_orientation : float) -> None:
        slam_snapshot(gps_position, side_angle_to_distance_mapper, robot_orientation)
        self.listx, self.listy = get_slam_list()
        pass

        


navigation = Navigation()
