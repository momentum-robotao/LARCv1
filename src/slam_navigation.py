from slam import slam
from types_and_constants import ROBOT_RADIUS, THRESHOLD_SLAM
from figure import fig
import numpy as np
from Node import Node

contador = 0


def slam_snapshot(gps_position: list, side_angle_to_distance_mapper: dict[float, float], robot_orientation: float) -> None:
    slam.take_snapshot(gps_position, side_angle_to_distance_mapper, robot_orientation)

def get_slam_list_atual() -> list:
    return slam.get_list_atual()

def get_slam_list() -> list:
    return slam.get_list()



class Navigation:
    def __init__(self) -> None:
        self.list_x_total = []
        self.list_y_total = []

        self.existing_nodes = set()
        self.current_node = None
        self.stack_visited_node = []

    def set_current_node(self, gps_position : list) -> None:
        condicao = False
        for node in self.existing_nodes:
            if node.get_node_position() == gps_position: # Não é o igual-igual pois tem que corrigir a imprecisão- METODO DO CIRUCLO DE RAIO
                self.current_node = node
                condicao = True
        if not condicao : self.current_node = Node(gps_position)
        
        

    
    def get_node(self, listx: list, listy: list, robot_position: list) -> None:
        x0, y0 = robot_position
        radius_offset = ROBOT_RADIUS + THRESHOLD_SLAM
        for lx, ly in zip(listx, listy):
            
            theta = np.arctan2(y0 - ly, lx - x0)
            px = lx - radius_offset * np.cos(theta)
            py = ly + radius_offset * np.sin(theta)
            
            # Adiciona somente se não estiver muito próximo de outro node e não sobreponha pontos existentes
            if all(
                (px - nx) ** 2 + (py - ny) ** 2 > radius_offset ** 2 for nx, ny in self.existing_nodes
            ) and all(
                (px - wx) ** 2 + (py - wy) ** 2 > ROBOT_RADIUS ** 2 for wx, wy in zip(listx, listy)
            ):  
                new_node = Node([px, py])
                self.existing_nodes.add(new_node)
                self.current_node.set_node_adjacentes(new_node)
        
            for node in self.existing_nodes:
                nx = node.get_node_position()[0]
                ny = node.get_node_position()[1]
                if ((px - nx) ** 2 + (py - ny) ** 2 < radius_offset ** 2):
                    self.current_node.set_node_adjacentes(node)
            




    def navigate(self, gps_position: list, side_angle_to_distance_mapper: dict[float, float], robot_orientation: float) -> None:
        global contador
        slam_snapshot(gps_position, side_angle_to_distance_mapper, robot_orientation) # Tirando O Snapshot
        new_x, new_y = get_slam_list_atual() # Pegar os pontos do snapshot
        
        self.get_node(new_x, new_y, gps_position) # Pegar os nodes

        #Adicionando os pontos do snapshot na lista total (SLAM)
        self.list_x_total.extend(new_x) 
        self.list_y_total.extend(new_y)
        

        if contador % 25 == 0:
            #print(f"LIST_X : {len(self.list_x_total)}, LIST_Y : {len(self.list_y_total)}, EXISTING_NODES : {len(self.existing_nodes)}") 
            fig.multi_plott(self.list_x_total, self.list_y_total, "SLAM", 'b', [node[0] for node in self.existing_nodes], [node[1] for node in self.existing_nodes], "Node", 'r')
            pass
      
        contador += 1

    def dfs(self, gps_position: list, side_angle_to_distance_mapper: dict[float, float], robot_orientation: float) -> list:
        '''  
        Primeiro de tudo o robo tira um snapshot, achando os pontos e nodes atuais. Depois disso, ele armazena esses pontos em uma classe Ponto 
        com atributos de sua coordenada, seu pai, se é visitado ou visto e seus filhos. Após isso, ele calcula ve qual o node mais proximo do robo 
        a partir da lista de nodes nodeX e nodeY, porém esse node tem que ser transversavel, ou seja, ele deve ter todos os filhos adjacentes visitados. Para isso, o robo tenta andar até o node mais próximo, e se ele conseguir, ele retorna o node, caso contrário, ele
        tenta o próximo node mais próximo. Ele vai fazendo até ele não conseguir ver mais nodes vistos transversaveis, aí o robo começa fazer a recursão, 
        no qual a cada recursão ele verifica novamente se é transversável até o ponto visto mais próximo (Tente ter uma lista de nodes visitados, e nodes vistos
        (a serem visitados))
        '''
        self.current_node = self.set_current_node(gps_position)
        self.current_node.visit_node()
        self.stack_visited_node.append(self.current_node)

        self.navigate(gps_position, side_angle_to_distance_mapper, robot_orientation)
        min_node = self.current_node.calculate_min_node(self.current_node)
        if min_node!= None : 
            #go_to_min_node
            pass
        else : 
            self.stack_visited_node.pop()
            #go to stack_visited_node[-1]
            pass
        pass






        
        
navigation = Navigation()
