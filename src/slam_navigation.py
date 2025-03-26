from slam import slam
from types_and_constants import ROBOT_RADIUS, THRESHOLD_SLAM
from figure import fig
import numpy as np
from Node import Node
from robot import Robot
from maze import Maze
from types_and_constants import MAX_SPEED

contador_dfs = 0

def get_start_angle() -> float:
    return slam.get_start_angle()

# Função para capturar um snapshot do SLAM com a posição GPS, mapeador de ângulos para distâncias e orientação do robô
def slam_snapshot(gps_position: list, side_angle_to_distance_mapper: dict[float, float], robot_orientation: float) -> None:
    slam.take_snapshot(gps_position, side_angle_to_distance_mapper, robot_orientation)

# Função para obter a lista atual de pontos do SLAM
def get_slam_list_atual() -> list:
    return slam.get_list_atual()

# Função para obter a lista completa de pontos do SLAM
def get_slam_list() -> list:
    return slam.get_list()




# Classe principal de navegação
class Navigation:
    def __init__(self) -> None:
        self.list_x_total = []  # Lista para armazenar todas as coordenadas X do SLAM
        self.list_y_total = []  # Lista para armazenar todas as coordenadas Y do SLAM

        self.existing_nodes = set()  # Conjunto de nodes existentes
        self.current_node = Node(None, None)  # Node atual do robô
        self.stack_visited_node = []  # Pilha de nodes visitados para navegação DFS

    # Define o nó atual com base na posição GPS
    def set_current_node(self, gps_position: list) -> None:
        past_node = self.current_node
        condicao = False
        for node in self.existing_nodes:
            # Verifica se a posição GPS coincide com algum node existente (considerando imprecisão)
            if all(node_coord + THRESHOLD_SLAM > gps_coord and node_coord - THRESHOLD_SLAM < gps_coord 
                   for node_coord, gps_coord in zip(node.get_node_position(), gps_position)):
                self.current_node = node
                condicao = True
        if not condicao:  # Caso não encontre, cria um novo node
            self.current_node = Node(gps_position, past_node)
    
    def move_to_node(self, node_position: list, robot: Robot, direction: str) -> None:
        total_distance = np.hypot(node_position[0] - robot.gps.get_position_AUGUSTO()[0],node_position[1] - robot.gps.get_position_AUGUSTO()[1])
        theta = -robot.imu.get_rotation_angle() + np.arctan2(node_position[1] - robot.gps.get_position_AUGUSTO()[1],node_position[0] - robot.gps.get_position_AUGUSTO()[0])
        print(f"distance : {total_distance}, theta : {theta}")
        if direction == "backward":
            robot.move('backward', total_distance, high_speed = 0.5*MAX_SPEED)
            robot.motor.stop()
        else:
            robot.rotate_to_angle(theta)
            robot.motor.set_velocity(0.8*MAX_SPEED, 0.8*MAX_SPEED)
            dist = total_distance
            while(dist > 0.005):
                robot.step()
                dist = np.hypot(node_position[0] - robot.gps.get_position_AUGUSTO()[0],node_position[1] - robot.gps.get_position_AUGUSTO()[1])
            robot.motor.stop()

    # Identifica e adiciona novos nodes com base nos pontos do SLAM
    def get_node(self, listx: list, listy: list, robot_position: list) -> None:
        x0, y0 = robot_position
        radius_offset = ROBOT_RADIUS + THRESHOLD_SLAM# Raio de tolerância para criação de node
        for lx, ly in zip(listx, listy):
            # Calcula a posição ajustada do node com base no raio de tolerância
            theta = np.arctan2(y0 - ly, lx - x0)
            px = lx - radius_offset * np.cos(theta)
            py = ly + radius_offset * np.sin(theta)

            # Adiciona o node somente se não estiver muito próximo de outros nós ou pontos existentes
            if all(
                (px - node.get_node_position()[0]) ** 2 + (py - node.get_node_position()[1]) ** 2 > radius_offset ** 2 for node in self.existing_nodes
            ) and all(
                (px - wx) ** 2 + (py - wy) ** 2 > ROBOT_RADIUS ** 2 for wx, wy in zip(listx, listy)
            ):
                new_node = Node([px, py], self.current_node)  # Cria um novo node
                self.existing_nodes.add(new_node)  # Adiciona o novo node ao conjunto de node existentes
                self.current_node.set_node_adjacentes(new_node)  # Define o node como adjacente ao node atual

                # Verifica e conecta nodes adjacentes próximos
                for node in self.existing_nodes:
                    nx = node.get_node_position()[0]
                    ny = node.get_node_position()[1]
                    if ((px - nx) ** 2 + (py - ny) ** 2 < (radius_offset + THRESHOLD_SLAM) ** 2):
                        # Evita adicionar nodes duplicados na lista de adjacentes
                        if node not in self.current_node.nodes_adjacentes and node != self.current_node:
                            self.current_node.set_node_adjacentes(node)

    # Função principal de navegação ( ESTÀ COM UM NOME ESTRANHO, NÃO FAZ O QUE O NOME DIZ)
    def navigate(self, gps_position: list, side_angle_to_distance_mapper: dict[float, float], robot_orientation: float) -> None:
        global contador
        # Captura um snapshot do SLAM
        slam_snapshot(gps_position, side_angle_to_distance_mapper, robot_orientation)
        # Obtém os pontos do snapshot
        new_x, new_y = get_slam_list_atual()

        # Identifica e adiciona nodes com base nos pontos do snapshot
        self.get_node(new_x, new_y, gps_position)

        # Adiciona os pontos do snapshot às listas totais
        self.list_x_total.extend(new_x)
        self.list_y_total.extend(new_y)

        # Plota os pontos e nodes a cada 25 iterações
        fig.multi_plott(
            self.list_x_total, self.list_y_total, "SLAM", 'b',
            [node.get_node_position()[0] for node in self.existing_nodes],
            [node.get_node_position()[1] for node in self.existing_nodes],
            "Node", 'r', gps_position, "Robot", 'y')

    # Implementação inicial de navegação DFS (Depth-First Search)
    def slam_dfs(self, gps_position: list, side_angle_to_distance_mapper: dict[float, float], robot_orientation: float, robot: Robot, maze : Maze) -> list:
        '''
        O robô utiliza DFS para explorar o ambiente:
        1. Tira um snapshot do SLAM e identifica os nós.
        2. Calcula o nó mais próximo que seja transversável (todos os adjacentes visitados).
        3. Move-se para o nó mais próximo ou retorna ao nó anterior se não houver mais opções.
        '''

        self.set_current_node(gps_position)  # Define o node atual
        print(f"Current node: {self.current_node.node_position}")  # Printa a posição do node atual
        self.current_node.visit_node()  # Marca o node atual como visitado
        self.stack_visited_node.append(self.current_node)  # Adiciona o node à pilha de nodes_visitados
        self.navigate(gps_position, side_angle_to_distance_mapper, robot_orientation)  # Cria Pontos do SLAM e Nodes
        min_node = self.current_node.calculate_min_node(self.current_node)  # Calcula o node mais próximo
        print("posicao_atual X: ", gps_position[0], "posicao_atual Y: ", gps_position[1])
        adjacentes_positions = [adj_node.get_node_position() for adj_node in self.current_node.nodes_adjacentes]
        print(f"Adjacentes: {adjacentes_positions}")
        print("-----------------------------------------------")
        print(f"orientacao = {robot_orientation}")
        print(f"start angle = {get_start_angle()}")

        if min_node is not None:
            # Move-se ao nó mais próximo e chama a DFS novamente
            print(f"Next_node : {min_node.node_position}")
            self.move_to_node(min_node.get_node_position(), robot, 'forward')
            print(f"cheguei no node {min_node.node_position}")
            self.slam_dfs(robot.gps.get_position_AUGUSTO(), robot.lidar.get_distances_by_side_angle_AUGUSTO(), robot.imu.get_rotation_angle(), robot, maze)
        elif self.stack_visited_node:
            # Backtracking para o nó anterior
            print("ENTREI NO BACKTRAKING")
            self.stack_visited_node.pop()  # Remove o nó atual da pilha
            if self.stack_visited_node:  # Verifica se ainda há nós na pilha
                node = self.stack_visited_node[-1]  # Obtém o nó anterior   
                self.move_to_node(node.get_node_position(), robot, 'backward')  # Move-se ao nó anterior
                self.slam_dfs(robot.gps.get_position_AUGUSTO(), robot.lidar.get_distances_by_side_angle_AUGUSTO(), robot.imu.get_rotation_angle(), robot, maze)
        else:
            # Termina a DFS quando não há mais nós para visitar
            print("DFS concluída. Todos os nós foram visitados.")


# Instância da classe Navigation
navigation = Navigation()
