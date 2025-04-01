from slam import slam
from types_and_constants import ROBOT_RADIUS, THRESHOLD_SLAM
from figure import fig
import numpy as np
from Node import Node
from robot import Robot
from maze import Maze
from types_and_constants import MAX_SPEED


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
        for node in self.existing_nodes:
            node_pos = node.get_node_position()
            distance = np.hypot(node_pos[0]-gps_position[0], node_pos[1]-gps_position[1])
            # Aumente a tolerância para reconhecer nodes próximos
            if distance <= THRESHOLD_SLAM * 1.5:  # 50% a mais de tolerância
                self.current_node = node
                return
        # Cria novo node somente se realmente não existir
        self.current_node = Node(gps_position, None)
        self.existing_nodes.add(self.current_node)
        
    
    def move_to_node(self, node_position: list, robot_start_position : list, start_orientation : float, robot: Robot, direction: str) -> bool:
        """Move o robô para um nó e retorna True se o movimento foi bem sucedido"""
        start_pos = robot_start_position
        dx = node_position[0] - start_pos[0]
        dy = node_position[1] - start_pos[1]
        target_distance = np.hypot(dx, dy)

        if target_distance <= THRESHOLD_SLAM:
            print("Já está próximo do node. Ignorando movimento.")
            return True
        
        if direction == "backward":
            # Novo cálculo de ângulo para movimento reverso
            dx_parent = node_position[0] - start_pos[0]
            dy_parent = node_position[1] - start_pos[1]
            
            target_angle = np.arctan2(-dy_parent, dx_parent)  # Ângulo inverso
            if target_angle < 0:
                target_angle += 2 * np.pi
            
            current_angle = robot.imu.get_rotation_angle()
            theta = (target_angle - current_angle + np.pi) % (2 * np.pi) - np.pi
            
            # Gira antes de mover para trás
            robot.rotate_to_angle(-theta)
            robot.motor.set_velocity(-0.5 * MAX_SPEED, -0.5 * MAX_SPEED)
        else:

            # Corrige arctan2 para seu sistema de coordenadas (y cresce para baixo, ângulo horário)
            target_angle = np.arctan2(-dy, dx)
            if target_angle < 0:
                target_angle += 2 * np.pi - start_orientation

            current_angle = robot.imu.get_rotation_angle()

            # Diferença angular, normalizada para [-π, π]
            theta = (target_angle - current_angle + np.pi) % (2 * np.pi) - np.pi + current_angle

            robot.rotate_to_angle(-theta)
            robot.motor.set_velocity(0.8 * MAX_SPEED, 0.8 * MAX_SPEED)

        previous_distance = float('inf')  # Inicializa com um valor infinito
        while True:
            robot.step()
            current_pos = robot.gps.get_position_AUGUSTO()
            remaining_distance = np.hypot(node_position[0] - current_pos[0],
                                        node_position[1] - current_pos[1])
            
            # Condição de parada aprimorada
            if remaining_distance <= THRESHOLD_SLAM or remaining_distance > previous_distance:
                break
            previous_distance = remaining_distance

        robot.motor.stop()
        return remaining_distance <= THRESHOLD_SLAM  # Retorna bool real
    

    # Identifica e adiciona novos nodes com base nos pontos do SLAM
    def get_node(self, listx: list, listy: list, robot_position: list) -> None:
        x0, y0 = robot_position
        radius_offset = ROBOT_RADIUS + THRESHOLD_SLAM/4# Raio de tolerância para criação de node
        for lx, ly in zip(listx, listy):
            # Calcula a posição ajustada do node com base no raio de tolerância
            theta = np.arctan2(y0 - ly, lx - x0)
            px = lx - radius_offset * np.cos(theta)
            py = ly + radius_offset * np.sin(theta)

            # Adiciona o node somente se não estiver muito próximo de outros nós ou pontos existentes
            if all((px - node.get_node_position()[0]) ** 2 + (py - node.get_node_position()[1]) ** 2 > radius_offset ** 2 for node in self.existing_nodes) and all((px - wx) ** 2 + (py - wy) ** 2 > radius_offset ** 2 for wx, wy in zip(listx, listy)):         
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
        print("entrei navigate")
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
    def slam_dfs(self, gps_position: list, side_angle_to_distance_mapper: dict[float, float], 
                robot_orientation: float, robot: Robot, maze: Maze, current_node: Node = None) -> list:
        
        if current_node is None: self.set_current_node(gps_position)
        else: self.current_node = current_node
        
        if not self.current_node.node_visited:
            self.current_node.visit_node()
            self.stack_visited_node.append(self.current_node)
            self.navigate(gps_position, side_angle_to_distance_mapper, robot_orientation)
            
            print(f"Current node: {self.current_node}")
            print("Adjacents:")
            for adjacent in self.current_node.nodes_adjacentes:
                print(f"Position: {adjacent.get_node_position()}")

        min_node = self.current_node.calculate_min_node(self.current_node)

        if min_node is not None:
            distance_to_min_node = np.hypot(
            min_node.get_node_position()[0] - self.current_node.get_node_position()[0],
            min_node.get_node_position()[1] - self.current_node.get_node_position()[1]
            )
            print(f"min_node : {min_node.get_node_position()}")
        
        if min_node is not None:
            print(f"Moving to next node: {min_node.node_position}")
            success = self.move_to_node(min_node.get_node_position(), robot.gps.get_position_AUGUSTO(), robot_orientation, robot, 'forward')
            if success:
                self.current_node = min_node
                return self.slam_dfs(robot.gps.get_position_AUGUSTO(), 
                                robot.lidar.get_distances_by_side_angle_AUGUSTO(), 
                                robot.imu.get_rotation_angle(), robot, maze)
        
        # Backtracking
        if len(self.stack_visited_node) > 1:
            print("Iniciando backtracking...")
            current_node = self.stack_visited_node.pop()  # Remove o nó atual
            parent_node = self.stack_visited_node[-1]  # Pega o nó pai

            print(f"Retornando para nó anterior: {parent_node.get_node_position()}")
            
            # Calcula a direção correta para o pai
            success = self.move_to_node(
                parent_node.get_node_position(), 
                robot.gps.get_position_AUGUSTO(), 
                robot.imu.get_rotation_angle(),  # Usa o ângulo ATUAL
                robot, 
                'backward'
            )
            
            if success:
                self.current_node = parent_node
                parent_node.visit_node()  # Marca o pai como revisitado
                return self.slam_dfs(robot.gps.get_position_AUGUSTO(), 
                                robot.lidar.get_distances_by_side_angle_AUGUSTO(), 
                                robot.imu.get_rotation_angle(), robot, maze, parent_node)
            else:
                print("Falha no backtracking! Abortando...")
                return []
        
        print("Exploração concluída")
        return []

# Instância da classe Navigation
navigation = Navigation()
