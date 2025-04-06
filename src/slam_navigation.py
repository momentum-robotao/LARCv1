from slam import slam
from types_and_constants import ROBOT_RADIUS, THRESHOLD_SLAM, TILE_SIZE
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
            
            target_angle = np.arctan2(-dy_parent, dx_parent)  - start_orientation # Ângulo inverso
            
            current_angle = robot.imu.get_rotation_angle()
            theta = target_angle + current_angle    
            
            # Gira antes de mover para trás
            robot.rotate_to_angle(-theta)
            robot.motor.set_velocity(0.5 * MAX_SPEED, 0.5 * MAX_SPEED)
        else:

            # Corrige arctan2 para seu sistema de coordenadas (y cresce para baixo, ângulo horário)
            target_angle = np.arctan2(-dy, dx) - start_orientation

            current_angle = robot.imu.get_rotation_angle()

            # Diferença angular, normalizada para [-π, π]
            theta = target_angle + current_angle

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
    
    def is_Transversable(self, initial_point: tuple, final_point: tuple) -> bool:
        px, py = initial_point
        nx, ny = final_point

        dx = nx - px
        dy = ny - py
        distance = np.hypot(dx, dy)

        if distance < ROBOT_RADIUS * 0.1:
            return True

        step = ROBOT_RADIUS * 1.5
        steps = int(distance / step) + 1

        # Combina todos os obstáculos (SLAM + paredes)
        all_obstacles = list(zip(self.list_x_total, self.list_y_total))

        for i in range(1, steps + 1):
            x = px + dx * (i / steps)
            y = py + dy * (i / steps)

            # Verifica colisão com margem de segurança
            for ox, oy in all_obstacles:
                if np.hypot(x - ox, y - oy) < ROBOT_RADIUS * 1.2:
                    return False

        return True

    def create_wall(self, robot: Robot):
        # Obtem a posição atual do robô e define metade do tamanho do tile
        initial_x, initial_y = self.current_node.get_node_position()
        half_tile = TILE_SIZE / 2
        print(f"Orientatation : {robot.imu.get_rotation_angle()}")
        print(f"start_angle : {slam.start_angle}")
        global_orientation = abs(robot.imu.get_rotation_angle() - slam.start_angle)
        print(f"global_orientation : {global_orientation}")
        THRESHOLD = 2*np.pi/15
    

        # Cria a parede com base na orientação global
        if 0 - THRESHOLD <= global_orientation <= 0 + THRESHOLD or 2 * np.pi - THRESHOLD <= global_orientation <= 2 * np.pi + THRESHOLD:  # Leste
            wall_x = initial_x - half_tile
            wall_start_y = initial_y - half_tile
            wall_end_y = initial_y + half_tile
            for y in np.linspace(wall_start_y, wall_end_y, 50):
                self.list_x_total.append(wall_x)
                self.list_y_total.append(y)
        elif np.pi / 2 - THRESHOLD <= global_orientation <= np.pi / 2 + THRESHOLD:  # Sul
            wall_y = initial_y - half_tile
            wall_start_x = initial_x - half_tile
            wall_end_x = initial_x + half_tile
            for x in np.linspace(wall_start_x, wall_end_x, 50):
                self.list_x_total.append(x)
                self.list_y_total.append(wall_y)
        elif np.pi - THRESHOLD <= global_orientation <= np.pi + THRESHOLD:  # Oeste
            wall_x = initial_x + half_tile
            wall_start_y = initial_y - half_tile
            wall_end_y = initial_y + half_tile
            for y in np.linspace(wall_start_y, wall_end_y, 50):
                self.list_x_total.append(wall_x)
                self.list_y_total.append(y)
        elif 3 * np.pi / 2 - THRESHOLD <= global_orientation <= 3 * np.pi / 2 + THRESHOLD:  # Norte
            wall_y = initial_y + half_tile
            wall_start_x = initial_x - half_tile
            wall_end_x = initial_x + half_tile
            for x in np.linspace(wall_start_x, wall_end_x, 50):
                self.list_x_total.append(x)
                self.list_y_total.append(wall_y)


    # Identifica e adiciona novos nodes com base nos pontos do SLAM
    def get_node(self, listx: list, listy: list, robot_position: list) -> None:
        x0, y0 = robot_position
        radius_offset = ROBOT_RADIUS * 2  # Aumente conforme necessário

        for lx, ly in zip(listx, listy):
            # Direção do obstáculo para o robô (considerando Y para baixo)
            dx = x0 - lx  # X do robô - X do obstáculo
            dy = y0 - ly  # Y do robô - Y do obstáculo

            theta = np.arctan2(dy, dx)  # Ângulo do obstáculo para o robô
            px = lx + radius_offset * np.cos(theta)  # Posiciona o nó NA DIREÇÃO DO ROBÔ
            py = ly + radius_offset * np.sin(theta)

            # --- Verificação de Proximidade ---
            # Verifica distância mínima de outros nós
            too_close_to_nodes = any(
                np.hypot(px - node.get_node_position()[0], py - node.get_node_position()[1]) < radius_offset
                for node in self.existing_nodes
            )

            # Verifica distância mínima de obstáculos (SLAM + paredes virtuais)
            all_obstacles_x = listx + self.list_x_total  # Combina obstáculos atuais e paredes
            all_obstacles_y = listy + self.list_y_total
            too_close_to_obstacles = any(
                np.hypot(px - ox, py - oy) < radius_offset * 0.8  # Margem de segurança
                for ox, oy in zip(all_obstacles_x, all_obstacles_y)
            )

            # --- Criação do Nó ---
            if not too_close_to_nodes and not too_close_to_obstacles:
                if self.is_Transversable((x0, y0), (px, py)):
                    new_node = Node([px, py], self.current_node)
                    self.existing_nodes.add(new_node)
                    self.current_node.set_node_adjacentes(new_node)
                    new_node.set_node_adjacentes(self.current_node)
                    print(f"Node criado em ({px:.2f}, {py:.2f})")


            

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



        fig.multi_plott(
            self.list_x_total, self.list_y_total, "SLAM", 'b',
            [node.get_node_position()[0] for node in self.existing_nodes],
            [node.get_node_position()[1] for node in self.existing_nodes],
            "Node", 'r', gps_position, "Robot", 'y')

    # Implementação inicial de navegação DFS (Depth-First Search)
    def slam_dfs(self, gps_position: list, side_angle_to_distance_mapper: dict[float, float], 
                robot_orientation: float, robot: Robot, maze: Maze, current_node: Node = None) -> None:
        
        if current_node is None: 
            slam.atualizar_start_angle(robot_orientation)
            self.set_current_node(gps_position)
            self.create_wall(robot)
        else: self.current_node = current_node
        
        if not self.current_node.node_visited:
            self.current_node.visit_node()
            self.stack_visited_node.append(self.current_node)
            self.navigate(gps_position, side_angle_to_distance_mapper, robot_orientation)
            
            print(f"Current node: {self.current_node}")
            print("Adjacents:")
            for adjacent in self.current_node.nodes_adjacentes:
                if adjacent is not None : print(f"Position: {adjacent.get_node_position()}")

        min_node = self.current_node.calculate_min_node()

        
        if min_node is not None:
            distance_to_min_node = np.hypot(min_node.get_node_position()[0] - self.current_node.get_node_position()[0],min_node.get_node_position()[1] - self.current_node.get_node_position()[1])
            print(f"Moving to next node: {min_node.node_position}")
            print(f"distance_to_min_node: {distance_to_min_node}")
            success = self.move_to_node(min_node.get_node_position(), robot.gps.get_position_AUGUSTO(), robot_orientation, robot, 'forward')
            print(f"success : {success}")
            while not success :
                robot.step()
                success = self.move_to_node(min_node.get_node_position(), robot.gps.get_position_AUGUSTO(), robot_orientation, robot, 'forward')
            if success:
                return self.slam_dfs(robot.gps.get_position_AUGUSTO(), 
                                robot.lidar.get_distances_by_side_angle_AUGUSTO(), 
                                robot.imu.get_rotation_angle(), robot, maze, current_node=min_node)
        
        # Backtracking
        if self.current_node.node_father is not None:
            print("Iniciando backtracking...")
            parent_node = current_node.get_node_father()

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
                parent_node.visit_node()  # Marca o pai como revisitado
                return self.slam_dfs(robot.gps.get_position_AUGUSTO(), 
                                robot.lidar.get_distances_by_side_angle_AUGUSTO(), 
                                robot.imu.get_rotation_angle(), robot, maze, parent_node)
            else:
                print("Falha no backtracking! Abortando...")
                return None
        
        print("Exploração concluída")
        return None

# Instância da classe Navigation
navigation = Navigation()