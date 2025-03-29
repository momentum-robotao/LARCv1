from __future__ import annotations
import numpy as np

class Node:
    def __init__(self, position: list, father: Node) -> None:
        if father is not None : 
            self.nodes_adjacentes = [father]
        else : 
            self.nodes_adjacentes = []
        self.node_position = position
        self.node_visited = False
        self.node_father = father
    
    def __str__(self):
        return f"Node: \n\t x = {self.node_position[0]}, y = {self.node_position[1]} \n\t size_adj = {len(self.nodes_adjacentes)}"

    def __hash__(self):
        # Se a posição for None, retorna um hash fixo; caso contrário, usa a tupla da posição
        return hash(tuple(self.node_position)) if self.node_position is not None else 0

    def __eq__(self, other):
        if not isinstance(other, Node):
            return False
        return self.node_position == other.node_position

    def set_node_adjacentes(self, node: Node) -> None:
        if node is not None : 
            self.nodes_adjacentes.append(node)

    def get_node_position(self) -> list:
        return self.node_position

    def visit_node(self) -> None:
        self.node_visited = True

    def set_node_father(self, father: Node) -> None:
        self.node_father = father

    def get_node_father(self) -> Node:
        return self.node_father


    def calculate_min_node(self, current: Node) -> Node:
        min_distance = float('inf')
        min_node = None
        for n in self.nodes_adjacentes:
            pos = n.get_node_position()
            current_pos = current.get_node_position()
            if pos is None or current_pos is None:
                continue
            if not n.node_visited:
                distance = np.hypot(pos[0] - current_pos[0], pos[1] - current_pos[1])
                if distance < min_distance:
                    min_distance = distance
                    min_node = n
        return min_node

