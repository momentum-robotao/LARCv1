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
            if not n.node_visited and n.get_node_position() is not None:
                distance = ((n.get_node_position()[0] - current.get_node_position()[0]) ** 2 +
                            (n.get_node_position()[1] - current.get_node_position()[1]) ** 2) ** 0.5
                if distance < min_distance:
                    min_distance = distance
                    min_node = n
        return min_node

        
