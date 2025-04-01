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
        return hash((self.node_position[0], self.node_position[1])) if self.node_position else 0

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
        return min(
            (n for n in self.nodes_adjacentes if not n.node_visited),
            key=lambda n: np.hypot(n.node_position[0]-current.node_position[0],
                                n.node_position[1]-current.node_position[1]),
            default=None
        )

