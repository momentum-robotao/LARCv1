from __future__ import annotations

class Node:
    
    def __init__(self, position : list) -> None:
        self.nodes_adjacentes = []
        self.node_position = position
        self.node_visited = False
        self.node_father = None


    def set_node_adjacentes(self, node : Node) -> None:
        self.nodes_adjacentes.append(node)

    def set_node_position(self, position : list) -> None:
        self.node_position = position
    
    def get_node_position(self) -> list:
        return self.node_position
    
    def visit_node(self) -> None:
        self.node_visited = True
    
    def set_node_father(self, father : Node) -> None:
        self.node_father = father

    def calculate_min_node(self, node : Node) -> Node:
        find_min_node = False
        for n in self.nodes_adjacentes:
            min_distance = float("inf")
            if n.node_visited:
                distance = ((n.node_position[0] - node.node_position[0])**2 + (n.node_position[1] - node.node_position[1])**2)**0.5
                if distance < min_distance:
                    min_distance = distance
                    min_node = n
                    find_min_node = True
        if not find_min_node : min_node = None
        return min_node
        
