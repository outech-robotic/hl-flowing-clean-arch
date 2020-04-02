"""
Simple pathfinding implementation
"""
from typing import Tuple, List
from dataclasses import dataclass

from src.util.geometry.segment import Segment
from src.util.geometry.vector import Vector2
from src.util.geometry.intersection import segment_segment_intersection
from src.robot.controller.pathfinding import PathfindingController


@dataclass
class Node:
    """
    Graph node
    """
    position: Vector2
    vertices: List[Vertice]


@dataclass
class Vertice:
    """
    Graph vertice
    """
    carrier: Segment
    f_node: Node
    s_node: Node


@dataclass
class State:
    """
    Contains the state of the pathfinding controller state
    """
    nodes: List[Node]
    vertices: List[Vertice]
    permanent_obstacles: Tuple[Segment]
    temporary_obstacles: Tuple[Vector2]


class DirtyPythfindingController(PathfindingController):

    def __init__(self):
        """
        Initialize nodes
        """
        nodes = List[Node]
        for x in range(-15, 15):
            for y in range(0, 20):
                nodes.append(Node(position=Vector2(x * 100, y * 100)))

        self._state = State(nodes=nodes)

    def init_permanent_obstacles(self, shape: Tuple[Segment]) -> None:
        """
        Initialize static graph form the main shape
        """
        self._state.permanent_obstacles = shape
        for i in range(len(self._state.nodes)):
            f_node = self._state.nodes[i]
            for j in range(i + 1, len(self._state.nodes)):
                s_node = self._state.nodes[j]
                carrier = Segment(f_node.position, s_node.position)
                for segment in shape:
                    if not segment_segment_intersection(carrier, segment):
                        vertice = Vertice(carrier=carrier,
                                          f_node=f_node,
                                          s_node=s_node)
                        f_node.vertices.append(vertice)
                        s_node.vertices.append(vertice)

    def update_temporary_obstacles(self, positions: Tuple[Vector2]) -> None:
        """
        Update temporary obstacles
        """
        self._state.temporary_obstacles = positions

    def find_path(self, start_pos: Vector2,
                  aim_position: Vector2) -> List[Vector2]:
        """Compute a path between two positions"""
        # TODO : implement [djikstra/a*] algorithm
