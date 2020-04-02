"""
Simple pathfinding implementation
"""
from typing import Tuple, List
from dataclasses import dataclass
from math import pi, sqrt, cos, sin

from src.util.geometry.segment import Segment
from src.util.geometry.vector import Vector2
from src.util.geometry.intersection import segment_segment_intersection
from src.robot.entity.configuration import Configuration
from src.robot.controller.pathfinding import PathfindingController


@dataclass
class Node:
    """
    Graph node
    """
    position: Vector2
    neighbors: List


@dataclass
class State:
    """
    Contains the state of the pathfinding controller state
    """
    nodes: List[Node]
    permanent_obstacles: Tuple[Vector2, ...]
    temporary_obstacles: Tuple[Vector2, ...]


class DirtyPythfindingController(PathfindingController):
    """
    Dirty implementation of a pathfinding using custom node and vertice implementation
    """

    def __init__(self, configuration: Configuration):
        """
        Initialize state
        """
        self._state = State(nodes=[],
                            permanent_obstacles=Tuple[Vector2, ...],
                            temporary_obstacles=Tuple[Vector2, ...])
        self.margin = sqrt((configuration.robot_length / 2)**2 +
                           (configuration.robot_width / 2)**2)
        self.node_gap = 50

    def init_permanent_obstacles(self, shape: Tuple[Vector2, ...]) -> None:
        """
        Initialize static graph form the main shape
        """
        self._state.permanent_obstacles = shape
        self._state.nodes.clear()

        # Place nodes
        for index in range(0, len(shape) - 1):
            segment = Segment(shape[index], shape[index + 1])
            carrier_vec = segment.end - segment.start
            angle = carrier_vec.to_angle()
            origin = Vector2(self.margin * cos(angle + pi / 2),
                             self.margin * sin(angle + pi / 2))
            radius = 0
            while radius < carrier_vec.euclidean_norm():
                node_position = origin + Vector2(radius * cos(angle),
                                                 radius * sin(angle))
                radius += self.node_gap
                self._state.nodes.append(Node(node_position, []))

        # Place vertices
        for i in range(len(self._state.nodes)):
            f_node = self._state.nodes[i]
            for j in range(i + 1, len(self._state.nodes)):
                s_node = self._state.nodes[j]
                carrier = Segment(f_node.position, s_node.position)
                for index in range(0, len(shape) - 1):
                    segment = Segment(shape[index], shape[index + 1])
                    if not segment_segment_intersection(carrier, segment):
                        f_node.neighbors.append(s_node)
                        s_node.neighbors.append(f_node)

    def update_temporary_obstacles(self, positions: Tuple[Vector2,
                                                          ...]) -> None:
        """
        Update temporary obstacles
        """
        self._state.temporary_obstacles = positions

    def find_path(self, start_pos: Vector2,
                  aim_position: Vector2) -> List[Vector2]:
        """Compute a path between two positions"""
