"""
Simple pathfinding implementation
"""
from typing import Tuple, List

from src.util.geometry.segment import Segment
from src.util.geometry.vector import Vector2
from src.robot.controller.pathfinding import PathfindingController


class DummyPythfindingController(PathfindingController):

    def init_permanent_obstacles(self, shape: Tuple[Segment]) -> None:
        """Initialize permanent obstacle from a shape"""
        # TODO : initialize graph (and store obstacle ?)

    def update_temporary_obstacles(self, positions: Tuple[Vector2]) -> None:
        """Update temporary obstacles"""
        # TODO : re-initialize graph

    def update_moving_obstacles(self, positions: Tuple[Vector2]) -> None:
        """Update moving obstacles positions"""
        # TODO : store obstacles

    def find_path(self, start_pos: Vector2, aim_position: Vector2) -> List[Vector2]:
        """Compute a path between two positions"""
        # TODO : implement [djikstra/a*] algorithm
