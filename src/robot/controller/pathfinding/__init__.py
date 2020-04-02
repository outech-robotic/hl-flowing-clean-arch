"""
Pathfinding module
"""
from abc import ABC, abstractmethod
from typing import Tuple, List

from src.util.geometry.segment import Segment
from src.util.geometry.vector import Vector2


class PathfindingController(ABC):
    """Pathfinding controller is an interface for computing path"""

    @abstractmethod
    def init_permanent_obstacles(self, shape: Tuple[Segment]) -> None:
        """Initialize permanent obstacle from a shape"""

    @abstractmethod
    def update_temporary_obstacles(self, positions: Tuple[Vector2]) -> None:
        """Update temporary obstacles"""

    @abstractmethod
    def find_path(self, start_pos: Vector2,
                  aim_position: Vector2) -> List[Vector2]:
        """Compute a path between two positions"""
