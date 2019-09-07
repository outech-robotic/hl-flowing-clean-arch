"""
Configuration module.
"""
from typing import Tuple

from attr import dataclass

from src.robot.entity.type import Radian, Millimeter
from src.robot.entity.vector import Vector2


@dataclass(frozen=True)
class Configuration:
    """
    Hold configuration parameters.
    """
    initial_position: Vector2
    initial_angle: Radian

    robot_width: float
    robot_length: float

    field_shape: Tuple[Millimeter, Millimeter]