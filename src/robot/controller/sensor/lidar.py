"""
Lidar controller module.
"""
from math import cos, sin
from typing import Tuple

from src.robot.controller.motion.localization import LocalizationController
from src.robot.entity.type import Millimeter, Radian
from src.simulation.controller.probe import SimulationProbe
from src.util.geometry.vector import Vector2


class LidarController:
    """Lidar controller is an controller for saving positions of obstacles."""

    def __init__(self, simulation_probe: SimulationProbe,
                 localization_controller: LocalizationController):
        self.seen_polar: Tuple[Tuple[Radian, Millimeter], ...] = ()
        self.seen_cartesian: Tuple[Vector2, ...] = ()
        self._localization_controller = localization_controller
        simulation_probe.attach("position_obstacles",
                                lambda: self.seen_cartesian)

    def set_detection(
            self, seen_polar: Tuple[Tuple[Radian, Millimeter], ...]) -> None:
        """ Set the detected obstacles to the desired value in the lidar controller. """
        current_position = self._localization_controller.get_position()
        current_angle = self._localization_controller.get_angle()
        self.seen_polar = seen_polar
        self.seen_cartesian = tuple(current_position + Vector2(
            radius * cos(current_angle + angle),
            radius * sin(current_angle + angle),
        ) for angle, radius in seen_polar)
