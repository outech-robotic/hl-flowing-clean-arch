"""
Simulation module.
"""
import math

import structlog

from src.entity.configuration import Configuration
from src.entity.geometry import Segment, Ray
from src.entity.type import Millimeter, Radian
from src.entity.vector import Vector2
from src.gateway.motion import MotionGateway
from src.handler.distance_sensor import DistanceSensorHandler
from src.handler.motion import MotionHandler
from src.util.geometry import ray_segments_intersection, forward, backward, right, left
from src.util.periodic import periodic_callback

LOGGER = structlog.get_logger()

TRANSLATION_SPEED = 10
ROTATION_SPEED = 90 / 180 * math.pi
FPS = 60


class Simulation(MotionGateway):
    """
    The Matrix.

    A simulation emulates the input and the output of the robot.
    """
    def __init__(self, configuration: Configuration,
                 motion_handler: MotionHandler,
                 distance_sensor_handler: DistanceSensorHandler):
        self.position = configuration.initial_position
        self.angle = configuration.initial_direction
        self.obstacles = [
            Segment(start=Vector2(0, 0), end=Vector2(0, 200)),
            Segment(start=Vector2(0, 0), end=Vector2(300, 0)),
            Segment(start=Vector2(300, 200), end=Vector2(0, 200)),
            Segment(start=Vector2(300, 200), end=Vector2(300, 0)),
        ]

        self.motion_handler = motion_handler
        self.distance_sensor_handler = distance_sensor_handler

    def _feedback_loop(self, _) -> None:
        """
        Call the handlers of the robot to notify any change in the environment.
        """
        self.motion_handler.position_update(self.position.x, self.position.y,
                                            self.angle)
        _, dist = ray_segments_intersection(
            Ray(origin=self.position, direction=forward(self.angle)),
            self.obstacles)
        if dist:
            self.distance_sensor_handler.distance_forward(dist)

        _, dist = ray_segments_intersection(
            Ray(origin=self.position, direction=backward(self.angle)),
            self.obstacles)
        if dist:
            self.distance_sensor_handler.distance_backward(dist)

        _, dist = ray_segments_intersection(
            Ray(origin=self.position, direction=right(self.angle)),
            self.obstacles)
        if dist:
            self.distance_sensor_handler.distance_right(dist)

        _, dist = ray_segments_intersection(
            Ray(origin=self.position, direction=left(self.angle)),
            self.obstacles)
        if dist:
            self.distance_sensor_handler.distance_left(dist)

    def rotate(self, angle: Radian) -> None:
        """
        Receive a rotate order from the robot.
        """
        LOGGER.info("simulation_rotate", angle=angle * 180 / math.pi)

        def func(frame, cls):
            cls.angle += ROTATION_SPEED / FPS * math.copysign(1, angle)
            done = frame >= angle / ROTATION_SPEED * FPS
            if done:
                self.motion_handler.movement_done()
            return done

        periodic_callback(func, 1 / FPS, self)

    def move_forward(self, distance: Millimeter) -> None:
        """
        Receive a move forward order from the robot.
        """
        LOGGER.info("simulation_move_forward", distance=distance)

        def func(frame, cls):
            cls.position += forward(self.angle) * TRANSLATION_SPEED / FPS
            done = frame >= distance / TRANSLATION_SPEED * FPS
            if done:
                self.motion_handler.movement_done()
            return done

        periodic_callback(func, 1 / FPS, self)

    async def run(self) -> None:
        """
        Run the simulation.
        """
        periodic_callback(self._feedback_loop, 1 / 10)
