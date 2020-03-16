"""
Simulation gateway module.
"""
import math

from src.robot.adapter.can import CANAdapter
from src.robot.adapter.lidar.simulated import SimulatedLIDARAdapter
from src.simulation.controller.probe import SimulationProbe
from src.simulation.entity.simulation_configuration import SimulationConfiguration
from src.util import can_id
from src.util.encoding import packet
from src.util.geometry.intersection import ray_segments_intersection
from src.util.geometry.ray import Ray
from src.util.geometry.vector import Vector2

LIDAR_RADIUS = 400


class SimulationGateway:
    """
    This is the gateway from the simulation world to the "real" world. 
    This gateway communicates with the sensors of the robot (and thus the handlers of the robot).
    """

    def __init__(self, simulation_configuration: SimulationConfiguration,
                 can_adapter: CANAdapter, lidar_adapter: SimulatedLIDARAdapter,
                 simulation_probe: SimulationProbe):
        self.simulation_configuration = simulation_configuration
        self.can_adapter = can_adapter
        self.lidar_adapter = lidar_adapter
        self.simulation_probe = simulation_probe

    async def movement_done(self) -> None:
        """
        Send the "movement done" signal to the robot.
        """
        await self.can_adapter.send(
            can_id.PROPULSION_MOVEMENT_DONE,
            packet.encode_propulsion_movement_done(
                packet.PropulsionMovementDonePacket(blocked=False,)))

    async def encoder_position(self, left_tick: int, right_tick: int) -> None:
        """
        Send encoder positions.
        """
        await self.can_adapter.send(
            can_id.PROPULSION_ENCODER_POSITION,
            packet.encode_propulsion_encoder_position(
                packet.PropulsionEncoderPositionPacket(
                    left_tick=left_tick,
                    right_tick=right_tick,
                )))

    async def push_lidar_readings(self) -> None:
        """
        Simulate the LIDAR sending its readings to the robot.
        """
        current_angle = self.simulation_probe.probe()['angle']
        current_position = self.simulation_probe.probe()['position']

        points = []
        for angle in range(0, 360, 10):
            angle_radian = angle * math.pi / 180
            dist = self._get_lidar_ray_distance(current_angle + angle_radian,
                                                current_position)
            points.append((angle_radian, dist))

        self.lidar_adapter.push_simulated_readings(tuple(points))

    def _get_lidar_ray_distance(self, angle: float, position: Vector2) -> float:
        direction = Vector2(math.cos(angle), math.sin(angle))
        ray = Ray(origin=position, direction=direction)
        _, distance = ray_segments_intersection(
            ray, self.simulation_configuration.obstacles)
        return min(distance, LIDAR_RADIUS) or LIDAR_RADIUS
