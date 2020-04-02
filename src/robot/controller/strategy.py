"""
Strategy module
"""
from src.logger import LOGGER
from src.robot.controller.motion.motion import MotionController
from src.robot.controller.pathfinding.dirty_pythfinding import DirtyPythfindingController
from src.util.geometry.vector import Vector2

SHAPE = (Vector2(1511, 300), Vector2(1511, 0), Vector2(2089, 0),
         Vector2(2089, 150), Vector2(2111, 150), Vector2(2111, 0),
         Vector2(3000, 0), Vector2(3000, 2000), Vector2(0, 2000), Vector2(0, 0),
         Vector2(589, 0), Vector2(589, 150), Vector2(611, 150), Vector2(611, 0),
         Vector2(1489, 0), Vector2(1489, 300))

PATH = [
    (Vector2(200, 795), False),
    (Vector2(375, 795), False),
    (Vector2(375, 795), False),
    (Vector2(375, 1200), True),
    (Vector2(375, 400), False),
    (Vector2(375, 120), False),
    (Vector2(375, 210), True),
    (Vector2(375, 210), False),
    (Vector2(1100, 210), False),
    (Vector2(850, 210), True),
    (Vector2(850, 210), False),
    (Vector2(850, 120), False),
    (Vector2(850, 210), True),
    (Vector2(850, 210), False),
    (Vector2(1046, 852), True),
    (Vector2(1046, 852), False),
    (Vector2(210, 800), False),
    (Vector2(210, 800), False),
    (Vector2(210, 700), False),
    (Vector2(210, 1600), True),
    (Vector2(210, 1600), False),
    (Vector2(120, 1600), False),
    (Vector2(210, 1600), True),
    (Vector2(210, 1600), False),
    (Vector2(210, 1270), False),
    (Vector2(210, 1360), True),
    (Vector2(210, 1360), False),
    (Vector2(1200, 1200), True),
    (Vector2(1200, 1200), False),
    (Vector2(1800, 1500), True),
    (Vector2(1800, 1500), False),
    (Vector2(1800, 1880), True),
    (Vector2(1800, 1600), False),
    (Vector2(1800, 1600), False),
    (Vector2(1800, 1720), False),
    (Vector2(1800, 1450), True),
    (Vector2(1800, 1450), False),
    (Vector2(300, 1450), True),
]


class StrategyController:
    """
    The strategy controller holds the high level algorithm executed by the robot.
    """

    def __init__(self, motion_controller: MotionController,
                 pathfinding_controller: DirtyPythfindingController):
        self.motion_controller = motion_controller
        self.pathfinding_controller = pathfinding_controller

    async def run(self) -> None:
        """
        Run the strategy.
        """
        self.pathfinding_controller.init_permanent_obstacles(SHAPE)
        for vec, reverse in PATH:
            LOGGER.get().info("move robot", destination=vec)
            await self.motion_controller.move_to(Vector2(vec.x, 2000 - vec.y),
                                                 reverse)

        LOGGER.get().info("Strategy algorithm finished running")  # lol
