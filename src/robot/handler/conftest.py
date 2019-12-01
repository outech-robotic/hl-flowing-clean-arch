"""
Fixtures.
"""
from unittest.mock import MagicMock

from _pytest.fixtures import fixture

from src.robot.controller.localization import LocalizationController
from src.robot.controller.map import MapController
from src.robot.handler.motion import MotionHandler


@fixture(name='map_controller')
def map_controller_mock():
    """
    Map controller mock.
    """
    return MagicMock(spec=MapController)


@fixture(name='localization_controller')
def localization_controller_mock():
    """
    Localization controller mock.
    """
    return MagicMock(spec=LocalizationController)


@fixture
def motion_handler(localization_controller):
    """
    Motion handler.
    """
    return MotionHandler(localization_controller=localization_controller)
