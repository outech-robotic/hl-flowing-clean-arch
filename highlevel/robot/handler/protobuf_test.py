"""
Test for protobuf handler controller
"""
import pytest

from proto.gen.python.outech_pb2 import (EncoderPositionMsg, BusMessage,
                                         LaserSensorMsg, PressureSensorMsg,
                                         MovementEndedMsg)
from highlevel.robot.handler.protobuf import ProtobufHandler


@pytest.fixture(name='protobuf_handler')
def protobuf_handler_setup(match_action_controller_mock,
                           localization_controller_mock):
    """
    Setup a handler for testing.
    """
    return ProtobufHandler(
        match_action_controller=match_action_controller_mock,
        localization_controller=localization_controller_mock,
    )


@pytest.mark.asyncio
async def test_dispatch_encoder_position(protobuf_handler,
                                         localization_controller_mock):
    """
    Dispatch encoder position to localization controller.
    """
    bus_message = BusMessage(encoderPosition=EncoderPositionMsg(
        left_tick=1,
        right_tick=-2,
    ))
    msg_bytes = bus_message.SerializeToString()
    await protobuf_handler.translate_message(msg_bytes)

    localization_controller_mock.update_odometry_position.assert_called_once_with(
        1, -2)


@pytest.mark.asyncio
async def test_dispatch_laser_sensor(protobuf_handler,
                                     match_action_controller_mock):
    """
        If LaserSensorMsg provided, should call match_action_controller.set_laser_distances once.
    """
    bus_message = BusMessage(
        laserSensor=LaserSensorMsg(distance_front_left=10,
                                   distance_front_right=10,
                                   distance_back_left=10,
                                   distance_back_right=10))
    msg_bytes = bus_message.SerializeToString()
    await protobuf_handler.translate_message(msg_bytes)
    match_action_controller_mock.set_laser_distances.assert_called_once_with()


@pytest.mark.asyncio
async def test_dispatch_pressure_sensor(protobuf_handler,
                                        match_action_controller_mock):
    """
        If PressureSensorMsg provided, should call match_action_controller.set_pressures once.
    """
    bus_message = BusMessage(
        pressureSensor=PressureSensorMsg(on_left=10,
                                         on_center_left=10,
                                         on_center=10,
                                         on_center_right=10,
                                         on_right=10))
    msg_bytes = bus_message.SerializeToString()
    await protobuf_handler.translate_message(msg_bytes)
    match_action_controller_mock.set_pressures.assert_called_once_with()


@pytest.mark.parametrize('blocked', [True, False])
@pytest.mark.asyncio
async def test_movement_ended(protobuf_handler, localization_controller_mock,
                              blocked):
    """
    Route movement ended messages to localization controller.
    """
    bus_message = BusMessage(movementEnded=MovementEndedMsg(blocked=blocked))
    msg_bytes = bus_message.SerializeToString()
    await protobuf_handler.translate_message(msg_bytes)
    localization_controller_mock.movement_done.assert_called_once_with(blocked)


@pytest.mark.asyncio
async def test_dispatch_does_not_throw_exception(protobuf_handler):
    """
        If random bytes provided, should not throw exception.
    """
    msg_bytes = b'46C7S7B767'
    await protobuf_handler.translate_message(msg_bytes)
