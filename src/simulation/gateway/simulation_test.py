"""
Test for simulation gateway module.
"""
import pytest

from src.simulation.gateway.simulation import SimulationGateway


@pytest.mark.asyncio
async def test_movement_done(can_adapter_mock, simulation_configuration_test,
                             simulated_lidar_adapter_mock,
                             simulation_probe_mock):
    """
    Should call movement_done on motion handler.
    """
    simulation_gateway = SimulationGateway(
        simulation_configuration=simulation_configuration_test,
        can_adapter=can_adapter_mock,
        lidar_adapter=simulated_lidar_adapter_mock,
        simulation_probe=simulation_probe_mock,
    )
    await simulation_gateway.movement_done()
    can_adapter_mock.send.assert_called_once()
