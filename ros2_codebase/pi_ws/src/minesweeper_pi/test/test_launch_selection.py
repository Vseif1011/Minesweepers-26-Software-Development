"""Verify hardware selection without opening any physical device."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


@pytest.fixture
def pi_launch():
    path = Path(__file__).resolve().parents[1] / 'launch/pi.launch.py'
    spec = importlib.util.spec_from_file_location('pi_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context(mock, gpio='false'):
    result = LaunchContext()
    result.launch_configurations.update(mock=mock, use_gpio=gpio, camera='false',
                                        config='unused.yaml', serial_port='/dev/nonexistent', baud='115200')
    return result


def test_mock_selects_simulation_without_agent(pi_launch):
    actions = pi_launch.select_hardware(context('true'))
    assert len(actions) == 1 and isinstance(actions[0], IncludeLaunchDescription)


def test_real_selects_only_serial_bridge_and_camera(pi_launch):
    actions = pi_launch.select_hardware(context('false'))
    assert len(actions) == 2 and all(isinstance(action, Node) for action in actions)
    assert actions[0].node_package == 'minesweeper_pi'
    assert actions[0].node_executable == 'stm32_bridge'
    assert actions[1].node_package == 'minesweeper_pi'


def test_real_stm32_rejects_gpio_drive(pi_launch):
    with pytest.raises(RuntimeError, match='STM32 deployment owns motor outputs'):
        pi_launch.select_hardware(context('false', 'true'))
