"""Default mock simulation; real hardware uses direct STM32 USB serial."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def select_hardware(context):
    share = get_package_share_directory('minesweeper_pi')
    mock = LaunchConfiguration('mock').perform(context).lower() == 'true'
    gpio = LaunchConfiguration('use_gpio').perform(context).lower() == 'true'
    if mock:
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'legacy_pi.launch.py')),
            launch_arguments={'mock': 'true', 'use_gpio': str(gpio).lower(),
                              'camera': LaunchConfiguration('camera'),
                              'config': LaunchConfiguration('config')}.items())]
    if gpio:
        raise RuntimeError('STM32 deployment owns motor outputs; use_gpio is only available in mock/legacy launch')
    return [
        Node(package='minesweeper_pi', executable='stm32_bridge', name='stm32_bridge',
             output='screen', parameters=[{'port': LaunchConfiguration('serial_port'),
                                           'baud': ParameterValue(LaunchConfiguration('baud'), value_type=int)}]),
        Node(package='minesweeper_pi', executable='camera', name='camera', output='screen',
             condition=IfCondition(LaunchConfiguration('camera')),
             parameters=[LaunchConfiguration('config'), {'mock': False}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='true'),
        DeclareLaunchArgument('use_gpio', default_value='false', description='Mock GPIO only; STM32 owns real outputs'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0',
                              description='STM32 application CDC port; prefer /dev/serial/by-id/...'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('config', default_value=get_package_share_directory('minesweeper_pi')+'/config/pi.yaml'),
        OpaqueFunction(function=select_hardware),
    ])
