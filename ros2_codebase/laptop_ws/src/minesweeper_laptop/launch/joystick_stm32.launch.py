"""Dedicated Logitech-to-STM control. Preview by default; no legacy controllers."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='true', description='False enables the real command topic'),
        DeclareLaunchArgument('config', default_value=get_package_share_directory('minesweeper_laptop')+
                              '/config/stm32_joystick.yaml'),
        Node(package='joy', executable='joy_node', name='stm32_joy', output='screen',
             parameters=[{'device_name': 'Logitech Extreme 3D pro', 'autorepeat_rate': 20.0,
                          'deadzone': 0.0, 'sticky_buttons': False, 'coalesce_interval_ms': 20}],
             remappings=[('joy', '/stm32/joy')]),
        Node(package='minesweeper_laptop', executable='stm32_teleop', name='stm32_teleop', output='screen',
             parameters=[LaunchConfiguration('config'),
                         {'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool)}],
             remappings=[('joy', '/stm32/joy')]),
    ])
