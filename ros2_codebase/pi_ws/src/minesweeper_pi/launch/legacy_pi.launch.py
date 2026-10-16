from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mock = LaunchConfiguration('mock')
    gpio = LaunchConfiguration('use_gpio')
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='true'),
        DeclareLaunchArgument('use_gpio', default_value='false', description='Drive via Pi GPIO instead of Arduino'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('config', default_value=get_package_share_directory('minesweeper_pi')+'/config/pi.yaml'),
        Node(package='minesweeper_pi', executable='arduino_bridge', name='arduino_bridge', output='screen',
             parameters=[config, {'mock': ParameterValue(mock, value_type=bool),
                                  'drive_motors': ParameterValue(PythonExpression(["'", gpio, "'.lower() != 'true'"]), value_type=bool)}]),
        Node(package='minesweeper_pi', executable='motor_controller', name='motor_controller', output='screen',
             condition=IfCondition(gpio), parameters=[config, {'mock': ParameterValue(mock, value_type=bool)}]),
        Node(package='minesweeper_pi', executable='imu', name='imu', output='screen',
             parameters=[config, {'mock': ParameterValue(mock, value_type=bool)}]),
        Node(package='minesweeper_pi', executable='camera', name='camera', output='screen',
             condition=IfCondition(LaunchConfiguration('camera')),
             parameters=[config, {'mock': ParameterValue(mock, value_type=bool)}]),
    ])
