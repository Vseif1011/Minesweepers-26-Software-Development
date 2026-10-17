from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = LaunchConfiguration('config')
    stm32 = PythonExpression(["'", LaunchConfiguration('hardware'), "' == 'stm32'"])
    stm32_teleop = PythonExpression(["'", LaunchConfiguration('teleop'), "' == 'true' and '",
                                    LaunchConfiguration('hardware'), "' == 'stm32'"])
    legacy_teleop = PythonExpression(["'", LaunchConfiguration('teleop'), "' == 'true' and '",
                                     LaunchConfiguration('hardware'), "' == 'legacy'"])
    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='true', description='Synthetic vision instead of YOLO'),
        DeclareLaunchArgument('hardware', default_value='legacy', choices=['legacy', 'stm32'],
                              description='legacy for simulation/Mega; stm32 for direct USB serial hardware'),
        DeclareLaunchArgument('teleop', default_value='false'),
        DeclareLaunchArgument('motor_dry_run', default_value='true', description='STM32 joystick preview unless false'),
        DeclareLaunchArgument('motor_config', default_value=get_package_share_directory('minesweeper_laptop')+
                              '/config/stm32_joystick.yaml'),
        DeclareLaunchArgument('joystick', default_value='false', description='Start the physical joystick driver'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('vision', default_value='true'),
        DeclareLaunchArgument('config', default_value=get_package_share_directory('minesweeper_laptop')+'/config/laptop.yaml'),
        Node(package='minesweeper_laptop', executable='odometry', name='odometry', output='screen',
             parameters=[config, {'encoder_source': LaunchConfiguration('hardware')}]),
        Node(package='minesweeper_laptop', executable='ekf', name='ekf', output='screen', parameters=[config],
             remappings=[('imu/data', PythonExpression(["'imu/data_raw' if '", LaunchConfiguration('hardware'),
                                                       "' == 'stm32' else 'imu/data'"]))]),
        Node(package='minesweeper_laptop', executable='stm32_detector', name='stm32_detector', output='screen',
             condition=IfCondition(stm32), parameters=[config]),
        Node(package='minesweeper_laptop', executable='mapping', name='mapping', output='screen',
             parameters=[config, {'show_gui': ParameterValue(LaunchConfiguration('gui'), value_type=bool)}]),
        Node(package='minesweeper_laptop', executable='vision', name='vision', output='screen',
             condition=IfCondition(LaunchConfiguration('vision')),
             parameters=[config, {'mock': ParameterValue(LaunchConfiguration('mock'), value_type=bool)}]),
        Node(package='minesweeper_laptop', executable='teleop', name='teleop', output='screen',
             condition=IfCondition(legacy_teleop), parameters=[config]),
        Node(package='minesweeper_laptop', executable='stm32_teleop', name='stm32_teleop', output='screen',
             condition=IfCondition(stm32_teleop), parameters=[LaunchConfiguration('motor_config'),
             {'dry_run': ParameterValue(LaunchConfiguration('motor_dry_run'), value_type=bool)}]),
        Node(package='joy', executable='joy_node', name='joy_node', output='screen',
             condition=IfCondition(LaunchConfiguration('joystick')), parameters=[{'autorepeat_rate': 20.0,
             'device_name': ParameterValue(PythonExpression(["'Logitech Extreme 3D pro' if '",
                  LaunchConfiguration('hardware'), "' == 'stm32' else ''"]), value_type=str),
             'deadzone': ParameterValue(PythonExpression(["0.0 if '", LaunchConfiguration('hardware'),
                  "' == 'stm32' else 0.05"]), value_type=float)}]),
    ])
