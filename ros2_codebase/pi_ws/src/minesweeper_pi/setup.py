from glob import glob
from setuptools import find_packages, setup

package_name = 'minesweeper_pi'
setup(
    name=package_name, version='1.0.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    extras_require={'test': ['pytest']},
    maintainer='Minesweepers A', maintainer_email='maintainers@example.com',
    description='ROS 2 port of Minesweepers A', license='Proprietary',
    entry_points={'console_scripts': ['stm32_bridge = minesweeper_pi.stm32_bridge:main', 'arduino_bridge = minesweeper_pi.arduino_bridge:main', 'motor_controller = minesweeper_pi.motor_controller:main', 'imu = minesweeper_pi.imu:main', 'camera = minesweeper_pi.camera:main']},
)
