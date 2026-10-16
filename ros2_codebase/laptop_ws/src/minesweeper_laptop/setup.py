from glob import glob
from setuptools import find_packages, setup

package_name = 'minesweeper_laptop'
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
    entry_points={'console_scripts': ['teleop = minesweeper_laptop.teleop:main', 'odometry = minesweeper_laptop.odometry:main', 'ekf = minesweeper_laptop.ekf:main', 'mapping = minesweeper_laptop.mapping:main', 'vision = minesweeper_laptop.vision:main', 'command_demo = minesweeper_laptop.command_demo:main', 'covariance_gui = minesweeper_laptop.covariance_gui:main', 'stm32_detector = minesweeper_laptop.stm32_detector:main']},
)
