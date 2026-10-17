# Laptop ROS 2 workspace

Ament Python package (requires the shared `interfaces/` underlay): `minesweeper_laptop`.

```bash
source /opt/ros/lyrical/setup.bash
cd ../interfaces
colcon build --symlink-install --executor sequential
cd ../laptop_ws
source ../interfaces/install/local_setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --executor sequential
source install/local_setup.bash
ros2 launch minesweeper_laptop laptop.launch.py mock:=true
```

Executables: `teleop`, `odometry`, `ekf`, `mapping`, `vision`, `command_demo`,
`covariance_gui`, `stm32_detector`, `stm32_teleop`. Default launch runs localization, mapping
and synthetic-image vision. It needs the Pi workspace's mock or real sensor publishers to produce
motion and detections, but the Pi package is not a build dependency.

`teleop:=true joystick:=true` enables the gamepad; `gui:=true` opens Pygame.
To open only the map window, install `python3-pygame` and run:

```bash
ros2 run minesweeper_laptop mapping --ros-args -p show_gui:=true
```

Running `mapping` without that parameter starts a headless ROS node. It publishes
`/mine_map` and `/mine_markers` without opening a window. The standalone window
shows an empty grid until the localization and sensor nodes publish data on
`/pose_combined`, `/odom`, `/detection`, `/camera_detection`, and `/camera/ready`.
Defaults cover the 20 m × 20 m competition field in 400 one-metre squares.
See [competition mapping](../docs/COMPETITION_MAPPING.md) for confirmation,
coordinate conventions and the remaining physical calibration work.

Configure geometry, joystick buttons, starting map corner/heading and perception
in `src/minesweeper_laptop/config/laptop.yaml`. `mock:=false` switches vision to
the bundled original YOLO model; install the optional `requirements-vision.txt`
dependencies in a ROS-compatible Python environment first.

The parent [README](../README.md) has deployment, networking and test commands.

For the STM32F411CEU6, use `hardware:=stm32`. Odometry subscribes to native
paired 64-bit encoder counts, the EKF reads `/imu/data_raw`, and the detector
adapter requires fresh valid `/motor/status`. `mock` only selects the vision
backend. `teleop:=true` with this hardware selects `stm32_teleop`; the default
is preview mode. The dedicated joystick launch and MDD10A firmware profile use
the confirmed PWM/DIR wiring with provisional left/right motor groups. Legacy
teleop cannot move the STM32. See [motor setup](../docs/MOTOR_CONTROL_SETUP.md)
before enabling live commands; the new motor image is prepared but not flashed.
See [STM32 deployment](../docs/STM32_DEPLOYMENT.md) and the
[ordered reading guide](../docs/READING_GUIDE.md).
