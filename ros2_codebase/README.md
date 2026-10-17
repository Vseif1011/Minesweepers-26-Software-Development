# Minesweepers A — ROS 2

The real controller is **STM32F411CEU6**, running
HAL/FreeRTOS firmware with direct USB serial. The Pi runs `stm32_bridge`;
`pi.launch.py mock:=false` selects that connection. The old
`arduino_bridge.py` is an ASCII Mega bridge retained for simulation and the
explicit `legacy_pi.launch.py` deployment. It is not the STM32 transport.

Start with the [ordered file/function reading guide](docs/READING_GUIDE.md).
See [STM32 deployment and current limits](docs/STM32_DEPLOYMENT.md) for build,
direct serial setup, native sensor topics and remaining motor channel assignments.
The separate [CubeIDE project](firmware/stm32f411/README.md) stays outside that
reading syllabus. Its commissioning profile rejects motor arming.

ROS 2 migration of the 2024 code in the parent repository. All new source lives
here; the original `catkin_ws` and `rpi_ws` are left intact, including existing
local edits. Built and tested on **ROS 2 Lyrical, Ubuntu 26.04, Python 3.14**.
Other ROS 2 distributions have not been tested.

```text
ros2_codebase/
├── pi_ws/src/minesweeper_pi/          # independent Pi colcon workspace
├── laptop_ws/src/minesweeper_laptop/  # independent laptop colcon workspace
├── interfaces/minesweeper_interfaces/ # shared STM32 ROS messages
├── firmware/                        # STM32 controller + retained Mega firmware
├── scripts/                          # repeatable builds and hardware-free checks
└── docs/                             # migration, interfaces, validation
```

The application workspaces do not depend on each other. Both workspaces depend
on the shared `interfaces/` underlay. Copy `interfaces/` plus `pi_ws` to the Pi,
and `interfaces/` plus `laptop_ws` to the laptop. No micro-ROS agent is required.
`--symlink-install` is for local development: rebuild
after copying source to another machine rather than copying `install/`.

## Build

Install ROS 2 and colcon first. On a machine with ROS 2 already installed:

```bash
source /opt/ros/lyrical/setup.bash
cd /path/to/Minesweepers_A/ros2_codebase
rosdep install --from-paths interfaces pi_ws/src laptop_ws/src --ignore-src -r -y
bash scripts/build.sh
```

`rosdep` must already have been initialized and updated. The core packages use
`rclpy`, standard ROS messages, TF2, NumPy, OpenCV, and the standard ROS 2 `joy`
driver. No PyTorch, YOLO, GPIO, I2C, camera, joystick, or Arduino is required for
the default mock launch. Pyserial is needed for the pseudo-terminal test:

```bash
sudo apt install python3-serial python3-pytest
```

For a standalone laptop build, build and source `interfaces/` first, then
enter `laptop_ws` and run `colcon build --symlink-install --executor sequential`.
The Pi package can build directly against ROS. Both application workspaces have
their own `build/`, `install/`, and `log/` directories. The sequential executor
also suits a memory-constrained Pi.

## Run without hardware

Start these in separate terminals, sourcing ROS 2 in every terminal.

Pi nodes:

```bash
source /opt/ros/lyrical/setup.bash
source pi_ws/install/local_setup.bash
ros2 launch minesweeper_pi pi.launch.py mock:=true
```

Laptop nodes:

```bash
source /opt/ros/lyrical/setup.bash
source interfaces/install/local_setup.bash
source laptop_ws/install/local_setup.bash
ros2 launch minesweeper_laptop laptop.launch.py mock:=true
```

Finite motion command, in a third terminal with the laptop workspace sourced:

```bash
ros2 run minesweeper_laptop command_demo --ros-args -p duration:=3.0
ros2 topic echo /odometry/filtered
```

The mock Pi integrates commands into encoder counts and IMU rotation, and sends
JPEG frames containing a red synthetic mine. Mock vision detects that image.
It does **not** run the trained neural network. These defaults never open GPIO,
I2C, serial, a camera device, or a physical joystick.

Inject metal detection to record mines while the command demo moves:

```bash
ros2 topic pub --rate 10 /mock/metal_detected std_msgs/msg/Bool '{data: true}'
```

Toggle the synthetic camera mine in another terminal:

```bash
ros2 topic pub --rate 10 /mock/camera_mine_visible std_msgs/msg/Bool '{data: false}'
```

Metal plus camera means surface; metal without camera means buried. Mines are
recorded by grid cell, and an existing surface classification is retained.
Stop the metal publisher to clear its flag with a false message if manually
testing: the mock hardware retains the last injected sensor setting.

```bash
ros2 service call /map/save std_srvs/srv/Trigger '{}'
ros2 service call /imu/reset std_srvs/srv/Trigger '{}'
```

The competition map covers **20 m × 20 m in 400 cells of 1 m × 1 m**. Detections
require consecutive confirming samples, fresh encoder-backed localization, and
valid camera input. Each changed cell is saved atomically, normally to
`~/.ros/minesweepers/mine_map.json`. Inspect `/mine_map` and `/mine_markers` in
RViz with fixed frame `map`. The grid is a semantic competition map, not a
navigation obstacle/cost map; unmarked cells are not verified clear. See
[competition mapping](docs/COMPETITION_MAPPING.md) for coordinates, configuration,
and the calibration needed for field accuracy.

## Repeat the automated checks

```bash
bash scripts/test.sh
```

This runs both workspaces' pytest suites through `colcon test`, then launches
the installed executables and checks the complete ROS graph without hardware.
It starts mock Pi nodes, laptop nodes, teleop, and an independent mock GPIO
driver; synthetic Joy messages provide the motion commands. It also tests the
real serial backend against a Linux pseudo terminal, without an Arduino.

Logs, a saved mine map, and `smoke_report.json` are written to `artifacts/`.
The smoke test uses ROS domain 87 and localhost discovery, so it cannot send
test commands to a robot in the usual domain. Override with
`MINESWEEPER_TEST_DOMAIN` if another local test already uses 87. Do not run two
copies of the smoke test in the same domain concurrently.

See [validation details](docs/VALIDATION.md) for the actual results and limits.

## Run on the Pi and laptop

Use the same ROS 2 distribution, `ROS_DOMAIN_ID`, and RMW implementation on
both machines. Enable multicast/ROS discovery on the connecting network, allow
the DDS traffic through the firewall, and synchronize machine clocks. Do not
set localhost discovery for this two-machine deployment. No ROS master,
`ROS_MASTER_URI`, hardcoded camera IP, or ROS 1 bridge is used.

For the STM32 connection, follow [the deployment guide](docs/STM32_DEPLOYMENT.md).
Build/source the shared interfaces and Pi package, then on Pi:

```bash
source interfaces/install/local_setup.bash
source pi_ws/install/local_setup.bash
ros2 launch minesweeper_pi pi.launch.py mock:=false camera:=false serial_port:=/dev/ttyACM0
```

On laptop, source `interfaces/install/local_setup.bash` and the laptop workspace,
then select `hardware:=stm32` in its launch. This consumes native paired encoder
samples, raw IMU gyro and valid detector samples. Add a camera when available.
The legacy teleop/demo `/cmd_vel` and `/controller` messages are not STM32 motor
commands. The new `stm32_teleop` path sends five signed PWM duties using the
Logitech stick, Z twist and base lever. It requires the new MDD10A motor firmware;
the last verified flashed image is still motor-disabled. See
[joystick and motor setup](docs/MOTOR_CONTROL_SETUP.md) for pin assignments,
provisional wheel groups, build/flash status and the preview/live launch commands.

The retained Arduino deployment is explicitly named `legacy_pi.launch.py`.
Its serial/GPIO/I2C parameters in `pi.yaml` do not configure the STM32. The
existing hardware-free mock launch and tests remain available.

On the laptop, install Ultralytics/PyTorch in a Python environment compatible
with the chosen ROS 2 installation; retain access to its system packages.
The optional dependencies are listed in
[requirements-vision.txt](laptop_ws/requirements-vision.txt). The original
`best.pt` is copied into the package and installed with it. No model download
is performed by mock mode. Then:

```bash
# On the laptop, after sourcing ROS and laptop_ws/install/local_setup.bash:
ros2 launch minesweeper_laptop laptop.launch.py hardware:=stm32 mock:=false
```

For legacy/mock joystick tests, select `teleop:=true joystick:=true`.
Check and configure button indices before use. Hold button 6 to
enable movement and gripper commands. Defaults preserve the old arm and magnet
command numbers, but standardized `Joy` indices replace the old `inputs`
event names. A disconnected or released controller stops movement. Avoid
running `command_demo` alongside teleop, since both publish `cmd_vel`.

The inherited encoder scales are 3800 left and 2255 right ticks/m. The retained
Mega firmware counts all quadrature transitions. STM32 channel assignments and
encoder scales still need confirmation. Measure both scales, wheel signs,
track width, and motor signs on the real robot before field use; these cannot
be validated without hardware. Track width is consistently 0.55 m in the port
instead of the conflicting values in the old motor and odometry code.

## Optional operator displays

```bash
sudo apt install python3-pygame python3-tk
ros2 launch minesweeper_laptop laptop.launch.py mock:=true gui:=true
ros2 run minesweeper_laptop covariance_gui
```

The Pygame map shows the robot, surface/buried cells, and both coordinate
tables. Press **S** to save the JSON map and a screenshot; closing the window
keeps the headless mapping node running. Initial corner and heading are set
by `start_x`, `start_y`, and `start_heading` in
[laptop.yaml](laptop_ws/src/minesweeper_laptop/config/laptop.yaml); coordinates
are meters and heading is radians. The configured start is `(0 m, 0 m)`, facing
right (+X), for a serpentine sweep: right across row 0, advance 1 m in +Y, left
across row 1, and repeat through row 19. This is the intended route; the mapper
does not command automatic motion. For the 20 m square, cell-center corners are
`(0.5, 0.5)`, `(19.5, 0.5)`, `(0.5, 19.5)`, `(19.5, 19.5)`. The view fits the
display; use the mouse wheel to scroll long mine tables.

The covariance GUI updates the running EKF through its ROS 2 parameter service.
You can also tune it directly:

```bash
ros2 param set /ekf odom_variances '[0.03, 0.03, 0.05, 0.04, 0.04]'
ros2 param set /ekf imu_variances '[0.1, 0.01]'
```

All variances must be positive and finite. Changes are in memory; edit YAML
to persist them. Other node parameters are startup configuration. Camera
overlays are always published on `/camera/annotated/compressed`; enable
`vision.display` in the YAML for a local OpenCV window. The line turns green
when a detected bounding box reaches `vision.line_y` (default 300 pixels).
It publishes `/camera/aligned` but does not autonomously actuate the gripper.

See [the interface and migration guide](docs/MIGRATION.md) for topic types,
firmware details, and mappings from every original component.

For the connected Logitech camera and a live YOLO window, see
[the tested live camera/model setup](docs/LIVE_YOLO_TEST.md). Run
`bash scripts/run_live_vision.sh` on the laptop after starting the Pi camera.
