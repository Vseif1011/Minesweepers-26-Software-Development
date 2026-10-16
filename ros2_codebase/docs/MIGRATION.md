# ROS 1 → ROS 2 migration

This document describes the retained Mega/mock migration. The current real
controller is STM32F411CEU6: see [STM32 deployment](STM32_DEPLOYMENT.md) for
its native topic contracts and [the reading guide](READING_GUIDE.md) for all
application files. `pi.launch.py mock:=false` now selects the Agent;
`legacy_pi.launch.py` explicitly selects the older hardware path described below.
STM32 native endpoints use absolute topic names and best-effort depth-1 QoS.
The identity IMU mounting TF described below belongs only to the legacy/mock
Pi IMU; the raw STM32 path requires a measured mounting transform.

## Component inventory

| Original code | ROS 2 replacement |
| --- | --- |
| `rpi_ws/src/motors/src/motor_controller.py` | `minesweeper_pi motor_controller`, optional Pi GPIO backend with mock support |
| `rpi_ws/src/mpu6050` C++/wiringPi | `minesweeper_pi imu`, Python/smbus2, Madgwick filter, stationary calibration, reset service |
| `rpi_ws/src/imu_msgs/Angle` | `/imu/rpy`, `geometry_msgs/Vector3Stamped` in radians |
| `mpu6050/ResetIMU` | `/imu/reset`, `std_srvs/Trigger` |
| Arduino `all_code_reformatted.ino` + rosserial | `firmware/minesweeper_mega` + `minesweeper_pi arduino_bridge` |
| `rpi_ws/stream.py` TCP camera stream | `minesweeper_pi camera`, JPEG compressed images over ROS 2 |
| `catkin_ws/src/joy/src/joystick.py` | standard ROS 2 `joy` driver + `minesweeper_laptop teleop` |
| `diffodom/scripts/diffodom.py` | `minesweeper_laptop odometry`, encoder-only differential odometry |
| `robot_ekf/src/kalman_dyn.py` | `minesweeper_laptop ekf`, five-state EKF with separate IMU correction |
| `robot_ekf/src/gui.py` | `minesweeper_laptop covariance_gui`, ROS parameter client instead of print-only sliders |
| `map/src/map.py`, `const.py`, `utils.py` | `minesweeper_laptop mapping`, headless grid + RViz + optional Pygame |
| `yolo.py`, `yolonode.py` status-file polling | `minesweeper_laptop vision`, direct image subscription and detection publisher |
| `yolo_with_line.py`, `testline.py` | vision overlay, vertical-flip option, alignment line and `/camera/aligned` |
| `PC.launch`, `RPI.launch`, `sammy.py`, `dawar.py` | Python ROS 2 launch files, with configuration arguments |
| Deprecated diffodom script / bundled `ros_lib` | superseded by the new odometry / standalone serial firmware |

The voltage/temperature scripts remain unchanged in the original `rpi_ws`:
they are operating-system utilities, not ROS nodes. The port does not add
autonomous navigation or automatic gripping; both were unfinished tasks in
the original README.

## Interfaces

Topics are relative in node source, so a coordinated namespace can be applied
to both machines. Defaults below are the root-namespace topic names.

| Topic | ROS 2 type | Publisher → consumer |
| --- | --- | --- |
| `/joy` | `sensor_msgs/Joy` | standard joystick driver → teleop |
| `/cmd_vel` | `geometry_msgs/Twist` | teleop or command demo → Pi output nodes |
| `/controller` | `std_msgs/Int16` | teleop → Arduino bridge |
| `/left_ticks`, `/right_ticks` | `std_msgs/Int32` | Arduino bridge → odometry |
| `/detection` | `std_msgs/Bool` | Arduino metal detector → mapping |
| `/motor_duty_cycle` | `std_msgs/Float64MultiArray` | bridge computed motor targets, FR/FL/RL/RR, signed percent |
| `/gpio_motor_duty_cycle` | `std_msgs/Float64MultiArray` | optional GPIO node targets, same order/units |
| `/imu/data` | `sensor_msgs/Imu` | MPU/mock → EKF |
| `/imu/rpy` | `geometry_msgs/Vector3Stamped` | IMU → diagnostics; x/y/z are roll/pitch/yaw radians |
| `/camera/image/compressed` | `sensor_msgs/CompressedImage` | Pi camera → laptop vision |
| `/camera/annotated/compressed` | `sensor_msgs/CompressedImage` | vision boxes and alignment overlay → display |
| `/camera_detection` | `std_msgs/Bool` | vision → mapping |
| `/camera/ready` | `std_msgs/Bool` | vision → mapping: fresh successfully processed image; false means unknown |
| `/camera/aligned` | `std_msgs/Bool` | vision → operator/integration |
| `/odom` | `nav_msgs/Odometry` | wheel odometry → EKF |
| `/robot_2d` | `geometry_msgs/PoseStamped` | wheel pose for diagnostics |
| `/encoder_dist` | `geometry_msgs/Vector3Stamped` | odometry; x=left delta, y=right delta, z=mean, meters |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF fused pose/twist/covariance |
| `/pose_combined` | `geometry_msgs/PoseStamped` | EKF → mapping |
| `/mine_pose` | `geometry_msgs/PoseStamped` | mapping: new/updated mine world position and detector heading |
| `/mine_map` | `nav_msgs/OccupancyGrid` | 20 × 20 semantic grid at 1 m: 0=unmarked, 50=surface, 100=buried |
| `/mine_markers` | `visualization_msgs/MarkerArray` | RViz mine positions and class colors |
| `/mock/metal_detected` | `std_msgs/Bool` | test injection → mock Arduino bridge only |
| `/mock/camera_mine_visible` | `std_msgs/Bool` | test injection → mock camera only |

Unlike the legacy `mine_pose`, the new pose's orientation is a quaternion,
not a numeric mine-class field. The grid and marker text/color carry classes.
`Pose2D`, removed from this installed ROS 2 distribution, is not used anywhere.
All distances are meters, rates use seconds, angular velocities use rad/s,
and all orientation calculations use radians/quaternions. Degrees appear
only in optional operator display labels if explicitly converted.

Image and IMU streams use the ROS 2 sensor-data QoS profile (best effort).
Their subscribers use compatible QoS. Command, encoder, and detection topics
use reliable depth-10 queues. Map and marker topics use reliable,
transient-local depth-1 durability, so late subscribers receive the latest
map. Use matching QoS when inspecting them with the ROS CLI.

Services:

| Service | Type | Effect |
| --- | --- | --- |
| `/imu/reset` | `std_srvs/Trigger` | Reset orientation and restart gyro/accelerometer calibration |
| `/map/save` | `std_srvs/Trigger` | Atomically replace the JSON grid and coordinate tables |
| `/ekf/set_parameters_atomically` | standard ROS 2 parameter service | Tune positive `odom_variances` / `imu_variances` |

TF ownership is explicit:

```text
map --mapping, static initial corner/heading--> odom
odom --EKF, dynamic--> base_link
base_link --IMU driver, static mounting identity--> imu_link
```

Wheel odometry does not publish TF by default, preventing two competing
`odom`→`base_link` broadcasters. If EKF is disabled and wheel odometry is used
alone, explicitly enable its `publish_tf` parameter. Camera image frames are
labeled `camera_optical_frame`; an actual camera mounting TF is intentionally
left for measured robot geometry.

## Correctness changes from the old implementation

- `rclpy` nodes, timers, declared parameters, ROS 2 services and launch replace
  `rospy`, catkin, ROS master setup and ROS 1 launch XML.
- GPIO PWM uses 0–100 percent instead of clipping 0–255 speeds directly into
  a percentage API. Saturation preserves the left/right ratio.
- Positive angular commands follow ROS counterclockwise convention. Motor
  wiring signs and FR/FL/RL/RR ordering come from the old code.
- Host and firmware watchdogs stop the drive and arm. Joystick input requires
  a deadman button and has a freshness timeout.
- Encoders use Int32 end to end, correct modulo-2^32 rollover, independent
  left/right tick scales and an actual received baseline before integrating.
- Wheel odometry uses exact constant-curvature integration and leaves IMU
  fusion to the EKF, avoiding double counting the same yaw source.
- The EKF uses NumPy arrays, wraps every angular innovation, updates each
  sensor sample once, uses nonzero variances and Joseph-form covariance
  correction, and stops extrapolating linear motion after odometry expires.
- IMU calibration is nonblocking and preserves gravity. The Madgwick filter
  handles zero-length correction gradients without NaNs. Sensor mounting TF
  is static instead of applying world orientation again as a mounting offset.
- Vision no longer passes state through a shared text file or a hardcoded
  TCP endpoint. Stale/missing images clear detection and alignment flags.
- Mapping covers the 20 m × 20 m field, confirms distinct metal observations,
  bounds-checks projected detector coordinates with floor, requires fresh
  encoder-backed odometry and ready vision, and atomically saves each changed
  cell with coordinate tables and metric detection positions. See
  [competition mapping](COMPETITION_MAPPING.md) for its accuracy limits.
- Initial map corner and heading are configured numerically before launch;
  the old mouse-selection UI is replaced by YAML configuration.

## Configuration and deployment boundaries

The original code disagreed on drive geometry (motor half-track 0.5 m versus
odometry track 0.55 m). The port consistently defaults to 0.55 m full track,
with inherited left/right tick scales. These are starting values, not new
hardware measurements. Match the firmware constants to both workspace YAMLs.

Only EKF covariance parameters are applied dynamically. Restart nodes after
changing other configuration. A saved map is an exported result; it is not
automatically loaded on startup. The GUI's start corner/heading configuration
also defines the static `map`→`odom` transform.

References: the upstream [ROS 2 migration documentation source](https://github.com/ros2/ros2_documentation/tree/rolling/source/How-To-Guides/Migrating-from-ROS1),
[QoS documentation source](https://github.com/ros2/ros2_documentation/blob/rolling/source/ROS-Framework/interfaces/topics/About-Quality-of-Service-Settings.rst),
and [Arduino CLI installation and command reference](https://docs.arduino.cc/arduino-cli/installation/).
