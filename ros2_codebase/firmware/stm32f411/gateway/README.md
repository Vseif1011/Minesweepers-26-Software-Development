# Raspberry Pi Lyrical micro-ROS Agent

> **Historical micro-ROS implementation:** the active transport is now direct
> USB serial. See [current serial validation](../../../docs/DIRECT_SERIAL_VALIDATION.md). The results below
> describe the earlier micro-ROS path and do not validate the serial replacement.

The firmware's matching Agent sources are locked in `agent.repos`. Build on the
Pi's existing ROS 2 Lyrical installation, with a native compiler and enough disk
space. The Pi build/runtime and cross-machine discovery have not been executed
from this laptop session.

```bash
source /opt/ros/lyrical/setup.bash
mkdir -p ~/minesweeper_agent_ws/src
cd ~/minesweeper_agent_ws
vcs import src < /path/to/stm32f411/gateway/agent.repos
rosdep install --from-paths src --ignore-src -r -y
MAKEFLAGS=-j1 colcon build --merge-install --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial/by-id/ACTUAL_STM32_CDC_ID -b 115200
```

The Agent SuperBuild pins XRCE Agent v3.0.2, matching Client v3.0.2. Inspect the
resolved dependency version in its build log before deploying. Do not install
an arbitrary newer Agent beside the matching source build. The serial baud is
a CDC line-coding value, not the USB bus rate. Use the application's CDC device,
not the HID bootloader. Grant serial access with the supplied `plugdev` device rules or suitable
serial permissions. The native laptop Agent build and live USB status/restart
checks passed; Pi runtime and network discovery still require deployment tests.

Build `ros2_codebase/interfaces/minesweeper_interfaces` on the laptop for ROS
inspection and command ownership. The Agent can create these DDS entities from
the client type definitions without a forwarding node. The same interface
source is needed anywhere a native ROS publisher/subscriber consumes these
custom message types.

The domain is compiled into the firmware with `make ROS_DOMAIN_ID=...`. After
changing it, run `make clean` and rebuild; changing only the Pi environment does
not move the firmware participants into that domain. Set the same domain and
RMW on laptop and Pi, enable network multicast/discovery, avoid localhost-only
discovery, and synchronize clocks. Keep optional camera encoding/streaming in
a separate Pi process. Run perception, EKF, mapping and the command arbiter on
the laptop.

Do not start the legacy `minesweeper_pi` real-hardware launch for this STM32
deployment: its Arduino bridge and Pi GPIO/IMU nodes use different interfaces.
The old mock launch remains available for its old integration tests.

To inspect after building the native interface package:

```bash
ros2 node info /stm32_controller
ros2 topic echo /motor/status --qos-reliability best_effort --qos-depth 1
ros2 topic echo /imu/data_raw --qos-reliability best_effort --qos-depth 1
ros2 topic info /motor/setpoints --verbose
```

Expect `armed=false`, `hardware_ok=false`, zero duty, and hardware/feedback fault
bits with this profile. E-stop is asserted when PA4's NC loop is open. Raw IMU
data appears only with the MPU6050 wired and successfully initialized. Encoders
and metal signal remain silent until electrically qualified and configured.
