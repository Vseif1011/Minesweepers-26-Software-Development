# STM32 direct serial deployment

The active STM32F411CEU6 firmware uses HAL, FreeRTOS and direct USB CDC serial.
No micro-ROS client library or micro-ROS agent is used by this path. Previous
micro-ROS source locks and reports are retained as historical references.

```text
Laptop ROS 2 <-- DDS network --> Pi stm32_bridge <-- USB serial v1 --> STM32
```

The Pi bridge maps the same ROS topics as before: `/motor/setpoints`,
`/motor/status`, `/encoders/ticks`, `/imu/data_raw`, `/metal_detector/signal`.
All use best effort, volatile, depth 1. The MCU owns the 200 Hz motor/PID loop,
watchdog, sensor acquisition and arming checks. No control loop moves to Linux.

## Build

On each ROS machine source `/opt/ros/lyrical/setup.bash`. Build `interfaces/`
first and source its `install/local_setup.bash` before building `pi_ws` or
`laptop_ws`. `bash scripts/build.sh` performs all three local builds. Install
`python3-serial` on the Pi. Never copy x86 build/install directories to ARM.
A single combined Pi workspace may instead contain both source packages:

```bash
source /opt/ros/lyrical/setup.bash
cd ~/minesweeper_serial_ws
MAKEFLAGS=-j1 colcon build --symlink-install --executor sequential
source install/local_setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch minesweeper_pi pi.launch.py mock:=false camera:=false serial_port:=/dev/ttyACM0
```

Use the actual device path, preferably `/dev/serial/by-id/...`; grant serial
access through the `dialout` group or appropriate device rules. Only the bridge
may open this port. The firmware USB product is `STM32 direct serial`.
`baud:=115200` is CDC line coding, not USB bus speed. A micro-ROS application
cannot be used with this bridge; flash the matching direct-serial image first.

On the laptop:

```bash
source /opt/ros/lyrical/setup.bash
source interfaces/install/local_setup.bash
source laptop_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch minesweeper_laptop laptop.launch.py hardware:=stm32 mock:=true vision:=false
```

Here `mock:=true` controls synthetic vision, not STM32 sensor data. Synchronize
Pi/laptop clocks (NTP/chrony), use matching ROS domains/RMW and allow network DDS
discovery. The MCU no longer has a compiled ROS domain; only the hosts use DDS.

## Safety and remaining hardware work

The bridge forwards the original command timestamp, session, sequence, explicit
arm flag and five targets. It drops commands older than 100 ms or more than
20 ms in the future, and requires matching status received within 150 ms. It
never queues commands across reconnects, generates targets or rearms. The MCU
independently checks all command fields and disarms after 250 ms without fresh
commands. Its local arming and fault state machine is unchanged.

The commissioning profile still has `BOARD_HARDWARE_APPROVED=0`: motor outputs
remain disabled. Motor/encoder roles, driver polarity/limits/PWM frequency,
encoder scales and detector interface still require physical definition. Legacy
`/cmd_vel` or `/controller` messages are not STM32 commands; a designated command
arbiter remains future work after channel assignments. No sensor reading is
invented when its hardware is unavailable.

Read [serial protocol](../firmware/stm32f411/docs/SERIAL_PROTOCOL.md) for framing,
clock synchronization, telemetry payloads and diagnostics. Existing historical
micro-ROS validation does not validate the new serial transport; consult the
current serial validation report for executed checks and deployment status.
