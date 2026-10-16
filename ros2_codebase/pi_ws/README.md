# Pi ROS 2 workspace

Real STM32 hardware uses **direct USB CDC serial**, with no micro-ROS agent.
The `stm32_bridge` node forwards `/motor/setpoints` as validated binary packets
and publishes the received status, encoder, IMU and detector topics. The STM32
runs control and safety locally. It must have the matching serial v1 firmware;
the older micro-ROS firmware is incompatible.

Build the shared interfaces first, then this workspace:

```bash
source /opt/ros/lyrical/setup.bash
cd interfaces
colcon build --symlink-install --executor sequential
cd ..
source interfaces/install/local_setup.bash
cd pi_ws
colcon build --symlink-install --executor sequential
source install/local_setup.bash
ros2 launch minesweeper_pi pi.launch.py mock:=false camera:=false serial_port:=/dev/ttyACM0
```

Install `python3-serial` for serial access. Prefer the actual `/dev/serial/by-id/`
path. Only one process may own the serial port. `camera:=true` enables a physical
camera. `mock:=true` retains the synthetic legacy graph without hardware.
`legacy_pi.launch.py` retains the old Mega ASCII protocol and Pi GPIO/IMU path.

The bridge never creates motor commands or automatically rearms. It drops stale
or wrong-session commands and keeps no command queue across reconnects. Motor
roles remain unassigned and the commissioning firmware cannot arm.
See [deployment](../docs/STM32_DEPLOYMENT.md) and [serial protocol](../firmware/stm32f411/docs/SERIAL_PROTOCOL.md).
