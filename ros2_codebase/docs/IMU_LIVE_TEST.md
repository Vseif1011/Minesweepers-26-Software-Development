# Live MPU6050 test

The MPU6050 is connected to the STM32F411 over I2C1: SCL=PB8, SDA=PB9,
AD0=GND for address 0x68, common GND and a compatible 3.3 V module supply.
The STM32 remains connected to the Pi over direct USB CDC serial. No micro-ROS
agent is used. The Pi bridge publishes `/imu/data_raw` and `/motor/status`.

The Pi bridge was not running after a reboot and was started. Initially the
CDC port opened without incoming packets. A user-triggered normal STM32 reset
restored status and valid IMU telemetry. No firmware changes or motor commands
were made during this test. The specific reason for the initial silent USB
connection was not isolated. Firmware probes the IMU once at startup, so reset
or power-cycle after adding the sensor.

## Results

| Measurement | Observed result |
|---|---|
| Status | `imu_valid=true` |
| Initial baseline | 509 IMU samples over 15 s |
| Baseline acceleration magnitude | 9.965 m/s² (about 1.6% above standard gravity) |
| Baseline mean gyro X/Y/Z | 0.238, -0.036, -0.446 °/s |
| Movement sample | 1557 samples over 45 s |
| Laptop receive rate during movement | 34.71 Hz |
| Median timestamp age during movement | 42.1 ms; includes host clock offset |
| Peak absolute gyro X/Y/Z | 227.7, 170.5, 216.1 °/s |
| Acceleration ranges X/Y/Z during movement | 11.55, 13.01, 8.62 m/s² |
| Integrity | All received axes finite; timestamps strictly increased |
| Motors | Disarmed, all duties zero; no `/motor/setpoints` publisher |

All three gyro axes and accelerometer axes responded during the user's
movement exercise. This establishes connectivity, fresh readings and movement
response. It does not certify calibration accuracy or robot-frame mounting.
The baseline has a small residual gyro bias; finalize calibration on the
mounted sensor before relying on integrated heading.

The firmware configures 50 Hz sensor acquisition; the laptop receive rate is
a separate Wi-Fi/ROS delivery measurement. Data uses `imu_link`, acceleration
in m/s² and angular velocity in rad/s. Orientation is not computed:
`orientation_covariance[0] = -1`.

## Inspect again

On the laptop, source ROS 2 and the interface package, then:

```bash
source /opt/ros/lyrical/setup.bash
source interfaces/install/local_setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ros2 topic echo /imu/data_raw --qos-reliability best_effort
```

The Pi serial bridge is running. After a Pi reboot, start it with
`bash ~/minesweeper_serial_ws/start_bridge.sh` if no bridge already owns the
serial port. Both host clocks must be synchronized.

Raw samples, summaries and the motion-response plot are saved in
`artifacts/imu_live/`: `baseline.json`, `movement.json`,
`movement_samples.json`, `motion_response.json`, and `imu_movement.png`.
