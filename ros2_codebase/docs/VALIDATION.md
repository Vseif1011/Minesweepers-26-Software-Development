# Validation — STM32 integration audit, 2026-10-16

Current physical STM32/Pi serial results are recorded separately in
[direct serial validation](DIRECT_SERIAL_VALIDATION.md).

The earlier ROS 2 migration report is preserved below as historical evidence.
This section describes the current Pi/STM32 application changes.

| Executed check | Result |
|---|---|
| Shared `minesweeper_interfaces` native build | Passed |
| Pi and laptop colcon builds against the shared underlay | Passed |
| Pinned native Lyrical micro-ROS Agent build | Passed; XRCE Agent 3.0.2, two packages; upstream compiler/CMake warnings remain |
| Sourced Agent package discovery and executable help | Passed; install at `agent_ws/install` |
| Pi pytest suite | 17 passed |
| Laptop pytest suite | 25 passed |
| Existing installed mock-system graph | 23 checks passed |
| Installed laptop launch with synthetic STM32-format telemetry | 6 checks passed |
| Pi real/mock/GPIO launch action selection | Covered by Pi tests; no devices opened |
| Launch argument introspection and shell syntax | Passed |
| Reading-guide coverage | All 61 authored files outside firmware; all 140 named Python functions/methods, including nested definitions |

Native telemetry tests use generated message bindings and best-effort DDS,
including paired signed counts around 2^40, synchronized acquisition stamps,
MCU reset, stream gaps, stale samples, detector validity and raw-gyro input to
the EKF. The first count test exposed NumPy int64 intermediate overflow in the
modulo-2^64 calculation; conversion to unbounded Python integers fixed it.
The native smoke source publishes at a bounded 50 Hz and drains callbacks
between emissions to avoid a test-generated subscription backlog.

The six native graph checks verify encoder processing, raw-gyro fusion,
detector adaptation, absence of legacy sensor publishers, absence of STM32
motor command publishers and no invented measurements when sensors disappear.
They run the installed laptop launch on isolated domain 88 with **synthetic**
telemetry. This is an application integration test, not a live USB/Agent/board
test. The original mock graph uses domain 87. Logs and reports are under
`artifacts/`, particularly `stm32_laptop.log`, `stm32_integration_report.json`
and `smoke_report.json`.

The firmware's existing flashing report records the previous successful flash,
CDC enumeration and native Agent status/restart tests. No STM32 CDC/HID device
or Raspberry Pi is connected during this audit; no firmware was flashed or
modified here. The prior temporary native Agent installation was unavailable,
so the exact recorded Agent/message source commits were rebuilt into the
ignored `agent_ws/` directory. Source its install before starting real Pi mode.

The user confirmed the five motor channels and two encoder channels remain
unassigned. No motor command mapping or automatic arming was introduced;
the flashed commissioning profile still rejects arming. Pi-native Agent
runtime, physical sensors, hardware assignments/calibration and cross-machine
discovery remain deployment checks. Optional real YOLO/GUI paths were not
executed as part of this integration change.

Repeat builds with `bash scripts/build.sh`, matching Agent construction with
`bash scripts/build_agent.sh`, and application tests with `bash scripts/test.sh`.
See [STM32 deployment](STM32_DEPLOYMENT.md) for source/underlay order and
[the complete reading guide](READING_GUIDE.md) for the file/function syllabus.

---

# Validation — 2026-10-15

Environment: Ubuntu 26.04.1, ROS 2 Lyrical from `/opt/ros/lyrical`, Python
3.14.4, NumPy 2.3.5, OpenCV 4.10.0, pytest 9.0.2. No robot hardware was
connected or accessed. No firmware was uploaded.

## Results

| Check | Result |
| --- | --- |
| Independent Pi workspace `colcon build --symlink-install --executor sequential` | Passed |
| Independent laptop workspace, same build command | Passed |
| Pi package pytest via `colcon test` | 14 passed; no failures/errors/skips |
| Laptop package pytest via `colcon test` | 22 passed; no failures/errors/skips |
| Installed executables, live ROS 2 graph smoke test | 23 checks passed |
| Standalone Mega 2560 firmware compilation | Passed; AVR core 1.8.8 |
| Shell syntax, build and test scripts | Passed |
| Copied YOLO checkpoint checksum | Matches original exactly |

Run the ROS package builds with `bash scripts/build.sh` and all hardware-free
tests with `bash scripts/test.sh` from the parent folder. Repeat firmware
compilation separately with `bash scripts/build_firmware.sh`, after installing
Arduino CLI and the `arduino:avr` core. Compilation used Arduino CLI
1.5.2-rc.1 and produced a sketch using 8988 bytes of flash and 357 bytes of
global SRAM. An initial full compile with `--warnings all` reported only
unused-parameter warnings in the upstream Arduino AVR core's `new.cpp`.

## Algorithm and serial tests

The 36 pytest cases cover:

- ROS positive turn convention, ratio-preserving speed saturation, physical
  motor signs and percentage PWM scaling; nonfinite command rejection.
- Serial command formatting, full-width Int32 telemetry, malformed and
  oversized frames, fragmented serial input and command expiry against a real
  pyserial connection to a Linux pseudo terminal.
- Stationary and rotating Madgwick filtering, including zero accelerometer
  input and a zero correction gradient.
- Encoder baselines, forward/reverse Int32 rollover, exact arc integration,
  separate tick scales and zero-dt handling.
- EKF angular wrap, invalid measurements, positive/symmetric covariance over
  repeated prediction/correction, and tracking a known trajectory.
- Live covariance updates, atomic rejection of invalid changes, and positive
  variance enforcement.
- First mine detection, duplicate suppression, buried-to-surface upgrades,
  map bounds, initial heading transformation, missing joystick axes,
  deadzone, clamping and nonfinite axis values.

## Live executable tests

The smoke test starts the installed ROS 2 launch files and executables:

- Pi: `arduino_bridge`, `imu`, `camera`, and the alternative
  `motor_controller`, all using mock backends.
- Laptop: `teleop`, `odometry`, `ekf`, `mapping`, `vision`, followed by the
  finite `command_demo` executable.

The 23 assertions check topic connectivity; JPEG encoding and transport;
synthetic image detection and gripper alignment; zero outputs before deadman
input; moving encoder feedback; finite fused poses; both mine classes;
bounded motor outputs; input-loss and publisher-loss watchdogs; stopped
encoders; the complete map/odom/base/IMU TF chain; reset/save services;
covariance service acceptance/rejection; saved coordinate tables; camera-loss
clearing of vision flags; and successful completion of `command_demo`.

After the camera-loss test, camera publishing is stopped while vision remains
running, starting from a positive detection. The final drive watchdog test
publishes a nonzero command and then stops publishing without sending a stop
message. Both drivers must independently expire that command.

`artifacts/smoke_report.json` contains the machine-readable result, message
counts, and a sample fused pose. `artifacts/pi.log`, `laptop.log`, `camera.log`,
`gpio.log`, `command_demo.log`, `firmware_build.log`, exported mine map and
firmware binaries are retained locally and ignored by Git. Nodes launched by
the smoke test are shut down afterward. Normal launch shutdown was checked
for clean child exits.

The bundled YOLO checkpoint SHA-256 is:

```text
e7639b962f1b7dcdeb625a7f903a6bf372e1864d46b977e11213f97dd64ef7ac
```

## Remaining validation on deployment

Physical serial timing, GPIO compatibility, I2C reads, sensor mounting,
encoder signs/scales, motor direction, gripper output and camera capture
require the robot. The firmware compiles but its actual interrupt timing and
hardware behavior have not been exercised. Network discovery across two
physical machines has not been tested; the live integration test runs both
workspaces in separate processes on localhost.

PyTorch/Ultralytics, Pygame and smbus2 are not installed in this environment.
The trained YOLO inference path, Pygame window and Tk covariance window were
not executed. Synthetic JPEG inference and the covariance parameter service
were tested directly. Optional GUI modules are included in the built package.

The environment's rosdep installation is not initialized, so automatic
dependency-resolution validation was unavailable. The ROS and Python
dependencies required for these builds and tests were already installed;
dependency installation commands and optional hardware/vision requirements
are documented in the README.
