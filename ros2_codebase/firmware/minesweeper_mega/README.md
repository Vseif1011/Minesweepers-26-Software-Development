# Arduino Mega 2560

`minesweeper_mega.ino` replaces the ROS 1 rosserial firmware. It uses only the
Arduino AVR core: there are no rosserial, micro-ROS, or external encoder-library
dependencies. It retains the original motor, gripper, detector and encoder pins.

Compile and upload with Arduino IDE (board: **Arduino Mega or Mega 2560**) or:

```bash
arduino-cli core install arduino:avr
arduino-cli compile --fqbn arduino:avr:mega /path/to/minesweeper_mega
arduino-cli upload --fqbn arduino:avr:mega --port /dev/ttyACM0 /path/to/minesweeper_mega
```

Uploading requires the physical board and is not part of the hardware-free
tests. Stop the Pi serial bridge before opening the serial port for upload.
The sketch was successfully compiled for `arduino:avr:mega` with AVR core
1.8.8: 8988 bytes of flash and 357 bytes of global SRAM. The repeatable
`scripts/build_firmware.sh` command builds it without uploading, and writes
the output under `artifacts/firmware/`.

At 115200 baud, commands and telemetry are newline-terminated ASCII:

```text
C,0.20000,0.10000,9
T,1234,5678,1
```

`C` fields: linear m/s, angular rad/s, controller command. `T` fields: signed
32-bit cumulative left count, right count, and metal flag (0/1). Telemetry is
published every 50 ms, including the false metal state. The Pi handles
fragmented serial reads and validates frame size, field count, and ranges.

Controller commands retain their legacy meaning:

| Command | Action |
| --- | --- |
| 5 / 6 | Speed up/down, applied by laptop teleop |
| 7 / 8 / 9 | Arm up / down / stop |
| 10 / 11 | Release / hold magnet |
| 0 | Stop arm |

A 500 ms firmware watchdog stops drive and arm outputs when valid commands
stop arriving, independently of the host watchdog. The magnet retains its last
state. The metal detector has a 40 ms time-based debounce. Oversized or malformed
commands are discarded without refreshing the watchdog.

Encoder inputs are pins 10/11 (left, PB4/PB5 pin-change interrupt) and pins 2/3
(right, external interrupts), with input pullups. Both use x4 quadrature counts.
Set `LEFT_ENCODER_SIGN` and `RIGHT_ENCODER_SIGN`, then measure ticks/meter.
`TRACK_WIDTH` and `MAX_SPEED` must agree with the workspace YAML configuration.
Positive ROS angular velocity is counterclockwise: right wheel faster than left.

The drive outputs retain the original FR, FL, RL, RR pin ordering and physical
signs. Pin mapping and motion direction remain unverified on real hardware.
