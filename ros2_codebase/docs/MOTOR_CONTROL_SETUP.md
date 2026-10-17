# Logitech joystick and five Cytron motor channels

This setup follows the supplied STM32F411 BlackPill diagram and the pictured
**Cytron MDD10A Rev2.0** drivers. The user confirmed USB communication with the
Pi and **no emergency-stop input connected to the STM32**.

## Last flashed firmware versus prepared replacement

The Pi's saved flash readback, `~/minesweeper_serial_ws/results/application_after.bin`,
matches the previously verified 28,360-byte image:

```text
0eef3621896b656cc8719f39d183284a14f3e476be99d915cd748ea442ebd022
```

That image deliberately disables motor hardware, leaves all five motor modes
unbound, requires separate driver ENABLE signals and encoder feedback, and
assumes an NC stop input on PA4. It cannot drive this PWM+DIR wiring.

The replacement `mdd10a` build profile implements the connections below, omits
the unused ENABLE and STM stop input, and runs **all five channels in signed
open-loop duty mode**. Encoders are not needed to test motor movement. Encoder
scales and channel roles remain uncommissioned, so encoder telemetry stays
invalid rather than inventing motion feedback. Wheel speed regulation and
accurate odometry are separate commissioning work.

The replacement has been built, tested and copied to the Pi, **but not flashed**.
The STM was not enumerated on the Pi during this setup, so its current memory
could not be read or written. The recorded readback identifies the last verified
flash, not a fresh read of the physically absent board.

## Physical channel assignments

| Channel | Role | PWM | Timer | DIR | Provisional group |
|---|---|---|---|---|---|
| 1 | Drive | PA0 | TIM2 CH1 | PB12 | Left |
| 2 | Drive | PA1 | TIM2 CH2 | PB13 | Left |
| 3 | Drive | PA2 | TIM2 CH3 | PB14 | Right |
| 4 | Drive | PA3 | TIM2 CH4 | PB15 | Right |
| 5 | Arm | PA8 | TIM1 CH1 | PB10 | Independent |

The MDD10A supports 3.3 V PWM/DIR inputs and sign-magnitude PWM up to 20 kHz;
this profile uses **20 kHz active-high PWM**, positive command = DIR high,
and a 10 ms zero-PWM interval when reversing direction. There is no separate
ENABLE wire: stopping sets PWM to zero. Sources:
[Cytron specifications](https://www.cytron.io/p-10amp-5v-30v-dc-motor-driver-2-channels),
[Cytron PWM+DIR wiring example](https://www.cytron.io/tutorial/mdd10a-maker-uno-dc-motor-control).

Left/right grouping and physical forward polarity are provisional until wheel
tests. Both can be changed in
[stm32_joystick.yaml](../laptop_ws/src/minesweeper_laptop/config/stm32_joystick.yaml)
without changing GPIO routing or reflashing. Channel arrays are zero-based:
`[0, 1]` means physical PWM1/PWM2. For example, change a motor's `motor_signs`
entry to `-1` if it needs the opposite electrical direction.

The communication link is **USB CDC to the Pi**, as before. PA9/PA10 and Pi
GPIO14/15 are not used by this firmware for communication.

## Logitech Extreme 3D Pro input

The actual USB device was detected as `Logitech Extreme 3D pro`, with six Linux
axes and twelve buttons. A read-only movement capture confirmed:

| Input | ROS axis | Effect |
|---|---|---|
| Stick forward/back | 1 | All four drive duties positive/negative |
| Twist about Z | 2 | Opposite signs for the two drive groups |
| Base lever, fourth axis | 3 | Arm motor only, positive/negative |
| Stick left/right | 0 | Unused for this control scheme |

ROS `joy_node` reverses Linux's raw axis signs to the ROS convention: forward
and left are positive. With the provisional groups, a right twist requests
`[+, +, -, -, 0]`; left requests `[-, -, +, +, 0]`. Combined translation/turn
is mixed proportionally and bounded. The arm is independent. Requested signs
become physical directions only after the wheel polarities are established.

Hold **printed button 2**, Linux button index 1, to enable. This is the button
observed in the user's capture. Center the stick, twist and base lever, release
the button, then hold it. The base lever is not spring-centered: its midpoint
commands zero; its ends command opposite arm directions. Its 20% neutral band
accommodates the approximate manual midpoint. Releasing button 2 stops all five
channels. Axis/direction settings and the deadman button are configurable.

Initial laptop limits are **20% drive** and **15% arm** duty. The motor firmware
caps drive at 25% and arm at 20%. These are PWM fractions, not measured speeds.
An off-center base lever prevents the initial enable sequence. No arm limit
switches, position sensing or gravity holding behavior have been configured.

## Launch from the laptop

After `bash scripts/build.sh`, first inspect input without motor commands:

```bash
bash scripts/run_stm32_joystick.sh
```

The default is `dry_run:=true`. `/motor/joystick_targets` shows the requested
five duties; this diagnostic is not proof that the STM is armed. Any handshake
messages go to `/motor/setpoints_preview`, not the real command topic.

Once the correct firmware has been flashed and the robot is ready for its
physical movement test, the live command is:

```bash
bash scripts/run_stm32_joystick.sh dry_run:=false
```

Only run one motor controller at a time. The dedicated launch starts the selected
Logitech driver and `stm32_teleop`; it does not start legacy teleop, vision,
odometry or mapping. The full laptop launch also selects `stm32_teleop` when
`hardware:=stm32 teleop:=true`; add `joystick:=true motor_dry_run:=false` for live
control there. Legacy/mock hardware retains the original teleop node.

The host waits for the MCU to acknowledge **zero/disarm → zero/arm** before
sending nonzero targets. Missing/invalid joystick data, a removed joystick,
status loss, hardware-not-ready, MCU reset or a changed session prevents motion
and requires button release and neutral before re-enabling. Joystick timeout
is 250 ms, status timeout 150 ms, and the MCU command watchdog remains 250 ms.
Fresh commands retain their original Unix timestamps; keep both machine clocks
synchronized. A software stop is not a hardwired motor-power disconnect; this
wiring has no STM stop input.

## Pi configuration

The existing serial bridge already forwards all five signed targets, unchanged.
Its installed source hash matches this repository. A systemd unit is now enabled
on `pi@10.154.60.6`:

```bash
systemctl status minesweeper-stm32-bridge.service
journalctl -u minesweeper-stm32-bridge.service -n 20
```

The unit is [deploy/minesweeper-stm32-bridge.service](../deploy/minesweeper-stm32-bridge.service)
and calls the existing `~/minesweeper_serial_ws/start_bridge.sh`. It uses ROS
domain 0, Fast DDS, subnet discovery and `/dev/ttyACM0`. It waits/retries while
the STM is disconnected. It never creates arm or movement commands. Do not
start a second bridge on the same port. Laptop discovery verified the Pi's
command subscriber and status publisher, with no live motor-command publishers.

## Build and later flash

```bash
bash scripts/build_stm32_firmware.sh mdd10a
```

This builds `firmware/stm32f411/Debug-mdd10a/Minesweeper_STM32F411.bin`.
The default `commissioning` profile still builds a motor-disabled image under
`Debug/`; separate output directories prevent stale objects crossing profiles.
CubeIDE's default Makefile invocation is also commissioning; use
`MOTOR_PROFILE=mdd10a` explicitly for the motor image.

The prepared 30,776-byte image has SHA256:

```text
a2b5489dc89adb39e506b65529bf7e98dd5550f75f630ee5e4adbae637865078
```

It is also on the Pi at
`~/minesweeper_serial_ws/firmware_pending/Minesweeper_STM32F411_mdd10a.bin`.
Flash only when the board is accessible and motor power is physically isolated.
The image starts at **0x08004000**, preserving the resident first-16-KiB bootloader.
Follow the existing [flashing instructions](../firmware/stm32f411/docs/FLASHING.md)
for the chosen bootloader mode. The HID wrapper runs on an x86 laptop and accepts
`--motor-power-isolated mdd10a`; do not run its x86 vendor tool on the Pi.
The source/vendor package with this image is generated by
`python3 firmware/stm32f411/scripts/package_project.py --profile mdd10a`.

## Validation

The host C suite passed 10,273 control assertions plus mocked PWM/DIR/optional
ENABLE, reversal, IMU and heap checks. Both commissioning and motor ARM images
built and passed vector/flash-bound checks. Python tests cover mixing, limits,
neutral/deadman, both handshake acknowledgements, stale/invalid input,
disconnections and session changes. The joystick ROS smoke test uses an isolated
domain and synthetic MCU acknowledgements; no serial device or robot is driven.

Actual Logitech input and preview topics were observed with zero real motor
publishers. Logs and captures are under `artifacts/joystick_motor_setup/` locally.
The final complete run passed **115 Python tests and 44 ROS integration checks**
(26 mock-system, 6 native STM32 telemetry, 12 joystick/MCU-handshake checks).
Physical motor outputs, wheel roles/polarities and arm direction remain for the
robot test after flashing; software checks cannot establish those measurements.
