# STM32F411CEU6 real-time firmware

This is a Cortex-M4 firmware project in C using STM32 HAL, FreeRTOS and
direct USB CDC serial (no micro-ROS library or agent). It produces an ARM ELF, HEX and BIN for the STM32, independently
of the Linux ROS workspaces. Import the included `.project` into STM32CubeIDE.

For an Arduino/ROS introduction and a shareable wiring checklist, start with
[the electrical-team handoff](docs/ELECTRICAL_TEAM_HANDOFF.md).

**The default commissioning profile cannot arm motors.** The new, explicit
`mdd10a` build profile follows the user-supplied PA0–PA3/PA8 PWM and
PB12–PB15/PB10 direction wiring, using 20 kHz PWM+DIR with no ENABLE or STM E-stop
input. It is open-loop duty control, independent of uncommissioned encoders.
See [current motor setup](../../docs/MOTOR_CONTROL_SETUP.md) for the pin table,
limits and joystick controls. The replacement motor image is prepared but has
not been flashed; the last verified image is still motor-disabled.

The application preserves the connected WeAct HID bootloader: flash starts at
`0x08004000`, leaving its first 16 KiB intact. Normal USB is sufficient for
bootloader flashing; SWD debugging requires an additional ST-Link.

## Build

The firmware requires GNU Arm GCC, newlib, Make, Python 3 and Git. ROS is not
required to build the STM32 image.

```bash
cd ros2_codebase/firmware/stm32f411
python3 scripts/fetch_dependencies.py
make -j4
make test
python3 scripts/inspect_image.py
```

For the motor profile, use `make -j4 MOTOR_PROFILE=mdd10a`; outputs go into
`Debug-mdd10a/`. The root wrapper is `bash scripts/build_stm32_firmware.sh mdd10a`.
Do not confuse it with the default motor-disabled `Debug/` image.

Set `TOOLCHAIN_PREFIX=/absolute/path/to/arm-none-eabi-` when the compiler is not
on PATH. Sources use immutable vendor commits. Outputs are in `Debug/`.
`python3 scripts/package_project.py` bundles the prepared CubeIDE project.
See [SERIAL_PROTOCOL.md](docs/SERIAL_PROTOCOL.md) for the active wire contract.
The previous `micro_ros/` and `gateway/` trees are historical and are excluded
from the active firmware build.

## Open in STM32CubeIDE

1. Run dependency preparation above once.
2. Choose **File → Import → General → Existing Projects into Workspace** and
   select this directory. Leave “Copy projects into workspace” unchecked.
3. Select `Minesweeper_STM32F411`, configuration **Debug**, then **Build Project**.
   It calls the included Makefile. Ensure `make` and `arm-none-eabi-gcc` are in
   the build environment PATH; the compiler can come from CubeIDE's GNU tools.
   Alternatively set `TOOLCHAIN_PREFIX` in the project's build environment.
4. Open `Minesweeper_STM32F411.ioc` for the active commissioning configuration.
   `docs/Minesweeper_WiringPlan.ioc` also displays the confirmed PWM routing.

The application owns `main.c`, task creation, interrupt dispatch and build files.
CubeMX is a peripheral configuration record, not a replacement for those files.
If regenerating with a newer CubeMX, export into a separate directory and review
the HAL peripheral differences. In particular, preserve the vector relocation,
SysTick integration, FreeRTOS stack sizes, custom transport and motor lock.
No CubeIDE/CubeMX executable was available here to validate GUI import or
regeneration; the GNU Arm build was executed successfully.

## Implementation map

| Location | Purpose |
|---|---|
| `Core/` | Clock/startup, interrupts, FreeRTOS tasks, queues, IWDG |
| `Control/` | Hardware-independent safety state machine, PID, encoder wrap logic |
| `Drivers/Board/` | HAL GPIO, five-channel PWM/DIR/ENABLE driver, timer encoders, MPU6050, detector adapters |
| `USB_DEVICE/` | STM32 USB FS CDC device and descriptors |
| `Serial/` | Bounded CRC framing, USB transport, command/telemetry task |
| `micro_ros/` | Historical micro-ROS sources; not built |
| `../../interfaces/minesweeper_interfaces/` | Fixed-size ROS command/status/encoder messages |
| `gateway/` | Historical Agent source lock; not used |
| `tests/` | Sanitized host safety tests, mocked motor hardware and project checks |

Read [FLASHING.md](docs/FLASHING.md) for USB/ST-Link steps,
[INTERFACES.md](docs/INTERFACES.md) for the wire contract,
[VALIDATION.md](docs/VALIDATION.md) for measured build/test results and physical
acceptance procedures, and [AUDIT.md](docs/AUDIT.md) for remaining architecture gaps.
