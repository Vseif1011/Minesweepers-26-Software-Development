# Verification report — 2026-10-16

> **Historical micro-ROS implementation:** the active transport is now direct
> USB serial. See [current serial validation](../../../docs/DIRECT_SERIAL_VALIDATION.md). The results below
> describe the earlier micro-ROS path and do not validate the serial replacement.

## Executed

| Check | Result |
|---|---|
| Official Lyrical source dependency selection | Client and Agent both XRCE 3.0.2; source locks recorded |
| Full Lyrical ARM static client build | PASS, 45 packages using shipped build script, Cortex-M4 hard float |
| Clean full firmware cross-build | PASS, ARM ELF/HEX/BIN/map; application entirely C |
| Linker layout | PASS, application at `0x08004000`, 16 KiB bootloader preserved, 496 KiB application flash |
| Initial SP and reset vector bounds | PASS, checked by `scripts/inspect_image.py` |
| Host safety/control tests | PASS, 10,271 assertions with AddressSanitizer and UndefinedBehaviorSanitizer |
| Mocked motor HAL tests | PASS, duty saturation, DIR and ENABLE polarity, reversal delay, stop, invalid inputs |
| Mocked MPU6050 HAL tests | PASS, identity, init/readback, six exact signed user offset registers, data-ready, SI units, saturation, bus failure |
| Project metadata/config checks | PASS, XML parse, motor-disabled profile, confirmed HSE clock, immutable source pins and bounded messages |
| Native Lyrical interface build | PASS, `minesweeper_interfaces` generated for laptop ROS |
| Native/embedded interface type hash equality | PASS for MotorCommand, MotorStatus, EncoderTicks |
| USB identification | WeAct Studio HID bootloader `0483:572a`, serial `3691358F3133` |
| USB write access | Initially blocked; user granted temporary ACL and confirmed motor power isolated |
| Physical application flashing | PASS: vendor tool reported Flash Done, reboot succeeded |
| Application USB enumeration | PASS: `0483:5740`, product “STM32 commissioning micro-ROS”, serial `005A00573133510836373538`, `/dev/ttyACM0` |
| Application serial access | Initially blocked; user granted ACL, read/write open confirmed |
| Prepared CubeIDE ZIP | PASS: extracted to a separate directory, clean cross-build produced byte-identical BIN |
| Native Lyrical Agent build | PASS, XRCE Agent 3.0.2 with bundled spdlog 1.9.2 |
| Updated firmware / live Agent session | PASS, 99 status messages in 5 s; all four publishers and command subscriber created |
| Agent process restart | PASS, 144 status messages in 8 s; MCU uptime continued, motors remained disarmed, all duties zero |
| Runtime memory (bench) | 47,352 bytes free micro-ROS heap before/after one restart; control stack 642 words free, micro-ROS stack 3,541 words free |
| Bounded libc heap regression | PASS, newlib srand allocation size, bounds, exhaustion and negative increments |
| Bootloader read query | Vendor CLI detected the device, then exited 139 without USB write permission; no flash operation performed |

The host control tests cover boot without arming; zero/disarm then zero/arm
handshake; invalid arm targets; watchdog boundary; HAL tick wrap; link loss;
no restart on recovery; physical-stop state; unverified hardware; missing encoder
feedback; unsynchronized clock; control deadline fault; NaN/infinity/range
rejection; command age/future timestamp; sequence/session replay; encoder
16/32-bit wrap and impossible deltas; PID anti-windup and invalid measurements.
Mock fixtures are synthetic test hardware, not claimed robot wiring.

Linked resource use for the final commissioning build:

```text
text   100,056 bytes
data     8,736 bytes
bss     93,848 bytes
flash  108,836 / 507,904 bytes (21.43% of application region)
RAM    102,576 / 131,072 bytes (78.26%, includes heap/stack reservations)
```

Link-time fit does not measure runtime heap usage or actual task stack margins.
The 48 KiB micro-ROS heap is reserved in BSS; the main stack reserve is included
in the linker RAM figure. On-board allocation success, reconnect fragmentation,
watermarks and worst-case interrupt latency still require measurement.
Upstream HAL/client compiler warnings remain; application C compiles with
implicit-function declarations treated as errors.

Repeat locally:

```bash
bash scripts/test.sh
make clean
make -j4
python3 scripts/inspect_image.py
```

For the native message check, source ROS 2 Lyrical and build
`ros2_codebase/interfaces` into a separate colcon workspace, then run:

```bash
python3 scripts/check_type_hashes.py \
  --native-build /absolute/path/to/native/build/minesweeper_interfaces
```

## First live Agent attempt and update

The matched Lyrical Agent built successfully on the laptop. Its first serial
connection caused the board to reset; the observer received zero status frames.
Inspection of the exact newlib ARM object confirmed `srand()` calls malloc(24)
and asserts on failure, while the initial `_sbrk` rejected allocations. The
updated image uses the already-reserved bounded 1 KiB libc heap. It also applies
the user's six MPU6050 hardware offsets with readback. The updated image was built, host-tested and reflashed. The live retry passed:
99 status frames in five seconds; one Agent process restart recovered with
144 frames in eight seconds. MCU uptime remained continuous, outputs stayed
zero/disarmed, and free micro-ROS heap remained 47,352 bytes. No motor commands
were sent during these observations.

## Not executed / remaining blockers

Pi-hosted Agent operation, bidirectional command delivery, physical clocks,
worst-case FreeRTOS timing, motor GPIO
levels/polarity, real PWM limits/frequency, physical encoder counts, MPU6050
readout, detector signals and independent emergency stop are **unverified**.
CubeIDE GUI import and CubeMX regeneration are also unverified because those
applications are not installed. No Pi login or connected Pi Agent was available. The live checks used the
matching native Lyrical Agent on the laptop. One restart is a bench regression
check, not the 100-cycle endurance procedure below.

Motor driver models, enable/brake wiring, encoder electrical outputs and
calibration, and the coil front end are not documented in the repository.
The user confirmed motor supply isolation before flashing, then clarified that
no motors or sensors are attached to the board. Energized motor operation
is blocked by the missing driver/interface data. The default firmware
therefore continues to reject arming.

## Physical acceptance procedure

Record measured values, instrument setup, firmware SHA256 and operator initials.
Do not record an expected outcome as a passed result.

1. **Isolate motor energy.** Disconnect the motor supply; do not rely on USB
   commands or driver logic for this isolation. Verify with a meter. Confirm the
   actual motor driver, input ratings, idle polarity, brake/coast and inhibit
   scheme before connecting control signals.
2. **Inspect powered-off wiring.** Check common reference, external biasing,
   driver input pulls and E-stop power contacts. Verify encoder receiver outputs
   are bounded to MCU-safe levels before connecting PA6/PA7/PB6/PB7. Identify the
   detector front end; do not connect a raw coil to PA5.
3. **Flash the motor-disabled image.** Follow FLASHING.md, verify ELF vector
   bounds, preserve bootloader sector 0. Expect application CDC enumeration
   after reboot. Confirm HSE=25 MHz, SYSCLK=96 MHz, USB=48 MHz, SysTick=1 kHz
   using ST-Link registers or appropriate external measurements.
4. **Verify startup GPIO states.** Scope every driver-related pin during power
   on, reset, bootloader entry, application start and Agent connection. In this
   profile PA0–PA3/PA8 must remain input/high impedance, not active PWM. Confirm
   the actual driver cannot energize from reset/input states using its own
   external inhibit circuit. Check PA4 reads STOP when its NC loop is open.
5. **Connect the matched Agent.** Use gateway/README.md. Observe status with
   best-effort QoS: `armed=false`, `hardware_ok=false`, zero duty, hardware and
   feedback faults. Publish zero commands and attempted arm messages; all must
   leave the hardware-unapproved profile disarmed. No valid encoder/detector
   sample may be invented for unbound peripherals.
6. **Validate the IMU.** Wire PB8/PB9, AD0=GND and appropriate 3.3 V pullups.
   On reset, verify identity 0x68 and driver configuration. At rest, raw gravity
   magnitude should be approximately 9.81 m/s² in the chip axes. Turn the sensor
   and check axis signs/rad/s. Disconnect it, short a bus line through a safe
   test fixture, or present an invalid identity: no zero-filled valid IMU
   measurements should be published. After correction, reset for reinitialization.
7. **Qualify feedback.** With driver power still isolated, feed a bounded
   quadrature generator and rotate each real encoder by a measured revolution.
   Verify both signs, TI12 decoding, actual counts/revolution, timer wrap, maximum
   count delta and input conditioning at maximum edge rate. Populate measured
   bindings. Do not infer the two motor associations from the old Mega code.
8. **Qualify driver outputs without motors energized.** Implement the confirmed
   topology and bind verified inactive levels, frequency, duty/target limits,
   encoder associations, gains and reversal deadtime. Only after review change
   the hardware sign-off setting. Boot/reset/reconnect must still keep enables
   inactive and PWM at zero. Scope the explicit zero/disarm → zero/arm sequence.
   Apply a small target into the disconnected driver/logic test fixture; check
   channel identity, PWM saturation and direction polarity. Reverse it and
   measure zero/inhibit time before polarity changes. Check E-stop inhibition
   with the CPU halted, USB unplugged and Pi absent.
9. **Fault injection without motor energy.** After arming the qualified logic
   fixture, stop command publication, unplug USB, stop the Agent, restart the
   Pi, disconnect the laptop/network and saturate the video stream. Expect a
   local stop no later than 250 ms plus one healthy 5 ms control period after
   the last accepted command. Reconnect/restart without operator arm: outputs
   must stay inhibited. Repeat the zero/disarm and zero/arm handshake deliberately.
10. **Reject dangerous commands.** Send NaN, infinity, excessive targets, old
    timestamps, future timestamps, duplicate/out-of-order sequence, old session
    and arm with nonzero targets. Expect disarm, cleared targets and fault status.
    Queued stale commands must not refresh the watchdog.
11. **Verify feedback/deadline faults.** Inject an impossible encoder delta and
    a controlled control-task overrun through the debugger/test build. Expect a
    latched stop. Loss of IMU/detector data must make those streams invalid/silent;
    local velocity control does not require those sensors, so higher-level
    autonomous motion must also enforce their freshness on the laptop.
12. **Verify independent watchdog.** In an isolated test setup, stall the control
    task or interrupts. Measure the IWDG reset time including LSI tolerance;
    do not claim the normal 250 ms communication bound for a frozen CPU.
    After reset, the robot must remain disarmed. Restore the normal build.
13. **Endurance.** Repeat at least 100 Agent disconnect/reconnect cycles, monitor
    the reported micro-ROS heap minimum and task stack high-water marks, and
    inspect control period jitter under telemetry/bus load. Require positive
    measured margins and no declining heap trend before accepting the profile.
14. **Enable motor power last.** Only after GPIO/driver polarity, PWM bounds,
    watchdog and independent physical E-stop have all passed, perform a supervised,
    current-limited low-duty mechanical test with an immediately accessible
    physical stop. Calibrate local PID and motor limits from measured response.

## Current outcome

A reproducible cross-built commissioning project and verified host logic are
available. The final commissioning image, including the user-provided MPU6050 offsets,
was flashed; USB enumeration and live Lyrical status/reconnect behavior passed.
Energized motor operation and physical driver/sensor qualification remain
unverified; the build continues to reject arming.
