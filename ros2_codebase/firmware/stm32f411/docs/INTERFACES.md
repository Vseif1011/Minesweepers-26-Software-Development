# STM32 ROS interface contract

The Pi runs `stm32_bridge`, which maps the MCU's direct USB serial v1 packets
to the ROS topics below. No micro-ROS agent is used. See [SERIAL_PROTOCOL.md](SERIAL_PROTOCOL.md).
Use ROS 2 Lyrical and the same interface package on both hosts.
The firmware does not consume `/cmd_vel` or the old numeric `/controller` topic.
Conversion and command arbitration belong on the laptop, after actual motor
roles, gearing and feedback assignments have been confirmed.

| Topic | Direction / ROS type | Nominal rate | Meaning |
|---|---|---|---|
| `/motor/setpoints` | Laptop → MCU, `minesweeper_interfaces/msg/MotorCommand` | 50 Hz recommended | Five validated targets, explicit arm flag, session, sequence and creation time |
| `/motor/status` | MCU → laptop, `minesweeper_interfaces/msg/MotorStatus` | 20 Hz | Arming state, session challenge, faults, sensor validity and memory diagnostics |
| `/encoders/ticks` | MCU → laptop, `minesweeper_interfaces/msg/EncoderTicks` | Up to 50 Hz | Two signed 64-bit cumulative counts and rad/s, only with confirmed encoder config |
| `/imu/data_raw` | MCU → laptop, `sensor_msgs/msg/Imu` | Up to 50 Hz | MPU6050 chip-axis m/s² and rad/s, fresh valid samples only |
| `/metal_detector/signal` | MCU → laptop, `std_msgs/msg/Float32` | Up to 50 Hz | Conditioned digital 0/1 or raw ADC counts, depending on confirmed front end |

All endpoints use best-effort, volatile, keep-last depth 1 QoS. Command delivery
must be refreshed; there is no reliable backlog or transient command replay.
The non-real-time communication task can reduce telemetry rate during Agent
operations; it never runs the PID. Sensor topics are silent when unconfigured,
invalid, stale or (for stamped data) unsynchronized. Status continues to report
invalidity. Never interpret missing sensor data as zero motion or no metal.

`/motor/status` carries safety state; a duplicate safety publisher is unnecessary.
Its fault mask is: hardware=1, link=2, command timeout=4, E-stop=8, feedback=16,
invalid/replayed command=32, control deadline=64, unsynchronized clock=128.
`sequence` is the last accepted command sequence; consult `session` and flags
after faults. `duty` is the controller's requested signed duty for diagnostics,
not a measured voltage or a guarantee that the driver actually switched.
The driver further enforces duty limits and direction reversal deadtime.

## Target units and ownership

Array indices identify motor channels 1–5 in the confirmed PWM order. No wheel,
arm or gripper function is presumed from an array index. A configured feedback
channel uses rad/s at its encoder shaft; a configured open-loop channel uses
signed normalized duty, within its measured safe range. `MOTOR_UNBOUND` cannot
accept commands. At most two distinct motors can use independent feedback.
The old four-wheel/two-encoder mechanical relationship needs an explicit design
decision rather than four independent fictional encoder measurements.

Only the designated laptop command arbiter may publish `/motor/setpoints`.
Check `ros2 topic info /motor/setpoints --verbose` for exactly that publisher.
The existing legacy teleop/demo nodes publish older interfaces and do not yet
form this arbiter. The session field is a replay/freshness mechanism, not
cryptographic authentication or DDS access control. Use an isolated trusted
robot network or an appropriately configured DDS security deployment if
publisher access must be enforced against other network participants.

## Time and explicit arming

`issued_at_ms` is Unix wall-clock milliseconds at command creation, not receipt
time and not simulated ROS time. Synchronize laptop and Pi clocks with NTP/chrony.
The MCU synchronizes against the Pi bridge every 500 ms using bounded round-trip exchanges. Encoder and IMU stamps use
the sampled MCU monotonic time translated to the synchronized epoch; status
also includes wrap-safe 32-bit uptime. `imu_link` is the chip-axis frame, with
no mounting transform invented. Orientation is absent: covariance[0]=-1.
Zero acceleration/rate covariance matrices mean unknown uncertainty.

1. Observe the current status `session`. Use a fresh nonzero sequence, strictly
   increasing for every command within that session.
2. Send fresh **arm=false, all five targets zero** until status says `ready=true`.
3. On a deliberate operator arm action, send **arm=true, all targets zero**
   with a higher sequence. Wait for `armed=true`.
4. Continue `arm=true` targets at 50 Hz. Each channel is range/finite checked.
5. To disarm, send arm=false with all zero targets. Any malformed, stale,
   repeated, wrong-session, nonfinite or out-of-range command latches a stop.

Commands may be at most 100 ms old or 20 ms in the future. Missing fresh commands
for 250 ms disarm locally, normally by the next 5 ms control iteration. HAL tick
wrap does not extend this watchdog. Sequence zero/wrap is rejected; start a new
session by disarming and timing out before exhausting a sequence range.

Any link, E-stop, feedback, clock, invalid-command or deadline fault clears the
arming handshake and changes the challenge when appropriate. Fixing the cause,
reconnecting USB or restarting the bridge does not arm. The zero/disarm and
zero/arm handshake must be performed again using the new challenge. No callback
directly drives a motor: it overwrites a bounded mailbox consumed by the
200 Hz control task, which validates the command at execution time.

In the supplied hardware-unapproved profile, the first arming step cannot
succeed. This is intentional and cannot be bypassed by topic traffic.

## Scheduling and resource allocation

| Task / interrupt | Priority | Period / stack |
|---|---|---|
| E-stop EXTI4 | NVIC 0, above FreeRTOS API ceiling | Immediate output inhibit; no RTOS calls |
| Local control / PID / watchdog | FreeRTOS 6 | `vTaskDelayUntil`, 5 ms, 768 words = 3 KiB |
| Sensors / MPU6050 / detector | FreeRTOS 3 | 20 ms, 768 words = 3 KiB; bounded HAL bus waits |
| Serial communication | FreeRTOS 2 | Nominal 10 ms, 1024 words = 4 KiB |
| USB FS | NVIC 6 | ISR RX ring only; no RTOS calls |
| FreeRTOS SysTick / PendSV | NVIC 15 | 1 kHz; SysTick also advances HAL milliseconds |

Task stacks and depth-one command/control/sensor queues are static. The serial parser has a 128-byte payload limit and CRC32 checks.
The RTOS heap is 8 KiB, bounded newlib heap 1 KiB, and main/ISR stack reserve
4 KiB. The previous micro-ROS heap and middleware pools are no longer linked.
Measure runtime stack watermarks during repeated reconnects.

IWDG is fed only by the control task, nominally every 5 ms. Prescaler 64 and
reload 499 yield approximately one second at nominal 32 kHz LSI; the actual
reset interval depends on oscillator tolerance. It is not a guaranteed 250 ms
output stop when interrupts/scheduling are entirely stalled. A physical
independent stop/inhibit remains required.
