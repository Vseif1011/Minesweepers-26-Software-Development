# Direct USB serial protocol v1

The active firmware and Pi `stm32_bridge` use this protocol. Micro-ROS and its
agent are not part of this path. USB CDC appears as `/dev/ttyACM*`; the USB
product string is `STM32 direct serial`. CDC baud is nominally 115200.

## Frames

All multi-byte numbers are little endian. Floats are IEEE754 binary32. There
is no C structure padding on the wire:

```text
A5 5A | version:u8=1 | type:u8 | payload_length:u16 | payload | CRC32:u32
```

CRC32 uses the reflected IEEE polynomial `0xEDB88320`, initial/final XOR
`0xFFFFFFFF` (Python `zlib.crc32`). It covers version, type, length and payload,
excluding magic and CRC. Maximum payload is 128 bytes. Receivers handle split
and concatenated packets, reject unsupported versions/oversized lengths/bad
CRC, and scan for the next magic. Partial receive state expires after a 100 ms
inter-byte gap. The MCU USB RX ring is 2048 bytes; overflow clears parser/link
state. USB writes have bounded waits and never overwrite a busy TX buffer.

| Type | Direction | Python struct format | Meaning |
|---|---|---|---|
| 1 | MCU → Pi | `<I` | Clock request: MCU tick token |
| 2 | Pi → MCU | `<IQ` | Echo token, Pi Unix epoch milliseconds |
| 3 | Pi → MCU | `<IIQB5f` | Session, sequence, original issued_at_ms, arm (0/1), five targets |
| 4 | MCU → Pi | `<IIIIB5fIIII` | Session, accepted sequence, faults, uptime_ms, flags, five duties, control stack free words, communication stack free words, RTOS heap free/min bytes |
| 5 | MCU → Pi | `<QI2q2f` | Sample epoch_ms, MCU sample tick, two int64 counts, two rad/s velocities |
| 6 | MCU → Pi | `<QI6f` | Sample epoch_ms, MCU sample tick, acceleration xyz in m/s², gyro xyz in rad/s |
| 7 | MCU → Pi | `<f` | Valid conditioned detector reading |

Status flag bits 0–7 are armed, ready, hardware_ok, encoders_valid, imu_valid,
metal_valid, clock_ok and link_ok. Unknown types or incorrect payload sizes are
ignored. Status is nominally 20 Hz; valid sensor streams are up to 50 Hz.
Unavailable, stale or unconfigured sensors emit no samples. Clock-dependent
samples require a synchronized clock.

## Clocks, commands and reconnects

The MCU sends a fresh tick token every 500 ms. A response is accepted once only,
with the same pending token, a round trip of at most 40 ms and a plausible Unix
epoch (2023–2099). It estimates return transit time as half the round trip.
Clock/link validity expires after 1500 ms without a successful exchange. This
is a bounded estimate, not a precision clock synchronization guarantee.
Pi and laptop must also have synchronized clocks.

The Pi forwards a ROS motor command only with matching status younger than
150 ms, finite targets, nonzero session/sequence and an original timestamp no
more than 100 ms old or 20 ms in the future. It does not rewrite that timestamp,
create commands, replay cached commands or rearm after reconnection. The MCU
performs the existing independent checks at the 200 Hz control task. Missing
fresh commands still cause a local stop after 250 ms. The explicit zero/disarm
then zero/arm handshake and hardware approval gate are unchanged.

The ROS message schema is preserved. `MotorStatus.uros_stack_free_words` is a
legacy field name: in this transport it reports serial-task stack headroom.
Heap fields expose the raw FreeRTOS counters, not the former separate micro-ROS heap.
With all tasks/queues statically allocated, heap_4 is never initialized and these
counters remain zero; this does not indicate exhausted memory.
The communication task stack is 1024 words; the 48 KiB micro-ROS heap is removed.

## Build and deployment

Build firmware using the project Makefile and GNU Arm GCC. Build both
`minesweeper_interfaces` and `minesweeper_pi` on the Pi, then run:

```bash
ros2 launch minesweeper_pi pi.launch.py mock:=false camera:=false serial_port:=/dev/ttyACM0
```

The old Mega ASCII bridge and the former micro-ROS agent are incompatible with
this firmware. See [current deployment](../../../docs/STM32_DEPLOYMENT.md).

The Pi polls serial at 50 Hz. Startup port/baud parameters remain available,
but runtime parameter services and unused default QoS event callbacks are
disabled to reduce executor overhead on Pi 3.
