# Direct USB serial validation — 2026-10-16

The STM32F411CEU6 now runs the direct serial v1 application. The Pi at
`pi@10.154.60.6` runs `stm32_bridge` in `~/minesweeper_serial_ws`; no micro-ROS
agent is needed or running. Its USB product is `STM32 direct serial`.
The existing micro-ROS application was backed up before replacement.

## Executed checks

| Check | Result |
|---|---|
| GNU Arm build and vector/flash bounds | PASS: application starts at 0x08004000, 28,360-byte BIN |
| Flash readback | PASS: byte-identical to the local tested BIN |
| WeAct bootloader preservation | PASS: 16,384 bytes at 0x08000000 identical before/after |
| Firmware linked RAM | 23,536 bytes / 128 KiB (17.96%); previous micro-ROS build 102,576 bytes |
| Pi package tests on laptop | 33 passed, including 500 randomized C/Python interoperability packets, fragmentation, CRC rejection, timestamp bounds, pseudo-terminal sync/commands, reconnect drops and sensor mapping |
| Pi package tests on ARM Pi | 33 passed; firmware protocol sources supplied explicitly for the cross-language test |
| Existing firmware control tests | 10,271 assertions passed with ASan/UBSan; mocked motor/MPU6050, bounded libc heap and project checks also passed |
| Raw serial, Pi ↔ MCU | 161 status packets in 8.015 s, 16 clock responses, zero CRC errors; final clock/link flags both set |
| Pi-local ROS status delivery | About 20.04 Hz; intervals 0.038–0.062 s in the observed sample |
| Laptop network ROS delivery | PASS for connectivity/contents: 113 messages in 30 s; throughput limited by current Wi-Fi conditions |
| Motor state throughout observations | Disarmed, hardware_ok=false, all five duties zero; no MotorCommand messages sent |
| Bridge restarts | Live telemetry recovered across process restarts; observed MCU uptime continued without reboot |

Firmware BIN SHA256:

```text
0eef3621896b656cc8719f39d183284a14f3e476be99d915cd748ea442ebd022
```

The serial task reported 829 words of stack headroom; the control task reported
642 words. Heap fields are raw FreeRTOS heap_4 counters. They stay zero before
its first allocation; all application tasks and queues are static, so these
zeros do not mean exhausted RAM.

## Pi load and limits

On this Raspberry Pi 3 B+ (Ubuntu 26.04.1, four ARM cores, 894 MiB visible RAM),
a 20-second steady-state sample measured the bridge at **17.00% of one CPU
core**, equivalent to about **4.25% of total four-core capacity**. RSS was
101,932 KiB, about **99.5 MiB**. This includes Python, ROS 2 and Fast DDS.
Serial polling runs at 50 Hz; unused parameter services and default QoS event
callbacks are disabled. Port/baud can still be selected at startup.

These are status-only bench measurements. No physical sensors or camera were
connected, and no motor commands were transmitted. This does not establish
full-load performance or a measured improvement over an agent on this Pi.
The reduction in linked STM32 RAM is established by the build results.

The raw USB/Pi-local path was about 20 Hz, but the laptop received about 3.8 Hz
in the 30-second Wi-Fi sample. Eight pings had no missing replies, three
duplicate replies, 13–205 ms RTT and 138 ms average. Signal readings were about
-81 dBm on Pi and -83 dBm on laptop; laptop Wi-Fi power saving was enabled.
These observations identify a network limitation, not lost serial framing.
The cause has not been isolated to a single setting. Command freshness limits
remain 100 ms old / 20 ms future and must not be relaxed to hide network delay.
Recheck with a stronger Wi-Fi link or Ethernet before motion commissioning.

## Running the deployed bridge

The bridge is currently running as a background process. Its files are:

- `~/minesweeper_serial_ws/start_bridge.sh`: foreground startup command with ROS environment.
- `~/minesweeper_serial_ws/results/bridge.pid`: current background PID.
- `~/minesweeper_serial_ws/results/bridge.log`: runtime log.
- `~/minesweeper_serial_ws/results/`: build, tests, flash readback, resource samples and backups.

After a Pi reboot, start it with:

```bash
bash ~/minesweeper_serial_ws/start_bridge.sh
```

No boot service was installed. Do not launch another copy while the background
bridge already owns the port. The Pi user was added to `dialout` for serial
access. The previous `~/rpi_ws` source tree was not replaced.

The standard repository launch is also available after sourcing the combined
workspace install:

```bash
source /opt/ros/lyrical/setup.bash
source ~/minesweeper_serial_ws/install/local_setup.bash
ros2 launch minesweeper_pi pi.launch.py mock:=false camera:=false serial_port:=/dev/ttyACM0
```

Detailed local evidence and backups are under
`artifacts/direct_serial_backup/`, including `raw_serial.json`,
`live_status_final.json`, `bridge_resources_steady.txt`, bootloader before/after
images and the previous application image. Build artifacts remain under
`firmware/stm32f411/Debug/`. See [deployment](STM32_DEPLOYMENT.md) and
[wire contract](../firmware/stm32f411/docs/SERIAL_PROTOCOL.md).

The commissioning hardware gate remains disabled. Physical motor/sensor tests,
full sensor/camera load, command delivery under network jitter, 100-cycle
reconnect endurance and power-cycle acceptance remain unexecuted.
