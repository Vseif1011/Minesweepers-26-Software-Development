# Flashing and debugging

## Session status

The STM32F411CEU6 is connected via USB-C and enumerates as **WeAct Studio HID
Bootloader**, VID:PID **0483:572a**. No ST-Link is attached. That HID device is
neither a CDC serial micro-ROS application nor an ST-Link debug probe.

The [manufacturer's bootloader documentation](https://github.com/WeActStudio/WeActStudio.MiniSTM32F4x1/blob/master/Soft/WeAct_HID_FW_Bootloader/README.md)
specifies a 16 KiB bootloader and application entry at `0x08004000`. This project's
linker script and relocated vector table use that address, preserving sector 0.
Do not flash the application at `0x08000000`, use full-chip erase or change option
bytes for this build. There is no mass-erase step in the supplied workflow.

Initially in this session, opening `/dev/bus/usb/001/005` for writing and `/dev/hidraw2`
was denied to `cypher`; noninteractive sudo is unavailable. The manufacturer's
old Linux CLI detected the board but crashed during a read-only version query
without device permissions. After the user granted a temporary ACL and confirmed
motor power disconnected, the supplied wrapper successfully flashed and rebooted
the board. It enumerated as CDC `0483:5740`, serial
`005A00573133510836373538`, on `/dev/ttyACM0`. The wrapper checks access before
launching the vendor binary. The normal application serial port requires its
own access permission (dialout membership or an ACL); the bootloader ACL does
not carry over to a different USB device. The persistent rules were subsequently
installed. The final image (with MPU6050 offsets and bounded libc heap) was
flashed, and live Lyrical Agent status/restart checks passed.

## USB HID flashing with the existing USB cable

First physically isolate motor power. GPIO reset/input mode cannot establish
the safe state of an unidentified external driver. The shipped build never
starts PWM or arms, and initializes only the sensor/USB/E-stop peripherals
specified in HARDWARE.md.

An administrator must grant this account device access. For the currently
enumerated device, a temporary ACL is sufficient (expires on reconnect):

```bash
sudo setfacl -m u:cypher:rw /dev/bus/usb/001/005
```

If `setfacl` is missing, install the `acl` package. The bus/device number can
change; find the current one with `lsusb -d 0483:572a` before applying the ACL.
For persistent access, from this firmware directory:

```bash
sudo install -m 644 scripts/99-minesweeper-stm32.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo usermod -aG plugdev,dialout "$USER"
```

Log out/in if group membership changed, then reconnect the USB cable. The
current account already belongs to `plugdev` in this session.

With motor supply isolated and the build reviewed:

```bash
python3 scripts/inspect_image.py
bash scripts/flash_hid.sh --motor-power-isolated
```

The script downloads the manufacturer's x86-64 Linux CLI with a pinned SHA256,
checks the device and application vector bounds, writes only the application
image using the resident bootloader and requests a reboot. This proprietary
vendor utility is not redistributed as source or claimed to perform verified
flash readback. A “Flash Done” message needs subsequent USB/Agent/status checks.
Do not call `erase`; do not use the unrelated Serasidis `hid-flash` protocol.

After reboot, expect a CDC application (example VID:PID `0483:5740`), visible
under `/dev/serial/by-id/` or `/dev/ttyACM*`. Its descriptor is
“STM32 commissioning micro-ROS”. If it remains HID, check oscillator start,
bootloader handoff and ELF vector values. To re-enter the existing bootloader,
follow the board manufacturer's KEY/reset procedure; BOOT0 enters ST's ROM
bootloader, which is a different protocol.

## STM32CubeIDE and ST-Link

For interactive debugging, connect a genuine/compatible ST-Link to board
**3.3 V reference, GND, PA13/SWDIO, PA14/SWCLK**, plus **NRST** if available.
Identify those connector pins on the actual probe; cable colors are not a
pinout. Keep motor power isolated, and avoid conflicting power supplies.

1. Import the project and build as described in README.md.
2. Open **Run → Debug Configurations → STM32 C/C++ Application**, create a
   configuration for project `Minesweeper_STM32F411` and executable
   `Debug/Minesweeper_STM32F411.elf`.
3. Select ST-Link, SWD, initially 1 MHz. Use connect-under-reset if needed.
   Select the actual F411CE target. Load the ELF's sections at their linked
   addresses; erase only affected application sectors. Do not select mass erase.
4. Break at `main`, then `control_task`, `uros_task`, and `on_command`. Inspect
   `app_control_heartbeat`, task stack watermarks, USB state and safety snapshots.
   Hardware-unapproved must remain set in the commissioning image.
5. Reset and run through the resident bootloader for a normal boot test. Verify
   `SCB->VTOR == 0x08004000`, SYSCLK=96 MHz, USB clock=48 MHz and 1 kHz SysTick.

An alternative with a separately installed OpenOCD and an attached ST-Link is:

```bash
openocd -f interface/stlink.cfg -c 'transport select hla_swd' \
  -f target/stm32f4x.cfg \
  -c 'program Debug/Minesweeper_STM32F411.elf verify reset exit'
```

Check the installed ST-Link interface's supported transport if it uses `dapdirect_swd`
instead of `hla_swd`. STM32CubeProgrammer also accepts the ELF over ST-Link,
with section download and verification at the linked application address.

IWDG is deliberately not frozen automatically by firmware. A breakpoint can
cause a reset in about one second. For bench debugging only, with motor power
physically disconnected, enable CubeIDE's watchdog-freeze option or set the
IWDG debug-freeze bit through the debugger. Never interpret halted CPU outputs
as a tested emergency-stop behavior.
