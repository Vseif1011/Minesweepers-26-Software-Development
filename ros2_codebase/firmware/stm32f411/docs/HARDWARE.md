# Confirmed facts and wiring plan

**Motor wiring update:** the supplied Cytron MDD10A PWM/DIR diagram supersedes
the earlier proposed motor/ENABLE/E-stop allocation below. See the
[current motor setup](../../../docs/MOTOR_CONTROL_SETUP.md). The explicit motor
profile has no STM E-stop input; the default commissioning profile remains locked.

The user confirmed **STM32F411CEU6** connected through the board USB-C connector,
PWM pins PA0, PA1, PA2, PA3 and PA8, MPU6050, and two
`YT06-OP-600B-6-24V-2M` rotary encoders, and a 25 MHz crystal. USB inspection found WeAct Studio HID
bootloader `0483:572a`, serial `3691358F3133`. The USB ID database's “F401” label
is not a chip identification measurement. The package/flash target now matches
the user's chip marking.

The following **chosen signal allocation** uses peripheral alternate functions
from the [ST F411 datasheet](https://www.st.com/resource/en/datasheet/stm32f411ce.pdf).
The user clarified that no motors or sensors are currently connected: this is
a bench firmware project for future wiring. The allocation below is the future
connection plan, not a report of existing peripheral wiring.

| Signal | STM32 pin | Peripheral / status |
|---|---|---|
| Motor 1 PWM | PA0 | TIM2 CH1, AF1; user confirmed |
| Motor 2 PWM | PA1 | TIM2 CH2, AF1; user confirmed |
| Motor 3 PWM | PA2 | TIM2 CH3, AF1; user confirmed |
| Motor 4 PWM | PA3 | TIM2 CH4, AF1; user confirmed |
| Motor 5 PWM | PA8 | TIM1 CH1, AF1; user confirmed |
| Encoder 0 A / B | PA6 / PA7 | TIM3 CH1/CH2, AF2; chosen, input conditioning pending |
| Encoder 1 A / B | PB6 / PB7 | TIM4 CH1/CH2, AF2; chosen, input conditioning pending |
| MPU6050 SCL / SDA | PB8 / PB9 | I2C1, AF4; chosen, 100 kHz, external pullups to 3.3 V |
| MPU6050 AD0 | GND | Chosen address 0x68; verify actual breakout/module |
| E-stop sense | PA4 | NC auxiliary loop to GND; open loop is STOP, pullup enabled |
| Conditioned detector output | PA5 | Reserved ADC1 IN5; NOT configured or connected to a bare coil |
| USB D− / D+ | PA11 / PA12 | Dedicated USB FS peripheral on the board USB connector |
| SWDIO / SWCLK | PA13 / PA14 | Reserved for ST-Link |
| Crystal | PH0 / PH1 | 25 MHz HSE, user confirmed |

Potential **reserved** driver control pins, only if the actual drivers use
PWM + DIR + independent ENABLE: DIR 1–5 = PB0, PB1, PB2, PB10, PB12;
ENABLE 1–5 = PB3, PB4, PB5, PB14, PB15. These are proposals, remain unconfigured,
and must be revised for drivers with IN1/IN2, RPWM/LPWM, active-low PWM,
braking requirements, shared enables or other electrical interfaces.
PB3 would preclude SWO trace; SWD remains available. No driver control pin has
been silently assigned in the executable board binding.

## Electrical information still required

- Driver model, input voltage thresholds, PWM polarity and frequency range,
  brake/coast semantics, independent inhibit input, reversal deadtime, current
  limits, motor role and sign for each of the five channels.
- Which motors actually have encoder feedback. Only two channels may use the
  independent encoder velocity PIDs. No wheel/arm role is inferred from numbering.
- Encoder manufacturer's output schematic, supply voltage, cable conductors,
  pulses/revolution, maximum speed and output edge rate. The model string alone
  is not accepted as a verified datasheet. Do not connect a 6–24 V output
  directly to the STM32. Use a qualified 3.3 V receiver/level interface; for a
  confirmed open-collector output, verify pullup and sink ratings before using
  3.3 V pullups. Differential outputs need a receiver.
- The selected encoder timer mode is TI12 quadrature, both inputs, 16-bit wrap,
  no digital filter. It normally gives four counts per A/B cycle; measure the
  actual count scale and polarity. No counts/revolution, speed limit or sign is
  filled in until measured. Encoder bindings therefore remain disabled.
- The coil needs a metal-detector front end: excitation, protection, analog
  conditioning and a defined voltage/digital/frequency output. A passive coil
  connected to a GPIO/ADC is not an implemented metal detector. PA5 is a reserved
  input for a future bounded analog output, not a coil drive pin. `metal.c`
  already supports conditioned ADC or debounced digital inputs; the actual
  front-end contract determines which one is bound. Frequency/pulse readout
  requires a timer-capture driver and a revised allocation.
- Physical E-stop contacts must inhibit motor energy/driver enable without MCU,
  ROS, USB or Pi cooperation. PA4 is an auxiliary sense input. Specify the actual
  circuit, reset states and external biasing. An internal pullup is sufficient
  for software commissioning, not a substitute for that electrical design.

## Active commissioning configuration

Only USB, I2C1, the encoder input peripheral configuration and PA4 E-stop sense
are initialized. PWM pins PA0–PA3 and PA8 retain reset input mode. Timer PWM is
never started. Direction/enable pins are not touched. `BOARD_HARDWARE_APPROVED=0`
and `BOARD_PWM_FREQUENCY_HZ=0` cannot be overridden by a ROS command.

The MCU uses the user-confirmed 25 MHz HSE crystal with PLLM=25, PLLN=192,
PLLP=2 and PLLQ=4: SYSCLK=96 MHz, APB1=48 MHz, timer clocks=96 MHz and USB=48 MHz.
HSI remains enabled as the startup clock. Check oscillator start and USB
operation on the actual board; the physical clock has not yet been measured.

The MPU6050 driver checks WHO_AM_I, resets the sensor, checks configuration
readback, uses a 50 Hz sample rate, DLPF 3, ±2 g and ±250 °/s ranges, and reads
only fresh, nonsaturated samples. It publishes raw acceleration in m/s² and
angular rate in rad/s, in the chip axes. AD0 must match the chosen 0x68 address.
No orientation or mounting rotation is fabricated. The user supplied hardware
offset-register values in `mpu6050_config.h`: accel X/Y/Z = -2789/-4452/1391,
gyro X/Y/Z = 97/-40/-51. Initialization writes those exact signed words after
reset and verifies readback. These match the I2Cdevlib setter semantics, not
a subtraction of raw counts or SI-unit measurements. The values are specific
to the user's calibrated sensor and require physical validation when connected. Sensor fusion and further bias/mounting estimation belong on the laptop. A failed initialization requires
reset after correcting wiring; missing samples do not become zero measurements.

## Completing the motor profile

Keep motor power isolated throughout these steps. Fill the board binding only
from verified hardware data, including encoder signs/scales/bounds and motor
limits/gains. Configure the actual direction/enable topology and inactive levels.
Set the driver-supported PWM frequency, then review `BOARD_HARDWARE_APPROVED`.
The existing driver supports active-high PWM, DIR and independent ENABLE only;
unsupported topologies deliberately fail validation rather than guessing.

The initialization sequence preloads inactive enables before output mode,
initializes DIR, configures zero PWM, muxes the confirmed PWM pins, then starts
timers with enables inactive. All channels sharing TIM2 share a PWM frequency;
TIM1 also uses that frequency. TIM2 is reserved exclusively for four PWM
channels, TIM1 for the fifth, TIM3/TIM4 for encoders, and SysTick for the RTOS.
There is no timer channel overlap. The manufacturer's WeAct bootloader source
also uses PA0 for KEY, sharing the user-selected motor 1 PWM pin. Resolve that
physical button circuit during hardware sign-off; KEY must not short an actively
driven PWM output. This does not affect the input-only commissioning profile. Recheck every AF/pin against the chosen
driver topology; the remaining pins are a limited resource.

Measure all items in VALIDATION.md with a scope and disconnected motor supply
before an explicit operator arming sequence is permitted.

## Legacy source evidence (searched after the user's clarification)

`catkin_ws/src/all_code_reformatted/all_code_reformatted.ino` defines four
PWM+DIR motors (Mega PWM 13/12/7/5, DIR 22/24/26/28), a gripper on PWM 9/DIR 8,
a separate magnet output on pin 6, and an active-high **digital** detector
input on pin 4. The newer Mega sketch preserves that interface.
`rpi_ws/src/motors/src/motor_controller.py` uses four PWM+DIR pairs and 100 Hz
software PWM; its README says that path was abandoned because of Pi timing.
Those are old platform pin numbers and software settings, not STM32 wiring or
verified driver frequency/polarity requirements. No driver model, enable/brake
schematic, encoder output circuit, detector front end or E-stop circuit was
found in source or hardware documents. The only sensor datasheets found are
for the MPU6050. Thus the old 100 Hz value and the absence of an enable pin
cannot establish a safe driver configuration for the current robot.

The legacy detector's digital input would use the existing digital adapter
rather than an ADC once the actual front end is confirmed. PA5 can serve either
input type. It remains unconfigured until that circuit is identified.

The offset writes follow the original [I2Cdevlib MPU6050 setters](https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050.cpp):
accelerometer words at 0x06/0x08/0x0A and gyro words at 0x13/0x15/0x17.
All six words are written after sensor reset, then verified byte for byte.
