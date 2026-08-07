# Validated STM32 UART Motor-Control Model

## Validation record

This hardware/software arrangement was confirmed working on 2026-07-22:

- Raspberry Pi 5 running Raspberry Pi OS and normal Python 3 in Thonny.
- STM32F103C6 Blue Pill running the dual-BTS7960 controller firmware.
- Direct 3.3 V, 115200-baud UART between the Pi and STM32.
- The Pi opened `/dev/ttyAMA1` successfully.
- Pi pin control reported GPIO0 as `TXD1` and GPIO1 as `RXD1`.
- The Python client sent `PING`; the STM32 received it and returned `OK PONG`.
- The client printed `STM32 UART ROUND TRIP VERIFIED: OK PONG`.
- A motor-control shell command was reported to operate successfully.

The `PING` result verifies the complete path in both directions, not merely that
the Linux serial device exists:

```text
Pi GPIO0 TX -> STM32 PA10 RX -> STM32 command parser
Pi GPIO1 RX <- STM32 PA9 TX <- OK PONG response
```

## Validated wiring

| Raspberry Pi 5 | Physical pin | STM32 Blue Pill | Function |
| --- | ---: | --- | --- |
| GPIO0 / UART1 TX | 27 | PA10 / USART1 RX | Commands to STM32 |
| GPIO1 / UART1 RX | 28 | PA9 / USART1 TX | Replies and telemetry |
| GND | Any Pi GND | GND | Common signal reference |

GPIO0 and GPIO1 are HAT identification pins by default. The Pi 5-specific UART1
overlay deliberately reassigns them, so this model must not use a HAT
identification EEPROM on those pins.

The current power arrangement is:

```text
XL4005 5 V output -> Raspberry Pi 5 V and GND rails
Raspberry Pi 5 V  -> STM32 Blue Pill 5V
Raspberry Pi GND  -> STM32 Blue Pill GND
```

The exact physical Pi header positions used by the existing 5 V and ground
wires have not yet been recorded. See [`PINOUT.md`](PINOUT.md) for power limits
and warnings about connecting multiple 5 V sources.

## Pi configuration

`/boot/firmware/config.txt` contains:

```ini
[all]
dtoverlay=uart1-pi5
```

After reboot, these checks succeeded:

```sh
ls -l /dev/ttyAMA1
pinctrl get 0-1
groups
```

The observed configuration included `/dev/ttyAMA1`, `TXD1` on GPIO0, `RXD1`
on GPIO1, and membership in the `dialout` group. Raspberry Pi OS package
`python3-serial` version 3.5-2 was installed.

## Working software

The validated host client is [`main.py`](main.py). Run it with Thonny's regular
Python 3 interpreter, not a MicroPython interpreter.

Successful startup is explicit:

```text
STM32 UART opened: /dev/ttyAMA1 at 115200 baud (Pi TX GPIO0, RX GPIO1)
STM32 link established: OK PONG
STM32 UART ROUND TRIP VERIFIED: OK PONG
```

The client provides:

- Explicit bidirectional connection verification with `check_connection()`.
- Motor 1 and Motor 2 direction A/B control.
- PWM duty control from 0 through 4095 or 0 through 100 percent.
- Individual and all-motor stop commands.
- Raw ADC and millivolt current-sense telemetry.
- STM32 heartbeat, watchdog, and link-state monitoring.
- Bounded UART lock waits so an unavailable response does not block forever.
- `pulse()` for short tests that request a stop in a `finally` block.
- Opt-in PA0-PA3 encoder ADC parsing and shell controls.
- Latest installed tank-drive orientation: direction A moves both tracks
  backward; direction B moves both tracks forward.

## Recommended shell test

Raise the driven wheels and provide a physical bridge-power disconnect before
testing. Confirm the link first:

```python
check_connection()
status()
read_currents()
```

Use a bounded low-duty pulse instead of leaving a motor running while entering a
second shell command:

```python
pulse(1, "A", 6, 0.5)
pulse(1, "B", 6, 0.5)
pulse(2, "A", 6, 0.5)
pulse(2, "B", 6, 0.5)
```

Each call requests 6 percent duty for 0.5 seconds and then stops that motor.
Direction A drives RPWM and direction B drives LPWM. On the latest installed
tank-drive test, direction B is forward for both tracks.

## Validated mobile tank-drive deployment

The Raspberry Pi-hosted controller was deployed and checked on 2026-07-23.
The deployed process is:

```text
/usr/bin/python3 /home/aditya/rover-controller/rover_web.py
```

It runs as `rover-web.service`, which was observed as active, enabled at boot,
and running with zero automatic crash restarts after deployment. The current
LAN endpoints are:

```text
Main controls: http://192.168.1.190:8080/
Tank joystick: http://192.168.1.190:8080/mobile
```

The mobile controller applies this signed track mapping:

| Requested track motion | STM32 command |
| --- | --- |
| Right forward | Motor 1, direction B |
| Right reverse | Motor 1, direction A |
| Left forward | Motor 2, direction B |
| Left reverse | Motor 2, direction A |

Forward and backward command both tracks together. Left and right use
opposite track directions for an in-place turn. The joystick performs
differential mixing and normalizes the pair so neither side exceeds 100
percent.

Motor output is slew-limited rather than stepping directly to the requested
throttle. The default ramp is 80 percent per second, the supported range is
10-300 percent per second, and the selected value persists in:

```text
/home/aditya/rover-controller/rover-web-config.json
```

While a control is held, the browser refreshes its target every 100 ms. Missing
targets for 500 ms trigger an immediate all-drive stop. Releasing a control
ramps toward zero, while the page's `STOP` and `STOP ALL` controls stop
immediately. A revision check prevents an older queued ramp update from
overriding a newer stop request.

Live validation confirmed:

- The mobile route returned HTTP 200.
- The STM32 heartbeat was alive and motor telemetry reported both motors
  stopped.
- ODESC, GD32, and BNO080 links remained connected while the dashboard ran.
- A zero-output tank target was accepted.
- `MSTOP ALL` was acknowledged by the STM32.
- The final deployment action requested a global software stop.

No nonzero motion command was issued during this deployment verification.
Physical direction assignments are based on the separately observed motor
tests supplied for this rover.

The combined dashboard also reports the ODESC battery bus voltage at a
five-second interval. Its GD32 section has separate X, Y, and Z controls.
Those stepper moves may be queued from the page but execute sequentially because
each existing UART transaction waits for planner completion with `M400`.
After deployment, the live status API reported 35.12 V, zero pending GD32
moves, and both STM32 drive motors stopped.

## Thonny and Pi Connect behavior

If Thonny displays a new `>>>` prompt, Python has completed the previous call.
If the prompt is visible but typing does not work, this is remote-desktop focus,
not a UART failure. Click directly after the prompt or use Pi Connect's reset
keyboard control. The terminal fallback is:

```sh
cd /path/to/Autonomous-Garden-Rover/legacy/raspberry-pi/stm32-drive
python3 -i main.py
```

## Not yet validated or calibrated

- R_IS/L_IS conversion from millivolts to motor current in amperes.
- Current-sense behavior under meaningful motor load.
- Long-duration operation, electrical-noise immunity, and thermal limits.
- Emergency-stop hardware independent of the Pi and STM32 software.
- Additional GD32 and future UART connections in this combined wiring model.
- PA0-PA3 analog encoder wiring, calibration, and telemetry under motion. The
  firmware/client extension is build-tested, but these pins are not yet wired.

These items must not be described as validated until they are tested and added
to this record.
