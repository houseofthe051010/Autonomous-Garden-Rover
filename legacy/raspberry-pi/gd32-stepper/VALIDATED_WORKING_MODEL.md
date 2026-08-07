# Validated GD32 Stepper-Control Model

## Validation record

This hardware/software arrangement was confirmed working on 2026-07-22:

- Raspberry Pi 5 running normal Python 3 through Thonny.
- Creality Ender-3 v4.2.2 mainboard with a GD32F303RET6 MCU.
- Custom rover firmware on the Ender board, with heaters disabled and stepper
  motion controlled through newline-delimited Marlin commands.
- Direct 3.3 V, 115200-baud UART between the Pi and GD32 board.
- Raspberry Pi serial device `/dev/ttyAMA3` opened successfully.
- The GD32 heartbeat was received, acknowledged by the Pi, and confirmed by the
  `Ender UART round trip verified` message.
- The Thonny command `zmove(4000)` successfully moved the Z stepper output by
  4,000 driver microsteps.

The heartbeat acknowledgement verifies communication in both directions:

```text
GD32 PA9 TX -> Pi GPIO9 RX -> HB sequence received
GD32 PA10 RX <- Pi GPIO8 TX <- M118 HB_ACK sequence
GD32 PA9 TX -> Pi GPIO9 RX -> HB_ACK_OK confirmation
```

## Validated wiring

| Raspberry Pi 5 | Physical pin | Ender-3 v4.2.2 | Function |
|---|---:|---|---|
| GPIO8 / UART TX | 24 | GD32 PA10 / USART1 RX | Commands to GD32 |
| GPIO9 / UART RX | 21 | GD32 PA9 / USART1 TX | Replies and telemetry |
| GND | 20 or another Pi GND | Ender GND | Common signal reference |

The Pi and Ender board are powered normally and share UART ground. Their power
rails are not connected together.

The UART is tapped at the Ender board's CH340 nets. The CH340 TXD output must be
isolated from GD32 PA10 before Pi GPIO8 drives that receive net. A USB host must
not be connected to the Ender micro-USB port while using these direct UART taps.

## Working software

The validated Raspberry Pi client is
[`stepper_controller.py`](stepper_controller.py). It opens `/dev/ttyAMA3` at
115200 baud, 8N1 and starts a background receive thread. Run it with Thonny's
**Local Python 3** interpreter, not a MicroPython interpreter.

The working shell test was:

```python
zmove(4000)
```

The command uses the Z driver's default speed of 120 RPM. With the firmware's
configured 400 Z steps per logical unit, the client transmits a relative 10-unit
move and tracks the tagged completion response asynchronously.

The same client implements these host commands:

```python
xmove(signed_steps, rpm=None)
ymove(signed_steps, rpm=None)
zmove(signed_steps, rpm=None)
emove(signed_steps, rpm=None)
move(axis, signed_steps, rpm=None, wait=False, timeout=None)
move_sps(axis, signed_steps, steps_per_second)
set_speed(axis, rpm)
enable_drivers()
disable_drivers()
stop()
status()
switch_status()
```

## Larger robot integration

This model is intended to become one UART subsystem in the rover's later main
control process. The current module can be imported after its directory is on
Python's module search path:

```python
import stepper_controller as gd32

gd32.zmove(4000)
gd32.status()
```

Importing the module opens `/dev/ttyAMA3` and starts its daemon receive thread.
Only one process or module instance may own `/dev/ttyAMA3` at a time. The future
robot program should leave heartbeat processing in that thread and send complete
movement transactions through this API; it should not stream individual STEP
pulses from the Pi.

Each additional MCU must use its own Linux serial device and dedicated TX/RX
pair. Keep device names, baud rates, locks, receive workers, and protocol parsers
separate so one slow UART cannot block the others.

## Implemented but not yet physically validated here

The firmware and client implement the following, but this validation session did
not independently confirm each item on physical hardware:

- Pi-controlled motion on the X, Y, and E driver outputs.
- Positive and negative direction for every output.
- Per-command RPM and steps-per-second selection over the full configured range.
- A 200-revolution command under the rover's installed mechanical load.
- X/Y/Z switch telemetry and callback behavior on the Pi.
- Emergency `M410` stop behavior during loaded motion.
- Long-duration operation, electrical-noise immunity, and driver temperatures.

These items should be added to this record only after hardware testing. Driver
current remains controlled by the board's VREF potentiometers, and all four
driver enable inputs remain physically shared.
