# Raspberry Pi 5 ODESC UART Client

This directory contains the regular Python 3/Thonny client for the rover's
ODESC (ODrive v3-compatible) motor controller. The bidirectional UART link was
hardware-validated on 2026-07-22. See
[`VALIDATED_WORKING_MODEL.md`](VALIDATED_WORKING_MODEL.md) for the observed
results.

## Wiring

| Raspberry Pi 5 | Physical pin | ODESC | Direction |
| --- | ---: | --- | --- |
| GPIO4 / UART2 TX | 7 | GPIO2 / UART RX | Pi to ODESC |
| GPIO5 / UART2 RX | 29 | GPIO1 / UART TX | ODESC to Pi |
| GND | Any Pi GND | GND | Common reference |

TX and RX must cross. Both devices use 3.3 V UART logic; do not put 5 V on a
UART signal. Power the ODESC from its intended motor power supply and connect
the grounds.

## Raspberry Pi setup

Add this under `[all]` in `/boot/firmware/config.txt`:

```ini
dtoverlay=uart2-pi5
```

Reboot, then verify:

```sh
ls -l /dev/ttyAMA2
pinctrl get 4-5
```

Install the only Python dependency:

```sh
sudo apt update
sudo apt install python3-serial
```

## Run the test

Open [`odesc_uart.py`](odesc_uart.py) in Thonny using its regular local Python 3
interpreter and run it. A successful result includes:

```text
ODESC UART ROUND TRIP VERIFIED
Bus voltage: 35.653049 V
UART enabled=1 baud=115200 | axis1 state=1 error=0
```

The voltage is only an example from the validation run and will follow the
connected motor supply. State `1` means idle. The firmware sometimes formats a
zero integer as `0d`; the client handles that representation.

## Robot API use

The client can be imported without performing I/O automatically:

```python
from odesc_uart import ODESCUART

odesc = ODESCUART()
odesc.open()
status = odesc.check_connection()
voltage = odesc.read_float("vbus_voltage")
odesc.close()
```

`query()` serializes access with a lock, validates one-line commands, and
applies a bounded timeout. The combined rover service keeps one `ODESCUART`
instance open so no other process may own `/dev/ttyAMA2`.

The client now provides:

- `telemetry()` for `vbus_voltage`, `ibus`, bus watts, and both axis snapshots.
- Per-axis position, velocity, measured/setpoint Iq, limits, calibration flags,
  state, component errors, sensorless-estimator velocity/error, motor pole
  pairs, flux linkage, and startup-ramp values.
- `configure_velocity_axis()` for volatile velocity mode and current/speed
  limits.
- `configure_sensorless_axis()` for legacy state-5 ramp settings expressed as
  mechanical turns/s and converted to electrical radians/s internally.
- `start_sensorless()`, `set_sensorless_velocity()`, and `stop_sensorless()`,
  including minimum-speed, fixed-direction, and IDLE-stop enforcement.
- `calibrate_motor()` for an encoderless axis that needs motor-only calibration.
- `clear_errors()` and guarded `calibrate_axis()`.
- Closed-loop state requests and signed velocity commands through the combined
  rover controller.

The deployed dashboard adds an 800 ms host-process velocity deadman, explicit
arm/calibration phrases, and persistent command/error records in
`~/rover-controller/rover-history.sqlite3`. Motor motion from the sensorless
controls has not yet been physically validated. The deadman cannot protect
against Pi power loss or every process failure; use an independent hardware
emergency stop.
