# Validated ODESC UART Model

## Validation record

This hardware/software arrangement was confirmed working on 2026-07-22:

- Raspberry Pi 5 running Raspberry Pi OS and regular Python 3 in Thonny.
- ODESC identified over USB as `1209:0d32 Generic ODrive Robotics ODrive v3`.
- Pi UART2 exposed as `/dev/ttyAMA2` at 115200 baud, 8N1.
- Pi GPIO4 transmitted to ODESC GPIO2/UART RX.
- Pi GPIO5 received from ODESC GPIO1/UART TX.
- The ODESC responded to multiple ODrive ASCII protocol property reads.
- `config.enable_uart` returned `1` and `config.uart_baudrate` returned `115200`.
- Axis1 reported idle state `1` and axis error `0d` (decimal zero).

Observed successful values:

```text
ODESC UART IS WORKING
Bus voltage: 35.653049 V
config.enable_uart: 1
config.uart_baudrate: 115200
axis1.current_state: 1
axis1.error: 0d
```

Each value was returned in response to a command sent by the Pi. This verifies
Pi TX, ODESC RX and command parsing, ODESC TX, and Pi RX. Merely opening the
Linux serial device would not verify those paths.

## Validated wiring

| Raspberry Pi 5 | Physical pin | ODESC | Function |
| --- | ---: | --- | --- |
| GPIO4 / UART2 TX | 7 | GPIO2 / UART RX | ASCII commands to ODESC |
| GPIO5 / UART2 RX | 29 | GPIO1 / UART TX | Replies from ODESC |
| GND | Any Pi GND | GND | Common signal reference |

UART is 3.3 V logic. The ODESC was separately powered and reported a 35.653049 V
DC bus during validation. The Pi must not power the ODESC motor bus.

## Pi and ODESC configuration

The Pi configuration uses:

```ini
dtoverlay=uart2-pi5
```

The corresponding port is `/dev/ttyAMA2`. Do not substitute a UART4 overlay
while using GPIO4/GPIO5. The ODESC's saved configuration was checked over its
USB CDC interface before the Pi test and already contained:

```text
config.enable_uart = 1
config.uart_baudrate = 115200
```

No ODESC configuration change was required.

## Working software

The reusable validated client is [`odesc_uart.py`](odesc_uart.py). Running the
file directly executes non-motion property reads and prints raw TX/RX bytes.
Importing it provides `ODESCUART` for later integration into the rover API.

## Not yet validated

- Physical ODESC motor movement from the new Raspberry Pi velocity controls.
- Axis1 sensorless startup and operation in the rover drivetrain.
- Controller-level watchdog behavior after complete Pi power/process loss.
- Software and independent hardware emergency stops.
- Long-duration operation near motor wiring and immunity to electrical noise.
- Recovery following an ODESC reboot, USB connection, or UART disconnection.

These items must not be treated as working merely because the UART property
read round trip is validated.

## 2026-07-24 diagnostic extension

The deployed web service successfully read live extended telemetry without
moving either motor:

```text
vbus_voltage approximately 34.18 V
ibus 0.0 A
bus power 0.0 W
axis0 state 1, all errors 0
axis1 state 1, all errors 0
axis0 motor pre-calibrated true, encoder pre-calibrated false
axis1 motor pre-calibrated false, encoder pre-calibrated false
```

Both axes reported velocity-control mode `2`, passthrough input mode `1`, a
10 A current limit, and a 15 turns/s velocity limit. The missing encoder
calibration explains why the former state-8 encoder velocity path was not
appropriate, but the clone also exposes the older state-5 sensorless path.

Additional read-only properties established:

```text
firmware 0.0.0 unreleased (custom clone build)
axis0 motor pre-calibrated true
axis0 sensorless ramp 4 A, approximately 5.00 mechanical turns/s
axis0 sensorless acceleration approximately 1.82 turns/s^2
axis0 pole pairs 7, PM flux linkage 0.004633 Wb
axis1 motor pre-calibrated false
```

`axis*.config.enable_sensorless_mode` is absent, while legacy
`axis*.config.startup_sensorless_control` and requested state `5` are present.
The driver and web server therefore use the legacy state-5 open-loop startup
ramp, enforce its minimum running speed, reject live direction reversal, and
stop by requesting IDLE. Axis 1 is blocked until deliberate motor-only
calibration.

The deployed web service was validated with no motor movement:

- all four UART links remained connected after deployment;
- axis 0 and axis 1 remained in IDLE with all reported faults zero;
- a read-only ODESC status press was persisted as a successful event;
- an intentionally wrong arm phrase was rejected and persisted as an error;
- the service remained active with `NRestarts=0`.

Physical motor movement remains outside this validated record until a
raised-mechanism test is performed with an independent power disconnect ready.
