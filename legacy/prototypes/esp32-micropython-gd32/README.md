# ESP32 MicroPython Motor Controller

[`main.py`](main.py) connects an ESP32 to the repurposed Ender-3 mainboard,
auto-detects UART wire orientation, acknowledges heartbeats, exposes motor-control
functions at the MicroPython REPL, and receives switch/completion events.

## Requirements

- ESP32 running MicroPython with `machine.UART` and `_thread` support.
- 3.3 V UART connection described in the [motor-controller pinout](../README.md#uart-wiring).
- GPIO18 left physically unconnected for safe receive-only orientation detection.
- Ender board flashed with [`ALLDRV1.BIN`](../gd32-marlin-stepper/README.md).

## GPIO configuration

```python
DATA_PIN_A = 16
DATA_PIN_B = 17
PROBE_TX_PIN = 18
STATUS_LED_PIN = 2
BAUDRATE = 115200
```

GPIO16 and GPIO17 may be connected in either orientation. During startup the code
uses unconnected GPIO18 as the UART TX pin and tests GPIO16, then GPIO17, for the
Ender's repeating heartbeat. Once detected, the heartbeat pin becomes ESP32 RX
and the other connected pin becomes ESP32 TX.

This software detection does not electrically isolate the CH340 output. Perform
the hardware isolation described in the parent README.

## Running

Run `main.py` from Thonny or place it at the root of the ESP32 MicroPython
filesystem for automatic startup. A successful session begins like this:

```text
Listening for Ender heartbeat on GPIO16...
Heartbeat found: ESP32 TX=GPIO17, RX=GPIO16
Four-driver UART worker started; returning to the MicroPython prompt.
Ender UART round trip verified
```

The script starts `link_worker()` on the second MicroPython thread and returns to
the normal `>>>` prompt. Re-running the script performs a soft reboot first, which
terminates the previous worker.

## Common commands

```python
move("X", 3200, 120)
move("Y", -3200, 120)
move_sps("Z", 800, 4000)
set_speed("E", 100)
emove(6400)

enable_drivers()
disable_drivers()
stop()
status()
switch_status()
```

See the [legacy UART protocol](../../../docs/protocols/gd32-stepper-uart-legacy.md)
for the complete API and message
formats.

## Configuration constants

```python
MOTOR_MICROSTEPS_PER_REVOLUTION = 3200.0
MAX_DRIVER_RPM = 1000.0
MAX_PENDING_MOVES = 8
AXIS_STEPS_PER_UNIT = {"X": 80.0, "Y": 80.0, "Z": 400.0, "E": 93.0}
```

Update both this table and the Ender Marlin configuration if the hardware
microstep mode or logical steps-per-unit settings change.

## Link behavior

- Heartbeat interval: 2 seconds from the Ender board.
- Round-trip timeout: 5 seconds.
- GPIO2 LED: on after confirmed round trips, off after timeout.
- UART receive buffer: 1024 bytes.
- Completion records: transaction ID, axis, and signed steps are verified.
- Switch callbacks run in the background worker and should return quickly.

The UART write lock prevents heartbeat acknowledgements and REPL movement
commands from corrupting each other's lines.

## Troubleshooting

### No heartbeat found

- Confirm the Ender firmware booted.
- Confirm a shared ground.
- Confirm 115200 baud and 3.3 V levels.
- Verify GPIO16/GPIO17 continuity to the GD32 TX/RX nets.
- Leave GPIO18 unconnected.
- Verify the CH340 output is isolated before the ESP32 drives the RX net.

### Link verifies but a motor does not turn

- Call `enable_drivers()`.
- Confirm motor wiring at the selected X/Y/Z/E socket.
- Check the relevant VREF potentiometer and driver temperature.
- Start with `move(axis, 3200, 60)`.
- Remember that all four enable inputs share one physical pin.

### Direction is reversed

Use a negative step count, reverse the motor connector only when powered off, or
change the associated Marlin `INVERT_*_DIR` setting and rebuild the firmware.
