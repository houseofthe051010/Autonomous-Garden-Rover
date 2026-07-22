# Motor Controller UART Protocol

## Transport

- Electrical level: 3.3 V UART
- Format: 115200 baud, 8N1
- Framing: newline-delimited ASCII
- Ender serial port: GD32 USART1 on PA9/PA10
- ESP32 port: UART2 with GPIO16/GPIO17 auto-orientation

Do not stream individual STEP pulses over UART. A movement command describes the
requested pulse count and speed; Marlin's hardware timer generates the pulses.

## Link messages

### Heartbeat

The Ender board sends a heartbeat every two seconds:

```text
HB 42
```

The ESP32 responds with standard Marlin `M118`:

```text
M118 HB_ACK 42
```

The Ender confirms the full round trip:

```text
HB_ACK_OK 42
```

The ESP32 link LED is on while valid acknowledgements arrive. It declares the
round trip down after five seconds without confirmation.

### Switch state

The Ender board sends all three debounced switch states on change and once per
second:

```text
SW X0 Y1 Z0
```

`1` means pressed/triggered according to the configured stock Ender switch
polarity. Switch reports do not stop normal motion automatically.

### Move transaction

For `move("X", 3200, 120)`, the ESP32 calculates units from the configured axis
steps-per-unit and sends a relative Marlin transaction similar to:

```text
G91
G0 X40.00000 F4800.00
M400
M118 DRV_DONE 1 X 3200
```

`M400` waits until physical planner motion completes. The final message is then
returned by Marlin:

```text
DRV_DONE 1 X 3200
```

The ESP32 checks the transaction ID, axis, and signed step count before marking
the movement complete.

## MicroPython API

Run [`esp32-micropython/main.py`](esp32-micropython/main.py). Its UART worker runs
in a background thread and returns control to the `>>>` prompt.

### Move by RPM

```python
move("X", 3200, 120)       # positive direction, one revolution
move("Y", -1600, 300)      # negative direction, half a revolution
move("Z", 800, 30)
move("E", -3200, 100)
```

Signed steps select direction. Physical clockwise/counterclockwise behavior also
depends on motor connector orientation and Marlin's per-axis inversion setting.

Convenience functions:

```python
xmove(3200, 120)
ymove(-3200, 120)
zmove(800, 30)
emove(1600, 100)
forward("X", 800, 60)
backward("X", 800, 60)
```

### Move by pulse rate

```python
move_sps("Y", -1600, 8000)
```

The third argument is driver STEP pulses per second.

### Default speeds

```python
set_speed("X", 250)
xmove(3200)                 # uses 250 RPM
```

Default speed is 120 RPM for every driver. The API cap is 1000 RPM.

### Enable and emergency stop

```python
enable_drivers()            # enables all four due to shared PC3
disable_drivers()           # disables all four
stop()                      # Marlin emergency M410 quick stop
status()
```

### Switch state and callback

```python
switch_status()

def switches_changed(states):
    if states["X"]:
        print("X switch pressed")

on_switch_change(switches_changed)
```

Callbacks execute in the UART worker thread. Keep them short. Exceptions are
caught and printed so they do not terminate link processing.

## Step conversion

The firmware and ESP32 currently use:

| Axis | Configured steps/unit | Units per 3200-step revolution |
|---|---:|---:|
| X | 80 steps/mm | 40 mm |
| Y | 80 steps/mm | 40 mm |
| Z | 400 steps/mm | 8 mm |
| E | 93 steps/mm | 34.4086 configured mm |

The API assumes 200 full motor steps/revolution and 1/16 microstepping, producing
3200 STEP pulses per revolution.

Two hundred revolutions are 640,000 microsteps:

```python
move("X", 640000, 300)      # 200 revolutions forward
move("X", -640000, 300)     # 200 revolutions backward to start
```

Moving directly from +200 revolutions to -200 revolutions spans 400 revolutions:

```python
move("X", -1280000, 300)
```

| Speed | Time for 200 revolutions | Pulse rate |
|---:|---:|---:|
| 120 RPM | 100 s | 6,400 steps/s |
| 300 RPM | 40 s | 16,000 steps/s |
| 600 RPM | 20 s | 32,000 steps/s |
| 1000 RPM | 12 s | 53,333 steps/s |

Start at a moderate speed and verify motor current, temperature, torque, and
missed steps under the rover's real mechanical load.

## Motion limits

The current protocol provides large finite moves, not a true indefinite velocity
mode:

- Each `move()` contains a finite signed step count.
- At most eight transactions are tracked by the ESP32 at once.
- `M400` drains the planner for each transaction, so separate transactions stop
  and accelerate again instead of forming a perfectly continuous stream.
- Marlin stores each planner block's step count in a 32-bit field and uses
  floating-point logical positions.
- Software endstops and the cold/length extrusion restrictions are disabled.
- Configured XYZ coordinate bounds are -1,000,000 to +10,000,000 units, but very
  large floating-point positions lose single-microstep precision.

The required rover range of -200 to +200 motor revolutions is comfortably inside
these limits. True simultaneous, independent, run-until-stopped speeds on all
four drivers would require a separate timer/phase-accumulator firmware mode.

## Driver and mechanical limits

The firmware's 1000 RPM cap is not a guarantee that a motor can reach 1000 RPM.
Available torque falls as speed rises. Reliable speed depends on motor winding,
24 V supply behavior, driver VREF/current, wiring, load inertia, and acceleration.

Endstop telemetry does not enforce travel. Never test hundreds of revolutions
while a motor is connected to a short printer axis or constrained rover mechanism.
