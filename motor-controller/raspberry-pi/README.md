# Raspberry Pi 5 Stepper Client

This CPython client lets a Raspberry Pi 5 control the Ender-3 v4.2.2 GD32
stepper board from Thonny. It uses the existing GD32 firmware and UART protocol;
the Ender firmware does not need to be reflashed when replacing the ESP32.

The Pi-to-GD32 round trip and a 4,000-step Z movement are hardware-confirmed.
See the dated [validated working model](VALIDATED_WORKING_MODEL.md).

## Wiring

The UART allocation shown in the Raspberry Pi configuration is:

| Pi 5 signal | Header pin | Device | Connect to Ender board |
|---|---:|---|---|
| GPIO8 / TX | 24 | `/dev/ttyAMA3` TX | GD32 PA10 / USART1 RX |
| GPIO9 / RX | 21 | `/dev/ttyAMA3` RX | GD32 PA9 / USART1 TX |
| GND | 20 or another GND | Ground | Ender GND |

The TX/RX descriptions are from the Pi's perspective: Pi TX goes to GD32 RX,
and Pi RX goes to GD32 TX. Do not connect either board's power rail to the other.
Power each board normally and connect only TX, RX, and a common ground.

Both UARTs use 3.3 V logic at 115200 baud, 8N1. Disconnect the ESP32 before
connecting the Pi.

### CH340 isolation

The GD32 UART is tapped at the onboard CH340 nets. CH340 pin 2 TXD is itself an
output connected to GD32 PA10 RX. That CH340 output must be electrically isolated
from the GD32 side of the net before Pi GPIO8 TX drives GD32 RX. Connect GPIO8 to
the **GD32/board side** of the isolation point. Do not attach a USB host to the
Ender micro-USB connector while using these direct UART taps.

## Pi setup

The UART overlay/allocation must expose `/dev/ttyAMA3`, as in the supplied Pi
configuration. Confirm it before running:

```bash
ls -l /dev/ttyAMA3
```

Install pyserial using either Thonny's package manager or Raspberry Pi OS:

```bash
sudo apt update
sudo apt install python3-serial
```

If opening the port reports permission denied, add the Pi user to `dialout`, then
log out and back in:

```bash
sudo usermod -aG dialout "$USER"
```

Do not run a serial console or `getty` on `/dev/ttyAMA3`; only this program should
open the port.

## Thonny use

1. Open [`stepper_controller.py`](stepper_controller.py) on the Pi.
2. Select **Local Python 3** as the Thonny interpreter, not MicroPython.
3. Run the file. A background thread handles heartbeat and incoming reports while
   the `>>>` shell remains usable.
4. Wait for `Ender UART round trip verified`, then issue commands.

```python
status()

xmove(3200, 120)             # X: one revolution forward at 120 RPM
ymove(-1600, 300)            # Y: half a revolution backward at 300 RPM
zmove(800, 30)               # Z: 800 driver microsteps
emove(-3200, 100)            # E: one revolution in the negative direction

move("X", 640000, 300)       # 200 revolutions
move_sps("Y", -1600, 8000)   # speed expressed as STEP pulses/second
move("Z", 3200, 60, wait=True, timeout=20)

set_speed("X", 250)
xmove(3200)                   # uses the saved X default of 250 RPM

enable_drivers()              # enables all four shared-enable drivers
disable_drivers()             # disables all four drivers
stop()                        # Marlin M410 quick stop
switch_status()
```

Positive and negative signed step counts select opposite directions. Which one
is physically clockwise depends on motor connector orientation and the firmware
axis inversion setting.

Register a callback for debounced X/Y/Z switch changes:

```python
def switches_changed(states):
    print(states)

on_switch_change(switches_changed)
```

Callbacks run in the UART reader thread and should return quickly. The switches
are telemetry only; they do not automatically stop motion.

## Hardware limits

All four driver enable inputs share one GD32 pin, so enable/disable affects every
driver. Driver current is set by the onboard VREF potentiometers. Microstep mode
and decay mode are fixed by PCB wiring; they cannot be changed over UART. See the
[complete UART protocol](../UART_PROTOCOL.md) for movement conversion and limits.
