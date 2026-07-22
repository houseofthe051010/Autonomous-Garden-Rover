# Raspberry Pi 5 UART Motor Client

This is a normal Python 3 program for Thonny on Raspberry Pi OS. A Raspberry Pi
5 does not use MicroPython's `machine.UART` for this application. Linux exposes
the selected hardware UART as `/dev/ttyAMA1`, and
[`main.py`](main.py) accesses it through `pyserial`.

The complete current and planned wiring map is in [`PINOUT.md`](PINOUT.md).

## Raspberry Pi 5 UART setup

UART1 is selected because GPIO0 and GPIO1 are directly beside each other on
physical header pins 27 and 28. The Pi 5 debug UART and GPIO14/GPIO15 are not
used. GPIO0/GPIO1 are normally reserved for HAT identification, so do not use
this assignment with a HAT that has an identification EEPROM.

1. Add this line under `[all]` in `/boot/firmware/config.txt`:

   ```ini
   dtoverlay=uart1-pi5
   ```

2. Reboot the Pi.
3. Install pyserial and give the current user serial-port access:

   ```sh
   sudo apt update
   sudo apt install python3-serial
   sudo usermod -aG dialout "$USER"
   ```

4. Log out and back in after changing group membership.
5. Confirm that the port exists:

   ```sh
   ls -l /dev/ttyAMA1
   ```

The UART pins are selected by the Device Tree overlay, not by Python. The
variables at the top of `main.py` make the expected device, baud, and pins easy
to find. If a different Pi UART is enabled, update all four variables to match
that overlay.

The Pi 5 UART overlay names and pin assignments are documented by Raspberry Pi
in its [UART configuration documentation](https://www.raspberrypi.com/documentation/computers/configuration.html#configure-uarts)
and [official overlay reference](https://github.com/raspberrypi/firmware/blob/master/boot/overlays/README).

## Additional UART allocation

Keep UART1 on GPIO0/GPIO1 dedicated to the STM32 motor controller. A suitable
separate interface for the future BNO080 UART-SHTP connection is Pi 5 UART2:

```ini
dtoverlay=uart2-pi5
```

That overlay uses GPIO4 TX (physical pin 7), GPIO5 RX (physical pin 29), and
`/dev/ttyAMA2`. UART-SHTP uses 3,000,000 baud rather than this motor protocol's
115,200 baud. The BNO080 needs its own driver and must not share either UART's
TX/RX wires with the STM32.

UART3 on GPIO8/GPIO9 is allocated to the additional GD32, and UART4 on
GPIO12/GPIO13 is reserved for a later device. See [`PINOUT.md`](PINOUT.md) for
physical pin numbers, mode configuration, power wiring, and conflicts.

## Wiring

All signals are 3.3 V UART. Do not connect 5 V to either data line.

| Raspberry Pi 5 | Physical pin | STM32 Blue Pill |
| --- | ---: | --- |
| GPIO0 / UART1 TX | 27 | PA10 / USART1 RX |
| GPIO1 / UART1 RX | 28 | PA9 / USART1 TX |
| GND | 6 or another GND | GND |

The data lines cross: transmitter connects to receiver. Power both boards from
their intended supplies and connect their grounds.

## Thonny

1. Select the Pi's **Python 3** interpreter in Thonny, not MicroPython.
2. Open and run `main.py`.
3. The background service requests telemetry every 400 ms. This also feeds the
   STM32 host watchdog while leaving the `>>>` shell available.

Useful shell commands:

```python
ping()
caps()
status()
read_currents()
read_currents(1)

direction_a(1, 512)          # about 12.5% duty
direction_b(1, 1024)         # about 25% duty
drive_percent(2, "A", 10)
drive(2, "B", 4095)         # maximum; only use after low-duty testing

stop_motor(1)
stop_all()
monitor(10)
```

Direction `A` drives RPWM and direction `B` drives LPWM. These names are
deliberately neutral because motor polarity and bridge-output wiring determine
which one physically moves the rover forward.

`read_currents()` reports the STM32's 12-bit ADC values and measured pin
millivolts. It does not report amperes until each BTS7960 module's current-sense
ratio and any external resistor network have been measured and calibrated.

## Safety behavior

The STM32 stops both motors if valid host commands are absent for 1.5 seconds.
Closing Thonny, stopping the script, disconnecting UART, or a Pi failure should
therefore stop the motors. This watchdog is not a substitute for a physical
emergency stop that removes bridge power or disables both BTS7960 modules.

The matching wire protocol is documented in
[`../UART_PROTOCOL.md`](../UART_PROTOCOL.md).
