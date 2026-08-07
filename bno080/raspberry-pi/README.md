# Raspberry Pi 5 BNO080 UART-SHTP Client

This client reads fused and raw BNO080 data through Pi 5 UART4. It is normal
Python 3 code for Raspberry Pi OS and Thonny, not MicroPython.

The UART-SHTP round trip and fused sensor-report path were hardware-validated on
2026-07-22. See [`VALIDATED_WORKING_MODEL.md`](VALIDATED_WORKING_MODEL.md) for
the observed output, exact wiring, and remaining calibration work.

## Wiring

| Raspberry Pi 5 | Physical pin | BNO080 |
| --- | ---: | --- |
| GPIO12 / UART4 TX | 32 | RX, sometimes labelled SCL/RX |
| GPIO13 / UART4 RX | 33 | TX, sometimes labelled SDA/TX |
| GPIO22 | 15 | H_INTN, optional but recommended |
| GPIO23 | 16 | NRST, optional for the first test |
| GND | Any GND | GND |

UART signals are 3.3 V. Set UART-SHTP mode before sensor reset or power-up:

```text
PS1 = HIGH
PS0 = LOW
```

Breakout-board power inputs differ. Do not connect its power pin until the exact
module documentation confirms whether the pin expects regulated 3.3 V or a
higher `VIN`. A bare BNO080 uses 3.3 V-class power and logic.

## Enable UART4

Under `[all]` in `/boot/firmware/config.txt`:

```ini
dtoverlay=uart4-pi5
```

Save, reboot, and verify:

```sh
ls -l /dev/ttyAMA4
pinctrl get 12-13
```

Expected pin functions are UART4 TX on GPIO12 and UART4 RX on GPIO13.

## Python dependency

The client is standalone except for `pyserial`, which is already used by the
working STM32 client. Install it from Raspberry Pi OS if needed:

```sh
sudo apt update
sudo apt install python3-serial
```

In Thonny, use the same regular Python 3 interpreter as the validated STM32
motor client. Paste or open and run [`bno080_uart.py`](bno080_uart.py). No
virtual environment, Adafruit package, `board`, or `busio` module is required.

## Connection result

Before initializing sensor reports, the program performs an SHTP Buffer Status
Query/Notification round trip. A correct connection prints:

```text
BNO080 UART opened: /dev/ttyAMA4 at 3000000 baud (Pi TX GPIO12, RX GPIO13)
BNO080 UART-SHTP ROUND TRIP VERIFIED: buffer=... bytes
BNO080 SHTP reset complete
BNO080 initialized; fused navigation reports received
BNO080 dynamic accel/gyro/magnetometer calibration started
```

The executable-channel reset makes repeated Thonny runs reliable while the
BNO080 remains powered. Without it, the host's channel sequence counters restart
at zero but the sensor can retain its previous SHTP state and ignore new Set
Feature commands even though Buffer Status Query still responds.

If this round trip fails, check crossed TX/RX, common ground, UART-SHTP mode,
sensor power, and `/dev/ttyAMA4`. Do not randomly change the baud rate:
UART-SHTP is fixed at 3,000,000 baud, 8N1.

## Thonny shell

Print one fused sample:

```python
print_sample()
```

Read only the calibrated magnetometer vector `(x, y, z)` in microtesla:

```python
magnetometer()
print_magnetometer()
stream_magnetometer(5, 10)
```

The last command prints magnetometer, fused magnetic heading, and accuracy at
5 Hz for ten seconds. Use `stream_magnetometer(5)` to continue until Ctrl+C.

Print fused heading and vectors at 5 Hz for ten seconds:

```python
stream(5, 10)
```

Stream until Ctrl+C:

```python
stream(5)
```

Enable and display raw sensor ADC values for calibration logging:

```python
enable_raw_reports()
print_raw()
stream_raw(5, 10)
```

Check the accuracy field from the calibrated magnetometer report:

```python
calibration_status()
```

The client starts dynamic calibration automatically. With the sensor mounted in
its intended orientation and the drive motors stopped, slowly rotate the robot
or sensor through multiple headings and tilt it through different orientations.
Repeat until `calibration_status()` reaches `2` or preferably `3`. Keep the
sensor away from steel, magnets, motor power cables, speakers, and high-current
wiring while calibrating. Call `begin_calibration()` to restart the process.

An accuracy value of `0` is the sensor's own `unreliable` status, not a Python
placeholder. It is normal before calibration, but persistent zero can indicate
strong magnetic interference or insufficient movement through different axes.

This standalone subset does not write calibration records. That is intentional
for the first hardware test; it avoids changing persistent sensor state.

## Data provided

- Magnetic-north heading derived from the 9-axis rotation-vector quaternion.
- Rotation-vector quaternion `(i, j, k, real)`.
- Calibrated magnetic field in microtesla.
- Calibrated gyro in radians per second.
- Acceleration and gravity-removed linear acceleration in m/s^2.
- Optional raw accelerometer, gyroscope, and magnetometer ADC values.

The embedded driver implements the framing, buffer handshake, Set Feature
commands, and selected report formats from CEVA's binary
[UART-SHTP protocol](https://www.ceva-ip.com/wp-content/uploads/Sensor-Hub-Transport-Protocol.pdf)
and [SH-2 reference](https://www.ceva-ip.com/wp-content/uploads/SH-2-Reference-Manual.pdf).
