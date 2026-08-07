# Validated BNO080 UART-SHTP Model

## Validation record

This hardware/software arrangement was confirmed working on 2026-07-22:

- Raspberry Pi 5 running Raspberry Pi OS and regular Python 3 in Thonny.
- BNO080 configured for UART-SHTP mode.
- Pi UART4 exposed as `/dev/ttyAMA4` at the protocol's fixed 3,000,000 baud.
- Pi GPIO12 transmitted to the BNO080 RX input.
- Pi GPIO13 received from the BNO080 TX output.
- The standalone Python client completed an SHTP Buffer Status round trip.
- The BNO080 reported a 256-byte host-write buffer.
- The client enabled and received fused navigation reports.
- Fused heading, quaternion, calibrated magnetometer, gyroscope,
  accelerometer, and linear-acceleration values were displayed in Thonny.

Observed successful startup:

```text
BNO080 UART opened: /dev/ttyAMA4 at 3000000 baud (Pi TX GPIO12, RX GPIO13)
BNO080 UART-SHTP ROUND TRIP VERIFIED: buffer=256 bytes
BNO080 SHTP reset complete
BNO080 initialized; fused navigation reports received
BNO080 dynamic accel/gyro/magnetometer calibration started
```

The buffer-status response verifies bidirectional communication. Receiving
changing fused reports also verifies SHTP packet framing, SH-2 feature setup,
and sensor-report decoding.

## Validated wiring

| Raspberry Pi 5 | Physical pin | BNO080 | Function |
| --- | ---: | --- | --- |
| GPIO12 / UART4 TX | 32 | RX / SCL-RX | Commands to BNO080 |
| GPIO13 / UART4 RX | 33 | TX / SDA-TX | Reports to Pi |
| GND | Any Pi GND | GND | Common signal reference |

The sensor was selected for UART-SHTP mode before startup with `PS1=HIGH` and
`PS0=LOW`. UART logic is 3.3 V. Breakout-board power requirements still depend
on the exact module design and must not be inferred from this logic wiring.

## Pi configuration

`/boot/firmware/config.txt` contains this entry under `[all]`:

```ini
dtoverlay=uart4-pi5
```

The working Linux serial device is `/dev/ttyAMA4`. The client uses 3,000,000
baud, 8 data bits, no parity, and one stop bit as required by UART-SHTP.

## Working software

The validated client is [`bno080_uart.py`](bno080_uart.py). It runs in Thonny's
regular Python 3 interpreter and only depends on Raspberry Pi OS `pyserial`; it
does not require a virtual environment or the Adafruit BNO08X package.

Validated shell output was produced by:

```python
print_sample()
```

Dedicated magnetometer commands are also available:

```python
magnetometer()
print_magnetometer()
stream_magnetometer(5, 10)
```

## Not yet validated or calibrated

- Magnetometer calibration: the observed report accuracy was `0` (unreliable).
- Heading accuracy after calibration in the robot's final mounting position.
- Magnetic interference from motors, steel, and high-current wiring.
- Persistent saving and restoration of BNO080 calibration records.
- Raw accel/gyro/magnetometer report streaming on physical hardware.
- Long-duration operation and recovery after sensor or UART interruption.

These items must remain separate from the validated UART and fused-report path.
