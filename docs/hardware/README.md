# Hardware Overview

## Current host links

| ESP32-P4 link | Host pins | Device pins | Settings | Purpose |
| --- | --- | --- | --- | --- |
| STM32 drive UART | GPIO21 TX, GPIO22 RX | PA10 RX, PA9 TX | 115200 8N1 | Dual BTS7960 control, current and encoder telemetry |
| GD32 stepper UART | GPIO2 TX, GPIO1 RX (passively detected) | PA10 RX, PA9 TX | 115200 8N1 | Both P4 pins remain inputs until a valid GD32 heartbeat identifies RX |
| ODESC UART | GPIO27 TX, GPIO47 RX | GPIO2 RX, GPIO1 TX | 115200 8N1 | Verified on 2026-08-08 using live VBUS queries |
| BNO080 UART-SHTP | GPIO5 TX, GPIO6 RX (passively detected) | RX, TX | 3000000 8N1 | Fused heading, quaternion, magnetometer, gyro, acceleration, and linear acceleration |

TX connects to the other device's RX. Use 3.3 V logic and a shared signal
ground. The firmware README is authoritative if an installed wiring revision
differs from this summary.

The ODESC link was tested continuously after OTA deployment and returned about
36.13-36.21 V with zero UART query failures. GPIO27/GPIO47 are therefore fixed
in this orientation; UART swapping is no longer required. The ODESC and GD32
now use separate UART pairs and can operate concurrently.

The prepared external-VBUS ODESC release uses GPIO3 for a 100 kOhm / 5.6 kOhm
battery divider with a 10 nF (`103`) capacitor from GPIO3 to ground. Battery
positive connects through 100 kOhm to GPIO3; 5.6 kOhm and the capacitor connect
from GPIO3 to battery ground. The P4 accepts readings above the onboard 36.3 V
ADC clipping point only when the ODESC reports the external source valid,
fault-free, and status zero. Full wiring, calibration, rollback, and supervised
validation are in
[`../../firmware/odesc-v42/releases/external-vbus-gpio3/README.md`](../../firmware/odesc-v42/releases/external-vbus-gpio3/README.md).

The GD32 link was verified after OTA deployment on 2026-08-08. GPIO1 received
the heartbeat, so the P4 selected GPIO1 as RX and GPIO2 as TX. Passive detection
still runs after every boot and does not enable either TX output until a valid
`HB` frame establishes the orientation.

The BNO080 link was verified on 2026-08-08. A receive-only GPIO5/GPIO6 probe
identified the installed orientation as GPIO5 TX and GPIO6 RX before the P4
enabled its transmitter. The sensor then returned the expected 256-byte
UART-SHTP host buffer and continuously delivered all five configured fused
reports. UART-SHTP mode must be selected before BNO080 power-up, and the BNO080
and P4 must share a 3.3 V logic ground.

The P4 stores IMU mounting calibration in NVS. Level calibration zeros the
displayed roll/pitch at the installed rover attitude. Forward calibration uses
only the controlled acceleration portion of a straight drive to estimate the
sensor-to-rover yaw offset; it is not a replacement for BNO080 magnetometer
calibration and cannot infer direction during constant-speed motion.

## Subsystems

- [`gd32-stepper-controller.md`](gd32-stepper-controller.md) documents the
  repurposed Creality Ender-3 v4.2.2/GD32 controller and its electrical limits.
- [`../../firmware/stm32-drive/README.md`](../../firmware/stm32-drive/README.md)
  documents the Blue Pill drive controller and diagnostic outputs.
- [`../../firmware/esp32-hose/README.md`](../../firmware/esp32-hose/README.md)
  documents the remote valve actuator, servo supply, and potentiometer input.

Never power motors or an MG996 servo from a microcontroller logic rail. Keep
motor power return paths sized correctly and connect grounds intentionally.
