# Hardware Overview

## Current host links

| ESP32-P4 link | Host pins | Device pins | Settings | Purpose |
| --- | --- | --- | --- | --- |
| STM32 drive UART | GPIO21 TX, GPIO22 RX | PA10 RX, PA9 TX | 115200 8N1 | Dual BTS7960 control, current and encoder telemetry |
| GD32 stepper UART | GPIO27 TX, GPIO47 RX | PA10 RX, PA9 TX | 115200 8N1 | Simultaneous X/Y/Z stepper control |
| ODESC test UART | GPIO46/GPIO33 | Device TX/RX | Experimental | ODrive ASCII diagnostics; hardware status unresolved |

TX connects to the other device's RX. Use 3.3 V logic and a shared signal
ground. The firmware README is authoritative if an installed wiring revision
differs from this summary.

## Subsystems

- [`gd32-stepper-controller.md`](gd32-stepper-controller.md) documents the
  repurposed Creality Ender-3 v4.2.2/GD32 controller and its electrical limits.
- [`../../firmware/stm32-drive/README.md`](../../firmware/stm32-drive/README.md)
  documents the Blue Pill drive controller and diagnostic outputs.
- [`../../firmware/esp32-hose/README.md`](../../firmware/esp32-hose/README.md)
  documents the remote valve actuator, servo supply, and potentiometer input.

Never power motors or an MG996 servo from a microcontroller logic rail. Keep
motor power return paths sized correctly and connect grounds intentionally.
