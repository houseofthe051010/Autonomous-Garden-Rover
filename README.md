# Autonomous Garden Rover

An autonomous garden-management rover intended to water plants and cut grass.

## System architecture

The rover uses an ESP32 as its high-level controller and a repurposed Ender-3
v4.2.2 mainboard as its four-channel stepper motor controller.

```text
ESP32 control code
       |
       | 115200 baud, 3.3 V UART
       v
Ender-3 v4.2.2 board (GD32F303RET6)
       |
       +-- X stepper driver
       +-- Y stepper driver
       +-- Z stepper driver
       +-- E stepper driver
       |
       +-- X/Y/Z switch inputs -> status returned to ESP32
```

The GD32 board generates accurately timed STEP and DIR signals. The ESP32 sends
motion commands and receives heartbeat, completion, and switch-status messages.
UART sends commands, not individual step pulses, so normal motor operation does
not depend on continuously streaming every pulse from the ESP32.

## Repository layout

- [`motor-controller/`](motor-controller/README.md): hardware wiring, limitations,
  resources, and system-level operating notes.
- [`motor-controller/UART_PROTOCOL.md`](motor-controller/UART_PROTOCOL.md): wire
  protocol, MicroPython API, motion units, and examples.
- [`motor-controller/esp32-micropython/`](motor-controller/esp32-micropython/README.md):
  ESP32 MicroPython controller source and deployment notes.
- [`motor-controller/ender3-firmware/`](motor-controller/ender3-firmware/README.md):
  flashable Ender firmware, checksum, source patch, and reproducible build steps.
- [`bts7960-controller/`](bts7960-controller/raspberry-pi/README.md): Raspberry
  Pi 5/Thonny UART client and protocol for the separate STM32 dual-BTS7960
  drive controller, including current-sense telemetry.
- [`Raspberry Pi 5 pinout`](bts7960-controller/raspberry-pi/PINOUT.md): current
  XL4005/STM32 wiring plus planned BNO080, GD32, and expansion UART allocation.

## Current motor-controller status

- Controls X, Y, Z, and E driver STEP/DIR behavior over UART.
- Supports signed finite moves in driver microsteps, RPM, or steps per second.
- Reports debounced X/Y/Z switch state changes to the ESP32.
- Provides heartbeat and tagged move-completion responses.
- Provides an emergency `M410` stop command.
- Supports at least 200 motor revolutions in either direction per command.
- Does not provide true indefinite, independently timed four-motor velocity mode.
  See [Motion limits](motor-controller/UART_PROTOCOL.md#motion-limits).

## Safety scope

This is dedicated stepper-tester/rover firmware. Heater temperature sensing and
thermal protection are intentionally disabled because the rover does not use the
printer heaters. Do not connect or operate a hotend or heated bed with this build.

The X/Y/Z switch messages are telemetry. They do not automatically stop ordinary
motion. Rover control code must decide how switch events affect movement.
