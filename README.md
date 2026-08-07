# Autonomous Garden Rover

An autonomous garden-management rover intended to water plants and cut grass.

## System architecture

The current stepper-control host is an ESP32-P4. It controls a repurposed
Ender-3 v4.2.2 mainboard used as a four-channel stepper motor controller.

```text
ESP32-P4 robot control firmware
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
       +-- X/Y/Z switch inputs -> status returned to ESP32-P4
```

The GD32 board generates accurately timed STEP and DIR signals. The ESP32-P4 sends
motion commands and receives heartbeat, completion, and switch-status messages.
UART sends commands, not individual step pulses, so normal motor operation does
not depend on streaming individual pulses from the ESP32-P4. Earlier Raspberry
Pi and ESP32 clients remain in the repository as validated reference models.

## Repository layout

- [`motor-controller/`](motor-controller/README.md): hardware wiring, limitations,
  resources, and system-level operating notes.
- [`motor-controller/UART_PROTOCOL.md`](motor-controller/UART_PROTOCOL.md): wire
  protocol, host API, motion units, and examples.
- [`motor-controller/raspberry-pi/`](motor-controller/raspberry-pi/README.md):
  Raspberry Pi 5 `/dev/ttyAMA3` Thonny client, wiring, and setup.
- [`motor-controller/esp32-p4-gd32-direct-motion/`](motor-controller/esp32-p4-gd32-direct-motion/README.md):
  ESP32-P4 handoff for simultaneous joystick velocity control, independent
  counted moves, UART timing, firmware, and reproducible GD32 source patch.
- [`Validated GD32 UART model`](motor-controller/raspberry-pi/VALIDATED_WORKING_MODEL.md):
  dated working Pi 5 UART, GD32 heartbeat, and Z-stepper configuration for later
  integration into the multi-UART rover controller.
- [`motor-controller/esp32-micropython/`](motor-controller/esp32-micropython/README.md):
  alternative ESP32 MicroPython controller source and deployment notes.
- [`motor-controller/ender3-firmware/`](motor-controller/ender3-firmware/README.md):
  flashable Ender firmware, checksum, source patch, and reproducible build steps.
- [`bts7960-controller/`](bts7960-controller/raspberry-pi/README.md): Raspberry
  Pi 5/Thonny UART client and protocol for the separate STM32 dual-BTS7960
  drive controller, including current-sense telemetry.
- [`Raspberry Pi 5 pinout`](bts7960-controller/raspberry-pi/PINOUT.md): current
  XL4005/STM32/BNO080 wiring plus planned GD32 and expansion UART allocation.
- [`Validated STM32 UART model`](bts7960-controller/raspberry-pi/VALIDATED_WORKING_MODEL.md):
  dated working Pi 5 UART and BTS7960 motor-control configuration, verification
  evidence, test commands, and remaining calibration work.
- [`STM32 controller firmware`](bts7960-controller/stm32-firmware/README.md):
  buildable PlatformIO source, binary artifact, analog encoder telemetry, and
  future Raspberry Pi UART flashing notes.
- [`ESP32-P4 STM32 UART test`](bts7960-controller/esp32-p4-micropython/README.md):
  standalone MicroPython/Thonny client for GPIO21/GPIO22 with heartbeat-first
  TX/RX orientation detection and verified PING/PONG.
- [`ESP32-P4 rover web and handheld control`](bts7960-controller/esp32-p4-stm32-ap/README.md):
  deployed AP/router web interface, mobile tank joystick, ramped dual-track
  control, authenticated handheld-controller Wi-Fi protocol, OTA, onboard
  speaker sound management, STM32, and GD32 integration.
- [`ESP32 hose valve controller`](hose-controller/esp32-hose-espnow/README.md):
  GPIO14 continuous-servo control, GPIO35 multi-turn potentiometer tracking,
  ESP-NOW protocol, watchdog behavior, wiring, and measured bench validation.
- [`BNO080 UART-SHTP client`](bno080/raspberry-pi/README.md): Pi 5 UART4 setup,
  Thonny client, fused heading/vector output, raw logging, and calibration API.
- [`Validated BNO080 UART model`](bno080/raspberry-pi/VALIDATED_WORKING_MODEL.md):
  dated working Pi 5 UART-SHTP round trip, fused sensor-report evidence, wiring,
  and remaining magnetometer calibration work.
- [`ODESC UART client`](odesc/raspberry-pi/README.md): validated Pi 5 UART2
  wiring, ODrive ASCII protocol test client, and integration notes.
- [`Validated ODESC UART model`](odesc/raspberry-pi/VALIDATED_WORKING_MODEL.md):
  dated working GPIO4/GPIO5 round trip, saved UART configuration, and observed
  ODESC status values.
- [`Combined four-UART shell`](robot/raspberry-pi/README.md): one standalone,
  directly pasteable Pi 5/Thonny controller for STM32, ODESC, GD32, and BNO08X
  commissioning, plus the deployed mobile tank-drive LAN dashboard and boot
  service.
- [`Recommended robot architecture`](robot/ARCHITECTURE.md): ROS 2, sensor
  fusion, vision, navigation, future encoder/GPS integration, and safety stages.

## Current motor-controller status

- Hardware-validated Pi 5/GD32 UART round trip and 4,000-step Z movement.
- Hardware-validated Pi 5/BNO080 UART-SHTP round trip and fused sensor reports.
- Hardware-validated Pi 5/ODESC UART2 ASCII property-read round trip.
- Controls X, Y, Z, and E driver STEP/DIR behavior over UART.
- Supports signed finite moves in driver microsteps, RPM, or steps per second.
- Reports debounced X/Y/Z switch state changes to the host.
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
