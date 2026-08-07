# Autonomous Garden Rover

Open-source firmware and hardware documentation for an experimental autonomous
garden rover. The current controller is a Waveshare ESP32-P4 board with its
onboard ESP32-C6 providing Wi-Fi. It coordinates the drive motors, stepper
axes, audio, storage, and handheld/web controls.

> [!WARNING]
> This is prototype robotics software, not a certified safety system. Test with
> wheels raised, mechanisms clear, and an independent power disconnect within
> reach. Review every pin assignment before powering motor hardware.

## System overview

```text
Phone or handheld controller
        |
        | Wi-Fi / HTTP
        v
Waveshare ESP32-P4 + ESP32-C6       microSD / speaker / microphone
        |
        +-- UART --> STM32F103 --> two BTS7960 drive channels
        |
        +-- UART --> GD32F303 Ender-3 board --> X/Y/Z stepper axes

Handheld controller -- ESP-NOW --> ESP32 hose-valve controller
```

The ESP32-P4 firmware is the supported host implementation. The earlier
Raspberry Pi 5 host and commissioning programs are retained under
[`legacy/`](legacy/README.md) as reference implementations.

## Repository layout

| Path | Status | Purpose |
| --- | --- | --- |
| [`firmware/esp32-p4/`](firmware/esp32-p4/README.md) | Primary | ESP-IDF rover host, web UI, Wi-Fi/OTA, UART links, audio, and storage |
| [`firmware/stm32-drive/`](firmware/stm32-drive/README.md) | Active | STM32F103 dual-BTS7960 drive controller and telemetry |
| [`firmware/gd32-stepper/`](firmware/gd32-stepper/README.md) | Active | GD32F303/Ender-3 simultaneous X/Y/Z stepper firmware |
| [`firmware/esp32-hose/`](firmware/esp32-hose/README.md) | Active | ESP32 hose-valve actuator and position tracking |
| [`docs/`](docs/README.md) | Reference | Architecture, wiring, protocols, and safety notes |
| [`legacy/raspberry-pi/`](legacy/raspberry-pi/README.md) | Legacy | Raspberry Pi 5 multi-UART controller and device clients |
| [`legacy/prototypes/`](legacy/prototypes/README.md) | Legacy | Commissioning tools and superseded firmware variants |

## Primary firmware quick start

The ESP32-P4 project requires ESP-IDF 6.0 or newer. Component dependencies are
declared in `main/idf_component.yml` and resolved by the IDF component manager.

```sh
cd firmware/esp32-p4
cp main/rover_control_key.example.h main/rover_control_key.h
# Edit rover_control_key.h with a private 32-byte controller key.
idf.py set-target esp32p4
idf.py build
idf.py flash monitor
```

Do not commit `main/rover_control_key.h`, Wi-Fi credentials, device backups, or
build output. The project-level and repository-level ignore rules exclude them.
See the [ESP32-P4 firmware guide](firmware/esp32-p4/README.md) for board-specific
flashing, wiring, web routes, OTA, and recovery instructions.

The STM32 controller uses PlatformIO:

```sh
cd firmware/stm32-drive
pio run
pio run --target upload
```

The GD32 firmware is distributed as a reproducible Marlin patch plus a checked
flash image because its Creality bootloader is updated by microSD. See
[`firmware/gd32-stepper/README.md`](firmware/gd32-stepper/README.md).

## Interfaces

The installed UART mapping and electrical constraints are documented in the
[hardware overview](docs/hardware/README.md). Protocol details live in
[`docs/protocols/`](docs/protocols/README.md) and beside firmware when the
protocol is tightly coupled to that target.

All MCU UART signals are 3.3 V logic. UART TX must connect to the receiving
device's RX, every device must share signal ground, and motor/servo loads need
appropriately sized external power supplies.

## Project status

This repository records a hardware-validated prototype, but not every subsystem
has the same maturity. The ESP32-P4, STM32 drive link, GD32 stepper link, web
control, microSD audio, and hose-controller paths have been exercised on the
installed hardware. Sensor fusion, autonomous navigation, and ODESC integration
remain experimental or future work. Individual READMEs state their validation
status and known limitations.

## Contributing

Read [`CONTRIBUTING.md`](CONTRIBUTING.md) before opening a change. Keep hardware
tests bounded, document the exact board and wiring used, and never publish
credentials or device-specific control keys. Security and safety reports are
covered by [`SECURITY.md`](SECURITY.md).

## License

Original code and documentation in this repository are licensed under the
[MIT License](LICENSE). Third-party components, patches, and derived firmware
remain subject to their upstream licenses; relevant notices are retained next
to those files and under [`LICENSES/`](LICENSES/).
