# Autonomous Garden Rover

This is my prototype garden rover: a mobile platform for carrying tools, moving a hose, and eventually navigating between garden beds on its own. The current version can be driven from a phone or handheld controller and has separate controllers for its drivetrain, stepper mechanisms, and hose valve.

The main controller is a Waveshare ESP32-P4. Its onboard ESP32-C6 provides Wi-Fi, while UART links connect the drivetrain and stepper controllers.

## How it works

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

The ESP32-P4 runs the web interface and coordinates the rover. The STM32 handles the two drive channels, the repurposed Ender-3 controller runs three NEMA 17 axes, and a separate ESP32 controls the hose valve. The older Raspberry Pi controller is kept in [`legacy/`](legacy/README.md) for reference.

## Bill of materials

| Item | Qty. | Unit price | Shipping | Line total | Link |
| --- | ---: | ---: | ---: | ---: | --- |
| Waveshare ESP32-P4-WIFI6 development board | 1 | $26.87 | $0.00 | $26.87 | [Amazon](https://www.amazon.com/dp/B0FM3SPXZG) |
| Steelworks 3/4 in × 8 ft aluminum channel | 2 | $19.98 | $0.00 | $39.96 | [Lowe's](https://www.lowes.com/pd/Steelworks-3-4-in-W-x-8-ft-L-Mill-Finished-Aluminum-Weldable-Trim-Channel/3058185) |
| LGXSHOP C6374 170KV sensored BLDC motor | 1 | $29.50 | $10.00 | $39.50 | [Amazon](https://www.amazon.com/dp/B0GR88K1XP) |
| STM32F103C6T6 Blue Pill development board | 1 | $1.75 | $0.00 | $1.75 | [AliExpress](https://www.aliexpress.us/item/3256809531654480.html) |
| BTS7960 high-current motor driver board | 2 | $5.56 | $0.00 | $11.12 | [AliExpress](https://www.aliexpress.us/item/3256812145540065.html) |
| DS3230 PRO drivetrain servo motor | 1 | $51.37 | $0.00 | $51.37 | [AliExpress](https://www.aliexpress.us/item/3256808314550897.html) |
| STEPPERONLINE NEMA 17 stepper motors (3-pack) | 1 | $25.99 | $0.00 | $25.99 | [Amazon](https://www.amazon.com/dp/B0B38GHRH8) |
| Arduino Nano + A4988 stepper-controller kit | 1 | $8.76 | $0.00 | $8.76 | [AliExpress](https://www.aliexpress.us/item/3256805832366199.html) |
| Flipsky ODESC v4.2 24 V single-axis controller | 1 | $39.99 | $0.00 | $39.99 | [Amazon](https://www.amazon.com/dp/B0CB64MVHC) |
| **Estimated total** |  |  |  | **$245.31** | |

The machine-readable version is in [`bom.csv`](bom.csv). Prices were checked on August 7, 2026 and are before tax. Shipping is only included where it was known. Amazon did not have an ODESC v3.6 under $40, so the BOM uses the cheapest in-stock ODESC listing I found: the $39.99 Flipsky v4.2 24 V single-axis board. It is intended for the experimental BLDC path; the current dual drive controller uses the STM32 and BTS7960 boards.

## Firmware

The main ESP32-P4 firmware uses ESP-IDF 6.0 or newer:

```sh
cd firmware/esp32-p4
cp main/rover_control_key.example.h main/rover_control_key.h
# Add a private 32-byte controller key to rover_control_key.h.
idf.py set-target esp32p4
idf.py build
idf.py flash monitor
```

The STM32 drive controller uses PlatformIO:

```sh
cd firmware/stm32-drive
pio run --target upload
```

The Ender-3/GD32 stepper firmware is flashed by microSD. Its build and flashing notes are in the [GD32 firmware guide](firmware/gd32-stepper/README.md). More detailed pinouts and protocol notes are under [`docs/`](docs/README.md).

## Repository map

| Folder | Contents |
| --- | --- |
| [`firmware/esp32-p4/`](firmware/esp32-p4/README.md) | Main rover controller, web UI, Wi-Fi, audio, storage, and UART links |
| [`firmware/stm32-drive/`](firmware/stm32-drive/README.md) | Dual-BTS7960 drivetrain controller |
| [`firmware/gd32-stepper/`](firmware/gd32-stepper/README.md) | Three-axis stepper firmware for the Ender-3 controller |
| [`firmware/esp32-hose/`](firmware/esp32-hose/README.md) | Hose-valve controller |
| [`docs/`](docs/README.md) | Wiring, architecture, protocols, and safety notes |
| [`legacy/`](legacy/README.md) | Earlier Raspberry Pi code and test programs |

## Current status

The ESP32-P4 host, STM32 drive link, GD32 stepper link, web controls, microSD audio, and hose-control path have been tested on their intended hardware. Autonomous navigation, sensor fusion, and ODESC integration are still experimental.

This is prototype robotics hardware. Test it with the wheels raised, keep clear of moving mechanisms, verify every pin before applying power, and keep an independent power disconnect within reach.

## License

My original code and documentation are released under the [MIT License](LICENSE). The Marlin-derived stepper patches and binaries keep their upstream GPL license under [`LICENSES/`](LICENSES/).
