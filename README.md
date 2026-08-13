# Autonomous Garden Rover

An open hardware and firmware project for a prototype garden rover: a mobile platform for carrying tools, moving a hose, and eventually navigating between garden beds on its own. The current version can be driven from a phone or handheld controller and has separate controllers for its drivetrain, stepper mechanisms, and hose valve.

The main controller is a Waveshare ESP32-P4. Its onboard ESP32-C6 provides Wi-Fi, while UART links connect the drivetrain and stepper controllers.

## Demo video

[Watch the rover mow grass and water plants](https://youtu.be/P-olpegfmmU?si=Mag2Q_cvqoKTUKmW)

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

| Item | Qty. | Unit price | Shipping | Total | Link |
| --- | ---: | ---: | ---: | ---: | --- |
| Waveshare ESP32-P4-WIFI6 development board | 1 | $26.87 | $0.00 | $26.87 | [Amazon](https://www.amazon.com/dp/B0FM3SPXZG) |
| Steelworks 3/4 in × 8 ft aluminum channel | 2 | $19.98 | $0.00 | $39.96 | [Lowe's](https://www.lowes.com/pd/Steelworks-3-4-in-W-x-8-ft-L-Mill-Finished-Aluminum-Weldable-Trim-Channel/3058185) |
| LGXSHOP C6374 170KV sensored BLDC motor | 1 | $29.50 | $10.00 | $39.50 | [Amazon](https://www.amazon.com/dp/B0GR88K1XP) |
| STM32F103C6T6 Blue Pill development board | 1 | $1.75 | $0.00 | $1.75 | [AliExpress](https://www.aliexpress.us/item/3256809531654480.html) |
| BTS7960 high-current motor driver board | 2 | $5.56 | $0.00 | $11.12 | [AliExpress](https://www.aliexpress.us/item/3256812145540065.html) |
| DS3230 PRO drivetrain servo motor | 1 | $51.37 | $0.00 | $51.37 | [AliExpress](https://www.aliexpress.us/item/3256808314550897.html) |
| STEPPERONLINE NEMA 17 stepper motors (3-pack) | 1 | $25.99 | $0.00 | $25.99 | [Amazon](https://www.amazon.com/dp/B0B38GHRH8) |
| CNC controller/stepper driver board (12-24 VDC) | 1 | $22.99 | $0.00 | $22.99 | [Amazon](https://www.amazon.com/Cutter-Control-PCBLaser-Engraver-12%E2%80%9124VDC/dp/B0CCVSMGXR/ref=sr_1_8?crid=3AAATVZPSOA2F&dib=eyJ2IjoiMSJ9.aWzcw_WIA8asa20FQSchbt_OHEjmR2H9c02dW500VTK0Zh7viqR8UD-O2liKhXq518VuWrnsAnzvrFg0OpocwPySXmTESd6QuvwzjMqyQpFbEy9nW_tIZf2RHFWTH51NieJmsNjF5AOFKCcgoPFKMQ1kMiat9qz9cC0JR4CG_JgIMWaK4w8S-mu0iLdcvP7TOwW_SIvHBkkTu-KY2cPFMXc6mGlgN_qlOfMjWE7K1dtaOljvCJsbDiK9EfR1HFKnh1Ub_iFOt6-cyxlZCrM-DfQMENAL4w00ucOnpZhGvS4.qlJW6awkxNKtk6nr6GXA36CZ6CU4FdrNQX-PGFT0lCo&dib_tag=se&keywords=cnc+mainboard&qid=1786660258&sprefix=cnc+mainboar%2Caps%2C106&sr=8-8) |
| EONO PETG 3D printer filament 1 kg black | 2 | $9.99 | $0.00 | $19.98 | [Amazon](https://www.amazon.com/EONO3D-Printer-Filament-1-75mm-2-2lbs/dp/B0G2BQQ5RT/ref=sr_1_1_sspa?crid=1L2ZTSMFNQU33&dib=eyJ2IjoiMSJ9.SEBI8_4wrU3elsHKfGW2ERnsN8KTs5kD7BhyZ9y__Z3QMtZ2FN7r_UnCcOA2tX0kFXVnp_JkbhH5ToHCJwa6H5mphhIVeCnlmKcQWw7EuqJqzap2wdZuLar07Rl_8Vn17IGjlAW7Z-r7gR4lC7Dtfe0nnChFDHjhZpz0rPiA5ZrW-2Vcxv6pA8Yn9m5KewAWe7rTjt5I6TuVCNnVJ1Y59HKh0GpZ1HSDrctdb1M_Mb8.zM1CrTE7JZozW5RqAC32XgZVFvmBgX-H2YCaIrZXDgk&dib_tag=se&keywords=petg+1kg&qid=1786660120&sprefix=petg+1kg%2Caps%2C128&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1) |
| 608 sealed steel bearings (20-pack) | 1 | $4.99 | $0.00 | $4.99 | [Amazon](https://www.amazon.com/Bearings-Premium-Steel-Sealed-Groove/dp/B0GX14YCFF/ref=sr_1_4?crid=10SNBKND149OQ&dib=eyJ2IjoiMSJ9.qWnyKcrC2gR1Mn2qmoQKOLT5AfvFyiZh06vPG-1zIPZez2VPXJQstfndu2ylGIEzapGf_3S58rWXK_9iR2qQ6GoZ9p_QSe4PqkSosthnsG6tbFhDlB-f5Zw_XfHw5twSB0SPWddbqdhwubVlA6fgAF-BLxibDqNPZHplWfyIEbFXKYd1u3x2aa1H0P1P9q2HaSQwlbPOQNzS8LIG2GRRPwsEuo-iG6U_rxuGc01EDqQ.OQoMtqLRcUuYDn2x0hnL31AfVoy9-2m7k8DtXQWT360&dib_tag=se&keywords=608%2Bbearings&qid=1786660407&sprefix=608%2Bbearings%2Caps%2C116&sr=8-4&th=1) |
| M3 screw kit (420-piece) | 1 | $8.98 | $0.00 | $8.98 | [Amazon](https://www.amazon.com/mxuteuk-420PCS-Screws-Socket-Wrench/dp/B0CSWD34KJ/ref=sr_1_8?crid=2EJ1MY8ARTFXO&dib=eyJ2IjoiMSJ9.mYqaF0B5tSbDeR49-ETzhwFfe-oW7XxaeFRt9f46-e0V6_ZIBIcWASPbMRPBz4MyZrarI0rTHsknSqSSA2Mv1c6gnKyHECdQ_mmvmfMkOmNTksfWn0dZaOG6Fq7Ao5GJKvRxG87OZhPjxuORRALFMlQedBTmzZiuwIRT_DwQbBebX8KVN70JGWwkieWeQChzPVrv0dhuZbYq25uJzkez_hJws5quPnK4NVaVnPXWPZI.nNFLVrEusWX8W3KP-ABp5YueLbETjq74Xuxoufqo8Lk&dib_tag=se&keywords=m3%2Bscrew%2B50mm&qid=1786660454&sprefix=m3%2Bscrew%2B50mm%2Caps%2C134&sr=8-8&th=1) |
| Flipsky ODESC v4.2 24 V single-axis controller | 1 | $39.99 | $0.00 | $39.99 | [Amazon](https://www.amazon.com/dp/B0CB64MVHC) |
| 22 AWG wire (10 m) | 1 | $2.81 | $0.00 | $2.81 | [AliExpress](https://www.aliexpress.us/item/3256801511977665.html) |
| 36 V 8 Ah lithium battery | 1 | $80.00 | $0.00 | $80.00 | [Amazon](https://www.amazon.com/SHEWAIHE-Lithium-Battery-Charger-Providing/dp/B0GWMQY5NT) |
| 20 A buck converter | 2 | $3.58 | $0.00 | $7.16 | [AliExpress](https://www.aliexpress.us/item/3256808333733098.html) |
| XL4005 buck converter | 1 | $1.99 | $0.00 | $1.99 | [AliExpress](https://www.aliexpress.us/item/3256808679872256.html) |
| MG996 servo motor | 1 | $3.44 | $0.00 | $3.44 | [AliExpress](https://www.aliexpress.us/item/3256802804659030.html) |
| ESP32-WROOM-32 development board with U.FL | 1 | $7.32 | $0.00 | $7.32 | [AliExpress](https://www.aliexpress.us/item/3256807142919728.html) |
| **Estimated total** |  |  |  | **$396.21** | |
## Firmware

For a fresh checkout and build instructions for every controller, start with
the [build-it-yourself guide](BUILDING.md).

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

# Third-party notices

The repository-level [MIT License](LICENSE) applies to original project code
and documentation only. It does not replace licenses in bundled or derived
third-party material.

| Material | Location | License / notice |
| --- | --- | --- |
| ODrive-derived ODESC source | [`firmware/odesc-v42/source/`](firmware/odesc-v42/source/) | MIT; retain the included [ODrive notice](firmware/odesc-v42/source/LICENSE.md) |
| Marlin-derived GD32 stepper patch, display assets, and binary | [`firmware/gd32-stepper/`](firmware/gd32-stepper/) | GPL-3.0-only; full text in [`LICENSES/Marlin-GPL-3.0.txt`](LICENSES/Marlin-GPL-3.0.txt) |
| Historical Marlin prototype artifacts | [`legacy/prototypes/gd32-marlin-stepper/`](legacy/prototypes/gd32-marlin-stepper/) | GPL-3.0-only; see the included documentation and license text |

Before copying, modifying, or redistributing third-party material, review the
license located with that material. Add a notice here when introducing new
vendored or derived code.


## License

Original project code and documentation are released under the [MIT License](LICENSE). Some bundled firmware is derived from third-party projects and retains its own license; see [third-party notices](THIRD_PARTY_NOTICES.md) before redistributing or modifying it. The Marlin-derived stepper patches and binaries are GPL-3.0-only.
