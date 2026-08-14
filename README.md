# Autonomous Garden Rover

An open-source outdoor robot that can **mow grass and water a garden**. I built the rover around an aluminum-channel chassis, 10-inch recycled Power Wheels tires, custom 3D-printed transmissions, a height-adjustable string mower, and a two-axis hose turret. An ESP32-P4 coordinates the drivetrain, mower, steppers, IMU, audio, web controls, and the separate wireless hose-valve controller.

This repository contains the firmware, wiring documentation, full Fusion 360 assemblies, neutral STEP exports, individual STEP parts, and 212 printable STL files used to build the prototype.

[**Watch the finished demo**](https://youtu.be/P-olpegfmmU) · [**Read the complete 128-hour Macondo build journal**](https://macondo.hackclub.com/projects/9276) · [**Browse the CAD**](CAD/)

[![Autonomous Garden Rover mowing and watering demo](https://i.ytimg.com/vi/P-olpegfmmU/maxresdefault.jpg)](https://youtu.be/P-olpegfmmU)

## What it does

- Drives over grass using four independently powered 10-inch wheels and 2.5:1 printed gear reductions.
- Cuts grass with a C6374 sensored BLDC motor and replaceable string-line head.
- Raises and lowers the mower using a NEMA 17, planetary reduction, dual rope spools, and four-point suspension.
- Aims a hose using a two-axis turret with a self-locking worm-drive yaw stage and a geared pitch stage.
- Opens and closes the garden hose with a weather-resistant ESP32/MG996 actuator mounted at the faucet.
- Runs from a phone or custom handheld controller, with OTA firmware updates, microSD logging/audio, and voice input/output.
- Demonstrates scripted autonomous mowing and watering routines using drivetrain encoder feedback and BNO080 magnetometer heading correction. General garden navigation and obstacle avoidance are future work.

## System architecture

```text
Phone / handheld controller
          |
          | Wi-Fi / HTTP
          v
Waveshare ESP32-P4 + ESP32-C6 ---- microSD / microphone / speaker
          |
          +-- UART --> STM32F103 --> 2 x BTS7960 --> four drive motors
          |
          +-- UART --> GD32F303 Ender-3 board --> mower lift + turret X/Y
          |
          +-- UART --> ODESC --> C6374 mower motor
          |
          +-- UART --> BNO080 IMU
          |
          +-- local wireless command --> ESP32 hose controller --> MG996 valve actuator
```

| Subsystem | Hardware | Job |
| --- | --- | --- |
| Primary host | Waveshare ESP32-P4-WIFI6 | Web UI, command arbitration, autonomous sequences, OTA, audio, storage, and telemetry |
| Drivetrain | STM32F103 + 2 × BTS7960 | Closed-loop left/right drive control using the motors' repurposed potentiometers as encoders |
| Tool motion | Salvaged GD32F303 Ender-3 board | Generates step/direction motion for mower height, turret yaw, and turret pitch |
| Mower | ODESC V4.2 + C6374 BLDC | Motor initialization, speed control, and telemetry over the ODrive ASCII protocol |
| Heading | BNO080 | Magnetometer heading used by the scripted heading-hold loop |
| Hose valve | ESP32-WROOM-32 + modified MG996 | Wireless valve actuation with potentiometer position feedback |

The current ESP32-P4 firmware lives in [`firmware/esp32-p4/`](firmware/esp32-p4/README.md). The earlier Raspberry Pi 5 controller and commissioning tools remain in [`legacy/`](legacy/README.md) so the build history is reproducible.

## Mechanical design

### Drivetrain

The chassis is cut from 3/4-inch aluminum U-channel so it is light, repairable, and able to flex slightly over uneven ground. Each DS3230 drivetrain motor uses its original potentiometer as an absolute encoder. The drive reduction began at 1.5:1, but outdoor tests showed that the rover needed more torque to turn in grass, so I redesigned it as a 16:40 tooth (2.5:1) PETG transmission with bearing-supported outputs.

### Mower and height control

The first cutting head used sacrificial razor-blade arms. Outdoor testing proved that the arms broke as intended, but too frequently, so the final head uses replaceable string line and a cover to keep grass out of the motor. A separate NEMA 17 gearbox winds two rope spools to lift the deck evenly from four points.

### Watering turret

The yaw axis is a large module-4 worm wheel riding on eight 608 bearings. The worm drive resists back-driving, and a later 4.36:1 planetary stage fixed slipping seen during outdoor tests. The pitch axis uses an 18:1 planetary transmission. At the faucet, a separate sealed ESP32 controller drives a continuous-rotation MG996 through 720 degrees and reports its encoder position.

## Build journey

The final rover came from repeated outdoor testing rather than one finished CAD pass. These are selected entries from the [full Macondo journal](https://macondo.hackclub.com/projects/9276).

| Early rolling chassis | Mower CAD |
| --- | --- |
| ![First aluminum-channel rolling chassis with Power Wheels tires](https://cdn.hackclub.com/019ed8c7-f932-7aa7-a0f4-63fcd1a28b7b/image.png) | ![CAD of the original mower motor and blade assembly](https://cdn.hackclub.com/019f2ad2-f248-7ab3-a5b5-6bd28f143fd7/image.png) |
| **Two-axis turret design** | **Weather-resistant hose valve controller** |
| ![CAD of the turret worm wheel, bearings, and NEMA 17 drive](https://cdn.hackclub.com/019f7615-8497-7da8-bab4-61416e4c4f07/image.png) | ![Finished ESP32-controlled hose actuator installed at the faucet](https://cdn.hackclub.com/019fc962-4971-7d67-b8b5-c9f91bc0d1e1/image.png) |
| **Electronics rebuild** | **Grass-covered rover after a mowing test** |
| ![Rover power electronics and motor drivers during integration](https://cdn.hackclub.com/019f769f-fdfa-77c3-b291-9a491c0dfe77/image.png) | ![Inside of the rover after a successful outdoor mowing test](https://cdn.hackclub.com/019fe9d0-c1e4-7c82-b45c-76ef7a3f0b54/IMG_8295.jpeg) |

Major iterations included:

1. Building a wood proof of concept, then replacing it with a hand-cut aluminum chassis.
2. Converting four DS3230 servos into geared drive motors with encoder feedback.
3. Replacing the original razor blades with string line after destructive lawn tests.
4. Reprinting the turret worm drive, strengthening its pitch shaft, and adding a modular yaw gearbox.
5. Replacing unreliable motor drivers, rebuilding after a vibration-induced short circuit, and migrating the host from Raspberry Pi 5 to ESP32-P4.
6. Designing a motorized actuator around the existing outdoor faucet instead of adding an unreliable inline valve.
7. Adding UART subsystem links, OTA updates, voice recognition/feedback, and scripted heading-controlled demonstrations.

### CAD timeline proof

The repository includes the editable Fusion designs and exports; the Macondo journal also contains recordings of each Fusion 360 feature timeline. One example is shown below.

![Fusion 360 design timeline playback](https://cdn.hackclub.com/019ffe4b-6828-72b7-a0dd-16e0c0f28e6b/Screen%20Recording%202026-08-13%20224616.gif)

## CAD and fabrication files

| Location | Contents |
| --- | --- |
| [`CAD/Assembly/`](CAD/Assembly/) | Full mower-body Fusion archive, complete STEP assembly, and individual STEP bodies |
| [`CAD/Sub Assembly/`](CAD/Sub%20Assembly/) | 11 editable subsystem designs with matching STEP exports and individual parts |
| [`CAD/Individual Printable STLs/`](CAD/Individual%20Printable%20STLs/) | 212 named STL exports, grouped into one flat folder per assembly or subassembly |

The CAD tree currently contains 12 Fusion archives/designs, 650 STEP files, and 212 STL files. Generic `Body##` parts are retained in the individual STEP exports so the complete design hierarchy is preserved; printable STLs favor human-named parts and are grouped by their source assembly without deeper nesting.

## Firmware and communication

The P4 is the only high-level command source. Each real-time subsystem owns its low-level timing and stops locally if commands expire.

- **STM32 drivetrain UART:** receives left/right drive commands and returns the four potentiometer-encoder readings. P0/P1 are the right wheels and P2/P3 are the left wheels.
- **GD32 stepper UART:** accepts compact commands such as `MOTOR 1 B 4095` and `MSTOP 1` for the three NEMA 17 axes.
- **ODESC UART:** uses the ODrive ASCII protocol for startup, mower velocity, current, voltage, and fault telemetry.
- **BNO080 UART:** supplies the magnetometer heading used for heading-hold correction.
- **Hose controller:** receives a local-network actuation command and closes the loop around the modified servo's potentiometer.

The autonomous demonstration code combines these links into two routines: a ten-foot mowing pass held near 188°, and a watering sweep held near 22°. The watering routine opens the faucet, raises the turret pitch axis, sweeps the yaw axis, and advances the chassis in timed half-foot increments. The module is compiled into the P4 firmware but remains dormant until a future GUI action calls it.

Protocol details and wiring notes are indexed in [`docs/`](docs/README.md).

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
## Build it yourself

This guide gets a fresh checkout to buildable firmware. It does **not** make a complete rover: assembling and validating the electrical system is a separate, hardware-specific task. Read the target README and the relevant documents under [`docs/`](docs/README.md) before connecting power or flashing a board.

### Safety first

This project controls motors and a hose-valve actuator. Build and test one controller at a time with drive wheels raised or mechanisms disconnected, use low command values, and keep an independent power disconnect within reach. Never power the MG996 hose servo from an ESP32 board. Do not flash the experimental ODESC images on a live rover without following its supervised bench-validation procedure.

### 1. Get the source

```sh
git clone https://github.com/houseofthe051010/Autonomous-Garden-Rover.git
cd Autonomous-Garden-Rover
```

### 2. Install the toolchain for the target you are building

| Target | Required toolchain | Build guide |
| --- | --- | --- |
| ESP32-P4 primary host | ESP-IDF 6.0 or newer | [`firmware/esp32-p4/README.md`](firmware/esp32-p4/README.md) |
| STM32F103 drivetrain | PlatformIO with ST-Link support | [`firmware/stm32-drive/README.md`](firmware/stm32-drive/README.md) |
| GD32 Ender-3 stepper board | Marlin 2.1.2.5 and PlatformIO | [`firmware/gd32-stepper/README.md`](firmware/gd32-stepper/README.md) |
| ESP32 hose controller | PlatformIO | [`firmware/esp32-hose/README.md`](firmware/esp32-hose/README.md) |
| ODESC V4.2 (experimental) | Ubuntu, Tup, and `gcc-arm-none-eabi` | [`firmware/odesc-v42/README.md`](firmware/odesc-v42/README.md) |

Install ESP-IDF using Espressif's official setup instructions, then open an ESP-IDF-enabled terminal before running `idf.py`. Install PlatformIO through its CLI or IDE integration and ensure the `pio` command is available in your shell.

### 3. Configure secrets locally

The ESP32-P4 needs a controller key that is unique to your own rover and handheld controller. Create the ignored local file from the example:

```sh
cd firmware/esp32-p4/main
cp rover_control_key.example.h rover_control_key.h
```

Replace all 32 `0x00` values with 32 cryptographically random bytes and place the identical key in the matching handheld-controller project. Do not commit this file, Wi-Fi credentials, device backups, or serial captures containing credentials. The repository ignores `rover_control_key.h` by design.

### 4. Build each target

#### ESP32-P4 primary host

```sh
cd firmware/esp32-p4
idf.py set-target esp32p4
idf.py build
```

To flash a connected board, replace the serial port with the one assigned by your operating system:

```sh
idf.py -p /dev/ttyACM0 flash monitor
```

The P4 uses Component Manager to fetch its declared dependencies during the build. See its README for required wiring, partition layout, OTA behavior, and network-security notes.

#### STM32F103 drivetrain controller

```sh
cd firmware/stm32-drive
pio run
```

With an ST-Link connected and the board safely isolated from motor power:

```sh
pio run --target upload
```

Check the [STM32 UART protocol](docs/protocols/stm32-drive-uart.md) before connecting it to the host controller.

#### GD32 Ender-3 stepper controller

The checked-in source is a patch against Marlin `2.1.2.5`, not a complete Marlin checkout. Rebuild it as follows:

```sh
git clone --branch 2.1.2.5 https://github.com/MarlinFirmware/Marlin.git
cd Marlin
git apply /path/to/Autonomous-Garden-Rover/firmware/gd32-stepper/source/marlin-2.1.2.5-gd32-direct-motion.patch
cp /path/to/Autonomous-Garden-Rover/firmware/gd32-stepper/source/_Bootscreen.h .
cp /path/to/Autonomous-Garden-Rover/firmware/gd32-stepper/source/_Statusscreen.h .
platformio run -e STM32F103RE_creality
```

Flash the resulting image from a FAT32 microSD card only after reviewing the [GD32 guide](firmware/gd32-stepper/README.md). This derived target is licensed under GPL-3.0-only, unlike the repository's original MIT-licensed material.

#### ESP32 hose controller

```sh
cd firmware/esp32-hose
pio run
```

To upload, use your board's port:

```sh
pio run -t upload --upload-port /dev/ttyUSB0
```

Read the hose-controller README before wiring the actuator; it requires a separate 5-6 V supply sized for servo stall current and a common ground.

#### ODESC V4.2 (experimental)

Build this target only on a bench after reading the full target README. Its current source and board-specific safety limitations are documented there.

```sh
cd firmware/odesc-v42/source/Firmware
tup init
tup
```

The output is created under `build/`. A successful build is not proof that the firmware is safe to flash or operate at pack voltage.

### 5. Verify before motion

1. Confirm the board revision, target, pinout, and firmware revision match.
2. Verify common ground and voltage levels before joining UART connections.
3. Start with mechanisms unloaded or wheels raised.
4. Confirm watchdog and stop behavior before nonzero motor commands.
5. Record a sanitized test result in the target documentation or local diagnostics archive.

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

The physical rover has completed outdoor drive, mowing, watering, turret, and scripted autonomous-sequence demonstrations. The ESP32-P4 host, STM32 drive link, GD32 stepper link, ODESC mower link, BNO080 link, web controls, microSD audio, and hose-control path have all been exercised on the intended hardware. The current autonomous code demonstrates fixed routines; general navigation, obstacle avoidance, and sensor-fused localization are still experimental.

This is prototype robotics hardware. Test it with the wheels raised, keep clear of moving mechanisms, verify every pin before applying power, and keep an independent power disconnect within reach.

## Third-party notices

The repository-level [MIT License](LICENSES/LICENSE) applies to original project code
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

Original project code and documentation are released under the [MIT License](LICENSES/LICENSE). Some bundled firmware is derived from third-party projects and retains its own license; review the third-party notices above before redistributing or modifying it. The Marlin-derived stepper patches and binaries are GPL-3.0-only.
