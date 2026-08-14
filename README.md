# Autonomous Garden Rover

An open-source outdoor robot that can **mow grass and water a garden**. I built the rover around an aluminum-channel chassis, 10-inch recycled Power Wheels tires, custom 3D-printed transmissions, a height-adjustable string mower, and a two-axis hose turret. An ESP32-P4 coordinates the drivetrain, mower, steppers, IMU, audio, web controls, and the separate wireless hose-valve controller.

I normally spend about two hours mowing the grass every two weeks, plus several more hours watering the garden. I wanted to build something that could take those repetitive jobs off my schedule while giving me a reason to learn outdoor robotics, mechanical design, motor control, and distributed embedded systems. The rover combines both jobs so one machine can maintain the yard instead of requiring a separate project for each chore.

This repository contains the firmware, wiring documentation, full Fusion 360 assemblies, neutral STEP exports, individual STEP parts, and 72 deduplicated printable STL files used to build the prototype.

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

The current ESP32-P4 firmware lives in [`firmware/esp32-p4/`](firmware/esp32-p4/README.md).

### Block-level electrical schematic

The block-level schematic shows the power rails, host computer, motor-control boards, sensors, and UART links used by the modular rover electronics.

[![Autonomous Garden Rover block-level electrical schematic](Schematics/Block-level%20schematic/rover.svg)](Schematics/Block-level%20schematic/rover.svg)

The editable KiCad source is in [`Schematics/Block-level schematic/`](Schematics/Block-level%20schematic/).

### Combined single-host mainboard prototype

The normal rover architecture uses separate drivetrain, stepper, BLDC, and host-controller boards connected to the ESP32-P4 through UART. This keeps the individual boards inexpensive, makes damaged subsystems easier to replace, and allows the rover to reuse readily available controller hardware.

An optional [`combined single-host mainboard prototype`](Schematics/Prototype%20single%20board%20host%20computer/) places the ESP32-P4/C6 host, four stepper drivers, two brushed-motor bridges, a dedicated STM32/DRV8301 BLDC stage, power conversion, IMU connection, USB, and expansion GPIO on one PCB. It reduces the number of separate boards and UART wiring harnesses when a compact single-board installation is preferred.

The combined board is substantially more expensive than building the individual subsystem boards and joining them with UART. Its large four-layer PCB, double-sided SMD population, high component count, and specialized motor-control parts increase fabrication and assembly costs. The checked-in KiCad and Gerber files are a prototype/quotation design; the modular UART architecture remains the lower-cost and more repairable configuration used by the rover.

![KiCad 3D render of the optional combined rover control PCB](assets/prototype%20single%20board%20computer%20pcb%20layout.png)

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
5. Replacing unreliable motor drivers, rebuilding after a vibration-induced short circuit, and consolidating control on the ESP32-P4 host.
6. Designing a motorized actuator around the existing outdoor faucet instead of adding an unreliable inline valve.
7. Adding UART subsystem links, OTA updates, voice recognition/feedback, and scripted heading-controlled demonstrations.

### CAD timeline proof

The repository includes the editable Fusion designs and exports; the Macondo journal also contains recordings of each Fusion 360 feature timeline. One example is shown below.

![Fusion 360 design timeline playback](https://cdn.hackclub.com/019ffe4b-6828-72b7-a0dd-16e0c0f28e6b/Screen%20Recording%202026-08-13%20224616.gif)

## CAD and fabrication files

![Complete Autonomous Garden Rover Fusion 360 assembly](assets/full%20mower%20assembly%20render%20fusion360.png)

| Location | Contents |
| --- | --- |
| [`CAD/Assembly/`](CAD/Assembly/) | Full mower-body Fusion archive, complete STEP assembly, and individual STEP bodies |
| [`CAD/Sub-assemblies/`](CAD/Sub-assemblies/) | 11 editable subsystem designs with matching STEP exports and individual parts |
| [`CAD/Individual Printable STLs/`](CAD/Individual%20Printable%20STLs/) | 72 named, deduplicated STL exports grouped by mechanical assembly and function |

The CAD tree currently contains 12 Fusion archives/designs, 650 STEP files, and 72 STL files. Generic `Body##` parts are retained in the individual STEP exports so the complete design hierarchy is preserved; printable STLs favor human-named parts, exclude purchased metal hardware, and are deduplicated and grouped by assembly and function.

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
| Steelworks 3/4 in x 8 ft aluminum channel | 2 | $19.98 | $0.00 | $39.96 | [Lowe's](https://www.lowes.com/pd/Steelworks-3-4-in-W-x-8-ft-L-Mill-Finished-Aluminum-Weldable-Trim-Channel/3058185) |
| LGXSHOP C6374 170KV sensored BLDC motor | 1 | $29.50 | $10.00 | $39.50 | [Amazon](https://www.amazon.com/dp/B0GR88K1XP) |
| STM32F103C6T6 Blue Pill development board | 1 | $1.75 | $0.00 | $1.75 | [AliExpress](https://www.aliexpress.us/item/3256809531654480.html) |
| BTS7960 high-current motor driver board | 2 | $5.56 | $0.00 | $11.12 | [AliExpress](https://www.aliexpress.us/item/3256812145540065.html) |
| DS3230 PRO drivetrain servo motors (4-pack) | 1 | $51.37 | $0.00 | $51.37 | [AliExpress](https://www.aliexpress.us/item/3256808314550897.html) |
| STEPPERONLINE NEMA 17 stepper motors (3-pack) | 1 | $25.99 | $0.00 | $25.99 | [Amazon](https://www.amazon.com/dp/B0B38GHRH8) |
| Stepper controller board (12-24 VDC) | 1 | $22.99 | $0.00 | $22.99 | [Amazon](https://www.amazon.com/dp/B0CCVSMGXR) |
| EONO PETG 3D printer filament 1 kg black | 2 | $9.99 | $0.00 | $19.98 | [Amazon](https://www.amazon.com/EONO3D-Printer-Filament-1-75mm-2-2lbs/dp/B0G2BQQ5RT/ref=sr_1_1_sspa?crid=1L2ZTSMFNQU33&dib=eyJ2IjoiMSJ9.SEBI8_4wrU3elsHKfGW2ERnsN8KTs5kD7BhyZ9y__Z3QMtZ2FN7r_UnCcOA2tX0kFXVnp_JkbhH5ToHCJwa6H5mphhIVeCnlmKcQWw7EuqJqzap2wdZuLar07Rl_8Vn17IGjlAW7Z-r7gR4lC7Dtfe0nnChFDHjhZpz0rPiA5ZrW-2Vcxv6pA8Yn9m5KewAWe7rTjt5I6TuVCNnVJ1Y59HKh0GpZ1HSDrctdb1M_Mb8.zM1CrTE7JZozW5RqAC32XgZVFvmBgX-H2YCaIrZXDgk&dib_tag=se&keywords=petg+1kg&qid=1786660120&sprefix=petg+1kg%2Caps%2C128&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1) |
| 608 sealed steel bearings (20-pack) | 1 | $4.99 | $0.00 | $4.99 | [Amazon](https://www.amazon.com/Bearings-Premium-Steel-Sealed-Groove/dp/B0GX14YCFF/ref=sr_1_4?crid=10SNBKND149OQ&dib=eyJ2IjoiMSJ9.qWnyKcrC2gR1Mn2qmoQKOLT5AfvFyiZh06vPG-1zIPZez2VPXJQstfndu2ylGIEzapGf_3S58rWXK_9iR2qQ6GoZ9p_QSe4PqkSosthnsG6tbFhDlB-f5Zw_XfHw5twSB0SPWddbqdhwubVlA6fgAF-BLxibDqNPZHplWfyIEbFXKYd1u3x2aa1H0P1P9q2HaSQwlbPOQNzS8LIG2GRRPwsEuo-iG6U_rxuGc01EDqQ.OQoMtqLRcUuYDn2x0hnL31AfVoy9-2m7k8DtXQWT360&dib_tag=se&keywords=608%2Bbearings&qid=1786660407&sprefix=608%2Bbearings%2Caps%2C116&sr=8-4&th=1) |
| M3 screw kit (420-piece) | 1 | $8.98 | $0.00 | $8.98 | [Amazon](https://www.amazon.com/mxuteuk-420PCS-Screws-Socket-Wrench/dp/B0CSWD34KJ/ref=sr_1_8?crid=2EJ1MY8ARTFXO&dib=eyJ2IjoiMSJ9.mYqaF0B5tSbDeR49-ETzhwFfe-oW7XxaeFRt9f46-e0V6_ZIBIcWASPbMRPBz4MyZrarI0rTHsknSqSSA2Mv1c6gnKyHECdQ_mmvmfMkOmNTksfWn0dZaOG6Fq7Ao5GJKvRxG87OZhPjxuORRALFMlQedBTmzZiuwIRT_DwQbBebX8KVN70JGWwkieWeQChzPVrv0dhuZbYq25uJzkez_hJws5quPnK4NVaVnPXWPZI.nNFLVrEusWX8W3KP-ABp5YueLbETjq74Xuxoufqo8Lk&dib_tag=se&keywords=m3%2Bscrew%2B50mm&qid=1786660454&sprefix=m3%2Bscrew%2B50mm%2Caps%2C134&sr=8-8&th=1) |
| Flipsky ODESC 56 V v4.2 single-axis controller | 1 | $39.99 | $0.00 | $39.99 | [Amazon](https://www.amazon.com/dp/B0CB64MVHC) |
| 22 AWG wire (10 m) | 1 | $2.81 | $0.00 | $2.81 | [AliExpress](https://www.aliexpress.us/item/3256801511977665.html) |
| 36 V 10.4 Ah lithium battery with charger and XT60 connector | 1 | $79.99 | $0.00 | $79.99 | [eBay](https://www.ebay.com/itm/318206384216) |
| 20 A buck converter | 2 | $3.58 | $0.00 | $7.16 | [AliExpress](https://www.aliexpress.us/item/3256808333733098.html) |
| XL4005 buck converter | 1 | $1.99 | $0.00 | $1.99 | [AliExpress](https://www.aliexpress.us/item/3256808679872256.html) |
| MG996 servo motor | 1 | $3.44 | $0.00 | $3.44 | [AliExpress](https://www.aliexpress.us/item/3256802804659030.html) |
| ESP32-WROOM-32 development board with U.FL | 1 | $7.32 | $0.00 | $7.32 | [AliExpress](https://www.aliexpress.us/item/3256807142919728.html) |
| BNO080/BNO085 9-DOF sensor module | 1 | $18.99 | $0.00 | $18.99 | [Amazon](https://www.amazon.com/dp/B0HCBRBZ76) |
| Hello Hobby 3/8 in x 36 in wood dowel | 1 | $0.78 | $0.00 | $0.78 | [Walmart](https://www.walmart.com/ip/684374236) |
| Tool Bench 40 ft diamond-braid rope with winder | 1 | $1.50 | $0.00 | $1.50 | [Dollar Tree](https://www.dollartree.com/tool-bench-40-ft-diamond-braid-rope-with-winder-1-ct/295406) |
| **Estimated total** |  |  |  | **$417.47** | |

The battery listing includes its charger and XT60 connector. The M3 kit includes the matching screws and nuts used throughout the printed assemblies. The wheels are printed from the listed PETG rather than purchased separately, and PETG strands are also used as the mower line. The prototype uses the listed 22 AWG wire, with paired conductors run in parallel where additional conductor capacity is needed.

## Build it yourself

This is the order used to turn the CAD into the physical prototype. Keep the complete [`mower body.step`](CAD/Assembly/mower%20body.step) open as the placement reference and use the files under [`CAD/Individual Printable STLs/`](CAD/Individual%20Printable%20STLs/) as the print queue. The individual STEP exports are useful for measurements and modifications; only the STL tree is intended as the printable-parts list.

### Safety first

This rover contains a 10S lithium battery, a high-speed mower, high-current motor controllers, geared steppers, and a powered hose valve. Fit a fuse and accessible battery disconnect, remove the cutting line while commissioning, and test with the wheels raised. Never power a motor or the MG996 from a microcontroller rail. Verify converter outputs before connecting electronics, and read the target README before flashing a board.

### Build order

#### 1. Print and sort the mechanical parts

Print the parts by folder so hardware does not get mixed between mechanisms:

| Folder | Parts used for |
| --- | --- |
| [`structural/`](CAD/Individual%20Printable%20STLs/structural/) | Aluminum-channel joints and the battery/frame hardware |
| [`front and rear wheels/`](CAD/Individual%20Printable%20STLs/front%20and%20rear%20wheels/) | Motor housings, 2.5:1 gears, wheel adapters, and output supports |
| [`motor blade attachment/`](CAD/Individual%20Printable%20STLs/motor%20blade%20attachment/) | C6374 casing, motor mount, shaft adapter, and string head |
| [`mower height control system/`](CAD/Individual%20Printable%20STLs/mower%20height%20control%20system/) | NEMA 17 planetary, rope spools, rollers, and shaft supports |
| [`hose turret/`](CAD/Individual%20Printable%20STLs/hose%20turret/) | Yaw worm drive, pitch gearbox, bearing supports, and hose adapter |
| [`electronics holders/`](CAD/Individual%20Printable%20STLs/electronics%20holders/) | Main host, ODESC, and stepper-controller enclosures |
| [`hose_control/`](CAD/Individual%20Printable%20STLs/hose_control/) | Separate faucet actuator enclosure and linkage |

All printable parts, including the 10-inch wheels, use the two kilograms of PETG listed in the BOM and were printed on an Anycubic Kobra S1. Use a 0.12 mm layer height, three walls, and 40% infill for general parts. Use 100% infill for gears and other highly loaded power-transmission parts. Orient parts so shafts and fastener loads run across continuous perimeters where possible, and use supports wherever the slicer identifies unsupported overhangs. The mower line is made from PETG filament strands rather than a separately purchased consumable. Drill and clean printed holes only after checking them against the M3 hardware and bearings.

#### 2. Build the aluminum frame first

1. Cut the 3/4-inch aluminum U-channel into the longitudinal rails and crossmembers shown in the complete STEP assembly. Lay every piece flat in its CAD position before drilling.
2. Orient the open sides of the channels inward where possible; the prototype uses the channels as protected wire routes.
3. Join the channels with the printed aluminum-channel mounts and M3 hardware. The hand-built prototype used slightly oversized holes with large washers, allowing the frame to be squared before final tightening.
4. Install the center battery cage and spacers. Keep the heavy 10S pack near the center of the wheelbase, leaving the front bay for the mower and the rear bay for the turret.
5. Add the wire-management cover and leave the enclosure lids off until every subsystem has passed its individual test.

| Frame layout in CAD | First aluminum rolling chassis |
| --- | --- |
| ![CAD of the aluminum-channel chassis](https://cdn.hackclub.com/019ecd9e-3048-763e-8904-38117268f75b/image.png) | ![Physical aluminum-channel rolling chassis](https://cdn.hackclub.com/019ed8c7-f932-7aa7-a0f4-63fcd1a28b7b/image.png) |

#### 3. Add the drivetrain modules

1. Convert the four DS3230 units into continuous drive motors while retaining each internal potentiometer as an encoder. Bring the motor pair and three potentiometer wires out through strain relief.
2. Assemble one geared corner at a time. The 16-tooth motor gear drives the 40-tooth PETG output gear for a 2.5:1 reduction.
3. Support the wheel shaft at both ends with the printed bearing support. The support removes the rover's radial load from the motor shaft.
4. Fit the correct front or rear wheel adapter, install the 10-inch wheel, and check that the gear mesh turns freely before mounting the next corner.
5. Mount two motors on each side. The firmware treats both right motors as one track and both left motors as the other track.

#### 4. Install the mower and lift

1. Bolt the C6374 BLDC into its printed casing and mount the casing in the front center opening of the frame.
2. Fit the shaft attachment and the final string-line head. The early sacrificial razor-blade arms are documented in the journal, but the demonstrated rover uses replaceable mower string because the printed blade arms broke too frequently outdoors.
3. Assemble the mower-height NEMA 17 planetary stage, output gear, two rope spools, 8 mm shafts, and their supports. Lock the gears to the shafts with the modeled M3 cross-fasteners rather than relying on glue alone.
4. Route four equal rope runs through the printed low-friction guides: two lift points at the front and two at the rear of the mower carriage. Wind the opposing spool directions so one side pays out as the other takes up.
5. Move the lift through a small range by hand, square the mower deck, then tension and lock the ropes. Do not install cutting line until height motion and emergency stop behavior are verified.

| Mower mounted in the frame | Rope/planetary height mechanism |
| --- | --- |
| ![Mounted mower motor and printed frame](https://cdn.hackclub.com/019f2d6b-d0ab-7b6d-bc03-4b0c55bb6498/IMG_7187.jpeg) | ![Assembled mower height gearbox and rope outputs](https://cdn.hackclub.com/019f7668-f18e-7c52-b7cd-cbff927192d5/image.png) |

#### 5. Assemble the two-axis hose turret

1. Install the large module-4 yaw worm wheel at the rear of the frame. The rotating base rides on eight purchased 608 bearings held by the printed bearing mounts.
2. Assemble the NEMA 17 yaw drive, worm, shaft supports, and the later 4.36:1 planetary input stage. Set the mesh so the wheel turns without binding but cannot jump teeth.
3. Bolt the two pitch uprights to the rotating base. Assemble the pitch NEMA 17 and its 18:1 planetary gearbox, using the strengthened output shaft from the final CAD revision.
4. Fit the hose rod and printed hose adapter, then retain the hose with zip ties. Turn the pitch motor so its cable exits downward and leave enough cable loop for the complete yaw range.
5. Jog yaw and pitch separately before connecting the hose. Confirm neither axis pulls its wiring into the worm wheel.

| Turret gearbox CAD | Turret base and pitch structure |
| --- | --- |
| ![CAD of the turret worm and bearing system](https://cdn.hackclub.com/019f7615-8497-7da8-bab4-61416e4c4f07/image.png) | ![Physical turret structure during assembly](https://cdn.hackclub.com/019f763e-3c00-7ab3-b682-3cfc93d84f86/image.png) |

#### 6. Mount the electronics and faucet controller

Mount the ODESC and GD32/Ender controller in their individual printed holders on the side of the chassis. Put the ESP32-P4, STM32 drive controller, BTS7960 modules, and power converters in the main electronics enclosure. Route high-current motor wiring separately from UART and encoder wires, twist signal pairs where practical, add strain relief, and seal cable openings only after testing.

The hose valve is a separate subsystem mounted at the faucet. Assemble the continuous-rotation MG996, potentiometer feedback, ESP32, output shaft, faucet adapter, and lid from [`hose_control/`](CAD/Individual%20Printable%20STLs/hose_control/). It communicates wirelessly with the rover; no long servo cable runs between the faucet and chassis.

| Electronics during integration | Finished faucet actuator |
| --- | --- |
| ![Rover electronics during subsystem integration](https://cdn.hackclub.com/019f769f-fdfa-77c3-b291-9a491c0dfe77/image.png) | ![Weather-resistant ESP32 hose actuator](https://cdn.hackclub.com/019fc962-4971-7d67-b8b5-c9f91bc0d1e1/image.png) |

The photos and design decisions above come from the [complete Macondo build journal](https://macondo.hackclub.com/projects/9276), including the failed 1.5:1 drivetrain, blade tests, gearbox rebuilds, short-circuit repair, and ESP32-P4 electronics integration.

### Wire the rover

#### Power distribution

The prototype uses a 10S battery: approximately 36 V nominal and 42 V fully charged. Split the protected battery output into parallel branches; do not cascade the high-current drivetrain and stepper loads through one another.

| Rail | Connects to |
| --- | --- |
| Protected battery bus | ODESC motor-power input and the inputs of the DC converters |
| 24 V buck output | Creality/GD32 board and its NEMA 17 drivers |
| 8 V high-current buck output | Both BTS7960 motor-power inputs and the drivetrain motors |
| Stable 5 V regulator | ESP32-P4 host input and low-voltage support electronics |
| 3.3 V logic | BNO080 logic and potentiometer endpoints where required |
| Separate 5-6 V supply at faucet | MG996 servo; its ground joins the hose ESP32 ground |

Join controller signal grounds intentionally. Do not use the aluminum chassis as a normal current-return conductor. Measure each converter with the load disconnected before plugging in a controller.

#### Host UART and sensor harness

| Link | ESP32-P4 | Subsystem | Rate |
| --- | --- | --- | ---: |
| Drivetrain | GPIO21 TX, GPIO22 RX | STM32 PA10 RX, PA9 TX | 115200 8N1 |
| Stepper controller | GPIO2 TX, GPIO1 RX | GD32 PA10 RX, PA9 TX | 115200 8N1 |
| Mower ODESC | GPIO27 TX, GPIO47 RX | ODESC GPIO2 RX, GPIO1 TX | 115200 8N1 |
| BNO080 | GPIO5 TX, GPIO6 RX | BNO080 RX, TX | 3,000,000 8N1 |

TX always goes to RX, every link uses 3.3 V signaling, and every pair needs a shared ground. Set the BNO080 interface straps for UART-SHTP before power-up. On the Creality board, isolate the CH340 TX output before attaching the P4 TX line and do not connect a USB host to the CH340 while the direct UART tap is in use. The detailed pin notes are in [`docs/hardware/`](docs/hardware/README.md).

#### STM32, BTS7960, and wheel encoders

| Function | Right track / BTS M1 | Left track / BTS M2 |
| --- | --- | --- |
| RPWM | STM32 PA8 | STM32 PB10 |
| LPWM | STM32 PA11 | STM32 PB11 |
| R_IS feedback | STM32 PA4 | STM32 PA6 |
| L_IS feedback | STM32 PA5 | STM32 PA7 |
| Wheel-pot ADCs | PA0 and PA1 | PA2 and PA3 |

Tie each BTS7960 module's `R_EN` and `L_EN` to its enabled logic level, connect module logic ground to STM32 ground, and connect the two module motor terminals to the two motors on that side. The installed rover uses BTS direction `B` as forward. If a side runs backward, correct the motor-output polarity consistently rather than changing only one wheel.

Power each encoder potentiometer from 3.3 V and ground, with its wiper going to the listed ADC input. The potentiometers expose about 220 degrees of each motor rotation; the firmware detects wraparound and combines the 2.5:1 motor-to-wheel reduction for travel estimation.

#### Stepper sockets and mechanisms

The current firmware convention uses Creality `X` for turret yaw, `Y` for turret pitch/deploy, and `Z` for mower height. Plug each four-wire NEMA 17 into the matching socket after identifying its two coil pairs with a meter; motor-wire colors are not standardized. The Ender board's four stepper drivers share one hardware enable, so test with only one mechanism connected at first. The X/Y/Z endstop sockets are available as debounced switch inputs but the direct-motion firmware does not automatically stop on them.

#### ODESC mower and hose controller

Connect the C6374's three phase wires to ODESC M0 and connect the ODESC to the protected battery bus. The demonstrated firmware uses the single physical axis (`axis0`) in sensorless mode; the repository also preserves the later Hall-wiring experiments. Follow the board-specific voltage-divider and firmware notes in [`firmware/odesc-v42/README.md`](firmware/odesc-v42/README.md).

At the faucet, connect hose-controller GPIO14 to the MG996 signal, GPIO35 to the potentiometer wiper, and join the ESP32, potentiometer, and servo-supply grounds. Keep GPIO35 below 3.3 V. Full wiring and packet behavior are in [`firmware/esp32-hose/README.md`](firmware/esp32-hose/README.md).

### Flash code to each subsystem

Flash and test one controller at a time before joining the UART harness. The recommended order is GD32 steppers, STM32 drivetrain, hose ESP32, ODESC, and finally the ESP32-P4 host.

#### 1. Get the source

```sh
git clone https://github.com/houseofthe051010/Autonomous-Garden-Rover.git
cd Autonomous-Garden-Rover
```

#### 2. Install the toolchain for the target you are building

| Target | Required toolchain | Build guide |
| --- | --- | --- |
| ESP32-P4 primary host | ESP-IDF 6.0 or newer | [`firmware/esp32-p4/README.md`](firmware/esp32-p4/README.md) |
| STM32F103 drivetrain | PlatformIO with ST-Link support | [`firmware/stm32-drive/README.md`](firmware/stm32-drive/README.md) |
| GD32 Ender-3 stepper board | Marlin 2.1.2.5 and PlatformIO | [`firmware/gd32-stepper/README.md`](firmware/gd32-stepper/README.md) |
| ESP32 hose controller | PlatformIO | [`firmware/esp32-hose/README.md`](firmware/esp32-hose/README.md) |
| ODESC V4.2 (experimental) | Ubuntu, Tup, and `gcc-arm-none-eabi` | [`firmware/odesc-v42/README.md`](firmware/odesc-v42/README.md) |

Install ESP-IDF using Espressif's official setup instructions, then open an ESP-IDF-enabled terminal before running `idf.py`. Install PlatformIO through its CLI or IDE integration and ensure the `pio` command is available in your shell.

#### 3. Configure secrets locally

The ESP32-P4 needs a controller key that is unique to your own rover and handheld controller. Create the ignored local file from the example:

```sh
cd firmware/esp32-p4/main
cp rover_control_key.example.h rover_control_key.h
```

Replace all 32 `0x00` values with 32 cryptographically random bytes and place the identical key in the matching handheld-controller project. Do not commit this file, Wi-Fi credentials, device backups, or serial captures containing credentials. The repository ignores `rover_control_key.h` by design.

#### 4. Build and flash each target

##### ESP32-P4 primary host

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

##### STM32F103 drivetrain controller

```sh
cd firmware/stm32-drive
pio run
```

With an ST-Link connected and the board safely isolated from motor power:

```sh
pio run --target upload
```

Check the [STM32 UART protocol](docs/protocols/stm32-drive-uart.md) before connecting it to the host controller.

##### GD32 Ender-3 stepper controller

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

##### ESP32 hose controller

```sh
cd firmware/esp32-hose
pio run
```

To upload, use your board's port:

```sh
pio run -t upload --upload-port /dev/ttyUSB0
```

Read the hose-controller README before wiring the actuator; it requires a separate 5-6 V supply sized for servo stall current and a common ground.

##### ODESC V4.2

The deployed, motor-tested image and rollback artifacts are under [`firmware/odesc-v42/releases/external-vbus-gpio3-stable-20260811/`](firmware/odesc-v42/releases/external-vbus-gpio3-stable-20260811/). Read that release and the target README before choosing an image. Build or flash this target only on an isolated bench with the mower phases disconnected for the first boot.

```sh
cd firmware/odesc-v42/source/Firmware
tup init
tup
```

The output is created under `build/`. A successful build is not proof that the firmware is safe to flash or operate at pack voltage.

#### 5. Bring the system online

1. With motor power disconnected, power the host and confirm the STM32, GD32, ODESC, and BNO080 links appear healthy.
2. Join the `rover` access point and open `http://192.168.4.1/`.
3. Verify drivetrain encoder values on the host, then test left and right tracks separately with the wheels raised.
4. Open `/steppers` and jog X yaw, Y pitch, and Z mower height at low speed. Correct direction and limits before attaching the hose or cutting line.
5. Open `/sensors`, calibrate the BNO080 in its final mounted position, and verify heading changes in the correct direction.
6. Test ODESC M0 without cutting line, then test the separate hose actuator from its controller page.
7. Confirm every watchdog and `STOP ALL` path before the first ground test. Seal the electronics enclosures only after this complete dry commissioning pass.

### Use the rover

1. Charge the 36 V battery with the included charger, secure the pack in the center cage, and make sure the physical battery disconnect is off before attaching the XT60 connector.
2. Move the rover outdoors onto a clear test area. Keep people, pets, loose clothing, and debris away from the mower and wheels.
3. Turn on the battery disconnect and wait for the ESP32-P4 host and subsystem links to initialize.
4. Connect a phone or computer to the `rover` Wi-Fi access point, then open `http://192.168.4.1/` in a browser.
5. Check the status page before moving. Confirm that the drivetrain, stepper controller, ODESC, and BNO080 are reporting correctly and that the emergency-stop control is available.
6. Use the main controls for low-speed driving. Use `/steppers` to position the mower height and hose-turret axes, and `/sensors` to check the mounted IMU heading.
7. For mowing, set the deck height first, move clear of the cutting area, start the mower, and then drive forward. Stop the mower before approaching, lifting, or servicing the rover.
8. For watering, attach the hose with enough slack for the full turret range, connect the separate faucet controller, test yaw and pitch at low speed, and only then open the valve.
9. To shut down, stop the mower and every moving axis in the web interface, use `STOP ALL`, switch off the battery disconnect, and unplug the XT60 connector before maintenance.

## Repository map

| Folder | Contents |
| --- | --- |
| [`firmware/esp32-p4/`](firmware/esp32-p4/README.md) | Main rover controller, web UI, Wi-Fi, audio, storage, and UART links |
| [`firmware/stm32-drive/`](firmware/stm32-drive/README.md) | Dual-BTS7960 drivetrain controller |
| [`firmware/gd32-stepper/`](firmware/gd32-stepper/README.md) | Three-axis stepper firmware for the Ender-3 controller |
| [`firmware/esp32-hose/`](firmware/esp32-hose/README.md) | Hose-valve controller |
| [`docs/`](docs/README.md) | Wiring, architecture, protocols, and safety notes |

## Current status

The physical rover has completed outdoor drive, mowing, watering, turret, and scripted autonomous-sequence demonstrations. The ESP32-P4 host, STM32 drive link, GD32 stepper link, ODESC mower link, BNO080 link, web controls, microSD audio, and hose-control path have all been exercised on the intended hardware. The current autonomous code demonstrates fixed routines; general navigation, obstacle avoidance, and sensor-fused localization are still experimental.

This is prototype robotics hardware. Test it with the wheels raised, keep clear of moving mechanisms, verify every pin before applying power, and keep an independent power disconnect within reach.

## What I would do differently next time

I would use stronger drivetrain motors so the rover has more turning torque and margin on thick or uneven grass. For watering, I would replace the continuously attached garden hose with an onboard tank: the rover could return to a stationary filling point, store water, leave to water the garden, and then come back to refill. That would remove hose drag and make the watering route much less constrained.

With a larger budget, I would also build a custom production PCB that combines the host, power conversion, drivetrain, stepper, sensor, and mower-control electronics. The current modular design uses separate microcontrollers and UART-connected subsystems because it is less expensive and easier to repair, but a validated combined board would reduce wiring and make the final electronics package more compact.

## Third-party notices

The repository-level [MIT License](LICENSES/LICENSE) applies to original project code
and documentation only. It does not replace licenses in bundled or derived
third-party material.

| Material | Location | License / notice |
| --- | --- | --- |
| ODrive-derived ODESC source | [`firmware/odesc-v42/source/`](firmware/odesc-v42/source/) | MIT; retain the included [ODrive notice](firmware/odesc-v42/source/LICENSE.md) |
| Marlin-derived GD32 stepper patch, display assets, and binary | [`firmware/gd32-stepper/`](firmware/gd32-stepper/) | GPL-3.0-only; full text in [`LICENSES/Marlin-GPL-3.0.txt`](LICENSES/Marlin-GPL-3.0.txt) |

Before copying, modifying, or redistributing third-party material, review the
license located with that material. Add a notice here when introducing new
vendored or derived code.


## License

Original project code and documentation are released under the [MIT License](LICENSES/LICENSE). Some bundled firmware is derived from third-party projects and retains its own license; review the third-party notices above before redistributing or modifying it. The Marlin-derived stepper patches and binaries are GPL-3.0-only.
