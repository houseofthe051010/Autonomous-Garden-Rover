# Autonomous Garden Rover

This is an open source outdoor garden robot that can mow grass and water plants. It features a powerful 3000 watt C6374 motor for mowing grass with electronic height control, and a large Nema17 turret system for watering at target places that supports a normal garden hose. The rover currently uses a 9 axis IMU + wheel encoders for autonomous function. The brain of this rover is a powerful ESP32-P4, which gives this rover the capability to listen and respond to commands using its built in microphone and speaker. 


# Inspiration

I spend hours per week gardening and mowing grass. As someone into robotics, I realized that I could make a garden rover that automates all of this for me. This robot gave be a reason to learn about robotics and helped automate my daily chores.

[**Watch the demo**](https://youtu.be/P-olpegfmmU) · [**Macondo journal**](https://macondo.hackclub.com/projects/9276) · [**See the assembly**](CAD/)

[![Autonomous Garden Rover demo](https://i.y    timg.com/vi/P-olpegfmmU/maxresdefault.jpg)](https://youtu.be/P-olpegfmmU)

## Features

* All four wheels are motorized
* C6374 Motor offers 3000W+ of power, more than most commercial systems
* Nema17 planetary motors lift and raise the mower using a rope system.
* The turret system uses high reduction worm + dual stage planetary drives to shoot water
* Water flow can be controlled using a seperate esp32 controller
* Can be controlled from a phone or handheld controller
* Uses a 9axis imu and encoders on all four wheels for autonomous

## System architecture

To replicate and built the rover, one should understand that there is one host computer, and many sub-system computers that control the whole rover. Each sub-system is linked using a UART channel to the host computer. This means that the host controller can be any computer you want, from a raspberry pi to a esp32.

| Subsystem       | Hardware                | Purpose                                            |
| --------------- | ----------------------- | -------------------------------------------------- |
| Main controller | ESP32-P4 + ESP32-C6     | Controls, AI processing and autonomous |
| Drivetrain      | STM32F103  | Encoder feedback and drivetrain  |
| Stepper motion  | GD32F303 Ender-3 board  | Controls mower height and turret axes              |
| Mower           | STM32 oDrive clone      | Controls the C6374 mowing motor                               |
| Pigeon         | BNO080                  | The 9 axis IMU                                   |
| Hose valve      | ESP32-WROOM-32   | Wireless faucet control                            |

### Electrical schematic

![Block-level electrical schematic](pictures/block%20level%20schematic%20picture%20kicad.png)

Editable KiCad files are available in [`Schematics/Block-level schematic/`](Schematics/Block-level%20schematic/).

### Optional single-board controller

I also designed an experimental four-layer PCB that combines the host, motor drivers, stepper drivers, power conversion, IMU, and expansion I/O.

The physical rover currently uses separate controller boards connected over UART because they are cheaper, easier to replace, and easier to repair.

![Prototype combined PCB](pictures/prototype%20single%20board%20computer%20pcb%20layout.png)

[KiCad source and Gerbers](Schematics/Prototype%20single%20board%20host%20computer/)

## Mechanical design

### Drivetrain

The rover uses an aluminum U-channel chassis and four independently powered wheels.

The original drivetrain used a 1.5:1 reduction, but outdoor testing showed that it did not have enough turning torque on grass. I redesigned the transmissions to use 16:40 gears for a **2.5:1 reduction**.

### Mower

The original mower used printed sacrificial blade arms. They broke too frequently during outdoor testing, so I replaced them with a string-line cutting head.

A NEMA 17 gearbox drives two rope spools that raise and lower the mower from four points.

### Watering turret

The turret uses:

* Module-4 worm-drive yaw axis
* Eight 608 bearings
* 4.36:1 yaw input gearbox
* 18:1 pitch gearbox
* NEMA 17 motors

A separate ESP32-controlled actuator mounted at the faucet opens and closes the water supply wirelessly.

## Build journey

The rover went through several major redesigns during outdoor testing:

* Wood proof of concept → aluminum chassis
* 1.5:1 drivetrain → 2.5:1 drivetrain for more torque
* Printed razor blades → string-line mower
* Redesigned turret gearbox after slipping
* Rebuilt electronics after motor-driver failures and a vibration-induced short
* Added the ESP32-P4 host, UART subsystem control, OTA updates, and autonomous routines

| Early chassis                                                                             | Mower CAD                                                                             |
| ----------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------- |
| ![Early chassis](https://cdn.hackclub.com/019ed8c7-f932-7aa7-a0f4-63fcd1a28b7b/image.png) | ![Mower CAD](https://cdn.hackclub.com/019f2ad2-f248-7ab3-a5b5-6bd28f143fd7/image.png) |

| Turret                                                                             | Faucet controller                                                                             |
| ---------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------- |
| ![Turret](https://cdn.hackclub.com/019f7615-8497-7da8-bab4-61416e4c4f07/image.png) | ![Faucet controller](https://cdn.hackclub.com/019fc962-4971-7d67-b8b5-c9f91bc0d1e1/image.png) |

The complete development process is documented in my [128-hour Macondo journal](https://macondo.hackclub.com/projects/9276).

## CAD and fabrication files

![Full Fusion 360 assembly](pictures/full%20mower%20assembly%20render%20fusion360.png)

| Location                                                               | Contents                                  |
| ---------------------------------------------------------------------- | ----------------------------------------- |
| [`CAD/Assembly/`](CAD/Assembly/)                                       | Complete Fusion assembly and STEP exports |
| [`CAD/Sub-assemblies/`](CAD/Sub-assemblies/)                           | Editable subsystem designs                |
| [`CAD/Individual Printable STLs/`](CAD/Individual%20Printable%20STLs/) | 72 printable STL files                    |

The repository includes the editable Fusion 360 designs, complete STEP exports, individual STEP parts, and printable STL files needed to reproduce the mechanical system.

## Firmware

The ESP32-P4 is the main controller. Individual controllers handle real-time motor control and communicate with it over UART.

* **STM32 drivetrain:** left/right drivetrain commands and wheel feedback
* **GD32 stepper controller:** mower height, turret yaw, and turret pitch
* **ODESC:** mower speed and telemetry
* **BNO080:** heading feedback
* **Hose ESP32:** wireless faucet control

The autonomous demo contains two scripted routines:

* A heading-controlled mowing pass
* A watering sweep that opens the faucet, positions the turret, sweeps the hose, and advances the rover

Firmware source is under [`firmware/`](firmware/).

## Bill of materials

**Keep the existing BOM table here.**

## Build it yourself

### Safety

This rover contains a 10S lithium battery, high-current motor controllers, a high-speed mower, geared mechanisms, and a powered hose valve.

* Use a fuse and accessible battery disconnect
* Remove mower line during initial testing
* Test the drivetrain with the wheels raised
* Never power motors from a microcontroller rail
* Verify regulator voltages before connecting electronics
* Keep people and pets away while testing

### Mechanical assembly

1. Print the parts from [`CAD/Individual Printable STLs/`](CAD/Individual%20Printable%20STLs/).
2. Build the aluminum chassis using the full STEP assembly as a reference.
3. Install the four 2.5:1 drivetrain modules.
4. Install the mower and rope-lift mechanism.
5. Assemble the yaw and pitch turret.
6. Mount the electronics.
7. Assemble the separate faucet controller.

### Wiring

Use the block-level schematic and wiring tables below when connecting the controllers.

**Keep your existing power-distribution, UART, STM32/BTS7960, stepper, ODESC, and hose-controller wiring tables here.**

### Firmware installation

Flash and test each controller separately before connecting the full system.

Recommended order:

1. GD32 stepper controller
2. STM32 drivetrain controller
3. Hose ESP32
4. ODESC
5. ESP32-P4 host

**Keep your existing build and flashing commands here.**

### First startup

1. Power the electronics with the motor outputs disconnected.
2. Verify communication with each subsystem.
3. Test the drivetrain with the wheels raised.
4. Jog each stepper axis.
5. Calibrate and verify the BNO080.
6. Test the mower without cutting line.
7. Test the faucet actuator.
8. Confirm the emergency-stop behavior before outdoor testing.


## What I would do differently next time

- Add cameras and GPS+RTK to the autonomous sensors
- Use stronger drivetrain motors to support a higher speed and better turning (though it would increase the price to build)
- Replace garden hose adapter system with a water tank onboard the rover
- Make a custom PCB that combines all the electronics and sub systems into one board with quick connectors for peripherals (much more expensive though)
