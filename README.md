# Autonomous Garden Rover

This is an open source outdoor garden robot that can mow grass and water plants for under $400. It features a powerful 3000 watt C6374 motor for mowing grass with electronic height control, and a large Nema17 turret system for watering at target places that supports a normal garden hose. The rover currently uses a 9 axis IMU + wheel encoders for autonomous function. The brain of this rover is a powerful ESP32-P4, which gives this rover the capability to listen and respond to commands using its built in microphone and speaker. 

## Inspiration

I spend hours each week gardening and mowing grass. As someone into robotics, I realized I could build a garden rover to automate some of these repetitive chores. This project gave me a reason to learn more about robotics while building something useful for my yard.

[**Watch the demo**](https://youtu.be/P-olpegfmmU) · [**Macondo journal**](https://macondo.hackclub.com/projects/9276) · [**See the assembly**](CAD/)

[![Autonomous Garden Rover demo](https://i.ytimg.com/vi/P-olpegfmmU/maxresdefault.jpg)](https://youtu.be/P-olpegfmmU)

## Features


* Under $400, anyone can build it
* All four wheels are motorized
* C6374 Motor offers 3000W+ of power, more than most commercial systems
* Nema17 planetary motors lift and raise the mower using a rope system.
* The turret system uses high reduction worm + dual stage planetary drives to shoot water
* Water flow can be controlled using a seperate esp32 controller
* Can be controlled from a phone or handheld controller
* Uses a 9axis imu and encoders on all four wheels for autonomous
* The aluminum chassis bends and acts as a suspension


# Building it yourself

The complete development process is documented in my [Macondo journal](https://macondo.hackclub.com/projects/9276).
Below is a guide on how build the rover.

## System architecture

To replicate and built the rover, one should understand that there is one host computer, and many sub-system computers that control the whole rover. Each sub-system is linked using a UART channel to the host computer. This means that the host controller can be any computer you want, from a raspberry pi using ROS 2 to a esp32.

| Subsystem       | Hardware                | Purpose                                            |
| --------------- | ----------------------- | -------------------------------------------------- |
| Main controller | ESP32-P4 + ESP32-C6     | Controls, AI processing and autonomous |
| Drivetrain      | STM32F103  | Encoder feedback and drivetrain  |
| Stepper motion  | GD32F303 Ender-3 board  | Controls mower height and turret axes              |
| Mower           | STM32 oDrive clone      | Controls the C6374 mowing motor                               |
| Pigeon         | BNO080                  | The 9 axis IMU                                   |
| Hose valve      | ESP32-WROOM-32   | Wireless faucet control                            |

## Why have many scattered subsystems?

I also designed a single-board host computer that combines the major electronics onto one PCB, with connectors for each peripheral. I haven't manufactured or tested it yet, as it is a option for people wanting to build this project to avoid the hassle of sub systems and communication links. The gerber files for the PCB is at:

[**Single-board host computer schematic and PCB files**](Schematics/Prototype%20single%20board%20host%20computer/)

The benefit with my current electronics layout is that it is significantly cheaper (helps keep the 400 dollar budget) and easier to repair, as the PCBs are already mass produced and working. Its like comparing a mini pc and a desktop pc, you can swap out parts in the desktop whenever you want. 

### Block schematic

This is a block level schematic of how each system is split

![Block-level electrical schematic](pictures/block%20level%20schematic%20picture%20kicad.png)

Kicad: [`Schematics/Block-level schematic/`](Schematics/Block-level%20schematic/).


## Mechanical design





## Firmware


## Bill of materials

| Item                                                         | Qty. | Unit price | Shipping |       Total | Link                                                                                                                 |
| ------------------------------------------------------------ | ---: | ---------: | -------: | ----------: | -------------------------------------------------------------------------------------------------------------------- |
| Waveshare ESP32-P4-WIFI6 development board                   |    1 |     $26.87 |    $0.00 |      $26.87 | [Amazon](https://www.amazon.com/dp/B0FM3SPXZG)                                                                       |
| Steelworks 3/4 in x 8 ft aluminum channel                    |    2 |     $19.98 |    $0.00 |      $39.96 | [Lowe's](https://www.lowes.com/pd/Steelworks-3-4-in-W-x-8-ft-L-Mill-Finished-Aluminum-Weldable-Trim-Channel/3058185) |
| LGXSHOP C6374 170KV sensored BLDC motor                      |    1 |     $29.50 |   $10.00 |      $39.50 | [Amazon](https://www.amazon.com/dp/B0GR88K1XP)                                                                       |
| STM32F103C6T6 Blue Pill development board                    |    1 |      $1.75 |    $0.00 |       $1.75 | [AliExpress](https://www.aliexpress.us/item/3256809531654480.html)                                                   |
| BTS7960 high-current motor driver board                      |    2 |      $5.56 |    $0.00 |      $11.12 | [AliExpress](https://www.aliexpress.us/item/3256812145540065.html)                                                   |
| DS3230 PRO drivetrain servo motors (4-pack)                  |    1 |     $51.37 |    $0.00 |      $51.37 | [AliExpress](https://www.aliexpress.us/item/3256808314550897.html)                                                   |
| STEPPERONLINE NEMA 17 stepper motors (3-pack)                |    1 |     $25.99 |    $0.00 |      $25.99 | [Amazon](https://www.amazon.com/dp/B0B38GHRH8)                                                                       |
| Stepper controller board (12-24 VDC)                         |    1 |     $22.99 |    $0.00 |      $22.99 | [Amazon](https://www.amazon.com/dp/B0CCVSMGXR)                                                                       |
| EONO PETG 3D printer filament 1 kg black                     |    2 |      $9.99 |    $0.00 |      $19.98 | [Amazon](https://www.amazon.com/EONO3D-Printer-Filament-1-75mm-2-2lbs/dp/B0G2BQQ5RT)                                 |
| 608 sealed steel bearings (20-pack)                          |    1 |      $4.99 |    $0.00 |       $4.99 | [Amazon](https://www.amazon.com/dp/B0GX14YCFF)                                                                       |
| M3 screw kit (420-piece)                                     |    1 |      $8.98 |    $0.00 |       $8.98 | [Amazon](https://www.amazon.com/dp/B0CSWD34KJ)                                                                       |
| Flipsky ODESC 56 V v4.2 single-axis controller               |    1 |     $39.99 |    $0.00 |      $39.99 | [Amazon](https://www.amazon.com/dp/B0CB64MVHC)                                                                       |
| 22 AWG wire (10 m)                                           |    1 |      $2.81 |    $0.00 |       $2.81 | [AliExpress](https://www.aliexpress.us/item/3256801511977665.html)                                                   |
| 36 V 10.4 Ah lithium battery with charger and XT60 connector |    1 |     $79.99 |    $0.00 |      $79.99 | [eBay](https://www.ebay.com/itm/318206384216)                                                                        |
| 20 A buck converter                                          |    2 |      $3.58 |    $0.00 |       $7.16 | [AliExpress](https://www.aliexpress.us/item/3256808333733098.html)                                                   |
| XL4005 buck converter                                        |    1 |      $1.99 |    $0.00 |       $1.99 | [AliExpress](https://www.aliexpress.us/item/3256808679872256.html)                                                   |
| MG996 servo motor                                            |    1 |      $3.44 |    $0.00 |       $3.44 | [AliExpress](https://www.aliexpress.us/item/3256802804659030.html)                                                   |
| ESP32-WROOM-32 development board with U.FL                   |    1 |      $7.32 |    $0.00 |       $7.32 | [AliExpress](https://www.aliexpress.us/item/3256807142919728.html)                                                   |
| BNO080/BNO085 9-DOF sensor module                            |    1 |     $18.99 |    $0.00 |      $18.99 | [Amazon](https://www.amazon.com/dp/B0HCBRBZ76)                                                                       |
| Hello Hobby 3/8 in x 36 in wood dowel                        |    1 |      $0.78 |    $0.00 |       $0.78 | [Walmart](https://www.walmart.com/ip/684374236)                                                                      |
| Tool Bench 40 ft diamond-braid rope with winder              |    1 |      $1.50 |    $0.00 |       $1.50 | [Dollar Tree](https://www.dollartree.com/tool-bench-40-ft-diamond-braid-rope-with-winder-1-ct/295406)                |
| **Estimated total**                                          |      |            |          | **$417.47** |                                                                                                                      |





## What I would do differently next time

- Add cameras and GPS+RTK to the autonomous sensors
- Use stronger drivetrain motors to support a higher speed and better turning (though it would increase the price to build)
- Replace garden hose adapter system with a water tank onboard the rover
- Make a custom PCB that combines all the electronics and sub systems into one board with quick connectors for peripherals (much more expensive though)
