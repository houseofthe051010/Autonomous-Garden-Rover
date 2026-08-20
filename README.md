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
* Water resistant
* Has a microphone and speaker and can run small ai vision/text recognition models
* Water flow can be controlled using a seperate esp32 controller
* Can be controlled from a phone or handheld controller
* Uses a 9axis imu and encoders on all four wheels for precise autonomous
* The aluminum chassis bends and acts as a suspension
* Supports raspberry pi 5 with ROS 2 as brain
* Supports a custom hand held controller from my previous FPV claw drone project
* Advanced flexibility for conneting new sensors, cameras, and electronics


# Building it yourself

Go to [**INSTRUCTIONS.md**](INSTRUCTIONS.md) to see the complete build instructions.

## Block schematic

This is a block level schematic of how each system is split

![Block-level electrical schematic](pictures/block%20level%20schematic%20picture%20kicad.png)

Kicad: [`Schematics/Block-level schematic/`](Schematics/Block-level%20schematic/).



## Bill of materials

| Item                                                         | Qty. | Unit price |       Total | Link                                                                                                                 |
| ------------------------------------------------------------ | ---: | ---------: | ----------: | -------------------------------------------------------------------------------------------------------------------- |
| Waveshare ESP32-P4-WIFI6 development board                   |    1 |     $26.87 |      $26.87 | [Amazon](https://www.amazon.com/dp/B0FM3SPXZG)                                                                       |
| Steelworks 3/4 in x 8 ft aluminum channel                    |    2 |     $19.98 |      $39.96 | [Lowe's](https://www.lowes.com/pd/Steelworks-3-4-in-W-x-8-ft-L-Mill-Finished-Aluminum-Weldable-Trim-Channel/3058185) |
| LGXSHOP C6374 170KV sensored BLDC motor                      |    1 |     $29.50 |      $29.50 | [Amazon](https://www.amazon.com/dp/B0GR88K1XP)                                                                       |
| STM32F103C6T6 Blue Pill development board                    |    1 |      $1.75 |       $1.75 | [AliExpress](https://www.aliexpress.us/item/3256809531654480.html)                                                   |
| BTS7960 high-current motor driver board                      |    2 |      $5.56 |      $11.12 | [AliExpress](https://www.aliexpress.us/item/3256812145540065.html)                                                   |
| DS3230 PRO drivetrain servo motors (4-pack)                  |    1 |     $41.37 |      $41.37 | [AliExpress](https://www.aliexpress.us/item/3256808314550897.html)                                                   |
| STEPPERONLINE NEMA 17 stepper motors (3-pack)                |    1 |     $25.99 |      $25.99 | [Amazon](https://www.amazon.com/dp/B0B38GHRH8)                                                                       |
| Stepper controller board (12-24 VDC)                         |    1 |     $22.99 |      $22.99 | [Amazon](https://www.amazon.com/dp/B0CCVSMGXR)                                                                       |
| EONO PETG 3D printer filament 1 kg black                     |    2 |      $9.99 |      $19.98 | [Amazon](https://www.amazon.com/EONO3D-Printer-Filament-1-75mm-2-2lbs/dp/B0G2BQQ5RT)                                 |
| 608 sealed steel bearings (20-pack)                          |    1 |      $4.99 |       $4.99 | [Amazon](https://www.amazon.com/dp/B0GX14YCFF)                                                                       |
| M3 screw kit (420-piece)                                     |    1 |      $8.98 |       $8.98 | [Amazon](https://www.amazon.com/dp/B0CSWD34KJ)                                                                       |
| Flipsky ODESC 56 V v4.2 single-axis controller               |    1 |     $39.99 |      $39.99 | [Amazon](https://www.amazon.com/dp/B0CB64MVHC)                                                                       |
| 22 AWG wire (10 m)                                           |    1 |      $2.81 |       $2.81 | [AliExpress](https://www.aliexpress.us/item/3256801511977665.html)                                                   |
| 36 V 10.4 Ah lithium battery with charger and XT60 connector |    1 |     $79.99 |      $79.99 | [eBay](https://www.ebay.com/itm/318206384216)                                                                        |
| 20 A buck converter                                          |    2 |      $3.58 |       $7.16 | [AliExpress](https://www.aliexpress.us/item/3256808333733098.html)                                                   |
| XL4005 buck converter                                        |    1 |      $1.99 |       $1.99 | [AliExpress](https://www.aliexpress.us/item/3256808679872256.html)                                                   |
| MG996 servo motor                                            |    1 |      $3.44 |       $3.44 | [AliExpress](https://www.aliexpress.us/item/3256802804659030.html)                                                   |
| ESP32-WROOM-32 development board with U.FL                   |    1 |      $7.32 |       $7.32 | [AliExpress](https://www.aliexpress.us/item/3256807142919728.html)                                                   |
| BNO080/BNO085 9-DOF sensor module                            |    1 |     $18.99 |      $18.99 | [Amazon](https://www.amazon.com/dp/B0HCBRBZ76)                                                                       |
| Hello Hobby 3/8 in x 36 in wood dowel                        |    1 |      $0.78 |       $0.78 | [Walmart](https://www.walmart.com/ip/684374236)                                                                      |
| Tool Bench 40 ft diamond-braid rope with winder              |    1 |      $1.50 |       $1.50 | [Dollar Tree](https://www.dollartree.com/tool-bench-40-ft-diamond-braid-rope-with-winder-1-ct/295406)                |
| **Estimated total**                                          |      |            | **$397.47** |                                                                                                                      |

## What I would do differently next time

- Add cameras and GPS+RTK to the autonomous sensors
- Use stronger drivetrain motors to support a higher speed and better turning (though it would increase the price to build)
- Replace garden hose adapter system with a water tank onboard the rover
- Make a custom PCB that combines all the electronics and sub systems into one board with quick connectors for peripherals (much more expensive though)


## Repository Structure

In this repo, I have included everything needed to replicate and modify this project

- Firmware images and source code
- CAD source, assemblies, steps, and STLs
- Visual instruction manual
- Electrical schematics
- 