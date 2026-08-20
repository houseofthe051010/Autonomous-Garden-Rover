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

### Why have many scattered subsystems?

I also designed a single-board host computer that combines the major electronics onto one PCB, with connectors for each peripheral. I haven't manufactured or tested it yet, as it is a option for people wanting to build this project to avoid the hassle of sub systems and communication links. The gerber files for the PCB is at:

[**Single-board host computer schematic and PCB files**](Schematics/Prototype%20single%20board%20host%20computer/)

The benefit with my current electronics layout is that it is significantly cheaper (helps keep the 400 dollar budget) and easier to repair, as the PCBs are already mass produced and working. Its like comparing a mini pc and a desktop pc, you can swap out parts in the desktop whenever you want. 

### Flashing the firmware


I have compiled images of firmware for every rover controller in [`firmware/`](firmware/). The `source/` folders also have their code if you want to modify them.


### ESP32-P4 main controller


[`firmware/esp32-p4/images/esp32-p4-complete.bin`](firmware/esp32-p4/images/esp32-p4-complete.bin)

Install [esptool](https://docs.espressif.com/projects/esptool/en/latest/esp32p4/installation.html), connect ESP32-P4, run

```sh
esptool --chip esp32p4 --port COM3 erase-flash
esptool --chip esp32p4 --port COM3 write-flash 0x0 firmware/esp32-p4/images/esp32-p4-complete.bin
```

COM3 Should be replaced with whatever your serial port number is.


### STM32 drivetrain controller

[`firmware/stm32-drive/images/firmware.bin`](firmware/stm32-drive/images/firmware.bin)

You need an **ST-Link** to program the STM32F103. If you are not familiar with connecting an ST-Link, this video shows the wiring and setup:

[**How to connect and use an ST-Link**](https://www.youtube.com/watch?v=KgR3uM21y7o)

1. Connect ST-link to STM32F103
2. Open STM32CubeProgrammer (You need to download this to program it)
3. Connect using ST-Link / SWD.
4. Select `firmware.bin`.
5. Flash it at 0x08000000


### GD32 stepper controller


[`firmware/gd32-stepper/images/GD3P4V1.BIN`](firmware/gd32-stepper/images/GD3P4V1.BIN)

The GD32 Stepper controller has a built-in microSD bootloader, so flashing it is pretty easy

1. Get a FAT32 formatted MicroSD card
2. Copy GD3P4V1.BIN into the card and eject
3. Insert it into the stepper controller
4. Power on the board to load the new firmware 


### ESP32 hose controller


[`firmware/esp32-hose/images/esp32-wroom-hose-complete.bin`](firmware/esp32-hose/images/esp32-wroom-hose-complete.bin)

Connect the ESP32 and run

```sh
esptool --chip esp32 --port COM3 erase-flash
esptool --chip esp32 --port COM3 write-flash 0x0 firmware/esp32-hose/images/esp32-wroom-hose-complete.bin
```

Make sure you use the COM port your esp32 is on and not my COM3


### ODESC V4.2 mower controller

This is the same process as the earlier STM32F103

[`firmware/odesc-v42/images/`](firmware/odesc-v42/images/)

If you are not familiar with connecting an ST-Link, this video shows the wiring and setup:

[**How to connect and use an ST-Link**](https://www.youtube.com/watch?v=KgR3uM21y7o)

1. Connect ST-link to the ODESC board's 5-pin connector that comes with it
2. Open STM32CubeProgrammer (You need to download this to program it)
3. Connect using ST-Link / SWD.
4. Select `firmware.bin`.
5. Flash it at 0x08000000






## What I would do differently next time

- Add cameras and GPS+RTK to the autonomous sensors
- Use stronger drivetrain motors to support a higher speed and better turning (though it would increase the price to build)
- Replace garden hose adapter system with a water tank onboard the rover
- Make a custom PCB that combines all the electronics and sub systems into one board with quick connectors for peripherals (much more expensive though)
