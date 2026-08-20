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

## Mechanical and electrical build

### 1. Print the parts

The printable files are grouped in [`CAD/Individual Printable STLs/`](CAD/Individual%20Printable%20STLs/).

#### Drivetrain

From [`front and rear wheels/`](CAD/Individual%20Printable%20STLs/front%20and%20rear%20wheels/):

- 4 × `mower wheel.stl`
- 4 × `motor spur gear.stl`
- 4 wheel output gears total: use the front and rear output-gear variants in their matching positions
- 6 drivetrain holders total: the CAD folder calls the two holder variants `drivetrain motor casing.stl` and `drivetrain motor holder.stl`; this guide refers to both as **drivetrain holders**

#### Mower head

From [`motor blade attachment/`](CAD/Individual%20Printable%20STLs/motor%20blade%20attachment/):

- `mower motor mount attachment.stl`
- `mower motor shaft attachment.stl`
- `mower main casing.stl`
- `mower motor casing.stl`
- `mower motor casing top.stl`
- `mower motor casing lid.stl`
- 4 × `blade arm.stl` or 4 × `string line mower attachment.stl`

#### Height control

Print the parts in [`mower height control system/`](CAD/Individual%20Printable%20STLs/mower%20height%20control%20system/) as they are called out in Section 4.

#### Structure and electronics

- [`aluminum channel mount.stl`](CAD/Individual%20Printable%20STLs/structural/aluminum%20channel%20mount.stl): 4 for the mower carriage, 2 for the main-electronics bridge, and 6 for the battery/turret rails
- 2 × [`battery holder perimeter part.stl`](CAD/Individual%20Printable%20STLs/structural/battery%20holder%20perimeter%20part.stl)
- Main enclosure: [`electronics holder bottom casing.stl`](CAD/Individual%20Printable%20STLs/electronics%20holders/main%20electronics%20holder/electronics%20holder%20bottom%20casing.stl), [`electronics holder top casing.stl`](CAD/Individual%20Printable%20STLs/electronics%20holders/main%20electronics%20holder/electronics%20holder%20top%20casing.stl), and [`wire management cover.stl`](CAD/Individual%20Printable%20STLs/electronics%20holders/main%20electronics%20holder/wire%20management%20cover.stl)
- ODESC enclosure: use the two files in [`odrive holder v2/`](CAD/Individual%20Printable%20STLs/electronics%20holders/odrive%20holder%20v2/)
- Stepper enclosure: use the two files in [`stepper controller holder/`](CAD/Individual%20Printable%20STLs/electronics%20holders/stepper%20controller%20holder/)

#### Hose turret and faucet controller

Use the files in [`hose turret/`](CAD/Individual%20Printable%20STLs/hose%20turret/) and [`hose_control/`](CAD/Individual%20Printable%20STLs/hose_control/) as named in Sections 6 and 7.

### 2. Cut and assemble the chassis

The complete cut list is:

| Use | Length | Quantity |
| --- | ---: | ---: |
| Main chassis cross rails | 420 | 3 |
| Main chassis side rails | 670 | 2 |
| Mower carriage side rails | 330 | 2 |
| Mower carriage crosspieces | 76 | 2 |
| Main-electronics supports | 150 | 2 |
| Battery and turret rails | 400 | 4 |
| ODESC/stepper support | 300 | 1 |

Lay out the 420 mm and 670 mm pieces as shown.

![Main chassis cut layout](pictures/assembly-guide/rover-build-01-frame-cut-layout.png)

#### Build the four powered wheel modules

1. Open each DS3230 PRO servo and remove its control PCB.
2. Bring out five wires: two motor leads, potentiometer 3.3 V, potentiometer ground, and the potentiometer wiper.
3. Reassemble the servo and place it in a drivetrain holder.

<p align="center">
  <img src="pictures/assembly-guide/rover-build-02-drivetrain-wheel-cad.png" alt="Complete drivetrain wheel CAD" width="48%">
  <img src="pictures/assembly-guide/rover-build-03-drivetrain-servo-rewire.png" alt="Opened drivetrain servo with motor and potentiometer wires" width="35%">
</p>

Attach `motor spur gear.stl` to the original servo horn with two M3 screws.

<p align="center">
  <img src="pictures/assembly-guide/rover-build-04-drivetrain-holder-cad.png" alt="Servo installed in drivetrain holder" width="48%">
  <img src="pictures/assembly-guide/rover-build-05-motor-spur-gear-fasteners.png" alt="Motor spur gear attached to servo horn" width="48%">
</p>

Push the wheel shaft through the matching output gear and wheel. Slide it through the holder, then lock it with an M3 screw in one of the three shaft slots.

<p align="center">
  <img src="pictures/assembly-guide/rover-build-06-wheel-output-gear-shaft.png" alt="Wheel output gear and shaft" width="48%">
  <img src="pictures/assembly-guide/rover-build-07-wheel-shaft-lock-screw.png" alt="M3 wheel shaft lock screw" width="48%">
</p>

Fit four powered holders at the corners and two unloaded holders at the center cross rail. Drill four M3 mounting holes at each corner and three at each center holder. Square the frame before tightening it.

![Chassis and drivetrain-holder layout](pictures/assembly-guide/rover-build-08-chassis-wheel-holder-layout.png)

### 3. Build the mower carriage and cutting head

Make the carriage from two 330 mm rails and two 76 mm crosspieces. Use four printed aluminum-channel mounts. The finished carriage is about 330 × 130 × 50 mm.

<p align="center">
  <img src="pictures/assembly-guide/rover-build-09-mower-carriage-dimensions.png" alt="Mower carriage dimensions" width="48%">
  <img src="pictures/assembly-guide/rover-build-10-mower-carriage-crosspiece.png" alt="Mower carriage 76 mm crosspiece" width="48%">
</p>

Drill the main chassis and bolt the carriage across its center.

![Mower carriage attached to chassis](pictures/assembly-guide/rover-build-11-mower-carriage-mounted.png)

1. Bolt `mower motor mount attachment.stl` to the C6374 using the four supplied M4 screws.
2. Press-fit `mower motor shaft attachment.stl` onto the motor shaft.
3. Attach `mower main casing.stl` with four 50 mm M3 screws.
4. Stack `mower motor casing.stl`, then glue on `mower motor casing top.stl`.
5. Keep `mower motor casing lid.stl` removable for service.

<p align="center">
  <img src="pictures/assembly-guide/rover-build-12-mower-motor-mount.png" alt="Mower motor mount" width="48%">
  <img src="pictures/assembly-guide/rover-build-13-mower-shaft-attachment.png" alt="Mower shaft attachment" width="48%">
</p>

<p align="center">
  <img src="pictures/assembly-guide/rover-build-14-mower-casing-stack.png" alt="Mower casing stack" width="48%">
  <img src="pictures/assembly-guide/rover-build-15-mower-casing-lid.png" alt="Mower casing lid" width="48%">
</p>

Attach four blade arms or four string-line holders to the shaft attachment with M3 screws. Leave the actual blades or line off until all electrical tests pass.

<p align="center">
  <img src="pictures/assembly-guide/rover-build-16-cutting-attachments.png" alt="Four mower cutting attachments" width="48%">
  <img src="pictures/assembly-guide/rover-build-17-rover-before-height-control.png" alt="Rover before height-control installation" width="48%">
</p>

### 4. Build the mower height control

#### Assemble the gearbox and rope shafts

1. Print `height control base shell.stl` and `rope rod top stabilizer.stl`.
2. Mount the stabilizer using the rear NEMA 17 holes and longer 35 mm M3 screws.
3. Install two `rope output rod attachment.stl` parts with two 608 bearings per shaft.

<p align="center">
  <img src="pictures/assembly-guide/rover-build-18-height-control-base-shell.png" alt="Height-control base shell" width="48%">
  <img src="pictures/assembly-guide/rover-build-19-height-control-rope-rod-stabilizer.png" alt="Rope rod stabilizer" width="48%">
</p>

![Rope output shafts and four bearings](pictures/assembly-guide/rover-build-20-height-control-rope-output-bearings.png)

Press `sun gear.stl` onto the NEMA 17 shaft. Fit the planet gears around it on `planetary output.stl`, then install the ring-gear casing and `height control base shell.stl`. Secure the stack with four 35 mm M3 screws.

<p align="center">
  <img src="pictures/assembly-guide/rover-build-21-height-control-planet-carrier.png" alt="Height-control planet gears and carrier" width="31%">
  <img src="pictures/assembly-guide/rover-build-22-height-control-sun-gear.png" alt="Height-control sun gear" width="31%">
  <img src="pictures/assembly-guide/rover-build-23-height-control-ring-gear-casing.png" alt="Height-control ring gear casing" width="31%">
</p>

![Height-control base installed above gearbox](pictures/assembly-guide/rover-build-24-height-control-base-mounted.png)

Bolt the module to the chassis with four M3 screws. Fit `nema17 planetary output gear.stl` in the center and one `output shaft rope rod gear.stl` on each side shaft. Drill and tap each shaft for an M3 locking screw.

<p align="center">
  <img src="pictures/assembly-guide/rover-build-25-height-control-module-on-chassis.png" alt="Height-control module mounted on chassis" width="48%">
  <img src="pictures/assembly-guide/rover-build-26-height-control-output-gears.png" alt="Height-control output gears" width="48%">
</p>

Fit the printed enclosure around the NEMA 17.

![Height-control motor enclosure](pictures/assembly-guide/rover-build-27-height-control-motor-enclosure.png)

#### Route the lift ropes

Mount four `spacer.stl` rollers at R1–R4 and `rope tensioner.stl` at R5. Add `height control system rope blocker.stl` above the rollers so the rope cannot jump out.

![Height-control spacer and tensioner positions](pictures/assembly-guide/rover-build-28-height-control-rope-spacers.png)

Install `rope friction reducer.stl` guides in 8 mm holes along the rope path.

![Height-control rope guides](pictures/assembly-guide/rover-build-29-height-control-rope-guides.png)

Anchor the ropes to the mower lid. Route them through the guides and rollers, then fasten them to the two winding shafts with M3 screws. Each shaft needs one rope run for lifting and one for lowering. Turn the system by hand and confirm that both sides remain level.

![Height-control rope routing](pictures/assembly-guide/rover-build-30-height-control-rope-routing.png)

### 5. Mount the main enclosure and battery rails

The main enclosure uses exactly these three printed parts: bottom casing, top casing, and wire-management cover.

![Main-electronics STL files](pictures/assembly-guide/rover-build-31-main-electronics-stl-list.png)

Cut two 150 mm channels. Mount them along the sides with two printed channel mounts and 50 mm M3 screws, then bolt the electronics enclosure across them with 25 mm M3 screws.

<p align="center">
  <img src="pictures/assembly-guide/rover-build-32-main-electronics-support-channels.png" alt="Main-electronics support channels" width="48%">
  <img src="pictures/assembly-guide/rover-build-33-main-electronics-enclosure.png" alt="Main-electronics enclosure on support channels" width="48%">
</p>

Cut four 400 mm rails. Two support the battery and two support the hose turret. Use six printed channel mounts. Install the battery-retaining rail vertically, following the offsets in the figure.

![Battery and turret rail layout](pictures/assembly-guide/rover-build-34-battery-turret-channel-layout.png)

Fit two `battery holder perimeter part.stl` pieces 215 mm apart, then place the battery between the vertical retainers.

<p align="center">
  <img src="pictures/assembly-guide/rover-build-35-battery-retainer-spacing.png" alt="Battery retainer spacing" width="48%">
  <img src="pictures/assembly-guide/rover-build-36-battery-installed-cad.png" alt="Battery installed between retaining rails" width="48%">
</p>

### 6. Build the hose turret

The completed yaw and pitch assembly should match this layout.

![Complete hose turret CAD](pictures/assembly-guide/rover-build-37-hose-turret-overview.png)

#### Yaw axis

Mount four `turret wheel bearing rod.stl` parts with 50 mm M3 screws and locknuts. Check that both diagonals measure 148 mm so the rods are square.

<p align="center">
  <img src="pictures/assembly-guide/rover-build-38-turret-bearing-rods.png" alt="Four turret bearing rods" width="48%">
  <img src="pictures/assembly-guide/rover-build-39-turret-bearing-rod-spacing.png" alt="Turret bearing rod diagonal spacing" width="48%">
</p>

Mount the yaw NEMA 17 and its planetary stage 67.5 mm from the corner rod. Use `turret yaw nema17 attachment.stl`, the gearbox support and casing, the sun gear, planet gears, and the first-stage output.

<p align="center">
  <img src="pictures/assembly-guide/rover-build-40-turret-yaw-stepper-mounted.png" alt="Yaw stepper mounted" width="48%">
  <img src="pictures/assembly-guide/rover-build-41-turret-yaw-stepper-offset.png" alt="Yaw stepper 67.5 mm offset" width="48%">
</p>

Fit `worm drive.stl` to `worm drive rod.stl`. Support its far end with one 608 bearing in `turret yaw bearing holder.stl`, fastened with two 30 mm M3 screws.

![Turret yaw worm drive](pictures/assembly-guide/rover-build-42-turret-worm-drive.png)

Install four 608 bearings on the rods, add `turret wheel bearing rod spacer.stl`, lower `worm wheel.stl` into place, then add four more bearings. Use washers under the M3 nuts so the bearings cannot lift off.

![Worm wheel between eight bearings](pictures/assembly-guide/rover-build-43-turret-worm-wheel-bearings.png)

#### Pitch axis

Bolt `turret pitch nema17 spacer.stl` to the worm wheel with 25 mm M3 screws. Mount `nema17 holder turret pitch.stl` 10 mm above the bottom edge.

<p align="center">
  <img src="pictures/assembly-guide/rover-build-44-turret-pitch-stepper-adapter.png" alt="Pitch stepper adapter on worm wheel" width="48%">
  <img src="pictures/assembly-guide/rover-build-45-turret-pitch-stepper-holder.png" alt="Pitch NEMA 17 holder" width="48%">
</p>

On the opposite side, install `turret pitch bearing adapter.stl`, a 608 bearing, and `axis stabilizer bearing holder 608.stl` with 25 mm M3 screws.

![Pitch-axis bearing adapter](pictures/assembly-guide/rover-build-46-turret-pitch-bearing-adapter.png)

Build the first planetary stage the same way as the height-control gearbox. For the second stage:

1. Fit `planetary 1st to 2nd stage coulper sun gear output holder.stl` to the first-stage output.
2. Add `planetary 2nd stage output shafts.stl` and the planet gears.
3. Close the stage with the matching `planteray 2nd stage output` part.
4. Attach `planetary output shaft axis stabilzier.stl` to the bearing side.

![Dual-stage pitch gearbox](pictures/assembly-guide/rover-build-47-turret-pitch-gearbox.png)

Insert the long pitch shaft into the axis stabilizer. Drill and tap an M3 hole about 10 mm deep to lock it. Fit `nema17 planetary output shaft extender.stl` at the top and lock it with two M3 screws about 50 mm from the top.

<p align="center">
  <img src="pictures/assembly-guide/rover-build-48-turret-pitch-output-shaft.png" alt="Pitch output shaft" width="48%">
  <img src="pictures/assembly-guide/rover-build-49-turret-pitch-shaft-extender.png" alt="Pitch shaft extender" width="48%">
</p>

Bolt `hose attachment adapter.stl` to the pitch shaft assembly. Fit `hose pipe adapter.stl` inside it for a standard US garden hose.

<p align="center">
  <img src="pictures/assembly-guide/rover-build-50-hose-attachment-adapter.png" alt="Hose attachment adapter" width="48%">
  <img src="pictures/assembly-guide/rover-build-51-hose-pipe-adapter.png" alt="Garden hose pipe adapter" width="35%">
</p>

### 7. Build the wireless faucet controller

Press-fit or glue the MG996 servo into `casing bottom shell mg996 holder.stl`. Attach `hose faucet control adapter.stl` to the servo horn with tapped 5 mm M3 screws.

<p align="center">
  <img src="pictures/assembly-guide/rover-build-52-hose-controller-servo-mount.png" alt="MG996 faucet servo mount" width="48%">
  <img src="pictures/assembly-guide/rover-build-53-hose-controller-servo-horn-adapter.png" alt="Faucet adapter on servo horn" width="48%">
</p>

Convert the MG996 for continuous rotation while keeping its potentiometer as an external encoder:

- Disconnect the potentiometer from the servo controller and complete the normal centered-feedback continuous-rotation conversion.
- Connect the potentiometer outer terminals to ESP32 **3.3 V** and ground; connect the wiper to **GPIO35**.
- Connect the servo signal to **GPIO14**.
- Power the servo from a separate 5 V branch capable of its stall current. Join its ground to the ESP32 ground; do not pass servo current through the ESP32 regulator.

Install the ESP32-WROOM-32 inside `casing middle section.stl`, route the USB power cable through the opening, and mount the U.FL connector through `mount top lid casnig.stl`.

<p align="center">
  <img src="pictures/assembly-guide/rover-build-54-hose-controller-electronics-enclosure.png" alt="Hose controller electronics enclosure" width="48%">
  <img src="pictures/assembly-guide/rover-build-55-hose-controller-wiring-inside.png" alt="ESP32 and wiring inside faucet controller" width="35%">
</p>

Slide `casing mount rod.stl` through the enclosure into `mount output shaft holder.stl`. Install `external mount.stl` with two 608 bearings.

<p align="center">
  <img src="pictures/assembly-guide/rover-build-56-hose-controller-casing-mount-rod.png" alt="Faucet-controller casing mount rod" width="48%">
  <img src="pictures/assembly-guide/rover-build-57-hose-controller-external-mount-bearings.png" alt="External faucet-controller mount and bearings" width="48%">
</p>

Clamp the assembly beside the faucet and power it from a 5 V power bank or outdoor-rated supply.

![Wireless faucet controller installed](pictures/assembly-guide/rover-build-58-hose-controller-installed.png)

### 8. Mount and wire the rover electronics

#### ODESC and stepper-controller enclosure

Cut one 300 mm channel and mount it on the front-right side. Bolt the ODESC V4.2 enclosure and stepper-controller enclosure to it with two screws per case.

<p align="center">
  <img src="pictures/assembly-guide/rover-build-59-odesc-stepper-support-channel.png" alt="ODESC and stepper support channel" width="35%">
  <img src="pictures/assembly-guide/rover-build-60-odesc-stepper-enclosures.png" alt="ODESC and stepper enclosures" width="35%">
</p>

Route the three NEMA 17 cables through the stepper case. Route the C6374 phase and sensor wires directly into the ODESC case.

![ODESC and stepper-controller wiring](pictures/assembly-guide/rover-build-61-odesc-stepper-wiring.png)

#### Power distribution

Use the [block-level KiCad schematic](Schematics/Block-level%20schematic/) as the connector reference.

![Block-level electrical schematic](pictures/block%20level%20schematic%20picture%20kicad.png)

| Source | Destination |
| --- | --- |
| 36 V battery | ODESC V4.2 battery input |
| 36 V battery → 24 V converter | GD32 stepper board and three NEMA 17 motors |
| 36 V battery → 9 V converter | Both BTS7960 motor-power inputs |
| 9 V rail → 5 V XL4005 | ESP32-P4, STM32, BNO080, and BTS7960 logic |

All rails must share ground.

Solder two fused branches in parallel at the XT60 battery output: one for the ODESC and one for the buck converters/main electronics.

#### Drivetrain controller

Each BTS7960 drives the two motors on one side in parallel. Connect both enable pins to 5 V and use these STM32 signals:

| Side | RPWM | LPWM | R current sense | L current sense |
| --- | --- | --- | --- | --- |
| Left / M1 | PA8 | PA11 | PA4 | PA5 |
| Right / M2 | PB10 | PB11 | PA6 | PA7 |

Connect the four wheel potentiometer wipers to **PA0, PA1, PA2, and PA3**. For each input, use the schematic's 1 kΩ series resistor and 100 nF capacitor to ground. Power the potentiometers from the 3.3 V sensor rail, not 5 V. Label which wheel uses each ADC input.

#### Host UART wiring

Cross TX to RX and connect a common ground for every link.

| ESP32-P4 | Subsystem | Baud |
| --- | --- | ---: |
| GPIO21 TX → PA10 RX; GPIO22 RX ← PA9 TX | STM32 drivetrain | 115200 |
| GPIO2 TX → PA10 RX; GPIO1 RX ← PA9 TX | GD32 stepper board | 115200 |
| GPIO27 TX → GPIO2 RX; GPIO47 RX ← GPIO1 TX | ODESC | 115200 |
| GPIO5 TX → BNO RX; GPIO6 RX ← BNO TX | BNO080 | 3000000 |

Connect mower height, turret yaw, and turret pitch to the GD32 **X, Y, and Z** outputs respectively, then label the plugs. The firmware exposes these axes as X/Y/Z.

The hose controller is separate and communicates wirelessly over ESP-NOW. Its firmware uses Wi-Fi channel 6 and the controller MAC configured in [`firmware/esp32-hose/source/main/main.c`](firmware/esp32-hose/source/main/main.c).

#### Pack the main enclosure

The enclosure layout is:

- Red: 36→9 V and 36→24 V converters
- Purple: two BTS7960 drivers
- Yellow: STM32 drivetrain controller
- Green: ESP32-P4
- Black: BNO080, mounted above the ESP32-P4
- White: Waveshare speaker
- Gray: 9→5 V XL4005, insulated against shorts

![Main-electronics component layout](pictures/assembly-guide/rover-build-64-main-electronics-layout.png)

Use `wire management cover.stl` to keep motor and encoder wires away from the mower lift ropes. Keep UART and sensor wires away from motor and battery wiring where possible.

<p align="center">
  <img src="pictures/assembly-guide/rover-build-65-main-electronics-wiring-photo.png" alt="Main-electronics wiring photo" width="40%">
  <img src="pictures/assembly-guide/rover-build-62-rover-electronics-open.png" alt="Rover electronics before lids" width="40%">
</p>

Fit every lid only after the electrical checks pass.

![Rover electronics with lids installed](pictures/assembly-guide/rover-build-63-rover-electronics-lids.png)

### 9. First power on

1. Connect to the rover Wi-Fi.
2. From there, you can control each part of the rover.
3. There is a tank drive mode, battery history mode, and sensor mode.
4. You can also record microphone inputs and respond using audio outputs.

### 10. Bonus handheld controller

1. I have also custom-made a handheld controller that connects to the rover, from which you can control the rover, mower, and turret. It is much better than the phone screen for controlling the rover. The controller is part of my Claw Drone project; view how to build it here: [Claw Drone](https://github.com/houseofthe051010/Claw-Drone).

## Flashing the firmware


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
