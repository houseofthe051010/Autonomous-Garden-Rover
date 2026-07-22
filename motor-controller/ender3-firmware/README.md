# Ender-3 v4.2.2 GD32 Stepper Firmware

## Target hardware

- Creality Ender-3 mainboard revision 4.2.2
- GD32F303RET6 MCU in LQFP64
- Four onboard standalone STEP/DIR driver channels: X, Y, Z, and E
- Primary UART through GD32 USART1 on PA9/PA10

The build uses Marlin's `STM32F103RE_creality` compatibility environment. The
GD32F303RET6 is capable of 120 MHz, but this compatibility build intentionally
runs at 72 MHz.

## Flashable image

File: [`ALLDRV1.BIN`](ALLDRV1.BIN)

```text
Size:    117104 bytes
SHA-256: b51e242a77e57c88dd8caeb84f8ecd6ea689b5c5daf4c32a2e767d8b47c27cab
```

Build resource report:

```text
Flash: 117104 / 524288 bytes (22.34%)
Static RAM: 6052 / 65536 bytes (9.23%)
CPU compatibility clock: 72 MHz
```

## Flashing

1. Use an 8 GB FAT32 microSD card or another card known to work with the Creality
   bootloader.
2. Place only `ALLDRV1.BIN` with a `.BIN` extension in the card root. Prior images
   may be retained with `.BAK` extensions.
3. Power the Ender mainboard off.
4. Insert the card and power the board on.
5. Allow the bootloader to finish before removing power. It may rename the image
   or otherwise mark it as consumed after a successful flash.
6. Power off before removing the card.

The filename differs from earlier firmware names because the Creality bootloader
may refuse to flash the same filename twice.

## Functional changes

The patch converts stock Marlin 2.1.2.5 into a stepper-controller build:

- Creality v4.2.2 board and CR-10 stock display configuration.
- USART1 at 115200 baud for ESP32 communication.
- Dummy 25 C hotend and bed temperature sensors.
- Heater thermal protection disabled for disconnected heater hardware.
- Cold-extrusion and maximum-extrusion-length restrictions disabled.
- XYZ software endstops disabled.
- XYZ logical ranges expanded to -1,000,000 through +10,000,000 units.
- Maximum X/Y/Z/E feedrate defaults raised to 1000 configured units/s.
- EEPROM and SD runtime features disabled to avoid unsupported/peripheral errors.
- Watchdog disabled for this compatibility build.
- `HB` heartbeat and `HB_ACK_OK` round-trip confirmation.
- `SW Xn Yn Zn` debounced switch telemetry.
- `M410` emergency parser enabled for immediate quick-stop handling.
- Tagged movement completion through `M118 DRV_DONE ...`.
- Corrected manual motion feedrate scaling and LCD speed controls.

## Reproducible source

The binary is based on upstream Marlin tag `2.1.2.5`, commit:

```text
e5a167aaa0a671ea6d9b9d98dc0518c88d5caed7
```

Included source assets:

- [`marlin-2.1.2.5-stepper-controller.patch`](marlin-2.1.2.5-stepper-controller.patch)
- [`marlin-assets/_Bootscreen.h`](marlin-assets/_Bootscreen.h)
- [`marlin-assets/_Statusscreen.h`](marlin-assets/_Statusscreen.h)
- [Marlin GPLv3 license](../licenses/Marlin-GPL-3.0.txt)

Rebuild procedure:

```bash
git clone --branch 2.1.2.5 https://github.com/MarlinFirmware/Marlin.git
cd Marlin
git apply /path/to/marlin-2.1.2.5-stepper-controller.patch
cp /path/to/_Bootscreen.h Marlin/_Bootscreen.h
cp /path/to/_Statusscreen.h Marlin/_Statusscreen.h
platformio run -e STM32F103RE_creality
```

The resulting binary is written below `.pio/build/STM32F103RE_creality/`. Build
scripts may add a timestamp to the filename.

This procedure was verified from a clean `2.1.2.5` archive. It produced the same
117,104-byte image and resource report as the included firmware. Marlin embeds
its compilation date and time, so a later build has a different SHA-256 checksum.
The verification build differed from `ALLDRV1.BIN` at only six timestamp bytes.

## Expected build warnings

Marlin warns that watchdog, thermal protections, and real thermistors are not
enabled. Those warnings are expected only because this is a dedicated rover
stepper-controller build with no heater hardware.

**Never use this image to control a printer hotend or heated bed.** It deliberately
removes the protections required for heater operation.

Marlin may also warn that Creality v4.2.2 boards shipped with different driver
variants. Confirm the physical board code or driver marking.

## Display

Display support remains enabled because it occupies little of the available
memory and this configuration is known to boot. The rover can operate with no
display connected; all normal control comes from the ESP32 UART link.
