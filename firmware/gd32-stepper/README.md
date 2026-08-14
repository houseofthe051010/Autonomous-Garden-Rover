# ESP32-P4 to GD32 Direct Stepper Model

## Scope and validation state

This folder is the handoff package for controlling the rover's repurposed
Ender-3 v4.2.2 mainboard from an ESP32-P4. The board uses a GD32F303RET6 and
four standalone STEP/DIR driver outputs named X, Y, Z, and E.

The physical 115200-baud UART connection, heartbeat round trip, and ordinary
finite Z motion were validated with the preceding host setup. The firmware in
this folder compiles successfully for the same board and adds timer-driven
simultaneous continuous/count-limited motion. The new direct-motion commands
must still be validated on the physical motors after flashing; do not treat the
build result alone as a completed hardware test.

## Handoff contents

- [`UART_PROTOCOL.md`](UART_PROTOCOL.md): authoritative ESP32-P4/GD32 command,
  response, timing, state-machine, and safety contract.
- [`firmware/GD3P4V1.BIN`](firmware/GD3P4V1.BIN): flashable Creality v4.2.2
  GD32F303RET6 image.
- [`source/marlin-2.1.2.5-gd32-direct-motion.patch`](source/marlin-2.1.2.5-gd32-direct-motion.patch):
  complete patch against upstream Marlin tag `2.1.2.5`.
- [`source/_Bootscreen.h`](source/_Bootscreen.h) and
  [`source/_Statusscreen.h`](source/_Statusscreen.h): required Marlin display
  assets.
- [`SHA256SUMS`](SHA256SUMS): artifact integrity checks.

Build resources:

| Resource | Used | Available |
|---|---:|---:|
| Flash | 120,680 bytes (23.0%) | 524,288 bytes |
| Static RAM | 6,192 bytes (9.4%) | 65,536 bytes |
| Compatibility CPU clock | 72 MHz | GD32F303RET6 supports up to 120 MHz |

## Flashing

The SD-card image is named `GD3P4V1.BIN` so it differs from prior firmware
filenames remembered by the Creality bootloader.

1. Power off the Ender board.
2. Insert the FAT32 microSD card containing only `GD3P4V1.BIN` as a `.BIN` file.
3. Power on and leave the board undisturbed while the bootloader processes it.
4. Power off before removing the card.
5. Reconnect the UART host and confirm `HB` / `HB_ACK_OK` before enabling motion.

The firmware intentionally disables heater sensing/protection because no heater
hardware is attached. Never use this build to operate a hotend or heated bed.

This firmware patch is derived from Marlin and remains subject to the
[GNU General Public License v3](../../Marlin-GPL-3.0.txt). The root MIT
license does not replace Marlin's license for these derived files.

## Initial hardware test

Raise or disconnect mechanisms that could hit a hard stop. Begin with one motor
at 5 RPM and keep sending the command at 60 Hz:

```text
M970 I1 X5 Y0 Z0
```

Center the joystick by continuing to send zero targets:

```text
M970 I2 X0 Y0 Z0
```

Leave direct mode with:

```text
M975
```

After each individual axis and direction is verified, test simultaneous low-speed
motion:

```text
M970 I3 X5 Y-7.5 Z3
```

Do not mark the direct-motion path hardware-validated until direction, timeout,
simultaneous stepping, `M975`, `M410`, and counted completion have all been
observed on the actual rover wiring.

## Rebuild

```bash
git clone --branch 2.1.2.5 https://github.com/MarlinFirmware/Marlin.git
cd Marlin
git apply /path/to/marlin-2.1.2.5-gd32-direct-motion.patch
cp /path/to/_Bootscreen.h Marlin/_Bootscreen.h
cp /path/to/_Statusscreen.h Marlin/_Statusscreen.h
platformio run -e STM32F103RE_creality
```

The patch is based on upstream commit
`e5a167aaa0a671ea6d9b9d98dc0518c88d5caed7`.
