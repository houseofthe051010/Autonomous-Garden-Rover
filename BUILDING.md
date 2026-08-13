# Build it yourself

This guide gets a fresh checkout to buildable firmware. It does **not** make a
complete rover: assembling and validating the electrical system is a separate,
hardware-specific task. Read the target README and the relevant documents under
[`docs/`](docs/README.md) before connecting power or flashing a board.

## Safety first

This project controls motors and a hose-valve actuator. Build and test one
controller at a time with drive wheels raised or mechanisms disconnected, use
low command values, and keep an independent power disconnect within reach.
Never power the MG996 hose servo from an ESP32 board. Do not flash the
experimental ODESC images on a live rover without following its supervised
bench-validation procedure.

## 1. Get the source

```sh
git clone https://github.com/houseofthe051010/Autonomous-Garden-Rover.git
cd Autonomous-Garden-Rover
```

The repository includes known firmware binaries and diagnostic artifacts for
reference. You only need the source tree to build a target.

## 2. Install the toolchain for the target you are building

| Target | Required toolchain | Build guide |
| --- | --- | --- |
| ESP32-P4 primary host | ESP-IDF 6.0 or newer | [`firmware/esp32-p4/README.md`](firmware/esp32-p4/README.md) |
| STM32F103 drivetrain | PlatformIO with ST-Link support | [`firmware/stm32-drive/README.md`](firmware/stm32-drive/README.md) |
| GD32 Ender-3 stepper board | Marlin 2.1.2.5 and PlatformIO | [`firmware/gd32-stepper/README.md`](firmware/gd32-stepper/README.md) |
| ESP32 hose controller | PlatformIO | [`firmware/esp32-hose/README.md`](firmware/esp32-hose/README.md) |
| ODESC V4.2 (experimental) | Ubuntu, Tup, and `gcc-arm-none-eabi` | [`firmware/odesc-v42/README.md`](firmware/odesc-v42/README.md) |

Install ESP-IDF using Espressif's official setup instructions, then open an
ESP-IDF-enabled terminal before running `idf.py`. Install PlatformIO through
its CLI or IDE integration and ensure the `pio` command is available in your
shell.

## 3. Configure secrets locally

The ESP32-P4 needs a controller key that is unique to your own rover and
handheld controller. Create the ignored local file from the example:

```sh
cd firmware/esp32-p4/main
cp rover_control_key.example.h rover_control_key.h
```

Replace all 32 `0x00` values with 32 cryptographically random bytes and place
the identical key in the matching handheld-controller project. Do not commit
this file, Wi-Fi credentials, device backups, or serial captures containing
credentials. The repository ignores `rover_control_key.h` by design.

## 4. Build each target

### ESP32-P4 primary host

```sh
cd firmware/esp32-p4
idf.py set-target esp32p4
idf.py build
```

To flash a connected board, replace the serial port with the one assigned by
your operating system:

```sh
idf.py -p /dev/ttyACM0 flash monitor
```

The P4 uses Component Manager to fetch its declared dependencies during the
build. See its README for required wiring, partition layout, OTA behavior, and
network-security notes.

### STM32F103 drivetrain controller

```sh
cd firmware/stm32-drive
pio run
```

With an ST-Link connected and the board safely isolated from motor power:

```sh
pio run --target upload
```

Check the [STM32 UART protocol](docs/protocols/stm32-drive-uart.md) before
connecting it to the host controller.

### GD32 Ender-3 stepper controller

The checked-in source is a patch against Marlin `2.1.2.5`, not a complete
Marlin checkout. Rebuild it as follows:

```sh
git clone --branch 2.1.2.5 https://github.com/MarlinFirmware/Marlin.git
cd Marlin
git apply /path/to/Autonomous-Garden-Rover/firmware/gd32-stepper/source/marlin-2.1.2.5-gd32-direct-motion.patch
cp /path/to/Autonomous-Garden-Rover/firmware/gd32-stepper/source/_Bootscreen.h .
cp /path/to/Autonomous-Garden-Rover/firmware/gd32-stepper/source/_Statusscreen.h .
platformio run -e STM32F103RE_creality
```

Flash the resulting image from a FAT32 microSD card only after reviewing the
[GD32 guide](firmware/gd32-stepper/README.md). This derived target is licensed
under GPL-3.0-only, unlike the repository's original MIT-licensed material.

### ESP32 hose controller

```sh
cd firmware/esp32-hose
pio run
```

To upload, use your board's port:

```sh
pio run -t upload --upload-port /dev/ttyUSB0
```

Read the hose-controller README before wiring the actuator; it requires a
separate 5-6 V supply sized for servo stall current and a common ground.

### ODESC V4.2 (experimental)

Build this target only on a bench after reading the full target README. Its
current source and board-specific safety limitations are documented there.

```sh
cd firmware/odesc-v42/source/Firmware
tup init
tup
```

The output is created under `build/`. A successful build is not proof that the
firmware is safe to flash or operate at pack voltage.

## 5. Verify before motion

1. Confirm the board revision, target, pinout, and firmware revision match.
2. Verify common ground and voltage levels before joining UART connections.
3. Start with mechanisms unloaded or wheels raised.
4. Confirm watchdog and stop behavior before nonzero motor commands.
5. Record a sanitized test result in the target documentation or diagnostics
   archive.

## Licensing

Original rover code and documentation are MIT licensed. Bundled ODrive- and
Marlin-derived material retains its own terms; read
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) before redistributing a build.
