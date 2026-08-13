# Firmware

Each directory is an independently built and flashed target. Do not assume a
binary intended for one controller is compatible with another board.

| Target | Role | Build / flashing guide |
| --- | --- | --- |
| [`esp32-p4/`](esp32-p4/README.md) | Primary rover host, web UI, Wi-Fi, storage, audio, and controller links | ESP-IDF |
| [`stm32-drive/`](stm32-drive/README.md) | Dual BTS7960 drivetrain controller | PlatformIO |
| [`gd32-stepper/`](gd32-stepper/README.md) | Ender-3/GD32 three-axis stepper controller | microSD image / Marlin rebuild |
| [`esp32-hose/`](esp32-hose/README.md) | ESP-NOW hose-valve controller | ESP-IDF / PlatformIO configuration |
| [`odesc-v42/`](odesc-v42/README.md) | Experimental ODESC V4.2 BLDC controller work | ODrive-based build |

## Safety

Firmware in this repository controls physical motion and power. Begin with the
drive wheels raised or mechanisms disconnected, use low command values, and
keep an independent power disconnect within reach. Target-specific validation
status and limitations are documented in each target's README.

## Credentials

`esp32-p4/main/rover_control_key.h` is intentionally ignored. Create it from
`rover_control_key.example.h` and use a new random 32-byte value for every
deployment. Never add Wi-Fi credentials, controller keys, device backups, or
serial captures containing credentials to a commit.
