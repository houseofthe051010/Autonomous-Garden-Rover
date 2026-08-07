# ESP32-P4 to STM32 UART diagnostic

This native ESP-IDF firmware is for ESP32-P4 revision 3.x boards that cannot
safely run the currently published generic MicroPython image.

## Wiring

Connect the ESP32-P4 GPIO21 and GPIO22 to STM32 PA9 and PA10 in either
orientation, plus a common ground. The firmware first listens on each ESP32
pin without enabling its TX output. After it finds the STM32 heartbeat, it uses
the other pin as TX and verifies the link with `PING` / `OK PONG`.

The diagnostic sends `MSTOP ALL` at connection time and never sends a motor
movement command.

## Build and flash

Use ESP-IDF 5.5.3 or newer for an ESP32-P4 revision 3.x chip:

```bash
idf.py set-target esp32p4
idf.py build
idf.py -p /dev/ttyACM0 flash
```

Do not flash a firmware image whose `esptool image-info` output has a maximum
chip revision below the revision reported by `esptool chip-id`.

## Terminal monitor

The board console is 115200 baud:

```bash
picocom -b 115200 /dev/ttyACM0
```

Exit picocom with `Ctrl-A`, then `Ctrl-X`.

Expected output includes:

```text
STM32 wiring detected: ESP32-P4 TX GPIO..., RX GPIO...
STM32: HB ...
UART ROUND TRIP VERIFIED: OK PONG
```
