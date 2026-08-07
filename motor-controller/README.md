# Ender-3 Rover Motor Controller

## Purpose

This subsystem repurposes a Creality Ender-3 v4.2.2 mainboard as a four-output
stepper controller for the autonomous garden rover. The board has a
GD32F303RET6 MCU and four onboard STEP/DIR drivers connected to the original
X, Y, Z, and E motor sockets.

The ESP32-P4 is the current rover control computer. It sends commands through the
Ender board's primary USART and receives heartbeat, motion completion, and X/Y/Z
switch data. Earlier Raspberry Pi 5 and ESP32 MicroPython clients remain as
reference implementations.

## Electrical warning

All UART signals are 3.3 V logic. Connect the host and Ender board grounds.
Never connect 5 V to a UART data line.

The CH340 TXD pin is an output. A Pi or ESP32 TX output must not fight it. Isolate
the CH340 TXD output from the GD32 RX net before driving that net from another
host. Do not connect a USB host to the CH340 while using the direct UART taps.
A series resistor may limit fault current, but it does not replace proper
isolation because the CH340 can hold the signal at the wrong logic level.

## UART wiring

UART settings are **115200 baud, 8 data bits, no parity, 1 stop bit (8N1)**.

For a CH340G-style SOP-16 package with the notch facing upward, numbering starts
at the upper-left corner and proceeds counterclockwise:

| Signal | Ender board net | CH340 package | Host |
|---|---|---|---|
| Ground | GND | Pin 1, GND | GND |
| Ender receives | GD32 PA10 / USART1 RX | Pin 2, CH340 TXD net | Host TX |
| Ender transmits | GD32 PA9 / USART1 TX | Pin 3, CH340 RXD net | Host RX |

Verify the CH340 part marking, package orientation, and continuity before
soldering. Similar USB-UART packages may use a different pinout.

The Raspberry Pi 5 UART allocation is `/dev/ttyAMA3`:

| Pi signal | Physical pin | Connect to |
|---|---:|---|
| GPIO8 / TX | 24 | GD32 PA10 / RX, on the isolated GD32 side |
| GPIO9 / RX | 21 | GD32 PA9 / TX |
| GND | 20 or another GND | Ender GND |

See the [Raspberry Pi client instructions](raspberry-pi/README.md) before wiring.

### ESP32 alternative

The supplied MicroPython code accepts either data-wire orientation:

| ESP32 GPIO | Function |
|---|---|
| GPIO16 | UART data wire A; auto-detected as TX or RX |
| GPIO17 | UART data wire B; auto-detected as TX or RX |
| GPIO18 | Detection-only dummy TX; **leave physically unconnected** |
| GPIO2 | Link-status LED, if present on the ESP32 board |

At startup the ESP32 listens for `HB` messages on GPIO16, then GPIO17. It assigns
the heartbeat wire as RX and the other wire as TX. GPIO18 prevents detection from
driving either connected data wire before orientation is known.

## Ender board I/O map

These are the Creality v4.2.x Marlin pin assignments used by this build:

| Output | STEP | DIR | ENABLE |
|---|---|---|---|
| X | PC2 | PB9 | PC3 |
| Y | PB8 | PB7 | PC3 |
| Z | PB6 | PB5 | PC3 |
| E | PB4 | PB3 | PC3 |

All four drivers share the physical `PC3` enable line. They can be stepped and
given directions independently, but they can only be enabled or disabled as a
group.

| Rover switch input | GD32 pin | UART report field |
|---|---|---|
| X endstop connector | PA5 | `X0` or `X1` |
| Y endstop connector | PA6 | `Y0` or `Y1` |
| Z endstop connector | PA7 | `Z0` or `Z1` |

Inputs use pull-ups and the stock normally-closed Ender switch polarity. Firmware
samples them every 5 ms, requires three equal samples (15 ms debounce), reports
changes immediately, and refreshes the complete state every second.

## Driver hardware limitations

The v4.2.2 board was manufactured with several standalone driver variants. This
build uses A4988-compatible STEP/DIR pulse timing. Confirm the board's driver code
or chip marking before assuming the exact driver model.

- Direction and step frequency are under MCU control.
- All four enables are physically tied together.
- Driver current is controlled by the four onboard VREF potentiometers, not UART.
- Microstep selection and decay-mode wiring are fixed by PCB straps.
- A4988 reset, sleep, thermal warning, and overcurrent state are not exposed to
  controllable/readable GD32 pins on this board.
- The drivers provide no encoder or missed-step feedback.

## MCU and firmware resources

| Resource | GD32F303RET6 hardware | Current firmware |
|---|---:|---:|
| CPU clock | 120 MHz maximum | 72 MHz compatibility clock |
| Flash | 512 KiB | 120,680 bytes (23.0%) |
| SRAM | 64 KiB | 6,192 bytes static (9.4%) |

The direct-motion build uses a 50 kHz timer ISR for independent simultaneous
X/Y/Z/E pulse generation. Continuous targets are limited to 600 RPM, ramp at
1,200 RPM/s, and stop after a 250 ms command timeout. Counted moves start all
included axes together and report exact completion through `COUNT_DONE`.

CPU load is dynamic: it is low while idle and rises with enabled axes and step
rate. At the 600 RPM limit and 3200 microsteps/revolution, each moving driver
requires 32,000 step pulses/s.

Hardware specifications are available on the
[GigaDevice GD32F303RET6 product page](https://www.gigadevice.com.cn/product/mcu/mcus-product-selector/gd32f303ret6)
and in the
[GD32F303xx datasheet](https://www.gd32mcu.com/data/documents/datasheet/GD32F303xx_Datasheet_Rev2.2.pdf).

The LCD code remains compiled because resource use is low and the known display
configuration boots reliably. The rover does not require the display to be
connected.

## Related documentation

- [ESP32-P4 continuous and counted direct-motion handoff](esp32-p4-gd32-direct-motion/README.md)
- [UART protocol and API](UART_PROTOCOL.md)
- [Raspberry Pi 5 Thonny client](raspberry-pi/README.md)
- [Validated Pi 5 and GD32 working model](raspberry-pi/VALIDATED_WORKING_MODEL.md)
- [ESP32 MicroPython code](esp32-micropython/README.md)
- [Ender firmware and flashing](ender3-firmware/README.md)
