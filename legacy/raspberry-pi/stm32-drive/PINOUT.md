# Raspberry Pi 5 Power and UART Pinout

This document separates wiring that is currently connected from proposed wiring.
GPIO numbers are BCM numbers; physical pin numbers identify positions on the
40-pin header. Every UART is 3.3 V logic and its TX wire crosses to the other
device's RX.

## Currently connected

The exact physical 5 V and ground header positions used by the existing power
wires have not been recorded. The Pi's two 5 V pins are physical pins 2 and 4;
all Pi ground pins are electrically common.

| Source | Destination | Purpose | State |
| --- | --- | --- | --- |
| XL4005 OUT+ | Pi 5 V, physical 2 or 4 | Pi power | Connected; exact pin unrecorded |
| XL4005 OUT- | Any Pi GND | Pi power return | Connected; exact pin unrecorded |
| Pi 5 V, physical 2 or 4 | STM32 Blue Pill 5V | STM32 power | Connected; exact Pi pin unrecorded |
| Pi GND | STM32 GND | Common logic ground | Connected; exact Pi pin unrecorded |
| Pi GPIO0, physical 27, UART1 TX | STM32 PA10, USART1 RX | Motor command data | Connected |
| Pi GPIO1, physical 28, UART1 RX | STM32 PA9, USART1 TX | Heartbeat/current telemetry | Connected |

UART1 appears as `/dev/ttyAMA1` and runs at 115200 baud for the STM32 dual
BTS7960 controller. GPIO0/GPIO1 cannot also be used by a HAT identification
EEPROM.

## Proposed GD32 UART

This allocation assumes the GD32 firmware uses USART1 on PA9/PA10, as does the
Ender-3 controller firmware in this repository.

| Raspberry Pi 5 | GD32 | Purpose |
| --- | --- | --- |
| GPIO8, physical 24, UART3 TX | PA10, USART1 RX | Pi commands to GD32 |
| GPIO9, physical 21, UART3 RX | PA9, USART1 TX | GD32 replies to Pi |
| GND, physical 20 | GND | Common logic ground |

UART3 appears as `/dev/ttyAMA3`. Use 115200 baud with the current Ender/GD32
protocol. GPIO8/GPIO9 cannot simultaneously be used for their normal SPI0
functions. Confirm the GD32 board model and firmware pin assignment before
connecting it. Do not power an unknown GD32 board from the Pi 5 V rail.

## Validated BNO080 UART-SHTP connection

| Raspberry Pi 5 | BNO080 label/function | Purpose |
| --- | --- | --- |
| GPIO12, physical 32, UART4 TX | RX / SCL-RX | Pi commands to BNO080 |
| GPIO13, physical 33, UART4 RX | TX / SDA-TX | Sensor reports to Pi |
| GPIO22, physical 15, input | H_INTN | Active-low data interrupt/timestamp |
| GPIO23, physical 16, output | NRST | Sensor reset control |
| 3V3, physical 17 | VDD/VIN only if the board accepts 3.3 V | Sensor power |
| GND, physical 9 | GND | Common logic ground |

UART4 appears as `/dev/ttyAMA4`. UART-SHTP is fixed at 3,000,000 baud, 8N1.
Select UART-SHTP mode with PS1 high and PS0 low during reset. Some breakout
boards provide solder jumpers or switches for these mode pins.

Do not connect BNO080 power until the exact breakout board has been identified.
A bare BNO080 is a 3.3 V-class device; a breakout marked `VIN` may include a
regulator, but that must be verified from that board's schematic. UART logic
must remain 3.3 V regardless.

## Validated ODESC UART connection

| Raspberry Pi 5 | ODESC | Purpose |
| --- | --- | --- |
| GPIO4, physical 7, UART2 TX | GPIO2 / UART RX | ODrive ASCII commands |
| GPIO5, physical 29, UART2 RX | GPIO1 / UART TX | Replies and status |
| Any available GND | GND | Common logic ground |

UART2 appears as `/dev/ttyAMA2` and runs at 115200 baud. The bidirectional ODESC
property-read test was hardware-validated on 2026-07-22. See the
[`ODESC working model`](../odesc/VALIDATED_WORKING_MODEL.md).
Pi 5 UART0 on GPIO14/GPIO15 remains unused as one more possible UART.

## Device Tree configuration

Add only the interfaces that are actually wired under `[all]` in
`/boot/firmware/config.txt`:

```ini
# Existing STM32 motor controller
dtoverlay=uart1-pi5

# Validated ODESC motor controller
dtoverlay=uart2-pi5

# Add when the GD32 is connected
dtoverlay=uart3-pi5

# Current BNO080 UART-SHTP connection
dtoverlay=uart4-pi5
```

Reboot after changing overlays, then verify the expected ports:

```sh
ls -l /dev/ttyAMA1 /dev/ttyAMA2 /dev/ttyAMA3 /dev/ttyAMA4
```

The official Pi 5 UART assignments are documented in Raspberry Pi's
[UART configuration guide](https://www.raspberrypi.com/documentation/computers/configuration.html#configure-uarts)
and [Device Tree overlay reference](https://github.com/raspberrypi/firmware/blob/master/boot/overlays/README).

## Power notes

- Set and measure the XL4005 output before attaching the Pi. Raspberry Pi
  specifies a 5 V supply and recommends 5 A for a Pi 5 with its full peripheral
  power budget.
- Confirm voltage at the Pi under CPU and USB load; buck-converter ratings
  depend on input voltage, cooling, wiring, and module quality.
- Use short, adequately sized power wires and an appropriately rated fuse near
  the buck output.
- Do not connect an independent USB-C supply while the XL4005 is driving the
  same Pi 5 V rail; two supplies can drive each other.
- Do not power the STM32 simultaneously from its USB connector or another 5 V
  source while the Pi 5 V rail is connected to the Blue Pill 5V pin.
- The Pi-to-STM32 5 V wire is a branch of the XL4005 supply, not a separately
  regulated Pi output.
- Keep motor-current return paths out of the Pi/IMU ground wiring. All logic
  grounds must connect, but high-current motor returns should go directly to
  the power distribution point.

Because a header-fed XL4005 cannot advertise its capacity over USB-PD, Pi 5
supports declaring a verified bench supply in its EEPROM bootloader
configuration. Only after proving that the converter, cooling, fuse, and wiring
can continuously supply the selected current, edit the EEPROM configuration:

```sh
sudo -E rpi-eeprom-config --edit
```

For a genuinely verified 5 A supply, add:

```ini
PSU_MAX_CURRENT=5000
```

Then save and reboot. Do not set this merely because an XL4005 listing claims
5 A; the setting increases the power budget the Pi assumes is available. The
official documentation specifically provides `PSU_MAX_CURRENT` for bench
supplies connected through the GPIO header.

Raspberry Pi's current guidance recommends 5 V/5 A for Pi 5; a 5 V/3 A supply
boots the Pi but reduces the USB peripheral budget. See the official
[power-supply documentation](https://www.raspberrypi.com/documentation/computers/getting-started.html#power-supply)
and [bootloader power properties](https://www.raspberrypi.com/documentation/computers/configuration.html#power-supply-properties-chosenpower).
