# STM32 Dual-BTS7960 UART Protocol

## Transport

- Electrical level: 3.3 V UART
- Format: 115200 baud, 8 data bits, no parity, 1 stop bit
- Framing: newline-delimited ASCII; commands end in `\n`
- STM32 port: USART1, PA9 TX and PA10 RX
- Raspberry Pi 5 port: UART1, GPIO0 TX and GPIO1 RX
- ESP32-P4 host port: UART1, GPIO21 TX and GPIO22 RX; orientation is also
  checked automatically by the [ESP32-P4 firmware](../../firmware/esp32-p4/README.md)

UART TX and RX must be crossed. Both devices must share ground.

## Commands

| Host command | STM32 response | Meaning |
| --- | --- | --- |
| `PING` | `OK PONG` | Link test and watchdog feed |
| `CAPS` | `CAPS BTS7960 ...` | Firmware capabilities |
| `KEEPALIVE` | `OK KEEPALIVE` | Feed watchdog without telemetry |
| `MOTOR n A duty` | `OK MOTOR n A duty` | Drive motor `n` through RPWM |
| `MOTOR n B duty` | `OK MOTOR n B duty` | Drive motor `n` through LPWM |
| `MOTOR n S 0` | `OK MOTOR n S 0` | Stop motor `n` |
| `MSTOP n` | `OK MSTOP n` | Stop one motor |
| `MSTOP ALL` | `OK MSTOP ALL` | Stop both motors |
| `MSTATUS` | `MSTAT ...` | Return motor and current telemetry |
| `ENCREAD` | `ENC ...` | One PA0-PA3 ADC sample set; streaming stays unchanged |
| `ENCON` | `OK ENCON 50` | Enable PA0-PA3 reports at the default 50 Hz |
| `ENCON hz` | `OK ENCON hz` | Enable reports at 10, 20, 25, 50, or 100 Hz |
| `ENCOFF` | `OK ENCOFF` | Disable periodic PA0-PA3 reports |
| `RESET` | `OK RESET` | Stop both motors and reset motor state |

`n` is `1` or `2`. PWM duty is an integer from `0` through `4095`; the STM32
generates 20 kHz hardware PWM.

## Asynchronous messages

The STM32 transmits these without waiting for a command:

```text
READY STM32F103C6 BTS7960 X2
HB 12345
FAULT HOST_TIMEOUT MOTORS_STOPPED
```

`HB` is sent once per second and its number is the STM32 uptime in milliseconds.
Heartbeat reception proves STM32-to-host communication. Commands such as
`PING` or `MSTATUS` prove the complete two-way link.

The Blue Pill's active-low PC13 onboard LED blinks while valid commands are
being received and turns off 2.5 seconds after the last valid command. PB4
continues toggling once per second as an independent run diagnostic.

The host must issue a valid command more frequently than every 1.5 seconds. If
not, the STM32 sets both PWM outputs to zero and emits the `FAULT` line.

## Telemetry format

`MSTATUS` returns exactly one line:

```text
MSTAT 1 A 1024 120 8 96 6 2 S 0 14 11 11 8 WD 0
```

Fields are positional:

| Index | Field |
| ---: | --- |
| 0 | `MSTAT` |
| 1 | Motor number `1` |
| 2 | Motor 1 direction: `A`, `B`, or `S` |
| 3 | Motor 1 duty, 0–4095 |
| 4–5 | Motor 1 R_IS and L_IS raw 12-bit ADC readings |
| 6–7 | Motor 1 R_IS and L_IS millivolts |
| 8 | Motor number `2` |
| 9 | Motor 2 direction: `A`, `B`, or `S` |
| 10 | Motor 2 duty, 0–4095 |
| 11–12 | Motor 2 R_IS and L_IS raw 12-bit ADC readings |
| 13–14 | Motor 2 R_IS and L_IS millivolts |
| 15–16 | `WD 0` normally or `WD 1` after watchdog stop |

ADC millivolts are current-sense output voltage, not motor current in amperes.
Conversion to amperes requires calibration for the installed modules and their
external current-sense resistor networks.

## Analog encoder telemetry

Encoder streaming is disabled at boot and only starts after `ENCON`. Each
report is one newline-delimited record:

```text
ENC 37 12540 2048 1001 4095 0
```

The fields are `ENC`, sequence number, STM32 uptime in milliseconds, then raw
12-bit ADC readings for PA0, PA1, PA2, and PA3. The sequence number lets the Pi
detect a missed record. `ENCREAD` returns one record even when streaming is off.

The recommended rate is 50 Hz. It is fast enough for low-speed rover steering
and potentiometer feedback while using roughly 1.5-2.3 kB/s, no more than about
20 percent of a 115200-baud 8N1 UART. Each channel uses a four-sample average plus one
discarded channel-change conversion. Use 10-25 Hz for slow mechanisms and
100 Hz only after measuring whether it improves control. The Pi performs
calibration, angle conversion, filtering, and motion estimation.

Unconnected analog pins float, so changing values before the encoders are
wired are expected and do not indicate a firmware fault.

Periodic encoder telemetry is lower priority than commands. The STM32 defers a
scheduled report while receive bytes are waiting or its transmit buffer is
crowded; timestamps therefore define the real sample timing. The Pi client
tracks `dropped` sequence numbers and `age_ms` so overload or wiring problems
are visible instead of silently using stale data.

## Error responses

Errors begin with `ERR`, including:

```text
ERR BAD_COMMAND
ERR BAD_MOTOR
ERR BAD_DIRECTION
ERR BAD_DUTY
ERR BAD_ENCODER_RATE
ERR LINE_TOO_LONG
```

The Raspberry Pi client raises these as Python `RuntimeError` exceptions.
