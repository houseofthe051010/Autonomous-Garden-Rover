# STM32 Dual-BTS7960 UART Protocol

## Transport

- Electrical level: 3.3 V UART
- Format: 115200 baud, 8 data bits, no parity, 1 stop bit
- Framing: newline-delimited ASCII; commands end in `\n`
- STM32 port: USART1, PA9 TX and PA10 RX
- Raspberry Pi 5 port: UART1, GPIO0 TX and GPIO1 RX

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

## Error responses

Errors begin with `ERR`, including:

```text
ERR BAD_COMMAND
ERR BAD_MOTOR
ERR BAD_DIRECTION
ERR BAD_DUTY
ERR LINE_TOO_LONG
```

The Raspberry Pi client raises these as Python `RuntimeError` exceptions.
