# ESP32-P4 MicroPython STM32 UART Test

[`main.py`](main.py) is a standalone Thonny client for checking the existing
STM32F103 dual-BTS7960 firmware from an ESP32-P4.

## Wiring

The script tests both possible orientations automatically:

| Preferred connection | Direction |
| --- | --- |
| ESP32-P4 GPIO22 | TX to STM32 PA10 / USART1 RX |
| ESP32-P4 GPIO21 | RX from STM32 PA9 / USART1 TX |
| ESP32-P4 GND | STM32 GND |

If GPIO21 and GPIO22 are reversed, the script detects the STM32 heartbeat and
reconfigures UART1 to the reversed orientation. Both devices use 3.3 V UART
logic at 115200 baud, 8N1.

## Thonny

Use a current ESP32-P4 MicroPython build. Save the complete script to the
ESP32-P4 as `main.py`, then reset the board. A working link prints:

```text
received HB ...
UART round trip verified: OK PONG
STM32 CONNECTED: ...
```

Useful shell commands:

```python
ping()
check_connection()
motor_status()
currents()
monitor(5)
diagnostics()
stop_all()
```

`ping()` must return `OK PONG`. A heartbeat alone verifies only the
STM32-to-ESP32 direction; the PING/PONG exchange verifies both directions.
No background thread is started, so the Thonny shell remains usable.
