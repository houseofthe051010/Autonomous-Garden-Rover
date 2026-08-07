# Protocols

- [`stm32-drive-uart.md`](stm32-drive-uart.md): STM32 dual-BTS7960 ASCII UART
  protocol, heartbeat, current sensing, and optional encoder streaming.
- [`../../firmware/gd32-stepper/UART_PROTOCOL.md`](../../firmware/gd32-stepper/UART_PROTOCOL.md):
  current GD32 simultaneous direct-motion protocol.
- [`gd32-stepper-uart-legacy.md`](gd32-stepper-uart-legacy.md): earlier finite
  Marlin motion protocol retained for compatibility and history.

Controller-specific Wi-Fi and ESP-NOW packet formats are documented beside the
corresponding firmware because both endpoints must be changed together.
