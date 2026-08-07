# Raspberry Pi 5 Legacy Host

The Raspberry Pi 5 was the original high-level rover controller. It is retained
as a working alternative and diagnostic reference, but current development uses
the ESP32-P4 host.

| Directory | Purpose |
| --- | --- |
| [`combined-controller/`](combined-controller/README.md) | Four-UART Thonny client, web UI, and service files |
| [`stm32-drive/`](stm32-drive/README.md) | STM32/BTS7960 client and validated wiring |
| [`gd32-stepper/`](gd32-stepper/README.md) | Ender/GD32 stepper client |
| [`bno080/`](bno080/README.md) | BNO080 UART-SHTP sensor client |
| [`odesc/`](odesc/README.md) | ODESC/ODrive ASCII diagnostics |

The combined controller contains a standalone Python file for a Pi without a
checkout and a modular development version. Its README explains the distinction.
