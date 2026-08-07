# STM32F103C6 BTS7960 Firmware

This is the preserved PlatformIO source and built binary for the Blue Pill
controller. It drives two BTS7960 modules, reports their current-sense ADCs,
maintains the motor watchdog/heartbeat, and provides opt-in PA0-PA3 analog
encoder telemetry.

## Build and ST-Link flash

From this directory:

```sh
pio run
pio run --target upload
```

The project targets `bluepill_f103c6` and uses ST-Link. The checked-in
`firmware.bin` was built and flashed on 2026-07-31 and has SHA-256:

```text
6b21a72cd9ee842a5128332eba70413e1d3e79ed70e79fac135bdc693edd89d1
```

## Hardware indicators

- PC13 performs three rapid flashes during startup.
- After startup, the active-low PC13 onboard LED blinks while valid UART host
  commands are arriving. It turns off 2.5 seconds after the last valid command.
- PB4 continues toggling once per second as an independent firmware-running
  diagnostic output.

An outgoing heartbeat alone does not enable the PC13 link indication. A valid
received command such as `PING` or `MSTATUS` is required, which proves the host
to STM32 direction of the UART connection.

## Encoder controls

Streaming is off at boot. `ENCREAD` requests one PA0-PA3 sample set. `ENCON 50`
starts the recommended 50 Hz stream and `ENCOFF` stops it. See the
[STM32 UART protocol](../../docs/protocols/stm32-drive-uart.md) for the complete
protocol.

Command parsing and the 1.5-second motor watchdog run before periodic
telemetry. UART receive work is bounded per main-loop pass, and encoder reports
are deferred when commands are waiting or the transmit buffer is crowded. The
Pi reader bounds incoming line length, tracks missed encoder sequence numbers,
and keeps asynchronous encoder/heartbeat messages separate from command
responses.

## Future flashing over Raspberry Pi UART

The existing PA9/PA10 connection is also the STM32F10x factory ROM bootloader's
USART1. UART flashing is therefore possible without changing the data wires.
The STM32 must first boot into system memory: manually set BOOT0 high and reset,
or later add two safely interfaced Pi GPIO controls for BOOT0 and NRST. Close
the normal `/dev/ttyAMA1` controller before running a flashing tool. After the
flash, set BOOT0 low and reset to run the application.

The bootloader uses 8 data bits, even parity, and one stop bit and begins with
the `0x7f` auto-baud byte. This is different from the application's 115200 8N1
protocol. Do not depend only on an application command that jumps to the ROM:
hardware BOOT0/NRST access is needed to recover from broken firmware. Verify
the ROM bootloader on the actual board before making it the only update method,
especially for an STM32-compatible or relabeled part.

References: ST [AN2606 system-memory boot mode](https://www.st.com/resource/en/application_note/an2606-stm32-microcontroller-system-memory-boot-mode-stmicroelectronics.pdf)
and [AN3155 USART bootloader protocol](https://www.st.com/resource/en/application_note/an3155-usart-protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf).
