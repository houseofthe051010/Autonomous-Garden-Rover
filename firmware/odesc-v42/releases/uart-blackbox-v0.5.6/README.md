# ODESC UART blackbox image

Build target: ODrive `fw-v0.5.6`, `v3.6-56V`, single physical M0 ODESC clone.

This image adds read-only UART diagnostics, receive-DMA recovery, and a
CRC-checked event journal in reserved STM32F405 flash sector 9. It does not
change the motor-control, gate-driver, FOC, current-limit, or sensorless-control
algorithms. Internal flash is programmed only while physical M0 is disarmed
and IDLE. Sector erase is never performed at runtime.

Artifacts:

| File | SHA-256 |
| --- | --- |
| `ODriveFirmware.bin` | `c89e406ae3b931a44594b7df6f9472620dca445013f0ab0ef8fb2581a9b4f85e` |
| `ODriveFirmware.hex` | `f98cf4f9f981713312fb52e045a3161629420b028a52203b0bb46c7edc80ad57` |
| `ODriveFirmware.elf` | `5beb7f352ca14b897cf4879c10d971d43a1f7e870e9a5a35a5d3636da409f168` |

The linked image uses 318,820 bytes of text/constant data and ends below
`0x0804F000`. The blackbox starts at `0x080A0000`, leaving more than 330 KiB
between the application and journal.

Deployment status on 2026-08-10: **flashed and verified**. ROM DFU wrote only
the application sectors and performed a one-time erase of reserved diagnostic
sector 9; configuration sectors 10/11 were not erased. USB and GPIO UART both
reported M0 IDLE, no axis/motor/estimator errors, retained 115200 baud ASCII
UART configuration, 20 A current limit, and 80 turns/s speed limit. The P4 read
the first CRC-checked boot record and reported zero UART hardware errors. The
previous known-working artifacts remain in
`../../images/pre-uart-blackbox-2026-08-10/`.

After a supervised flash, verify at idle over UART:

```text
ds
da 0
db 0
dc 0
dd 0
```

`ds` must begin with `DS`. The four record reads must begin with `DA`, `DB`,
`DC`, and `DD`. Do not run the motor until ordinary `vbus_voltage`, state, and
error reads match the pre-update configuration.
