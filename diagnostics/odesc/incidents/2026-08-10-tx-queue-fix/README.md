# ODESC UART shutdown diagnosis

The P4 persistent log showed the mower starting at boot uptime 1110.427 s and
the first missing ODESC reply at 1197.376 s, 86.949 seconds later. The failed
request was `r ibus`. USB inspection after the event showed:

- ODESC MCU and USB still responsive;
- axis idle with `axis.error = 2048` (`WATCHDOG_TIMER_EXPIRED`);
- motor, controller, and sensorless-estimator errors all zero;
- UART RX bytes increasing, with zero DMA restarts and zero UART errors;
- the P4 repeatedly transmitting probes but receiving no replies;
- the separate P4-GD32 UART remaining operational.

The watchdog was therefore a consequence, not the initiating fault. The ODESC
UART TX-complete interrupt posted its sole completion event to a four-entry
queue already receiving 8 kHz poll events and ignored queue-full failures. A
dropped completion permanently blocked the asynchronous TX stream.

The transport fix is archived under
`firmware/odesc-v42/releases/uart-tx-recovery-v0.5.6/` and was flashed and
verified on 2026-08-10. `preflash-config.json` is the configuration backup made
immediately before deployment.
