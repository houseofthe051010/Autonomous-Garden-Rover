# ODESC v0.5.6 UART TX recovery

This image is based on the rover's working ODrive v0.5.6, v3.6-56V,
single-axis ODESC configuration. It fixes a UART transport deadlock without
changing motor-control, FOC, voltage-sense, or current-limit code.

## Failure fixed

The UART thread receives an 8 kHz polling event and TX-complete events through
one four-entry RTOS queue. The TX interrupt ignored a failed queue insertion.
If its one completion event was dropped, the asynchronous TX stream remained
busy and the ODESC stopped replying until reboot, even though RX and USB still
worked. The axis watchdog then expired because the P4 could no longer feed it.

The fix:

- latches TX completion independently of the queue;
- services that latch on the next UART-thread wakeup;
- makes stale TX queue entries harmless;
- increases the shared queue from four to eight entries;
- appends the TX-completion queue-drop count to the `ds` diagnostic response.

`ds` now returns:

```text
DS records rx_bytes dma_restarts uart_errors active_silence tx_queue_drops
```

## Deployment

Flashed over USB DFU on 2026-08-10. After boot, USB and P4 UART were both
verified while the axis remained idle. The saved motor configuration was
unchanged: 20 A current limit, 80 turns/s velocity limit, sensorless enabled,
and motor pre-calibrated.

The preceding deployed image is preserved at:

```text
../uart-blackbox-v0.5.6/ODriveFirmware.bin
```

`SHA256SUMS` identifies the exact release artifacts.
