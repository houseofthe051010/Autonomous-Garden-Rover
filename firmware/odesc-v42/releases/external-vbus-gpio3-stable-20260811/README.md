# ODESC V4.2 stable external-VBUS sensorless image

This is the deployed image for the single-axis Flipsky/SEQURE-style ODESC V4.2
clone. It keeps the GPIO3 external battery divider as the voltage used by FOC
and fixes the current-measurement failure found during sensorless startup.

## Fixed faults

- `ODrive.ERROR_DC_BUS_OVER_VOLTAGE` was being latched by noisy, unfiltered
  GPIO3 readings even when the displayed average was below 44 V.
- M0 disarmed about 130 ms after startup with
  `Motor.ERROR_UNKNOWN_CURRENT_MEASUREMENT` (`0x40000000`). The clone's
  nominal zero-vector ADC sample contained phase current while PWM was active,
  corrupting phase-A DC calibration from about 3.8 A to its 6.075 A validity
  ceiling. DC offsets now calibrate continuously in IDLE and remain fixed only
  while armed. Live phase-current samples and every current limit remain active.
- External-VBUS plausibility validation no longer runs in the phase-current
  interrupt. Open/short/ADC-range checks remain synchronous with control.
- GPIO3 voltage uses a 0.10 IIR coefficient for isolated ADC outliers. Three
  consecutive raw samples over the configured trip voltage bypass the filter,
  giving an overvoltage response below 0.5 ms at the 8 kHz control rate.

## Voltage calibration

The deployed scale is `19.231321981`, calibrated from the user's simultaneous
42.5 V multimeter reading and a GPIO3 ADC mean near 2.210 V. Final idle USB
sampling reported:

- selected VBUS: 42.186 to 42.469 V, mean 42.309 V
- external VBUS: 42.194 to 42.423 V, mean 42.307 V
- GPIO3 ADC mean: 2.200329 V
- external valid/fault/status: `true / false / 0`

## Motor verification

On 2026-08-11, M0 was tested unloaded in sensorless velocity mode at 10 turns/s.
The USB test remained armed for 2.5 seconds and reached about 9.4 turns/s with
zero system, axis, motor, or estimator errors. A second test through the
ESP32-P4 HTTP/UART path reached 9.80 turns/s (587.9 RPM), also with zero errors,
then returned to IDLE cleanly.

## Artifacts

```text
BIN  021ab5c32c88bd8e5123425e94c1727e612bbc702a5f829984a93f8664e75b67
HEX  4d1de07a22a1827b41051732b1460bda5406cd18b43fa36f3d485bc09903257a
ELF  0dbd8df9a29b16ec3ff4e91b46020c191e93dbf122da3258587c5e14d4c32b47
```

Build size: 321,032 bytes text, 3,296 bytes data, 143,784 bytes BSS.

The previous image remains at
`../external-vbus-gpio3/ODESC-v42-external-vbus-gpio3.bin`. The pre-external-
VBUS rollback remains at
`../external-vbus-gpio3/ROLLBACK-ODriveFirmware-uart-tx-recovery.bin`.

## Safety

The configured bus overvoltage trip remains 44.0 V and the brake resistor
remains disabled. Do not raise the trip to hide faults. A normal balanced 10S
Li-ion pack is 42.0 V at 4.20 V/cell; a 42.5 V pack should be disconnected from
the charger and its charger setting and individual cell-group voltages checked.

