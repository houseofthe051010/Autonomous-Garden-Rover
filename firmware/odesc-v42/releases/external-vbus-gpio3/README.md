# ODESC V4.2 external VBUS input on GPIO3

This is a **superseded bench-validation release**. The tested replacement is
stored in `../external-vbus-gpio3-stable-20260811/`. This image replaces the
clipped onboard bus-voltage reading used by FOC with a calibrated external
divider on GPIO3 while retaining the original ADC as independent telemetry and
as a lower-bound plausibility check.

## Hardware

Wire the divider exactly as follows:

```text
battery + ---- 100 kOhm ----+---- ODESC GPIO3
                            |
                            +---- 5.6 kOhm ---- battery GND / ODESC GND
                            |
                            +---- 10 nF (103) - battery GND / ODESC GND
```

The capacitor is from GPIO3 to ground, not directly across the battery. GPIO3
must remain in `GPIO_MODE_ANALOG_IN`. The measured calibration point was:

- multimeter battery voltage: 36.200 V
- GPIO3 ADC voltage: 1.935543 V mean over 200 paired samples
- calibrated scale: 18.702757793
- expected GPIO3 voltage at 42.0 V: approximately 2.246 V

## Control behavior

- GPIO3 is sampled by the existing ADC1 DMA scan at roughly 32 kHz.
- 32 consecutive valid control samples are required before GPIO3 is selected.
- 64 consecutive invalid samples latch `EXTERNAL_VBUS_INVALID` and disarm PWM.
- The onboard ADC must not exceed the external result by more than 1 V or 3%.
- Below 34 V, both ADCs must agree in either direction within the same limit.
- Above the onboard ADC clipping point, an external reading above the onboard
  reading is allowed.
- Once the external source has been accepted, it is the value used by FOC,
  modulation, power, brake calculations, voltage trips, CAN, and `vbus_voltage`.
- A latched fault requires restoring the divider, waiting for validity, and
  explicitly clearing errors. It does not silently re-arm the motor.

Validation status bits are: `1` warmup, `2` GPIO mode/channel, `4` low,
`8` ADC high, and `16` disagreement with the onboard ADC.

## Read over USB

With `odrivetool` connected:

```python
odrv0.vbus_voltage                    # selected value used by control
odrv0.vbus_voltage_internal           # old onboard converted VBUS value
odrv0.vbus_voltage_internal_raw       # original ADC1 JDR1 count
odrv0.vbus_voltage_internal_adc       # original ADC pin voltage
odrv0.vbus_voltage_external           # calibrated GPIO3 battery voltage
odrv0.vbus_voltage_external_adc       # GPIO3 ADC pin voltage
odrv0.vbus_voltage_external_scale     # 18.702757793
odrv0.vbus_voltage_external_valid
odrv0.vbus_voltage_external_fault
odrv0.vbus_voltage_external_status
```

## Read over UART ASCII

Each property is also available through the existing 115200-baud UART:

```text
r vbus_voltage
r vbus_voltage_internal
r vbus_voltage_internal_raw
r vbus_voltage_internal_adc
r vbus_voltage_external
r vbus_voltage_external_adc
r vbus_voltage_external_scale
r vbus_voltage_external_valid
r vbus_voltage_external_fault
r vbus_voltage_external_status
```

This preserves direct access to the original onboard ADC over both USB and
UART even though it is no longer the selected FOC bus-voltage source.

## Required staged validation

Do not begin with a fully charged pack or an attached load.

1. Keep M0 idle and mechanically unloaded. Use the present approximately
   36.2 V battery condition for the first flash.
2. Verify `vbus_voltage_external_valid == True`, fault is false, status is zero,
   GPIO3 ADC is about 1.936 V, and selected VBUS agrees with the multimeter.
3. Confirm the original onboard fields are still readable and show the expected
   approximately 36.2 V / near-full-scale behavior.
4. Configure and verify a 10S overvoltage trip of 44.0 V before allowing PWM.
   Existing saved configuration can override firmware defaults.
5. With M0 still idle, disconnect only the GPIO3 sense lead. Confirm status/fault
   latch and M0 cannot arm. Restore the lead, wait for valid status, then clear
   errors manually.
6. Run a supervised, current-limited, low-speed no-load motor test at 36.2 V.
7. Increase pack voltage in stages and compare against a multimeter at each
   stage before finally reaching 42.0 V.

Do not install an ordinary 1/4 W resistor as a brake resistor. Keep regenerative
braking disabled unless a correctly sized power resistor and thermal design are
installed.

## Artifacts and rollback

- New BIN SHA-256:
  `eb7165f266e0a48aa865b952223da0386aa873284c2beade1fb9c3b4f2f0a7f1`
- Rollback BIN SHA-256:
  `ca2ce1f227941927e795adf9271b30970cd4c7619aedecf79ed1c6b84d0b35d4`
- Build size: 320,904 bytes text, 3,296 bytes data, 143,776 bytes BSS.

The rollback image is the UART-TX-recovery firmware that was running before
this external-VBUS change.

## ESP32-P4 integration

The repository P4 source now queries `vbus_voltage_external_valid`,
`vbus_voltage_external_fault`, and `vbus_voltage_external_status`. It accepts a
selected voltage above 36.3 V only when all new validity checks pass. If an old
ODESC image is detected, the original 36.3 V clipping interlock remains active.
A missing validity reply from new firmware locks motion until a complete valid
set returns.

The matching P4 source built successfully with ESP-IDF 6.0.1 on 2026-08-11.
Its application size is 1,173,312 bytes, leaving 72% of each 4 MiB OTA slot
free.

## 2026-08-11 deployment record

The release HEX was flashed through native USB DFU to ODESC serial
`357F356D3135`. DFU erase, write, and readback verification all completed. The
configuration was restored, the 10S overvoltage trip was changed from 59.92 V
to 44.0 V, brake-resistor operation remained disabled, startup closed-loop
control remained disabled, and M0 was left idle.

At the approximately 36.2 V validation point the board reported:

- selected VBUS: approximately 36.10 V
- original onboard VBUS: approximately 36.18 V, raw ADC approximately 4083
- external VBUS: approximately 36.06 V, GPIO3 ADC approximately 1.926 V
- external valid: true; fault: false; status: zero
- ODESC system, axis, motor, controller, encoder, and estimator errors: zero

The matching P4 application, SHA-256
`9c65fded087991afaa23ea9fd94553ae427eb28414e27251376f37c64559e70a`,
was OTA-deployed to `ota_0`. It recognized the new validity fields and retained
a healthy GPIO27/GPIO47 UART link for a 12-sample observation: zero command
failures, FIFO overflows, frame errors, parity errors, or motor motion.

Preflash and postflash JSON configuration backups are stored beside this file.
They differ only by the intentional 44.0 V overvoltage-trip setting. The
physical GPIO3-disconnect interlock test, supervised motor test, and staged
comparison at higher pack voltages through 42.0 V remain required. This record
does not qualify the system for unattended charging or operation.
