# ODESC V4.2 safe ADC probe

This diagnostic exists only to identify which STM32F405 ADC1 channel is wired
to the clone board's DC-bus divider. It is not motor firmware.

## Electrical shutdown design

The diagnostic image has independent shutdown layers:

1. `EN_GATE` (PB12) is held low, keeping the DRV8301 in reset.
2. TIM1 and TIM8 have `MOE`, `AOE`, `CEN`, and every `CCER` output cleared.
3. All six M0 phase-control pins are analog inputs instead of timer outputs.
4. TIM2 is stopped and disconnected, and PB10/PB11 AUX controls are analog.
5. Gate-driver setup, current calibration, motor PWM startup, brake PWM startup,
   analog control, and both axis state-machine threads are omitted.
6. A 1 ms loop checks all shutdown conditions and immediately reasserts them.
7. USB-CDC rejects every ASCII command except ADC/status and read-only queries.

`astatus` must return `SAFE 0xfff faults 0` before and after a scan. Any other
value means stop the test and remove power.

## Required bench state

- Disconnect all three motor phase wires from the ODESC.
- Do not connect a brake resistor or load to AUX during channel identification.
- Disconnect the ESP32-P4 UART and other ODESC GPIO wiring.
- Use USB only for diagnostics.
- Start with a current-limited bench supply at 12 V, not the 10S battery.
- Limit the supply to approximately 0.25 A for the logic-only test.
- Keep fingers and probes away from the MCU and power stage while energized.

The firmware cannot make a connected high-energy battery inherently safe. The
disconnected motor and current-limited low-voltage supply are required barriers.

## Image

Flash only:

`releases/adc-probe-safe-v0.5.6/ODESC-V4.2-v0.5.6-adc-probe-safe.hex`

SHA-256:

`3298e5f8c465b91c94e224a2cc207dbf0213281584571052dd7b0a43532b8d33`

The known working sensorless rollback image remains separately preserved under
`releases/working-sensorless-v0.5.6-vbus11/`.

## Read all channels

Install the Raspberry Pi OS/Ubuntu serial package if it is not already present:

```bash
sudo apt install python3-serial
```

Capture one scan at 12 V and another at 24 V:

```bash
python3 tools/odesc_adc_probe.py --port /dev/ttyACM0 --label 12V --csv /tmp/odesc-adc.csv
python3 tools/odesc_adc_probe.py --port /dev/ttyACM0 --label 24V --csv /tmp/odesc-adc.csv
```

The real VBUS channel will rise in direct proportion to supply voltage. Channels
that remain fixed, noisy, or saturated are not usable as VBUS evidence. Compute
the physical divider ratio only after the same channel scales correctly at both
known voltages.

After the probe, restore the archived working sensorless image. Do not enable a
motor until the true ADC channel/divider has been verified and overvoltage
protection has been tested against an independent multimeter.
