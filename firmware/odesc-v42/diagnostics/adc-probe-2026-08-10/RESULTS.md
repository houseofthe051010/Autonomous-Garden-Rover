# ODESC VBUS ADC probe results

Date: 2026-08-10

## Test conditions

- Independently measured DC bus: 24.7 V
- Firmware: `ODESC-V4.2-v0.5.6-adc-probe-safe`
- Motor phases, AUX output, and external GPIO disconnected
- Safety status before and after each scan: `SAFE 0xfff faults 0`
- ADC reference used by firmware conversion: 3.3 V
- ADC full scale used by firmware conversion: 4096 counts

## Result

ADC1 channel 6 (STM32 PA6, the configured `VBUS_S` input) was stable:

| Samples | Average | Minimum | Maximum | Firmware ADC voltage |
|---:|---:|---:|---:|---:|
| 250 | 2743.37 | 2738 | 2748 | 2.21034 V |

Using the independently measured 24.7 V bus gives an effective calibrated
divider ratio of approximately 11.175:1:

```text
ratio = 24.7 / (2743.37 * 3.3 / 4096) = 11.175
```

The corresponding maximum measurable bus voltage is approximately 36.87 V:

```text
Vbus_max = 24.7 * 4095 / 2743.37 = 36.87 V
```

This explains the former exact 36.29 V reading with the nominal 11:1 firmware:
the physical VBUS input saturates near 37 V. Software cannot recover the actual
voltage of a 40-42 V 10S pack after the ADC reaches 4095.

Channel 6 is the correct VBUS ADC channel. Other channels that happened to
produce values resembling another divider ratio at this single operating point
are unrelated board signals or floating inputs.

The closest alternate numerical candidate was ADC1 channel 9: 1.34677 V at a
24.7 V bus implies 18.34:1. This MCU input is PB1, labelled `M1_CL` in the ODrive
v3.6 pin map, and is an unused second-axis phase-output pin on this single-axis
board. Its 250-sample range was 1624-1721 counts, compared with 2738-2748 for
the labelled VBUS channel 6. A second known bus voltage can conclusively reject
channel 9 by checking whether it scales linearly with the bus.

## Required correction for 10S operation

The physical resistor divider must be changed, or a correctly scaled external
VBUS signal must be connected to PA6. A nominal 19:1 divider provides suitable
measurement range for a 10S pack and margin above 42 V. Firmware must use the
actual measured divider ratio after the hardware change. Do not select another
ADC channel or apply a software-only 19:1 multiplier to the existing divider.

Raw scans are stored in `adc-scan.csv`, `adc-scan.txt`, and
`adc-scan-24.7V.txt` in this directory.

## 42.5 V confirmation

A second scan was captured with an independently reported 42.5 V battery:

- The diagnostic interlock remained `SAFE 0xfff faults 0` before and after.
- ADC1 channel 6 was exactly 4095 for every sample, confirming saturation.
- ADC1 channel 9 averaged 2191.24. If its 24.7 V reading had represented a
  proportional 18.34:1 VBUS divider, it would have risen to approximately
  2876 counts at 42.5 V.
- No other scanned channel scaled in direct proportion to the bus voltage.

Therefore PA6 / ADC1 channel 6 is the board's only VBUS measurement, and the
existing physical divider cannot measure a full 10S battery. The raw comparison
is in `adc-scan-42.5V.txt` and `adc-scan.csv`.
