# ESP32 hose valve controller

This ESP32 controls the continuous-rotation MG996 valve actuator and reports
its shaft potentiometer to the handheld controller over ESP-NOW.

## Wiring

| Hose ESP32 | Connection |
| --- | --- |
| GPIO14 | MG996 signal, 50 Hz, 500-2500 us |
| GPIO35 | Potentiometer wiper, ADC1 channel 7 |
| GND | Common controller, potentiometer, and servo-supply ground |

The MG996 requires a separate 5-6 V regulator sized for servo stall current.
Do not power it from the ESP32. GPIO35 must never exceed 3.3 V; power the
potentiometer from 3.3 V or divide its output.

The hose ESP32 station MAC is `a4:f0:0f:66:d2:d0`. It accepts packets only from
the handheld controller MAC `68:09:47:5c:04:c4`. Both use ESP-NOW channel 6,
which is also the rover P4 access-point channel.

## Handheld controls

Navigate TFT rover page -> input diagnostics -> hose page. On the hose page,
the upper/lower left regions are hold-to-run direction `+1`/`-1`, upper right
cycles speed from 100 through 1000, lower right resets zero, and the center dial
returns to the rover page. Pico GP15 enables the hose link on this page and GP14
disables it. Releasing a direction region commands neutral.

## Safety and position tracking

- Startup output is always 1500 us neutral.
- Enabled control packets are required at least every 350 ms or the output
  returns to neutral.
- Speed 100-1000 maps to 100-1000 us away from neutral. Direction `+1` maps to
  1600-2500 us; direction `-1` maps to 1400-500 us.
- GPIO35 is sampled at 50 Hz with a median filter and IIR smoothing.
- The measured installation has decreasing ADC values in the `+1` direction.
  The tracker counts completed, direction-aware traversals across the
  potentiometer's zero-output blind region. It uses the ADC value on each side
  of that region to reject a move that enters and exits from the same side.
- The displayed signed total changes by exactly one revolution per validated
  blind-region traversal. ADC is still reported independently for the dial.
- Reset-zero offsets only the accumulated revolution count. It does not zero,
  rescale, or discard the raw/filtered ADC reading.

The first bounded USB diagnostic on 2026-08-01 used only 1600 and 1400 us.
GPIO35 moved smoothly from approximately 1606 down toward 630, then back to
approximately 1606, confirming both directions, ADC feedback, and neutral.

## Protocol

Commands are 16 bytes at 20 Hz: magic `0xC1`, version, signed direction,
enable/reset flags, speed, sequence, CRC16-CCITT, and trailing version. Status
is 24 bytes at 10 Hz: magic `0xC2`, version, raw/filtered ADC, signed total
revolutions encoded in thousandths, pulse width, direction, flags, speed,
sequence, CRC16, and trailing version. Both receivers reject the wrong length, magic,
version, CRC, value ranges, and source MAC.

## Build and flash

```bash
cd firmware/esp32-hose
~/.platformio/penv/bin/pio run
~/.platformio/penv/bin/pio run -t upload --upload-port /dev/ttyUSB0
```

The permanent image was built, flashed, and serial-validated on 2026-08-01.
It booted at 1500 us, initialized its first valid ADC position as `0.000`,
received increasing valid controller packet counts with zero rejects, and
returned increasing delivered telemetry counts.

On 2026-08-02 the image was rebuilt and hash-verified while flashing MAC
`a4:f0:0f:66:d2:d0` through `/dev/ttyUSB0`. With the servo wiring attached and
both controller boards powered from the computer's USB bus, the hose ESP32 then
repeatedly reported `Brownout detector was triggered` during Wi-Fi RF
calibration, before ESP-NOW initialization completed. The firmware already uses
the ESP32's lowest brownout threshold. This is a supply/cable/regulator problem,
not an ESP-NOW protocol or flash failure.

Power the MG996 from a separate adequately rated 5-6 V supply and connect its
ground to ESP32 ground. Do not power the servo from the ESP32 5 V/3.3 V pins.
The ESP32 itself also needs a stable USB or regulated 5 V source. After power is
correct, the boot log must reach `ready MAC=... servo neutral` and continue with
periodic status instead of restarting.
