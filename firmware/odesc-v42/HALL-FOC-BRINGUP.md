# Hall-sensored FOC bring-up

This is the supervised procedure for the single physical M0 axis on the
Flipsky/ODESC V4.2. Do not use `axis1`. The rover is currently powered off, so
no device configuration, flashing, calibration, or motor movement was attempted
while preparing this procedure.

## Engineering decision

Do **not** patch the FOC implementation or hard-code the orange Hall wire's
inversion. The exact firmware already has the required correction mechanism:

- `encoder.cpp::decode_hall_samples()` maps A/B/C to bits `1/2/4`.
- `encoder.cpp::run_hall_polarity_calibration()` identifies no inversion or one
  inverted A, B, or C channel and stores the mask in `hall_polarity`.
- `encoder.cpp::update()` decodes `hall_state ^ hall_polarity` and rejects
  non-adjacent transitions.
- `axis.cpp::start_closed_loop_control()` selects `encoder.phase` and
  `encoder.phase_vel` as the FOC phase sources when sensorless mode is disabled.

This is the upstream ODrive 0.5.6 design for exactly this situation. Adding a
second inversion in the current-control path could double-invert the signal and
commutate the motor incorrectly. The Hall-capable image is therefore the same
byte-verified image that already passed sensorless testing.

## Preserved rollback image

The immutable rollback release is:

`releases/working-sensorless-v0.5.6-vbus11/`

Important SHA-256 values:

| File | SHA-256 |
| --- | --- |
| `ODESC-V4.2-v0.5.6-56V-vbus11-working-sensorless.hex` | `8348a31b722ac61204d32ae4105f08b6a271b833838cd65cbecca848916ab925` |
| `ODESC-V4.2-v0.5.6-56V-vbus11-working-sensorless.elf` | `b9af977dfeae2bdc708d3c1abf4d51c0fc7593b66d72c52c16d9518a3bd1adb1` |

The HEX produced by a clean source rebuild was compared byte for byte with the
rollback HEX and matched. This proves that the saved source, board target, and
VBUS-divider patch reproduce the flashable program. ELF files can contain
build metadata and need not have matching whole-file hashes; the generated HEX
is the flash-byte comparison.

## Hall connector

Use the five-pin ABZ encoder input:

| ODESC label | Hall function | Firmware bit |
| --- | --- | --- |
| `A` | Hall A | `1` (`0b001`) |
| `B` | Hall B | `2` (`0b010`) |
| `Z` | Hall C | `4` (`0b100`) |
| `5V` | Hall supply | not a signal |
| `GND` | Hall ground | not a signal |

The orange motor wire's correction mask depends on which ODESC input it reaches,
not its color: A uses mask 1, B uses mask 2, and Z/C uses mask 4. Do not swap 5 V
and ground, and do not change motor phase wiring during Hall diagnosis.

## No brake resistor

There is no brake resistor installed. Keep
`config.enable_brake_resistor = False`. A battery/BMS that cannot absorb
regenerative current can overvoltage or trip during deceleration even with that
setting. For initial tests:

- mechanically unload the motor and lift/disconnect the drivetrain;
- provide a physical emergency power disconnect;
- keep automatic calibration and closed-loop startup disabled;
- use low calibration current, velocity, and acceleration;
- never command a high-speed reversal or abrupt high-speed stop;
- do not increase the negative DC-current limit merely to suppress a fault.

## Gate 1: read-only identity and backup

Install the one host dependency if needed:

```bash
sudo apt install python3-serial
```

With USB connected and M0 IDLE, capture the actual live configuration before
any write:

```bash
cd /home/aditya/Documents/Autonomous-Garden-Rover
python3 firmware/odesc-v42/tools/odesc_hall_setup.py \
  --port /dev/ttyACM0 status \
  --output diagnostics/odesc/hall-before-setup.json
```

Stop if system, axis, or motor errors are nonzero, VBUS is implausible, M0 is
not IDLE (`1`), the physical board identity differs, or the snapshot cannot be
saved. Confirm the rollback HEX hash before considering any flash:

```bash
cd firmware/odesc-v42/releases/working-sensorless-v0.5.6-vbus11
sha256sum -c SHA256SUMS
```

Do not reflash a healthy board merely to configure Hall feedback.

## Gate 2: stage Hall inputs without movement

First inspect the dry run:

```bash
python3 firmware/odesc-v42/tools/odesc_hall_setup.py \
  --port /dev/ttyACM0 prepare-hall
```

It sets encoder mode Hall, CPR to `6 * pole_pairs` (42 for this 7-pole-pair
motor), encoder bandwidth to 100, encoder GPIO9/10/11 to digital, all automatic
motor startup options off, sensorless mode off, and brake-resistor use off. It
also clears the Hall calibration flags. It does not energize the motor.

Only after reviewing the dry run and confirming M0 is IDLE, apply it:

```bash
python3 firmware/odesc-v42/tools/odesc_hall_setup.py \
  --port /dev/ttyACM0 prepare-hall --execute \
  --confirm CONFIGURE-HALL-WITH-MOTOR-IDLE \
  --snapshot diagnostics/odesc/hall-before-setup.json
```

Power-cycle the ODESC afterward because encoder and GPIO mode changes take
effect at boot. It must remain IDLE after reboot.

## Gate 3: manually verify the inversion

This gate does not energize the motor. Slowly turn the unloaded shaft by hand
through at least two complete mechanical revolutions:

```bash
python3 firmware/odesc-v42/tools/odesc_hall_setup.py \
  --port /dev/ttyACM0 observe --seconds 20
```

A valid result must find all six corrected states in adjacent order and exactly
one mask. Raw `000` and `111` can appear when one physical channel is inverted;
they must disappear after the candidate XOR mask. Stop for missing states,
multiple candidate masks, skipped transitions, unstable readings while the
shaft is stationary, or a result that differs between slow forward and reverse
rotation.

The firmware's motor-driven polarity calibration in Gate 4 should independently
select the same mask. `set-mask` exists only as a supervised fallback and is a
dry run unless both its execution flag and confirmation token are supplied.

## Gate 4: supervised Hall polarity calibration

This gate moves the motor. Keep the motor mechanically unloaded and a hand on
the emergency power disconnect. Use `odrivetool` over USB so all errors can be
inspected immediately. Before requesting state 12:

1. Confirm M0 is IDLE and all error fields are zero.
2. Confirm `axis0.motor.is_calibrated` is true and the stored phase resistance
   and inductance match the known working values.
3. Confirm automatic startup remains disabled and sensorless mode is false.
4. Start `axis0.config.calibration_lockin.current` at 3 A. Increase only to the
   previously validated 5 A if cogging prevents smooth rotation.
5. Confirm `config.enable_brake_resistor` remains false.

Then request only:

```python
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
```

The source runs this for about three seconds at its configured electrical
lock-in velocity. Immediately request IDLE or remove power for violent motion,
wrong direction changes, severe vibration, unusual sound, rising VBUS, or any
fault. On success, require:

```python
odrv0.axis0.current_state == AXIS_STATE_IDLE
odrv0.axis0.error == 0
odrv0.axis0.motor.error == 0
odrv0.axis0.encoder.error == 0
odrv0.axis0.encoder.config.hall_polarity_calibrated == True
```

The resulting `hall_polarity` must equal the read-only candidate from Gate 3.
Do not proceed if it differs.

## Gate 5: phase/offset calibration

Run the standard state 7 encoder offset calibration only after Gate 4 passes:

```python
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
```

This aligns the Hall sequence to rotor electrical phase while using the existing
FOC current controller. Do not modify `motor.cpp`, current-sense scaling, PWM
timing, phase order, or FOC gains. Require return to IDLE with every error field
zero and `axis0.encoder.is_ready == True`.

Do not set `encoder.config.pre_calibrated`, save configuration, or enable startup
until repeated low-current tests survive a reboot and reproduce the same Hall
sequence and direction.

## Gate 6: bounded closed-loop test

Start in velocity control with a low current limit and a very low command. Keep
the motor unloaded. Test one direction, return gently to zero, request IDLE,
then test the other direction. Monitor at least:

- axis, motor, encoder, and controller errors;
- Hall state, encoder velocity, `Iq_setpoint`, and `Iq_measured`;
- VBUS, bus current, and motor/inverter temperatures;
- unexpected state changes or non-adjacent Hall transitions.

Only save Hall calibration after both directions work at low speed and the
motor starts reliably from every rotor position. Retain the automatic startup
flags as false. The ESP32-P4 must be updated from sensorless-estimator telemetry
to `axis0.encoder.vel_estimate` before it is allowed to command this mode.

## Rollback

If Hall bring-up fails, request IDLE, remove motor power, retain the diagnostic
snapshot and error log, and restore configuration before considering a firmware
flash. The rollback HEX path and hash are listed above. Flash rollback only
with the board physically supervised and identified as this exact single-axis
ODESC V4.2; never use an image selected only by a similar product name.
