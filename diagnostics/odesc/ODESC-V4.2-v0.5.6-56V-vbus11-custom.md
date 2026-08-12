# ODESC V4.2 custom ODrive v0.5.6 build

This is not a SEQURE or Flipsky factory image. It is ODrive firmware tag
`fw-v0.5.6`, built as `v3.6-56V`, with one board-level change:

```c
#define VBUS_S_DIVIDER_RATIO 11.0f
```

The stock `v3.6-56V` build uses `19.0f`. On this single-axis Flipsky/SEQURE
ODESC V4.2, that image reported about 62.64 V while the factory firmware had
reported about 36.27 V. The ratio between those readings matches `19 / 11`.

The custom build deliberately reports firmware `0.5.6` with
`fw_version_unreleased = 1` so it cannot be confused with an official release.
It retains the `v3.6-56V` hardware identity and its default 59.92 V DC-bus
overvoltage trip level.

## Artifacts

- `ODESC-V4.2-v0.5.6-56V-vbus11-custom.hex`
  - SHA-256: `8348a31b722ac61204d32ae4105f08b6a271b833838cd65cbecca848916ab925`
- `ODESC-V4.2-v0.5.6-56V-vbus11-custom.elf`
  - SHA-256: `b9af977dfeae2bdc708d3c1abf4d51c0fc7593b66d72c52c16d9518a3bd1adb1`
- `config-before-custom-vbus11-fw.json`: configuration captured immediately
  before flashing this build.
- `vendor-config-before-fw-0.5.6.json`: configuration captured before the
  original vendor firmware was replaced.

## Post-flash read-only verification

- USB serial: `357F356D3135`
- Hardware identity: `3.6-56V`
- Firmware identity: `0.5.6`, unreleased/custom
- VBUS: 36.22-36.29 V over 12 samples
- DC-bus overvoltage trip level: 59.92 V
- Axis 0 state: IDLE (`1`)
- Axis 0 error: `0`
- Motor 0 error: `0`
- Controller 0 error: `0`

No motor calibration or motor movement was attempted during this verification.

## Axis 0 sensorless validation

Only the physical M0/Axis 0 output was tested. The C6734 170 KV motor passed
motor calibration at 5 A:

- Pole pairs: 7
- Torque constant: 0.048647 Nm/A (`8.27 / 170`)
- Phase resistance: 0.069027 ohm
- Phase inductance: 22.879 uH
- Motor current limit: 10 A
- Sensorless startup current: 5 A
- Sensorless ramp velocity: 400 electrical rad/s
- Sensorless ramp acceleration: 200 electrical rad/s^2
- Sensorless flux linkage: 0.004633 Wb
- Velocity limit: 15 turns/s
- DC maximum negative current: -2 A

The first sensorless attempt exposed a default `dc_max_negative_current` of
-0.01 A and stopped with `DC_BUS_OVER_REGEN_CURRENT`. Raising the conservative
limit to -2 A removed that configuration fault.

The successful test entered sensorless closed-loop control, commanded 9.09
turns/s (about 545 RPM), reported approximately 8.4-9.8 turns/s, and used about
0.7-1.8 A phase current. It stopped back to IDLE with zero system, axis, motor,
controller, and sensorless-estimator errors.

## ESP32-P4 UART motion validation

On 2026-08-08 the P4 drove the same M0 output over its fixed GPIO27 TX / GPIO47
RX UART link. The deployed controller verified `enable_sensorless_mode=1` and
requested closed-loop state 8; this ODrive 0.5.6 state machine does not implement
the older standalone state 5. A 9.1 turns/s command ramped to approximately
555 RPM. Independent USB reads of `axis0.sensorless_estimator.vel_estimate`
agreed with the P4 at roughly 9.01-9.41 turns/s. Axis, motor, controller, and
sensorless-estimator errors remained zero. STOP returned state 1 and zero RPM,
then twelve post-stop UART checks completed with zero failures.

The subsequent control-limit update added IDLE-only phase-current, velocity,
and acceleration-ramp settings with readback verification. The live ODESC
reported a 10 A phase-current limit, 8 A violation margin, 60.75 A measurement
range, 15 turns/s velocity limit, and 9.095 turns/s sensorless minimum. The
same values were saved to controller NVM, the controller restarted, and the P4
reconnected with input mode 2 (velocity ramp) and zero faults. A bounded ramped
test moved from about 550 RPM to 650-670 RPM, then STOP returned state 1 and
zero RPM. The P4 motion lease is 2 seconds so normal telemetry transactions do
not starve the 260 ms browser keepalive.

For the installed 170 KV motor, the page displays `VBUS * 170` as an electrical
no-load estimate. At 36.1 V this is about 6,140 RPM, with an informational 80%
voltage control ceiling around 4,910 RPM. Limit saves and motion commands are
rejected above that 80% ceiling to preserve controller voltage headroom. This
is still not a mechanical safety rating or guaranteed loaded speed; rotor,
bearing, motor thermal, gearing, wheel, and drivetrain limits must be
established independently and can require a substantially lower setting.

The validated configuration is saved in controller NVM. Automatic calibration
and automatic closed-loop startup remain disabled, so power-up stays IDLE.

After a later extended run, the P4 recorded 2,505 successful ASCII-property
queries before its UART driver stopped receiving replies even though ODESC USB
remained healthy. The reconnect path was corrected to delete and recreate UART3
on every probe instead of reusing the failed driver. Property reads retry once,
idle motion polling is five seconds, and full configuration polling is one
minute. Link loss now immediately invalidates displayed RPM/state rather than
leaving stale motion telemetry on the page. Active motion also arms ODESC's
internal three-second axis watchdog and feeds it with each browser hold
keepalive. Normal STOP requests IDLE and disables the watchdog. This provides a
controller-local stop if the UART becomes silent before the P4 can send IDLE.

On 2026-08-08, a later negative-direction run latched axis error `64`
(`MOTOR_FAILED`) and motor error `4096` (`CURRENT_LIMIT_VIOLATION`). ODrive
0.5.6 defines this as measured phase current exceeding
`current_lim + current_lim_margin`. The commanded-current limit remained 10 A;
the transient margin was temporarily raised from 8 A to 15 A for diagnosis,
without saving it to controller NVM. This does not authorize more than 10 A
commanded current. The subsequent bounded low-speed request coincided with the
rover power disappearing, so a higher margin must not be treated as the cure
until motor phases, mechanical loading, current sensing, and supply/BMS
integrity are checked. The P4 UI exposes the two values separately, warns about
masking the fault, and decodes this fault by name.

The P4 now also keeps a 256-entry volatile ODESC black box. It records UART
commands/replies and captures axis, motor, controller, and sensorless-estimator
fault values before Clear Errors writes zero. The log is intentionally RAM-only
so abrupt rover shutdown cannot corrupt the microSD; it clears at every reboot.
