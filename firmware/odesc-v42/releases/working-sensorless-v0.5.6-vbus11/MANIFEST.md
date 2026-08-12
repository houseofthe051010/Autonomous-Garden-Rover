# Working sensorless rollback image

This release is the immutable rollback point for the single-axis ODESC V4.2
that was working in sensorless mode before hall-sensor development.

Do not replace these files with later builds, even if they use the same ODrive
version string.

## Identity

- Upstream ODrive tag: `fw-v0.5.6`
- Upstream commit: `a308314ed2ca613164b81e7bbdfacc53cd1859ff`
- Build target: `v3.6-56V`
- Board patch: `VBUS_S_DIVIDER_RATIO=11.0f`
- Firmware-reported version: `0.5.6`, unreleased/custom
- Physical output: M0/Axis 0 only

## Immutable artifacts

| File | SHA-256 |
| --- | --- |
| `ODESC-V4.2-v0.5.6-56V-vbus11-working-sensorless.hex` | `8348a31b722ac61204d32ae4105f08b6a271b833838cd65cbecca848916ab925` |
| `ODESC-V4.2-v0.5.6-56V-vbus11-working-sensorless.elf` | `b9af977dfeae2bdc708d3c1abf4d51c0fc7593b66d72c52c16d9518a3bd1adb1` |
| `config-before-custom-vbus11-fw.json` | recorded in `SHA256SUMS` |
| `vendor-config-before-fw-0.5.6.json` | recorded in `SHA256SUMS` |

The HEX is the primary rollback image. The ELF is retained for symbol-level
diagnostics and section comparison.

## Safety state

This image does not imply that the saved device configuration is safe for every
motor or battery. On the tested controller, automatic startup was disabled and
M0 powered up in IDLE. No brake resistor was connected. The validated
sensorless setup used a 10 A motor-current limit, 5 A sensorless startup
current, and `dc_max_negative_current=-2 A`.

Do not enable regenerative braking or increase negative bus-current allowance
without a brake resistor, a battery/BMS that can accept charge current, and a
verified overvoltage shutdown margin.

## Rollback gate

Before rollback, verify the target is the same single-axis ODESC and confirm the
HEX checksum. Flash only under supervision with motor power disabled or the
motor mechanically unloaded. After flashing, restore and inspect configuration
through USB before enabling M0. The detailed procedure is in
`../../HALL-FOC-BRINGUP.md`.

The clean-build comparison is recorded in `BASELINE-REBUILD.md`.
