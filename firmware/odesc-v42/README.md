# Flipsky/SEQURE ODESC V4.2 firmware

This directory contains the editable firmware for the rover's single-axis
ODESC V4.2. The modified upstream source is vendored under `source/`; the exact
upstream revision, rollback images, and board-specific changes are retained so
the firmware remains independently reproducible.

## Source identity

- Upstream: <https://github.com/odriverobotics/ODrive>
- Tag: `fw-v0.5.6`
- Commit: `a308314ed2ca613164b81e7bbdfacc53cd1859ff`
- Build target: `v3.6-56V`
- Local patch: `patches/0001-odesc-v42-56v-vbus-divider.patch`

The original board patch changes `VBUS_S_DIVIDER_RATIO` from `19.0f` to `11.0f` for the
56 V build. This reproduced the factory reading near 36.27 V, but later testing
with a fully charged 10S pack exposed that 36.291138 V is exactly ADC full
scale (`4095 * 3.3 / 4096 * 11`). The measurement clips above approximately
36.3 V. Do not treat this build as providing valid 42 V bus measurement or
overvoltage protection. The ESP32-P4 now interlocks ODESC motion whenever this
condition is detected. Correct repair requires tracing/changing the physical
divider and then matching the firmware ratio; increasing the software constant
alone cannot recover a saturated ADC signal. The prepared GPIO3 external-VBUS
release addresses this with a separately measured divider and a fail-closed
selector; see `releases/external-vbus-gpio3/README.md`.

The flashed firmware identifies as `0.5.6` with the unreleased flag set. This
distinguishes it from an untouched official release.

## Edit the saved source

The directly editable tree is `source/`. Build it without downloading anything:

```bash
cd firmware/odesc-v42/source/Firmware
tup init
tup
```

Generated firmware appears under `source/Firmware/build/`. That directory is
generated locally and can be deleted and rebuilt. The tested HEX and ELF are
retained separately under `diagnostics/odesc`.

The external-VBUS tree was rebuilt successfully on 2026-08-11. The resulting
ELF uses 320,904 bytes of code/constant data, 3,296 bytes of initialized data,
and 143,776 bytes of BSS according to `arm-none-eabi-size`. Its BIN SHA-256 is
`eb7165f266e0a48aa865b952223da0386aa873284c2beade1fb9c3b4f2f0a7f1`.
This is a prepared bench-validation image and is not yet the flashed field
image.

## External GPIO3 VBUS source

The current editable source selects a calibrated 100 kOhm / 5.6 kOhm divider
on GPIO3 after startup and plausibility validation. It retains the original
injected-ADC conversion as read-only USB and UART telemetry, compares it to the
external input where the onboard ADC is still trustworthy, and latches
`EXTERNAL_VBUS_INVALID` if the external source fails. FOC, modulation, power,
voltage trips, brake calculations, CAN telemetry, and `vbus_voltage` all use
the validated selected value.

The exact image, rollback BIN, source patches, hashes, selector tests, wiring,
USB/UART commands, and supervised test sequence are in
`releases/external-vbus-gpio3/`. Do not flash or run this release first at
42 V; validate it at the present approximately 36.2 V condition with M0 idle.

## Build on Ubuntu

Install the ARM toolchain and Tup:

```bash
sudo apt install git git-lfs tup gcc-arm-none-eabi
```

Fetch the exact source and apply the hardware patch:

```bash
git clone --recurse-submodules https://github.com/odriverobotics/ODrive.git
cd ODrive
git checkout a308314ed2ca613164b81e7bbdfacc53cd1859ff
git submodule update --init --recursive
git apply /path/to/Autonomous-Garden-Rover/firmware/odesc-v42/patches/0001-odesc-v42-56v-vbus-divider.patch
```

Create `Firmware/tup.config`:

```text
CONFIG_BOARD_VERSION=v3.6-56V
CONFIG_DEBUG=false
CONFIG_DOCTEST=false
CONFIG_USE_LTO=false
```

Build:

```bash
cd Firmware
tup init
tup
```

Outputs:

- `Firmware/build/ODriveFirmware.elf`
- `Firmware/build/ODriveFirmware.hex`
- `Firmware/build/ODriveFirmware.bin`

## Flashed artifacts

The currently deployed and motor-tested external-VBUS image is in
`releases/external-vbus-gpio3-stable-20260811/`. Its README records the exact
hashes, the GPIO3 calibration, the clone-specific armed offset-hold fix, and
USB plus ESP32-P4 motion verification. Use that release for the present board.

The exact tested outputs are retained under `diagnostics/odesc`:

- `ODESC-V4.2-v0.5.6-56V-vbus11-custom.hex`
- `ODESC-V4.2-v0.5.6-56V-vbus11-custom.elf`
- `ODESC-V4.2-v0.5.6-56V-vbus11-custom.md`

Do not replace those artifacts merely because a rebuilt file has a different
hash. Toolchain-version differences can change output bytes. Validate a new
build on the bench before deploying it.

## Hardware scope

The physical board has one motor power stage, M0. Use only `axis0`. The current
firmware is still based on ODrive's dual-axis v3 source; it is not yet a true
single-axis board port. Future board-level work should define an ODESC V4.2
target that compiles out the unused second motor-control path after the exact
STM32 pin routing is traced and verified.

The validated M0 sensorless parameters and measurements are recorded in
`diagnostics/odesc/ODESC-V4.2-v0.5.6-56V-vbus11-custom.md`.

## Read-only ADC identification image

The separate image under `releases/adc-probe-safe-v0.5.6/` scans ADC1 channels
0 through 15 while keeping the DRV8301, motor PWM, and AUX/brake output disabled.
It is not drive firmware and must not be used with motor phases attached. See
[ADC-PROBE-SAFETY.md](ADC-PROBE-SAFETY.md) for the shutdown design, required
bench wiring, image hash, and USB reader command.

## Hall-sensored FOC

The saved ODrive 0.5.6 source already supports one inverted Hall channel with
the persisted `axis0.encoder.config.hall_polarity` bit mask. No FOC source patch
is needed, and no hard-coded orange-wire inversion should be added. The exact
working sensorless image and configuration captures are preserved under
`releases/working-sensorless-v0.5.6-vbus11/`.

Use [HALL-FOC-BRINGUP.md](HALL-FOC-BRINGUP.md) for the gated, supervised setup.
The read-first host tool is `tools/odesc_hall_setup.py`; its inversion analyzer
has tests in `tests/test_hall_analysis.py`. The tool never performs motor
calibration, and all configuration-changing commands are dry-run by default.

## ESP32-P4 UART link

The ODESC UART is configured for 115200 baud, 8N1, ASCII plus stdout:

- ODESC GPIO1: UART TX
- ODESC GPIO2: UART RX
- ESP32-P4 GPIO27: UART TX to ODESC GPIO2
- ESP32-P4 GPIO47: UART RX from ODESC GPIO1
- Both controllers must share signal ground

The deployed P4 uses this fixed, validated orientation and probes it with a
read-only `vbus_voltage` query. Its sensorless control is the ODrive 0.5.6
state-8 closed-loop path with `axis0.config.enable_sensorless_mode=1`; state 5
is invalid in this source revision. Before motion, the P4 checks calibration,
the sensorless flag, pole pairs, ramp speed, velocity limit, and axis/motor/
estimator errors. It rejects below-ramp speed and live direction reversal.

The 2026-08-08 P4 test commanded 9.1 turns/s. Sensorless telemetry converged to
about 9.25 turns/s (555 RPM), phase Iq remained approximately 0.1-1.3 A, and all
fault fields stayed zero. STOP returned M0 to state 1 with zero estimated RPM.
The P4 link then remained connected for twelve consecutive post-stop checks
with zero UART failures.

## UART blackbox diagnostic image

The prepared diagnostic image reserves STM32F405 flash sector 9
(`0x080A0000..0x080BFFFF`, 128 KiB) by reducing the linker `FLASH` region from
768 KiB to 640 KiB. The application ends below `0x0804F000`, so it cannot
overlap the reserved sector. Sectors 10 and 11 remain exclusively owned by the
existing ODrive configuration NVM implementation.

The ODESC appends 64-byte CRC-checked records for boot, UART HAL errors, receive
DMA restarts, and a 1.5-second UART silence while M0 is armed. A record captures
UART status/error state, DMA position, byte/restart counters, M0 axis/motor/
sensorless errors, state, and estimated velocity. The validity word is written
last so interrupted writes are rejected. The logger never erases sector 9 and
never programs flash while any axis is armed or outside IDLE. Capacity is 2048
events; a full/non-erased sector disables further writes rather than erasing at
runtime.

ASCII diagnostic commands are `ds` (summary) and `da N`, `db N`, `dc N`,
`dd N` (four compact chunks of record N). The P4 polls `ds` only while M0 is
idle, retrieves up to eight new records per minute, and copies them into its
microSD persistent ring. This permits post-reboot retrieval through
`/api/odrive/blackbox/persistent` without reconnecting USB to the ODESC.

The exact firmware from before this instrumentation is retained under
`images/pre-uart-blackbox-2026-08-10/`; its BIN SHA-256 is
`a3c1f2c93ad9e927c1cacb8af07b29e0fc7fb6a3416a37751d01ec6890e0d0c1`.
