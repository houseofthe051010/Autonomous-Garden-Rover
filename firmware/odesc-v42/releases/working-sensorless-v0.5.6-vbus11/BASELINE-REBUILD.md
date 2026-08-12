# Baseline source reproduction

The vendored source was built from a clean disposable copy on 2026-08-08 before
any Hall work. No Hall or FOC source modification was present.

## Toolchain

- `arm-none-eabi-gcc 13.2.1 20231009`
- `tup 0.7.11-4`
- target from `Firmware/tup.config`: `v3.6-56V`
- only board patch: `VBUS_S_DIVIDER_RATIO=11.0f`

## Flash-byte comparison

The clean-build HEX and archived working HEX were byte-identical:

```text
8348a31b722ac61204d32ae4105f08b6a271b833838cd65cbecca848916ab925  clean/ODriveFirmware.hex
8348a31b722ac61204d32ae4105f08b6a271b833838cd65cbecca848916ab925  working-sensorless.hex
HEX TEXT BYTE MATCH
```

Converting both ELFs to flat load images with
`arm-none-eabi-objcopy -O binary -S` also produced byte-identical binaries:

```text
a3c1f2c93ad9e927c1cacb8af07b29e0fc7fb6a3416a37751d01ec6890e0d0c1  archived-working.bin
a3c1f2c93ad9e927c1cacb8af07b29e0fc7fb6a3416a37751d01ec6890e0d0c1  clean-rebuild.bin
ELF LOAD IMAGE BYTE MATCH
```

Whole ELF hashes can differ because ELF metadata is not necessarily
reproducible. The HEX and extracted load-image comparisons prove that the bytes
written to MCU flash are reproduced exactly.

## Size

```text
   text    data     bss     dec     hex
 315768    3284  143600  462652   70f3c
```

This baseline remains the Hall-capable firmware. Hall inversion is a persisted
encoder configuration mask, not a separate executable image.
