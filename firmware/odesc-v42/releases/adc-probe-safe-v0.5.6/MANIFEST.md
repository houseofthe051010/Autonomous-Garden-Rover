# ODESC V4.2 read-only ADC probe

Build configuration:

```text
CONFIG_BOARD_VERSION=v3.6-56V
CONFIG_DEBUG=false
CONFIG_DOCTEST=false
CONFIG_USE_LTO=false
CONFIG_ADC_DIAGNOSTIC=true
```

Artifacts:

| File | SHA-256 |
|---|---|
| `ODESC-V4.2-v0.5.6-adc-probe-safe.bin` | `41159ba89bd0903b2369876dc8c3111c827ce5502b963af968232bd3882867e0` |
| `ODESC-V4.2-v0.5.6-adc-probe-safe.hex` | `3298e5f8c465b91c94e224a2cc207dbf0213281584571052dd7b0a43532b8d33` |
| `ODESC-V4.2-v0.5.6-adc-probe-safe.elf` | `ab0bb2c7c1477641a1124ee4ca504a9196b070049a47291fdbe939d6507542ba` |

ELF size:

```text
text    253632
data      2876
bss     143344
```

The linked `rtos_main()` was disassembled after the build. Its only calls are
USB initialization, the general ADC1 DMA scanner, USB communication startup,
the safety-state checker, the hard shutdown function, and `osDelay(1)`. It has
no calls to gate-driver setup, motor setup, motor/brake PWM startup, or an axis
state-machine thread.

The source tree was returned to `CONFIG_ADC_DIAGNOSTIC=false` after archiving
this image. Its normal production HEX rebuilt to SHA-256
`8348a31b722ac61204d32ae4105f08b6a271b833838cd65cbecca848916ab925`,
which exactly matches the preserved working sensorless rollback HEX.
