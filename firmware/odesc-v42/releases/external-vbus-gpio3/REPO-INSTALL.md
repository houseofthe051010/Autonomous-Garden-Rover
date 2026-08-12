# Repository integration

This release is stored at:

```text
firmware/odesc-v42/releases/external-vbus-gpio3/
```

The five source patch parts have been applied to the corresponding files under:

```text
firmware/odesc-v42/source/Firmware/
```

`firmware/odesc-v42/README.md` identifies GPIO3 external VBUS as the prepared
selected control source and retains the prior working BIN as rollback. The
matching ESP32-P4 validity checks are integrated in
`firmware/esp32-p4/main/odesc_link.c`.
