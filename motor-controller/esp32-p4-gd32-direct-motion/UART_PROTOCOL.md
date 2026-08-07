# GD32 Direct-Motion UART Protocol

## Transport

- Electrical interface: 3.3 V UART with common ground
- Format: 115200 baud, 8 data bits, no parity, 1 stop bit
- Framing: newline-terminated ASCII
- GD32 port: USART1, PA9 TX and PA10 RX
- Host: ESP32-P4 UART selected by the ESP32-P4 firmware

Only one host may drive GD32 PA10. Keep the onboard CH340 TX output isolated from
that net, and do not connect an Ender USB host while the ESP32-P4 UART is attached.

## Operating modes

The GD32 has two mutually exclusive motion paths:

1. **Direct mode** uses a 50 kHz timer ISR for independent X/Y/Z/E stepping.
   It supports simultaneous continuous RPM targets and simultaneous finite step
   counts.
2. **Planner mode** preserves ordinary Marlin `G0/G1`, `M400`, and tagged
   `M118 DRV_DONE` transactions from the earlier protocol.

Send `M975` before switching from direct mode to planner-mode finite movement.
`G0/G1` is rejected with `DIRECT_MODE_ACTIVE` while direct mode is active.

## Continuous velocity: M970

```text
M970 I<id> X<signed_rpm> Y<signed_rpm> Z<signed_rpm> [E<signed_rpm>]
```

Example:

```text
M970 I1842 X30.000 Y-12.500 Z4.250
```

- The sign selects direction; zero stops that axis.
- Range is -600.000 through +600.000 RPM per axis.
- All axes are generated independently and may move simultaneously.
- Missing axes retain their previous targets. The ESP32-P4 should send X, Y, and
  Z every update so each packet represents complete joystick state.
- Target changes use a fixed 1,200 RPM/s ramp. Direction reversal ramps through
  zero before changing the DIR pin.
- The GD32 replies `VEL_ACK I<id>` and then Marlin's normal `ok`.
- Rejected commands reply `VEL_ERR I<id>`.

Send updates at 60 Hz (one every 16.67 ms). A valid M970 command refreshes a
250 ms deadman timer. If updates stop, the GD32 immediately stops direct motion,
returns to planner mode, and sends:

```text
VEL_TIMEOUT I<last_id>
```

Continue sending `M970 ... X0 Y0 Z0` while the joystick is centered if direct mode
should remain armed. Use `M975` when the host intentionally leaves joystick mode.

At 60 Hz, representative command/ack traffic remains well below the practical
11,520-byte/s payload rate of a 115200-baud 8N1 link. Do not send faster than
60 Hz; newer complete-state packets supersede the need for a higher update rate.

## Simultaneous finite steps: M971 and M972

Configure each axis's positive counted-move speed while direct mode is idle:

```text
M972 X120 Y90 Z30
```

The GD32 replies `SPEED_ACK`; invalid or busy requests return `SPEED_ERR`.
Configured speeds must be greater than zero and at most 600 RPM.

Start a simultaneous counted move:

```text
M971 I55 X3200 Y-1600 Z800
```

- Step counts are signed 32-bit integers and must be nonzero for each included
  axis.
- Each included axis uses its own speed from `M972`.
- Axes start together and each stops when its own exact step count is exhausted.
- The command is rejected while another direct move is active.
- Counted motion does not require 60 Hz refresh packets and has no deadman timeout.

Responses:

```text
COUNT_ACK I55
COUNT_DONE I55 X3200 Y-1600 Z800 E0
```

Errors return `COUNT_ERR I55`. `COUNT_DONE` is emitted only after every included
axis has generated its requested number of STEP pulses.

The original planner-mode finite protocol remains available for compatibility:

```text
M975
G91
G0 Z10.00000 F4800.00
M400
M118 DRV_DONE 56 Z 4000
```

## Fourth driver arming: M973

The E output is ignored by direct-motion commands after boot. Arm it explicitly:

```text
M973 E1
```

Disarm it while direct mode is idle:

```text
M973 E0
```

Responses are `E_ARM_ACK E1`, `E_ARM_ACK E0`, or `E_ARM_ERR`. Although E pulses
are software-disabled by default, all four physical driver enable inputs share
PC3 on the Creality board. Enabling X/Y/Z therefore also electrically enables
the E driver chip; do not interpret software disarming as independent power-off.

## Stop and status

```text
M974
```

returns:

```text
DIRECT_STATUS M<mode> I<id> X<actual_rpm> Y<actual_rpm> Z<actual_rpm> E<actual_rpm> A<e_armed>
```

Mode values are 0 idle, 1 continuous velocity, 2 counted motion, and 3 counted
completion pending. Mode 3 is normally too brief to observe.

```text
M975
```

immediately stops direct pulses, synchronizes Marlin's logical position to the
actual ISR step counters, returns to planner mode, and replies `DIRECT_STOPPED`.
Emergency `M410` also aborts direct motion.

## Heartbeat and switches

The existing link protocol is unchanged. The GD32 sends every two seconds:

```text
HB 42
```

The ESP32-P4 must respond:

```text
M118 HB_ACK 42
```

The GD32 confirms:

```text
HB_ACK_OK 42
```

Debounced switch telemetry remains:

```text
SW X0 Y1 Z0
```

Switch reports are telemetry only and do not automatically stop direct motion.
The ESP32-P4 must decide whether a switch event should produce zero targets,
`M975`, or emergency `M410`.

## ESP32-P4 task contract

Use one UART-owner task and one outbound queue. That task should:

1. Read and frame newline-delimited GD32 responses continuously.
2. Answer every `HB` without waiting for the joystick loop.
3. Send the latest complete X/Y/Z M970 state every 16.67 ms in joystick mode.
4. Increment `I` modulo 32 bits and match `VEL_ACK`, `COUNT_ACK`, and
   `COUNT_DONE` without blocking UART reception.
5. On joystick-task failure, transmit `M975` if possible; rely on the GD32's
   250 ms timeout if the host cannot transmit.
6. Serialize finite automation commands and joystick mode transitions so a
   counted move cannot overlap continuous mode.

Suggested transition sequence:

```text
automation -> joystick: wait for finite completion, then begin M970 updates
joystick -> automation: send zero targets, send M975, wait DIRECT_STOPPED,
                       configure M972, then send M971 or ordinary G0/G1
```
