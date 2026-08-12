# ESP32-P4 + ESP32-C6 STM32 web controller

This ESP-IDF firmware runs on the Waveshare ESP32-P4/C6 board. The ESP32-P4
uses the onboard ESP32-C6 through ESP-Hosted over SDIO to provide Wi-Fi.

## STM32 wiring

- ESP32-P4 GPIO21 TX connects to STM32 PA10 RX.
- ESP32-P4 GPIO22 RX connects to STM32 PA9 TX.
- The firmware tests both GPIO21/GPIO22 orientations at boot and keeps the one
  that completes a `PING` / `OK PONG` round trip.
- Both boards must share ground and use 3.3 V UART levels.

## GD32 Ender 3 stepper wiring

- ESP32-P4 GPIO1 and GPIO2 are the two GD32 UART wires.
- At startup, both P4 pins remain inputs. The firmware listens for a valid GD32
  `HB` frame on each wire and makes that pin RX; only then does it enable the
  other pin as TX. This safely recovers either wire orientation without briefly
  connecting two UART outputs together.
- The installed wiring was verified as GPIO2 TX and GPIO1 RX. Passive detection
  remains enabled so both pins are still safe inputs until the heartbeat arrives.
- Both boards must share ground and use 3.3 V UART levels.
- The firmware uses UART2 at 115200 baud, 8N1. UART1 remains dedicated to the
  STM32 motor controller, while UART3 uses GPIO27/GPIO47 for the ODESC.

## BNO080 UART-SHTP wiring

- ESP32-P4 GPIO5 TX connects to BNO080 RX.
- ESP32-P4 GPIO6 RX connects to BNO080 TX.
- Both devices share ground and use 3.3 V UART logic.
- UART4 runs at the BNO080 UART-SHTP fixed rate of 3,000,000 baud, 8N1.
- At boot, GPIO5 and GPIO6 stay as inputs while passive level and framed-data
  checks identify the sensor TX wire. The P4 only enables its TX output after
  orientation is known, preventing two UART outputs from being connected.
- Set the BNO080 interface straps for UART-SHTP before powering or resetting
  the sensor. If the page remains on `passive RX orientation probe`, power-cycle
  the BNO080 so the P4 can observe its startup traffic.

The installed orientation was hardware-verified as GPIO5 TX and GPIO6 RX. The
P4 completed the UART-SHTP Buffer Status round trip with a 256-byte grant and
received rotation vector, calibrated magnetometer, gyroscope, acceleration,
and linear-acceleration reports concurrently with the other rover UARTs.

## Web interface

1. Join the open Wi-Fi network `rover` (no password).
2. Open `http://192.168.4.1/`.
3. Hold a Direction A/B button to run a motor. Releasing it sends stop.
4. `STOP ALL` stops both motors immediately.

The main page also estimates current and electrical power for each BTS7960
motor and the combined drivetrain. It assumes the configured 8 V motor bus,
the BTS7960 nominal `kILIS=8500`, and the common 1 kOhm IS resistor, giving
approximately `I_load = 8.5 * V_IS`. The STM32 averages each IS ADC input and
the P4 uses the larger active half-bridge reading. Clone-module resistor values,
PWM sampling, temperature, and the device's broad low-current tolerance limit
accuracy; calibrate the factor against a trusted inline ammeter before treating
these values as absolute measurements.

Additional pages:

- `/mobile` provides a touch joystick and full-throttle forward, backward,
  left, and right hold buttons. The default ramp is 80 percent per second and
  is adjustable from 10 through 300 percent per second. Releasing a control
  ramps to zero; the red stop button stops immediately.
- `/steppers` controls the GD32 X/Y/Z drivers independently. Each axis provides
  explicit negative, signed, and positive finite moves, RPM, open-loop zero,
  negative/positive limits, switch state, and bounded hold-to-jog chunks.
  The page reports driver enable state and the last queued, completed, or
  rejected command. It also provides simultaneous X/Y/Z continuous signed-RPM
  control and simultaneous exact signed step-count moves.
- `/battery` (`/batter` alias) stores one durable ODESC power history per P4
  boot on the microSD. It provides selectable sessions, voltage/SOC and
  current/power charts, session energy, uptime, internet-synchronized sample
  times, and a clearly labeled five-minute voltage-slope current estimate.
- `/odrive` controls the single physical ODESC M0 output in sensorless mode.
  The P4 verifies motor calibration, `enable_sensorless_mode`, pole pairs,
  startup speed, speed limit, and all relevant fault fields before requesting
  state 8. Hold-to-run commands refresh a 2 second deadman and use ODrive's
  velocity-ramp input mode. While M0 is active, every command also feeds a
  three-second watchdog inside ODESC; normal STOP disables it after requesting
  IDLE. Runtime and persistent controls separately set commanded phase current,
  transient current-fault margin, velocity, and acceleration limits only while
  M0 is IDLE; persistent save
  restarts ODESC and is read back after reconnection. The page reports estimator
  turns/s and RPM, phase Iq, bus current, bus power, FET temperature, motor
  electrical power, FOC voltage-vector magnitude, state, and faults. A sticky
  mobile strip keeps speed, current, temperature, and power visible while the
  hold controls are in use. Six in-memory charts retain the latest 60 seconds
  of speed, bus current, bus power, FET temperature, motor electrical power,
  and FOC voltage through `/api/odrive/history`. Every chart shows its sampled
  average and peak; tapping any chart selects the nearest synchronized point
  and displays every available ODESC measurement at that instant. These live
  charts are diagnostic and are not written to the microSD. The P4 command
  guard is fixed at 7,000 RPM
  (116.67 turns/s). The 80-percent `VBUS * 170 KV` value remains visible as an
  informational headroom estimate, but battery sag no longer changes the limit
  of a held command or causes the browser to issue STOP. ODESC's configured
  speed/current limits and its electrical, thermal, sensorless, and watchdog
  protections remain authoritative. Release commands IDLE and refresh stopped
  telemetry immediately.
  New external-VBUS ODESC firmware is recognized through its validity, fault,
  and status properties. Values above the legacy 36.3 V ADC ceiling are usable
  only when all three checks pass. Legacy firmware retains the original
  clipping lock, and any missing validity reply from new firmware fails closed.
  A 256-entry rolling black box records UART TX/RX, reconnects, deadman actions,
  starts, stops, command-guard rejections, and pre-clear fault snapshots. The
  live trace is held in P4 RAM and exposed at `/api/odrive/blackbox`; the
  important events are also copied to the bounded persistent microSD log
  described below.
- `/mower-logs` lists the durable ODESC mowing sessions stored on microSD.
  It reports boot duration, actual mower-active duration, energy, average and
  peak bus power/current, `Iq`, `Id`, phase-current magnitude, speed, motor
  voltage/power, VBUS, FET temperature, telemetry gaps, and fault count. Any
  boot session can be selected and downloaded as CSV for full-duration
  analysis; this is separate from both the 60-second live charts and the
  five-boot fault/UART event black box.
- `/sensors` shows a live artificial horizon, corrected rover heading,
  roll/pitch, angular rates, quaternion, magnetic field, acceleration, UART
  diagnostics, and calibration state. `Calibrate level` averages stationary
  orientation for 1.2 seconds and stores mounting roll/pitch offsets in NVS.
  `Calibrate forward heading` reserves the drivetrain, gives a three-second
  warning, drives straight for three seconds, and estimates mounting yaw from
  the sensor-frame acceleration vector during a controlled 100-percent/second
  ramp. It rejects stale, weak, incoherent, or turning samples. Handheld and
  normal web drive commands are blocked while calibration
  owns the drivetrain; abort, stale telemetry, and completion all request an
  immediate STM32 stop. Test throttle is restricted to 20-100 percent and
  defaults to 100 percent.

  Forward calibration corrects BNO080-to-rover mounting yaw only. An
  accelerometer cannot determine direction at constant velocity, and this does
  not correct magnetic hard/soft-iron distortion or establish true north. Run
  dynamic magnetometer calibration away from motor phase wires and large steel
  parts, then validate heading in both travel directions. State is available at
  `/api/sensors`; actions are POST requests to
  `/api/sensors/calibration/level`, `/forward?throttle=20..100`, `/abort`,
  `/reset`, and `/heading-offset?degrees=-180..180`. The latest forward attempt
  retains up to 160 full IMU samples in
  P4 RAM. `/api/sensors/calibration/log` downloads a CSV containing timestamps,
  acceptance decisions, attitude, accelerometer, linear acceleration, gyro,
  magnetometer, and the final vector-quality summary. A new attempt or reboot
  replaces this diagnostic trace.
- `/speaker` controls the onboard ES8311 codec and NS4150B amplifier. It shows
  storage capacity and a structured sound library with format, duration, and
  size metadata. Playback has persistent 0-100 codec volume, optional bounded
  +3/+6 dB digital boost, pause, resume, and immediate stop controls. The
  page uploads/deletes supported WAV files, tests the built-in 150 Hz and
  600 Hz tones, and assigns either tone or an uploaded file to handheld
  controller buttons GP10 and GP11. It also lists microphone recordings from
  the microSD and can play or delete them without copying files.
- `/mic` records the board's onboard microphone to a separate recordings
  library. It shows live recording time and byte count, and can play, download,
  or delete each finalized recording.
- `/wifi` scans nearby networks, visibly previews the password exactly as
  typed, saves credentials in ESP-IDF's Wi-Fi NVS, and connects in station
  mode. The AP remains available at `192.168.4.1` during setup and reconnects.
- `/update` accepts the native compiled `esp32_p4_stm32_ap.bin`, writes it to
  the inactive OTA slot, validates it, stops the motors, and reboots.
- `/api/odrive/blackbox` returns the rolling 256-record live UART trace.
  `/api/odrive/blackbox/persistent` returns the CRC-checked 256 KiB microSD
  event ring at `/storage/diagnostics/odesc-p4-blackbox.bin`. Routine telemetry
  is deliberately excluded; boot, link loss/recovery, UART hardware errors,
  motor faults, control actions, and retrieved ODESC records are retained.
  Downloads are limited to the latest five boot sessions, repeated probe
  failures are rate-limited, and the on-disk ring never grows beyond 256 KiB.
  Records are flushed and `fsync`ed individually so a sudden rover power loss
  can at most damage the record being written, not the rest of the ring.

The password preview is generated entirely in the browser. The firmware does
not print the password or return it from a status API. Leading and trailing
spaces are preserved; the preview also displays spaces as middle dots and
shows the exact character count.

The ESP32-P4 enforces an 800 ms motor command lease in addition to the STM32
1.5 second watchdog. Encoder streaming remains off until enabled in the page.
The stepper page preserves the finite Marlin planner path and adds the direct
`M970`/`M971` protocol. The browser refreshes a 400 ms host lease while direct
velocity mode is active; one P4 UART-owner task emits complete X/Y/Z targets at
about 60 Hz. If browser refresh ends, the P4 sends zero targets and `M975`. The
GD32 independently applies a 250 ms UART deadman. Quick stop clears queued host
commands and sends `M410` followed by `M975`.

The ODESC UART driver is torn down immediately after three failed voltage polls
or a four-second reply gap. Reconnect creates a fresh ESP-IDF UART driver,
clears buffered input, sends blank lines to reset the ODrive ASCII parser, and
reprobes `vbus_voltage`. FIFO overflow, driver-buffer-full, frame, parity, and
break counters are exposed by `/api/odrive/status` and written to the persistent
log when nonzero. Numeric property reads retry malformed replies instead of
treating the first noise-corrupted line as valid telemetry.

Continuous velocity makes the P4's open-loop position unknown because the GD32
does not report exact accumulated steps in that mode. Reset zero before using
software limits again. Counted `M971` moves update position only after the
matching `COUNT_DONE` frame arrives.

## Installed tank orientation

The latest physical direction test supersedes the older Raspberry Pi mapping:

| Track motion | STM32 output |
| --- | --- |
| Right forward | Motor 1, direction B |
| Right reverse | Motor 1, direction A |
| Left forward | Motor 2, direction B |
| Left reverse | Motor 2, direction A |

The mobile joystick uses differential mixing and normalizes left/right targets
to the 0-4095 STM32 duty range. Direction changes ramp through zero before the
opposite BTS7960 input is enabled. Firmware maps the left target to Motor 2 and
the right target to Motor 1.

## Handheld controller Wi-Fi handoff

The custom controller firmware in
`/home/aditya/Documents/Claw-Drone/Code/controller/esp32_tx` joins the open
`rover` AP when its right joystick button (GP15) is pressed. GP14 sends a signed
stop/exit packet and disconnects. Its Pico supplies all four axes and the button
mask at 50 Hz. The controller ESP32 forwards a 48-byte `RVR2` packet to
`192.168.4.1:4210`; bytes 24-29 contain signed X/Y/Z RPM, bytes 30-31 contain
the ODESC mower target in 0.1 turns/s, and bytes 32-47 contain the truncated
HMAC-SHA256 tag. Control flag `0x04` requests mower operation. The P4 replies
with a signed 48-byte `RVA2` packet containing measured mower telemetry.

Only device ID 1 can command motion. The left stick controls tank drive. The
right stick maps left/right to X negative/positive and up/down to Y
positive/negative. GP5/GP2 maps to Z positive/negative. Maximum RPM is 50-600;
the controller defaults are X 300, Y 100, and Z 150 RPM:
GP9/GP8 adjust X, GP0/GP1 adjust Y, and GP3/GP4 adjust Z in steps of 5. The P4
renews the existing GD32 400 ms direct-motion lease from each valid packet and
queues `M17` when the first nonzero controller command needs the drivers.
On the normal rover TFT page, GP10 toggles the ODESC mower and GP11 opens the
dedicated mower page. On that page GP10 decreases and GP11 increases the target
by 1.0 turns/s; holding either button accelerates the repeat rate. The touch
Start/Stop control toggles the mower and Back returns to the rover page. The TFT
shows `MOWER STARTED` or `MOWER STOPPED`, target turns/s, estimated sensorless
RPM, ODESC bus current and power, and battery bus voltage. A controller lease
expires after 550 ms, forcing zero velocity and IDLE when packets stop.
The P4 enforces a 200 percent/second minimum drivetrain ramp for authenticated
handheld packets, even if an older controller requests the former 80 percent
rate. This reaches full duty in about 0.5 seconds. Direction reversal still
ramps through zero and immediate stop or lease expiry remains un-ramped. The
browser `/mobile` ramp remains independently adjustable from 10-300 percent.
In the signed `RVA2` acknowledgement, byte 6 reports mower active, connected,
current-valid, fault, and controller-owned flags. Bytes 12-13 are target
deci-turns/s, 14-15 estimated RPM, 16-17 bus centivolts, 18-19 signed bus
centiamps, 20-23 signed centiwatts, byte 24 axis state, and bytes 26-29 the
minimum and maximum deci-turns/s accepted by the ODESC configuration.

The AP is always enabled. In normal operation the P4/C6 can run its AP and a
router station concurrently, subject to the single C6 radio/channel. The P4
pauses its router station only after a packet passes HMAC-SHA256 authentication.
When valid packets stop for 400 ms, the P4 immediately stops both tracks,
sends GD32 `M410`/`M975`, and reconnects the saved router. A phone merely
joining the open AP cannot trigger router takeover.

The controller pauses drone ESP-NOW transmissions as soon as it associates
with `rover`, preventing the same joystick packet from controlling both
vehicles. If `rover` disappears, it restores the drone radio path and retries
every three seconds only while GP15-requested rover mode remains active.

The real 32-byte shared key is held in each project's ignored
`main/rover_control_key.h`. The committed `.example.h` files contain no key.
Generate a replacement and rebuild both devices if either binary or source key
is disclosed.

## OTA flash layout

The one-time wired installation uses a rollback-capable 16 MB layout:

| Partition | Offset | Size |
| --- | ---: | ---: |
| NVS | `0x9000` | 24 KB |
| OTA metadata | `0xF000` | 8 KB |
| `ota_0` | `0x20000` | 4 MB |
| `ota_1` | `0x420000` | 4 MB |
| Internal data storage | `0x820000` | 7.875 MB |

The current PSRAM-enabled application build is 1,211,168 bytes (SHA-256
`b74d0ee7c126bda054f1625457f82fa42952c9e45b4931130ba7bc27f8a13f95`),
leaving 71 percent of either OTA slot free. Large audio, model, map, and log
files should be placed on microSD rather than embedded in the application
image.

## Battery history and abrupt power loss

Battery history is stored under `/storage/battery`. Each physical power session
creates one 512 KiB journal that is fully allocated before sampling begins.
Software, OTA, panic, and watchdog resets resume the active journal through an
NVS pointer; power-on, brownout, and deep-sleep wake create a new journal. A
one-time migration finds and resumes the newest valid journal created by older
firmware that did not store the NVS pointer. New journal headers record the
initial reset reason, while older files appear as legacy/unknown.

Two independently validated 512-byte header sectors describe the journal.
Every five seconds, one sample is written to a 64-byte fixed record with a
sequence number and CRC, followed by `fflush()` and `fsync()`. Missing or stale
ODESC telemetry still creates a durable record, marked unavailable; the API
returns null measurements and the chart breaks the line across the gap instead
of showing stale data or shortening the apparent session. A journal holds 8,176
records, or about 11.4 hours. The file never grows while logging, so normal
samples do not modify the FAT allocation chain. On restart, readers stop at the
first missing or invalid CRC and retain every earlier valid sample. Because
eight compact records share one physical 512-byte SD sector, an interrupted
sector program can discard up to the newest 40 seconds rather than only the
final sample; it does not invalidate completed earlier sectors. The reader
remains compatible with version-1 8 MiB/512-byte-record and version-2 compact
journals already on the card.

The logger reads cached ODESC telemetry rather than issuing extra UART queries.
It records bus voltage, signed `ibus`, bus power, positive session watt-hours,
10S4P voltage SOC, physical-session elapsed time, and Unix time after SNTP
synchronizes through the router. The dashboard reports durable, measured, and
missing-telemetry counts and scales charts by elapsed time rather than sample
index. Elapsed time remains valid without internet; absolute timestamps cannot
be known before synchronization because this board currently has no
battery-backed RTC.

The configured pack is 10S4P with 2.6 Ah cells, or 10.4 Ah nominal. Voltage SOC
and its five-minute derivative are rough estimates affected by load sag,
recovery, chemistry, temperature, imbalance, and charging. ODESC `ibus` is the
better live measurement but excludes the P4 and accessories powered outside
the ODESC current path.

Tapping either battery chart selects the nearest durable sample. The dashboard
then fits voltage and SOC against time by least squares using valid samples up
to five minutes before and after the selection. It reports voltage trend in
V/hour, a SOC-derived current estimate, fit `R-squared`, and point count, and
draws the fitted voltage segment over the chart. A low `R-squared` means the
estimate should be ignored. This calculated current is not a substitute for
ODESC's measured `ibus`, especially while load changes, charging, or voltage
recovery are occurring.

Audio uploads use synced `.part` files and a recoverable `.bak` transaction when
replacing an existing sound. Startup restores the old file or promotes a fully
received new file after an interrupted replacement. Microphone recordings
checkpoint a valid WAV header and sync approximately once per second; startup
repairs an interrupted recording to its last durable sample boundary. Mount
failure never automatically formats the microSD. These measures bound typical
data loss, but software cannot guarantee that FAT or an SD card's internal
flash translation layer survives power removal during a physical program/erase
operation. A hold-up capacitor/supervisor that provides enough time for a
controlled final sync is required for that guarantee.

The 2026-08-09 deployment resumed the pre-update active journal across OTA,
reconstructed its elapsed time from its last synchronized record, and appended
five-second missing-telemetry records while the ODESC UART was offline. The
record count, gap count, and null API measurements advanced correctly without
creating another short session. Previously closed journals remain readable.

## Mower session telemetry

Detailed mower telemetry is stored under `/storage/mower`, independently of
the compact battery-history journal. Every P4 boot creates one preallocated
8 MiB `MOW_XXXXXXXX.jrn` file. It records at 1 Hz for 65,528 samples, or about
18.2 hours. Each 128-byte record has a sequence number and CRC and is followed
by `fflush()` and `fsync()`. The file is allocated before logging starts, so
normal samples do not extend its FAT allocation chain. After an abrupt power
cut, readers retain the valid record prefix and reject an incomplete final
record.

Each record contains P4 uptime and synchronized Unix time when available,
cumulative mower-active time and positive bus energy, link/validity/activity
flags, ODESC state and all error words, VBUS, signed bus current, bus power,
commanded turns/s, estimated RPM, measured and requested `Iq` and `Id`, phase
current magnitude, motor voltage, motor electrical power, and FET temperature.
Missing or stale telemetry is written as a marked gap rather than repeating an
old value.

Open `/mower-logs` to select a boot session, inspect its aggregate values, and
download the CRC-validated records as CSV. The summary averages current and
power over valid active samples and reports average/peak VBUS, bus current and
power, `Iq`, `Id`, phase-current magnitude, absolute RPM, motor voltage/power,
and FET temperature; accumulated Wh integrates positive bus power while the
mower is active. These are 1 Hz UART samples, so a short hardware overcurrent
can appear only in the recorded fault words and need not equal the peak sampled
current. Download completed sessions after mowing when practical because
exporting a long current session temporarily owns the shared microSD interface.

## Speaker and sound storage

The board audio path uses I2C GPIO7/GPIO8, I2S GPIO9-GPIO13, and active-high
amplifier enable GPIO53. These are onboard signals and must not be assigned to
external rover devices. Playback uses the ES8311 codec at volume 100 and moves
WAV stream buffers into PSRAM so audio cannot overflow the FreeRTOS task stack.

Uploads must be uncompressed 16-bit PCM WAV, mono or stereo, with a sample rate
from 8 kHz through 48 kHz. Filenames may contain letters, numbers, dots,
underscores, and hyphens. The HTTP upload limit is 64 MB per file.

Uploaded playback assets are stored under `/storage/sounds`. Microphone captures
are stored under `/storage/recordings` as 16 kHz, 16-bit stereo PCM WAV files,
at roughly 64 KB/s, with a ten-minute per-file guard. Capture first writes a
`.part` file and publishes the `.wav` only after the data length and RIFF header
have been finalized. Playback and capture share one codec mutex, while SD access
is locked only for individual reads and writes; status and stop requests remain
responsive during long songs.

Codec volume 100 is the ES8311's maximum normal output setting. Optional
digital boost multiplies PCM samples by approximately 1.41 (+3 dB) or 2.0
(+6 dB) and saturates values outside the signed 16-bit range. It improves quiet
material but can add clipping distortion and does not increase the NS4150B
amplifier's electrical power. Volume and boost are persisted in NVS.

`/mic` exposes the ES8311's 0-42 dB hardware microphone gain in supported 6 dB
steps and stores the selected value in NVS. The default is 24 dB. Higher gain is
useful behind a case but also raises mechanical/electrical noise and can clip;
compare recordings at 24 and 30 dB before going higher. Recording creation is
non-destructive: if `recording_01.wav` exists, another request with that name is
saved as `recording_02.wav`, then `recording_03.wav`, and so on. All finalized
recordings remain on the mounted microSD `/storage/recordings` directory until
deleted from `/mic`.

Gain selection saves immediately when changed. The status beside the selector
shows `Saved at XX dB`; there is no separate save button for the periodic status
refresh to interfere with.

Periodic speaker/microphone status polling updates telemetry and file lists but
does not overwrite locally edited volume, boost, GP10/GP11 assignment, recording
name, or active gain controls. A pending speaker edit is synchronized back to
device state only after its save request succeeds.

Use a consistent asset naming scheme as the voice interface grows:

| Prefix | Purpose | Example |
| --- | --- | --- |
| `music_` | Music and longer audio | `music_interstellar.wav` |
| `response_` | Prerecorded spoken responses | `response_status_complete.wav` |
| `effect_` | Alerts and short effects | `effect_warning.wav` |
| `recording_` | Onboard microphone tests | `recording_01.wav` |

### Planned voice-command pipeline

The microphone page validates capture hardware and provides training/test
samples. A production voice path should use this bounded pipeline:

1. Capture microphone frames into a short ring buffer and apply voice activity
   detection so silence does not invoke recognition.
2. Recognize an allowlisted command or intent locally when the vocabulary is
   small; send audio to a more capable host/API only when free-form language is
   required.
3. Dispatch the recognized intent through the same safety-checked motor and
   sensor APIs used by the web interface. Speech must not bypass motor leases,
   range limits, watchdogs, or emergency stop.
4. Build the truthful response from current rover state, then select a
   prerecorded `response_*.wav` asset or synthesize speech. For example,
   `status_today` should only say mowing or watering completed when the task log
   confirms it.
5. Record the transcript, matched intent, action result, and response asset in
   a bounded event log on microSD.

A practical first implementation on this ESP32-P4 is Espressif ESP-SR: feed
16 kHz, 16-bit mono frames from the codec into its AFE, enable noise suppression,
automatic gain control, voice activity detection and WakeNet, then send active
speech frames to the English MultiNet 7 command recognizer. MultiNet recognizes
an allowlist of up to 300 commands offline; it is command recognition, not
general speech-to-text. Espressif reports approximately 18 KB internal RAM,
2.92 MB PSRAM and 8 ms processing per 32 ms frame for MultiNet 7 on ESP32-P4.
The official references are the [ESP-SR ESP32-P4 guide](https://docs.espressif.com/projects/esp-sr/en/latest/esp32p4/getting_started/readme.html),
[AFE guide](https://docs.espressif.com/projects/esp-sr/en/latest/esp32p4/audio_front_end/README.html),
and [ESP32-P4 benchmarks](https://docs.espressif.com/projects/esp-sr/en/latest/esp32p4/benchmark/README.html).

A future `/storage/voice/commands.json` manifest can map command IDs, intent
names, accepted phrase variants, and response assets without changing rover
control code. Example intents are `STATUS_TODAY`, `PLAY_MUSIC`, `STOP_AUDIO`,
and `EMERGENCY_STOP`. Keeping recognition, command execution, and response
playback as separate stages allows local MultiNet to be replaced or supplemented
by a network speech-to-text service without changing rover safety logic. Use an
external Whisper-class service when arbitrary questions or complete transcripts
are required; the ESP32 should stream only speech segments selected by VAD, then
receive text or a compact intent response.

At the 2026-08-01 hardware check, the internal FAT partition reported
8,155,136 bytes total and 8,151,040 bytes free when empty (about 7.77 MiB
usable). A mono 16 kHz/16-bit file consumes about 32,000 bytes per second, so
the empty internal partition holds roughly 4.2 minutes in that format.

The TF card path uses SDMMC slot 0 on GPIO39-GPIO44 and ESP32-P4 LDO channel 4.
The firmware safely falls back to internal flash when mounting fails and shows
that warning on `/speaker`; it never automatically formats removable storage.
On 2026-08-04, the nominal 8 GB card was checked on Linux, passed a 32 MiB
write/read checksum, was reformatted as a single FAT32 volume labeled
`ROVER_SD`, passed the post-format filesystem check, and reported 7.4 GiB
usable. Reinsert it and reboot the P4; `/speaker` should report `microSD`. If it
still fails, diagnose the P4 SDMMC electrical/initialization path rather than
formatting the card again.

This board has **16 MB NOR flash** and **32 MB stacked PSRAM**. The PSRAM is
enabled at 200 MHz using the vendor board-check settings. `/update` reports
total and free PSRAM so model-memory requirements can be checked before loading
a model from microSD. Model data can be loaded into PSRAM; native executable
firmware still boots from one of the 4 MB OTA partitions in the 16 MB NOR flash.

Future application updates only need the application binary:

```text
build/esp32_p4_stm32_ap.bin
```

Do not upload the bootloader or partition-table binaries through `/update`.
Partition-table and bootloader changes still require a wired flash.

## Network security

The setup AP and P4 HTTP server are open and are not administrator-authenticated
or encrypted. The web form keeps credentials out of source control and
diagnostic logs, but another device already on either network can access the
P4 HTTP interface. Handheld UDP drive packets are separately authenticated,
replay-checked, and deadman-protected. Router provisioning to the authenticated
handheld uses a signed request and AES-256-GCM encrypted response; the password
is never sent as plaintext over the open AP. Do not expose port 80 to the public
internet. Add HTTPS, signed firmware, and administrator authentication before
field deployment.

## Verified hardware result

On 2026-07-31 this image was flashed to the connected ESP32-P4 revision 3.1.
ESP-Hosted detected the onboard ESP32-C6 over four-bit SDIO, started the AP and
DHCP server at `192.168.4.1`, and verified the STM32 link on GPIO21 TX/GPIO22 RX.
Continuous `HB` and `MSTAT` records were received without UART errors.

Earlier powered-GD32 testing detected P4 GPIO27 TX and GPIO47 RX. Heartbeat and
`HB_ACK_OK` counters remained synchronized
without reconnecting. X, Y, and Z each completed separate +16 and -16 microstep
transactions and returned to software zero. A +1001 X request was also rejected
without being queued when the configured range was -1000 to +1000.

The GD32 was then updated with `GD3P4V1.BIN`. The deployed P4 received a valid
`DIRECT_STATUS` response to `M974`, streamed 25 zero-RPM `M970` updates with
matching `VEL_ACK` responses, and returned to direct mode 0 after its browser
lease expired. This validates firmware detection, the 60 Hz host stream, UART
deadman path, and stop transition without moving a motor. Physical direction,
simultaneous nonzero motion, and `COUNT_DONE` still require a guarded motor test.

The serial boot log also verified that the new bootloader selected `ota_0`,
both OTA slots are 4 MB, and the internal storage partition is 7.875 MB.

The onboard C6 currently reports coprocessor firmware version `0.0.0`, while
the host component is version 2.12.0. Wi-Fi initialization succeeds, but update
the C6 coprocessor firmware if ESP-Hosted later reports RPC timeouts.

The `RVR2` controller build was OTA-deployed on 2026-07-31 at router address
`192.168.1.201`. After reboot it reported partition `ota_1`, AP SSID `rover`,
router connection active, and controller inactive as expected before GP15 is
pressed. The controller ESP32 was built and flashed over `/dev/ttyUSB0`; serial
logs showed continuous valid Pico input/status frames with no malformed UART
frames. A centered GP15/GP14 round-trip then produced 78 accepted `RVR2`
packets, zero rejected packets, 45 acknowledged zero-RPM direct updates, a GD32
`M410`/`M975` stop, and automatic router reconnection. Nonzero motor/stepper
direction still requires a guarded test with the rover raised and a physical
power cutoff.

On 2026-08-01, the speaker image was OTA-deployed and the handheld-controller
ESP32 was flashed over `/dev/ttyUSB0`. Live tests verified percent-decoded tone
commands, both default assignments, ES8311 playback state, a generated 1-second
mono 16 kHz PCM WAV upload/play/delete cycle, responsive HTTP status during
streaming, 7.77 MiB internal capacity reporting, and an uninterrupted STM32
heartbeat. The attached speaker's audible level and the physical GP10/GP11
buttons still require an operator check.

On 2026-08-02, the encrypted handheld router-provisioning response was added
and OTA-deployed to `192.168.1.201`. The P4 rebooted from `ota_1` to `ota_0`,
rejoined the saved router, kept AP `rover` on channel 6, and retained a healthy
STM32 heartbeat. The matching controller image was installed with a 4 MB
dual-OTA partition map. A physical first-page GP15 provisioning action followed
by second-page GP15 router connection is still required to validate the complete
over-air controller update path.

On 2026-08-04, the full-duplex audio image was OTA-deployed to `ota_1` at
`192.168.1.201`. During a three-minute WAV stream, the sound remained visible
in the library and pause, resume, and stop all changed live state correctly. A
3.136-second onboard-microphone capture finalized to a valid 200,748-byte,
16 kHz, 16-bit stereo PCM WAV. Download analysis measured non-silent audio at
-35.5 dB mean and -2.4 dB peak; recording playback and deletion also succeeded.
All main pages returned HTTP 200 and the STM32 heartbeat remained healthy.

The subsequent audio/controller image added persistent output volume, bounded
digital boost, shared `/speaker` access to `/storage/recordings`, and a
handheld-only 200 percent/second minimum ramp. It was OTA-deployed and verified
with the installed microSD: `recording_01.wav` appeared in `/speaker`, played
through the recording source at volume 100 with +3 dB boost, stopped normally,
and did not disturb the STM32 heartbeat. The deployed output setting is volume
100 with +3 dB boost; it can be changed or disabled from `/speaker`.

On 2026-08-11, the external-VBUS-aware image was OTA-deployed to `ota_0` at
`192.168.1.201`. The P4 detected ODESC serial `357F356D3135` on GPIO27 TX and
GPIO47 RX, reported the external VBUS source supported, valid, fault-free, and
status zero, and retained the selected reading without applying the legacy
36.3 V clipping lock. A 12-sample observation completed with zero UART command
failures, FIFO overflows, frame errors, or parity errors. M0, both BTS7960
motors, and all command targets remained stopped. Validation above 36.3 V and
the physical external-sense disconnect test remain pending.

## Build

Use ESP-IDF 6.0 or later. Component Manager downloads `esp_hosted`,
`esp_wifi_remote`, and `esp_codec_dev` from Espressif's component registry.

```bash
idf.py set-target esp32p4
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```
