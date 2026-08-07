# Combined Raspberry Pi 5 Rover UART Shell

[`robot_controller_standalone.py`](robot_controller_standalone.py) combines all
four rover UARTs in one pasteable regular Python 3 program for Thonny. It embeds
the STM32, ODESC, GD32, and current BNO08X drivers and does not require the rover
repository to exist on the Pi. It is not MicroPython: Raspberry Pi OS exposes
hardware UARTs as `/dev/ttyAMA*`, and the program uses `pyserial`.

## Complete UART allocation

| UART | Linux device | Pi TX | Pi RX | Connected device | Baud/protocol |
| --- | --- | --- | --- | --- | --- |
| UART1 | `/dev/ttyAMA1` | GPIO0, pin 27 | GPIO1, pin 28 | STM32 BTS7960 controller | 115200 ASCII |
| UART2 | `/dev/ttyAMA2` | GPIO4, pin 7 | GPIO5, pin 29 | ODESC/ODrive clone | 115200 ODrive ASCII |
| UART3 | `/dev/ttyAMA3` | GPIO8, pin 24 | GPIO9, pin 21 | GD32 stepper board | 115200 Marlin ASCII |
| UART4 | `/dev/ttyAMA4` | GPIO12, pin 32 | GPIO13, pin 33 | BNO080 | 3000000 UART-SHTP |

Every TX connects to the other device's RX. All devices need a common signal
ground and 3.3 V UART logic. Motor-controller power must follow the separate
power documentation; never power a motor bus from a Pi header pin.

`/boot/firmware/config.txt` contains these entries under `[all]`:

```ini
dtoverlay=uart1-pi5
dtoverlay=uart2-pi5
dtoverlay=uart3-pi5
dtoverlay=uart4-pi5
```

After reboot, verify all devices:

```sh
ls -l /dev/ttyAMA1 /dev/ttyAMA2 /dev/ttyAMA3 /dev/ttyAMA4
pinctrl get 0-1 4-5 8-9 12-13
```

## Running in Thonny

Use Thonny's local/system Python 3 interpreter. The only external Python module
is `pyserial`, provided by the already-installed `python3-serial` package.

1. Open [`robot_controller_standalone.py`](robot_controller_standalone.py) on
   the development computer.
2. Paste the complete file into one new Thonny editor tab on the Pi.
3. Run it with Thonny's regular local Python 3 interpreter.

Do not paste the repository-development file `robot_controller.py` on a Pi that
does not have the repository. That modular file intentionally imports the four
drivers by repository path. The standalone file has no `ROVER_REPO` setting and
is the correct direct-paste deployment artifact.

Run the file. It checks each device independently, so one failed UART does not
prevent the other three from connecting. Then use:

```python
links()
print_imu()
magnetometer()
currents()
read_encoders()
encoder_start(50)
encoders()
encoder_stop()
odesc_status()
```

Bounded STM32 motor and GD32 stepper tests:

```python
pulse_motor(1, "A", 6, 0.5)
step("X", 3200, 120, wait=True, timeout=10)
stop_all()
```

The STM32 has a host-command watchdog. The GD32 only accepts finite step moves
through this API. `stop_all()` requests stops from every connected motor
controller, but it is not a replacement for a physical emergency-stop circuit.

STM32 PA0-PA3 analog encoder reports are opt-in. Use `read_encoders()` for one
sample or `encoder_start(50)` for the recommended 50 Hz stream. `encoders()`
returns the latest cached values without adding UART traffic, and
`encoder_stop()` turns the stream off. Other supported rates are 10, 20, 25,
and 100 Hz. The returned state includes `age_ms` and `dropped` diagnostics.

The embedded BNO08X driver performs a UART-SHTP buffer handshake, resets the
sensor's executable/SHTP state, enables fused reports, and only then starts
dynamic calibration. This allows repeated Thonny runs while the BNO080/BNO085
remains powered without retaining incompatible channel sequence state.

## ODESC sensorless commissioning

The installed clone runs custom legacy firmware (`0.0.0 unreleased`) and exposes
sensorless control as axis state `5`. The earlier Start velocity control used
encoder closed-loop state `8` at 0.25 turns/s, so it could not operate this
encoderless setup. A 2026-07-24 read-only diagnostic found:

| Axis | Motor calibrated | Sensorless startup |
| --- | --- | --- |
| 0 | Yes | 4 A, 5.00 turns/s, 1.82 turns/s² |
| 1 | No | Blocked until deliberate motor-only calibration |

Use the main dashboard's open **Legacy sensorless control** section. For axis 0:

1. Raise the driven mechanism and keep a physical motor-power disconnect ready.
2. Select axis 0 and keep Running speed at or above Startup speed, currently
   5 turns/s.
3. To change the volatile current/ramp limits, press **Apply sensorless config**.
   Configuration disarms motion, so apply it before arming.
4. Enter `ARM ODESC TEST`, press **Arm**, then press **Start sensorless**.
5. Press **STOP sensorless** before changing direction. STOP requests IDLE.

The browser sends a keepalive every 250 ms and the server stops the axis after
800 ms without one. Sensorless operation has a minimum speed and cannot hold
zero or reverse while running. Reversing requires STOP followed by a new
startup ramp. Axis 1 requires the separate motor-calibration phrase
`CALIBRATE ODESC MOTOR`; calibration energizes and moves the motor.

Every ODESC dashboard action is committed to `odesc_events` in
`~/rover-controller/rover-history.sqlite3` before execution. Completion,
errors, request values, and the latest axis fault telemetry are then recorded.
The page's **Persistent ODESC action and error history** panel reads
`GET /api/odesc/events`. The same records are printed to the service journal:

```sh
journalctl -u rover-web.service -g odesc-event --no-pager
```

UART status and configuration are validated, but physical sensorless motion is
not yet part of the validated record. The host deadman cannot protect against
Pi power loss, controller firmware lockup, or power-stage faults. An independent
hardware emergency stop remains required.

## Scope

This program is the single-shell commissioning layer. It is useful for wiring,
link, motor, current-sense, stepper, and IMU tests. It is intentionally not the
final autonomous navigation loop; see [`../ARCHITECTURE.md`](../ARCHITECTURE.md).

The standalone artifact is reproducibly assembled from the validated clients by
[`build_standalone.py`](build_standalone.py). Regenerate it whenever any
individual UART driver changes.

## LAN web dashboard

[`rover_web.py`](rover_web.py) imports the standalone controller and serves a
mobile dashboard on port 8080. It controls STM32 drive/current/
encoder functions, finite GD32 moves, live BNO08X telemetry, and guarded ODESC
commissioning actions.

Only one process can own each `/dev/ttyAMA*` device. Stop Thonny's running
controller before starting the web service:

```sh
cd ~/rover-controller
ROVER_WEB_AUTH=0 python3 rover_web.py
```

Then open `http://<pi-address>:8080/` from the Pi or a phone on the same LAN.
The deployed service intentionally has no login. Anyone on that LAN can issue
motor commands, so do not port-forward it, expose it to the internet, or use it
on an untrusted network.

### Tank-drive page

Open `http://<pi-address>:8080/mobile` for the touch controller. The installed
track orientation is:

| Track | STM32 motor | Forward | Reverse |
| --- | ---: | --- | --- |
| Right | 1 | Direction A | Direction B |
| Left | 2 | Direction B | Direction A |

The joystick uses differential mixing. The four full-throttle controls target
forward, backward, and in-place left/right turns while pressed. Targets are
slew-limited by the tank ramp setting on the main page; the default is 80
percent per second and the accepted range is 10-300 percent per second. The
setting persists in `~/rover-controller/rover-web-config.json`.

The browser resends its target every 100 ms. If messages disappear for 500 ms,
the Pi immediately stops both BTS7960 motors rather than completing the normal
deceleration ramp. Releasing a control ramps normally toward zero. `STOP` and
`STOP ALL` are immediate.

### BNO080 sensor and trip page

Open `http://<pi-address>:8080/sensors` for the dedicated BNO080 dashboard. It
shows the fused magnetic heading, corrected robot heading, calibration
accuracy, magnetic field, quaternion, gyro, acceleration, and gravity-removed
linear acceleration.

`Start trip` clears the previous in-memory route and records:

- corrected heading over time;
- east/north velocity and displacement from double-integrated linear
  acceleration;
- integrated path distance and speed; and
- an automatically scaled backtrack plot with start and current markers.

The BNO080 X axis is assumed to point forward and its Y axis to point right.
Enter a heading offset from -180 through 180 degrees if the sensor is rotated
relative to the rover. `Use current IMU heading as robot 0 degrees` calculates
the offset from the current fused heading. The offset persists in
`rover-web-config.json` and can only be changed while a trip is stopped.

The route is an experiment, not odometry. Double integration accumulates
accelerometer bias, vibration, tilt error, and stopping error very quickly.
Heading and turn history are generally more useful than the displayed inertial
distance. Do not use this position estimate for autonomous safety or precise
navigation. Fuse PA0-PA3 wheel encoders and GPS/RTK before relying on position.
Trip points remain in RAM until a new trip starts or the service restarts.

### Battery and Pi monitoring

Open `http://<pi-address>:8080/battery`; `/batter` is also accepted as an alias.
The main page now displays Raspberry Pi CPU temperature, whole-system uptime,
and NetworkManager Wi-Fi strength as a percentage plus a five-bar
weak/fair/good indicator.
The battery page contains:

- ODESC bus voltage, signed `ibus`, bus power, and session watt-hours;
- an approximate 10S lithium-ion voltage-to-SOC conversion;
- voltage/SOC and current/power graphs with a touch sample inspector;
- one selectable history for each Raspberry Pi boot; and
- a rough current estimate from SOC decline over at least five minutes.

The configured pack is 10S4P with 2600 mAh cells, or 10.4 Ah nominal pack
capacity. The OCV curve assumes 3.0-4.2 V per cell. Voltage-based SOC and current
are strongly affected by chemistry, load sag, temperature, cell imbalance, and
charging. They must not replace a calibrated current sensor or battery
management system. ODESC `ibus` is direct controller telemetry but excludes Pi
and accessory loads that do not pass through the ODESC measurement.

Battery data is committed after each successful ODESC refresh, normally every
five seconds, to:

```text
~/rover-controller/rover-history.sqlite3
```

The database uses SQLite WAL journaling and `synchronous=FULL`. Each row is
committed separately, so abruptly removing power should lose at most the sample
being written. Sessions are keyed by the Linux boot ID; restarting only the web
service continues the same power-on history.

### Validated web deployment

This deployment was checked on the rover Raspberry Pi 5 on 2026-07-23 and
extended on 2026-07-24:

| Item | Validated result |
| --- | --- |
| Main dashboard | `http://192.168.1.190:8080/` |
| Mobile tank page | `http://192.168.1.190:8080/mobile` |
| IMU sensor page | `http://192.168.1.190:8080/sensors` |
| Battery history page | `http://192.168.1.190:8080/battery` |
| Installed application | `/home/aditya/rover-controller/rover_web.py` |
| Service | `rover-web.service` active and enabled at boot |
| Service stability | Running with zero automatic crash restarts after deployment |
| STM32 link | Heartbeat alive and `MSTAT` telemetry received |
| Other UARTs | ODESC, GD32, and BNO080 all reported connected |
| ODESC battery reading | Live `vbus_voltage` observed at 35.12 V |
| GD32 web controls | Separate X, Y, and Z panels; zero pending moves after deployment |
| Safe API test | Zero tank target and immediate stop commands acknowledged |
| ODESC telemetry | Live voltage, `ibus`, watts, Iq, velocity, position, calibration, and errors |
| IMU trip API | Start/record/stop validated while stationary; fused heading and samples received |
| Battery persistence | Per-boot SQLite session and live ODESC samples validated |
| Pi monitoring | CPU temperature and whole-system uptime available from the main status API |
| Router Wi-Fi | `netplan-wlan0-mojo` active on `wlan0`; 48-49% signal during validation |
| AP helper | Restricted `status` command validated; mode switching not triggered remotely |

The address is the Pi's current LAN address and can change if DHCP assigns a
different address. Use `hostname -I` on the Pi if the URLs stop responding.

The service file is [`rover-web.service`](rover-web.service). Install or update
the deployment with:

```sh
sudo cp rover-web.service /etc/systemd/system/rover-web.service
sudo systemctl daemon-reload
sudo systemctl enable --now rover-web.service
```

The live service owns all four UARTs. Do not run the Thonny combined controller
at the same time. Useful service checks are:

```sh
systemctl is-active rover-web.service
systemctl is-enabled rover-web.service
systemctl status rover-web.service --no-pager
```

The deployment check intentionally sent no nonzero motor output. Track
orientation comes from the physical direction tests recorded below. Perform
the first joystick maneuver with the tracks raised and a physical motor-power
disconnect available.

### ODESC voltage and GD32 axis controls

The dashboard displays the ODESC `vbus_voltage` value as the battery bus
voltage on the main, mobile, and battery pages. The server refreshes it over
UART every five seconds and displays the sample age. The battery page also
shows an explicitly approximate SOC based on the configured 10S pack; calibrate
the curve against the actual cells and BMS before treating it as a charge
gauge.

The GD32 dashboard provides separate signed-step and RPM inputs for X, Y, and Z;
the extruder axis is intentionally omitted. The host can accept and track
multiple queued transactions, but the current protocol appends `M400` to each
move. Separate X/Y/Z submissions therefore execute sequentially, not as three
independent simultaneous velocity commands. `Stop all steppers` sends the GD32
quick-stop command and clears pending host transactions.

The 2026-07-24 ODESC diagnostic read found both axes idle with no error flags.
Axis 0 reported motor pre-calibration present but encoder pre-calibration
absent. Axis 1 reported both motor and encoder pre-calibration absent. This
explains why entering closed loop or sending velocity before calibration may
not move a motor.

The commissioning workflow in the dashboard is:

1. Raise the mechanism and prepare a physical motor-power disconnect.
2. Select the ODESC axis and apply conservative current/velocity limits.
3. Enter `CALIBRATE ODESC AXIS` and run calibration. The motor will move.
4. Enter `ARM ODESC TEST`, arm motion, and enable closed-loop control.
5. Start with a low signed velocity such as `0.25` turns/s.
6. Use `Stop axis` or `STOP ALL` before changing wiring or mechanics.

The page resends an active ODESC velocity every 250 ms. The server commands
zero velocity after 800 ms without a keepalive. This is a process-level
deadman, not an independent controller or hardware watchdog: loss of Pi power
or a hard process failure can defeat it. A physical emergency stop remains
required.

ODESC electrical telemetry includes DC bus voltage, signed `ibus`, instantaneous
bus watts, session watt-hours, per-axis Iq, velocity, position, calibration
flags, and axis/motor/encoder/controller errors. Session watt-hours count
positive bus power only and reset when requested or when the service restarts.

### Stepper coordinates and jogging

X, Y, and Z each have a host-side open-loop step position:

- `Reset zero` defines the current physical location as step zero.
- Min/max limits are inclusive and persist in `rover-web-config.json`.
- Completed `DRV_DONE` reports advance the displayed position.
- Queued targets are rejected before they cross a configured limit.
- Hold `-` or `+` to queue one small jog chunk at a time.
- Jog keepalives expire after 600 ms, preventing further chunks.

The current GD32/Marlin protocol finishes each chunk with `M400`, so jogging can
have small acceleration pauses and is not true continuous independent velocity
control. `M410` can stop partway through a chunk; when that occurs, the actual
open-loop position is unknowable and the page requires zero to be reset before
enforcing limits again. The software limits are not physical limit switches.

### Router and field AP modes

The main and mobile pages can schedule a Wi-Fi mode change after a three-second
countdown:

| Mode | Network/address |
| --- | --- |
| Router | Saved NetworkManager profile `netplan-wlan0-mojo` |
| Field AP | Open SSID `robot`, controller at `http://10.42.0.1:8080/` |

The field mobile URL is `http://10.42.0.1:8080/mobile`. When router mode is
disconnected or its reported signal stays below 15% for 20 seconds, the server
schedules AP mode automatically. Returning to router mode gives NetworkManager
45 seconds to reconnect before failover monitoring resumes.

[`rover_wifi_mode.py`](rover_wifi_mode.py) is installed root-owned as
`/usr/local/sbin/rover-wifi-mode`. The web service can run only its fixed
`status`, `ap`, and `router` subcommands through
[`rover-web-wifi.sudoers`](rover-web-wifi.sudoers).

The AP was self-tested on 2026-07-24 without changing the development
computer's Wi-Fi. NetworkManager created `robot` on fixed channel 6 with the
Pi's permanent Wi-Fi MAC, assigned `10.42.0.1/24`, and started dnsmasq on UDP
67 for DHCP and UDP 53 for DNS. The dashboard responded locally at
`http://10.42.0.1:8080/api/status`, after which the helper successfully restored
`netplan-wlan0-mojo` at `192.168.1.190`. This verifies the Pi-side AP, DHCP,
web server, and router recovery. A phone association still depends on its own
Wi-Fi client behavior.

The `robot` AP has no password and the web controller has no login. Anyone
within radio range can access motor controls. Do not use this configuration
around untrusted people or expose either web address through port forwarding.
