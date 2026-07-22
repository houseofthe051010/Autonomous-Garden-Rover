"""Raspberry Pi 5 UART client for the STM32 dual-BTS7960 controller.

Run this file with the normal Python 3 interpreter in Thonny. Raspberry Pi OS
does not use MicroPython's ``machine.UART``; Linux exposes UARTs as device files
and this client accesses one with pyserial.

Default Pi 5 UART1 wiring:
    GPIO0 / physical pin 27 / TX -> STM32 PA10 / RX
    GPIO1 / physical pin 28 / RX <- STM32 PA9  / TX
    Pi GND                       <-> STM32 GND

Enable UART1 with ``dtoverlay=uart1-pi5`` before running this file. See the
adjacent README.md for complete setup and safety instructions.
"""

import threading
import time

try:
    import serial
except ImportError as exc:
    raise ImportError(
        "pyserial is required; install Raspberry Pi OS package python3-serial"
    ) from exc


# User configuration. GPIO numbers describe the Device Tree overlay wiring;
# changing them here documents a changed overlay but cannot remap Linux UARTs.
UART_DEVICE = "/dev/ttyAMA1"
UART_BAUD = 115200
UART_TX_GPIO = 0
UART_RX_GPIO = 1

UART_TIMEOUT_S = 0.10
COMMAND_TIMEOUT_S = 1.0
STATUS_INTERVAL_S = 0.40
LINK_TIMEOUT_S = 3.0
PWM_MAX_DUTY = 4095


def _new_motor_state():
    return {
        "direction": "S",
        "duty": 0,
        "r_is_raw": 0,
        "l_is_raw": 0,
        "r_is_mv": 0,
        "l_is_mv": 0,
    }


class STM32MotorController:
    """Thread-safe client for the STM32 newline-delimited ASCII protocol."""

    def __init__(self, device=UART_DEVICE, baudrate=UART_BAUD):
        self.device = str(device)
        self.baudrate = int(baudrate)
        self.port = None

        self._stop_event = threading.Event()
        self._write_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._reply_condition = threading.Condition()
        self._state_lock = threading.Lock()
        self._pending_prefixes = None
        self._pending_reply = None
        self._reader_thread = None
        self._service_thread = None

        self.last_line = ""
        self.last_error = ""
        self.last_seen_monotonic = None
        self.heartbeat_ms = None
        self.heartbeat_count = 0
        self.status_count = 0
        self.watchdog_stopped = False
        self.motors = [_new_motor_state(), _new_motor_state()]

    def start(self):
        """Open the UART and start receive/watchdog-service threads."""
        if self.port is not None and self.port.is_open:
            return self

        self._stop_event.clear()
        self.port = serial.Serial(
            port=self.device,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=UART_TIMEOUT_S,
            write_timeout=COMMAND_TIMEOUT_S,
        )
        self.port.reset_input_buffer()

        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="stm32-uart-reader", daemon=True
        )
        self._service_thread = threading.Thread(
            target=self._service_loop, name="stm32-uart-service", daemon=True
        )
        self._reader_thread.start()
        self._service_thread.start()
        print(
            "STM32 UART opened: {} at {} baud (Pi TX GPIO{}, RX GPIO{})".format(
                self.device, self.baudrate, UART_TX_GPIO, UART_RX_GPIO
            )
        )
        return self

    def close(self):
        """Stop both motors, terminate workers, and close the serial device."""
        if self.port is None:
            return
        try:
            if self.port.is_open:
                self.command("MSTOP ALL", ("OK MSTOP ALL",), 0.5)
        except Exception:
            pass
        self._stop_event.set()
        with self._reply_condition:
            self._reply_condition.notify_all()
        for worker in (self._service_thread, self._reader_thread):
            if worker and worker is not threading.current_thread():
                worker.join(timeout=0.5)
        if self.port.is_open:
            self.port.close()
        self.port = None

    def _write_line(self, line):
        if self.port is None or not self.port.is_open:
            raise RuntimeError("UART is not connected; call connect()")
        payload = (line.strip() + "\n").encode("ascii")
        with self._write_lock:
            self.port.write(payload)
            self.port.flush()

    @staticmethod
    def _matches(line, prefixes):
        return any(line == prefix or line.startswith(prefix + " ") for prefix in prefixes)

    def command(self, line, expected_prefixes=("OK",), timeout_s=COMMAND_TIMEOUT_S):
        """Send one command and wait for its matching non-heartbeat response."""
        if isinstance(expected_prefixes, str):
            expected_prefixes = (expected_prefixes,)
        expected_prefixes = tuple(expected_prefixes)

        with self._command_lock:
            with self._reply_condition:
                self._pending_prefixes = expected_prefixes
                self._pending_reply = None
            self._write_line(line)

            deadline = time.monotonic() + float(timeout_s)
            with self._reply_condition:
                while self._pending_reply is None and not self._stop_event.is_set():
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    self._reply_condition.wait(remaining)
                reply = self._pending_reply
                self._pending_prefixes = None
                self._pending_reply = None

            if reply is None:
                message = "No STM32 reply to {!r}".format(line)
                with self._state_lock:
                    self.last_error = message
                raise TimeoutError(message)
            if reply.startswith("ERR "):
                raise RuntimeError(reply)
            return reply

    def _reader_loop(self):
        while not self._stop_event.is_set():
            try:
                raw = self.port.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", "replace").strip()
                if line:
                    self._handle_line(line)
            except (OSError, serial.SerialException) as exc:
                if not self._stop_event.is_set():
                    with self._state_lock:
                        self.last_error = "UART read failed: {}".format(exc)
                    print(self.last_error)
                    self._stop_event.wait(0.25)

    def _handle_line(self, line):
        now = time.monotonic()
        with self._state_lock:
            was_alive = self._alive_locked(now)
            self.last_line = line

            if line.startswith("HB "):
                fields = line.split()
                if len(fields) == 2:
                    try:
                        self.heartbeat_ms = int(fields[1])
                    except ValueError:
                        self.last_error = "Malformed heartbeat: " + line
                    else:
                        self.heartbeat_count += 1
                        self.last_seen_monotonic = now
            elif line.startswith("READY "):
                self.last_seen_monotonic = now
            elif line.startswith("MSTAT "):
                if self._parse_mstat_locked(line):
                    self.status_count += 1
                    self.last_seen_monotonic = now
                    self.last_error = ""
            elif line.startswith("FAULT "):
                self.watchdog_stopped = True
                self.last_seen_monotonic = now
                self.last_error = line
                for motor in self.motors:
                    motor["direction"] = "S"
                    motor["duty"] = 0
            elif line.startswith(("OK ", "CAPS ")):
                self.last_seen_monotonic = now
                self.last_error = ""

            became_alive = not was_alive and self._alive_locked(now)

        if became_alive:
            print("STM32 link established: " + line)

        with self._reply_condition:
            prefixes = self._pending_prefixes
            if prefixes and (line.startswith("ERR ") or self._matches(line, prefixes)):
                self._pending_reply = line
                self._reply_condition.notify_all()

    def _parse_mstat_locked(self, line):
        fields = line.split()
        if len(fields) != 17 or fields[0:2] != ["MSTAT", "1"] or fields[8] != "2":
            self.last_error = "Malformed MSTAT: " + line
            return False
        try:
            for offset, motor in ((2, self.motors[0]), (9, self.motors[1])):
                motor.update(
                    direction=fields[offset],
                    duty=int(fields[offset + 1]),
                    r_is_raw=int(fields[offset + 2]),
                    l_is_raw=int(fields[offset + 3]),
                    r_is_mv=int(fields[offset + 4]),
                    l_is_mv=int(fields[offset + 5]),
                )
            self.watchdog_stopped = fields[15] == "WD" and int(fields[16]) != 0
        except ValueError:
            self.last_error = "Malformed numeric MSTAT field: " + line
            return False
        return True

    def _alive_locked(self, now=None):
        if self.last_seen_monotonic is None:
            return False
        if now is None:
            now = time.monotonic()
        return now - self.last_seen_monotonic <= LINK_TIMEOUT_S

    def _service_loop(self):
        """Poll telemetry frequently enough to feed the STM32 host watchdog."""
        next_poll = 0.0
        link_was_alive = False
        while not self._stop_event.is_set():
            now = time.monotonic()
            if now >= next_poll:
                next_poll = now + STATUS_INTERVAL_S
                try:
                    self.command("MSTATUS", ("MSTAT",), 0.8)
                except (TimeoutError, RuntimeError, OSError, serial.SerialException):
                    pass

            with self._state_lock:
                alive = self._alive_locked()
            if link_was_alive and not alive:
                print("STM32 link lost; its watchdog should stop both motors")
            link_was_alive = alive
            self._stop_event.wait(0.02)

    def snapshot(self):
        """Return a copy of current link and motor telemetry."""
        with self._state_lock:
            age_ms = (
                None
                if self.last_seen_monotonic is None
                else round((time.monotonic() - self.last_seen_monotonic) * 1000)
            )
            return {
                "alive": self._alive_locked(),
                "age_ms": age_ms,
                "heartbeat_ms": self.heartbeat_ms,
                "heartbeat_count": self.heartbeat_count,
                "status_count": self.status_count,
                "watchdog_stopped": self.watchdog_stopped,
                "last_line": self.last_line,
                "last_error": self.last_error,
                "motors": [dict(motor) for motor in self.motors],
            }


controller = None


def connect(device=UART_DEVICE, baudrate=UART_BAUD):
    """Connect or reconnect using a Linux serial device and baud rate."""
    global controller
    if controller is not None:
        controller.close()
    controller = STM32MotorController(device, baudrate).start()
    return controller


def disconnect():
    global controller
    if controller is not None:
        controller.close()
        controller = None


def _link():
    if controller is None or controller.port is None:
        raise RuntimeError("UART is not connected; call connect()")
    return controller


def ping():
    return _link().command("PING", ("OK PONG",))


def caps():
    return _link().command("CAPS", ("CAPS",))


def drive(motor, direction, duty):
    """Drive motor 1 or 2 in physical direction A or B at duty 0..4095."""
    motor = int(motor)
    direction = str(direction).strip().upper()
    duty = int(duty)
    if motor not in (1, 2):
        raise ValueError("motor must be 1 or 2")
    if direction not in ("A", "B"):
        raise ValueError("direction must be 'A' or 'B'")
    if not 0 <= duty <= PWM_MAX_DUTY:
        raise ValueError("duty must be from 0 to {}".format(PWM_MAX_DUTY))
    return _link().command(
        "MOTOR {} {} {}".format(motor, direction, duty), ("OK MOTOR",)
    )


def drive_percent(motor, direction, percent):
    """Drive using a percentage from 0 through 100."""
    percent = float(percent)
    if not 0.0 <= percent <= 100.0:
        raise ValueError("percent must be from 0 to 100")
    duty = round(percent * PWM_MAX_DUTY / 100.0)
    return drive(motor, direction, duty)


def direction_a(motor, duty=1024):
    return drive(motor, "A", duty)


def direction_b(motor, duty=1024):
    return drive(motor, "B", duty)


def stop_motor(motor):
    motor = int(motor)
    if motor not in (1, 2):
        raise ValueError("motor must be 1 or 2")
    return _link().command("MSTOP {}".format(motor), ("OK MSTOP",))


def stop_all():
    return _link().command("MSTOP ALL", ("OK MSTOP ALL",))


def reset_motors():
    return _link().command("RESET", ("OK RESET",))


def read_currents(motor=None):
    """Refresh and return raw ADC and millivolt readings, not calibrated amps."""
    _link().command("MSTATUS", ("MSTAT",))
    motors = _link().snapshot()["motors"]
    if motor is None:
        return motors
    motor = int(motor)
    if motor not in (1, 2):
        raise ValueError("motor must be 1 or 2")
    return motors[motor - 1]


def status():
    snapshot = _link().snapshot()
    print(
        "STM32 {} | age={} ms | heartbeat={} | watchdog={}".format(
            "alive" if snapshot["alive"] else "missing",
            snapshot["age_ms"],
            snapshot["heartbeat_ms"],
            snapshot["watchdog_stopped"],
        )
    )
    for number, motor in enumerate(snapshot["motors"], 1):
        print(
            "Motor {}: dir={} duty={} R_IS={} raw/{} mV L_IS={} raw/{} mV".format(
                number,
                motor["direction"],
                motor["duty"],
                motor["r_is_raw"],
                motor["r_is_mv"],
                motor["l_is_raw"],
                motor["l_is_mv"],
            )
        )
    if snapshot["last_error"]:
        print("Last error:", snapshot["last_error"])
    return snapshot


def raw(command, expected_prefixes=("OK", "CAPS", "MSTAT")):
    """Send an advanced protocol command and return its response line."""
    return _link().command(str(command), expected_prefixes)


def monitor(seconds=10, interval=0.5):
    """Print live status/current telemetry for a limited number of seconds."""
    deadline = time.monotonic() + float(seconds)
    while time.monotonic() < deadline:
        status()
        time.sleep(float(interval))


def show_help():
    print("Raspberry Pi -> STM32 dual BTS7960 commands:")
    print("  ping() | caps() | status() | read_currents() | monitor(10)")
    print("  direction_a(1, 1024) | direction_b(2, 2048)")
    print("  drive(1, 'A', 4095) | drive_percent(2, 'B', 25)")
    print("  stop_motor(1) | stop_all() | reset_motors()")
    print("  connect('/dev/ttyAMA1', 115200) | disconnect()")
    print("Direction A/B must be identified safely for each motor before renaming.")


try:
    connect()
except (OSError, serial.SerialException) as exc:
    print("STM32 UART not opened: {}".format(exc))
    print("Complete the UART setup, then call connect().")

show_help()
