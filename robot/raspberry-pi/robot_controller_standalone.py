"""Standalone four-UART Raspberry Pi 5 rover controller.

Generated from the repository's validated UART clients. This file can be
pasted directly into Thonny and has no rover-repository path dependency.
"""

import atexit
import threading
import time
import types

STM32_PORT = "/dev/ttyAMA1"
ODESC_PORT = "/dev/ttyAMA2"
GD32_PORT = "/dev/ttyAMA3"
BNO080_PORT = "/dev/ttyAMA4"
ASCII_BAUD = 115200
BNO080_BAUD = 3_000_000
ODESC_MAX_TEST_TURNS_S = 2.0
ODESC_MAX_TEST_SECONDS = 5.0
ODESC_MAX_CONTROL_TURNS_S = 15.0
ODESC_ARM_PHRASE = "ARM ODESC TEST"
ODESC_CALIBRATE_PHRASE = "CALIBRATE ODESC AXIS"
ODESC_MOTOR_CALIBRATE_PHRASE = "CALIBRATE ODESC MOTOR"

def _embedded_module(name, source):
    module = types.ModuleType(name)
    module.__dict__['__name__'] = name
    exec(compile(source, '<embedded ' + name + '>', 'exec'), module.__dict__)
    return module

stm32_driver_source = '"""Raspberry Pi 5 UART client for the STM32 dual-BTS7960 controller.\n\nRun this file with the normal Python 3 interpreter in Thonny. Raspberry Pi OS\ndoes not use MicroPython\'s ``machine.UART``; Linux exposes UARTs as device files\nand this client accesses one with pyserial.\n\nDefault Pi 5 UART1 wiring:\n    GPIO0 / physical pin 27 / TX -> STM32 PA10 / RX\n    GPIO1 / physical pin 28 / RX <- STM32 PA9  / TX\n    Pi GND                       <-> STM32 GND\n\nEnable UART1 with ``dtoverlay=uart1-pi5`` before running this file. See the\nadjacent README.md for complete setup and safety instructions.\n"""\n\nimport threading\nimport time\n\ntry:\n    import serial\nexcept ImportError as exc:\n    raise ImportError(\n        "pyserial is required; install Raspberry Pi OS package python3-serial"\n    ) from exc\n\n\n# User configuration. GPIO numbers describe the Device Tree overlay wiring;\n# changing them here documents a changed overlay but cannot remap Linux UARTs.\nUART_DEVICE = "/dev/ttyAMA1"\nUART_BAUD = 115200\nUART_TX_GPIO = 0\nUART_RX_GPIO = 1\n\nUART_TIMEOUT_S = 0.10\nCOMMAND_TIMEOUT_S = 1.0\nSTATUS_INTERVAL_S = 0.40\nSERVICE_REPLY_TIMEOUT_S = 0.25\nLINK_TIMEOUT_S = 3.0\nMAX_RX_LINE_BYTES = 256\nPWM_MAX_DUTY = 4095\nENCODER_RATES_HZ = (10, 20, 25, 50, 100)\nDEFAULT_ENCODER_RATE_HZ = 50\n\n\ndef _new_motor_state():\n    return {\n        "direction": "S",\n        "duty": 0,\n        "r_is_raw": 0,\n        "l_is_raw": 0,\n        "r_is_mv": 0,\n        "l_is_mv": 0,\n    }\n\n\nclass STM32MotorController:\n    """Thread-safe client for the STM32 newline-delimited ASCII protocol."""\n\n    def __init__(self, device=UART_DEVICE, baudrate=UART_BAUD):\n        self.device = str(device)\n        self.baudrate = int(baudrate)\n        self.port = None\n\n        self._stop_event = threading.Event()\n        self._write_lock = threading.Lock()\n        self._command_lock = threading.Lock()\n        self._reply_condition = threading.Condition()\n        self._state_lock = threading.Lock()\n        self._pending_prefixes = None\n        self._pending_reply = None\n        self._reader_thread = None\n        self._service_thread = None\n\n        self.last_line = ""\n        self.last_error = ""\n        self.last_seen_monotonic = None\n        self.heartbeat_ms = None\n        self.heartbeat_count = 0\n        self.status_count = 0\n        self.encoder_count = 0\n        self.encoder_dropped = 0\n        self.encoder_sequence = None\n        self.encoder_stm32_ms = None\n        self.encoder_seen_monotonic = None\n        self.encoder_values = [0, 0, 0, 0]\n        self.encoder_stream_hz = 0\n        self.watchdog_stopped = False\n        self.motors = [_new_motor_state(), _new_motor_state()]\n\n    def start(self):\n        """Open the UART and start receive/watchdog-service threads."""\n        if self.port is not None and self.port.is_open:\n            return self\n\n        self._stop_event.clear()\n        self.port = serial.Serial(\n            port=self.device,\n            baudrate=self.baudrate,\n            bytesize=serial.EIGHTBITS,\n            parity=serial.PARITY_NONE,\n            stopbits=serial.STOPBITS_ONE,\n            timeout=UART_TIMEOUT_S,\n            write_timeout=COMMAND_TIMEOUT_S,\n        )\n        self.port.reset_input_buffer()\n\n        self._reader_thread = threading.Thread(\n            target=self._reader_loop, name="stm32-uart-reader", daemon=True\n        )\n        self._service_thread = threading.Thread(\n            target=self._service_loop, name="stm32-uart-service", daemon=True\n        )\n        self._reader_thread.start()\n        self._service_thread.start()\n        print(\n            "STM32 UART opened: {} at {} baud (Pi TX GPIO{}, RX GPIO{})".format(\n                self.device, self.baudrate, UART_TX_GPIO, UART_RX_GPIO\n            )\n        )\n        return self\n\n    def close(self):\n        """Stop both motors, terminate workers, and close the serial device."""\n        if self.port is None:\n            return\n        try:\n            if self.port.is_open:\n                self.command("MSTOP ALL", ("OK MSTOP ALL",), 0.5)\n        except Exception:\n            pass\n        self._stop_event.set()\n        with self._reply_condition:\n            self._reply_condition.notify_all()\n        for worker in (self._service_thread, self._reader_thread):\n            if worker and worker is not threading.current_thread():\n                worker.join(timeout=0.5)\n        if self.port.is_open:\n            self.port.close()\n        self.port = None\n\n    def _write_line(self, line):\n        if self.port is None or not self.port.is_open:\n            raise RuntimeError("UART is not connected; call connect()")\n        payload = (line.strip() + "\\n").encode("ascii")\n        with self._write_lock:\n            self.port.write(payload)\n            self.port.flush()\n\n    @staticmethod\n    def _matches(line, prefixes):\n        return any(line == prefix or line.startswith(prefix + " ") for prefix in prefixes)\n\n    def command(self, line, expected_prefixes=("OK",), timeout_s=COMMAND_TIMEOUT_S):\n        """Send one command and wait for its matching non-heartbeat response."""\n        if isinstance(expected_prefixes, str):\n            expected_prefixes = (expected_prefixes,)\n        expected_prefixes = tuple(expected_prefixes)\n        timeout_s = float(timeout_s)\n\n        if not self._command_lock.acquire(timeout=max(0.05, timeout_s)):\n            message = "UART busy before {!r}".format(line)\n            with self._state_lock:\n                self.last_error = message\n            raise TimeoutError(message)\n\n        try:\n            with self._reply_condition:\n                self._pending_prefixes = expected_prefixes\n                self._pending_reply = None\n            self._write_line(line)\n\n            deadline = time.monotonic() + timeout_s\n            with self._reply_condition:\n                while self._pending_reply is None and not self._stop_event.is_set():\n                    remaining = deadline - time.monotonic()\n                    if remaining <= 0:\n                        break\n                    self._reply_condition.wait(remaining)\n                reply = self._pending_reply\n                self._pending_prefixes = None\n                self._pending_reply = None\n\n            if reply is None:\n                message = "No STM32 reply to {!r}".format(line)\n                with self._state_lock:\n                    self.last_error = message\n                raise TimeoutError(message)\n            if reply.startswith("ERR "):\n                raise RuntimeError(reply)\n            return reply\n        finally:\n            with self._reply_condition:\n                self._pending_prefixes = None\n                self._pending_reply = None\n            self._command_lock.release()\n\n    def _reader_loop(self):\n        while not self._stop_event.is_set():\n            try:\n                raw = self.port.read_until(b"\\n", MAX_RX_LINE_BYTES)\n                if not raw:\n                    continue\n                line = raw.decode("ascii", "replace").strip()\n                if line:\n                    self._handle_line(line)\n            except (OSError, serial.SerialException) as exc:\n                if not self._stop_event.is_set():\n                    with self._state_lock:\n                        self.last_error = "UART read failed: {}".format(exc)\n                    print(self.last_error)\n                    self._stop_event.wait(0.25)\n\n    def _handle_line(self, line):\n        now = time.monotonic()\n        with self._state_lock:\n            was_alive = self._alive_locked(now)\n            self.last_line = line\n\n            if line.startswith("HB "):\n                fields = line.split()\n                if len(fields) == 2:\n                    try:\n                        self.heartbeat_ms = int(fields[1])\n                    except ValueError:\n                        self.last_error = "Malformed heartbeat: " + line\n                    else:\n                        self.heartbeat_count += 1\n                        self.last_seen_monotonic = now\n            elif line.startswith("READY "):\n                self.last_seen_monotonic = now\n                self.encoder_stream_hz = 0\n                self.encoder_sequence = None\n            elif line.startswith("MSTAT "):\n                if self._parse_mstat_locked(line):\n                    self.status_count += 1\n                    self.last_seen_monotonic = now\n                    self.last_error = ""\n            elif line.startswith("ENC "):\n                if self._parse_encoder_locked(line):\n                    self.encoder_count += 1\n                    self.encoder_seen_monotonic = now\n                    self.last_seen_monotonic = now\n                    self.last_error = ""\n            elif line.startswith("FAULT "):\n                self.watchdog_stopped = True\n                self.last_seen_monotonic = now\n                self.last_error = line\n                for motor in self.motors:\n                    motor["direction"] = "S"\n                    motor["duty"] = 0\n            elif line.startswith(("OK ", "CAPS ")):\n                self.last_seen_monotonic = now\n                self.last_error = ""\n                if line.startswith("OK ENCON "):\n                    try:\n                        self.encoder_stream_hz = int(line.split()[2])\n                    except (IndexError, ValueError):\n                        self.last_error = "Malformed ENCON response: " + line\n                elif line == "OK ENCOFF":\n                    self.encoder_stream_hz = 0\n\n            became_alive = not was_alive and self._alive_locked(now)\n\n        if became_alive:\n            print("STM32 link established: " + line)\n\n        with self._reply_condition:\n            prefixes = self._pending_prefixes\n            if prefixes and (line.startswith("ERR ") or self._matches(line, prefixes)):\n                self._pending_reply = line\n                self._reply_condition.notify_all()\n\n    def _parse_mstat_locked(self, line):\n        fields = line.split()\n        if len(fields) != 17 or fields[0:2] != ["MSTAT", "1"] or fields[8] != "2":\n            self.last_error = "Malformed MSTAT: " + line\n            return False\n        try:\n            for offset, motor in ((2, self.motors[0]), (9, self.motors[1])):\n                motor.update(\n                    direction=fields[offset],\n                    duty=int(fields[offset + 1]),\n                    r_is_raw=int(fields[offset + 2]),\n                    l_is_raw=int(fields[offset + 3]),\n                    r_is_mv=int(fields[offset + 4]),\n                    l_is_mv=int(fields[offset + 5]),\n                )\n            self.watchdog_stopped = fields[15] == "WD" and int(fields[16]) != 0\n        except ValueError:\n            self.last_error = "Malformed numeric MSTAT field: " + line\n            return False\n        return True\n\n    def _parse_encoder_locked(self, line):\n        fields = line.split()\n        if len(fields) != 7 or fields[0] != "ENC":\n            self.last_error = "Malformed ENC: " + line\n            return False\n        try:\n            sequence = int(fields[1])\n            stm32_ms = int(fields[2])\n            values = [int(value) for value in fields[3:7]]\n        except ValueError:\n            self.last_error = "Malformed numeric ENC field: " + line\n            return False\n        if any(value < 0 or value > 4095 for value in values):\n            self.last_error = "ENC ADC value outside 0..4095: " + line\n            return False\n        if self.encoder_sequence is not None:\n            expected = (self.encoder_sequence + 1) & 0xFFFFFFFF\n            missing = (sequence - expected) & 0xFFFFFFFF\n            if missing < 0x80000000:\n                self.encoder_dropped += missing\n        self.encoder_sequence = sequence\n        self.encoder_stm32_ms = stm32_ms\n        self.encoder_values = values\n        return True\n\n    def _alive_locked(self, now=None):\n        if self.last_seen_monotonic is None:\n            return False\n        if now is None:\n            now = time.monotonic()\n        return now - self.last_seen_monotonic <= LINK_TIMEOUT_S\n\n    def _service_loop(self):\n        """Poll telemetry frequently enough to feed the STM32 host watchdog."""\n        next_poll = 0.0\n        link_was_alive = False\n        while not self._stop_event.is_set():\n            now = time.monotonic()\n            if now >= next_poll:\n                next_poll = now + STATUS_INTERVAL_S\n                try:\n                    self.command("MSTATUS", ("MSTAT",), SERVICE_REPLY_TIMEOUT_S)\n                except (TimeoutError, RuntimeError, OSError, serial.SerialException):\n                    pass\n\n            with self._state_lock:\n                alive = self._alive_locked()\n            if link_was_alive and not alive:\n                print("STM32 link lost; its watchdog should stop both motors")\n            link_was_alive = alive\n            self._stop_event.wait(0.02)\n\n    def snapshot(self):\n        """Return a copy of current link and motor telemetry."""\n        with self._state_lock:\n            age_ms = (\n                None\n                if self.last_seen_monotonic is None\n                else round((time.monotonic() - self.last_seen_monotonic) * 1000)\n            )\n            encoder_age_ms = (\n                None\n                if self.encoder_seen_monotonic is None\n                else round((time.monotonic() - self.encoder_seen_monotonic) * 1000)\n            )\n            return {\n                "alive": self._alive_locked(),\n                "age_ms": age_ms,\n                "heartbeat_ms": self.heartbeat_ms,\n                "heartbeat_count": self.heartbeat_count,\n                "status_count": self.status_count,\n                "encoder_count": self.encoder_count,\n                "encoder_dropped": self.encoder_dropped,\n                "encoder_age_ms": encoder_age_ms,\n                "encoder_sequence": self.encoder_sequence,\n                "encoder_stm32_ms": self.encoder_stm32_ms,\n                "encoder_values": list(self.encoder_values),\n                "encoder_stream_hz": self.encoder_stream_hz,\n                "watchdog_stopped": self.watchdog_stopped,\n                "last_line": self.last_line,\n                "last_error": self.last_error,\n                "motors": [dict(motor) for motor in self.motors],\n            }\n\n\ncontroller = None\n\n\ndef connect(device=UART_DEVICE, baudrate=UART_BAUD, verify=True):\n    """Open the UART and optionally verify a complete STM32 round trip."""\n    global controller\n    if controller is not None:\n        controller.close()\n    controller = STM32MotorController(device, baudrate).start()\n    if verify:\n        check_connection()\n    return controller\n\n\ndef disconnect():\n    global controller\n    if controller is not None:\n        controller.close()\n        controller = None\n\n\ndef _link():\n    if controller is None or controller.port is None:\n        raise RuntimeError("UART is not connected; call connect()")\n    return controller\n\n\ndef ping():\n    return _link().command("PING", ("OK PONG",))\n\n\ndef check_connection(timeout_s=2.0):\n    """Verify Pi TX, STM32 RX, STM32 TX, and Pi RX with PING/PONG."""\n    try:\n        reply = _link().command("PING", ("OK PONG",), float(timeout_s))\n    except (TimeoutError, RuntimeError, OSError, serial.SerialException) as exc:\n        print("STM32 UART ROUND TRIP FAILED: {}".format(exc))\n        print("Expected: Pi GPIO0 TX -> STM32 PA10 RX")\n        print("          Pi GPIO1 RX <- STM32 PA9 TX, with common GND")\n        return False\n    print("STM32 UART ROUND TRIP VERIFIED: {}".format(reply))\n    return True\n\n\ndef caps():\n    return _link().command("CAPS", ("CAPS",))\n\n\ndef drive(motor, direction, duty):\n    """Drive motor 1 or 2 in physical direction A or B at duty 0..4095."""\n    motor = int(motor)\n    direction = str(direction).strip().upper()\n    duty = int(duty)\n    if motor not in (1, 2):\n        raise ValueError("motor must be 1 or 2")\n    if direction not in ("A", "B"):\n        raise ValueError("direction must be \'A\' or \'B\'")\n    if not 0 <= duty <= PWM_MAX_DUTY:\n        raise ValueError("duty must be from 0 to {}".format(PWM_MAX_DUTY))\n    return _link().command(\n        "MOTOR {} {} {}".format(motor, direction, duty), ("OK MOTOR",)\n    )\n\n\ndef drive_percent(motor, direction, percent):\n    """Drive using a percentage from 0 through 100."""\n    percent = float(percent)\n    if not 0.0 <= percent <= 100.0:\n        raise ValueError("percent must be from 0 to 100")\n    duty = round(percent * PWM_MAX_DUTY / 100.0)\n    return drive(motor, direction, duty)\n\n\ndef pulse(motor, direction="A", percent=6, seconds=0.5):\n    """Run one motor briefly and always request a stop before returning."""\n    seconds = float(seconds)\n    if seconds <= 0 or seconds > 5:\n        raise ValueError("seconds must be greater than 0 and at most 5")\n    try:\n        reply = drive_percent(motor, direction, percent)\n        print(reply)\n        time.sleep(seconds)\n    finally:\n        try:\n            print(stop_motor(motor))\n        except Exception as exc:\n            # Send an unacknowledged emergency stop even if response matching\n            # failed; the STM32 watchdog remains the final fallback.\n            try:\n                _link()._write_line("MSTOP ALL")\n            finally:\n                print("Stop acknowledgement failed; sent MSTOP ALL: {}".format(exc))\n\n\ndef direction_a(motor, duty=1024):\n    return drive(motor, "A", duty)\n\n\ndef direction_b(motor, duty=1024):\n    return drive(motor, "B", duty)\n\n\ndef stop_motor(motor):\n    motor = int(motor)\n    if motor not in (1, 2):\n        raise ValueError("motor must be 1 or 2")\n    return _link().command("MSTOP {}".format(motor), ("OK MSTOP",))\n\n\ndef stop_all():\n    return _link().command("MSTOP ALL", ("OK MSTOP ALL",))\n\n\ndef reset_motors():\n    return _link().command("RESET", ("OK RESET",))\n\n\ndef read_currents(motor=None):\n    """Refresh and return raw ADC and millivolt readings, not calibrated amps."""\n    _link().command("MSTATUS", ("MSTAT",))\n    motors = _link().snapshot()["motors"]\n    if motor is None:\n        return motors\n    motor = int(motor)\n    if motor not in (1, 2):\n        raise ValueError("motor must be 1 or 2")\n    return motors[motor - 1]\n\n\ndef read_encoders():\n    """Request one PA0..PA3 ADC sample set and return four raw 12-bit values."""\n    _link().command("ENCREAD", ("ENC",))\n    return list(_link().snapshot()["encoder_values"])\n\n\ndef encoder_start(rate_hz=DEFAULT_ENCODER_RATE_HZ):\n    """Enable opt-in PA0..PA3 reports at a supported samples-per-second rate."""\n    rate_hz = int(rate_hz)\n    if rate_hz not in ENCODER_RATES_HZ:\n        raise ValueError("rate_hz must be one of {}".format(ENCODER_RATES_HZ))\n    reply = _link().command("ENCON {}".format(rate_hz), ("OK ENCON",))\n    return reply\n\n\ndef encoder_stop():\n    """Disable periodic encoder ADC reports; one-shot ENCREAD remains available."""\n    reply = _link().command("ENCOFF", ("OK ENCOFF",))\n    return reply\n\n\ndef encoder_status():\n    """Return the latest cached PA0..PA3 ADC report without UART traffic."""\n    snapshot = _link().snapshot()\n    return {\n        "pa0": snapshot["encoder_values"][0],\n        "pa1": snapshot["encoder_values"][1],\n        "pa2": snapshot["encoder_values"][2],\n        "pa3": snapshot["encoder_values"][3],\n        "sequence": snapshot["encoder_sequence"],\n        "stm32_ms": snapshot["encoder_stm32_ms"],\n        "reports": snapshot["encoder_count"],\n        "dropped": snapshot["encoder_dropped"],\n        "age_ms": snapshot["encoder_age_ms"],\n        "rate_hz": snapshot["encoder_stream_hz"],\n    }\n\n\ndef status():\n    snapshot = _link().snapshot()\n    print(\n        "STM32 {} | age={} ms | heartbeat={} | watchdog={}".format(\n            "alive" if snapshot["alive"] else "missing",\n            snapshot["age_ms"],\n            snapshot["heartbeat_ms"],\n            snapshot["watchdog_stopped"],\n        )\n    )\n    for number, motor in enumerate(snapshot["motors"], 1):\n        print(\n            "Motor {}: dir={} duty={} R_IS={} raw/{} mV L_IS={} raw/{} mV".format(\n                number,\n                motor["direction"],\n                motor["duty"],\n                motor["r_is_raw"],\n                motor["r_is_mv"],\n                motor["l_is_raw"],\n                motor["l_is_mv"],\n            )\n        )\n    if snapshot["last_error"]:\n        print("Last error:", snapshot["last_error"])\n    return snapshot\n\n\ndef raw(command, expected_prefixes=("OK", "CAPS", "MSTAT")):\n    """Send an advanced protocol command and return its response line."""\n    return _link().command(str(command), expected_prefixes)\n\n\ndef monitor(seconds=10, interval=0.5):\n    """Print live status/current telemetry for a limited number of seconds."""\n    deadline = time.monotonic() + float(seconds)\n    while time.monotonic() < deadline:\n        status()\n        time.sleep(float(interval))\n\n\ndef show_help():\n    print("Raspberry Pi -> STM32 dual BTS7960 commands:")\n    print("  check_connection() | ping() | caps() | status()")\n    print("  read_currents() | monitor(10)")\n    print("  read_encoders() | encoder_start(50) | encoder_status() | encoder_stop()")\n    print("  direction_a(1, 1024) | direction_b(2, 2048)")\n    print("  drive(1, \'A\', 4095) | drive_percent(2, \'B\', 25)")\n    print("  pulse(1, \'A\', 6, 0.5)  # run briefly, then stop")\n    print("  stop_motor(1) | stop_all() | reset_motors()")\n    print("  connect(\'/dev/ttyAMA1\', 115200) | disconnect()")\n    print("Direction A/B must be identified safely for each motor before renaming.")\n\n\nif __name__ == "__main__":\n    try:\n        connect()\n    except (OSError, serial.SerialException) as exc:\n        print("STM32 UART not opened: {}".format(exc))\n        print("Complete the UART setup, then call connect().")\n\n    show_help()\n'
stm32_driver = _embedded_module('stm32_driver', stm32_driver_source)

odesc_driver_source = '"""Raspberry Pi 5 UART client for an ODESC/ODrive v3-compatible controller.\n\nValidated wiring:\n    Pi GPIO4 / physical 7  / UART2 TX -> ODESC GPIO2 / UART RX\n    Pi GPIO5 / physical 29 / UART2 RX <- ODESC GPIO1 / UART TX\n    Pi GND                               -> ODESC GND\n\nThis is regular Python 3 for Raspberry Pi OS and Thonny, not MicroPython.\nDependency: pyserial (Raspberry Pi OS package ``python3-serial``).\n"""\n\nimport threading\nimport time\nimport math\n\ntry:\n    import serial\nexcept ImportError as exc:\n    raise ImportError(\n        "pyserial is required; install Raspberry Pi OS package python3-serial"\n    ) from exc\n\n\nUART_DEVICE = "/dev/ttyAMA2"\nUART_BAUD = 115200\nUART_TIMEOUT_SECONDS = 1.0\nAXIS_STATE_IDLE = 1\nAXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3\nAXIS_STATE_MOTOR_CALIBRATION = 4\nAXIS_STATE_SENSORLESS_CONTROL = 5\nAXIS_STATE_CLOSED_LOOP_CONTROL = 8\nCONTROL_MODE_VELOCITY_CONTROL = 2\nINPUT_MODE_PASSTHROUGH = 1\n\n\nclass ODESCUART:\n    """Synchronized client for the ODrive v0.5.x ASCII protocol."""\n\n    def __init__(\n        self,\n        device=UART_DEVICE,\n        baudrate=UART_BAUD,\n        timeout=UART_TIMEOUT_SECONDS,\n    ):\n        self.device = str(device)\n        self.baudrate = int(baudrate)\n        self.timeout = float(timeout)\n        self.port = None\n        self._lock = threading.Lock()\n\n    def open(self):\n        if self.port is not None and self.port.is_open:\n            return self\n\n        self.port = serial.Serial(\n            port=self.device,\n            baudrate=self.baudrate,\n            bytesize=serial.EIGHTBITS,\n            parity=serial.PARITY_NONE,\n            stopbits=serial.STOPBITS_ONE,\n            timeout=self.timeout,\n            write_timeout=self.timeout,\n            exclusive=True,\n        )\n        self.port.reset_input_buffer()\n        self.port.reset_output_buffer()\n        print(\n            "ODESC UART opened: {} at {} baud (Pi TX GPIO4, RX GPIO5)".format(\n                self.device, self.baudrate\n            )\n        )\n        return self\n\n    def close(self):\n        if self.port is not None:\n            try:\n                self.port.close()\n            finally:\n                self.port = None\n\n    def query(self, command, log=True):\n        """Send one ASCII command and return its non-empty response line."""\n        command = str(command).strip()\n        if not command or "\\n" in command or "\\r" in command:\n            raise ValueError("command must contain exactly one non-empty line")\n        if self.port is None or not self.port.is_open:\n            raise RuntimeError("ODESC UART is not open")\n\n        packet = (command + "\\n").encode("ascii")\n        with self._lock:\n            self.port.reset_input_buffer()\n            self.port.write(packet)\n            self.port.flush()\n            response = self.port.readline()\n\n        if log:\n            print("TX:", repr(packet))\n            print("RX:", repr(response))\n        if not response:\n            raise TimeoutError("No ODESC reply to {!r}".format(command))\n\n        text = response.decode("ascii", errors="replace").strip()\n        if not text:\n            raise RuntimeError("ODESC returned an empty response")\n        return text\n\n    def send(self, command):\n        """Send one ASCII command that does not produce a response."""\n        command = str(command).strip()\n        if not command or "\\n" in command or "\\r" in command:\n            raise ValueError("command must contain exactly one non-empty line")\n        if self.port is None or not self.port.is_open:\n            raise RuntimeError("ODESC UART is not open")\n\n        packet = (command + "\\n").encode("ascii")\n        with self._lock:\n            self.port.write(packet)\n            self.port.flush()\n        print("TX:", repr(packet))\n\n    def write_property(self, name, value):\n        self.send("w {} {}".format(str(name).strip(), value))\n\n    def request_state(self, axis, state):\n        axis = self._axis(axis)\n        self.write_property("axis{}.requested_state".format(axis), int(state))\n\n    def set_velocity(self, axis, turns_per_second, torque_ff=0.0):\n        axis = self._axis(axis)\n        self.send(\n            "v {} {:.6f} {:.6f}".format(\n                axis, float(turns_per_second), float(torque_ff)\n            )\n        )\n\n    def stop_axis(self, axis):\n        self.set_velocity(axis, 0.0, 0.0)\n\n    def feedback(self, axis):\n        axis = self._axis(axis)\n        fields = self.query("f {}".format(axis)).split()\n        if len(fields) != 2:\n            raise RuntimeError("Malformed ODESC feedback: {!r}".format(fields))\n        return {"position_turns": float(fields[0]), "velocity_turns_s": float(fields[1])}\n\n    @staticmethod\n    def _axis(axis):\n        axis = int(axis)\n        if axis not in (0, 1):\n            raise ValueError("ODESC axis must be 0 or 1")\n        return axis\n\n    def read_property(self, name, log=True):\n        return self.query("r {}".format(str(name).strip()), log=log)\n\n    def read_float(self, name, log=True):\n        return float(self.read_property(name, log=log))\n\n    def read_int(self, name, log=True):\n        value = self.read_property(name, log=log)\n        # ODrive v0.5.x can append "d" to decimal integer responses.\n        if value.endswith("d"):\n            value = value[:-1]\n        return int(value, 0)\n\n    def axis_snapshot(self, axis, log=False):\n        """Read motion, electrical, configuration, and fault state for one axis."""\n        axis = self._axis(axis)\n        prefix = "axis{}.".format(axis)\n        pole_pairs = self.read_int(\n            prefix + "motor.config.pole_pairs", log=log\n        )\n        ramp_velocity_electrical = self.read_float(\n            prefix + "config.sensorless_ramp.vel", log=log\n        )\n        ramp_accel_electrical = self.read_float(\n            prefix + "config.sensorless_ramp.accel", log=log\n        )\n        sensorless_velocity_electrical = self.read_float(\n            prefix + "sensorless_estimator.vel_estimate", log=log\n        )\n        electrical_per_mechanical_turn = 2.0 * math.pi * pole_pairs\n        return {\n            "state": self.read_int(prefix + "current_state", log=log),\n            "axis_error": self.read_int(prefix + "error", log=log),\n            "motor_error": self.read_int(prefix + "motor.error", log=log),\n            "encoder_error": self.read_int(prefix + "encoder.error", log=log),\n            "controller_error": self.read_int(\n                prefix + "controller.error", log=log\n            ),\n            "position_turns": self.read_float(\n                prefix + "encoder.pos_estimate", log=log\n            ),\n            "velocity_turns_s": self.read_float(\n                prefix + "encoder.vel_estimate", log=log\n            ),\n            "iq_measured_a": self.read_float(\n                prefix + "motor.current_control.Iq_measured", log=log\n            ),\n            "iq_setpoint_a": self.read_float(\n                prefix + "motor.current_control.Iq_setpoint", log=log\n            ),\n            "current_limit_a": self.read_float(\n                prefix + "motor.config.current_lim", log=log\n            ),\n            "velocity_limit_turns_s": self.read_float(\n                prefix + "controller.config.vel_limit", log=log\n            ),\n            "control_mode": self.read_int(\n                prefix + "controller.config.control_mode", log=log\n            ),\n            "input_mode": self.read_int(\n                prefix + "controller.config.input_mode", log=log\n            ),\n            "motor_pre_calibrated": bool(\n                self.read_int(prefix + "motor.config.pre_calibrated", log=log)\n            ),\n            "pole_pairs": pole_pairs,\n            "encoder_pre_calibrated": bool(\n                self.read_int(prefix + "encoder.config.pre_calibrated", log=log)\n            ),\n            "sensorless_estimator_error": self.read_int(\n                prefix + "sensorless_estimator.error", log=log\n            ),\n            "sensorless_velocity_turns_s": (\n                sensorless_velocity_electrical\n                / electrical_per_mechanical_turn\n            ),\n            "sensorless_ramp_current_a": self.read_float(\n                prefix + "config.sensorless_ramp.current", log=log\n            ),\n            "sensorless_ramp_velocity_turns_s": (\n                ramp_velocity_electrical\n                / electrical_per_mechanical_turn\n            ),\n            "sensorless_ramp_accel_turns_s2": (\n                ramp_accel_electrical\n                / electrical_per_mechanical_turn\n            ),\n            "sensorless_pm_flux_linkage": self.read_float(\n                prefix + "sensorless_estimator.config.pm_flux_linkage",\n                log=log,\n            ),\n            "sensorless_observer_gain": self.read_float(\n                prefix + "sensorless_estimator.config.observer_gain",\n                log=log,\n            ),\n            "sensorless_pll_bandwidth": self.read_float(\n                prefix + "sensorless_estimator.config.pll_bandwidth",\n                log=log,\n            ),\n        }\n\n    def telemetry(self):\n        """Return live DC-bus power and both motor-axis telemetry snapshots."""\n        voltage = self.read_float("vbus_voltage", log=False)\n        bus_current = self.read_float("ibus", log=False)\n        return {\n            "vbus_voltage": voltage,\n            "ibus_a": bus_current,\n            "bus_power_w": voltage * bus_current,\n            "axes": [self.axis_snapshot(0), self.axis_snapshot(1)],\n        }\n\n    def clear_errors(self, axis):\n        """Clear writable axis/component error registers on ODrive v0.5.x."""\n        axis = self._axis(axis)\n        prefix = "axis{}.".format(axis)\n        self.stop_axis(axis)\n        for name in (\n            "error",\n            "motor.error",\n            "encoder.error",\n            "controller.error",\n        ):\n            self.write_property(prefix + name, 0)\n        return self.axis_snapshot(axis)\n\n    def configure_velocity_axis(\n        self,\n        axis,\n        current_limit_a,\n        velocity_limit_turns_s,\n    ):\n        """Apply volatile velocity-control settings without saving or rebooting."""\n        axis = self._axis(axis)\n        current_limit_a = float(current_limit_a)\n        velocity_limit_turns_s = float(velocity_limit_turns_s)\n        if not 0.5 <= current_limit_a <= 30.0:\n            raise ValueError("current limit must be from 0.5 through 30 A")\n        if not 0.1 <= velocity_limit_turns_s <= 50.0:\n            raise ValueError("velocity limit must be from 0.1 through 50 turns/s")\n        prefix = "axis{}.".format(axis)\n        self.stop_axis(axis)\n        self.write_property(\n            prefix + "controller.config.control_mode",\n            CONTROL_MODE_VELOCITY_CONTROL,\n        )\n        self.write_property(\n            prefix + "controller.config.input_mode",\n            INPUT_MODE_PASSTHROUGH,\n        )\n        self.write_property(prefix + "motor.config.current_lim", current_limit_a)\n        self.write_property(\n            prefix + "controller.config.vel_limit",\n            velocity_limit_turns_s,\n        )\n        return self.axis_snapshot(axis)\n\n    def configure_sensorless_axis(\n        self,\n        axis,\n        current_limit_a,\n        startup_current_a,\n        startup_velocity_turns_s,\n        startup_accel_turns_s2,\n        velocity_limit_turns_s,\n    ):\n        """Apply volatile legacy ODrive v3 sensorless settings.\n\n        The old firmware expresses its startup ramp in electrical radians per\n        second. This API accepts mechanical turns per second and converts using\n        the configured motor pole-pair count.\n        """\n        axis = self._axis(axis)\n        current_limit_a = float(current_limit_a)\n        startup_current_a = float(startup_current_a)\n        startup_velocity_turns_s = abs(float(startup_velocity_turns_s))\n        startup_accel_turns_s2 = abs(float(startup_accel_turns_s2))\n        velocity_limit_turns_s = abs(float(velocity_limit_turns_s))\n        if not 0.5 <= current_limit_a <= 30.0:\n            raise ValueError("current limit must be from 0.5 through 30 A")\n        if not 0.5 <= startup_current_a <= current_limit_a:\n            raise ValueError(\n                "startup current must be from 0.5 A through the current limit"\n            )\n        if not 1.0 <= startup_velocity_turns_s <= 30.0:\n            raise ValueError(\n                "sensorless startup speed must be from 1 through 30 turns/s"\n            )\n        if not 0.1 <= startup_accel_turns_s2 <= 30.0:\n            raise ValueError(\n                "sensorless startup acceleration must be from 0.1 through "\n                "30 turns/s^2"\n            )\n        if not startup_velocity_turns_s <= velocity_limit_turns_s <= 50.0:\n            raise ValueError(\n                "velocity limit must be at least the startup speed and at "\n                "most 50 turns/s"\n            )\n\n        prefix = "axis{}.".format(axis)\n        self.request_state(axis, AXIS_STATE_IDLE)\n        if not self.read_int(prefix + "motor.config.pre_calibrated", log=False):\n            raise RuntimeError(\n                "axis {} motor is not calibrated; run motor calibration first".format(\n                    axis\n                )\n            )\n        pole_pairs = self.read_int(\n            prefix + "motor.config.pole_pairs", log=False\n        )\n        if not 1 <= pole_pairs <= 100:\n            raise RuntimeError("invalid motor pole-pair count {}".format(pole_pairs))\n        electrical_scale = 2.0 * math.pi * pole_pairs\n\n        self.write_property(\n            prefix + "controller.config.control_mode",\n            CONTROL_MODE_VELOCITY_CONTROL,\n        )\n        self.write_property(\n            prefix + "controller.config.input_mode",\n            INPUT_MODE_PASSTHROUGH,\n        )\n        self.write_property(prefix + "motor.config.current_lim", current_limit_a)\n        self.write_property(\n            prefix + "controller.config.vel_limit", velocity_limit_turns_s\n        )\n        self.write_property(\n            prefix + "config.sensorless_ramp.current", startup_current_a\n        )\n        self.write_property(\n            prefix + "config.sensorless_ramp.vel",\n            startup_velocity_turns_s * electrical_scale,\n        )\n        self.write_property(\n            prefix + "config.sensorless_ramp.accel",\n            startup_accel_turns_s2 * electrical_scale,\n        )\n        self.write_property(prefix + "config.sensorless_ramp.finish_on_vel", 1)\n        self.write_property(\n            prefix + "config.sensorless_ramp.finish_on_distance", 0\n        )\n        return self.axis_snapshot(axis)\n\n    def start_sensorless(self, axis, direction):\n        """Start the legacy state-5 open-loop ramp and sensorless estimator."""\n        axis = self._axis(axis)\n        direction = 1 if int(direction) >= 0 else -1\n        prefix = "axis{}.".format(axis)\n        self.request_state(axis, AXIS_STATE_IDLE)\n        if not self.read_int(prefix + "motor.config.pre_calibrated", log=False):\n            raise RuntimeError(\n                "axis {} motor is not calibrated; run motor calibration first".format(\n                    axis\n                )\n            )\n        ramp_velocity = abs(\n            self.read_float(prefix + "config.sensorless_ramp.vel", log=False)\n        )\n        ramp_accel = abs(\n            self.read_float(prefix + "config.sensorless_ramp.accel", log=False)\n        )\n        pole_pairs = self.read_int(\n            prefix + "motor.config.pole_pairs", log=False\n        )\n        if pole_pairs <= 0 or ramp_accel <= 0.0:\n            raise RuntimeError("invalid sensorless ramp configuration")\n        self.write_property(\n            prefix + "config.sensorless_ramp.vel",\n            direction * ramp_velocity,\n        )\n        self.request_state(axis, AXIS_STATE_SENSORLESS_CONTROL)\n        time.sleep(0.20)\n        state = self.read_int(prefix + "current_state", log=False)\n        axis_error = self.read_int(prefix + "error", log=False)\n        motor_error = self.read_int(prefix + "motor.error", log=False)\n        estimator_error = self.read_int(\n            prefix + "sensorless_estimator.error", log=False\n        )\n        if (\n            state != AXIS_STATE_SENSORLESS_CONTROL\n            or axis_error\n            or motor_error\n            or estimator_error\n        ):\n            self.request_state(axis, AXIS_STATE_IDLE)\n            raise RuntimeError(\n                "sensorless startup failed: state={} axis_error={} "\n                "motor_error={} estimator_error={}".format(\n                    state, axis_error, motor_error, estimator_error\n                )\n            )\n        electrical_scale = 2.0 * math.pi * pole_pairs\n        return {\n            "axis": axis,\n            "state": state,\n            "axis_error": axis_error,\n            "motor_error": motor_error,\n            "sensorless_estimator_error": estimator_error,\n            "sensorless_ramp_velocity_turns_s": (\n                direction * ramp_velocity / electrical_scale\n            ),\n            "sensorless_ramp_accel_turns_s2": (\n                ramp_accel / electrical_scale\n            ),\n        }\n\n    def set_sensorless_velocity(self, axis, turns_per_second):\n        """Set velocity while enforcing legacy sensorless speed/direction limits."""\n        axis = self._axis(axis)\n        velocity = float(turns_per_second)\n        prefix = "axis{}.".format(axis)\n        state = self.read_int(prefix + "current_state", log=False)\n        if state != AXIS_STATE_SENSORLESS_CONTROL:\n            raise RuntimeError(\n                "axis {} is not in sensorless state 5".format(axis)\n            )\n        pole_pairs = self.read_int(\n            prefix + "motor.config.pole_pairs", log=False\n        )\n        ramp_velocity_electrical = self.read_float(\n            prefix + "config.sensorless_ramp.vel", log=False\n        )\n        startup_velocity = (\n            ramp_velocity_electrical / (2.0 * math.pi * pole_pairs)\n        )\n        if velocity == 0.0:\n            raise ValueError("sensorless cannot command zero; stop to IDLE instead")\n        if velocity * startup_velocity <= 0.0:\n            raise ValueError(\n                "sensorless direction cannot reverse while running; stop and restart"\n            )\n        if abs(velocity) < abs(startup_velocity):\n            raise ValueError(\n                "sensorless speed must remain at or above {:.3f} turns/s".format(\n                    abs(startup_velocity)\n                )\n            )\n        velocity_limit = self.read_float(\n            prefix + "controller.config.vel_limit", log=False\n        )\n        if abs(velocity) > velocity_limit:\n            raise ValueError(\n                "speed exceeds configured limit {:.3f} turns/s".format(\n                    velocity_limit\n                )\n            )\n        self.set_velocity(axis, velocity)\n        return {\n            "axis": axis,\n            "velocity_turns_s": velocity,\n            "minimum_turns_s": abs(startup_velocity),\n        }\n\n    def stop_sensorless(self, axis):\n        """Stop sensorless operation immediately by returning the axis to IDLE."""\n        axis = self._axis(axis)\n        self.request_state(axis, AXIS_STATE_IDLE)\n        time.sleep(0.10)\n        return self.axis_snapshot(axis)\n\n    def calibrate_motor(self, axis, timeout_s=25.0):\n        """Run motor-only calibration for an encoderless sensorless axis."""\n        axis = self._axis(axis)\n        timeout_s = float(timeout_s)\n        prefix = "axis{}.".format(axis)\n        self.request_state(axis, AXIS_STATE_IDLE)\n        self.clear_errors(axis)\n        self.request_state(axis, AXIS_STATE_MOTOR_CALIBRATION)\n        deadline = time.monotonic() + timeout_s\n        entered_calibration = False\n        while time.monotonic() < deadline:\n            state = self.read_int(prefix + "current_state", log=False)\n            if state == AXIS_STATE_MOTOR_CALIBRATION:\n                entered_calibration = True\n            if entered_calibration and state == AXIS_STATE_IDLE:\n                snapshot = self.axis_snapshot(axis)\n                if snapshot["axis_error"] or snapshot["motor_error"]:\n                    raise RuntimeError(\n                        "ODESC axis {} motor calibration failed: {}".format(\n                            axis, snapshot\n                        )\n                    )\n                return snapshot\n            time.sleep(0.25)\n        self.request_state(axis, AXIS_STATE_IDLE)\n        raise TimeoutError(\n            "ODESC axis {} motor calibration did not finish within {} seconds".format(\n                axis, timeout_s\n            )\n        )\n\n    def calibrate_axis(self, axis, timeout_s=35.0):\n        """Run the ODrive full calibration sequence and return final diagnostics."""\n        axis = self._axis(axis)\n        timeout_s = float(timeout_s)\n        self.stop_axis(axis)\n        self.clear_errors(axis)\n        self.request_state(axis, AXIS_STATE_FULL_CALIBRATION_SEQUENCE)\n        deadline = time.monotonic() + timeout_s\n        entered_calibration = False\n        while time.monotonic() < deadline:\n            state = self.read_int(\n                "axis{}.current_state".format(axis), log=False\n            )\n            if state == AXIS_STATE_FULL_CALIBRATION_SEQUENCE:\n                entered_calibration = True\n            if entered_calibration and state == AXIS_STATE_IDLE:\n                snapshot = self.axis_snapshot(axis)\n                if any(\n                    snapshot[name]\n                    for name in (\n                        "axis_error",\n                        "motor_error",\n                        "encoder_error",\n                        "controller_error",\n                    )\n                ):\n                    raise RuntimeError(\n                        "ODESC axis {} calibration failed: {}".format(\n                            axis, snapshot\n                        )\n                    )\n                return snapshot\n            time.sleep(0.25)\n        self.request_state(axis, AXIS_STATE_IDLE)\n        raise TimeoutError(\n            "ODESC axis {} calibration did not finish within {} seconds".format(\n                axis, timeout_s\n            )\n        )\n\n    def check_connection(self):\n        """Verify both UART directions and return a configuration snapshot."""\n        snapshot = {\n            "vbus_voltage": self.read_float("vbus_voltage"),\n            "uart_enabled": self.read_int("config.enable_uart"),\n            "uart_baudrate": self.read_int("config.uart_baudrate"),\n            "axis1_state": self.read_int("axis1.current_state"),\n            "axis1_error": self.read_int("axis1.error"),\n        }\n        if snapshot["uart_enabled"] != 1:\n            raise RuntimeError("ODESC reports that UART is disabled")\n        if snapshot["uart_baudrate"] != self.baudrate:\n            raise RuntimeError(\n                "ODESC baud is {}, but client baud is {}".format(\n                    snapshot["uart_baudrate"], self.baudrate\n                )\n            )\n\n        print("ODESC UART ROUND TRIP VERIFIED")\n        print("Bus voltage: {:.6f} V".format(snapshot["vbus_voltage"]))\n        print(\n            "UART enabled={} baud={} | axis1 state={} error={}".format(\n                snapshot["uart_enabled"],\n                snapshot["uart_baudrate"],\n                snapshot["axis1_state"],\n                snapshot["axis1_error"],\n            )\n        )\n        return snapshot\n\n\ndef run_self_test(device=UART_DEVICE, baudrate=UART_BAUD):\n    controller = ODESCUART(device, baudrate)\n    try:\n        controller.open()\n        return controller.check_connection()\n    finally:\n        controller.close()\n        print("ODESC UART closed")\n\n\nif __name__ == "__main__":\n    try:\n        run_self_test()\n    except Exception as exc:\n        print("ODESC UART TEST FAILED: {}".format(exc))\n        print("Check crossed TX/RX, common ground, /dev/ttyAMA2, and 115200 baud.")\n'
odesc_driver = _embedded_module('odesc_driver', odesc_driver_source)

gd32_driver_source = '"""Raspberry Pi 5 UART client for the Ender-3 GD32 stepper controller.\n\nRun this file with Thonny\'s local Python 3 interpreter. The background reader\nkeeps heartbeat acknowledgements, switch reports, and move completions flowing\nwhile the Thonny shell remains available for commands.\n"""\n\nimport atexit\nimport threading\nimport time\n\ntry:\n    import serial\nexcept ImportError as error:\n    raise RuntimeError(\n        "pyserial is required; install python3-serial or the pyserial package"\n    ) from error\n\n\nSERIAL_PORT = "/dev/ttyAMA3"\nBAUDRATE = 115200\nLINK_TIMEOUT_SECONDS = 5.0\nMOTOR_MICROSTEPS_PER_REVOLUTION = 3200.0\nMAX_DRIVER_RPM = 1000.0\nMAX_PENDING_MOVES = 8\nAXIS_STEPS_PER_UNIT = {"X": 80.0, "Y": 80.0, "Z": 400.0, "E": 93.0}\n\n\nclass StepperController:\n    def __init__(self, port=SERIAL_PORT, baudrate=BAUDRATE):\n        self.port = port\n        self.baudrate = baudrate\n        self.serial = None\n        self._running = False\n        self._reader = None\n        self._write_lock = threading.Lock()\n        self._state_lock = threading.Lock()\n        self._move_sequence = 0\n        self._pending_moves = {}\n        self._last_heartbeat = None\n        self._last_round_trip = None\n        self._link_was_up = False\n        self._switch_callback = None\n        self.switches = {"X": None, "Y": None, "Z": None}\n        self.driver_rpm = {"X": 120.0, "Y": 120.0, "Z": 120.0, "E": 120.0}\n        self.position_steps = {"X": 0, "Y": 0, "Z": 0, "E": 0}\n        self.target_steps = {"X": 0, "Y": 0, "Z": 0, "E": 0}\n        self.position_known = {"X": False, "Y": False, "Z": False, "E": False}\n        self.step_limits = {\n            "X": (None, None),\n            "Y": (None, None),\n            "Z": (None, None),\n            "E": (None, None),\n        }\n\n    def start(self):\n        if self._running:\n            return\n        try:\n            self.serial = serial.Serial(\n                self.port,\n                self.baudrate,\n                bytesize=serial.EIGHTBITS,\n                parity=serial.PARITY_NONE,\n                stopbits=serial.STOPBITS_ONE,\n                timeout=0.2,\n                write_timeout=1.0,\n            )\n        except serial.SerialException as error:\n            raise RuntimeError(\n                "Cannot open {}. Check the UART mapping and add your user to "\n                "the dialout group if permission is denied.".format(self.port)\n            ) from error\n\n        self.serial.reset_input_buffer()\n        self._running = True\n        self._reader = threading.Thread(\n            target=self._reader_loop,\n            name="ender3-uart",\n            daemon=True,\n        )\n        self._reader.start()\n        print("Ender UART open on {} at {} baud".format(self.port, self.baudrate))\n\n    def close(self):\n        self._running = False\n        if self.serial is not None and self.serial.is_open:\n            self.serial.close()\n\n    @staticmethod\n    def _axis(axis):\n        axis = str(axis).upper()\n        if axis not in AXIS_STEPS_PER_UNIT:\n            raise ValueError("axis must be X, Y, Z, or E")\n        return axis\n\n    @staticmethod\n    def _validate_rpm(rpm):\n        rpm = float(rpm)\n        if rpm <= 0 or rpm > MAX_DRIVER_RPM:\n            raise ValueError(\n                "rpm must be greater than 0 and at most {}".format(MAX_DRIVER_RPM)\n            )\n        return rpm\n\n    def _send_lines(self, commands):\n        if not self._running or self.serial is None or not self.serial.is_open:\n            raise RuntimeError("UART is not open")\n        payload = "".join(command.strip() + "\\n" for command in commands).encode("ascii")\n        with self._write_lock:\n            self.serial.write(payload)\n            self.serial.flush()\n\n    def send_gcode(self, command):\n        """Send one raw Marlin command. Normal motion should use move()."""\n        self._send_lines((command,))\n\n    def enable_drivers(self):\n        """Enable all drivers; the board has one shared enable signal."""\n        self.send_gcode("M17")\n\n    def disable_drivers(self):\n        """Disable all drivers; the board has one shared enable signal."""\n        self.send_gcode("M18")\n\n    def set_speed(self, axis, rpm):\n        axis = self._axis(axis)\n        rpm = self._validate_rpm(rpm)\n        with self._state_lock:\n            self.driver_rpm[axis] = rpm\n        return rpm\n\n    def reset_position(self, axis):\n        """Define the current open-loop location as step zero."""\n        axis = self._axis(axis)\n        with self._state_lock:\n            if any(record["axis"] == axis for record in self._pending_moves.values()):\n                raise RuntimeError(\n                    "cannot reset {} while that axis has a pending move".format(axis)\n                )\n            self.position_steps[axis] = 0\n            self.target_steps[axis] = 0\n            self.position_known[axis] = True\n        return self.axis_position(axis)\n\n    def set_limits(self, axis, minimum_steps, maximum_steps):\n        """Set inclusive host-side limits relative to the defined zero."""\n        axis = self._axis(axis)\n        minimum_steps = int(minimum_steps)\n        maximum_steps = int(maximum_steps)\n        if minimum_steps >= maximum_steps:\n            raise ValueError("minimum step limit must be less than maximum")\n        with self._state_lock:\n            if self.position_known[axis] and not (\n                minimum_steps <= self.target_steps[axis] <= maximum_steps\n            ):\n                raise ValueError(\n                    "{} current target is outside the requested limits".format(axis)\n                )\n            self.step_limits[axis] = (minimum_steps, maximum_steps)\n        return self.axis_position(axis)\n\n    def axis_position(self, axis):\n        axis = self._axis(axis)\n        with self._state_lock:\n            minimum, maximum = self.step_limits[axis]\n            return {\n                "axis": axis,\n                "position_steps": self.position_steps[axis],\n                "target_steps": self.target_steps[axis],\n                "known": self.position_known[axis],\n                "minimum_steps": minimum,\n                "maximum_steps": maximum,\n            }\n\n    def move(self, axis, steps, rpm=None, wait=False, timeout=None):\n        """Queue signed driver microsteps; sign selects the direction."""\n        axis = self._axis(axis)\n        steps = int(steps)\n        if steps == 0:\n            raise ValueError("steps must not be zero")\n\n        with self._state_lock:\n            rpm = self.driver_rpm[axis] if rpm is None else rpm\n            rpm = self._validate_rpm(rpm)\n            if len(self._pending_moves) >= MAX_PENDING_MOVES:\n                raise RuntimeError("too many queued moves; wait for DRV_DONE")\n            target_steps = self.target_steps[axis] + steps\n            minimum_steps, maximum_steps = self.step_limits[axis]\n            if minimum_steps is not None or maximum_steps is not None:\n                if not self.position_known[axis]:\n                    raise RuntimeError(\n                        "{} position is unknown; reset its zero before using limits".format(\n                            axis\n                        )\n                    )\n                if (\n                    minimum_steps is not None\n                    and target_steps < minimum_steps\n                ) or (\n                    maximum_steps is not None\n                    and target_steps > maximum_steps\n                ):\n                    raise ValueError(\n                        "{} target {} is outside limits {}..{}".format(\n                            axis, target_steps, minimum_steps, maximum_steps\n                        )\n                    )\n            self._move_sequence += 1\n            move_id = self._move_sequence\n            record = {\n                "axis": axis,\n                "steps": steps,\n                "event": threading.Event(),\n                "completed": False,\n            }\n            self._pending_moves[move_id] = record\n            self.target_steps[axis] = target_steps\n\n        steps_per_unit = AXIS_STEPS_PER_UNIT[axis]\n        distance_units = steps / steps_per_unit\n        feedrate_units_min = rpm * MOTOR_MICROSTEPS_PER_REVOLUTION / steps_per_unit\n        commands = ["G91"]\n        if axis == "E":\n            commands.append("M83")\n        commands.extend(\n            (\n                "G0 {}{:.5f} F{:.2f}".format(axis, distance_units, feedrate_units_min),\n                "M400",\n                "M118 DRV_DONE {} {} {}".format(move_id, axis, steps),\n            )\n        )\n\n        try:\n            self._send_lines(commands)\n        except Exception:\n            with self._state_lock:\n                self._pending_moves.pop(move_id, None)\n                self.target_steps[axis] -= steps\n            raise\n\n        print(\n            "Queued {} move {}: {} microsteps at {} RPM".format(\n                axis, move_id, steps, rpm\n            )\n        )\n        if wait:\n            if not record["event"].wait(timeout):\n                raise TimeoutError("move {} did not complete before timeout".format(move_id))\n            if not record["completed"]:\n                raise RuntimeError("move {} was stopped before completion".format(move_id))\n        return move_id\n\n    def move_sps(self, axis, steps, steps_per_second, wait=False, timeout=None):\n        steps_per_second = float(steps_per_second)\n        if steps_per_second <= 0:\n            raise ValueError("steps_per_second must be greater than zero")\n        rpm = steps_per_second * 60.0 / MOTOR_MICROSTEPS_PER_REVOLUTION\n        return self.move(axis, steps, rpm, wait=wait, timeout=timeout)\n\n    def stop(self):\n        """Quick-stop planner motion and mark pending host transactions stopped."""\n        with self._state_lock:\n            records = tuple(self._pending_moves.values())\n            self._pending_moves.clear()\n            affected_axes = {record["axis"] for record in records}\n            for axis in affected_axes:\n                self.target_steps[axis] = self.position_steps[axis]\n                self.position_known[axis] = False\n        for record in records:\n            record["event"].set()\n        self.send_gcode("M410")\n        print("Quick stop requested")\n\n    def forward(self, axis, steps, rpm=None, wait=False, timeout=None):\n        return self.move(axis, abs(int(steps)), rpm, wait=wait, timeout=timeout)\n\n    def backward(self, axis, steps, rpm=None, wait=False, timeout=None):\n        return self.move(axis, -abs(int(steps)), rpm, wait=wait, timeout=timeout)\n\n    def on_switch_change(self, callback):\n        """Register callback(states) or pass None to remove it."""\n        if callback is not None and not callable(callback):\n            raise TypeError("callback must be callable or None")\n        with self._state_lock:\n            self._switch_callback = callback\n\n    def switch_status(self):\n        with self._state_lock:\n            states = dict(self.switches)\n        print("Switches X={} Y={} Z={}".format(states["X"], states["Y"], states["Z"]))\n        return states\n\n    def status(self):\n        snapshot = self.link_snapshot()\n        print(\n            "UART {} link_rx={} round_trip={} pending={}".format(\n                self.port,\n                "up" if snapshot["receiving"] else "down",\n                "up" if snapshot["round_trip"] else "down",\n                snapshot["pending_moves"],\n            )\n        )\n        speeds = snapshot["driver_rpm"]\n        print(\n            "Default RPM X={} Y={} Z={} E={}".format(\n                speeds["X"], speeds["Y"], speeds["Z"], speeds["E"]\n            )\n        )\n        return self.switch_status()\n\n    def link_snapshot(self):\n        """Return link state without printing or sending a command."""\n        now = time.monotonic()\n        with self._state_lock:\n            last_heartbeat = self._last_heartbeat\n            last_round_trip = self._last_round_trip\n            pending = len(self._pending_moves)\n            speeds = dict(self.driver_rpm)\n            switches = dict(self.switches)\n            positions = {\n                axis: {\n                    "position_steps": self.position_steps[axis],\n                    "target_steps": self.target_steps[axis],\n                    "known": self.position_known[axis],\n                    "minimum_steps": self.step_limits[axis][0],\n                    "maximum_steps": self.step_limits[axis][1],\n                }\n                for axis in "XYZE"\n            }\n        receiving = last_heartbeat is not None and now - last_heartbeat <= LINK_TIMEOUT_SECONDS\n        round_trip = (\n            last_round_trip is not None and now - last_round_trip <= LINK_TIMEOUT_SECONDS\n        )\n        return {\n            "receiving": receiving,\n            "round_trip": round_trip,\n            "pending_moves": pending,\n            "driver_rpm": speeds,\n            "switches": switches,\n            "positions": positions,\n        }\n\n    def check_connection(self, timeout=6.0):\n        """Wait for a heartbeat acknowledgement round trip."""\n        deadline = time.monotonic() + float(timeout)\n        while time.monotonic() < deadline:\n            snapshot = self.link_snapshot()\n            if snapshot["round_trip"]:\n                print("GD32 UART ROUND TRIP VERIFIED")\n                return True\n            time.sleep(0.05)\n        raise TimeoutError("No GD32 heartbeat round trip on {}".format(self.port))\n\n    def _reader_loop(self):\n        while self._running:\n            try:\n                raw_line = self.serial.readline()\n            except serial.SerialException as error:\n                if self._running:\n                    print("Ender UART read error:", error)\n                self._running = False\n                break\n            if raw_line:\n                line = raw_line.decode("ascii", errors="replace").strip()\n                if line:\n                    self._handle_line(line)\n            self._check_link_timeout()\n\n    def _handle_line(self, line):\n        if line.startswith("HB "):\n            sequence = line[3:].strip()\n            with self._state_lock:\n                self._last_heartbeat = time.monotonic()\n            self.send_gcode("M118 HB_ACK " + sequence)\n            return\n\n        if line.startswith("HB_ACK_OK"):\n            with self._state_lock:\n                self._last_round_trip = time.monotonic()\n                announce = not self._link_was_up\n                self._link_was_up = True\n            if announce:\n                print("Ender UART round trip verified")\n            return\n\n        if line.startswith("DRV_DONE "):\n            self._handle_move_complete(line)\n            return\n\n        if line.startswith("SW "):\n            self._handle_switch_report(line)\n            return\n\n        if line == "ok" or line.startswith("HB_ACK "):\n            return\n        print("Ender:", line)\n\n    def _handle_move_complete(self, line):\n        try:\n            _, move_id_text, axis, steps_text = line.split()\n            move_id = int(move_id_text)\n            steps = int(steps_text)\n        except (ValueError, IndexError):\n            print("Malformed driver completion:", line)\n            return\n\n        with self._state_lock:\n            record = self._pending_moves.pop(move_id, None)\n        if record is None or record["axis"] != axis or record["steps"] != steps:\n            print("Unexpected driver completion:", line)\n            return\n        with self._state_lock:\n            self.position_steps[axis] += steps\n        record["completed"] = True\n        record["event"].set()\n        print("{} move {} complete: {} microsteps".format(axis, move_id, steps))\n\n    def _handle_switch_report(self, line):\n        try:\n            fields = line.split()\n            if len(fields) != 4:\n                raise ValueError\n            new_states = {field[0]: bool(int(field[1:])) for field in fields[1:]}\n            if set(new_states) != {"X", "Y", "Z"}:\n                raise ValueError\n        except (ValueError, IndexError):\n            print("Malformed switch report:", line)\n            return\n\n        with self._state_lock:\n            changed = any(self.switches[axis] != new_states[axis] for axis in "XYZ")\n            self.switches.update(new_states)\n            callback = self._switch_callback\n            states = dict(self.switches)\n        if changed:\n            print("Switches X={} Y={} Z={}".format(states["X"], states["Y"], states["Z"]))\n            if callback is not None:\n                try:\n                    callback(states)\n                except Exception as error:\n                    print("Switch callback error:", error)\n\n    def _check_link_timeout(self):\n        with self._state_lock:\n            if (\n                self._link_was_up\n                and self._last_round_trip is not None\n                and time.monotonic() - self._last_round_trip > LINK_TIMEOUT_SECONDS\n            ):\n                self._link_was_up = False\n                announce = True\n            else:\n                announce = False\n        if announce:\n            print("Ender UART round trip lost")\n\n\ncontroller = StepperController()\natexit.register(controller.close)\n\n\ndef move(axis, steps, rpm=None, wait=False, timeout=None):\n    return controller.move(axis, steps, rpm, wait=wait, timeout=timeout)\n\n\ndef move_sps(axis, steps, steps_per_second, wait=False, timeout=None):\n    return controller.move_sps(\n        axis, steps, steps_per_second, wait=wait, timeout=timeout\n    )\n\n\ndef xmove(steps, rpm=None, wait=False, timeout=None):\n    return move("X", steps, rpm, wait=wait, timeout=timeout)\n\n\ndef ymove(steps, rpm=None, wait=False, timeout=None):\n    return move("Y", steps, rpm, wait=wait, timeout=timeout)\n\n\ndef zmove(steps, rpm=None, wait=False, timeout=None):\n    return move("Z", steps, rpm, wait=wait, timeout=timeout)\n\n\ndef emove(steps, rpm=None, wait=False, timeout=None):\n    return move("E", steps, rpm, wait=wait, timeout=timeout)\n\n\ndef forward(axis, steps, rpm=None, wait=False, timeout=None):\n    return controller.forward(axis, steps, rpm, wait=wait, timeout=timeout)\n\n\ndef backward(axis, steps, rpm=None, wait=False, timeout=None):\n    return controller.backward(axis, steps, rpm, wait=wait, timeout=timeout)\n\n\ndef set_speed(axis, rpm):\n    return controller.set_speed(axis, rpm)\n\n\ndef enable_drivers():\n    return controller.enable_drivers()\n\n\nenable_steppers = enable_drivers\n\n\ndef disable_drivers():\n    return controller.disable_drivers()\n\n\ndisable_steppers = disable_drivers\n\n\ndef stop():\n    return controller.stop()\n\n\ndef status():\n    return controller.status()\n\n\ndef switch_status():\n    return controller.switch_status()\n\n\ndef on_switch_change(callback):\n    return controller.on_switch_change(callback)\n\n\ndef send_gcode(command):\n    return controller.send_gcode(command)\n\n\ndef close():\n    return controller.close()\n\n\ndef show_help():\n    print("Thonny shell commands:")\n    print("  move(\'X\', 3200, 120) | move(\'E\', -800, 60)")\n    print("  xmove(...) | ymove(...) | zmove(...) | emove(...)")\n    print("  move_sps(\'Y\', -1600, 8000) | set_speed(\'Z\', 30)")\n    print("  forward(\'X\', 800, 60) | backward(\'X\', 800, 60)")\n    print("  stop() | enable_drivers() | disable_drivers() | status()")\n    print("Add wait=True to block until DRV_DONE, with optional timeout seconds.")\n    print("Driver current and microstep mode are set by board hardware, not UART.")\n\n\nif __name__ == "__main__":\n    controller.start()\n    print("Raspberry Pi stepper client started; returning to the Thonny prompt.")\n    show_help()\n'
gd32_driver = _embedded_module('gd32_driver', gd32_driver_source)

bno080_driver_source = '"""Standalone BNO080 UART-SHTP client for Raspberry Pi 5 and Thonny.\n\nDependency: pyserial (Raspberry Pi OS package ``python3-serial``).\n\nDefault wiring:\n    Pi GPIO12 / physical 32 / UART4 TX -> BNO080 RX\n    Pi GPIO13 / physical 33 / UART4 RX <- BNO080 TX\n    Pi GND                              -> BNO080 GND\n\nSet the BNO080 interface mode before power-up: PS1 high and PS0 low.\nThis is normal Python 3 code, not MicroPython code.\n"""\n\nimport math\nimport struct\nimport time\n\ntry:\n    import serial\nexcept ImportError as exc:\n    raise ImportError("Install Raspberry Pi OS package python3-serial") from exc\n\n\nUART_DEVICE = "/dev/ttyAMA4"\nUART_BAUD = 3_000_000\nUART_TX_GPIO = 12\nUART_RX_GPIO = 13\n\n# SH-2 report IDs used by this client.\nREPORT_ACCELEROMETER = 0x01\nREPORT_GYROSCOPE = 0x02\nREPORT_MAGNETOMETER = 0x03\nREPORT_LINEAR_ACCELERATION = 0x04\nREPORT_ROTATION_VECTOR = 0x05\nREPORT_RAW_ACCELEROMETER = 0x14\nREPORT_RAW_GYROSCOPE = 0x15\nREPORT_RAW_MAGNETOMETER = 0x16\nREPORT_BASE_TIMESTAMP = 0xFB\nSET_FEATURE_COMMAND = 0xFD\nCOMMAND_REQUEST = 0xF2\nME_CALIBRATE_COMMAND = 0x07\nME_CAL_CONFIG = 0x00\n\nCHANNEL_CONTROL = 2\nCHANNEL_INPUT_NORMAL = 3\nCHANNEL_INPUT_WAKE = 4\nCHANNEL_EXECUTABLE = 1\n\n# Report interval values are microseconds.\nNAVIGATION_REPORTS = (\n    (REPORT_ROTATION_VECTOR, 20_000),\n    (REPORT_MAGNETOMETER, 20_000),\n    (REPORT_ACCELEROMETER, 20_000),\n    (REPORT_LINEAR_ACCELERATION, 20_000),\n    (REPORT_GYROSCOPE, 10_000),\n)\nRAW_REPORTS = (\n    (REPORT_RAW_ACCELEROMETER, 20_000),\n    (REPORT_RAW_GYROSCOPE, 20_000),\n    (REPORT_RAW_MAGNETOMETER, 20_000),\n)\n\nREPORT_FORMAT = {\n    REPORT_ACCELEROMETER: (10, 3, 2.0 ** -8, True, "acceleration_m_s2"),\n    REPORT_GYROSCOPE: (10, 3, 2.0 ** -9, True, "gyro_rad_s"),\n    REPORT_MAGNETOMETER: (10, 3, 2.0 ** -4, True, "magnetic_uT"),\n    REPORT_LINEAR_ACCELERATION: (10, 3, 2.0 ** -8, True, "linear_acceleration_m_s2"),\n    REPORT_ROTATION_VECTOR: (14, 4, 2.0 ** -14, True, "quaternion"),\n    REPORT_RAW_ACCELEROMETER: (16, 3, 1.0, False, "raw_acceleration"),\n    REPORT_RAW_GYROSCOPE: (16, 3, 1.0, False, "raw_gyro"),\n    REPORT_RAW_MAGNETOMETER: (16, 3, 1.0, False, "raw_magnetic"),\n}\n\nACCURACY_LABELS = ("unreliable", "low", "medium", "high")\n\n\nclass BNO080UART:\n    """Small UART-SHTP/SH-2 implementation for selected BNO080 reports."""\n\n    def __init__(self, device=UART_DEVICE, baudrate=UART_BAUD):\n        self.device = str(device)\n        self.baudrate = int(baudrate)\n        self.port = None\n        self.tx_sequence = [0] * 6\n        self.command_sequence = 0\n        self.readings = {}\n        self.accuracy = {}\n        self.report_sequence = {}\n        self.last_packet_monotonic = None\n        self.raw_reports_enabled = False\n\n    def open(self):\n        self.close()\n        self.port = serial.Serial(\n            port=self.device,\n            baudrate=self.baudrate,\n            bytesize=serial.EIGHTBITS,\n            parity=serial.PARITY_NONE,\n            stopbits=serial.STOPBITS_ONE,\n            timeout=0.05,\n            write_timeout=1.0,\n        )\n        self.port.reset_input_buffer()\n        print(\n            "BNO080 UART opened: {} at {} baud (Pi TX GPIO{}, RX GPIO{})".format(\n                self.device, self.baudrate, UART_TX_GPIO, UART_RX_GPIO\n            )\n        )\n\n        available = self._request_buffer(2.0)\n        if available is None:\n            self.close()\n            raise RuntimeError(\n                "No UART-SHTP response; check TX/RX, ground, power, PS1/PS0"\n            )\n        print("BNO080 UART-SHTP ROUND TRIP VERIFIED: buffer={} bytes".format(available))\n\n        self.soft_reset()\n        self._enable_navigation_reports()\n\n        started = time.monotonic()\n        retry_at = started + 2.0\n        deadline = started + 6.0\n        retried = False\n        while time.monotonic() < deadline:\n            self.poll(0.25)\n            if REPORT_ROTATION_VECTOR in self.readings and REPORT_MAGNETOMETER in self.readings:\n                print("BNO080 initialized; fused navigation reports received")\n                try:\n                    self.begin_calibration()\n                except Exception as exc:\n                    print(\n                        "BNO080 calibration start failed; sensor remains connected: {}".format(\n                            exc\n                        )\n                    )\n                return self\n            if not retried and time.monotonic() >= retry_at:\n                retried = True\n                print("No fused reports yet; retrying BNO080 feature setup")\n                self._enable_navigation_reports()\n        self.close()\n        raise RuntimeError("BNO080 accepted SHTP but no fused reports arrived")\n\n    def _enable_navigation_reports(self):\n        for report_id, interval_us in NAVIGATION_REPORTS:\n            self.enable_report(report_id, interval_us)\n\n    def soft_reset(self):\n        """Reset SH-2 state so repeated host runs restart channel sequencing."""\n        print("Resetting BNO080 SHTP state...")\n        self._send_shtp(CHANNEL_EXECUTABLE, b"\\x01")\n        time.sleep(0.5)\n        self._send_shtp(CHANNEL_EXECUTABLE, b"\\x01")\n        time.sleep(0.5)\n        self.readings.clear()\n        self.accuracy.clear()\n        self.report_sequence.clear()\n        self.poll(0.25)\n        print("BNO080 SHTP reset complete")\n\n    def close(self):\n        if self.port is not None:\n            try:\n                self.port.close()\n            finally:\n                self.port = None\n\n    @staticmethod\n    def _escape(data):\n        encoded = bytearray()\n        for value in data:\n            if value in (0x7D, 0x7E):\n                encoded.extend((0x7D, value ^ 0x20))\n            else:\n                encoded.append(value)\n        return encoded\n\n    def _slow_write(self, data):\n        """Write with the minimum UART-SHTP host inter-byte delay."""\n        for value in data:\n            self.port.write(bytes((value,)))\n            self.port.flush()\n            time.sleep(0.001)\n\n    def _write_uart_frame(self, protocol_id, payload=b""):\n        body = bytes((protocol_id,)) + bytes(payload)\n        frame = b"\\x7e" + self._escape(body) + b"\\x7e"\n        self._slow_write(frame)\n\n    def _read_uart_frame(self, timeout_s):\n        deadline = time.monotonic() + float(timeout_s)\n        frame = bytearray()\n        in_frame = False\n        escaped = False\n\n        while time.monotonic() < deadline:\n            data = self.port.read(1)\n            if not data:\n                continue\n            value = data[0]\n\n            if value == 0x7E:\n                if in_frame and frame:\n                    return bytes(frame)\n                frame.clear()\n                in_frame = True\n                escaped = False\n                continue\n            if not in_frame:\n                continue\n            if escaped:\n                frame.append(value ^ 0x20)\n                escaped = False\n            elif value == 0x7D:\n                escaped = True\n            else:\n                frame.append(value)\n        return None\n\n    def _request_buffer(self, timeout_s=1.0):\n        """Send Buffer Status Query and return granted host-write bytes."""\n        self._write_uart_frame(0x00)\n        deadline = time.monotonic() + float(timeout_s)\n        while time.monotonic() < deadline:\n            frame = self._read_uart_frame(deadline - time.monotonic())\n            if frame is None:\n                break\n            protocol_id = frame[0]\n            payload = frame[1:]\n            if protocol_id == 0x00 and len(payload) >= 2:\n                return payload[0] | (payload[1] << 8)\n            if protocol_id == 0x01:\n                self._handle_shtp(payload)\n        return None\n\n    def _send_shtp(self, channel, payload):\n        if not 0 <= channel < len(self.tx_sequence):\n            raise ValueError("Unsupported SHTP channel")\n        payload = bytes(payload)\n        cargo_length = len(payload) + 4\n        available = self._request_buffer(1.0)\n        if available is None or available < cargo_length:\n            raise RuntimeError(\n                "BNO080 did not grant {} SHTP write bytes".format(cargo_length)\n            )\n\n        sequence = self.tx_sequence[channel]\n        header = struct.pack("<HBB", cargo_length, channel, sequence)\n        self._write_uart_frame(0x01, header + payload)\n        self.tx_sequence[channel] = (sequence + 1) & 0xFF\n\n    def enable_report(self, report_id, interval_us):\n        """Send an SH-2 Set Feature command for one periodic sensor report."""\n        payload = struct.pack(\n            "<BBBHIII",\n            SET_FEATURE_COMMAND,\n            int(report_id),\n            0,                 # feature flags\n            0,                 # change sensitivity\n            int(interval_us),\n            0,                 # batch interval\n            0,                 # sensor-specific configuration\n        )\n        self._send_shtp(CHANNEL_CONTROL, payload)\n\n    def enable_raw_reports(self):\n        for report_id, interval_us in RAW_REPORTS:\n            self.enable_report(report_id, interval_us)\n        self.raw_reports_enabled = True\n        print("BNO080 raw accel/gyro/magnetometer reports enabled")\n\n    def _send_command(self, command, parameters=()):\n        parameters = tuple(int(value) & 0xFF for value in parameters)\n        if len(parameters) > 9:\n            raise ValueError("SH-2 commands accept at most nine parameters")\n        payload = bytearray(12)\n        payload[0] = COMMAND_REQUEST\n        payload[1] = self.command_sequence\n        payload[2] = int(command) & 0xFF\n        payload[3 : 3 + len(parameters)] = bytes(parameters)\n        self._send_shtp(CHANNEL_CONTROL, payload)\n        self.command_sequence = (self.command_sequence + 1) & 0xFF\n\n    def begin_calibration(self):\n        """Enable dynamic accelerometer, gyroscope, and magnetometer calibration."""\n        self._send_command(\n            ME_CALIBRATE_COMMAND,\n            (1, 1, 1, ME_CAL_CONFIG, 0, 0, 0, 0, 0),\n        )\n        print("BNO080 dynamic accel/gyro/magnetometer calibration started")\n\n    def poll(self, timeout_s=0.1):\n        """Receive and process UART frames for up to timeout_s."""\n        deadline = time.monotonic() + float(timeout_s)\n        processed = 0\n        while time.monotonic() < deadline:\n            frame = self._read_uart_frame(min(0.05, deadline - time.monotonic()))\n            if frame is None:\n                continue\n            if frame[0] == 0x01:\n                self._handle_shtp(frame[1:])\n                processed += 1\n        return processed\n\n    def _handle_shtp(self, cargo):\n        if len(cargo) < 4:\n            return\n        packet_length, channel, _sequence = struct.unpack_from("<HBB", cargo)\n        packet_length &= 0x7FFF\n        if packet_length < 4 or packet_length > len(cargo):\n            return\n        payload = cargo[4:packet_length]\n        self.last_packet_monotonic = time.monotonic()\n        if channel in (CHANNEL_INPUT_NORMAL, CHANNEL_INPUT_WAKE):\n            self._parse_report_batch(payload)\n\n    def _parse_report_batch(self, payload):\n        offset = 0\n        while offset < len(payload):\n            report_id = payload[offset]\n            if report_id == REPORT_BASE_TIMESTAMP:\n                report_length = 5\n            elif report_id in REPORT_FORMAT:\n                report_length = REPORT_FORMAT[report_id][0]\n            else:\n                # Control/advertisement reports are irrelevant to sensor output.\n                return\n            if offset + report_length > len(payload):\n                return\n            report = payload[offset : offset + report_length]\n            if report_id in REPORT_FORMAT:\n                self._parse_sensor_report(report)\n            offset += report_length\n\n    def _parse_sensor_report(self, report):\n        report_id = report[0]\n        _length, count, scalar, signed, name = REPORT_FORMAT[report_id]\n        values = []\n        fmt = "<h" if signed else "<H"\n        for index in range(count):\n            raw = struct.unpack_from(fmt, report, 4 + index * 2)[0]\n            values.append(raw * scalar)\n        self.readings[report_id] = tuple(values)\n        self.accuracy[report_id] = report[2] & 0x03\n        self.report_sequence[report_id] = report[1]\n        self.readings[name] = tuple(values)\n\n    def snapshot(self, timeout_s=0.2):\n        self.poll(timeout_s)\n        required = (\n            "quaternion",\n            "magnetic_uT",\n            "gyro_rad_s",\n            "acceleration_m_s2",\n            "linear_acceleration_m_s2",\n        )\n        missing = [name for name in required if name not in self.readings]\n        if missing:\n            raise RuntimeError("Missing BNO080 reports: " + ", ".join(missing))\n        quaternion = self.readings["quaternion"]\n        return {\n            "heading_deg": quaternion_to_heading(quaternion),\n            "quaternion": quaternion,\n            "magnetic_uT": self.readings["magnetic_uT"],\n            "gyro_rad_s": self.readings["gyro_rad_s"],\n            "acceleration_m_s2": self.readings["acceleration_m_s2"],\n            "linear_acceleration_m_s2": self.readings["linear_acceleration_m_s2"],\n            "magnetometer_accuracy": self.accuracy.get(REPORT_MAGNETOMETER, 0),\n            "monotonic_s": time.monotonic(),\n        }\n\n    def raw_snapshot(self, timeout_s=0.2):\n        if not self.raw_reports_enabled:\n            raise RuntimeError("Raw reports are disabled; call enable_raw_reports()")\n        self.poll(timeout_s)\n        required = ("raw_acceleration", "raw_gyro", "raw_magnetic")\n        missing = [name for name in required if name not in self.readings]\n        if missing:\n            raise RuntimeError("Missing raw reports: " + ", ".join(missing))\n        return {name: self.readings[name] for name in required}\n\n\ndef quaternion_to_heading(quaternion):\n    """Convert (i, j, k, real) rotation quaternion to 0..360 degree yaw."""\n    i, j, k, real = quaternion\n    yaw = math.atan2(\n        2.0 * (real * k + i * j),\n        1.0 - 2.0 * (j * j + k * k),\n    )\n    return math.degrees(yaw) % 360.0\n\n\nsensor = None\n\n\ndef connect(device=UART_DEVICE, baudrate=UART_BAUD):\n    global sensor\n    disconnect()\n    sensor = BNO080UART(device, baudrate)\n    try:\n        sensor.open()\n    except Exception:\n        sensor = None\n        raise\n    return sensor\n\n\ndef disconnect():\n    global sensor\n    if sensor is not None:\n        sensor.close()\n        sensor = None\n\n\ndef _sensor():\n    if sensor is None:\n        raise RuntimeError("BNO080 is not connected; call connect()")\n    return sensor\n\n\ndef sample():\n    return _sensor().snapshot()\n\n\ndef magnetometer():\n    """Return calibrated (x, y, z) magnetic field in microtesla."""\n    reading = _sensor().snapshot()\n    return reading["magnetic_uT"]\n\n\ndef print_magnetometer():\n    """Print calibrated magnetic field, fused heading, and accuracy."""\n    reading = _sensor().snapshot()\n    values = reading["magnetic_uT"]\n    accuracy = reading["magnetometer_accuracy"]\n    print(\n        "mag x={:+.4f} y={:+.4f} z={:+.4f} uT | heading={:.2f} deg | accuracy={} ({})".format(\n            values[0], values[1], values[2], reading["heading_deg"],\n            accuracy, ACCURACY_LABELS[accuracy]\n        )\n    )\n    return values\n\n\ndef stream_magnetometer(rate_hz=5.0, seconds=None):\n    """Print only magnetometer and heading data until Ctrl+C or a time limit."""\n    rate_hz = float(rate_hz)\n    if rate_hz <= 0 or rate_hz > 20:\n        raise ValueError("rate_hz must be greater than 0 and at most 20")\n    deadline = None if seconds is None else time.monotonic() + float(seconds)\n    interval = 1.0 / rate_hz\n    print("Streaming BNO080 magnetometer data; press Ctrl+C to stop")\n    try:\n        while deadline is None or time.monotonic() < deadline:\n            started = time.monotonic()\n            print_magnetometer()\n            time.sleep(max(0.0, interval - (time.monotonic() - started)))\n    except KeyboardInterrupt:\n        print("BNO080 magnetometer stream stopped")\n\n\ndef enable_raw_reports():\n    _sensor().enable_raw_reports()\n\n\ndef raw_sample():\n    return _sensor().raw_snapshot()\n\n\ndef _vector(label, values, unit):\n    print(\n        "{} x={:+.4f} y={:+.4f} z={:+.4f} {}".format(\n            label, values[0], values[1], values[2], unit\n        )\n    )\n\n\ndef print_sample(reading=None):\n    if reading is None:\n        reading = sample()\n    q_i, q_j, q_k, q_real = reading["quaternion"]\n    accuracy = reading["magnetometer_accuracy"]\n    print(\n        "heading={:.2f} deg magnetic | mag accuracy={} ({})".format(\n            reading["heading_deg"], accuracy, ACCURACY_LABELS[accuracy]\n        )\n    )\n    print(\n        "quat i={:+.5f} j={:+.5f} k={:+.5f} real={:+.5f}".format(\n            q_i, q_j, q_k, q_real\n        )\n    )\n    _vector("mag", reading["magnetic_uT"], "uT")\n    _vector("gyro", reading["gyro_rad_s"], "rad/s")\n    _vector("accel", reading["acceleration_m_s2"], "m/s^2")\n    _vector("linear", reading["linear_acceleration_m_s2"], "m/s^2")\n    return reading\n\n\ndef print_raw(reading=None):\n    if reading is None:\n        reading = raw_sample()\n    print("raw accel:", reading["raw_acceleration"])\n    print("raw gyro: ", reading["raw_gyro"])\n    print("raw mag:  ", reading["raw_magnetic"])\n    return reading\n\n\ndef stream(rate_hz=5.0, seconds=None):\n    """Print fused navigation data until Ctrl+C or an optional time limit."""\n    rate_hz = float(rate_hz)\n    if rate_hz <= 0 or rate_hz > 20:\n        raise ValueError("rate_hz must be greater than 0 and at most 20")\n    deadline = None if seconds is None else time.monotonic() + float(seconds)\n    interval = 1.0 / rate_hz\n    print("Streaming fused BNO080 data; press Ctrl+C to stop")\n    try:\n        while deadline is None or time.monotonic() < deadline:\n            started = time.monotonic()\n            print_sample()\n            print()\n            time.sleep(max(0.0, interval - (time.monotonic() - started)))\n    except KeyboardInterrupt:\n        print("BNO080 stream stopped")\n\n\ndef stream_raw(rate_hz=5.0, seconds=None):\n    """Enable and print raw vectors until Ctrl+C or an optional time limit."""\n    if not _sensor().raw_reports_enabled:\n        enable_raw_reports()\n    rate_hz = float(rate_hz)\n    if rate_hz <= 0 or rate_hz > 20:\n        raise ValueError("rate_hz must be greater than 0 and at most 20")\n    deadline = None if seconds is None else time.monotonic() + float(seconds)\n    interval = 1.0 / rate_hz\n    print("Streaming raw BNO080 data; press Ctrl+C to stop")\n    try:\n        while deadline is None or time.monotonic() < deadline:\n            started = time.monotonic()\n            print_raw()\n            time.sleep(max(0.0, interval - (time.monotonic() - started)))\n    except KeyboardInterrupt:\n        print("BNO080 raw stream stopped")\n\n\ndef calibration_status():\n    """Print the accuracy field from the latest calibrated magnetometer report."""\n    _sensor().poll(0.2)\n    value = _sensor().accuracy.get(REPORT_MAGNETOMETER, 0)\n    print("Magnetometer calibration: {} ({})".format(value, ACCURACY_LABELS[value]))\n    return value\n\n\ndef begin_calibration():\n    _sensor().begin_calibration()\n\n\ndef show_help():\n    print("BNO080 standalone UART-SHTP shell commands:")\n    print("  sample() | print_sample() | stream(5) | stream(5, 10)")\n    print("  magnetometer() | print_magnetometer() | stream_magnetometer(5, 10)")\n    print("  begin_calibration() | calibration_status()")\n    print("  enable_raw_reports() | raw_sample() | print_raw() | stream_raw(5, 10)")\n    print("  connect(\'/dev/ttyAMA4\', 3000000) | disconnect()")\n\n\nif __name__ == "__main__":\n    try:\n        connect()\n    except Exception as exc:\n        print("BNO080 initialization failed: {}".format(exc))\n        print("Fix wiring/mode, then call connect().")\n\n    show_help()\n'
bno080_driver = _embedded_module('bno080_driver', bno080_driver_source)

class RoverController:
    """Own all rover UART clients and expose a safe commissioning API."""

    def __init__(self):
        self.stm32 = None
        self.odesc = None
        self.gd32 = None
        self.bno080 = None
        self.odesc_snapshot = None
        self.odesc_armed = False
        self.errors = {}
        self._lifecycle_lock = threading.RLock()

    def connect_stm32(self):
        self.disconnect_stm32()
        link = stm32_driver.STM32MotorController(STM32_PORT, ASCII_BAUD).start()
        try:
            reply = link.command("PING", ("OK PONG",), 2.0)
        except Exception:
            link.close()
            raise
        self.stm32 = link
        print("STM32 UART1 ROUND TRIP VERIFIED: " + reply)
        return link.snapshot()

    def connect_odesc(self):
        self.disconnect_odesc()
        link = odesc_driver.ODESCUART(ODESC_PORT, ASCII_BAUD).open()
        try:
            snapshot = link.check_connection()
        except Exception:
            link.close()
            raise
        self.odesc = link
        self.odesc_snapshot = snapshot
        self.odesc_armed = False
        return dict(snapshot)

    def connect_gd32(self):
        self.disconnect_gd32()
        link = gd32_driver.StepperController(GD32_PORT, ASCII_BAUD)
        link.start()
        try:
            link.check_connection(6.0)
        except Exception:
            link.close()
            raise
        self.gd32 = link
        return link.link_snapshot()

    def connect_bno080(self):
        self.disconnect_bno080()
        link = bno080_driver.BNO080UART(BNO080_PORT, BNO080_BAUD)
        try:
            link.open()
            snapshot = link.snapshot()
        except Exception:
            link.close()
            raise
        self.bno080 = link
        return snapshot

    def connect_all(self):
        """Connect every UART independently; one failure does not hide the others."""
        with self._lifecycle_lock:
            results = {}
            self.errors.clear()
            connectors = (
                ("stm32", self.connect_stm32),
                ("odesc", self.connect_odesc),
                ("gd32", self.connect_gd32),
                ("bno080", self.connect_bno080),
            )
            for name, connector in connectors:
                try:
                    connector()
                    results[name] = True
                except Exception as exc:
                    self.errors[name] = str(exc)
                    results[name] = False
                    print("{} connection failed: {}".format(name.upper(), exc))
            self.print_links()
            return results

    def disconnect_stm32(self):
        if self.stm32 is not None:
            self.stm32.close()
            self.stm32 = None

    def disconnect_odesc(self):
        if self.odesc is not None:
            if self.odesc_armed:
                try:
                    self._stop_odesc_unchecked()
                except Exception:
                    pass
            self.odesc.close()
            self.odesc = None
        self.odesc_armed = False

    def disconnect_gd32(self):
        if self.gd32 is not None:
            try:
                self.gd32.stop()
                self.gd32.disable_drivers()
            except Exception:
                pass
            self.gd32.close()
            self.gd32 = None

    def disconnect_bno080(self):
        if self.bno080 is not None:
            self.bno080.close()
            self.bno080 = None

    def close(self):
        """Request motor stops and release all four Linux serial devices."""
        with self._lifecycle_lock:
            had_links = any(
                link is not None
                for link in (self.stm32, self.odesc, self.gd32, self.bno080)
            )
            self.disconnect_odesc()
            self.disconnect_stm32()
            self.disconnect_gd32()
            self.disconnect_bno080()
        if had_links:
            print("All rover UARTs closed")

    def _require(self, name):
        link = getattr(self, name)
        if link is None:
            raise RuntimeError("{} is not connected".format(name.upper()))
        return link

    def link_status(self):
        now = time.monotonic()
        result = {
            "stm32": {"connected": self.stm32 is not None},
            "odesc": {"connected": self.odesc is not None},
            "gd32": {"connected": self.gd32 is not None},
            "bno080": {"connected": self.bno080 is not None},
            "errors": dict(self.errors),
        }
        if self.stm32 is not None:
            result["stm32"].update(self.stm32.snapshot())
        if self.odesc is not None:
            result["odesc"].update(self.odesc_snapshot or {})
            result["odesc"]["armed"] = self.odesc_armed
        if self.gd32 is not None:
            result["gd32"].update(self.gd32.link_snapshot())
        if self.bno080 is not None:
            age = None
            if self.bno080.last_packet_monotonic is not None:
                age = round((now - self.bno080.last_packet_monotonic) * 1000)
            result["bno080"].update(
                alive=age is not None and age <= 1500,
                age_ms=age,
                magnetometer_accuracy=self.bno080.accuracy.get(
                    bno080_driver.REPORT_MAGNETOMETER, 0
                ),
            )
        return result

    def print_links(self):
        links = self.link_status()
        stm = links["stm32"]
        print("UART1 STM32: {}".format("alive" if stm.get("alive") else "down"))
        od = links["odesc"]
        print(
            "UART2 ODESC: {}{}".format(
                "connected" if od["connected"] else "down",
                " (ARMED)" if od.get("armed") else "",
            )
        )
        gd = links["gd32"]
        print("UART3 GD32: {}".format("round trip up" if gd.get("round_trip") else "down"))
        imu = links["bno080"]
        print(
            "UART4 BNO080: {} | mag accuracy={}".format(
                "alive" if imu.get("alive") else "down",
                imu.get("magnetometer_accuracy", "unknown"),
            )
        )
        for name, message in links["errors"].items():
            print("{} error: {}".format(name.upper(), message))
        return links

    # STM32 dual BTS7960 API.
    def drive_motor(self, motor, direction, percent):
        percent = float(percent)
        if not 0.0 <= percent <= 100.0:
            raise ValueError("percent must be from 0 through 100")
        duty = round(percent * stm32_driver.PWM_MAX_DUTY / 100.0)
        return self._require("stm32").command(
            "MOTOR {} {} {}".format(int(motor), str(direction).upper(), duty),
            ("OK MOTOR",),
        )

    def pulse_motor(self, motor, direction="A", percent=6, seconds=0.5):
        seconds = float(seconds)
        if seconds <= 0 or seconds > 5:
            raise ValueError("seconds must be greater than 0 and at most 5")
        try:
            print(self.drive_motor(motor, direction, percent))
            time.sleep(seconds)
        finally:
            print(self.stop_motor(motor))

    def stop_motor(self, motor):
        motor = int(motor)
        return self._require("stm32").command(
            "MSTOP {}".format(motor), ("OK MSTOP",)
        )

    def stop_drive(self):
        return self._require("stm32").command("MSTOP ALL", ("OK MSTOP ALL",))

    def currents(self):
        link = self._require("stm32")
        link.command("MSTATUS", ("MSTAT",))
        return link.snapshot()["motors"]

    def read_encoders(self):
        link = self._require("stm32")
        link.command("ENCREAD", ("ENC",))
        return list(link.snapshot()["encoder_values"])

    def start_encoders(self, rate_hz=50):
        rate_hz = int(rate_hz)
        if rate_hz not in stm32_driver.ENCODER_RATES_HZ:
            raise ValueError(
                "rate_hz must be one of {}".format(stm32_driver.ENCODER_RATES_HZ)
            )
        reply = self._require("stm32").command(
            "ENCON {}".format(rate_hz), ("OK ENCON",)
        )
        return reply

    def stop_encoders(self):
        reply = self._require("stm32").command("ENCOFF", ("OK ENCOFF",))
        return reply

    def encoders(self):
        snapshot = self._require("stm32").snapshot()
        return {
            "pa0": snapshot["encoder_values"][0],
            "pa1": snapshot["encoder_values"][1],
            "pa2": snapshot["encoder_values"][2],
            "pa3": snapshot["encoder_values"][3],
            "sequence": snapshot["encoder_sequence"],
            "stm32_ms": snapshot["encoder_stm32_ms"],
            "reports": snapshot["encoder_count"],
            "dropped": snapshot["encoder_dropped"],
            "age_ms": snapshot["encoder_age_ms"],
            "rate_hz": snapshot["encoder_stream_hz"],
        }

    # GD32 stepper API.
    def step(self, axis, steps, rpm=120, wait=False, timeout=None):
        return self._require("gd32").move(
            axis, steps, rpm, wait=wait, timeout=timeout
        )

    def step_sps(self, axis, steps, steps_per_second, wait=False, timeout=None):
        return self._require("gd32").move_sps(
            axis, steps, steps_per_second, wait=wait, timeout=timeout
        )

    def stop_steppers(self):
        return self._require("gd32").stop()

    def reset_stepper_position(self, axis):
        return self._require("gd32").reset_position(axis)

    def set_stepper_limits(self, axis, minimum_steps, maximum_steps):
        return self._require("gd32").set_limits(
            axis, minimum_steps, maximum_steps
        )

    def stepper_position(self, axis):
        return self._require("gd32").axis_position(axis)

    def switches(self):
        return self._require("gd32").switch_status()

    # BNO080 API.
    def imu(self):
        return self._require("bno080").snapshot()

    def print_imu(self):
        reading = self.imu()
        print("heading={:.2f} deg".format(reading["heading_deg"]))
        print("magnetic_uT={}".format(reading["magnetic_uT"]))
        print("gyro_rad_s={}".format(reading["gyro_rad_s"]))
        print("acceleration_m_s2={}".format(reading["acceleration_m_s2"]))
        print("linear_acceleration_m_s2={}".format(reading["linear_acceleration_m_s2"]))
        print("magnetometer_accuracy={}".format(reading["magnetometer_accuracy"]))
        return reading

    def magnetometer(self):
        reading = self.imu()
        return {
            "magnetic_uT": reading["magnetic_uT"],
            "heading_deg": reading["heading_deg"],
            "accuracy": reading["magnetometer_accuracy"],
        }

    # ODESC API. Motion remains guarded commissioning control.
    def odesc_status(self):
        link = self._require("odesc")
        snapshot = dict(self.odesc_snapshot or {})
        snapshot.update(link.telemetry())
        self.odesc_snapshot = snapshot
        return dict(snapshot)

    def odesc_read(self, property_name):
        return self._require("odesc").read_property(property_name)

    def arm_odesc(self, phrase):
        if str(phrase) != ODESC_ARM_PHRASE:
            raise ValueError("Pass the exact phrase {!r}".format(ODESC_ARM_PHRASE))
        self.odesc_status()
        self.odesc_armed = True
        print("ODESC test motion armed; keep a physical power disconnect ready")
        return self.odesc_status()

    def _require_odesc_armed(self):
        link = self._require("odesc")
        if not self.odesc_armed:
            raise RuntimeError("ODESC motion is disarmed; call arm_odesc('ARM ODESC TEST')")
        return link

    def enable_odesc_axis(self, axis):
        link = self._require_odesc_armed()
        axis = link._axis(axis)
        link.request_state(axis, 8)
        time.sleep(0.25)
        state = link.read_int("axis{}.current_state".format(axis))
        error = link.read_int("axis{}.error".format(axis))
        if state != 8 or error != 0:
            raise RuntimeError(
                "ODESC axis {} did not enter closed loop: state={} error={}".format(
                    axis, state, error
                )
            )
        print("ODESC axis {} closed-loop control enabled".format(axis))
        return state

    def configure_odesc_axis(
        self,
        axis,
        current_limit_a=10.0,
        velocity_limit_turns_s=15.0,
    ):
        link = self._require("odesc")
        axis = link._axis(axis)
        self.odesc_armed = False
        return link.configure_velocity_axis(
            axis,
            current_limit_a,
            velocity_limit_turns_s,
        )

    def configure_odesc_sensorless_axis(
        self,
        axis,
        current_limit_a=10.0,
        startup_current_a=4.0,
        startup_velocity_turns_s=5.0,
        startup_accel_turns_s2=1.8,
        velocity_limit_turns_s=15.0,
    ):
        link = self._require("odesc")
        axis = link._axis(axis)
        self.odesc_armed = False
        return link.configure_sensorless_axis(
            axis,
            current_limit_a,
            startup_current_a,
            startup_velocity_turns_s,
            startup_accel_turns_s2,
            velocity_limit_turns_s,
        )

    def clear_odesc_errors(self, axis):
        link = self._require("odesc")
        self.odesc_armed = False
        return link.clear_errors(axis)

    def calibrate_odesc_axis(self, axis, phrase):
        if str(phrase) != ODESC_CALIBRATE_PHRASE:
            raise ValueError(
                "Pass the exact phrase {!r}".format(ODESC_CALIBRATE_PHRASE)
            )
        link = self._require("odesc")
        axis = link._axis(axis)
        self.odesc_armed = False
        return link.calibrate_axis(axis)

    def calibrate_odesc_motor(self, axis, phrase):
        if str(phrase) != ODESC_MOTOR_CALIBRATE_PHRASE:
            raise ValueError(
                "Pass the exact phrase {!r}".format(
                    ODESC_MOTOR_CALIBRATE_PHRASE
                )
            )
        link = self._require("odesc")
        axis = link._axis(axis)
        self.odesc_armed = False
        return link.calibrate_motor(axis)

    def start_odesc_sensorless(self, axis, direction=1):
        link = self._require_odesc_armed()
        axis = link._axis(axis)
        return link.start_sensorless(axis, direction)

    def set_odesc_sensorless_velocity(self, axis, turns_per_second):
        link = self._require_odesc_armed()
        axis = link._axis(axis)
        velocity = float(turns_per_second)
        if abs(velocity) > ODESC_MAX_CONTROL_TURNS_S:
            raise ValueError(
                "speed magnitude must be at most {} turns/s".format(
                    ODESC_MAX_CONTROL_TURNS_S
                )
            )
        return link.set_sensorless_velocity(axis, velocity)

    def stop_odesc_sensorless(self, axis):
        link = self._require("odesc")
        axis = link._axis(axis)
        result = link.stop_sensorless(axis)
        self.odesc_armed = False
        return result

    def set_odesc_velocity(self, axis, turns_per_second):
        link = self._require_odesc_armed()
        axis = link._axis(axis)
        velocity = float(turns_per_second)
        if abs(velocity) > ODESC_MAX_CONTROL_TURNS_S:
            raise ValueError(
                "speed magnitude must be at most {} turns/s".format(
                    ODESC_MAX_CONTROL_TURNS_S
                )
            )
        state = link.read_int("axis{}.current_state".format(axis), log=False)
        if state != odesc_driver.AXIS_STATE_CLOSED_LOOP_CONTROL:
            raise RuntimeError("Call enable_odesc_axis({}) first".format(axis))
        link.set_velocity(axis, velocity)
        return {"axis": axis, "velocity_turns_s": velocity}

    def pulse_odesc(self, axis, turns_per_second=0.25, seconds=0.5):
        link = self._require_odesc_armed()
        axis = link._axis(axis)
        velocity = float(turns_per_second)
        seconds = float(seconds)
        if abs(velocity) > ODESC_MAX_TEST_TURNS_S:
            raise ValueError(
                "test speed magnitude must be at most {} turns/s".format(
                    ODESC_MAX_TEST_TURNS_S
                )
            )
        if seconds <= 0 or seconds > ODESC_MAX_TEST_SECONDS:
            raise ValueError(
                "test duration must be greater than 0 and at most {} seconds".format(
                    ODESC_MAX_TEST_SECONDS
                )
            )
        state = link.read_int("axis{}.current_state".format(axis))
        if state != 8:
            raise RuntimeError("Call enable_odesc_axis({}) first".format(axis))
        try:
            link.set_velocity(axis, velocity)
            time.sleep(seconds)
        finally:
            link.stop_axis(axis)
        return link.feedback(axis)

    def disable_odesc_axis(self, axis):
        link = self._require("odesc")
        axis = link._axis(axis)
        link.stop_axis(axis)
        link.request_state(axis, 1)
        self.odesc_armed = False
        return axis

    def _stop_odesc_unchecked(self):
        if self.odesc is not None:
            for axis in (0, 1):
                self.odesc.stop_axis(axis)
                self.odesc.request_state(axis, odesc_driver.AXIS_STATE_IDLE)

    def stop_odesc(self):
        self._stop_odesc_unchecked()
        self.odesc_armed = False
        print("ODESC axes commanded to zero; software motion disarmed")

    def stop_all(self):
        """Best-effort stop across every connected motor controller."""
        failures = []
        for name, connected, action in (
            ("odesc", self.odesc is not None, self.stop_odesc),
            ("stm32", self.stm32 is not None, self.stop_drive),
            ("gd32", self.gd32 is not None, self.stop_steppers),
        ):
            if not connected:
                continue
            try:
                action()
            except Exception as exc:
                failures.append("{}: {}".format(name, exc))
        if failures:
            raise RuntimeError("; ".join(failures))
        print("All software motor stops requested")


robot = RoverController()
atexit.register(robot.close)


def connect_all():
    return robot.connect_all()


def links():
    return robot.print_links()


def drive_motor(motor, direction, percent):
    return robot.drive_motor(motor, direction, percent)


def pulse_motor(motor, direction="A", percent=6, seconds=0.5):
    return robot.pulse_motor(motor, direction, percent, seconds)


def currents():
    return robot.currents()


def read_encoders():
    return robot.read_encoders()


def encoder_start(rate_hz=50):
    return robot.start_encoders(rate_hz)


def encoder_stop():
    return robot.stop_encoders()


def encoders():
    return robot.encoders()


def step(axis, steps, rpm=120, wait=False, timeout=None):
    return robot.step(axis, steps, rpm, wait, timeout)


def magnetometer():
    return robot.magnetometer()


def print_imu():
    return robot.print_imu()


def odesc_status():
    return robot.odesc_status()


def stop_all():
    return robot.stop_all()


def close():
    return robot.close()


def show_help():
    print("Combined rover UART shell commands:")
    print("  links() | connect_all() | stop_all() | close()")
    print("  pulse_motor(1, 'A', 6, 0.5) | drive_motor(2, 'B', 10)")
    print("  robot.stop_motor(1) | robot.stop_drive() | currents()")
    print("  read_encoders() | encoder_start(50) | encoders() | encoder_stop()")
    print("  step('X', 3200, 120, wait=True, timeout=10) | robot.stop_steppers()")
    print("  print_imu() | magnetometer()")
    print("  odesc_status() | robot.odesc_read('axis1.error')")
    print("  ODESC motion: see README; explicit arm and bounded pulse are required")


if __name__ == "__main__":
    connect_all()
    show_help()
