"""Raspberry Pi 5 UART client for the Ender-3 GD32 stepper controller.

Run this file with Thonny's local Python 3 interpreter. The background reader
keeps heartbeat acknowledgements, switch reports, and move completions flowing
while the Thonny shell remains available for commands.
"""

import atexit
import threading
import time

try:
    import serial
except ImportError as error:
    raise RuntimeError(
        "pyserial is required; install python3-serial or the pyserial package"
    ) from error


SERIAL_PORT = "/dev/ttyAMA3"
BAUDRATE = 115200
LINK_TIMEOUT_SECONDS = 5.0
MOTOR_MICROSTEPS_PER_REVOLUTION = 3200.0
MAX_DRIVER_RPM = 1000.0
MAX_PENDING_MOVES = 8
AXIS_STEPS_PER_UNIT = {"X": 80.0, "Y": 80.0, "Z": 400.0, "E": 93.0}


class StepperController:
    def __init__(self, port=SERIAL_PORT, baudrate=BAUDRATE):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self._running = False
        self._reader = None
        self._write_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._move_sequence = 0
        self._pending_moves = {}
        self._last_heartbeat = None
        self._last_round_trip = None
        self._link_was_up = False
        self._switch_callback = None
        self.switches = {"X": None, "Y": None, "Z": None}
        self.driver_rpm = {"X": 120.0, "Y": 120.0, "Z": 120.0, "E": 120.0}
        self.position_steps = {"X": 0, "Y": 0, "Z": 0, "E": 0}
        self.target_steps = {"X": 0, "Y": 0, "Z": 0, "E": 0}
        self.position_known = {"X": False, "Y": False, "Z": False, "E": False}
        self.step_limits = {
            "X": (None, None),
            "Y": (None, None),
            "Z": (None, None),
            "E": (None, None),
        }

    def start(self):
        if self._running:
            return
        try:
            self.serial = serial.Serial(
                self.port,
                self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.2,
                write_timeout=1.0,
            )
        except serial.SerialException as error:
            raise RuntimeError(
                "Cannot open {}. Check the UART mapping and add your user to "
                "the dialout group if permission is denied.".format(self.port)
            ) from error

        self.serial.reset_input_buffer()
        self._running = True
        self._reader = threading.Thread(
            target=self._reader_loop,
            name="ender3-uart",
            daemon=True,
        )
        self._reader.start()
        print("Ender UART open on {} at {} baud".format(self.port, self.baudrate))

    def close(self):
        self._running = False
        if self.serial is not None and self.serial.is_open:
            self.serial.close()

    @staticmethod
    def _axis(axis):
        axis = str(axis).upper()
        if axis not in AXIS_STEPS_PER_UNIT:
            raise ValueError("axis must be X, Y, Z, or E")
        return axis

    @staticmethod
    def _validate_rpm(rpm):
        rpm = float(rpm)
        if rpm <= 0 or rpm > MAX_DRIVER_RPM:
            raise ValueError(
                "rpm must be greater than 0 and at most {}".format(MAX_DRIVER_RPM)
            )
        return rpm

    def _send_lines(self, commands):
        if not self._running or self.serial is None or not self.serial.is_open:
            raise RuntimeError("UART is not open")
        payload = "".join(command.strip() + "\n" for command in commands).encode("ascii")
        with self._write_lock:
            self.serial.write(payload)
            self.serial.flush()

    def send_gcode(self, command):
        """Send one raw Marlin command. Normal motion should use move()."""
        self._send_lines((command,))

    def enable_drivers(self):
        """Enable all drivers; the board has one shared enable signal."""
        self.send_gcode("M17")

    def disable_drivers(self):
        """Disable all drivers; the board has one shared enable signal."""
        self.send_gcode("M18")

    def set_speed(self, axis, rpm):
        axis = self._axis(axis)
        rpm = self._validate_rpm(rpm)
        with self._state_lock:
            self.driver_rpm[axis] = rpm
        return rpm

    def reset_position(self, axis):
        """Define the current open-loop location as step zero."""
        axis = self._axis(axis)
        with self._state_lock:
            if any(record["axis"] == axis for record in self._pending_moves.values()):
                raise RuntimeError(
                    "cannot reset {} while that axis has a pending move".format(axis)
                )
            self.position_steps[axis] = 0
            self.target_steps[axis] = 0
            self.position_known[axis] = True
        return self.axis_position(axis)

    def set_limits(self, axis, minimum_steps, maximum_steps):
        """Set inclusive host-side limits relative to the defined zero."""
        axis = self._axis(axis)
        minimum_steps = int(minimum_steps)
        maximum_steps = int(maximum_steps)
        if minimum_steps >= maximum_steps:
            raise ValueError("minimum step limit must be less than maximum")
        with self._state_lock:
            if self.position_known[axis] and not (
                minimum_steps <= self.target_steps[axis] <= maximum_steps
            ):
                raise ValueError(
                    "{} current target is outside the requested limits".format(axis)
                )
            self.step_limits[axis] = (minimum_steps, maximum_steps)
        return self.axis_position(axis)

    def axis_position(self, axis):
        axis = self._axis(axis)
        with self._state_lock:
            minimum, maximum = self.step_limits[axis]
            return {
                "axis": axis,
                "position_steps": self.position_steps[axis],
                "target_steps": self.target_steps[axis],
                "known": self.position_known[axis],
                "minimum_steps": minimum,
                "maximum_steps": maximum,
            }

    def move(self, axis, steps, rpm=None, wait=False, timeout=None):
        """Queue signed driver microsteps; sign selects the direction."""
        axis = self._axis(axis)
        steps = int(steps)
        if steps == 0:
            raise ValueError("steps must not be zero")

        with self._state_lock:
            rpm = self.driver_rpm[axis] if rpm is None else rpm
            rpm = self._validate_rpm(rpm)
            if len(self._pending_moves) >= MAX_PENDING_MOVES:
                raise RuntimeError("too many queued moves; wait for DRV_DONE")
            target_steps = self.target_steps[axis] + steps
            minimum_steps, maximum_steps = self.step_limits[axis]
            if minimum_steps is not None or maximum_steps is not None:
                if not self.position_known[axis]:
                    raise RuntimeError(
                        "{} position is unknown; reset its zero before using limits".format(
                            axis
                        )
                    )
                if (
                    minimum_steps is not None
                    and target_steps < minimum_steps
                ) or (
                    maximum_steps is not None
                    and target_steps > maximum_steps
                ):
                    raise ValueError(
                        "{} target {} is outside limits {}..{}".format(
                            axis, target_steps, minimum_steps, maximum_steps
                        )
                    )
            self._move_sequence += 1
            move_id = self._move_sequence
            record = {
                "axis": axis,
                "steps": steps,
                "event": threading.Event(),
                "completed": False,
            }
            self._pending_moves[move_id] = record
            self.target_steps[axis] = target_steps

        steps_per_unit = AXIS_STEPS_PER_UNIT[axis]
        distance_units = steps / steps_per_unit
        feedrate_units_min = rpm * MOTOR_MICROSTEPS_PER_REVOLUTION / steps_per_unit
        commands = ["G91"]
        if axis == "E":
            commands.append("M83")
        commands.extend(
            (
                "G0 {}{:.5f} F{:.2f}".format(axis, distance_units, feedrate_units_min),
                "M400",
                "M118 DRV_DONE {} {} {}".format(move_id, axis, steps),
            )
        )

        try:
            self._send_lines(commands)
        except Exception:
            with self._state_lock:
                self._pending_moves.pop(move_id, None)
                self.target_steps[axis] -= steps
            raise

        print(
            "Queued {} move {}: {} microsteps at {} RPM".format(
                axis, move_id, steps, rpm
            )
        )
        if wait:
            if not record["event"].wait(timeout):
                raise TimeoutError("move {} did not complete before timeout".format(move_id))
            if not record["completed"]:
                raise RuntimeError("move {} was stopped before completion".format(move_id))
        return move_id

    def move_sps(self, axis, steps, steps_per_second, wait=False, timeout=None):
        steps_per_second = float(steps_per_second)
        if steps_per_second <= 0:
            raise ValueError("steps_per_second must be greater than zero")
        rpm = steps_per_second * 60.0 / MOTOR_MICROSTEPS_PER_REVOLUTION
        return self.move(axis, steps, rpm, wait=wait, timeout=timeout)

    def stop(self):
        """Quick-stop planner motion and mark pending host transactions stopped."""
        with self._state_lock:
            records = tuple(self._pending_moves.values())
            self._pending_moves.clear()
            affected_axes = {record["axis"] for record in records}
            for axis in affected_axes:
                self.target_steps[axis] = self.position_steps[axis]
                self.position_known[axis] = False
        for record in records:
            record["event"].set()
        self.send_gcode("M410")
        print("Quick stop requested")

    def forward(self, axis, steps, rpm=None, wait=False, timeout=None):
        return self.move(axis, abs(int(steps)), rpm, wait=wait, timeout=timeout)

    def backward(self, axis, steps, rpm=None, wait=False, timeout=None):
        return self.move(axis, -abs(int(steps)), rpm, wait=wait, timeout=timeout)

    def on_switch_change(self, callback):
        """Register callback(states) or pass None to remove it."""
        if callback is not None and not callable(callback):
            raise TypeError("callback must be callable or None")
        with self._state_lock:
            self._switch_callback = callback

    def switch_status(self):
        with self._state_lock:
            states = dict(self.switches)
        print("Switches X={} Y={} Z={}".format(states["X"], states["Y"], states["Z"]))
        return states

    def status(self):
        snapshot = self.link_snapshot()
        print(
            "UART {} link_rx={} round_trip={} pending={}".format(
                self.port,
                "up" if snapshot["receiving"] else "down",
                "up" if snapshot["round_trip"] else "down",
                snapshot["pending_moves"],
            )
        )
        speeds = snapshot["driver_rpm"]
        print(
            "Default RPM X={} Y={} Z={} E={}".format(
                speeds["X"], speeds["Y"], speeds["Z"], speeds["E"]
            )
        )
        return self.switch_status()

    def link_snapshot(self):
        """Return link state without printing or sending a command."""
        now = time.monotonic()
        with self._state_lock:
            last_heartbeat = self._last_heartbeat
            last_round_trip = self._last_round_trip
            pending = len(self._pending_moves)
            speeds = dict(self.driver_rpm)
            switches = dict(self.switches)
            positions = {
                axis: {
                    "position_steps": self.position_steps[axis],
                    "target_steps": self.target_steps[axis],
                    "known": self.position_known[axis],
                    "minimum_steps": self.step_limits[axis][0],
                    "maximum_steps": self.step_limits[axis][1],
                }
                for axis in "XYZE"
            }
        receiving = last_heartbeat is not None and now - last_heartbeat <= LINK_TIMEOUT_SECONDS
        round_trip = (
            last_round_trip is not None and now - last_round_trip <= LINK_TIMEOUT_SECONDS
        )
        return {
            "receiving": receiving,
            "round_trip": round_trip,
            "pending_moves": pending,
            "driver_rpm": speeds,
            "switches": switches,
            "positions": positions,
        }

    def check_connection(self, timeout=6.0):
        """Wait for a heartbeat acknowledgement round trip."""
        deadline = time.monotonic() + float(timeout)
        while time.monotonic() < deadline:
            snapshot = self.link_snapshot()
            if snapshot["round_trip"]:
                print("GD32 UART ROUND TRIP VERIFIED")
                return True
            time.sleep(0.05)
        raise TimeoutError("No GD32 heartbeat round trip on {}".format(self.port))

    def _reader_loop(self):
        while self._running:
            try:
                raw_line = self.serial.readline()
            except serial.SerialException as error:
                if self._running:
                    print("Ender UART read error:", error)
                self._running = False
                break
            if raw_line:
                line = raw_line.decode("ascii", errors="replace").strip()
                if line:
                    self._handle_line(line)
            self._check_link_timeout()

    def _handle_line(self, line):
        if line.startswith("HB "):
            sequence = line[3:].strip()
            with self._state_lock:
                self._last_heartbeat = time.monotonic()
            self.send_gcode("M118 HB_ACK " + sequence)
            return

        if line.startswith("HB_ACK_OK"):
            with self._state_lock:
                self._last_round_trip = time.monotonic()
                announce = not self._link_was_up
                self._link_was_up = True
            if announce:
                print("Ender UART round trip verified")
            return

        if line.startswith("DRV_DONE "):
            self._handle_move_complete(line)
            return

        if line.startswith("SW "):
            self._handle_switch_report(line)
            return

        if line == "ok" or line.startswith("HB_ACK "):
            return
        print("Ender:", line)

    def _handle_move_complete(self, line):
        try:
            _, move_id_text, axis, steps_text = line.split()
            move_id = int(move_id_text)
            steps = int(steps_text)
        except (ValueError, IndexError):
            print("Malformed driver completion:", line)
            return

        with self._state_lock:
            record = self._pending_moves.pop(move_id, None)
        if record is None or record["axis"] != axis or record["steps"] != steps:
            print("Unexpected driver completion:", line)
            return
        with self._state_lock:
            self.position_steps[axis] += steps
        record["completed"] = True
        record["event"].set()
        print("{} move {} complete: {} microsteps".format(axis, move_id, steps))

    def _handle_switch_report(self, line):
        try:
            fields = line.split()
            if len(fields) != 4:
                raise ValueError
            new_states = {field[0]: bool(int(field[1:])) for field in fields[1:]}
            if set(new_states) != {"X", "Y", "Z"}:
                raise ValueError
        except (ValueError, IndexError):
            print("Malformed switch report:", line)
            return

        with self._state_lock:
            changed = any(self.switches[axis] != new_states[axis] for axis in "XYZ")
            self.switches.update(new_states)
            callback = self._switch_callback
            states = dict(self.switches)
        if changed:
            print("Switches X={} Y={} Z={}".format(states["X"], states["Y"], states["Z"]))
            if callback is not None:
                try:
                    callback(states)
                except Exception as error:
                    print("Switch callback error:", error)

    def _check_link_timeout(self):
        with self._state_lock:
            if (
                self._link_was_up
                and self._last_round_trip is not None
                and time.monotonic() - self._last_round_trip > LINK_TIMEOUT_SECONDS
            ):
                self._link_was_up = False
                announce = True
            else:
                announce = False
        if announce:
            print("Ender UART round trip lost")


controller = StepperController()
atexit.register(controller.close)


def move(axis, steps, rpm=None, wait=False, timeout=None):
    return controller.move(axis, steps, rpm, wait=wait, timeout=timeout)


def move_sps(axis, steps, steps_per_second, wait=False, timeout=None):
    return controller.move_sps(
        axis, steps, steps_per_second, wait=wait, timeout=timeout
    )


def xmove(steps, rpm=None, wait=False, timeout=None):
    return move("X", steps, rpm, wait=wait, timeout=timeout)


def ymove(steps, rpm=None, wait=False, timeout=None):
    return move("Y", steps, rpm, wait=wait, timeout=timeout)


def zmove(steps, rpm=None, wait=False, timeout=None):
    return move("Z", steps, rpm, wait=wait, timeout=timeout)


def emove(steps, rpm=None, wait=False, timeout=None):
    return move("E", steps, rpm, wait=wait, timeout=timeout)


def forward(axis, steps, rpm=None, wait=False, timeout=None):
    return controller.forward(axis, steps, rpm, wait=wait, timeout=timeout)


def backward(axis, steps, rpm=None, wait=False, timeout=None):
    return controller.backward(axis, steps, rpm, wait=wait, timeout=timeout)


def set_speed(axis, rpm):
    return controller.set_speed(axis, rpm)


def enable_drivers():
    return controller.enable_drivers()


enable_steppers = enable_drivers


def disable_drivers():
    return controller.disable_drivers()


disable_steppers = disable_drivers


def stop():
    return controller.stop()


def status():
    return controller.status()


def switch_status():
    return controller.switch_status()


def on_switch_change(callback):
    return controller.on_switch_change(callback)


def send_gcode(command):
    return controller.send_gcode(command)


def close():
    return controller.close()


def show_help():
    print("Thonny shell commands:")
    print("  move('X', 3200, 120) | move('E', -800, 60)")
    print("  xmove(...) | ymove(...) | zmove(...) | emove(...)")
    print("  move_sps('Y', -1600, 8000) | set_speed('Z', 30)")
    print("  forward('X', 800, 60) | backward('X', 800, 60)")
    print("  stop() | enable_drivers() | disable_drivers() | status()")
    print("Add wait=True to block until DRV_DONE, with optional timeout seconds.")
    print("Driver current and microstep mode are set by board hardware, not UART.")


if __name__ == "__main__":
    controller.start()
    print("Raspberry Pi stepper client started; returning to the Thonny prompt.")
    show_help()
