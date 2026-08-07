"""Raspberry Pi 5 UART client for an ODESC/ODrive v3-compatible controller.

Validated wiring:
    Pi GPIO4 / physical 7  / UART2 TX -> ODESC GPIO2 / UART RX
    Pi GPIO5 / physical 29 / UART2 RX <- ODESC GPIO1 / UART TX
    Pi GND                               -> ODESC GND

This is regular Python 3 for Raspberry Pi OS and Thonny, not MicroPython.
Dependency: pyserial (Raspberry Pi OS package ``python3-serial``).
"""

import threading
import time
import math

try:
    import serial
except ImportError as exc:
    raise ImportError(
        "pyserial is required; install Raspberry Pi OS package python3-serial"
    ) from exc


UART_DEVICE = "/dev/ttyAMA2"
UART_BAUD = 115200
UART_TIMEOUT_SECONDS = 1.0
AXIS_STATE_IDLE = 1
AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
AXIS_STATE_MOTOR_CALIBRATION = 4
AXIS_STATE_SENSORLESS_CONTROL = 5
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
CONTROL_MODE_VELOCITY_CONTROL = 2
INPUT_MODE_PASSTHROUGH = 1


class ODESCUART:
    """Synchronized client for the ODrive v0.5.x ASCII protocol."""

    def __init__(
        self,
        device=UART_DEVICE,
        baudrate=UART_BAUD,
        timeout=UART_TIMEOUT_SECONDS,
    ):
        self.device = str(device)
        self.baudrate = int(baudrate)
        self.timeout = float(timeout)
        self.port = None
        self._lock = threading.Lock()

    def open(self):
        if self.port is not None and self.port.is_open:
            return self

        self.port = serial.Serial(
            port=self.device,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
            write_timeout=self.timeout,
            exclusive=True,
        )
        self.port.reset_input_buffer()
        self.port.reset_output_buffer()
        print(
            "ODESC UART opened: {} at {} baud (Pi TX GPIO4, RX GPIO5)".format(
                self.device, self.baudrate
            )
        )
        return self

    def close(self):
        if self.port is not None:
            try:
                self.port.close()
            finally:
                self.port = None

    def query(self, command, log=True):
        """Send one ASCII command and return its non-empty response line."""
        command = str(command).strip()
        if not command or "\n" in command or "\r" in command:
            raise ValueError("command must contain exactly one non-empty line")
        if self.port is None or not self.port.is_open:
            raise RuntimeError("ODESC UART is not open")

        packet = (command + "\n").encode("ascii")
        with self._lock:
            self.port.reset_input_buffer()
            self.port.write(packet)
            self.port.flush()
            response = self.port.readline()

        if log:
            print("TX:", repr(packet))
            print("RX:", repr(response))
        if not response:
            raise TimeoutError("No ODESC reply to {!r}".format(command))

        text = response.decode("ascii", errors="replace").strip()
        if not text:
            raise RuntimeError("ODESC returned an empty response")
        return text

    def send(self, command):
        """Send one ASCII command that does not produce a response."""
        command = str(command).strip()
        if not command or "\n" in command or "\r" in command:
            raise ValueError("command must contain exactly one non-empty line")
        if self.port is None or not self.port.is_open:
            raise RuntimeError("ODESC UART is not open")

        packet = (command + "\n").encode("ascii")
        with self._lock:
            self.port.write(packet)
            self.port.flush()
        print("TX:", repr(packet))

    def write_property(self, name, value):
        self.send("w {} {}".format(str(name).strip(), value))

    def request_state(self, axis, state):
        axis = self._axis(axis)
        self.write_property("axis{}.requested_state".format(axis), int(state))

    def set_velocity(self, axis, turns_per_second, torque_ff=0.0):
        axis = self._axis(axis)
        self.send(
            "v {} {:.6f} {:.6f}".format(
                axis, float(turns_per_second), float(torque_ff)
            )
        )

    def stop_axis(self, axis):
        self.set_velocity(axis, 0.0, 0.0)

    def feedback(self, axis):
        axis = self._axis(axis)
        fields = self.query("f {}".format(axis)).split()
        if len(fields) != 2:
            raise RuntimeError("Malformed ODESC feedback: {!r}".format(fields))
        return {"position_turns": float(fields[0]), "velocity_turns_s": float(fields[1])}

    @staticmethod
    def _axis(axis):
        axis = int(axis)
        if axis not in (0, 1):
            raise ValueError("ODESC axis must be 0 or 1")
        return axis

    def read_property(self, name, log=True):
        return self.query("r {}".format(str(name).strip()), log=log)

    def read_float(self, name, log=True):
        return float(self.read_property(name, log=log))

    def read_int(self, name, log=True):
        value = self.read_property(name, log=log)
        # ODrive v0.5.x can append "d" to decimal integer responses.
        if value.endswith("d"):
            value = value[:-1]
        return int(value, 0)

    def axis_snapshot(self, axis, log=False):
        """Read motion, electrical, configuration, and fault state for one axis."""
        axis = self._axis(axis)
        prefix = "axis{}.".format(axis)
        pole_pairs = self.read_int(
            prefix + "motor.config.pole_pairs", log=log
        )
        ramp_velocity_electrical = self.read_float(
            prefix + "config.sensorless_ramp.vel", log=log
        )
        ramp_accel_electrical = self.read_float(
            prefix + "config.sensorless_ramp.accel", log=log
        )
        sensorless_velocity_electrical = self.read_float(
            prefix + "sensorless_estimator.vel_estimate", log=log
        )
        electrical_per_mechanical_turn = 2.0 * math.pi * pole_pairs
        return {
            "state": self.read_int(prefix + "current_state", log=log),
            "axis_error": self.read_int(prefix + "error", log=log),
            "motor_error": self.read_int(prefix + "motor.error", log=log),
            "encoder_error": self.read_int(prefix + "encoder.error", log=log),
            "controller_error": self.read_int(
                prefix + "controller.error", log=log
            ),
            "position_turns": self.read_float(
                prefix + "encoder.pos_estimate", log=log
            ),
            "velocity_turns_s": self.read_float(
                prefix + "encoder.vel_estimate", log=log
            ),
            "iq_measured_a": self.read_float(
                prefix + "motor.current_control.Iq_measured", log=log
            ),
            "iq_setpoint_a": self.read_float(
                prefix + "motor.current_control.Iq_setpoint", log=log
            ),
            "current_limit_a": self.read_float(
                prefix + "motor.config.current_lim", log=log
            ),
            "velocity_limit_turns_s": self.read_float(
                prefix + "controller.config.vel_limit", log=log
            ),
            "control_mode": self.read_int(
                prefix + "controller.config.control_mode", log=log
            ),
            "input_mode": self.read_int(
                prefix + "controller.config.input_mode", log=log
            ),
            "motor_pre_calibrated": bool(
                self.read_int(prefix + "motor.config.pre_calibrated", log=log)
            ),
            "pole_pairs": pole_pairs,
            "encoder_pre_calibrated": bool(
                self.read_int(prefix + "encoder.config.pre_calibrated", log=log)
            ),
            "sensorless_estimator_error": self.read_int(
                prefix + "sensorless_estimator.error", log=log
            ),
            "sensorless_velocity_turns_s": (
                sensorless_velocity_electrical
                / electrical_per_mechanical_turn
            ),
            "sensorless_ramp_current_a": self.read_float(
                prefix + "config.sensorless_ramp.current", log=log
            ),
            "sensorless_ramp_velocity_turns_s": (
                ramp_velocity_electrical
                / electrical_per_mechanical_turn
            ),
            "sensorless_ramp_accel_turns_s2": (
                ramp_accel_electrical
                / electrical_per_mechanical_turn
            ),
            "sensorless_pm_flux_linkage": self.read_float(
                prefix + "sensorless_estimator.config.pm_flux_linkage",
                log=log,
            ),
            "sensorless_observer_gain": self.read_float(
                prefix + "sensorless_estimator.config.observer_gain",
                log=log,
            ),
            "sensorless_pll_bandwidth": self.read_float(
                prefix + "sensorless_estimator.config.pll_bandwidth",
                log=log,
            ),
        }

    def telemetry(self):
        """Return live DC-bus power and both motor-axis telemetry snapshots."""
        voltage = self.read_float("vbus_voltage", log=False)
        bus_current = self.read_float("ibus", log=False)
        return {
            "vbus_voltage": voltage,
            "ibus_a": bus_current,
            "bus_power_w": voltage * bus_current,
            "axes": [self.axis_snapshot(0), self.axis_snapshot(1)],
        }

    def clear_errors(self, axis):
        """Clear writable axis/component error registers on ODrive v0.5.x."""
        axis = self._axis(axis)
        prefix = "axis{}.".format(axis)
        self.stop_axis(axis)
        for name in (
            "error",
            "motor.error",
            "encoder.error",
            "controller.error",
        ):
            self.write_property(prefix + name, 0)
        return self.axis_snapshot(axis)

    def configure_velocity_axis(
        self,
        axis,
        current_limit_a,
        velocity_limit_turns_s,
    ):
        """Apply volatile velocity-control settings without saving or rebooting."""
        axis = self._axis(axis)
        current_limit_a = float(current_limit_a)
        velocity_limit_turns_s = float(velocity_limit_turns_s)
        if not 0.5 <= current_limit_a <= 30.0:
            raise ValueError("current limit must be from 0.5 through 30 A")
        if not 0.1 <= velocity_limit_turns_s <= 50.0:
            raise ValueError("velocity limit must be from 0.1 through 50 turns/s")
        prefix = "axis{}.".format(axis)
        self.stop_axis(axis)
        self.write_property(
            prefix + "controller.config.control_mode",
            CONTROL_MODE_VELOCITY_CONTROL,
        )
        self.write_property(
            prefix + "controller.config.input_mode",
            INPUT_MODE_PASSTHROUGH,
        )
        self.write_property(prefix + "motor.config.current_lim", current_limit_a)
        self.write_property(
            prefix + "controller.config.vel_limit",
            velocity_limit_turns_s,
        )
        return self.axis_snapshot(axis)

    def configure_sensorless_axis(
        self,
        axis,
        current_limit_a,
        startup_current_a,
        startup_velocity_turns_s,
        startup_accel_turns_s2,
        velocity_limit_turns_s,
    ):
        """Apply volatile legacy ODrive v3 sensorless settings.

        The old firmware expresses its startup ramp in electrical radians per
        second. This API accepts mechanical turns per second and converts using
        the configured motor pole-pair count.
        """
        axis = self._axis(axis)
        current_limit_a = float(current_limit_a)
        startup_current_a = float(startup_current_a)
        startup_velocity_turns_s = abs(float(startup_velocity_turns_s))
        startup_accel_turns_s2 = abs(float(startup_accel_turns_s2))
        velocity_limit_turns_s = abs(float(velocity_limit_turns_s))
        if not 0.5 <= current_limit_a <= 30.0:
            raise ValueError("current limit must be from 0.5 through 30 A")
        if not 0.5 <= startup_current_a <= current_limit_a:
            raise ValueError(
                "startup current must be from 0.5 A through the current limit"
            )
        if not 1.0 <= startup_velocity_turns_s <= 30.0:
            raise ValueError(
                "sensorless startup speed must be from 1 through 30 turns/s"
            )
        if not 0.1 <= startup_accel_turns_s2 <= 30.0:
            raise ValueError(
                "sensorless startup acceleration must be from 0.1 through "
                "30 turns/s^2"
            )
        if not startup_velocity_turns_s <= velocity_limit_turns_s <= 50.0:
            raise ValueError(
                "velocity limit must be at least the startup speed and at "
                "most 50 turns/s"
            )

        prefix = "axis{}.".format(axis)
        self.request_state(axis, AXIS_STATE_IDLE)
        if not self.read_int(prefix + "motor.config.pre_calibrated", log=False):
            raise RuntimeError(
                "axis {} motor is not calibrated; run motor calibration first".format(
                    axis
                )
            )
        pole_pairs = self.read_int(
            prefix + "motor.config.pole_pairs", log=False
        )
        if not 1 <= pole_pairs <= 100:
            raise RuntimeError("invalid motor pole-pair count {}".format(pole_pairs))
        electrical_scale = 2.0 * math.pi * pole_pairs

        self.write_property(
            prefix + "controller.config.control_mode",
            CONTROL_MODE_VELOCITY_CONTROL,
        )
        self.write_property(
            prefix + "controller.config.input_mode",
            INPUT_MODE_PASSTHROUGH,
        )
        self.write_property(prefix + "motor.config.current_lim", current_limit_a)
        self.write_property(
            prefix + "controller.config.vel_limit", velocity_limit_turns_s
        )
        self.write_property(
            prefix + "config.sensorless_ramp.current", startup_current_a
        )
        self.write_property(
            prefix + "config.sensorless_ramp.vel",
            startup_velocity_turns_s * electrical_scale,
        )
        self.write_property(
            prefix + "config.sensorless_ramp.accel",
            startup_accel_turns_s2 * electrical_scale,
        )
        self.write_property(prefix + "config.sensorless_ramp.finish_on_vel", 1)
        self.write_property(
            prefix + "config.sensorless_ramp.finish_on_distance", 0
        )
        return self.axis_snapshot(axis)

    def start_sensorless(self, axis, direction):
        """Start the legacy state-5 open-loop ramp and sensorless estimator."""
        axis = self._axis(axis)
        direction = 1 if int(direction) >= 0 else -1
        prefix = "axis{}.".format(axis)
        self.request_state(axis, AXIS_STATE_IDLE)
        if not self.read_int(prefix + "motor.config.pre_calibrated", log=False):
            raise RuntimeError(
                "axis {} motor is not calibrated; run motor calibration first".format(
                    axis
                )
            )
        ramp_velocity = abs(
            self.read_float(prefix + "config.sensorless_ramp.vel", log=False)
        )
        ramp_accel = abs(
            self.read_float(prefix + "config.sensorless_ramp.accel", log=False)
        )
        pole_pairs = self.read_int(
            prefix + "motor.config.pole_pairs", log=False
        )
        if pole_pairs <= 0 or ramp_accel <= 0.0:
            raise RuntimeError("invalid sensorless ramp configuration")
        self.write_property(
            prefix + "config.sensorless_ramp.vel",
            direction * ramp_velocity,
        )
        self.request_state(axis, AXIS_STATE_SENSORLESS_CONTROL)
        time.sleep(0.20)
        state = self.read_int(prefix + "current_state", log=False)
        axis_error = self.read_int(prefix + "error", log=False)
        motor_error = self.read_int(prefix + "motor.error", log=False)
        estimator_error = self.read_int(
            prefix + "sensorless_estimator.error", log=False
        )
        if (
            state != AXIS_STATE_SENSORLESS_CONTROL
            or axis_error
            or motor_error
            or estimator_error
        ):
            self.request_state(axis, AXIS_STATE_IDLE)
            raise RuntimeError(
                "sensorless startup failed: state={} axis_error={} "
                "motor_error={} estimator_error={}".format(
                    state, axis_error, motor_error, estimator_error
                )
            )
        electrical_scale = 2.0 * math.pi * pole_pairs
        return {
            "axis": axis,
            "state": state,
            "axis_error": axis_error,
            "motor_error": motor_error,
            "sensorless_estimator_error": estimator_error,
            "sensorless_ramp_velocity_turns_s": (
                direction * ramp_velocity / electrical_scale
            ),
            "sensorless_ramp_accel_turns_s2": (
                ramp_accel / electrical_scale
            ),
        }

    def set_sensorless_velocity(self, axis, turns_per_second):
        """Set velocity while enforcing legacy sensorless speed/direction limits."""
        axis = self._axis(axis)
        velocity = float(turns_per_second)
        prefix = "axis{}.".format(axis)
        state = self.read_int(prefix + "current_state", log=False)
        if state != AXIS_STATE_SENSORLESS_CONTROL:
            raise RuntimeError(
                "axis {} is not in sensorless state 5".format(axis)
            )
        pole_pairs = self.read_int(
            prefix + "motor.config.pole_pairs", log=False
        )
        ramp_velocity_electrical = self.read_float(
            prefix + "config.sensorless_ramp.vel", log=False
        )
        startup_velocity = (
            ramp_velocity_electrical / (2.0 * math.pi * pole_pairs)
        )
        if velocity == 0.0:
            raise ValueError("sensorless cannot command zero; stop to IDLE instead")
        if velocity * startup_velocity <= 0.0:
            raise ValueError(
                "sensorless direction cannot reverse while running; stop and restart"
            )
        if abs(velocity) < abs(startup_velocity):
            raise ValueError(
                "sensorless speed must remain at or above {:.3f} turns/s".format(
                    abs(startup_velocity)
                )
            )
        velocity_limit = self.read_float(
            prefix + "controller.config.vel_limit", log=False
        )
        if abs(velocity) > velocity_limit:
            raise ValueError(
                "speed exceeds configured limit {:.3f} turns/s".format(
                    velocity_limit
                )
            )
        self.set_velocity(axis, velocity)
        return {
            "axis": axis,
            "velocity_turns_s": velocity,
            "minimum_turns_s": abs(startup_velocity),
        }

    def stop_sensorless(self, axis):
        """Stop sensorless operation immediately by returning the axis to IDLE."""
        axis = self._axis(axis)
        self.request_state(axis, AXIS_STATE_IDLE)
        time.sleep(0.10)
        return self.axis_snapshot(axis)

    def calibrate_motor(self, axis, timeout_s=25.0):
        """Run motor-only calibration for an encoderless sensorless axis."""
        axis = self._axis(axis)
        timeout_s = float(timeout_s)
        prefix = "axis{}.".format(axis)
        self.request_state(axis, AXIS_STATE_IDLE)
        self.clear_errors(axis)
        self.request_state(axis, AXIS_STATE_MOTOR_CALIBRATION)
        deadline = time.monotonic() + timeout_s
        entered_calibration = False
        while time.monotonic() < deadline:
            state = self.read_int(prefix + "current_state", log=False)
            if state == AXIS_STATE_MOTOR_CALIBRATION:
                entered_calibration = True
            if entered_calibration and state == AXIS_STATE_IDLE:
                snapshot = self.axis_snapshot(axis)
                if snapshot["axis_error"] or snapshot["motor_error"]:
                    raise RuntimeError(
                        "ODESC axis {} motor calibration failed: {}".format(
                            axis, snapshot
                        )
                    )
                return snapshot
            time.sleep(0.25)
        self.request_state(axis, AXIS_STATE_IDLE)
        raise TimeoutError(
            "ODESC axis {} motor calibration did not finish within {} seconds".format(
                axis, timeout_s
            )
        )

    def calibrate_axis(self, axis, timeout_s=35.0):
        """Run the ODrive full calibration sequence and return final diagnostics."""
        axis = self._axis(axis)
        timeout_s = float(timeout_s)
        self.stop_axis(axis)
        self.clear_errors(axis)
        self.request_state(axis, AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
        deadline = time.monotonic() + timeout_s
        entered_calibration = False
        while time.monotonic() < deadline:
            state = self.read_int(
                "axis{}.current_state".format(axis), log=False
            )
            if state == AXIS_STATE_FULL_CALIBRATION_SEQUENCE:
                entered_calibration = True
            if entered_calibration and state == AXIS_STATE_IDLE:
                snapshot = self.axis_snapshot(axis)
                if any(
                    snapshot[name]
                    for name in (
                        "axis_error",
                        "motor_error",
                        "encoder_error",
                        "controller_error",
                    )
                ):
                    raise RuntimeError(
                        "ODESC axis {} calibration failed: {}".format(
                            axis, snapshot
                        )
                    )
                return snapshot
            time.sleep(0.25)
        self.request_state(axis, AXIS_STATE_IDLE)
        raise TimeoutError(
            "ODESC axis {} calibration did not finish within {} seconds".format(
                axis, timeout_s
            )
        )

    def check_connection(self):
        """Verify both UART directions and return a configuration snapshot."""
        snapshot = {
            "vbus_voltage": self.read_float("vbus_voltage"),
            "uart_enabled": self.read_int("config.enable_uart"),
            "uart_baudrate": self.read_int("config.uart_baudrate"),
            "axis1_state": self.read_int("axis1.current_state"),
            "axis1_error": self.read_int("axis1.error"),
        }
        if snapshot["uart_enabled"] != 1:
            raise RuntimeError("ODESC reports that UART is disabled")
        if snapshot["uart_baudrate"] != self.baudrate:
            raise RuntimeError(
                "ODESC baud is {}, but client baud is {}".format(
                    snapshot["uart_baudrate"], self.baudrate
                )
            )

        print("ODESC UART ROUND TRIP VERIFIED")
        print("Bus voltage: {:.6f} V".format(snapshot["vbus_voltage"]))
        print(
            "UART enabled={} baud={} | axis1 state={} error={}".format(
                snapshot["uart_enabled"],
                snapshot["uart_baudrate"],
                snapshot["axis1_state"],
                snapshot["axis1_error"],
            )
        )
        return snapshot


def run_self_test(device=UART_DEVICE, baudrate=UART_BAUD):
    controller = ODESCUART(device, baudrate)
    try:
        controller.open()
        return controller.check_connection()
    finally:
        controller.close()
        print("ODESC UART closed")


if __name__ == "__main__":
    try:
        run_self_test()
    except Exception as exc:
        print("ODESC UART TEST FAILED: {}".format(exc))
        print("Check crossed TX/RX, common ground, /dev/ttyAMA2, and 115200 baud.")
