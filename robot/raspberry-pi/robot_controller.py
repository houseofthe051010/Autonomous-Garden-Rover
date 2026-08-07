"""Combined Raspberry Pi 5 shell client for all four rover UART devices.

Run with Thonny's regular Python 3 interpreter, not MicroPython. Each UART has
one owning client and protocol parser:

    UART1 /dev/ttyAMA1 / GPIO0,1   STM32 dual BTS7960 controller
    UART2 /dev/ttyAMA2 / GPIO4,5   ODESC ODrive v3.6-compatible controller
    UART3 /dev/ttyAMA3 / GPIO8,9   GD32 Ender stepper controller
    UART4 /dev/ttyAMA4 / GPIO12,13 BNO080 UART-SHTP IMU

The existing per-device modules remain the protocol implementations. Set
ROVER_REPO below if this file is not stored inside the rover repository.
"""

import atexit
import importlib.util
import os
from pathlib import Path
import threading
import time


DEFAULT_REPO = Path.home() / "Documents" / "Autonomous-Garden-Rover"
ROVER_REPO = Path(os.environ.get("ROVER_REPO", DEFAULT_REPO))

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


def _find_repo_root():
    if "__file__" in globals():
        candidate = Path(__file__).resolve().parents[2]
        if (candidate / "README.md").is_file():
            return candidate
    if (ROVER_REPO / "README.md").is_file():
        return ROVER_REPO
    raise RuntimeError(
        "Rover repository not found; set ROVER_REPO at the top of robot_controller.py"
    )


REPO_ROOT = _find_repo_root()


def _load_module(name, relative_path):
    path = REPO_ROOT / relative_path
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError("Cannot load {}".format(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


stm32_driver = _load_module(
    "rover_stm32_driver", "bts7960-controller/raspberry-pi/main.py"
)
odesc_driver = _load_module(
    "rover_odesc_driver", "odesc/raspberry-pi/odesc_uart.py"
)
gd32_driver = _load_module(
    "rover_gd32_driver", "motor-controller/raspberry-pi/stepper_controller.py"
)
bno080_driver = _load_module(
    "rover_bno080_driver", "bno080/raspberry-pi/bno080_uart.py"
)


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
