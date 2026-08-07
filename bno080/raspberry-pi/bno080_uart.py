"""Standalone BNO080 UART-SHTP client for Raspberry Pi 5 and Thonny.

Dependency: pyserial (Raspberry Pi OS package ``python3-serial``).

Default wiring:
    Pi GPIO12 / physical 32 / UART4 TX -> BNO080 RX
    Pi GPIO13 / physical 33 / UART4 RX <- BNO080 TX
    Pi GND                              -> BNO080 GND

Set the BNO080 interface mode before power-up: PS1 high and PS0 low.
This is normal Python 3 code, not MicroPython code.
"""

import math
import struct
import time

try:
    import serial
except ImportError as exc:
    raise ImportError("Install Raspberry Pi OS package python3-serial") from exc


UART_DEVICE = "/dev/ttyAMA4"
UART_BAUD = 3_000_000
UART_TX_GPIO = 12
UART_RX_GPIO = 13

# SH-2 report IDs used by this client.
REPORT_ACCELEROMETER = 0x01
REPORT_GYROSCOPE = 0x02
REPORT_MAGNETOMETER = 0x03
REPORT_LINEAR_ACCELERATION = 0x04
REPORT_ROTATION_VECTOR = 0x05
REPORT_RAW_ACCELEROMETER = 0x14
REPORT_RAW_GYROSCOPE = 0x15
REPORT_RAW_MAGNETOMETER = 0x16
REPORT_BASE_TIMESTAMP = 0xFB
SET_FEATURE_COMMAND = 0xFD
COMMAND_REQUEST = 0xF2
ME_CALIBRATE_COMMAND = 0x07
ME_CAL_CONFIG = 0x00

CHANNEL_CONTROL = 2
CHANNEL_INPUT_NORMAL = 3
CHANNEL_INPUT_WAKE = 4
CHANNEL_EXECUTABLE = 1

# Report interval values are microseconds.
NAVIGATION_REPORTS = (
    (REPORT_ROTATION_VECTOR, 20_000),
    (REPORT_MAGNETOMETER, 20_000),
    (REPORT_ACCELEROMETER, 20_000),
    (REPORT_LINEAR_ACCELERATION, 20_000),
    (REPORT_GYROSCOPE, 10_000),
)
RAW_REPORTS = (
    (REPORT_RAW_ACCELEROMETER, 20_000),
    (REPORT_RAW_GYROSCOPE, 20_000),
    (REPORT_RAW_MAGNETOMETER, 20_000),
)

REPORT_FORMAT = {
    REPORT_ACCELEROMETER: (10, 3, 2.0 ** -8, True, "acceleration_m_s2"),
    REPORT_GYROSCOPE: (10, 3, 2.0 ** -9, True, "gyro_rad_s"),
    REPORT_MAGNETOMETER: (10, 3, 2.0 ** -4, True, "magnetic_uT"),
    REPORT_LINEAR_ACCELERATION: (10, 3, 2.0 ** -8, True, "linear_acceleration_m_s2"),
    REPORT_ROTATION_VECTOR: (14, 4, 2.0 ** -14, True, "quaternion"),
    REPORT_RAW_ACCELEROMETER: (16, 3, 1.0, False, "raw_acceleration"),
    REPORT_RAW_GYROSCOPE: (16, 3, 1.0, False, "raw_gyro"),
    REPORT_RAW_MAGNETOMETER: (16, 3, 1.0, False, "raw_magnetic"),
}

ACCURACY_LABELS = ("unreliable", "low", "medium", "high")


class BNO080UART:
    """Small UART-SHTP/SH-2 implementation for selected BNO080 reports."""

    def __init__(self, device=UART_DEVICE, baudrate=UART_BAUD):
        self.device = str(device)
        self.baudrate = int(baudrate)
        self.port = None
        self.tx_sequence = [0] * 6
        self.command_sequence = 0
        self.readings = {}
        self.accuracy = {}
        self.report_sequence = {}
        self.last_packet_monotonic = None
        self.raw_reports_enabled = False

    def open(self):
        self.close()
        self.port = serial.Serial(
            port=self.device,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            write_timeout=1.0,
        )
        self.port.reset_input_buffer()
        print(
            "BNO080 UART opened: {} at {} baud (Pi TX GPIO{}, RX GPIO{})".format(
                self.device, self.baudrate, UART_TX_GPIO, UART_RX_GPIO
            )
        )

        available = self._request_buffer(2.0)
        if available is None:
            self.close()
            raise RuntimeError(
                "No UART-SHTP response; check TX/RX, ground, power, PS1/PS0"
            )
        print("BNO080 UART-SHTP ROUND TRIP VERIFIED: buffer={} bytes".format(available))

        self.soft_reset()
        self._enable_navigation_reports()

        started = time.monotonic()
        retry_at = started + 2.0
        deadline = started + 6.0
        retried = False
        while time.monotonic() < deadline:
            self.poll(0.25)
            if REPORT_ROTATION_VECTOR in self.readings and REPORT_MAGNETOMETER in self.readings:
                print("BNO080 initialized; fused navigation reports received")
                try:
                    self.begin_calibration()
                except Exception as exc:
                    print(
                        "BNO080 calibration start failed; sensor remains connected: {}".format(
                            exc
                        )
                    )
                return self
            if not retried and time.monotonic() >= retry_at:
                retried = True
                print("No fused reports yet; retrying BNO080 feature setup")
                self._enable_navigation_reports()
        self.close()
        raise RuntimeError("BNO080 accepted SHTP but no fused reports arrived")

    def _enable_navigation_reports(self):
        for report_id, interval_us in NAVIGATION_REPORTS:
            self.enable_report(report_id, interval_us)

    def soft_reset(self):
        """Reset SH-2 state so repeated host runs restart channel sequencing."""
        print("Resetting BNO080 SHTP state...")
        self._send_shtp(CHANNEL_EXECUTABLE, b"\x01")
        time.sleep(0.5)
        self._send_shtp(CHANNEL_EXECUTABLE, b"\x01")
        time.sleep(0.5)
        self.readings.clear()
        self.accuracy.clear()
        self.report_sequence.clear()
        self.poll(0.25)
        print("BNO080 SHTP reset complete")

    def close(self):
        if self.port is not None:
            try:
                self.port.close()
            finally:
                self.port = None

    @staticmethod
    def _escape(data):
        encoded = bytearray()
        for value in data:
            if value in (0x7D, 0x7E):
                encoded.extend((0x7D, value ^ 0x20))
            else:
                encoded.append(value)
        return encoded

    def _slow_write(self, data):
        """Write with the minimum UART-SHTP host inter-byte delay."""
        for value in data:
            self.port.write(bytes((value,)))
            self.port.flush()
            time.sleep(0.001)

    def _write_uart_frame(self, protocol_id, payload=b""):
        body = bytes((protocol_id,)) + bytes(payload)
        frame = b"\x7e" + self._escape(body) + b"\x7e"
        self._slow_write(frame)

    def _read_uart_frame(self, timeout_s):
        deadline = time.monotonic() + float(timeout_s)
        frame = bytearray()
        in_frame = False
        escaped = False

        while time.monotonic() < deadline:
            data = self.port.read(1)
            if not data:
                continue
            value = data[0]

            if value == 0x7E:
                if in_frame and frame:
                    return bytes(frame)
                frame.clear()
                in_frame = True
                escaped = False
                continue
            if not in_frame:
                continue
            if escaped:
                frame.append(value ^ 0x20)
                escaped = False
            elif value == 0x7D:
                escaped = True
            else:
                frame.append(value)
        return None

    def _request_buffer(self, timeout_s=1.0):
        """Send Buffer Status Query and return granted host-write bytes."""
        self._write_uart_frame(0x00)
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            frame = self._read_uart_frame(deadline - time.monotonic())
            if frame is None:
                break
            protocol_id = frame[0]
            payload = frame[1:]
            if protocol_id == 0x00 and len(payload) >= 2:
                return payload[0] | (payload[1] << 8)
            if protocol_id == 0x01:
                self._handle_shtp(payload)
        return None

    def _send_shtp(self, channel, payload):
        if not 0 <= channel < len(self.tx_sequence):
            raise ValueError("Unsupported SHTP channel")
        payload = bytes(payload)
        cargo_length = len(payload) + 4
        available = self._request_buffer(1.0)
        if available is None or available < cargo_length:
            raise RuntimeError(
                "BNO080 did not grant {} SHTP write bytes".format(cargo_length)
            )

        sequence = self.tx_sequence[channel]
        header = struct.pack("<HBB", cargo_length, channel, sequence)
        self._write_uart_frame(0x01, header + payload)
        self.tx_sequence[channel] = (sequence + 1) & 0xFF

    def enable_report(self, report_id, interval_us):
        """Send an SH-2 Set Feature command for one periodic sensor report."""
        payload = struct.pack(
            "<BBBHIII",
            SET_FEATURE_COMMAND,
            int(report_id),
            0,                 # feature flags
            0,                 # change sensitivity
            int(interval_us),
            0,                 # batch interval
            0,                 # sensor-specific configuration
        )
        self._send_shtp(CHANNEL_CONTROL, payload)

    def enable_raw_reports(self):
        for report_id, interval_us in RAW_REPORTS:
            self.enable_report(report_id, interval_us)
        self.raw_reports_enabled = True
        print("BNO080 raw accel/gyro/magnetometer reports enabled")

    def _send_command(self, command, parameters=()):
        parameters = tuple(int(value) & 0xFF for value in parameters)
        if len(parameters) > 9:
            raise ValueError("SH-2 commands accept at most nine parameters")
        payload = bytearray(12)
        payload[0] = COMMAND_REQUEST
        payload[1] = self.command_sequence
        payload[2] = int(command) & 0xFF
        payload[3 : 3 + len(parameters)] = bytes(parameters)
        self._send_shtp(CHANNEL_CONTROL, payload)
        self.command_sequence = (self.command_sequence + 1) & 0xFF

    def begin_calibration(self):
        """Enable dynamic accelerometer, gyroscope, and magnetometer calibration."""
        self._send_command(
            ME_CALIBRATE_COMMAND,
            (1, 1, 1, ME_CAL_CONFIG, 0, 0, 0, 0, 0),
        )
        print("BNO080 dynamic accel/gyro/magnetometer calibration started")

    def poll(self, timeout_s=0.1):
        """Receive and process UART frames for up to timeout_s."""
        deadline = time.monotonic() + float(timeout_s)
        processed = 0
        while time.monotonic() < deadline:
            frame = self._read_uart_frame(min(0.05, deadline - time.monotonic()))
            if frame is None:
                continue
            if frame[0] == 0x01:
                self._handle_shtp(frame[1:])
                processed += 1
        return processed

    def _handle_shtp(self, cargo):
        if len(cargo) < 4:
            return
        packet_length, channel, _sequence = struct.unpack_from("<HBB", cargo)
        packet_length &= 0x7FFF
        if packet_length < 4 or packet_length > len(cargo):
            return
        payload = cargo[4:packet_length]
        self.last_packet_monotonic = time.monotonic()
        if channel in (CHANNEL_INPUT_NORMAL, CHANNEL_INPUT_WAKE):
            self._parse_report_batch(payload)

    def _parse_report_batch(self, payload):
        offset = 0
        while offset < len(payload):
            report_id = payload[offset]
            if report_id == REPORT_BASE_TIMESTAMP:
                report_length = 5
            elif report_id in REPORT_FORMAT:
                report_length = REPORT_FORMAT[report_id][0]
            else:
                # Control/advertisement reports are irrelevant to sensor output.
                return
            if offset + report_length > len(payload):
                return
            report = payload[offset : offset + report_length]
            if report_id in REPORT_FORMAT:
                self._parse_sensor_report(report)
            offset += report_length

    def _parse_sensor_report(self, report):
        report_id = report[0]
        _length, count, scalar, signed, name = REPORT_FORMAT[report_id]
        values = []
        fmt = "<h" if signed else "<H"
        for index in range(count):
            raw = struct.unpack_from(fmt, report, 4 + index * 2)[0]
            values.append(raw * scalar)
        self.readings[report_id] = tuple(values)
        self.accuracy[report_id] = report[2] & 0x03
        self.report_sequence[report_id] = report[1]
        self.readings[name] = tuple(values)

    def snapshot(self, timeout_s=0.2):
        self.poll(timeout_s)
        required = (
            "quaternion",
            "magnetic_uT",
            "gyro_rad_s",
            "acceleration_m_s2",
            "linear_acceleration_m_s2",
        )
        missing = [name for name in required if name not in self.readings]
        if missing:
            raise RuntimeError("Missing BNO080 reports: " + ", ".join(missing))
        quaternion = self.readings["quaternion"]
        return {
            "heading_deg": quaternion_to_heading(quaternion),
            "quaternion": quaternion,
            "magnetic_uT": self.readings["magnetic_uT"],
            "gyro_rad_s": self.readings["gyro_rad_s"],
            "acceleration_m_s2": self.readings["acceleration_m_s2"],
            "linear_acceleration_m_s2": self.readings["linear_acceleration_m_s2"],
            "magnetometer_accuracy": self.accuracy.get(REPORT_MAGNETOMETER, 0),
            "monotonic_s": time.monotonic(),
        }

    def raw_snapshot(self, timeout_s=0.2):
        if not self.raw_reports_enabled:
            raise RuntimeError("Raw reports are disabled; call enable_raw_reports()")
        self.poll(timeout_s)
        required = ("raw_acceleration", "raw_gyro", "raw_magnetic")
        missing = [name for name in required if name not in self.readings]
        if missing:
            raise RuntimeError("Missing raw reports: " + ", ".join(missing))
        return {name: self.readings[name] for name in required}


def quaternion_to_heading(quaternion):
    """Convert (i, j, k, real) rotation quaternion to 0..360 degree yaw."""
    i, j, k, real = quaternion
    yaw = math.atan2(
        2.0 * (real * k + i * j),
        1.0 - 2.0 * (j * j + k * k),
    )
    return math.degrees(yaw) % 360.0


sensor = None


def connect(device=UART_DEVICE, baudrate=UART_BAUD):
    global sensor
    disconnect()
    sensor = BNO080UART(device, baudrate)
    try:
        sensor.open()
    except Exception:
        sensor = None
        raise
    return sensor


def disconnect():
    global sensor
    if sensor is not None:
        sensor.close()
        sensor = None


def _sensor():
    if sensor is None:
        raise RuntimeError("BNO080 is not connected; call connect()")
    return sensor


def sample():
    return _sensor().snapshot()


def magnetometer():
    """Return calibrated (x, y, z) magnetic field in microtesla."""
    reading = _sensor().snapshot()
    return reading["magnetic_uT"]


def print_magnetometer():
    """Print calibrated magnetic field, fused heading, and accuracy."""
    reading = _sensor().snapshot()
    values = reading["magnetic_uT"]
    accuracy = reading["magnetometer_accuracy"]
    print(
        "mag x={:+.4f} y={:+.4f} z={:+.4f} uT | heading={:.2f} deg | accuracy={} ({})".format(
            values[0], values[1], values[2], reading["heading_deg"],
            accuracy, ACCURACY_LABELS[accuracy]
        )
    )
    return values


def stream_magnetometer(rate_hz=5.0, seconds=None):
    """Print only magnetometer and heading data until Ctrl+C or a time limit."""
    rate_hz = float(rate_hz)
    if rate_hz <= 0 or rate_hz > 20:
        raise ValueError("rate_hz must be greater than 0 and at most 20")
    deadline = None if seconds is None else time.monotonic() + float(seconds)
    interval = 1.0 / rate_hz
    print("Streaming BNO080 magnetometer data; press Ctrl+C to stop")
    try:
        while deadline is None or time.monotonic() < deadline:
            started = time.monotonic()
            print_magnetometer()
            time.sleep(max(0.0, interval - (time.monotonic() - started)))
    except KeyboardInterrupt:
        print("BNO080 magnetometer stream stopped")


def enable_raw_reports():
    _sensor().enable_raw_reports()


def raw_sample():
    return _sensor().raw_snapshot()


def _vector(label, values, unit):
    print(
        "{} x={:+.4f} y={:+.4f} z={:+.4f} {}".format(
            label, values[0], values[1], values[2], unit
        )
    )


def print_sample(reading=None):
    if reading is None:
        reading = sample()
    q_i, q_j, q_k, q_real = reading["quaternion"]
    accuracy = reading["magnetometer_accuracy"]
    print(
        "heading={:.2f} deg magnetic | mag accuracy={} ({})".format(
            reading["heading_deg"], accuracy, ACCURACY_LABELS[accuracy]
        )
    )
    print(
        "quat i={:+.5f} j={:+.5f} k={:+.5f} real={:+.5f}".format(
            q_i, q_j, q_k, q_real
        )
    )
    _vector("mag", reading["magnetic_uT"], "uT")
    _vector("gyro", reading["gyro_rad_s"], "rad/s")
    _vector("accel", reading["acceleration_m_s2"], "m/s^2")
    _vector("linear", reading["linear_acceleration_m_s2"], "m/s^2")
    return reading


def print_raw(reading=None):
    if reading is None:
        reading = raw_sample()
    print("raw accel:", reading["raw_acceleration"])
    print("raw gyro: ", reading["raw_gyro"])
    print("raw mag:  ", reading["raw_magnetic"])
    return reading


def stream(rate_hz=5.0, seconds=None):
    """Print fused navigation data until Ctrl+C or an optional time limit."""
    rate_hz = float(rate_hz)
    if rate_hz <= 0 or rate_hz > 20:
        raise ValueError("rate_hz must be greater than 0 and at most 20")
    deadline = None if seconds is None else time.monotonic() + float(seconds)
    interval = 1.0 / rate_hz
    print("Streaming fused BNO080 data; press Ctrl+C to stop")
    try:
        while deadline is None or time.monotonic() < deadline:
            started = time.monotonic()
            print_sample()
            print()
            time.sleep(max(0.0, interval - (time.monotonic() - started)))
    except KeyboardInterrupt:
        print("BNO080 stream stopped")


def stream_raw(rate_hz=5.0, seconds=None):
    """Enable and print raw vectors until Ctrl+C or an optional time limit."""
    if not _sensor().raw_reports_enabled:
        enable_raw_reports()
    rate_hz = float(rate_hz)
    if rate_hz <= 0 or rate_hz > 20:
        raise ValueError("rate_hz must be greater than 0 and at most 20")
    deadline = None if seconds is None else time.monotonic() + float(seconds)
    interval = 1.0 / rate_hz
    print("Streaming raw BNO080 data; press Ctrl+C to stop")
    try:
        while deadline is None or time.monotonic() < deadline:
            started = time.monotonic()
            print_raw()
            time.sleep(max(0.0, interval - (time.monotonic() - started)))
    except KeyboardInterrupt:
        print("BNO080 raw stream stopped")


def calibration_status():
    """Print the accuracy field from the latest calibrated magnetometer report."""
    _sensor().poll(0.2)
    value = _sensor().accuracy.get(REPORT_MAGNETOMETER, 0)
    print("Magnetometer calibration: {} ({})".format(value, ACCURACY_LABELS[value]))
    return value


def begin_calibration():
    _sensor().begin_calibration()


def show_help():
    print("BNO080 standalone UART-SHTP shell commands:")
    print("  sample() | print_sample() | stream(5) | stream(5, 10)")
    print("  magnetometer() | print_magnetometer() | stream_magnetometer(5, 10)")
    print("  begin_calibration() | calibration_status()")
    print("  enable_raw_reports() | raw_sample() | print_raw() | stream_raw(5, 10)")
    print("  connect('/dev/ttyAMA4', 3000000) | disconnect()")


if __name__ == "__main__":
    try:
        connect()
    except Exception as exc:
        print("BNO080 initialization failed: {}".format(exc))
        print("Fix wiring/mode, then call connect().")

    show_help()
