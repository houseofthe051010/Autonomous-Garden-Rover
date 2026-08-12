#!/usr/bin/env python3
"""Read-first ODESC v4.2 Hall setup helper for ODrive firmware 0.5.6.

The default commands never energize the motor. Configuration writes are gated
by --execute and an exact confirmation token. Motor calibration is deliberately
not automated by this tool; use the supervised checklist in HALL-FOC-BRINGUP.md.
"""

import argparse
import datetime
import json
import pathlib
import re
import sys
import time

from hall_analysis import MASK_NAMES, analyze_states, format_state

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial is required: sudo apt install python3-serial") from exc


BAUD = 115200
READ_TIMEOUT_S = 1.0
PREPARE_CONFIRMATION = "CONFIGURE-HALL-WITH-MOTOR-IDLE"
MASK_CONFIRMATION = "SAVE-VERIFIED-HALL-MASK"
NUMBER = re.compile(r"^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$")

STATUS_PROPERTIES = (
    "vbus_voltage",
    "error",
    "config.enable_brake_resistor",
    "config.dc_max_negative_current",
    "axis0.current_state",
    "axis0.error",
    "axis0.motor.error",
    "axis0.motor.is_calibrated",
    "axis0.motor.config.pole_pairs",
    "axis0.motor.config.calibration_current",
    "axis0.motor.config.current_lim",
    "axis0.motor.config.pre_calibrated",
    "axis0.encoder.error",
    "axis0.encoder.is_ready",
    "axis0.encoder.hall_state",
    "axis0.encoder.config.mode",
    "axis0.encoder.config.cpr",
    "axis0.encoder.config.bandwidth",
    "axis0.encoder.config.hall_polarity",
    "axis0.encoder.config.hall_polarity_calibrated",
    "axis0.encoder.config.pre_calibrated",
    "axis0.config.enable_sensorless_mode",
    "axis0.config.calibration_lockin.current",
    "axis0.config.calibration_lockin.ramp_time",
    "axis0.config.calibration_lockin.accel",
    "axis0.config.calibration_lockin.vel",
    "axis0.config.startup_motor_calibration",
    "axis0.config.startup_encoder_offset_calibration",
    "axis0.config.startup_closed_loop_control",
    "config.gpio9_mode",
    "config.gpio10_mode",
    "config.gpio11_mode",
)


class OdescAscii:
    def __init__(self, port):
        self.serial = serial.Serial(
            port=port,
            baudrate=BAUD,
            timeout=0.05,
            write_timeout=0.5,
        )
        time.sleep(0.15)
        self.serial.reset_input_buffer()

    def close(self):
        self.serial.close()

    def command(self, command):
        self.serial.write((command.rstrip() + "\n").encode("ascii"))
        self.serial.flush()

    def read_property(self, name, timeout=READ_TIMEOUT_S):
        self.serial.reset_input_buffer()
        self.command("r " + name)
        deadline = time.monotonic() + timeout
        lines = []
        while time.monotonic() < deadline:
            raw = self.serial.readline()
            if not raw:
                continue
            line = raw.decode("ascii", "replace").strip()
            if not line:
                continue
            lines.append(line)
            if NUMBER.fullmatch(line):
                return line
            if "invalid property" in line.lower() or "error" in line.lower():
                raise RuntimeError("{}: {}".format(name, line))
        raise TimeoutError("No numeric reply for {} (received: {})".format(name, lines))

    def read_int(self, name):
        return int(float(self.read_property(name)))

    def write_verified(self, name, value):
        self.command("w {} {}".format(name, value))
        time.sleep(0.04)
        actual = self.read_property(name)
        expected = float(value)
        if abs(float(actual) - expected) > max(1e-6, abs(expected) * 1e-5):
            raise RuntimeError("{} readback {} != {}".format(name, actual, value))
        print("verified {} = {}".format(name, actual))


def snapshot(device):
    result = {
        "captured_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "properties": {},
    }
    for name in STATUS_PROPERTIES:
        try:
            result["properties"][name] = device.read_property(name)
        except Exception as exc:
            result["properties"][name] = "READ_ERROR: {}".format(exc)
    return result


def save_snapshot(data, output):
    path = pathlib.Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")
    print("Snapshot written to {}".format(path.resolve()))


def print_status(device, output=None):
    data = snapshot(device)
    print(json.dumps(data, indent=2, sort_keys=True))
    if output:
        save_snapshot(data, output)


def require_idle(device):
    state = device.read_int("axis0.current_state")
    if state != 1:
        raise RuntimeError("Axis 0 must be IDLE (1); current state is {}".format(state))
    for name in ("error", "axis0.error", "axis0.motor.error"):
        value = device.read_int(name)
        if value:
            raise RuntimeError("{} is nonzero: {}".format(name, value))


def prepare_hall(device, args):
    pole_pairs = device.read_int("axis0.motor.config.pole_pairs")
    if pole_pairs <= 0 or pole_pairs > 100:
        raise RuntimeError("Implausible pole-pair count: {}".format(pole_pairs))
    writes = (
        ("axis0.config.startup_motor_calibration", 0),
        ("axis0.config.startup_encoder_offset_calibration", 0),
        ("axis0.config.startup_closed_loop_control", 0),
        ("axis0.config.enable_sensorless_mode", 0),
        ("config.enable_brake_resistor", 0),
        ("config.gpio9_mode", 0),
        ("config.gpio10_mode", 0),
        ("config.gpio11_mode", 0),
        ("axis0.encoder.config.mode", 1),
        ("axis0.encoder.config.cpr", pole_pairs * 6),
        ("axis0.encoder.config.bandwidth", 100),
        ("axis0.encoder.config.ignore_illegal_hall_state", 0),
        ("axis0.encoder.config.hall_polarity", 0),
        ("axis0.encoder.config.hall_polarity_calibrated", 0),
        ("axis0.encoder.config.pre_calibrated", 0),
    )
    print("Planned Hall preparation (no motor movement):")
    for name, value in writes:
        print("  {} = {}".format(name, value))
    print("  ss  (save configuration; reboot manually afterward)")

    if not args.execute:
        print("DRY RUN: no values were written")
        return
    if args.confirm != PREPARE_CONFIRMATION:
        raise RuntimeError("Refusing writes: confirmation token does not match")
    require_idle(device)
    before = snapshot(device)
    save_snapshot(before, args.snapshot)
    for name, value in writes:
        device.write_verified(name, value)
    require_idle(device)
    device.command("ss")
    time.sleep(0.5)
    print("Configuration saved. Keep the motor unloaded and reboot the ODESC manually.")
    print("After reboot, run status and then observe while slowly turning the shaft by hand.")


def observe(device, seconds, interval):
    require_idle(device)
    if device.read_int("axis0.encoder.config.mode") != 1:
        raise RuntimeError("Encoder mode is not HALL (1); run prepare-hall first")
    print("Turn the unpowered motor shaft slowly through at least two full revolutions.")
    samples = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        state = device.read_int("axis0.encoder.hall_state")
        if not samples or samples[-1] != state:
            print("raw Hall state {} ({})".format(state, format_state(state)))
        samples.append(state)
        time.sleep(interval)

    report = analyze_states(samples)
    transitions = " ".join(format_state(value) for value in report["raw_transitions"])
    print("Raw transitions: {}".format(transitions or "none"))
    for item in report["reports"]:
        corrected = " ".join(format_state(value) for value in item["coverage"])
        print(
            "mask {} ({:<16}) coverage=[{}] invalid={} jumps={}".format(
                item["mask"], item["name"], corrected,
                len(item["invalid"]), item["jumps"]
            )
        )
    candidate = report["candidate"]
    if candidate is None:
        raise RuntimeError(
            "No unique safe mask. Rotate more slowly through two full turns and inspect wiring/noise."
        )
    print("VERIFIED CANDIDATE: mask {} ({})".format(candidate["mask"], candidate["name"]))
    print("This is diagnostic only; no configuration was changed.")


def set_mask(device, args):
    mask = args.mask
    print("Planned mask: {} ({})".format(mask, MASK_NAMES[mask]))
    if not args.execute:
        print("DRY RUN: no values were written")
        return
    if args.confirm != MASK_CONFIRMATION:
        raise RuntimeError("Refusing writes: confirmation token does not match")
    require_idle(device)
    if device.read_int("axis0.encoder.config.mode") != 1:
        raise RuntimeError("Encoder mode must be HALL (1)")
    device.write_verified("axis0.encoder.config.hall_polarity", mask)
    device.write_verified("axis0.encoder.config.hall_polarity_calibrated", 1)
    require_idle(device)
    device.command("ss")
    time.sleep(0.5)
    print("Mask saved. This command did not energize or calibrate the motor.")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    subparsers = parser.add_subparsers(dest="action", required=True)

    status = subparsers.add_parser("status", help="read configuration and errors")
    status.add_argument("--output", help="optional JSON snapshot path")

    prepare = subparsers.add_parser("prepare-hall", help="stage Hall mode; dry-run by default")
    prepare.add_argument("--execute", action="store_true")
    prepare.add_argument("--confirm", default="")
    prepare.add_argument("--snapshot", default="odesc-before-hall-setup.json")

    observe_parser = subparsers.add_parser("observe", help="sample raw Hall states while IDLE")
    observe_parser.add_argument("--seconds", type=float, default=15.0)
    observe_parser.add_argument("--interval", type=float, default=0.03)

    set_mask_parser = subparsers.add_parser("set-mask", help="save a manually verified mask")
    set_mask_parser.add_argument("--mask", type=int, choices=tuple(MASK_NAMES), required=True)
    set_mask_parser.add_argument("--execute", action="store_true")
    set_mask_parser.add_argument("--confirm", default="")
    return parser.parse_args()


def main():
    args = parse_args()
    device = OdescAscii(args.port)
    try:
        if args.action == "status":
            print_status(device, args.output)
        elif args.action == "prepare-hall":
            prepare_hall(device, args)
        elif args.action == "observe":
            observe(device, args.seconds, args.interval)
        elif args.action == "set-mask":
            set_mask(device, args)
    finally:
        device.close()


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, TimeoutError, ValueError) as exc:
        print("ERROR: {}".format(exc), file=sys.stderr)
        raise SystemExit(2)
