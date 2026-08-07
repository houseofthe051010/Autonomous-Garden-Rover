"""Read-only ODESC v3 sensorless configuration diagnostic.

Stop any other process that owns /dev/ttyAMA2 before running this file.
No motor state or configuration writes are performed.
"""

import json
import sys
from pathlib import Path


CONTROLLER_DIR = Path("/home/aditya/rover-controller")
if str(CONTROLLER_DIR) not in sys.path:
    sys.path.insert(0, str(CONTROLLER_DIR))

import robot_controller_standalone as rover


GLOBAL_PROPERTIES = (
    "fw_version_major",
    "fw_version_minor",
    "fw_version_revision",
    "fw_version_unreleased",
    "vbus_voltage",
    "config.enable_brake_resistor",
    "config.brake_resistance",
)

AXIS_PROPERTIES = (
    "current_state",
    "error",
    "config.enable_sensorless_mode",
    "config.startup_sensorless_control",
    "config.startup_closed_loop_control",
    "config.enable_step_dir",
    "config.sensorless_ramp.current",
    "config.sensorless_ramp.vel",
    "config.sensorless_ramp.accel",
    "config.sensorless_ramp.finish_distance",
    "config.sensorless_ramp.finish_on_vel",
    "config.sensorless_ramp.finish_on_distance",
    "sensorless_estimator.error",
    "sensorless_estimator.vel_estimate",
    "sensorless_estimator.config.pm_flux_linkage",
    "sensorless_estimator.config.observer_gain",
    "sensorless_estimator.config.pll_bandwidth",
    "motor.error",
    "motor.config.pre_calibrated",
    "motor.config.motor_type",
    "motor.config.pole_pairs",
    "motor.config.current_lim",
    "motor.config.calibration_current",
    "motor.config.resistance_calib_max_voltage",
    "motor.phase_resistance",
    "motor.phase_inductance",
    "encoder.error",
    "encoder.config.pre_calibrated",
    "controller.error",
    "controller.config.control_mode",
    "controller.config.input_mode",
    "controller.config.vel_limit",
    "controller.config.vel_gain",
    "controller.config.vel_integrator_gain",
)


def read_optional(link, property_name):
    try:
        return {"supported": True, "value": link.read_property(property_name)}
    except Exception as exc:
        return {"supported": False, "error": str(exc)}


def main():
    link = rover.odesc_driver.ODESCUART(
        rover.ODESC_PORT, rover.ASCII_BAUD, timeout=0.35
    ).open()
    try:
        report = {
            "global": {
                name: read_optional(link, name) for name in GLOBAL_PROPERTIES
            },
            "axes": {},
        }
        for axis in (0, 1):
            prefix = "axis{}.".format(axis)
            report["axes"][str(axis)] = {
                name: read_optional(link, prefix + name)
                for name in AXIS_PROPERTIES
            }
        print(json.dumps(report, indent=2, sort_keys=True))
    finally:
        link.close()


if __name__ == "__main__":
    main()
