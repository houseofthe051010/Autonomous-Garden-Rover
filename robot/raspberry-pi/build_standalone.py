"""Generate the single-file Thonny rover controller from validated drivers."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MODULAR_CONTROLLER = ROOT / "robot" / "raspberry-pi" / "robot_controller.py"
OUTPUT = ROOT / "robot" / "raspberry-pi" / "robot_controller_standalone.py"

DRIVERS = (
    ("stm32_driver", ROOT / "bts7960-controller" / "raspberry-pi" / "main.py"),
    ("odesc_driver", ROOT / "odesc" / "raspberry-pi" / "odesc_uart.py"),
    ("gd32_driver", ROOT / "motor-controller" / "raspberry-pi" / "stepper_controller.py"),
    ("bno080_driver", ROOT / "bno080" / "raspberry-pi" / "bno080_uart.py"),
)


def build():
    controller_source = MODULAR_CONTROLLER.read_text(encoding="ascii")
    controller_body = controller_source[controller_source.index("class RoverController:") :]

    lines = [
        '"""Standalone four-UART Raspberry Pi 5 rover controller.',
        "",
        "Generated from the repository's validated UART clients. This file can be",
        "pasted directly into Thonny and has no rover-repository path dependency.",
        '"""',
        "",
        "import atexit",
        "import threading",
        "import time",
        "import types",
        "",
        'STM32_PORT = "/dev/ttyAMA1"',
        'ODESC_PORT = "/dev/ttyAMA2"',
        'GD32_PORT = "/dev/ttyAMA3"',
        'BNO080_PORT = "/dev/ttyAMA4"',
        "ASCII_BAUD = 115200",
        "BNO080_BAUD = 3_000_000",
        "ODESC_MAX_TEST_TURNS_S = 2.0",
        "ODESC_MAX_TEST_SECONDS = 5.0",
        "ODESC_MAX_CONTROL_TURNS_S = 15.0",
        'ODESC_ARM_PHRASE = "ARM ODESC TEST"',
        'ODESC_CALIBRATE_PHRASE = "CALIBRATE ODESC AXIS"',
        'ODESC_MOTOR_CALIBRATE_PHRASE = "CALIBRATE ODESC MOTOR"',
        "",
        "def _embedded_module(name, source):",
        "    module = types.ModuleType(name)",
        "    module.__dict__['__name__'] = name",
        "    exec(compile(source, '<embedded ' + name + '>', 'exec'), module.__dict__)",
        "    return module",
        "",
    ]

    for name, path in DRIVERS:
        source = path.read_text(encoding="ascii")
        lines.append("{}_source = {!r}".format(name, source))
        lines.append("{} = _embedded_module({!r}, {}_source)".format(name, name, name))
        lines.append("")

    lines.append(controller_body)
    OUTPUT.write_text("\n".join(lines), encoding="ascii")
    print("Generated {}".format(OUTPUT))


if __name__ == "__main__":
    build()
