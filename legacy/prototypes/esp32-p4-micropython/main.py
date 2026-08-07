"""ESP32-P4 MicroPython link test for the STM32 rover motor controller.

Copy this complete file to the ESP32-P4 as ``main.py`` using Thonny.

Connected wires:
    ESP32-P4 GPIO21 and GPIO22 -> STM32 PA9 and PA10
    ESP32-P4 GND               -> STM32 GND

The exact TX/RX orientation does not need to be known. At startup the script
tries:

    1. ESP32 GPIO22 TX -> STM32 PA10 RX
       ESP32 GPIO21 RX <- STM32 PA9 TX
    2. ESP32 GPIO21 TX -> STM32 PA10 RX
       ESP32 GPIO22 RX <- STM32 PA9 TX

It only accepts a connection after receiving a valid STM32 heartbeat and the
exact ``OK PONG`` reply to ``PING``. No background thread is used, so the
Thonny shell remains interactive.
"""

import time
from machine import Pin, UART


UART_ID = 1
UART_BAUD = 115200
GPIO_A = 21
GPIO_B = 22

# Preferred orientation is tried first.
PREFERRED_TX_PIN = GPIO_B
PREFERRED_RX_PIN = GPIO_A

HEARTBEAT_WAIT_MS = 1600
PING_TIMEOUT_MS = 700
LINE_LIMIT = 220
EVENT_LIMIT = 30

uart = None
uart_tx_pin = None
uart_rx_pin = None
connected = False
last_heartbeat_ms = None
last_heartbeat_received_ms = None
last_reply = ""
last_error = ""
rx_buffer = b""
events = []


def _ticks_ms():
    return time.ticks_ms()


def _ticks_add(value, delta):
    return time.ticks_add(value, delta)


def _ticks_diff(a, b):
    return time.ticks_diff(a, b)


def _remember(line):
    events.append(line)
    if len(events) > EVENT_LIMIT:
        events.pop(0)


def _close_uart():
    global uart
    if uart is not None:
        try:
            uart.deinit()
        except Exception:
            pass
        uart = None
    # Release both connected wires before assigning the next orientation.
    Pin(GPIO_A, Pin.IN)
    Pin(GPIO_B, Pin.IN)
    time.sleep_ms(20)


def _open_uart(tx_pin, rx_pin):
    global uart, uart_tx_pin, uart_rx_pin, rx_buffer
    _close_uart()
    uart_tx_pin = int(tx_pin)
    uart_rx_pin = int(rx_pin)
    rx_buffer = b""
    uart = UART(
        UART_ID,
        baudrate=UART_BAUD,
        bits=8,
        parity=None,
        stop=1,
        tx=Pin(uart_tx_pin),
        rx=Pin(uart_rx_pin),
        timeout=20,
    )
    return uart


def _read_lines():
    global rx_buffer
    lines = []
    if uart is None:
        return lines

    while uart.any():
        data = uart.read()
        if not data:
            break
        for byte in data:
            if byte == 10:
                line = rx_buffer.decode("ascii", "ignore").strip()
                rx_buffer = b""
                if line:
                    lines.append(line)
            elif byte != 13:
                if 32 <= byte <= 126 and len(rx_buffer) < LINE_LIMIT:
                    rx_buffer += bytes((byte,))
                elif len(rx_buffer) >= LINE_LIMIT:
                    rx_buffer = b""
    return lines


def _handle_line(line, show=False):
    global last_heartbeat_ms, last_heartbeat_received_ms, last_reply
    _remember(line)
    if line.startswith("HB "):
        fields = line.split()
        if len(fields) == 2:
            try:
                last_heartbeat_ms = int(fields[1])
                last_heartbeat_received_ms = _ticks_ms()
                if show:
                    print("[stm32] " + line)
                return "heartbeat"
            except ValueError:
                pass
    if line.startswith("READY STM32F103C6"):
        last_heartbeat_received_ms = _ticks_ms()
        if show:
            print("[stm32] " + line)
        return "ready"
    last_reply = line
    if show:
        print("[stm32] " + line)
    return "reply"


def _wait_for_heartbeat(timeout_ms):
    deadline = _ticks_add(_ticks_ms(), int(timeout_ms))
    while _ticks_diff(deadline, _ticks_ms()) > 0:
        for line in _read_lines():
            kind = _handle_line(line)
            if kind in ("heartbeat", "ready"):
                return line
        time.sleep_ms(5)
    return ""


def _wait_for_reply(expected, timeout_ms):
    deadline = _ticks_add(_ticks_ms(), int(timeout_ms))
    while _ticks_diff(deadline, _ticks_ms()) > 0:
        for line in _read_lines():
            kind = _handle_line(line)
            if kind in ("heartbeat", "ready"):
                continue
            if line == expected or line.startswith(expected + " "):
                return line
            if line.startswith("ERR"):
                return line
        time.sleep_ms(5)
    return ""


def _drain(duration_ms=80):
    deadline = _ticks_add(_ticks_ms(), int(duration_ms))
    while _ticks_diff(deadline, _ticks_ms()) > 0:
        for line in _read_lines():
            _handle_line(line)
        time.sleep_ms(5)


def _probe(tx_pin, rx_pin):
    print(
        "Trying ESP32-P4 TX GPIO{} / RX GPIO{}...".format(
            tx_pin, rx_pin
        )
    )
    _open_uart(tx_pin, rx_pin)

    # The STM32 emits HB once per second. Do not send on a candidate wiring
    # until its RX side has first proved that it is connected to STM32 PA9 TX.
    heartbeat = _wait_for_heartbeat(HEARTBEAT_WAIT_MS)
    if not heartbeat:
        print("  no valid STM32 heartbeat")
        return False

    print("  received {}".format(heartbeat))
    for attempt in range(1, 4):
        _drain(20)
        uart.write(b"PING\n")
        reply = _wait_for_reply("OK PONG", PING_TIMEOUT_MS)
        if reply == "OK PONG":
            print("  UART round trip verified: OK PONG")
            return True
        if reply.startswith("ERR"):
            print("  STM32 error: {}".format(reply))
            return False
        print("  PING attempt {} timed out".format(attempt))
    return False


def auto_connect():
    """Try GPIO21/22 in both orientations and verify a complete round trip."""
    global connected, last_error
    connected = False
    last_error = ""

    candidates = (
        (PREFERRED_TX_PIN, PREFERRED_RX_PIN),
        (PREFERRED_RX_PIN, PREFERRED_TX_PIN),
    )
    errors = []
    for tx_pin, rx_pin in candidates:
        try:
            if _probe(tx_pin, rx_pin):
                connected = True
                print(
                    "STM32 CONNECTED: ESP32 TX GPIO{} -> STM32 PA10, "
                    "ESP32 RX GPIO{} <- STM32 PA9".format(
                        uart_tx_pin, uart_rx_pin
                    )
                )
                return True
            errors.append(
                "TX GPIO{} RX GPIO{} did not verify".format(tx_pin, rx_pin)
            )
        except Exception as exc:
            errors.append(
                "TX GPIO{} RX GPIO{}: {}".format(tx_pin, rx_pin, exc)
            )

    _close_uart()
    last_error = "; ".join(errors)
    print("STM32 NOT FOUND: " + last_error)
    print("Check STM32 power, common GND, PA9/PA10 wiring, and 115200 firmware.")
    return False


def check_connection(retry_swapped=True):
    """Return True only when PING receives the exact STM32 OK PONG reply."""
    global connected, last_error
    if uart is None:
        return auto_connect() if retry_swapped else False
    try:
        _drain(20)
        uart.write(b"PING\n")
        reply = _wait_for_reply("OK PONG", PING_TIMEOUT_MS)
        connected = reply == "OK PONG"
        if connected:
            last_error = ""
            print("STM32 link OK: {}".format(reply))
            return True
        last_error = reply or "No reply to PING"
    except Exception as exc:
        connected = False
        last_error = str(exc)

    print("STM32 link failed: {}".format(last_error))
    return auto_connect() if retry_swapped else False


def ping():
    """Ping the STM32 and return ``OK PONG`` or raise a clear error."""
    global connected, last_error
    if uart is None and not auto_connect():
        raise RuntimeError(last_error or "STM32 is not connected")
    _drain(20)
    uart.write(b"PING\n")
    reply = _wait_for_reply("OK PONG", PING_TIMEOUT_MS)
    if reply != "OK PONG":
        connected = False
        last_error = reply or "No reply from STM32"
        raise RuntimeError(last_error)
    connected = True
    last_error = ""
    return reply


def command(text, expected=None, timeout_ms=1000):
    """Send one raw STM32 command and return its non-heartbeat response."""
    global connected, last_error
    if uart is None and not auto_connect():
        raise RuntimeError(last_error or "STM32 is not connected")
    text = str(text).strip()
    if not text or "\n" in text or "\r" in text:
        raise ValueError("command must be exactly one non-empty line")
    _drain(20)
    uart.write((text + "\n").encode("ascii"))
    deadline = _ticks_add(_ticks_ms(), int(timeout_ms))
    while _ticks_diff(deadline, _ticks_ms()) > 0:
        for line in _read_lines():
            kind = _handle_line(line)
            if kind in ("heartbeat", "ready"):
                continue
            if expected is None or line == expected or line.startswith(expected):
                connected = not line.startswith("ERR")
                if line.startswith("ERR"):
                    last_error = line
                else:
                    last_error = ""
                return line
        time.sleep_ms(5)
    connected = False
    last_error = "No STM32 reply to {}".format(text)
    raise RuntimeError(last_error)


def motor_status():
    """Return the raw MSTAT telemetry line without changing motor state."""
    return command("MSTATUS", "MSTAT ", 1200)


def currents():
    """Return both BTS7960 current-sense ADC readings as a dictionary."""
    line = motor_status()
    fields = line.split()
    if len(fields) != 17 or fields[0] != "MSTAT":
        raise RuntimeError("Malformed STM32 MSTAT: {!r}".format(line))
    return {
        "motor1": {
            "direction": fields[2],
            "duty": int(fields[3]),
            "r_is_raw": int(fields[4]),
            "l_is_raw": int(fields[5]),
            "r_is_mv": int(fields[6]),
            "l_is_mv": int(fields[7]),
        },
        "motor2": {
            "direction": fields[9],
            "duty": int(fields[10]),
            "r_is_raw": int(fields[11]),
            "l_is_raw": int(fields[12]),
            "r_is_mv": int(fields[13]),
            "l_is_mv": int(fields[14]),
        },
        "watchdog_stopped": fields[15] == "WD" and fields[16] == "1",
    }


def stop_all():
    """Immediately command both STM32 BTS7960 channels to stop."""
    return command("MSTOP ALL", "OK MSTOP ALL")


def monitor(seconds=5):
    """Print asynchronous STM32 heartbeat/fault lines for a fixed duration."""
    deadline = _ticks_add(_ticks_ms(), int(float(seconds) * 1000))
    while _ticks_diff(deadline, _ticks_ms()) > 0:
        for line in _read_lines():
            _handle_line(line, show=True)
        time.sleep_ms(10)


def diagnostics():
    age = None
    if last_heartbeat_received_ms is not None:
        age = _ticks_diff(_ticks_ms(), last_heartbeat_received_ms)
    result = {
        "connected": connected,
        "uart_id": UART_ID,
        "tx_pin": uart_tx_pin,
        "rx_pin": uart_rx_pin,
        "heartbeat_stm32_ms": last_heartbeat_ms,
        "heartbeat_age_ms": age,
        "last_reply": last_reply,
        "last_error": last_error,
        "recent_lines": list(events),
    }
    print(result)
    return result


def stm32_help():
    print("ESP32-P4 STM32 UART shell commands:")
    print("  auto_connect()              try GPIO21/22 both ways")
    print("  check_connection()          verify PING, retry swapped if needed")
    print("  ping()                      returns 'OK PONG'")
    print("  motor_status()              raw motor/current telemetry")
    print("  currents()                  parsed current-sense ADC values")
    print("  stop_all()                  stop both BTS7960 channels")
    print("  monitor(5)                  print heartbeat/faults for 5 seconds")
    print("  command('CAPS')             send one raw protocol command")
    print("  diagnostics()               pins, heartbeat age, and recent lines")


try:
    auto_connect()
except Exception as startup_error:
    last_error = str(startup_error)
    print("STM32 UART startup error: {}".format(startup_error))

stm32_help()
