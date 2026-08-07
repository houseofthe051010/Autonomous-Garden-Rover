from machine import Pin, UART
import _thread
import time


BAUDRATE = 115200
DATA_PIN_A = 16
DATA_PIN_B = 17
# Leave this GPIO physically unconnected. It lets the ESP32 listen on each data
# wire without driving the other wire until the heartbeat identifies GD32 TX.
PROBE_TX_PIN = 18
STATUS_LED_PIN = 2
LINK_TIMEOUT_MS = 5000
DETECT_WINDOW_MS = 4500
MOTOR_MICROSTEPS_PER_REVOLUTION = 3200.0
MAX_DRIVER_RPM = 1000.0
MAX_PENDING_MOVES = 8
AXIS_STEPS_PER_UNIT = {"X": 80.0, "Y": 80.0, "Z": 400.0, "E": 93.0}
driver_rpm = {"X": 120.0, "Y": 120.0, "Z": 120.0, "E": 120.0}

status_led = Pin(STATUS_LED_PIN, Pin.OUT, value=0)
last_round_trip_ms = None
link_alive = False
receive_buffer = b""
move_sequence = 0
pending_moves = {}
switches = {"X": None, "Y": None, "Z": None}
switch_callback = None
uart_write_lock = _thread.allocate_lock()


def make_uart(tx_pin, rx_pin):
    return UART(
        2,
        baudrate=BAUDRATE,
        bits=8,
        parity=None,
        stop=1,
        tx=Pin(tx_pin),
        rx=Pin(rx_pin),
        rxbuf=1024,
    )


def heartbeat_seen_on(rx_pin):
    """Listen without driving either of the two connected data wires."""
    Pin(DATA_PIN_A, Pin.IN)
    Pin(DATA_PIN_B, Pin.IN)
    probe = make_uart(PROBE_TX_PIN, rx_pin)
    started_ms = time.ticks_ms()
    probe_buffer = b""

    while time.ticks_diff(time.ticks_ms(), started_ms) < DETECT_WINDOW_MS:
        waiting = probe.any()
        if waiting:
            chunk = probe.read(waiting)
            if chunk:
                probe_buffer = (probe_buffer + chunk)[-256:]
                if b"HB " in probe_buffer:
                    probe.deinit()
                    return True
        time.sleep_ms(10)

    probe.deinit()
    return False


def detect_uart_orientation():
    while True:
        print("Listening for Ender heartbeat on GPIO16...")
        if heartbeat_seen_on(DATA_PIN_A):
            return DATA_PIN_B, DATA_PIN_A

        print("No heartbeat on GPIO16; trying GPIO17...")
        if heartbeat_seen_on(DATA_PIN_B):
            return DATA_PIN_A, DATA_PIN_B

        print("No heartbeat found; checking both orientations again")


TX_PIN, RX_PIN = detect_uart_orientation()
link = make_uart(TX_PIN, RX_PIN)
print("Heartbeat found: ESP32 TX=GPIO{}, RX=GPIO{}".format(TX_PIN, RX_PIN))


def send_gcode(command):
    """Send one newline-terminated Marlin command."""
    uart_write_lock.acquire()
    try:
        link.write((command.strip() + "\n").encode())
    finally:
        uart_write_lock.release()


def enable_drivers():
    """Enable the shared hardware enable line for all four drivers."""
    send_gcode("M17")


def disable_drivers():
    """Disable the shared hardware enable line for all four drivers."""
    send_gcode("M18")


enable_steppers = enable_drivers
disable_steppers = disable_drivers


def _axis(axis):
    axis = axis.upper()
    if axis not in AXIS_STEPS_PER_UNIT:
        raise ValueError("axis must be X, Y, Z, or E")
    return axis


def set_speed(axis, rpm):
    """Set an axis's default speed for future move() calls."""
    axis = _axis(axis)
    rpm = float(rpm)
    if rpm <= 0 or rpm > MAX_DRIVER_RPM:
        raise ValueError("rpm must be greater than 0 and at most {}".format(MAX_DRIVER_RPM))
    driver_rpm[axis] = rpm
    return rpm


def move(axis, steps, rpm=None):
    """Queue signed driver microsteps; the sign controls DIR."""
    global move_sequence
    axis = _axis(axis)
    steps = int(steps)
    if not steps:
        raise ValueError("steps must not be zero")
    if len(pending_moves) >= MAX_PENDING_MOVES:
        raise RuntimeError("too many queued moves; wait for a DRV_DONE response")
    rpm = driver_rpm[axis] if rpm is None else float(rpm)
    if rpm <= 0 or rpm > MAX_DRIVER_RPM:
        raise ValueError("rpm must be greater than 0 and at most {}".format(MAX_DRIVER_RPM))

    move_sequence += 1
    move_id = move_sequence
    steps_per_unit = AXIS_STEPS_PER_UNIT[axis]
    distance_units = steps / steps_per_unit
    feedrate_units_min = rpm * MOTOR_MICROSTEPS_PER_REVOLUTION / steps_per_unit
    pending_moves[move_id] = (axis, steps)

    send_gcode("G91")
    if axis == "E":
        send_gcode("M83")
    send_gcode("G0 {}{:.5f} F{:.2f}".format(axis, distance_units, feedrate_units_min))
    send_gcode("M400")
    send_gcode("M118 DRV_DONE {} {} {}".format(move_id, axis, steps))
    print("Queued {} move {}: {} microsteps at {} RPM".format(axis, move_id, steps, rpm))
    return move_id


def move_sps(axis, steps, steps_per_second):
    """Queue a move using driver step pulses per second instead of RPM."""
    steps_per_second = float(steps_per_second)
    rpm = steps_per_second * 60.0 / MOTOR_MICROSTEPS_PER_REVOLUTION
    return move(axis, steps, rpm)


def xmove(steps, rpm=None):
    return move("X", steps, rpm)


def ymove(steps, rpm=None):
    return move("Y", steps, rpm)


def zmove(steps, rpm=None):
    return move("Z", steps, rpm)


def emove(steps, rpm=None):
    return move("E", steps, rpm)


def stop():
    """Immediately stop planner motion through Marlin's emergency parser."""
    pending_moves.clear()
    send_gcode("M410")
    print("Quick stop requested")


stop_x = stop


def forward(axis, steps, rpm=None):
    return move(axis, abs(int(steps)), rpm)


def backward(axis, steps, rpm=None):
    return move(axis, -abs(int(steps)), rpm)


def on_switch_change(callback):
    """Register callback(states) for debounced X/Y/Z switch changes."""
    global switch_callback
    switch_callback = callback


def switch_status():
    print("Switches X={} Y={} Z={}".format(
        switches["X"], switches["Y"], switches["Z"]
    ))
    return switches


def status():
    print("UART TX=GPIO{}, RX=GPIO{}, link={}, pending={}".format(
        TX_PIN, RX_PIN, "up" if link_alive else "down", len(pending_moves)
    ))
    print("Default RPM X={} Y={} Z={} E={}".format(
        driver_rpm["X"], driver_rpm["Y"], driver_rpm["Z"], driver_rpm["E"]
    ))
    switch_status()


def show_help():
    print("At the MicroPython >>> prompt use:")
    print("  move('X', 3200, 120) | move('E', -800, 60)")
    print("  xmove(...) | ymove(...) | zmove(...) | emove(...)")
    print("  move_sps('Y', -1600, 8000) | set_speed('Z', 30)")
    print("  forward('X', 800, 60) | backward('X', 800, 60)")
    print("  stop() | enable_drivers() | disable_drivers() | status()")
    print("Current and microstep mode are controlled by board hardware, not UART.")


def service_link():
    """Acknowledge heartbeats and verify a complete UART round trip."""
    global last_round_trip_ms, link_alive, receive_buffer
    waiting = link.any()
    if waiting:
        chunk = link.read(waiting)
        if chunk:
            receive_buffer += chunk

    while b"\n" in receive_buffer:
        line, receive_buffer = receive_buffer.split(b"\n", 1)
        line = line.strip()
        if not line:
            continue
        if line.startswith(b"HB "):
            sequence = line[3:].decode().strip()
            send_gcode("M118 HB_ACK " + sequence)
        elif line.startswith(b"HB_ACK_OK"):
            last_round_trip_ms = time.ticks_ms()
            if not link_alive:
                print("Ender UART round trip verified")
            link_alive = True
        elif line.startswith(b"DRV_DONE "):
            try:
                fields = line.decode().split()
                move_id = int(fields[1])
                axis = fields[2]
                steps = int(fields[3])
                expected_move = pending_moves.pop(move_id, None)
                if expected_move == (axis, steps):
                    print("{} move {} complete: {} microsteps".format(axis, move_id, steps))
                else:
                    print("Unexpected driver completion:", line)
            except (ValueError, IndexError):
                print("Malformed driver completion:", line)
        elif line.startswith(b"SW "):
            try:
                fields = line.decode().split()
                new_states = {
                    fields[1][0]: bool(int(fields[1][1:])),
                    fields[2][0]: bool(int(fields[2][1:])),
                    fields[3][0]: bool(int(fields[3][1:])),
                }
                changed = any(switches[axis] != new_states[axis] for axis in ("X", "Y", "Z"))
                switches.update(new_states)
                if changed:
                    switch_status()
                    if switch_callback:
                        try:
                            switch_callback(dict(switches))
                        except Exception as error:
                            print("Switch callback error:", error)
            except (ValueError, IndexError, KeyError):
                print("Malformed switch report:", line)
        elif line == b"ok" or line.startswith(b"HB_ACK "):
            pass
        else:
            print("Ender:", line)


def update_link_status():
    """Turn the status LED off if acknowledgments stop returning."""
    global link_alive
    if last_round_trip_ms is None:
        link_alive = False
    elif time.ticks_diff(time.ticks_ms(), last_round_trip_ms) > LINK_TIMEOUT_MS:
        if link_alive:
            print("Ender UART link lost")
        link_alive = False
    status_led.value(1 if link_alive else 0)


def link_worker():
    while True:
        service_link()
        update_link_status()
        time.sleep_ms(10)


_thread.start_new_thread(link_worker, ())
print("Four-driver UART worker started; returning to the MicroPython prompt.")
show_help()
