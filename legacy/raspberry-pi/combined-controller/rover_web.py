"""Authenticated LAN dashboard for the four-UART Raspberry Pi rover controller.

Run this with Raspberry Pi OS Python 3, not MicroPython. This process must be
the only owner of /dev/ttyAMA1 through /dev/ttyAMA4, so stop Thonny's running
controller before starting it.
"""

import argparse
import base64
import hmac
import json
import math
import os
import sqlite3
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import robot_controller_standalone as rover_module


HOST = os.environ.get("ROVER_WEB_HOST", "0.0.0.0")
PORT = int(os.environ.get("ROVER_WEB_PORT", "8080"))
AUTH_USER = os.environ.get("ROVER_WEB_USER", "rover")
AUTH_PASSWORD = os.environ.get("ROVER_WEB_PASSWORD", "")
AUTH_PASSWORD_FILE = os.environ.get("ROVER_WEB_PASSWORD_FILE", "")
AUTH_REQUIRED = os.environ.get("ROVER_WEB_AUTH", "1").lower() not in (
    "0",
    "false",
    "no",
    "off",
)
MAX_BODY_BYTES = 8192
TANK_UPDATE_HZ = 15.0
TANK_DEADMAN_S = 0.50
ODESC_STATUS_INTERVAL_S = 5.0
ODESC_MOTION_DEADMAN_S = 0.8
ODESC_EVENT_LIMIT = 100
STEPPER_JOG_INTERVAL_S = 0.05
STEPPER_JOG_DEADMAN_S = 0.6
NETWORK_HELPER = os.environ.get(
    "ROVER_WIFI_HELPER", "/usr/local/sbin/rover-wifi-mode"
)
NETWORK_SWITCH_DELAY_S = 3.0
NETWORK_CHECK_INTERVAL_S = 5.0
NETWORK_FAILOVER_SECONDS = 20.0
NETWORK_MIN_SIGNAL_PERCENT = 15
TRIP_SAMPLE_INTERVAL_S = 0.20
TRIP_MAX_SAMPLES = 12000
TRIP_ACCEL_DEADBAND_M_S2 = 0.12
TRIP_VELOCITY_DAMPING_PER_S = 0.08
BATTERY_CELL_COUNT = 10
BATTERY_PARALLEL_COUNT = 4
BATTERY_CELL_CAPACITY_AH = 2.6
BATTERY_PACK_CAPACITY_AH = (
    BATTERY_PARALLEL_COUNT * BATTERY_CELL_CAPACITY_AH
)
BATTERY_HISTORY_PATH = Path(
    os.environ.get(
        "ROVER_HISTORY_DB",
        str(Path(__file__).with_name("rover-history.sqlite3")),
    )
)
DEFAULT_TANK_RAMP_PERCENT_S = 80.0
CONFIG_PATH = Path(
    os.environ.get(
        "ROVER_WEB_CONFIG",
        str(Path(__file__).with_name("rover-web-config.json")),
    )
)

if not AUTH_PASSWORD and AUTH_PASSWORD_FILE:
    AUTH_PASSWORD = Path(AUTH_PASSWORD_FILE).read_text(encoding="utf-8").strip()

ROBOT = rover_module.robot
ACTION_LOCK = threading.RLock()
IMU_IO_LOCK = threading.Lock()
IMU_STATE_LOCK = threading.Lock()
ODESC_STATE_LOCK = threading.Lock()
ODESC_MOTION_LOCK = threading.Lock()
STEPPER_JOG_LOCK = threading.Lock()
NETWORK_STATE_LOCK = threading.Lock()
TRIP_STATE_LOCK = threading.Lock()
BATTERY_DB_LOCK = threading.Lock()
CONFIG_LOCK = threading.Lock()
STARTED_AT = time.monotonic()
SERVER_STATE = {
    "connecting": False,
    "last_action": "server starting",
    "last_error": "",
}
IMU_STATE = {"reading": None, "error": ""}
ODESC_STATE = {
    "last_refresh_monotonic": None,
    "last_power_monotonic": None,
    "energy_used_wh": 0.0,
    "error": "",
}
ODESC_MOTION_STATE = {
    "active_axis": None,
    "mode": None,
    "velocity_turns_s": 0.0,
    "startup_started_monotonic": 0.0,
    "startup_duration_s": 0.0,
    "sensorless_min_turns_s": 0.0,
    "sensorless_command_sent": False,
    "last_keepalive_monotonic": 0.0,
    "deadman_stops": 0,
    "last_error": "",
}
STEPPER_JOG_STATE = {
    "axis": None,
    "direction": 0,
    "chunk_steps": 0,
    "rpm": 0.0,
    "last_keepalive_monotonic": 0.0,
    "last_error": "",
}
NETWORK_STATE = {
    "status": {
        "mode": "unknown",
        "connected": False,
        "connection": "",
        "signal_percent": None,
        "ipv4": "",
        "ap_ssid": "robot",
        "ap_url": "http://10.42.0.1:8080/",
    },
    "scheduled_mode": None,
    "switch_at_monotonic": None,
    "last_error": "",
    "weak_since_monotonic": None,
    "router_grace_until": 0.0,
}
TRIP_STATE = {
    "active": False,
    "trip_id": 0,
    "started_wall_time": None,
    "started_monotonic": None,
    "stopped_wall_time": None,
    "heading_offset_deg": 0.0,
    "last_update_monotonic": None,
    "last_sample_monotonic": None,
    "east_m": 0.0,
    "north_m": 0.0,
    "velocity_east_m_s": 0.0,
    "velocity_north_m_s": 0.0,
    "path_distance_m": 0.0,
    "samples": [],
}
BATTERY_SESSION_ID = None
CONFIG_DATA = {}
TANK_STATE_LOCK = threading.Lock()
TANK_STATE = {
    "target_left": 0.0,
    "target_right": 0.0,
    "current_left": 0.0,
    "current_right": 0.0,
    "revision": 0,
    "ramp_percent_s": DEFAULT_TANK_RAMP_PERCENT_S,
    "last_target_monotonic": 0.0,
    "source": "idle",
    "deadman_stops": 0,
    "last_error": "",
}


def _load_config():
    global CONFIG_DATA
    try:
        config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
        if not isinstance(config, dict):
            raise ValueError("configuration root must be an object")
        CONFIG_DATA = config
        ramp = float(config.get("tank_ramp_percent_s", DEFAULT_TANK_RAMP_PERCENT_S))
        if 10.0 <= ramp <= 300.0:
            TANK_STATE["ramp_percent_s"] = ramp
        heading_offset = float(config.get("imu_heading_offset_deg", 0.0))
        if -180.0 <= heading_offset <= 180.0:
            TRIP_STATE["heading_offset_deg"] = heading_offset
    except FileNotFoundError:
        pass
    except Exception as exc:
        SERVER_STATE["last_error"] = "config load: {}".format(exc)


_load_config()


def _save_config_value(name, value):
    with CONFIG_LOCK:
        CONFIG_DATA[name] = value
        temporary = CONFIG_PATH.with_suffix(CONFIG_PATH.suffix + ".tmp")
        temporary.write_text(
            json.dumps(CONFIG_DATA, sort_keys=True, indent=2) + "\n",
            encoding="utf-8",
        )
        os.replace(str(temporary), str(CONFIG_PATH))


def _pi_uptime_s():
    try:
        return float(Path("/proc/uptime").read_text(encoding="ascii").split()[0])
    except Exception:
        return time.monotonic()


def _pi_temperature_c():
    paths = (
        Path("/sys/class/thermal/thermal_zone0/temp"),
        Path("/sys/devices/virtual/thermal/thermal_zone0/temp"),
    )
    for path in paths:
        try:
            value = float(path.read_text(encoding="ascii").strip())
            return value / 1000.0 if value > 200.0 else value
        except Exception:
            continue
    return None


def _boot_id():
    try:
        return Path("/proc/sys/kernel/random/boot_id").read_text(
            encoding="ascii"
        ).strip()
    except Exception:
        return "unknown-{}".format(round(time.time()))


def _battery_soc_percent(voltage):
    """Approximate rested 10S lithium-ion SOC from pack voltage."""
    per_cell = float(voltage) / BATTERY_CELL_COUNT
    curve = (
        (3.00, 0.0),
        (3.30, 3.0),
        (3.50, 10.0),
        (3.60, 20.0),
        (3.70, 35.0),
        (3.80, 55.0),
        (3.90, 70.0),
        (4.00, 82.0),
        (4.10, 92.0),
        (4.20, 100.0),
    )
    if per_cell <= curve[0][0]:
        return 0.0
    if per_cell >= curve[-1][0]:
        return 100.0
    for (v0, s0), (v1, s1) in zip(curve, curve[1:]):
        if per_cell <= v1:
            ratio = (per_cell - v0) / (v1 - v0)
            return s0 + ratio * (s1 - s0)
    return 0.0


def _battery_connect():
    connection = sqlite3.connect(str(BATTERY_HISTORY_PATH), timeout=10.0)
    connection.execute("PRAGMA journal_mode=WAL")
    connection.execute("PRAGMA synchronous=FULL")
    connection.execute("PRAGMA foreign_keys=ON")
    return connection


def _battery_init():
    global BATTERY_SESSION_ID
    BATTERY_HISTORY_PATH.parent.mkdir(parents=True, exist_ok=True)
    boot_id = _boot_id()
    uptime = _pi_uptime_s()
    boot_wall_time = time.time() - uptime
    with BATTERY_DB_LOCK:
        connection = _battery_connect()
        try:
            connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS boot_sessions (
                    id INTEGER PRIMARY KEY,
                    boot_id TEXT NOT NULL UNIQUE,
                    boot_wall_time REAL NOT NULL,
                    first_service_wall_time REAL NOT NULL,
                    last_sample_wall_time REAL
                );
                CREATE TABLE IF NOT EXISTS battery_samples (
                    id INTEGER PRIMARY KEY,
                    session_id INTEGER NOT NULL,
                    wall_time REAL NOT NULL,
                    uptime_s REAL NOT NULL,
                    voltage_v REAL NOT NULL,
                    bus_current_a REAL,
                    bus_power_w REAL,
                    energy_used_wh REAL,
                    FOREIGN KEY(session_id) REFERENCES boot_sessions(id)
                );
                CREATE INDEX IF NOT EXISTS battery_samples_session_time
                ON battery_samples(session_id, wall_time);
                CREATE TABLE IF NOT EXISTS odesc_events (
                    id INTEGER PRIMARY KEY,
                    wall_time REAL NOT NULL,
                    completed_wall_time REAL,
                    action TEXT NOT NULL,
                    request_json TEXT NOT NULL,
                    outcome TEXT NOT NULL,
                    result_json TEXT,
                    error TEXT,
                    telemetry_json TEXT
                );
                CREATE INDEX IF NOT EXISTS odesc_events_wall_time
                ON odesc_events(wall_time DESC);
                """
            )
            connection.execute(
                """
                INSERT OR IGNORE INTO boot_sessions
                (boot_id, boot_wall_time, first_service_wall_time)
                VALUES (?, ?, ?)
                """,
                (boot_id, boot_wall_time, time.time()),
            )
            row = connection.execute(
                "SELECT id FROM boot_sessions WHERE boot_id = ?", (boot_id,)
            ).fetchone()
            BATTERY_SESSION_ID = int(row[0])
            connection.commit()
        finally:
            connection.close()


def _json_text(value):
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        default=str,
    )


def _odesc_event_start(action, request=None):
    wall_time = time.time()
    with BATTERY_DB_LOCK:
        connection = _battery_connect()
        try:
            cursor = connection.execute(
                """
                INSERT INTO odesc_events
                (wall_time, action, request_json, outcome)
                VALUES (?, ?, ?, 'requested')
                """,
                (wall_time, str(action), _json_text(request or {})),
            )
            event_id = int(cursor.lastrowid)
            connection.commit()
        finally:
            connection.close()
    print(
        "[odesc-event] id={} requested {} {}".format(
            event_id, action, _json_text(request or {})
        )
    )
    return event_id


def _odesc_cached_telemetry():
    try:
        links = ROBOT.link_status()
        return links.get("odesc", {})
    except Exception as exc:
        return {"snapshot_error": str(exc)}


def _odesc_event_finish(
    event_id,
    outcome,
    result=None,
    error="",
    telemetry=None,
):
    completed = time.time()
    telemetry = (
        _odesc_cached_telemetry() if telemetry is None else telemetry
    )
    with BATTERY_DB_LOCK:
        connection = _battery_connect()
        try:
            connection.execute(
                """
                UPDATE odesc_events
                SET completed_wall_time = ?, outcome = ?, result_json = ?,
                    error = ?, telemetry_json = ?
                WHERE id = ?
                """,
                (
                    completed,
                    str(outcome),
                    _json_text(result),
                    str(error),
                    _json_text(telemetry),
                    int(event_id),
                ),
            )
            connection.commit()
        finally:
            connection.close()
    print(
        "[odesc-event] id={} {}{}"
        .format(
            event_id,
            outcome,
            "" if not error else " error=" + str(error),
        )
    )


def _odesc_events(limit=ODESC_EVENT_LIMIT):
    limit = max(1, min(500, int(limit)))
    with BATTERY_DB_LOCK:
        connection = _battery_connect()
        try:
            rows = connection.execute(
                """
                SELECT id, wall_time, completed_wall_time, action,
                       request_json, outcome, result_json, error,
                       telemetry_json
                FROM odesc_events
                ORDER BY id DESC
                LIMIT ?
                """,
                (limit,),
            ).fetchall()
        finally:
            connection.close()

    events = []
    for row in rows:
        event = {
            "id": row[0],
            "wall_time": row[1],
            "completed_wall_time": row[2],
            "action": row[3],
            "outcome": row[5],
            "error": row[7] or "",
        }
        for key, value in (
            ("request", row[4]),
            ("result", row[6]),
            ("telemetry", row[8]),
        ):
            try:
                event[key] = json.loads(value) if value else None
            except (TypeError, json.JSONDecodeError):
                event[key] = value
        events.append(event)
    return {"ok": True, "events": events}


def _battery_record(snapshot):
    if BATTERY_SESSION_ID is None or "vbus_voltage" not in snapshot:
        return
    wall_time = time.time()
    with ODESC_STATE_LOCK:
        energy = ODESC_STATE["energy_used_wh"]
    values = (
        BATTERY_SESSION_ID,
        wall_time,
        _pi_uptime_s(),
        float(snapshot["vbus_voltage"]),
        float(snapshot.get("ibus_a", 0.0)),
        float(snapshot.get("bus_power_w", 0.0)),
        float(energy),
    )
    with BATTERY_DB_LOCK:
        connection = _battery_connect()
        try:
            connection.execute(
                """
                INSERT INTO battery_samples
                (session_id, wall_time, uptime_s, voltage_v, bus_current_a,
                 bus_power_w, energy_used_wh)
                VALUES (?, ?, ?, ?, ?, ?, ?)
                """,
                values,
            )
            connection.execute(
                """
                UPDATE boot_sessions SET last_sample_wall_time = ?
                WHERE id = ?
                """,
                (wall_time, BATTERY_SESSION_ID),
            )
            connection.commit()
        finally:
            connection.close()


def _downsample_rows(rows, maximum=4000):
    if len(rows) <= maximum:
        return rows
    stride = math.ceil(len(rows) / maximum)
    sampled = rows[::stride]
    if sampled[-1] != rows[-1]:
        sampled.append(rows[-1])
    return sampled


def _battery_history(session_id=None):
    with BATTERY_DB_LOCK:
        connection = _battery_connect()
        try:
            session_rows = connection.execute(
                """
                SELECT s.id, s.boot_id, s.boot_wall_time,
                       s.first_service_wall_time, s.last_sample_wall_time,
                       COUNT(b.id), MIN(b.voltage_v), MAX(b.voltage_v)
                FROM boot_sessions s
                LEFT JOIN battery_samples b ON b.session_id = s.id
                GROUP BY s.id
                ORDER BY s.boot_wall_time DESC
                """
            ).fetchall()
            if session_id is None:
                session_id = BATTERY_SESSION_ID
            sample_rows = connection.execute(
                """
                SELECT wall_time, uptime_s, voltage_v, bus_current_a,
                       bus_power_w, energy_used_wh
                FROM battery_samples
                WHERE session_id = ?
                ORDER BY wall_time
                """,
                (int(session_id),),
            ).fetchall()
        finally:
            connection.close()

    samples = []
    sampled_rows = _downsample_rows(sample_rows)
    for index, row in enumerate(sampled_rows):
        wall_time, uptime, voltage, current, power, energy = row
        soc = _battery_soc_percent(voltage)
        estimated_current = None
        for previous in reversed(samples):
            elapsed = wall_time - previous["wall_time"]
            if elapsed >= 300.0:
                soc_drop = previous["soc_percent"] - soc
                estimated_current = (
                    soc_drop
                    * BATTERY_PACK_CAPACITY_AH
                    / 100.0
                    / (elapsed / 3600.0)
                )
                if not -20.0 <= estimated_current <= 100.0:
                    estimated_current = None
                break
        samples.append(
            {
                "wall_time": wall_time,
                "uptime_s": uptime,
                "voltage_v": voltage,
                "bus_current_a": current,
                "bus_power_w": power,
                "energy_used_wh": energy,
                "soc_percent": soc,
                "estimated_pack_current_a": estimated_current,
            }
        )

    sessions = [
        {
            "id": row[0],
            "boot_id": row[1],
            "boot_wall_time": row[2],
            "first_service_wall_time": row[3],
            "last_sample_wall_time": row[4],
            "sample_count": row[5],
            "minimum_voltage_v": row[6],
            "maximum_voltage_v": row[7],
            "current": row[0] == BATTERY_SESSION_ID,
        }
        for row in session_rows
    ]
    return {
        "ok": True,
        "selected_session_id": int(session_id),
        "sessions": sessions,
        "samples": samples,
        "pack": {
            "series_cells": BATTERY_CELL_COUNT,
            "parallel_cells": BATTERY_PARALLEL_COUNT,
            "cell_capacity_ah": BATTERY_CELL_CAPACITY_AH,
            "pack_capacity_ah": BATTERY_PACK_CAPACITY_AH,
            "full_voltage_v": 4.2 * BATTERY_CELL_COUNT,
            "nominal_voltage_v": 3.7 * BATTERY_CELL_COUNT,
            "minimum_voltage_v": 3.0 * BATTERY_CELL_COUNT,
        },
        "pi_uptime_s": _pi_uptime_s(),
    }


def _corrected_heading(raw_heading):
    with TRIP_STATE_LOCK:
        offset = TRIP_STATE["heading_offset_deg"]
    return (float(raw_heading) + offset) % 360.0


def _trip_update(reading):
    now = time.monotonic()
    raw_heading = float(reading["heading_deg"])
    linear = reading.get("linear_acceleration_m_s2", (0.0, 0.0, 0.0))
    with TRIP_STATE_LOCK:
        if not TRIP_STATE["active"]:
            return
        previous_time = TRIP_STATE["last_update_monotonic"]
        TRIP_STATE["last_update_monotonic"] = now
        if previous_time is None:
            return
        elapsed = now - previous_time
        if elapsed <= 0.0 or elapsed > 1.0:
            return

        heading = (
            raw_heading + TRIP_STATE["heading_offset_deg"]
        ) % 360.0
        heading_rad = math.radians(heading)
        forward_accel = float(linear[0])
        right_accel = float(linear[1])
        if abs(forward_accel) < TRIP_ACCEL_DEADBAND_M_S2:
            forward_accel = 0.0
        if abs(right_accel) < TRIP_ACCEL_DEADBAND_M_S2:
            right_accel = 0.0
        accel_east = (
            forward_accel * math.sin(heading_rad)
            + right_accel * math.cos(heading_rad)
        )
        accel_north = (
            forward_accel * math.cos(heading_rad)
            - right_accel * math.sin(heading_rad)
        )

        old_ve = TRIP_STATE["velocity_east_m_s"]
        old_vn = TRIP_STATE["velocity_north_m_s"]
        damping = math.exp(-TRIP_VELOCITY_DAMPING_PER_S * elapsed)
        new_ve = old_ve * damping + accel_east * elapsed
        new_vn = old_vn * damping + accel_north * elapsed
        delta_east = (old_ve + new_ve) * 0.5 * elapsed
        delta_north = (old_vn + new_vn) * 0.5 * elapsed
        TRIP_STATE["velocity_east_m_s"] = new_ve
        TRIP_STATE["velocity_north_m_s"] = new_vn
        TRIP_STATE["east_m"] += delta_east
        TRIP_STATE["north_m"] += delta_north
        TRIP_STATE["path_distance_m"] += math.hypot(
            delta_east, delta_north
        )

        last_sample = TRIP_STATE["last_sample_monotonic"]
        if last_sample is None or now - last_sample >= TRIP_SAMPLE_INTERVAL_S:
            elapsed_trip = now - TRIP_STATE["started_monotonic"]
            TRIP_STATE["samples"].append(
                {
                    "t": round(elapsed_trip, 3),
                    "raw_heading": round(raw_heading, 2),
                    "heading": round(heading, 2),
                    "east": round(TRIP_STATE["east_m"], 4),
                    "north": round(TRIP_STATE["north_m"], 4),
                    "speed": round(math.hypot(new_ve, new_vn), 4),
                    "accel_forward": round(forward_accel, 4),
                    "accel_right": round(right_accel, 4),
                }
            )
            if len(TRIP_STATE["samples"]) > TRIP_MAX_SAMPLES:
                TRIP_STATE["samples"] = TRIP_STATE["samples"][::2]
            TRIP_STATE["last_sample_monotonic"] = now


def _trip_start():
    now = time.monotonic()
    with TRIP_STATE_LOCK:
        TRIP_STATE["active"] = True
        TRIP_STATE["trip_id"] += 1
        TRIP_STATE["started_wall_time"] = time.time()
        TRIP_STATE["started_monotonic"] = now
        TRIP_STATE["stopped_wall_time"] = None
        TRIP_STATE["last_update_monotonic"] = now
        TRIP_STATE["last_sample_monotonic"] = None
        TRIP_STATE["east_m"] = 0.0
        TRIP_STATE["north_m"] = 0.0
        TRIP_STATE["velocity_east_m_s"] = 0.0
        TRIP_STATE["velocity_north_m_s"] = 0.0
        TRIP_STATE["path_distance_m"] = 0.0
        TRIP_STATE["samples"] = []
    return _trip_snapshot()


def _trip_stop():
    with TRIP_STATE_LOCK:
        TRIP_STATE["active"] = False
        TRIP_STATE["stopped_wall_time"] = time.time()
        TRIP_STATE["last_update_monotonic"] = None
    return _trip_snapshot()


def _trip_set_heading_offset(offset):
    offset = float(offset)
    if not -180.0 <= offset <= 180.0:
        raise ValueError("heading offset must be from -180 through 180 degrees")
    with TRIP_STATE_LOCK:
        if TRIP_STATE["active"]:
            raise RuntimeError("stop the active trip before changing heading offset")
        TRIP_STATE["heading_offset_deg"] = offset
    _save_config_value("imu_heading_offset_deg", offset)
    return _trip_snapshot()


def _trip_snapshot(include_samples=False):
    with TRIP_STATE_LOCK:
        start = TRIP_STATE["started_monotonic"]
        elapsed = None if start is None else time.monotonic() - start
        result = {
            "active": TRIP_STATE["active"],
            "trip_id": TRIP_STATE["trip_id"],
            "started_wall_time": TRIP_STATE["started_wall_time"],
            "stopped_wall_time": TRIP_STATE["stopped_wall_time"],
            "elapsed_s": elapsed,
            "heading_offset_deg": TRIP_STATE["heading_offset_deg"],
            "east_m": TRIP_STATE["east_m"],
            "north_m": TRIP_STATE["north_m"],
            "velocity_east_m_s": TRIP_STATE["velocity_east_m_s"],
            "velocity_north_m_s": TRIP_STATE["velocity_north_m_s"],
            "speed_m_s": math.hypot(
                TRIP_STATE["velocity_east_m_s"],
                TRIP_STATE["velocity_north_m_s"],
            ),
            "displacement_m": math.hypot(
                TRIP_STATE["east_m"], TRIP_STATE["north_m"]
            ),
            "path_distance_m": TRIP_STATE["path_distance_m"],
            "sample_count": len(TRIP_STATE["samples"]),
        }
        if include_samples:
            result["samples"] = _downsample_rows(
                list(TRIP_STATE["samples"]), maximum=1800
            )
    return result


def _sensors_payload():
    with IMU_STATE_LOCK:
        reading = (
            None
            if IMU_STATE["reading"] is None
            else dict(IMU_STATE["reading"])
        )
        error = IMU_STATE["error"]
    if reading is not None:
        reading["corrected_heading_deg"] = _corrected_heading(
            reading["heading_deg"]
        )
    return {
        "ok": True,
        "reading": reading,
        "error": error,
        "trip": _trip_snapshot(include_samples=True),
    }


INDEX_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<meta name="theme-color" content="#182129">
<title>Garden Rover Control</title>
<style>
*{box-sizing:border-box;letter-spacing:0}
:root{--ink:#182129;--muted:#60707d;--line:#ccd5dc;--page:#edf1f4;--panel:#fff;--blue:#1268a8;--green:#26734d;--amber:#a96800;--red:#bd241c}
html,body{width:100%;max-width:100%;overflow-x:hidden}body{margin:0;background:var(--page);color:var(--ink);font:15px Arial,sans-serif}
header{position:sticky;top:0;z-index:10;background:#182129;color:#fff;border-bottom:3px solid #d99a16}
.head{max-width:1120px;margin:auto;padding:10px 14px;display:grid;grid-template-columns:1fr auto;gap:12px;align-items:center}
h1{font-size:19px;margin:0 0 5px}.summary{font-size:12px;color:#cbd5dc}
button,input,select{font:inherit;min-width:0;max-width:100%;min-height:42px;border:1px solid #aebac3;border-radius:6px}
button{padding:8px 12px;background:#e7edf1;color:var(--ink);font-weight:700;cursor:pointer}
button:hover{filter:brightness(.96)}button:disabled{opacity:.55;cursor:wait}
.danger{background:var(--red);border-color:var(--red);color:#fff}.primary{background:var(--blue);border-color:var(--blue);color:#fff}
.safe{background:var(--green);border-color:var(--green);color:#fff}.warn{background:#fff0c9;border-color:#d19a21}
.estop{min-width:128px;min-height:54px;font-size:16px}
main{max-width:1120px;margin:auto;padding:12px 12px 40px}
.device{min-width:0;max-width:100%;background:var(--panel);border:1px solid var(--line);border-radius:8px;margin-bottom:12px;overflow:hidden}
.device-head{padding:10px 12px;border-bottom:1px solid var(--line);display:flex;justify-content:space-between;align-items:center;gap:10px}
.device-head h2{font-size:17px;margin:0}.device-body{padding:12px}
.badge{font-size:12px;padding:5px 8px;border-radius:5px;background:#e6ebef;font-weight:700}
.up{background:#d7efdf;color:#175b38}.down{background:#f8d9d7;color:#8e1d18}.armed{background:#fff0c9;color:#7b4900}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:10px}.grid3{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.grid4{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}
.grid2>*,.grid3>*,.grid4>*,.head>*{min-width:0}
.motor{min-width:0;max-width:100%;border:1px solid var(--line);border-radius:7px;padding:10px}.motor h3{font-size:15px;margin:0 0 9px}
.axis-grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px}
.jog-grid{display:grid;grid-template-columns:1fr 1fr;gap:7px;margin-top:8px}.jog{background:#dce8f1;border-color:#8ba7bb;touch-action:none}
label{display:grid;gap:4px;color:var(--muted);font-size:12px}input,select{width:100%;padding:8px;background:#fff;color:var(--ink)}
.actions{display:flex;min-width:0;max-width:100%;flex-wrap:wrap;gap:7px;margin-top:9px}.actions button{flex:1;min-width:105px}
.metric{border-left:4px solid #16816a;background:#f6f8f9;padding:8px;border-radius:4px;min-height:62px}
.metric span{display:block;font-size:11px;color:var(--muted)}.metric strong{display:block;font-size:18px;margin-top:4px;overflow-wrap:anywhere}
.signal-row{display:flex;align-items:flex-end;gap:10px;margin-top:5px}.signal-row strong{margin:0}
.signal-bars{height:27px;display:flex;align-items:flex-end;gap:3px;color:#8b98a1}.signal-bars i{display:block;width:6px;background:#d6dde2;border-radius:2px 2px 0 0}.signal-bars i:nth-child(1){height:7px}.signal-bars i:nth-child(2){height:12px}.signal-bars i:nth-child(3){height:17px}.signal-bars i:nth-child(4){height:22px}.signal-bars i:nth-child(5){height:27px}
.signal-bars[data-level="1"] i:nth-child(1),.signal-bars[data-level="2"] i:nth-child(-n+2),.signal-bars[data-level="3"] i:nth-child(-n+3),.signal-bars[data-level="4"] i:nth-child(-n+4),.signal-bars[data-level="5"] i:nth-child(-n+5){background:currentColor}
.signal-bars.weak{color:#bd241c}.signal-bars.fair{color:#a96800}.signal-bars.good{color:#26734d}.signal-quality{margin-top:3px}
.subhead{font-size:13px;margin:13px 0 7px}.note{font-size:12px;color:var(--muted);line-height:1.45;margin:8px 0 0}
details{border-top:1px solid var(--line);margin-top:12px;padding-top:10px}summary{cursor:pointer;font-weight:700}
#events,#odEventLog{margin:0;max-height:260px;overflow:auto;white-space:pre-wrap;word-break:break-word;background:#101820;color:#d6f3e2;padding:10px;border-radius:6px;font:12px monospace}
@media(max-width:720px){.grid2,.grid3,.grid4{grid-template-columns:1fr 1fr}.axis-grid{grid-template-columns:1fr}.head{grid-template-columns:1fr 112px}.estop{min-width:112px}.motor-grid{grid-template-columns:1fr}}
@media(max-width:430px){main{padding:8px 8px 30px}.device-body{padding:9px}.grid3,.grid4{grid-template-columns:1fr 1fr}.actions button{min-width:90px}.head{padding:8px}.summary{font-size:11px}}
</style>
</head>
<body>
<header><div class="head"><div><h1>Autonomous Garden Rover</h1><div id="summary" class="summary">Connecting UART devices...</div></div><button class="danger estop" onclick="stopAll()">STOP ALL</button></div></header>
<main>
<section class="device">
 <div class="device-head"><h2>STM32 Dual BTS7960</h2><span id="stmBadge" class="badge down">offline</span></div>
 <div class="device-body">
  <div class="grid2 motor-grid">
   <div class="motor"><h3>Motor 1</h3><div class="grid2"><label>Direction<select id="m1dir"><option>A</option><option>B</option></select></label><label>Output %<input id="m1pct" type="number" min="0" max="100" value="6"></label></div><div class="actions"><button class="primary" onclick="drive(1)">Drive</button><button class="warn" onclick="pulse(1)">0.5 s pulse</button><button class="danger" onclick="motorStop(1)">Stop</button></div></div>
   <div class="motor"><h3>Motor 2</h3><div class="grid2"><label>Direction<select id="m2dir"><option>A</option><option>B</option></select></label><label>Output %<input id="m2pct" type="number" min="0" max="100" value="6"></label></div><div class="actions"><button class="primary" onclick="drive(2)">Drive</button><button class="warn" onclick="pulse(2)">0.5 s pulse</button><button class="danger" onclick="motorStop(2)">Stop</button></div></div>
  </div>
  <h3 class="subhead">Current sense</h3>
  <div class="grid4"><div class="metric"><span>M1 R_IS</span><strong id="m1r">--</strong></div><div class="metric"><span>M1 L_IS</span><strong id="m1l">--</strong></div><div class="metric"><span>M2 R_IS</span><strong id="m2r">--</strong></div><div class="metric"><span>M2 L_IS</span><strong id="m2l">--</strong></div></div>
  <div class="actions"><button onclick="act('/api/stm32/currents')">Refresh currents</button></div>
  <h3 class="subhead">Analog encoders PA0-PA3</h3>
  <div class="grid4"><div class="metric"><span>PA0</span><strong id="pa0">--</strong></div><div class="metric"><span>PA1</span><strong id="pa1">--</strong></div><div class="metric"><span>PA2</span><strong id="pa2">--</strong></div><div class="metric"><span>PA3</span><strong id="pa3">--</strong></div></div>
  <div class="actions"><label>Rate<select id="encRate"><option>10</option><option>20</option><option>25</option><option selected>50</option><option>100</option></select></label><button class="safe" onclick="encoderStart()">Start stream</button><button onclick="act('/api/stm32/encoders/read')">Read once</button><button onclick="act('/api/stm32/encoders/stop')">Stop stream</button></div>
  <p id="encInfo" class="note">stream off</p>
 </div>
</section>

<section class="device">
 <div class="device-head"><h2>GD32 Stepper Controller</h2><span id="gdBadge" class="badge down">offline</span></div>
 <div class="device-body">
	  <div class="axis-grid">
	   <div class="motor"><h3>X axis</h3><div class="grid2"><div class="metric"><span>Position</span><strong id="xPosition">unknown</strong></div><div class="metric"><span>Allowed range</span><strong id="xRange">--</strong></div></div><div class="grid2"><label>Signed steps<input id="xSteps" type="number" value="200"></label><label>RPM<input id="xRpm" type="number" min="1" max="1000" value="30"></label><label>Minimum<input id="xMin" type="number" value="-10000"></label><label>Maximum<input id="xMax" type="number" value="10000"></label><label>Jog chunk<input id="xChunk" type="number" min="1" max="10000" value="50"></label></div><div class="actions"><button class="primary" onclick="stepMove('X')">Move X</button><button onclick="stepperZero('X')">Reset zero</button><button onclick="stepperLimits('X')">Set limits</button></div><div class="jog-grid"><button id="xJogNeg" class="jog">Hold -</button><button id="xJogPos" class="jog">Hold +</button></div></div>
	   <div class="motor"><h3>Y axis</h3><div class="grid2"><div class="metric"><span>Position</span><strong id="yPosition">unknown</strong></div><div class="metric"><span>Allowed range</span><strong id="yRange">--</strong></div></div><div class="grid2"><label>Signed steps<input id="ySteps" type="number" value="200"></label><label>RPM<input id="yRpm" type="number" min="1" max="1000" value="30"></label><label>Minimum<input id="yMin" type="number" value="-10000"></label><label>Maximum<input id="yMax" type="number" value="10000"></label><label>Jog chunk<input id="yChunk" type="number" min="1" max="10000" value="50"></label></div><div class="actions"><button class="primary" onclick="stepMove('Y')">Move Y</button><button onclick="stepperZero('Y')">Reset zero</button><button onclick="stepperLimits('Y')">Set limits</button></div><div class="jog-grid"><button id="yJogNeg" class="jog">Hold -</button><button id="yJogPos" class="jog">Hold +</button></div></div>
	   <div class="motor"><h3>Z axis</h3><div class="grid2"><div class="metric"><span>Position</span><strong id="zPosition">unknown</strong></div><div class="metric"><span>Allowed range</span><strong id="zRange">--</strong></div></div><div class="grid2"><label>Signed steps<input id="zSteps" type="number" value="200"></label><label>RPM<input id="zRpm" type="number" min="1" max="1000" value="30"></label><label>Minimum<input id="zMin" type="number" value="-10000"></label><label>Maximum<input id="zMax" type="number" value="10000"></label><label>Jog chunk<input id="zChunk" type="number" min="1" max="10000" value="50"></label></div><div class="actions"><button class="primary" onclick="stepMove('Z')">Move Z</button><button onclick="stepperZero('Z')">Reset zero</button><button onclick="stepperLimits('Z')">Set limits</button></div><div class="jog-grid"><button id="zJogNeg" class="jog">Hold -</button><button id="zJogPos" class="jog">Hold +</button></div></div>
  </div>
  <div class="actions"><button class="danger" onclick="act('/api/gd32/stop')">Stop all steppers</button><button onclick="act('/api/gd32/switches')">Read switches</button></div>
  <p id="gdInfo" class="note">X/Y/Z moves can be queued, but this firmware executes separate moves sequentially.</p>
 </div>
</section>

<section class="device">
 <div class="device-head"><h2>BNO080 / BNO085 IMU</h2><span id="imuBadge" class="badge down">offline</span></div>
 <div class="device-body"><div class="grid4"><div class="metric"><span>Heading</span><strong id="heading">--</strong></div><div class="metric"><span>Mag accuracy</span><strong id="magacc">--</strong></div><div class="metric"><span>Magnetic vector</span><strong id="magvec">--</strong></div><div class="metric"><span>Packet age</span><strong id="imuage">--</strong></div></div></div>
</section>

<section class="device">
 <div class="device-head"><h2>ODESC / ODrive Clone</h2><span id="odBadge" class="badge down">offline</span></div>
 <div class="device-body">
	 <div class="grid4"><div class="metric"><span>Battery bus voltage</span><strong id="busVoltage">--</strong></div><div class="metric"><span>DC bus current</span><strong id="busCurrent">--</strong></div><div class="metric"><span>DC bus power</span><strong id="busPower">--</strong></div><div class="metric"><span>Session energy used</span><strong id="busEnergy">--</strong></div><div class="metric"><span>Sample age</span><strong id="busVoltageAge">--</strong></div></div>
	 <p class="note">Signed bus power is voltage times ODESC ibus; negative values indicate regeneration. Charge percentage requires battery chemistry and series cell count.</p>
	 <div class="actions"><button onclick="act('/api/odesc/status')">Refresh status</button><button onclick="act('/api/odesc/energy-reset')">Reset energy counter</button><button class="danger" onclick="act('/api/odesc/stop')">Stop ODESC</button></div>
	 <details open>
	  <summary>Legacy sensorless control</summary>
	  <p class="note">This ODESC firmware uses sensorless axis state 5. Sensorless startup immediately spins the motor through an open-loop ramp. It cannot hold zero speed or reverse while running; STOP returns the axis to IDLE. Raise the mechanism and keep a physical power disconnect ready.</p>
	  <div class="grid4">
	   <label>Axis<select id="odAxis" onchange="odescAxisChanged()"><option>0</option><option>1</option></select></label>
	   <label>Direction<select id="odSensorlessDirection"><option value="1">Positive</option><option value="-1">Negative</option></select></label>
	   <label>Running speed turns/s<input id="odSensorlessSpeed" type="number" min="1" max="15" step="0.1" value="5"></label>
	   <label>Current limit A<input id="odCurrentLimit" type="number" min="0.5" max="30" step="0.5" value="10"></label>
	   <label>Startup current A<input id="odStartupCurrent" type="number" min="0.5" max="30" step="0.25" value="4"></label>
	   <label>Startup speed turns/s<input id="odStartupSpeed" type="number" min="1" max="30" step="0.1" value="5"></label>
	   <label>Startup acceleration turns/s²<input id="odStartupAccel" type="number" min="0.1" max="30" step="0.1" value="1.8"></label>
	   <label>Velocity limit turns/s<input id="odVelocityLimit" type="number" min="1" max="50" step="0.5" value="15"></label>
	  </div>
	  <div class="grid4">
	   <div class="metric"><span>Axis state</span><strong id="odState">--</strong></div>
	   <div class="metric"><span>Sensorless velocity</span><strong id="odSensorlessVelocity">--</strong></div>
	   <div class="metric"><span>Encoder velocity</span><strong id="odVelocity">--</strong></div>
	   <div class="metric"><span>Motor Iq</span><strong id="odIq">--</strong></div>
	   <div class="metric"><span>Errors axis / motor / estimator / controller</span><strong id="odErrors">--</strong></div>
	   <div class="metric"><span>Motor calibration</span><strong id="odCalibration">--</strong></div>
	   <div class="metric"><span>Pole pairs / flux linkage</span><strong id="odMotorModel">--</strong></div>
	   <div class="metric"><span>Configured startup ramp</span><strong id="odRamp">--</strong></div>
	  </div>
	  <div class="grid2">
	   <label>Arm phrase<input id="odPhrase" placeholder="ARM ODESC TEST"></label>
	   <label>Motor-calibration phrase<input id="odMotorCalPhrase" placeholder="CALIBRATE ODESC MOTOR"></label>
	  </div>
	  <div class="actions">
	   <button onclick="odescSensorlessConfigure()">Apply sensorless config</button>
	   <button onclick="odescClearErrors()">Clear errors</button>
	   <button class="warn" onclick="odescMotorCalibrate()">Calibrate motor</button>
	   <button class="warn" onclick="odescArm()">Arm</button>
	   <button class="primary" onclick="odescSensorlessStart()">Start sensorless</button>
	   <button class="danger" onclick="odescSensorlessStop()">STOP sensorless</button>
	  </div>
	  <p id="odMotionInfo" class="note">Sensorless control stopped.</p>
	 </details>
	 <details>
	  <summary>Encoder closed-loop commissioning</summary>
	  <p class="note">Use this only after an encoder is connected and calibrated. It requests state 8 and is not the sensorless path.</p>
	  <div class="grid2"><label>Encoder speed turns/s<input id="odSpeed" type="number" min="-15" max="15" step="0.05" value="0.25"></label><label>Full-calibration phrase<input id="odCalPhrase" placeholder="CALIBRATE ODESC AXIS"></label></div>
	  <div class="actions"><button onclick="odescConfigure()">Apply encoder velocity config</button><button class="warn" onclick="odescCalibrate()">Full calibration</button><button onclick="odescEnable()">Enable state 8</button><button class="primary" onclick="odescStart()">Start encoder velocity</button><button class="danger" onclick="odescStop()">Stop axis</button><button onclick="odescDisable()">Disable axis</button></div>
	 </details>
	 <details><summary>Persistent ODESC action and error history</summary><div class="actions"><button onclick="refreshOdescEvents()">Refresh diagnostic log</button></div><pre id="odEventLog">No ODESC actions loaded.</pre></details>
 </div>
</section>

<section class="device"><div class="device-head"><h2>System</h2><span id="serverBadge" class="badge up">online</span></div><div class="device-body">
	 <div class="grid4"><div class="metric"><span>Pi CPU temperature</span><strong id="piTemperature">--</strong></div><div class="metric"><span>Pi uptime</span><strong id="piUptime">--</strong></div><button class="primary" onclick="location.href='/mobile'">Tank joystick</button><button onclick="location.href='/sensors'">IMU sensors</button><button onclick="location.href='/battery'">Battery history</button></div>
	 <div class="grid2"><label>Tank ramp (% per second)<input id="tankRamp" type="number" min="10" max="300" value="80"></label><button onclick="saveTankRamp()">Save ramp rate</button></div>
	 <div class="actions"><button onclick="act('/api/connect-all')">Reconnect all UARTs</button><button onclick="refresh()">Refresh status</button></div>
	 <h3 class="subhead">Wi-Fi mode</h3><div class="grid3"><div class="metric"><span>Mode</span><strong id="wifiMode">--</strong></div><div class="metric"><span>Connection / address</span><strong id="wifiConnection">--</strong></div><div class="metric"><span>Signal strength</span><div class="signal-row"><div id="wifiBars" class="signal-bars" data-level="0" role="img" aria-label="Wi-Fi signal unavailable"><i></i><i></i><i></i><i></i><i></i></div><strong id="wifiSignal">--</strong></div><span id="wifiQuality" class="signal-quality">unavailable</span></div></div><div class="actions"><button class="warn" onclick="scheduleWifi('ap')">Switch to robot AP</button><button onclick="scheduleWifi('router')">Reconnect router Wi-Fi</button><button onclick="act('/api/network/cancel')">Cancel countdown</button></div><p id="wifiInfo" class="note">AP mode creates open network robot at http://10.42.0.1:8080/.</p>
 <p class="note">Do not run the Thonny UART controller while this service is active. Only one process may own each serial port.</p><pre id="events">Dashboard loaded.</pre></div></section>
</main>
<script>
const $=id=>document.getElementById(id);
function value(id){return $(id).value}
function log(message){let e=$('events');e.textContent=new Date().toLocaleTimeString()+' '+message+'\n'+e.textContent}
function badge(id,up,label,armed=false){let e=$(id);e.textContent=label;e.className='badge '+(armed?'armed':(up?'up':'down'))}
async function request(path,body){
 const options=body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body||{})};
 let response=await fetch(path,options), data=await response.json();
 if(!response.ok||data.ok===false)throw new Error(data.error||('HTTP '+response.status));
 return data;
}
async function act(path,body={}){
 try{let data=await request(path,body);log(data.result===undefined?'OK':JSON.stringify(data.result));await refresh()}
 catch(error){log('ERROR: '+error.message)}
 finally{if(path.startsWith('/api/odesc'))refreshOdescEvents()}
}
function motor(n){return {motor:n,direction:value('m'+n+'dir'),percent:Number(value('m'+n+'pct'))}}
function drive(n){if(confirm('Run motor '+n+' continuously? Use STOP when finished.'))act('/api/stm32/drive',motor(n))}
function pulse(n){act('/api/stm32/pulse',{...motor(n),seconds:0.5})}
function motorStop(n){act('/api/stm32/stop',{motor:n})}
function stopAll(){odRunning=false;odSensorlessRunning=false;jogAxis=null;act('/api/stop-all')}
function encoderStart(){act('/api/stm32/encoders/start',{rate_hz:Number(value('encRate'))})}
function saveTankRamp(){act('/api/tank/config',{ramp_percent_s:Number(value('tankRamp'))})}
function stepMove(axis){let key=axis.toLowerCase();act('/api/gd32/move',{axis:axis,steps:Number(value(key+'Steps')),rpm:Number(value(key+'Rpm'))})}
function stepperZero(axis){if(confirm('Define the current '+axis+' position as zero?'))act('/api/gd32/reset-zero',{axis:axis})}
function stepperLimits(axis){let key=axis.toLowerCase();act('/api/gd32/limits',{axis:axis,minimum_steps:Number(value(key+'Min')),maximum_steps:Number(value(key+'Max'))})}
let jogAxis=null,jogDirection=0,jogPending=false;
async function sendJog(){
 if(!jogAxis||jogPending)return;jogPending=true;let key=jogAxis.toLowerCase();
 try{await request('/api/gd32/jog',{axis:jogAxis,direction:jogDirection,chunk_steps:Number(value(key+'Chunk')),rpm:Number(value(key+'Rpm'))})}
 catch(error){log('JOG ERROR: '+error.message);jogAxis=null}
 finally{jogPending=false}
}
function startJog(axis,direction){jogAxis=axis;jogDirection=direction;sendJog()}
function stopJog(){if(!jogAxis)return;jogAxis=null;jogDirection=0;request('/api/gd32/jog-stop',{}).catch(error=>log('JOG STOP ERROR: '+error.message))}
function bindJog(axis,direction){let id=axis.toLowerCase()+'Jog'+(direction>0?'Pos':'Neg'),button=$(id);button.addEventListener('pointerdown',event=>{button.setPointerCapture(event.pointerId);startJog(axis,direction)});button.addEventListener('pointerup',stopJog);button.addEventListener('pointercancel',stopJog);button.addEventListener('lostpointercapture',stopJog)}
['X','Y','Z'].forEach(axis=>{bindJog(axis,-1);bindJog(axis,1)});
setInterval(()=>{if(jogAxis)sendJog()},200);
function odescArm(){act('/api/odesc/arm',{phrase:value('odPhrase')})}
function odescEnable(){act('/api/odesc/enable',{axis:Number(value('odAxis'))})}
function odescConfigure(){act('/api/odesc/configure',{axis:Number(value('odAxis')),current_limit_a:Number(value('odCurrentLimit')),velocity_limit_turns_s:Number(value('odVelocityLimit'))})}
function odescClearErrors(){act('/api/odesc/clear-errors',{axis:Number(value('odAxis'))})}
function odescCalibrate(){if(confirm('Calibration will move the selected ODESC motor. Continue?'))act('/api/odesc/calibrate',{axis:Number(value('odAxis')),phrase:value('odCalPhrase')})}
function odescSensorlessConfigBody(){
 return {
  axis:Number(value('odAxis')),
  current_limit_a:Number(value('odCurrentLimit')),
  startup_current_a:Number(value('odStartupCurrent')),
  startup_velocity_turns_s:Number(value('odStartupSpeed')),
  startup_accel_turns_s2:Number(value('odStartupAccel')),
  velocity_limit_turns_s:Number(value('odVelocityLimit'))
 };
}
function odescSensorlessConfigure(){act('/api/odesc/sensorless/configure',odescSensorlessConfigBody())}
function odescMotorCalibrate(){
 if(confirm('Motor-only calibration energizes and moves the selected ODESC motor. Lift the mechanism and continue?'))
  act('/api/odesc/sensorless/calibrate-motor',{axis:Number(value('odAxis')),phrase:value('odMotorCalPhrase')});
}
let odRunning=false,odPending=false;
async function pushOdescVelocity(){
 if(!odRunning||odPending)return;odPending=true;
 try{await request('/api/odesc/velocity',{axis:Number(value('odAxis')),turns_per_second:Number(value('odSpeed'))})}
 catch(error){odRunning=false;log('ODESC VELOCITY ERROR: '+error.message);refreshOdescEvents()}
 finally{odPending=false}
}
function odescStart(){if(!confirm('Start continuous ODESC velocity with browser deadman protection?'))return;odRunning=true;pushOdescVelocity()}
function odescStop(){odRunning=false;act('/api/odesc/velocity-stop',{axis:Number(value('odAxis'))})}
function odescDisable(){odRunning=false;act('/api/odesc/disable',{axis:Number(value('odAxis'))})}
let odSensorlessRunning=false,odSensorlessPending=false;
async function pushOdescSensorless(){
 if(!odSensorlessRunning||odSensorlessPending)return;
 odSensorlessPending=true;
 try{
  await request('/api/odesc/sensorless/keepalive',{
   axis:Number(value('odAxis')),
   turns_per_second:Number(value('odSensorlessSpeed'))
  });
 }catch(error){
  odSensorlessRunning=false;
  log('ODESC SENSORLESS ERROR: '+error.message);
  refreshOdescEvents();
 }finally{odSensorlessPending=false}
}
async function odescSensorlessStart(){
 if(!confirm('Sensorless startup spins immediately through an open-loop ramp. Lift the mechanism, keep a power disconnect ready, and continue?'))return;
 odRunning=false;
 try{
  let data=await request('/api/odesc/sensorless/start',{
   axis:Number(value('odAxis')),
   direction:Number(value('odSensorlessDirection')),
   turns_per_second:Number(value('odSensorlessSpeed'))
  });
  odSensorlessRunning=true;
  log('SENSORLESS START: '+JSON.stringify(data.result));
  await refresh();
 }catch(error){
  odSensorlessRunning=false;
  log('ODESC SENSORLESS START ERROR: '+error.message);
 }finally{refreshOdescEvents()}
}
function odescSensorlessStop(){
 odSensorlessRunning=false;
 act('/api/odesc/sensorless/stop',{axis:Number(value('odAxis'))});
}
function odescAxisChanged(){
 if(odSensorlessRunning)odescSensorlessStop();
 if(odRunning)odescStop();
}
setInterval(()=>{if(odRunning)pushOdescVelocity()},250);
setInterval(()=>{if(odSensorlessRunning)pushOdescSensorless()},250);
async function refreshOdescEvents(){
 try{
  let data=await request('/api/odesc/events?limit=40');
  let rows=(data.events||[]).map(event=>{
   let when=new Date(event.wall_time*1000).toLocaleString();
   let requestText=event.request&&Object.keys(event.request).length?' request='+JSON.stringify(event.request):'';
   let errorText=event.error?' error='+event.error:'';
   let telemetry=event.telemetry||{}, axes=telemetry.axes||[];
   let faults=axes.map((axis,index)=>'a'+index+'['+[axis.axis_error,axis.motor_error,axis.sensorless_estimator_error,axis.controller_error].map(v=>v??'--').join('/')+']').join(' ');
   return '#'+event.id+' '+when+' '+event.outcome.toUpperCase()+' '+event.action+requestText+errorText+(faults?' faults='+faults:'');
  });
  $('odEventLog').textContent=rows.length?rows.join('\n\n'):'No ODESC actions recorded.';
 }catch(error){$('odEventLog').textContent='Unable to load ODESC history: '+error.message}
}
function scheduleWifi(mode){if(confirm('Switch Wi-Fi mode in 3 seconds? This page will disconnect and must be reopened at the new address.'))act('/api/network/'+mode)}
function text(id,value,suffix=''){$(id).textContent=(value===undefined||value===null?'--':value)+suffix}
function duration(seconds){seconds=Math.max(0,Math.round(Number(seconds)||0));let d=Math.floor(seconds/86400),h=Math.floor(seconds%86400/3600),m=Math.floor(seconds%3600/60);return (d?d+'d ':'')+h+'h '+m+'m'}
function renderSignal(signal,mode){
 let bars=$('wifiBars'),quality='unavailable',level=0,style='';
 if(mode==='ap'){quality='field AP active'}
 else if(signal!==undefined&&signal!==null){
  signal=Math.max(0,Math.min(100,Number(signal)));level=signal>=80?5:signal>=65?4:signal>=45?3:signal>=25?2:(signal>0?1:0);
  quality=signal>=65?'good':(signal>=45?'fair':'weak');style=quality;
 }
 bars.dataset.level=level;bars.className='signal-bars '+style;bars.setAttribute('aria-label',quality+(signal===undefined||signal===null?'':' '+Math.round(signal)+' percent'));
 text('wifiSignal',signal===undefined||signal===null?null:Math.round(signal),'%');$('wifiQuality').textContent=quality;
}
function render(data){
 let l=data.links||{}, stm=l.stm32||{}, gd=l.gd32||{}, imu=l.bno080||{}, od=l.odesc||{};
 let tank=data.tank||{};if(document.activeElement!==$('tankRamp'))$('tankRamp').value=tank.ramp_percent_s||80;
 badge('stmBadge',!!stm.alive,stm.alive?'alive':'offline');badge('gdBadge',!!gd.round_trip,gd.round_trip?'linked':'offline');badge('imuBadge',!!imu.alive,imu.alive?'streaming':'offline');badge('odBadge',!!od.connected,od.armed?'armed':(od.connected?'connected':'offline'),!!od.armed);
 let motors=stm.motors||[];
 if(motors[0]){text('m1r',motors[0].r_is_mv,' mV');text('m1l',motors[0].l_is_mv,' mV')}
 if(motors[1]){text('m2r',motors[1].r_is_mv,' mV');text('m2l',motors[1].l_is_mv,' mV')}
 let enc=stm.encoder_values||[];['pa0','pa1','pa2','pa3'].forEach((id,i)=>text(id,enc[i]));
 $('encInfo').textContent='rate '+(stm.encoder_stream_hz||0)+' Hz | age '+(stm.encoder_age_ms??'--')+' ms | reports '+(stm.encoder_count||0)+' | dropped '+(stm.encoder_dropped||0);
 let positions=gd.positions||{};['X','Y','Z'].forEach(axis=>{let key=axis.toLowerCase(),p=positions[axis]||{};$(key+'Position').textContent=p.known?p.position_steps+' (target '+p.target_steps+')':'unknown - reset zero';$(key+'Range').textContent=(p.minimum_steps??'--')+' .. '+(p.maximum_steps??'--');if(p.minimum_steps!==null&&p.minimum_steps!==undefined&&document.activeElement!==$(key+'Min'))$(key+'Min').value=p.minimum_steps;if(p.maximum_steps!==null&&p.maximum_steps!==undefined&&document.activeElement!==$(key+'Max'))$(key+'Max').value=p.maximum_steps});
 let jog=data.stepper_jog||{};$('gdInfo').textContent='queued '+(gd.pending_moves||0)+' | '+(jog.axis?'jogging '+jog.axis:'jog stopped')+' | separate moves execute sequentially'+(jog.last_error?' | '+jog.last_error:'');
 let reading=imu.reading||{};text('heading',reading.heading_deg===undefined?null:Number(reading.heading_deg).toFixed(1),'°');text('magacc',reading.magnetometer_accuracy);text('magvec',reading.magnetic_uT?reading.magnetic_uT.join(', '):null);text('imuage',imu.age_ms,' ms');
 text('busVoltage',od.vbus_voltage===undefined?null:Number(od.vbus_voltage).toFixed(2),' V');text('busCurrent',od.ibus_a===undefined?null:Number(od.ibus_a).toFixed(2),' A');text('busPower',od.bus_power_w===undefined?null:Number(od.bus_power_w).toFixed(1),' W');text('busEnergy',od.energy_used_wh===undefined?null:Number(od.energy_used_wh).toFixed(3),' Wh');text('busVoltageAge',od.voltage_age_ms,' ms');
 let odAxis=Number(value('odAxis')),odAxes=od.axes||[],oa=odAxes[odAxis]||{};
 text('odState',oa.state);
 text('odSensorlessVelocity',oa.sensorless_velocity_turns_s===undefined?null:Number(oa.sensorless_velocity_turns_s).toFixed(2),' t/s');
 text('odVelocity',oa.velocity_turns_s===undefined?null:Number(oa.velocity_turns_s).toFixed(2),' t/s');
 text('odIq',oa.iq_measured_a===undefined?null:Number(oa.iq_measured_a).toFixed(2),' A');
 $('odErrors').textContent=[oa.axis_error,oa.motor_error,oa.sensorless_estimator_error,oa.controller_error].map(v=>v??'--').join('/');
 $('odCalibration').textContent=oa.motor_pre_calibrated?'motor ready':'motor calibration needed';
 $('odMotorModel').textContent=(oa.pole_pairs??'--')+' / '+(oa.sensorless_pm_flux_linkage===undefined?'--':Number(oa.sensorless_pm_flux_linkage).toFixed(6))+' Wb';
 $('odRamp').textContent=(oa.sensorless_ramp_current_a===undefined?'--':Number(oa.sensorless_ramp_current_a).toFixed(2))+' A / '+(oa.sensorless_ramp_velocity_turns_s===undefined?'--':Math.abs(Number(oa.sensorless_ramp_velocity_turns_s)).toFixed(2))+' t/s / '+(oa.sensorless_ramp_accel_turns_s2===undefined?'--':Math.abs(Number(oa.sensorless_ramp_accel_turns_s2)).toFixed(2))+' t/s²';
 if(!odSensorlessRunning){
  if(oa.current_limit_a!==undefined&&document.activeElement!==$('odCurrentLimit'))$('odCurrentLimit').value=oa.current_limit_a;
  if(oa.velocity_limit_turns_s!==undefined&&document.activeElement!==$('odVelocityLimit'))$('odVelocityLimit').value=oa.velocity_limit_turns_s;
  if(oa.sensorless_ramp_current_a!==undefined&&document.activeElement!==$('odStartupCurrent'))$('odStartupCurrent').value=Number(oa.sensorless_ramp_current_a).toFixed(2);
  if(oa.sensorless_ramp_velocity_turns_s!==undefined&&document.activeElement!==$('odStartupSpeed'))$('odStartupSpeed').value=Math.abs(Number(oa.sensorless_ramp_velocity_turns_s)).toFixed(2);
  if(oa.sensorless_ramp_accel_turns_s2!==undefined&&document.activeElement!==$('odStartupAccel'))$('odStartupAccel').value=Math.abs(Number(oa.sensorless_ramp_accel_turns_s2)).toFixed(2);
 }
 let motion=od.motion||{};
 if(motion.active_axis===null||motion.active_axis===undefined){
  odRunning=false;odSensorlessRunning=false;
  $('odMotionInfo').textContent='ODESC motion stopped.';
 }else if(motion.mode==='sensorless'){
  $('odMotionInfo').textContent='Axis '+motion.active_axis+' sensorless '+motion.velocity_turns_s+' turns/s | startup '+Number(motion.startup_remaining_s||0).toFixed(1)+' s remaining | minimum '+Number(motion.sensorless_min_turns_s||0).toFixed(2)+' turns/s | keepalive '+motion.keepalive_age_ms+' ms';
 }else{
  $('odMotionInfo').textContent='Axis '+motion.active_axis+' encoder velocity '+motion.velocity_turns_s+' turns/s | keepalive '+motion.keepalive_age_ms+' ms';
 }
 let net=data.network||{};text('wifiMode',net.mode);$('wifiConnection').textContent=(net.connection||'--')+' | '+(net.ipv4||'--');renderSignal(net.signal_percent,net.mode);$('wifiInfo').textContent=net.scheduled_mode?'Switching to '+net.scheduled_mode+' in '+Math.ceil((net.switch_in_ms||0)/1000)+' seconds...':(net.last_error?'Wi-Fi error: '+net.last_error:(net.mode==='ap'?'Open network robot | '+net.ap_url:'Automatic AP failover after '+net.auto_failover_seconds+' seconds disconnected or below '+net.minimum_signal_percent+'% signal.'));
 let pi=data.pi||{};text('piTemperature',pi.temperature_c===null||pi.temperature_c===undefined?null:Number(pi.temperature_c).toFixed(1),' °C');text('piUptime',duration(pi.uptime_s));
 let names=['STM32','ODESC','GD32','IMU'], states=[stm.alive,od.connected,gd.round_trip,imu.alive];
 $('summary').textContent=names.map((n,i)=>n+' '+(states[i]?'up':'down')).join(' | ')+' | Wi-Fi '+(net.signal_percent??(net.mode==='ap'?'AP':'--'))+(net.signal_percent===undefined||net.signal_percent===null?'':'%');
 if(data.server&&data.server.last_error)$('serverBadge').className='badge down';
}
async function refresh(){try{render(await request('/api/status'))}catch(error){badge('serverBadge',false,'server error');log('STATUS ERROR: '+error.message)}}
document.addEventListener('visibilitychange',()=>{if(document.hidden){stopJog();if(odSensorlessRunning)odescSensorlessStop();if(odRunning)odescStop()}});
setInterval(refresh,1000);setInterval(refreshOdescEvents,10000);refresh();refreshOdescEvents();
</script>
</body></html>"""


MOBILE_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<meta name="theme-color" content="#172129">
<title>Rover Tank Drive</title>
<style>
*{box-sizing:border-box;letter-spacing:0}
:root{--ink:#172129;--muted:#61717d;--line:#cbd4da;--page:#edf1f4;--blue:#1268a8;--red:#c3261e;--green:#23734b}
html,body{width:100%;max-width:100%;margin:0;overflow-x:hidden;background:var(--page);color:var(--ink);font:15px Arial,sans-serif}
header{position:sticky;top:0;z-index:5;background:#172129;color:#fff;border-bottom:3px solid #d99a16}
.head{display:grid;grid-template-columns:1fr 112px;gap:10px;align-items:center;padding:9px 10px}
h1{font-size:18px;margin:0 0 4px}.status{font-size:11px;color:#ccd7de}.up{color:#78dfa8}.down{color:#ff8d86}
button{min-width:0;min-height:46px;border:1px solid #aeb9c1;border-radius:7px;background:#e5ebef;color:var(--ink);font:700 15px Arial,sans-serif;touch-action:none}
.stop{background:var(--red);border-color:var(--red);color:#fff}.estop{height:54px}
main{width:100%;max-width:520px;margin:auto;padding:10px 10px 34px}
.panel{background:#fff;border:1px solid var(--line);border-radius:8px;padding:11px;margin-bottom:11px}
.panel-head{display:flex;justify-content:space-between;gap:10px;align-items:center;margin-bottom:8px}.panel-head h2{font-size:16px;margin:0}.value{font-size:12px;color:var(--muted)}
.joystick-wrap{display:grid;place-items:center;padding:5px 0 9px}
#joystick{position:relative;width:min(78vw,340px);aspect-ratio:1;border-radius:50%;background:#dce3e8;border:2px solid #9eabb5;box-shadow:inset 0 0 0 2px #f8fafb;touch-action:none}
#joystick:before,#joystick:after{content:"";position:absolute;background:#aab6bf}
#joystick:before{left:50%;top:8%;bottom:8%;width:1px}#joystick:after{top:50%;left:8%;right:8%;height:1px}
#knob{position:absolute;z-index:2;left:50%;top:50%;width:30%;aspect-ratio:1;transform:translate(-50%,-50%);border-radius:50%;background:var(--blue);border:3px solid #fff;box-shadow:0 2px 8px #0004;pointer-events:none}
.tracks{display:grid;grid-template-columns:1fr 1fr;gap:8px}.track{background:#f4f7f8;border-left:4px solid var(--green);border-radius:5px;padding:8px}.track span{display:block;font-size:11px;color:var(--muted)}.track strong{display:block;font-size:18px;margin-top:3px}
.pad{display:grid;grid-template-columns:1fr 1fr 1fr;grid-template-areas:". forward ." "left center right" ". backward .";gap:8px}.pad button{min-height:64px}
#forward{grid-area:forward}#left{grid-area:left}#center{grid-area:center}#right{grid-area:right}#backward{grid-area:backward}
.drive{background:#176fae;border-color:#176fae;color:#fff}.turn{background:#416f2b;border-color:#416f2b;color:#fff}
.note{font-size:12px;color:var(--muted);line-height:1.4;margin:8px 0 0}.footer{display:flex;gap:8px}.footer button{flex:1}
@media(min-width:600px){.head{max-width:520px;margin:auto}#joystick{width:330px}}
</style>
</head>
<body>
<header><div class="head"><div><h1>Tank Drive</h1><div id="link" class="status down">STM32 link checking</div></div><button class="stop estop" onclick="emergencyStop()">STOP ALL</button></div></header>
<main>
 <section class="panel">
  <div class="panel-head"><h2>Joystick</h2><span id="ramp" class="value">ramp -- %/s</span></div>
  <div class="joystick-wrap"><div id="joystick"><div id="knob"></div></div></div>
  <div class="tracks"><div class="track"><span>Left track</span><strong id="leftValue">0%</strong></div><div class="track"><span>Right track</span><strong id="rightValue">0%</strong></div></div>
  <p id="driveState" class="note">Release control to ramp toward stop.</p>
 </section>
 <section class="panel">
  <div class="panel-head"><h2>Full throttle</h2><span class="value">press and hold</span></div>
  <div class="pad">
   <button id="forward" class="drive">↑<br>Forward</button>
   <button id="left" class="turn">←<br>Left</button>
   <button id="center" class="stop" onclick="emergencyStop()">STOP</button>
   <button id="right" class="turn">→<br>Right</button>
   <button id="backward" class="drive">↓<br>Backward</button>
  </div>
 </section>
 <section class="panel"><div class="panel-head"><h2>Wi-Fi</h2><span id="mobileWifi" class="value">checking</span></div><div class="footer"><button onclick="mobileWifiSwitch('ap')">Robot AP</button><button onclick="mobileWifiSwitch('router')">Router Wi-Fi</button></div><p id="mobileWifiInfo" class="note">AP network robot uses http://10.42.0.1:8080/mobile.</p></section>
 <section class="panel"><div class="footer"><button onclick="location.href='/'">Main controls</button><button onclick="refresh()">Refresh</button></div><p class="note">The server stops immediately if control messages disappear for 0.5 seconds.</p></section>
</main>
<script>
const $=id=>document.getElementById(id);
let desiredLeft=0,desiredRight=0,controlActive=false,commandPending=false;
async function api(path,body={}){
 let response=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
 let data=await response.json();if(!response.ok||data.ok===false)throw new Error(data.error||('HTTP '+response.status));return data;
}
function displayTargets(){ $('leftValue').textContent=Math.round(desiredLeft)+'%';$('rightValue').textContent=Math.round(desiredRight)+'%' }
function setTarget(left,right,active=true){desiredLeft=Math.max(-100,Math.min(100,left));desiredRight=Math.max(-100,Math.min(100,right));controlActive=active;displayTargets()}
async function pushTarget(){
 if(commandPending)return;commandPending=true;
 try{await api('/api/tank/target',{left:desiredLeft,right:desiredRight,source:'mobile'})}
 catch(error){$('driveState').textContent='Command error: '+error.message;controlActive=false}
 finally{commandPending=false}
}
function releaseControl(){setTarget(0,0,false);pushTarget();centerKnob()}
function emergencyStop(){controlActive=false;desiredLeft=0;desiredRight=0;displayTargets();centerKnob();api('/api/tank/stop').catch(()=>{});api('/api/stop-all').catch(()=>{})}
function mobileWifiSwitch(mode){if(confirm('Switch Wi-Fi in 3 seconds? Reopen the controller at the new address.'))api('/api/network/'+mode).catch(()=>{})}
const base=$('joystick'),knob=$('knob');
function centerKnob(){knob.style.left='50%';knob.style.top='50%'}
function joystickMove(event){
 let box=base.getBoundingClientRect(),cx=box.left+box.width/2,cy=box.top+box.height/2,max=box.width*.35;
 let dx=event.clientX-cx,dy=event.clientY-cy,length=Math.hypot(dx,dy);if(length>max){dx*=max/length;dy*=max/length}
 knob.style.left=(50+dx/box.width*100)+'%';knob.style.top=(50+dy/box.height*100)+'%';
 let turn=dx/max,forward=-dy/max;if(Math.abs(turn)<.06)turn=0;if(Math.abs(forward)<.06)forward=0;
 let left=forward+turn,right=forward-turn,scale=Math.max(1,Math.abs(left),Math.abs(right));
 setTarget(left/scale*100,right/scale*100,true);pushTarget();
}
base.addEventListener('pointerdown',e=>{base.setPointerCapture(e.pointerId);joystickMove(e)});
base.addEventListener('pointermove',e=>{if(base.hasPointerCapture(e.pointerId))joystickMove(e)});
base.addEventListener('pointerup',releaseControl);base.addEventListener('pointercancel',releaseControl);
function bindHold(id,left,right){
 let button=$(id);
 button.addEventListener('pointerdown',e=>{button.setPointerCapture(e.pointerId);setTarget(left,right,true);pushTarget()});
 button.addEventListener('pointerup',releaseControl);button.addEventListener('pointercancel',releaseControl);button.addEventListener('lostpointercapture',()=>{if(controlActive)releaseControl()});
}
bindHold('forward',100,100);bindHold('backward',-100,-100);bindHold('left',-100,100);bindHold('right',100,-100);
setInterval(()=>{if(controlActive)pushTarget()},100);
async function refresh(){
 try{
  let response=await fetch('/api/status'),data=await response.json(),stm=data.links.stm32||{},od=data.links.odesc||{},tank=data.tank||{},net=data.network||{};
  let battery=od.vbus_voltage===undefined?'battery --':Number(od.vbus_voltage).toFixed(2)+' V';
  $('link').textContent=(stm.alive?'STM32 alive':'STM32 offline')+' | '+battery;$('link').className='status '+(stm.alive?'up':'down');
  $('ramp').textContent='ramp '+(tank.ramp_percent_s||'--')+' %/s';
  $('driveState').textContent='output L '+Math.round(tank.current_left||0)+'% | R '+Math.round(tank.current_right||0)+'% | '+(tank.source||'idle');
  $('mobileWifi').textContent=(net.mode||'unknown')+(net.signal_percent===null||net.signal_percent===undefined?'':' | '+net.signal_percent+'%');
  $('mobileWifiInfo').textContent=net.scheduled_mode?'Switching to '+net.scheduled_mode+' in '+Math.ceil((net.switch_in_ms||0)/1000)+' seconds':(net.mode==='ap'?'Open network robot | http://10.42.0.1:8080/mobile':'Router '+(net.connection||'Wi-Fi')+' | '+(net.ipv4||''));
 }catch(error){$('link').textContent='Server connection lost';$('link').className='status down'}
}
document.addEventListener('visibilitychange',()=>{if(document.hidden)emergencyStop()});
window.addEventListener('pagehide',()=>navigator.sendBeacon('/api/tank/stop','{}'));
setInterval(refresh,500);centerKnob();refresh();
</script>
</body></html>"""


SENSORS_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<meta name="theme-color" content="#172129">
<title>Rover IMU Sensors</title>
<style>
*{box-sizing:border-box;letter-spacing:0}
:root{--ink:#172129;--muted:#60707d;--line:#cbd5dc;--page:#edf1f4;--blue:#1268a8;--green:#24734c;--red:#bd241c}
body{margin:0;background:var(--page);color:var(--ink);font:15px Arial,sans-serif}
header{position:sticky;top:0;z-index:5;background:#172129;color:#fff;border-bottom:3px solid #d99a16}
.head{max-width:1100px;margin:auto;padding:10px 12px;display:grid;grid-template-columns:1fr auto;gap:10px;align-items:center}
h1{font-size:19px;margin:0 0 4px}.sub{font-size:12px;color:#cbd5dc}
nav{display:flex;gap:6px}nav a{color:#fff;text-decoration:none;border:1px solid #61717d;border-radius:5px;padding:8px}
main{max-width:1100px;margin:auto;padding:12px 12px 36px}
.panel{background:#fff;border:1px solid var(--line);border-radius:8px;padding:12px;margin-bottom:12px}
.toolbar{display:grid;grid-template-columns:1fr 1fr;gap:8px}.grid2{display:grid;grid-template-columns:1fr 1fr;gap:10px}.grid4{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}
button,input{font:inherit;min-height:44px;border:1px solid #aebac3;border-radius:6px;padding:8px}
button{font-weight:700;cursor:pointer;background:#e7edf1}.start{background:var(--green);border-color:var(--green);color:#fff}.stop{background:var(--red);border-color:var(--red);color:#fff}
label{display:grid;gap:5px;color:var(--muted);font-size:12px}input{width:100%;background:#fff;color:var(--ink)}
.metric{border-left:4px solid #16816a;background:#f5f8f9;padding:8px;border-radius:4px;min-height:62px}.metric span{display:block;color:var(--muted);font-size:11px}.metric strong{display:block;font-size:18px;margin-top:4px;overflow-wrap:anywhere}
.visual{display:grid;grid-template-columns:270px 1fr;gap:12px}.canvas-wrap{min-width:0}.canvas-title{display:flex;justify-content:space-between;font-size:12px;font-weight:700;margin-bottom:6px}
canvas{display:block;width:100%;background:#f7f9fa;border:1px solid var(--line);border-radius:6px}.compass{aspect-ratio:1}.route{height:430px}.history{height:180px}
.note{font-size:12px;color:var(--muted);line-height:1.45;margin:8px 0 0}.error{color:#a11f18}.active{color:#69dea0}
@media(max-width:760px){.visual{grid-template-columns:1fr}.compass{max-width:260px;margin:auto}.grid4{grid-template-columns:1fr 1fr}.route{height:330px}}
@media(max-width:440px){main{padding:8px}.head{grid-template-columns:1fr}.grid2,.toolbar{grid-template-columns:1fr}.route{height:300px}nav a{flex:1;text-align:center}}
</style>
</head>
<body>
<header><div class="head"><div><h1>BNO080 Navigation Sensors</h1><div id="link" class="sub">Waiting for IMU...</div></div><nav><a href="/">Controls</a><a href="/mobile">Drive</a><a href="/battery">Battery</a></nav></div></header>
<main>
<section class="panel">
 <div class="toolbar"><button id="startButton" class="start" onclick="startTrip()">Start trip</button><button id="stopButton" class="stop" onclick="stopTrip()">Stop trip</button></div>
 <div class="grid2" style="margin-top:10px"><label>Robot heading offset (-180 to 180 degrees)<input id="offset" type="number" min="-180" max="180" step="0.1" value="0"></label><button onclick="saveOffset()">Save heading offset</button><button onclick="setCurrentForward()">Use current IMU heading as robot 0 degrees</button><div class="metric"><span>Trip state</span><strong id="tripState">Stopped</strong></div></div>
 <p class="note">The offset rotates the IMU heading and horizontal acceleration into the robot frame. Stop the trip before changing it.</p>
</section>

<section class="panel visual">
 <div class="canvas-wrap"><div class="canvas-title"><span>Robot compass</span><span id="headingLabel">--</span></div><canvas id="compass" class="compass"></canvas></div>
 <div class="canvas-wrap"><div class="canvas-title"><span>Inertial backtrack</span><span id="scaleLabel">--</span></div><canvas id="route" class="route"></canvas></div>
</section>

<section class="panel">
 <div class="grid4">
  <div class="metric"><span>Corrected heading</span><strong id="corrected">--</strong></div>
  <div class="metric"><span>Raw IMU heading</span><strong id="rawHeading">--</strong></div>
  <div class="metric"><span>Mag accuracy 0-3</span><strong id="accuracy">--</strong></div>
  <div class="metric"><span>Trip elapsed</span><strong id="elapsed">--</strong></div>
  <div class="metric"><span>East / north</span><strong id="position">--</strong></div>
  <div class="metric"><span>Displacement</span><strong id="displacement">--</strong></div>
  <div class="metric"><span>Integrated path</span><strong id="pathDistance">--</strong></div>
  <div class="metric"><span>Estimated speed</span><strong id="speed">--</strong></div>
 </div>
</section>

<section class="panel"><div class="canvas-title"><span>Heading history</span><span>degrees / trip time</span></div><canvas id="headingHistory" class="history"></canvas></section>

<section class="panel">
 <div class="grid2">
  <div class="metric"><span>Magnetic field X / Y / Z</span><strong id="magnetic">--</strong></div>
  <div class="metric"><span>Quaternion i / j / k / real</span><strong id="quaternion">--</strong></div>
  <div class="metric"><span>Gyroscope X / Y / Z</span><strong id="gyro">--</strong></div>
  <div class="metric"><span>Acceleration X / Y / Z</span><strong id="accel">--</strong></div>
  <div class="metric"><span>Linear acceleration X / Y / Z</span><strong id="linear">--</strong></div>
  <div class="metric"><span>Trip samples</span><strong id="samples">0</strong></div>
 </div>
 <p class="note"><strong>Important:</strong> heading and turns come from BNO080 fusion. Position is a crude experiment based only on double integration of linear acceleration. Bias, tilt error, vibration, wheel slip, and stopping can make distance drift quickly. Do not use it for autonomous safety or precise navigation; fuse wheel encoders and GPS/RTK when available.</p>
 <p id="error" class="note error"></p>
</section>
</main>
<script>
const $=id=>document.getElementById(id);
let latest=null;
function fmt(value,digits=2){return value===undefined||value===null?'--':Number(value).toFixed(digits)}
function vector(values,digits=3){return values?values.map(v=>Number(v).toFixed(digits)).join(' / '):'--'}
function duration(seconds){if(seconds===null||seconds===undefined)return '--';seconds=Math.max(0,Math.round(seconds));let h=Math.floor(seconds/3600),m=Math.floor(seconds%3600/60),s=seconds%60;return (h?h+'h ':'')+(m?m+'m ':'')+s+'s'}
async function request(path,body){let options=body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)};let response=await fetch(path,options),data=await response.json();if(!response.ok||data.ok===false)throw new Error(data.error||'request failed');return data}
async function action(path,body={}){try{await request(path,body);await refresh()}catch(error){$('error').textContent=error.message}}
function startTrip(){if(confirm('Start a new trip and clear the current route?'))action('/api/trip/start')}
function stopTrip(){action('/api/trip/stop')}
function saveOffset(){action('/api/trip/offset',{heading_offset_deg:Number($('offset').value)})}
function setCurrentForward(){if(!latest||!latest.reading)return;$('offset').value=((360-Number(latest.reading.heading_deg)+180)%360-180).toFixed(1);saveOffset()}
function canvasContext(id){
 let canvas=$(id),ratio=window.devicePixelRatio||1,width=Math.max(280,canvas.clientWidth),height=Math.max(160,canvas.clientHeight);
 if(canvas.width!==Math.round(width*ratio)||canvas.height!==Math.round(height*ratio)){canvas.width=Math.round(width*ratio);canvas.height=Math.round(height*ratio)}
 let ctx=canvas.getContext('2d');ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,width,height);return {ctx,width,height}
}
function drawCompass(heading){
 let {ctx,width,height}=canvasContext('compass'),cx=width/2,cy=height/2,r=Math.min(width,height)*.39;
 ctx.strokeStyle='#9eacb6';ctx.lineWidth=2;ctx.beginPath();ctx.arc(cx,cy,r,0,Math.PI*2);ctx.stroke();
 ctx.fillStyle='#60707d';ctx.font='bold 13px Arial';ctx.textAlign='center';ctx.textBaseline='middle';
 [['N',0,-1],['E',1,0],['S',0,1],['W',-1,0]].forEach(v=>ctx.fillText(v[0],cx+v[1]*(r+16),cy+v[2]*(r+16)));
 for(let d=0;d<360;d+=15){let a=d*Math.PI/180-Math.PI/2,inner=r-(d%45===0?10:5);ctx.beginPath();ctx.moveTo(cx+Math.cos(a)*inner,cy+Math.sin(a)*inner);ctx.lineTo(cx+Math.cos(a)*r,cy+Math.sin(a)*r);ctx.stroke()}
 if(heading===null||heading===undefined)return;
 let a=Number(heading)*Math.PI/180-Math.PI/2;ctx.strokeStyle='#bd241c';ctx.fillStyle='#bd241c';ctx.lineWidth=4;ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(cx+Math.cos(a)*r*.78,cy+Math.sin(a)*r*.78);ctx.stroke();ctx.beginPath();ctx.arc(cx,cy,6,0,Math.PI*2);ctx.fill();
}
function drawRoute(samples){
 let {ctx,width,height}=canvasContext('route'),pad=34;
 ctx.strokeStyle='#d9e0e5';ctx.lineWidth=1;for(let i=1;i<5;i++){let x=pad+(width-2*pad)*i/5,y=pad+(height-2*pad)*i/5;ctx.beginPath();ctx.moveTo(x,pad);ctx.lineTo(x,height-pad);ctx.stroke();ctx.beginPath();ctx.moveTo(pad,y);ctx.lineTo(width-pad,y);ctx.stroke()}
 if(!samples||samples.length<1){ctx.fillStyle='#60707d';ctx.textAlign='center';ctx.font='14px Arial';ctx.fillText('Start a trip to record a route',width/2,height/2);$('scaleLabel').textContent='--';return}
 let max=Math.max(1,...samples.map(p=>Math.max(Math.abs(p.east),Math.abs(p.north))))*1.12,scale=Math.min((width-2*pad)/(2*max),(height-2*pad)/(2*max)),x=east=>width/2+east*scale,y=north=>height/2-north*scale;
 ctx.strokeStyle='#1268a8';ctx.lineWidth=3;ctx.lineJoin='round';ctx.beginPath();samples.forEach((p,i)=>i?ctx.lineTo(x(p.east),y(p.north)):ctx.moveTo(x(p.east),y(p.north)));ctx.stroke();
 ctx.fillStyle='#24734c';ctx.beginPath();ctx.arc(x(samples[0].east),y(samples[0].north),6,0,Math.PI*2);ctx.fill();
 let p=samples[samples.length-1];ctx.fillStyle='#bd241c';ctx.beginPath();ctx.arc(x(p.east),y(p.north),7,0,Math.PI*2);ctx.fill();
 let a=p.heading*Math.PI/180-Math.PI/2;ctx.strokeStyle='#bd241c';ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(x(p.east),y(p.north));ctx.lineTo(x(p.east)+Math.cos(a)*24,y(p.north)+Math.sin(a)*24);ctx.stroke();
 ctx.fillStyle='#60707d';ctx.font='11px Arial';ctx.textAlign='left';ctx.fillText('N',width/2+5,pad-7);$('scaleLabel').textContent='±'+max.toFixed(max<10?1:0)+' m';
}
function drawHeading(samples){
 let {ctx,width,height}=canvasContext('headingHistory'),pad=28;ctx.strokeStyle='#d9e0e5';ctx.lineWidth=1;
 [0,90,180,270,360].forEach(d=>{let y=height-pad-d*(height-2*pad)/360;ctx.beginPath();ctx.moveTo(pad,y);ctx.lineTo(width-pad,y);ctx.stroke();ctx.fillStyle='#60707d';ctx.font='10px Arial';ctx.fillText(d+'°',2,y+3)});
 if(!samples||samples.length<2)return;let end=Math.max(1,samples[samples.length-1].t),previous=null;ctx.strokeStyle='#26734d';ctx.lineWidth=2;ctx.beginPath();samples.forEach((p,i)=>{let x=pad+p.t/end*(width-2*pad),y=height-pad-p.heading/360*(height-2*pad),wrap=previous!==null&&Math.abs(p.heading-previous)>180;(i&&!wrap)?ctx.lineTo(x,y):ctx.moveTo(x,y);previous=p.heading});ctx.stroke();
}
function render(data){
 latest=data;let r=data.reading||{},t=data.trip||{},connected=!!data.reading&&!data.error;
 $('link').textContent=connected?'IMU streaming | '+(t.active?'trip recording':'trip stopped'):'IMU unavailable'+(data.error?' | '+data.error:'');$('link').className='sub '+(connected?'active':'error');
 if(document.activeElement!==$('offset'))$('offset').value=Number(t.heading_offset_deg||0).toFixed(1);
 $('startButton').disabled=!!t.active;$('stopButton').disabled=!t.active;$('tripState').textContent=t.active?'Recording':'Stopped';
 $('headingLabel').textContent=fmt(r.corrected_heading_deg,1)+'°';$('corrected').textContent=fmt(r.corrected_heading_deg,1)+'°';$('rawHeading').textContent=fmt(r.heading_deg,1)+'°';$('accuracy').textContent=r.magnetometer_accuracy??'--';
 $('elapsed').textContent=duration(t.elapsed_s);$('position').textContent=fmt(t.east_m)+' E / '+fmt(t.north_m)+' N m';$('displacement').textContent=fmt(t.displacement_m)+' m';$('pathDistance').textContent=fmt(t.path_distance_m)+' m';$('speed').textContent=fmt(t.speed_m_s)+' m/s';
 $('magnetic').textContent=vector(r.magnetic_uT)+' µT';$('quaternion').textContent=vector(r.quaternion,4);$('gyro').textContent=vector(r.gyro_rad_s)+' rad/s';$('accel').textContent=vector(r.acceleration_m_s2)+' m/s²';$('linear').textContent=vector(r.linear_acceleration_m_s2)+' m/s²';$('samples').textContent=t.sample_count||0;$('error').textContent=data.error||'';
 drawCompass(r.corrected_heading_deg);drawRoute(t.samples||[]);drawHeading(t.samples||[]);
}
async function refresh(){try{render(await request('/api/sensors'))}catch(error){$('error').textContent=error.message}}
window.addEventListener('resize',()=>{if(latest)render(latest)});setInterval(refresh,500);refresh();
</script>
</body></html>"""


BATTERY_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<meta name="theme-color" content="#172129">
<title>Rover Battery History</title>
<style>
*{box-sizing:border-box;letter-spacing:0}
:root{--ink:#172129;--muted:#60707d;--line:#cbd5dc;--page:#edf1f4;--green:#24734c;--blue:#1268a8;--amber:#a96800}
body{margin:0;background:var(--page);color:var(--ink);font:15px Arial,sans-serif}
header{position:sticky;top:0;z-index:5;background:#172129;color:#fff;border-bottom:3px solid #d99a16}.head{max-width:1100px;margin:auto;padding:10px 12px;display:grid;grid-template-columns:1fr auto;gap:10px;align-items:center}
h1{font-size:19px;margin:0 0 4px}.sub{font-size:12px;color:#cbd5dc}nav{display:flex;gap:6px}nav a{color:#fff;text-decoration:none;border:1px solid #61717d;border-radius:5px;padding:8px}
main{max-width:1100px;margin:auto;padding:12px 12px 36px}.panel{background:#fff;border:1px solid var(--line);border-radius:8px;padding:12px;margin-bottom:12px}
.grid3{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.grid4{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}.metric{border-left:4px solid #16816a;background:#f5f8f9;padding:8px;border-radius:4px;min-height:62px}.metric span{display:block;color:var(--muted);font-size:11px}.metric strong{display:block;font-size:18px;margin-top:4px;overflow-wrap:anywhere}
label{display:grid;gap:5px;color:var(--muted);font-size:12px}select{font:inherit;width:100%;min-height:44px;border:1px solid #aebac3;border-radius:6px;padding:8px;background:#fff}
.chart-head{display:flex;justify-content:space-between;gap:8px;font-size:12px;font-weight:700;margin-bottom:6px}canvas{display:block;width:100%;height:260px;background:#f7f9fa;border:1px solid var(--line);border-radius:6px;touch-action:none}
.note{font-size:12px;color:var(--muted);line-height:1.45;margin:8px 0 0}.session{font-size:12px;color:var(--muted);margin-top:7px}
@media(max-width:720px){.grid4{grid-template-columns:1fr 1fr}.grid3{grid-template-columns:1fr}canvas{height:220px}}
@media(max-width:440px){main{padding:8px}.head{grid-template-columns:1fr}nav a{flex:1;text-align:center}}
</style>
</head>
<body>
<header><div class="head"><div><h1>Battery and Power History</h1><div id="headerStatus" class="sub">Loading durable history...</div></div><nav><a href="/">Controls</a><a href="/mobile">Drive</a><a href="/sensors">Sensors</a></nav></div></header>
<main>
<section class="panel">
 <div class="grid3"><label>Power-on session<select id="session" onchange="loadSession(this.value)"></select><div id="sessionInfo" class="session"></div></label><div class="metric"><span>Pi uptime this boot</span><strong id="uptime">--</strong></div><div class="metric"><span>Pack configuration</span><strong id="pack">10S4P / 10.4 Ah</strong></div></div>
</section>
<section class="panel"><div class="grid4">
 <div class="metric"><span>Latest bus voltage</span><strong id="voltage">--</strong></div>
 <div class="metric"><span>Approximate voltage SOC</span><strong id="soc">--</strong></div>
 <div class="metric"><span>ODESC bus current</span><strong id="current">--</strong></div>
 <div class="metric"><span>ODESC bus power</span><strong id="power">--</strong></div>
 <div class="metric"><span>ODESC session energy</span><strong id="energy">--</strong></div>
 <div class="metric"><span>Voltage-slope current estimate</span><strong id="estimatedCurrent">--</strong></div>
 <div class="metric"><span>Selected sample time</span><strong id="sampleTime">--</strong></div>
 <div class="metric"><span>Samples saved</span><strong id="sampleCount">0</strong></div>
</div></section>
<section class="panel"><div class="chart-head"><span>Pack voltage and estimated SOC</span><span id="voltageRange">--</span></div><canvas id="voltageChart"></canvas><p class="note">Touch or point at the chart to inspect a sample.</p></section>
<section class="panel"><div class="chart-head"><span>ODESC current and power</span><span>signed bus telemetry</span></div><canvas id="powerChart"></canvas></section>
<section class="panel">
 <p class="note">A separate history is created for each Raspberry Pi boot. A committed row is written after each successful ODESC refresh, normally every five seconds. SQLite WAL mode with full synchronization limits abrupt-unplug data loss to the current in-progress sample.</p>
 <p class="note">Voltage SOC and voltage-slope current are rough 10S lithium-ion estimates. Cell chemistry, load sag, temperature, cell imbalance, and charging strongly affect them. The slope estimate needs at least five minutes and must not replace a current sensor. ODESC <code>ibus</code> is direct controller telemetry, but it does not include Pi or accessory loads connected outside the ODESC bus measurement.</p>
 <p id="error" class="note"></p>
</section>
</main>
<script>
const $=id=>document.getElementById(id);let data=null,selectedIndex=null;
function duration(seconds){seconds=Math.max(0,Math.round(Number(seconds)||0));let d=Math.floor(seconds/86400),h=Math.floor(seconds%86400/3600),m=Math.floor(seconds%3600/60),s=seconds%60;return (d?d+'d ':'')+h+'h '+m+'m '+s+'s'}
function value(v,d=2,s=''){return v===undefined||v===null?'--':Number(v).toFixed(d)+s}
function canvasContext(id){let canvas=$(id),ratio=window.devicePixelRatio||1,width=Math.max(280,canvas.clientWidth),height=Math.max(180,canvas.clientHeight);if(canvas.width!==Math.round(width*ratio)||canvas.height!==Math.round(height*ratio)){canvas.width=Math.round(width*ratio);canvas.height=Math.round(height*ratio)}let ctx=canvas.getContext('2d');ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,width,height);return {ctx,width,height}}
function lineChart(id,samples,series){
 let {ctx,width,height}=canvasContext(id),pad={l:42,r:16,t:18,b:28};ctx.strokeStyle='#d9e0e5';ctx.lineWidth=1;
 for(let i=0;i<5;i++){let y=pad.t+(height-pad.t-pad.b)*i/4;ctx.beginPath();ctx.moveTo(pad.l,y);ctx.lineTo(width-pad.r,y);ctx.stroke()}
 if(!samples.length){ctx.fillStyle='#60707d';ctx.textAlign='center';ctx.fillText('No samples in this power-on session',width/2,height/2);return}
 let ranges=series.map(s=>{let values=samples.map(s.get).filter(v=>v!==null&&v!==undefined&&Number.isFinite(Number(v))).map(Number),min=s.fixedMin??Math.min(...values),max=s.fixedMax??Math.max(...values);if(max-min<.01){min-=1;max+=1}else if(s.fixedMin===undefined){let margin=(max-min)*.08;min-=margin;max+=margin}return {min,max}});
 let x=i=>pad.l+i*(width-pad.l-pad.r)/Math.max(1,samples.length-1),y=(v,r)=>pad.t+(r.max-v)*(height-pad.t-pad.b)/(r.max-r.min);
 series.forEach((s,index)=>{ctx.strokeStyle=s.color;ctx.lineWidth=2;ctx.beginPath();let started=false;samples.forEach((p,i)=>{let raw=s.get(p);if(raw===null||raw===undefined)return;let px=x(i),py=y(Number(raw),ranges[index]);started?ctx.lineTo(px,py):ctx.moveTo(px,py);started=true});ctx.stroke()});
 ctx.font='10px Arial';ctx.fillStyle=series[0].color;ctx.textAlign='right';ctx.fillText(ranges[0].max.toFixed(1),pad.l-5,pad.t+4);ctx.fillText(ranges[0].min.toFixed(1),pad.l-5,height-pad.b);
 if(series[1]){ctx.fillStyle=series[1].color;ctx.textAlign='left';ctx.fillText(ranges[1].max.toFixed(1),width-pad.r+3,pad.t+4);ctx.fillText(ranges[1].min.toFixed(1),width-pad.r+3,height-pad.b)}
 ctx.textAlign='left';series.forEach((s,i)=>{ctx.fillStyle=s.color;ctx.fillText(s.label,pad.l+i*120,12)});
 if(selectedIndex!==null&&selectedIndex<samples.length){let px=x(selectedIndex);ctx.strokeStyle='#172129';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(px,pad.t);ctx.lineTo(px,height-pad.b);ctx.stroke()}
}
function draw(){if(!data)return;let samples=data.samples||[];lineChart('voltageChart',samples,[{label:'voltage V',color:'#1268a8',get:p=>p.voltage_v},{label:'SOC %',color:'#24734c',fixedMin:0,fixedMax:100,get:p=>p.soc_percent}]);lineChart('powerChart',samples,[{label:'current A',color:'#a96800',get:p=>p.bus_current_a},{label:'power W',color:'#bd241c',get:p=>p.bus_power_w}]);}
function inspect(index){if(!data||!data.samples.length)return;selectedIndex=Math.max(0,Math.min(data.samples.length-1,index));let p=data.samples[selectedIndex];$('voltage').textContent=value(p.voltage_v,2,' V');$('soc').textContent=value(p.soc_percent,1,'%');$('current').textContent=value(p.bus_current_a,2,' A');$('power').textContent=value(p.bus_power_w,1,' W');$('energy').textContent=value(p.energy_used_wh,3,' Wh');$('estimatedCurrent').textContent=value(p.estimated_pack_current_a,2,' A');$('sampleTime').textContent=new Date(p.wall_time*1000).toLocaleString();draw()}
function chartPointer(event){if(!data||!data.samples.length)return;let rect=event.currentTarget.getBoundingClientRect(),ratio=Math.max(0,Math.min(1,(event.clientX-rect.left-42)/(rect.width-58)));inspect(Math.round(ratio*(data.samples.length-1)))}
['voltageChart','powerChart'].forEach(id=>{$(id).addEventListener('pointerdown',chartPointer);$(id).addEventListener('pointermove',event=>{if(event.pointerType==='mouse'||event.buttons)chartPointer(event)})});
function render(payload){
 data=payload;let sessions=payload.sessions||[],samples=payload.samples||[],selected=String(payload.selected_session_id);
 $('session').innerHTML=sessions.map(s=>'<option value="'+s.id+'"'+(String(s.id)===selected?' selected':'')+'>'+new Date(s.boot_wall_time*1000).toLocaleString()+(s.current?' (current)':'')+'</option>').join('');
 let session=sessions.find(s=>String(s.id)===selected);$('sessionInfo').textContent=session?'samples '+session.sample_count+' | '+value(session.minimum_voltage_v,2,' V')+' to '+value(session.maximum_voltage_v,2,' V'):'No session';
 $('uptime').textContent=duration(payload.pi_uptime_s);$('pack').textContent=payload.pack.series_cells+'S'+payload.pack.parallel_cells+'P / '+payload.pack.pack_capacity_ah.toFixed(1)+' Ah';$('sampleCount').textContent=session?session.sample_count:samples.length;
 $('headerStatus').textContent=samples.length?'History loaded | latest '+new Date(samples[samples.length-1].wall_time*1000).toLocaleTimeString():'Waiting for ODESC voltage samples';
 if(samples.length){let values=samples.map(p=>p.voltage_v);$('voltageRange').textContent=Math.min(...values).toFixed(2)+' to '+Math.max(...values).toFixed(2)+' V';inspect(samples.length-1)}else{$('voltageRange').textContent='--';selectedIndex=null;draw()}
}
async function loadSession(id=''){try{let response=await fetch('/api/battery'+(id?'?session_id='+encodeURIComponent(id):'')),payload=await response.json();if(!response.ok||payload.ok===false)throw new Error(payload.error||'request failed');render(payload);$('error').textContent=''}catch(error){$('error').textContent='Battery history error: '+error.message}}
window.addEventListener('resize',draw);setInterval(()=>loadSession($('session').value),5000);loadSession();
</script>
</body></html>"""


def _number(data, name, low, high, integer=False):
    if name not in data:
        raise ValueError("missing field: " + name)
    value = int(data[name]) if integer else float(data[name])
    if value < low or value > high:
        raise ValueError("{} must be between {} and {}".format(name, low, high))
    return value


def _tank_snapshot():
    with TANK_STATE_LOCK:
        age_ms = (
            None
            if not TANK_STATE["last_target_monotonic"]
            else round(
                (time.monotonic() - TANK_STATE["last_target_monotonic"]) * 1000
            )
        )
        return {
            "target_left": round(TANK_STATE["target_left"], 1),
            "target_right": round(TANK_STATE["target_right"], 1),
            "current_left": round(TANK_STATE["current_left"], 1),
            "current_right": round(TANK_STATE["current_right"], 1),
            "ramp_percent_s": TANK_STATE["ramp_percent_s"],
            "command_age_ms": age_ms,
            "source": TANK_STATE["source"],
            "deadman_stops": TANK_STATE["deadman_stops"],
            "last_error": TANK_STATE["last_error"],
        }


def _tank_reset_state(source):
    with TANK_STATE_LOCK:
        TANK_STATE["target_left"] = 0.0
        TANK_STATE["target_right"] = 0.0
        TANK_STATE["current_left"] = 0.0
        TANK_STATE["current_right"] = 0.0
        TANK_STATE["revision"] += 1
        TANK_STATE["last_target_monotonic"] = 0.0
        TANK_STATE["source"] = source


def _tank_stop():
    _tank_reset_state("stopped")
    if ROBOT.stm32 is None:
        return "STM32 not connected; tank state cleared"
    with ACTION_LOCK:
        return ROBOT.stop_drive()


def _stop_all():
    _tank_reset_state("STOP ALL")
    _stepper_jog_stop()
    with ODESC_MOTION_LOCK:
        ODESC_MOTION_STATE["active_axis"] = None
        ODESC_MOTION_STATE["mode"] = None
        ODESC_MOTION_STATE["velocity_turns_s"] = 0.0
        ODESC_MOTION_STATE["startup_started_monotonic"] = 0.0
        ODESC_MOTION_STATE["startup_duration_s"] = 0.0
        ODESC_MOTION_STATE["sensorless_min_turns_s"] = 0.0
        ODESC_MOTION_STATE["sensorless_command_sent"] = False
        ODESC_MOTION_STATE["last_keepalive_monotonic"] = 0.0
    with ACTION_LOCK:
        return ROBOT.stop_all()


def _tank_set_target(left, right, source):
    with TANK_STATE_LOCK:
        TANK_STATE["target_left"] = float(left)
        TANK_STATE["target_right"] = float(right)
        TANK_STATE["revision"] += 1
        TANK_STATE["last_target_monotonic"] = time.monotonic()
        TANK_STATE["source"] = str(source)[:32] or "remote"
        TANK_STATE["last_error"] = ""
    return _tank_snapshot()


def _tank_set_ramp(rate):
    rate = float(rate)
    with TANK_STATE_LOCK:
        TANK_STATE["ramp_percent_s"] = rate
    _save_config_value("tank_ramp_percent_s", rate)
    return _tank_snapshot()


def _approach(current, target, maximum_change):
    if current < target:
        return min(target, current + maximum_change)
    if current > target:
        return max(target, current - maximum_change)
    return current


def _apply_tank(left, right):
    # Installed orientation:
    #   right track, motor 1: A forward, B reverse
    #   left track, motor 2:  B forward, A reverse
    with ACTION_LOCK:
        if abs(left) < 0.05 and abs(right) < 0.05:
            return ROBOT.stop_drive()
        left_direction = "B" if left >= 0 else "A"
        right_direction = "A" if right >= 0 else "B"
        left_reply = ROBOT.drive_motor(2, left_direction, abs(left))
        right_reply = ROBOT.drive_motor(1, right_direction, abs(right))
    return [left_reply, right_reply]


def _tank_loop():
    interval = 1.0 / TANK_UPDATE_HZ
    previous = time.monotonic()
    last_applied = (0.0, 0.0)
    while True:
        started = time.monotonic()
        elapsed = min(0.2, max(0.0, started - previous))
        previous = started
        deadman = False
        with TANK_STATE_LOCK:
            command_age = (
                started - TANK_STATE["last_target_monotonic"]
                if TANK_STATE["last_target_monotonic"]
                else 0.0
            )
            moving_target = (
                abs(TANK_STATE["target_left"]) > 0.05
                or abs(TANK_STATE["target_right"]) > 0.05
            )
            if moving_target and command_age > TANK_DEADMAN_S:
                TANK_STATE["target_left"] = 0.0
                TANK_STATE["target_right"] = 0.0
                TANK_STATE["current_left"] = 0.0
                TANK_STATE["current_right"] = 0.0
                TANK_STATE["revision"] += 1
                TANK_STATE["source"] = "deadman stop"
                TANK_STATE["deadman_stops"] += 1
                deadman = True
            else:
                maximum_change = TANK_STATE["ramp_percent_s"] * elapsed
                TANK_STATE["current_left"] = _approach(
                    TANK_STATE["current_left"],
                    TANK_STATE["target_left"],
                    maximum_change,
                )
                TANK_STATE["current_right"] = _approach(
                    TANK_STATE["current_right"],
                    TANK_STATE["target_right"],
                    maximum_change,
                )
            current = (
                TANK_STATE["current_left"],
                TANK_STATE["current_right"],
            )
            revision = TANK_STATE["revision"]

        changed = (
            abs(current[0] - last_applied[0]) >= 0.25
            or abs(current[1] - last_applied[1]) >= 0.25
        )
        if deadman or changed:
            try:
                if ROBOT.stm32 is not None:
                    with ACTION_LOCK:
                        with TANK_STATE_LOCK:
                            still_current = TANK_STATE["revision"] == revision
                        if still_current:
                            if deadman:
                                ROBOT.stop_drive()
                            else:
                                _apply_tank(*current)
                last_applied = current
                with TANK_STATE_LOCK:
                    TANK_STATE["last_error"] = ""
            except Exception as exc:
                _tank_reset_state("drive fault")
                with TANK_STATE_LOCK:
                    TANK_STATE["last_error"] = str(exc)
                try:
                    if ROBOT.stm32 is not None:
                        ROBOT.stop_drive()
                except Exception:
                    pass
                last_applied = (0.0, 0.0)
        time.sleep(max(0.0, interval - (time.monotonic() - started)))


def _odesc_motion_snapshot():
    with ODESC_MOTION_LOCK:
        age_ms = (
            None
            if not ODESC_MOTION_STATE["last_keepalive_monotonic"]
            else round(
                (
                    time.monotonic()
                    - ODESC_MOTION_STATE["last_keepalive_monotonic"]
                )
                * 1000
            )
        )
        startup_elapsed = (
            0.0
            if not ODESC_MOTION_STATE["startup_started_monotonic"]
            else time.monotonic()
            - ODESC_MOTION_STATE["startup_started_monotonic"]
        )
        return {
            "active_axis": ODESC_MOTION_STATE["active_axis"],
            "mode": ODESC_MOTION_STATE["mode"],
            "velocity_turns_s": ODESC_MOTION_STATE["velocity_turns_s"],
            "startup_remaining_s": max(
                0.0,
                ODESC_MOTION_STATE["startup_duration_s"] - startup_elapsed,
            ),
            "sensorless_min_turns_s": ODESC_MOTION_STATE[
                "sensorless_min_turns_s"
            ],
            "sensorless_command_sent": ODESC_MOTION_STATE[
                "sensorless_command_sent"
            ],
            "keepalive_age_ms": age_ms,
            "deadman_stops": ODESC_MOTION_STATE["deadman_stops"],
            "last_error": ODESC_MOTION_STATE["last_error"],
        }


def _odesc_velocity(axis, velocity):
    axis = int(axis)
    velocity = float(velocity)
    with ODESC_MOTION_LOCK:
        previous_axis = ODESC_MOTION_STATE["active_axis"]
        previous_mode = ODESC_MOTION_STATE["mode"]
        changed = (
            previous_axis != axis
            or previous_mode != "encoder"
            or ODESC_MOTION_STATE["velocity_turns_s"] != velocity
        )
    if previous_axis is not None and previous_axis != axis:
        link = ROBOT._require("odesc")
        if previous_mode == "sensorless":
            link.stop_sensorless(previous_axis)
        else:
            link.stop_axis(previous_axis)
    if changed:
        ROBOT.set_odesc_velocity(axis, velocity)
    with ODESC_MOTION_LOCK:
        ODESC_MOTION_STATE["active_axis"] = axis
        ODESC_MOTION_STATE["mode"] = "encoder"
        ODESC_MOTION_STATE["velocity_turns_s"] = velocity
        ODESC_MOTION_STATE["startup_started_monotonic"] = 0.0
        ODESC_MOTION_STATE["startup_duration_s"] = 0.0
        ODESC_MOTION_STATE["sensorless_min_turns_s"] = 0.0
        ODESC_MOTION_STATE["sensorless_command_sent"] = False
        ODESC_MOTION_STATE["last_keepalive_monotonic"] = time.monotonic()
        ODESC_MOTION_STATE["last_error"] = ""
    return _odesc_motion_snapshot()


def _odesc_sensorless_start(axis, direction, velocity_turns_s):
    axis = int(axis)
    direction = 1 if int(direction) >= 0 else -1
    speed = abs(float(velocity_turns_s))
    link = ROBOT._require("odesc")
    before = link.axis_snapshot(axis)
    minimum_speed = abs(before["sensorless_ramp_velocity_turns_s"])
    velocity_limit = abs(before["velocity_limit_turns_s"])
    if speed < minimum_speed:
        raise ValueError(
            "sensorless target must be at least {:.3f} turns/s".format(
                minimum_speed
            )
        )
    if speed > velocity_limit:
        raise ValueError(
            "sensorless target exceeds {:.3f} turns/s velocity limit".format(
                velocity_limit
            )
        )
    result = ROBOT.start_odesc_sensorless(axis, direction)
    accel = abs(result["sensorless_ramp_accel_turns_s2"])
    if accel <= 0.0:
        ROBOT.stop_odesc_sensorless(axis)
        raise RuntimeError("sensorless startup acceleration is zero")
    startup_duration = minimum_speed / accel + 0.35
    with ODESC_MOTION_LOCK:
        ODESC_MOTION_STATE["active_axis"] = axis
        ODESC_MOTION_STATE["mode"] = "sensorless"
        ODESC_MOTION_STATE["velocity_turns_s"] = direction * speed
        ODESC_MOTION_STATE["startup_started_monotonic"] = time.monotonic()
        ODESC_MOTION_STATE["startup_duration_s"] = startup_duration
        ODESC_MOTION_STATE["sensorless_min_turns_s"] = minimum_speed
        ODESC_MOTION_STATE["sensorless_command_sent"] = False
        ODESC_MOTION_STATE["last_keepalive_monotonic"] = time.monotonic()
        ODESC_MOTION_STATE["last_error"] = ""
    return {
        "motion": _odesc_motion_snapshot(),
        "axis": result,
    }


def _odesc_sensorless_keepalive(axis, velocity_turns_s):
    axis = int(axis)
    speed = abs(float(velocity_turns_s))
    with ODESC_MOTION_LOCK:
        active_axis = ODESC_MOTION_STATE["active_axis"]
        mode = ODESC_MOTION_STATE["mode"]
        commanded = ODESC_MOTION_STATE["velocity_turns_s"]
        minimum_speed = ODESC_MOTION_STATE["sensorless_min_turns_s"]
        started = ODESC_MOTION_STATE["startup_started_monotonic"]
        startup_duration = ODESC_MOTION_STATE["startup_duration_s"]
        command_sent = ODESC_MOTION_STATE["sensorless_command_sent"]
    if active_axis != axis or mode != "sensorless":
        raise RuntimeError("sensorless startup is not active for axis {}".format(axis))
    if speed < minimum_speed:
        raise ValueError(
            "sensorless speed must remain at or above {:.3f} turns/s".format(
                minimum_speed
            )
        )
    direction = 1 if commanded >= 0.0 else -1
    target = direction * speed
    startup_complete = time.monotonic() - started >= startup_duration
    if startup_complete and (not command_sent or target != commanded):
        ROBOT.set_odesc_sensorless_velocity(axis, target)
        command_sent = True
    with ODESC_MOTION_LOCK:
        ODESC_MOTION_STATE["velocity_turns_s"] = target
        ODESC_MOTION_STATE["sensorless_command_sent"] = command_sent
        ODESC_MOTION_STATE["last_keepalive_monotonic"] = time.monotonic()
        ODESC_MOTION_STATE["last_error"] = ""
    return _odesc_motion_snapshot()


def _odesc_motion_stop(axis=None, idle=False):
    link = ROBOT._require("odesc")
    with ODESC_MOTION_LOCK:
        active_axis = ODESC_MOTION_STATE["active_axis"]
        active_mode = ODESC_MOTION_STATE["mode"]
    axes = (int(axis),) if axis is not None else (0, 1)
    for selected in axes:
        if active_mode == "sensorless" and selected == active_axis:
            link.stop_sensorless(selected)
            ROBOT.odesc_armed = False
        else:
            link.stop_axis(selected)
        if idle and not (
            active_mode == "sensorless" and selected == active_axis
        ):
            link.request_state(selected, 1)
    with ODESC_MOTION_LOCK:
        if axis is None or active_axis == int(axis):
            ODESC_MOTION_STATE["active_axis"] = None
            ODESC_MOTION_STATE["mode"] = None
            ODESC_MOTION_STATE["velocity_turns_s"] = 0.0
            ODESC_MOTION_STATE["startup_started_monotonic"] = 0.0
            ODESC_MOTION_STATE["startup_duration_s"] = 0.0
            ODESC_MOTION_STATE["sensorless_min_turns_s"] = 0.0
            ODESC_MOTION_STATE["sensorless_command_sent"] = False
            ODESC_MOTION_STATE["last_keepalive_monotonic"] = 0.0
    return _odesc_motion_snapshot()


def _odesc_full_stop():
    ROBOT.stop_odesc()
    with ODESC_MOTION_LOCK:
        ODESC_MOTION_STATE["active_axis"] = None
        ODESC_MOTION_STATE["mode"] = None
        ODESC_MOTION_STATE["velocity_turns_s"] = 0.0
        ODESC_MOTION_STATE["startup_started_monotonic"] = 0.0
        ODESC_MOTION_STATE["startup_duration_s"] = 0.0
        ODESC_MOTION_STATE["sensorless_min_turns_s"] = 0.0
        ODESC_MOTION_STATE["sensorless_command_sent"] = False
        ODESC_MOTION_STATE["last_keepalive_monotonic"] = 0.0
    return _odesc_motion_snapshot()


def _odesc_motion_loop():
    while True:
        with ODESC_MOTION_LOCK:
            active_axis = ODESC_MOTION_STATE["active_axis"]
            last_keepalive = ODESC_MOTION_STATE["last_keepalive_monotonic"]
        if (
            active_axis is not None
            and last_keepalive
            and time.monotonic() - last_keepalive > ODESC_MOTION_DEADMAN_S
        ):
            event_id = None
            try:
                event_id = _odesc_event_start(
                    "ODESC motion deadman stop",
                    {"axis": active_axis},
                )
                with ACTION_LOCK:
                    result = _odesc_motion_stop(active_axis, idle=True)
                with ODESC_MOTION_LOCK:
                    ODESC_MOTION_STATE["deadman_stops"] += 1
                _odesc_event_finish(event_id, "success", result=result)
            except Exception as exc:
                if event_id is not None:
                    _odesc_event_finish(
                        event_id, "error", error=str(exc)
                    )
                with ODESC_MOTION_LOCK:
                    ODESC_MOTION_STATE["last_error"] = str(exc)
                    ODESC_MOTION_STATE["active_axis"] = None
                    ODESC_MOTION_STATE["mode"] = None
                    ODESC_MOTION_STATE["velocity_turns_s"] = 0.0
        time.sleep(0.05)


def _stepper_jog_snapshot():
    with STEPPER_JOG_LOCK:
        age_ms = (
            None
            if not STEPPER_JOG_STATE["last_keepalive_monotonic"]
            else round(
                (
                    time.monotonic()
                    - STEPPER_JOG_STATE["last_keepalive_monotonic"]
                )
                * 1000
            )
        )
        return {
            "axis": STEPPER_JOG_STATE["axis"],
            "direction": STEPPER_JOG_STATE["direction"],
            "chunk_steps": STEPPER_JOG_STATE["chunk_steps"],
            "rpm": STEPPER_JOG_STATE["rpm"],
            "keepalive_age_ms": age_ms,
            "last_error": STEPPER_JOG_STATE["last_error"],
        }


def _stepper_jog_start(axis, direction, chunk_steps, rpm):
    with STEPPER_JOG_LOCK:
        STEPPER_JOG_STATE["axis"] = axis
        STEPPER_JOG_STATE["direction"] = direction
        STEPPER_JOG_STATE["chunk_steps"] = chunk_steps
        STEPPER_JOG_STATE["rpm"] = rpm
        STEPPER_JOG_STATE["last_keepalive_monotonic"] = time.monotonic()
        STEPPER_JOG_STATE["last_error"] = ""
    return _stepper_jog_snapshot()


def _stepper_jog_stop():
    with STEPPER_JOG_LOCK:
        STEPPER_JOG_STATE["axis"] = None
        STEPPER_JOG_STATE["direction"] = 0
        STEPPER_JOG_STATE["last_keepalive_monotonic"] = 0.0
    return _stepper_jog_snapshot()


def _stepper_jog_loop():
    while True:
        with STEPPER_JOG_LOCK:
            axis = STEPPER_JOG_STATE["axis"]
            direction = STEPPER_JOG_STATE["direction"]
            chunk_steps = STEPPER_JOG_STATE["chunk_steps"]
            rpm = STEPPER_JOG_STATE["rpm"]
            last_keepalive = STEPPER_JOG_STATE["last_keepalive_monotonic"]
        if axis is not None:
            if time.monotonic() - last_keepalive > STEPPER_JOG_DEADMAN_S:
                _stepper_jog_stop()
            elif ROBOT.gd32 is not None:
                snapshot = ROBOT.gd32.link_snapshot()
                if snapshot["pending_moves"] == 0:
                    try:
                        with ACTION_LOCK:
                            ROBOT.step(
                                axis,
                                direction * chunk_steps,
                                rpm,
                                wait=False,
                            )
                    except Exception as exc:
                        _stepper_jog_stop()
                        with STEPPER_JOG_LOCK:
                            STEPPER_JOG_STATE["last_error"] = str(exc)
        time.sleep(STEPPER_JOG_INTERVAL_S)


def _stepper_set_limits(axis, minimum_steps, maximum_steps):
    result = ROBOT.set_stepper_limits(axis, minimum_steps, maximum_steps)
    saved = dict(CONFIG_DATA.get("stepper_limits", {}))
    saved[axis] = [minimum_steps, maximum_steps]
    _save_config_value("stepper_limits", saved)
    return result


def _restore_stepper_limits():
    if ROBOT.gd32 is None:
        return
    limits = CONFIG_DATA.get("stepper_limits", {})
    if not isinstance(limits, dict):
        return
    for axis in "XYZ":
        values = limits.get(axis)
        if isinstance(values, list) and len(values) == 2:
            ROBOT.set_stepper_limits(axis, int(values[0]), int(values[1]))


def _network_helper(command):
    arguments = (
        (NETWORK_HELPER, command)
        if command == "status"
        else ("sudo", "-n", NETWORK_HELPER, command)
    )
    result = subprocess.run(
        arguments,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=25,
    )
    output = result.stdout.strip()
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or output or "Wi-Fi helper failed")
    data = json.loads(output)
    if "error" in data:
        raise RuntimeError(data["error"])
    return data


def _network_snapshot():
    with NETWORK_STATE_LOCK:
        switch_at = NETWORK_STATE["switch_at_monotonic"]
        return {
            **dict(NETWORK_STATE["status"]),
            "scheduled_mode": NETWORK_STATE["scheduled_mode"],
            "switch_in_ms": (
                None
                if switch_at is None
                else max(0, round((switch_at - time.monotonic()) * 1000))
            ),
            "last_error": NETWORK_STATE["last_error"],
            "auto_failover_seconds": NETWORK_FAILOVER_SECONDS,
            "minimum_signal_percent": NETWORK_MIN_SIGNAL_PERCENT,
        }


def _schedule_network_mode(mode):
    if mode not in ("ap", "router"):
        raise ValueError("network mode must be ap or router")
    with NETWORK_STATE_LOCK:
        NETWORK_STATE["scheduled_mode"] = mode
        NETWORK_STATE["switch_at_monotonic"] = (
            time.monotonic() + NETWORK_SWITCH_DELAY_S
        )
        NETWORK_STATE["last_error"] = ""
        if mode == "router":
            NETWORK_STATE["router_grace_until"] = time.monotonic() + 45.0
    return _network_snapshot()


def _network_loop():
    while True:
        now = time.monotonic()
        with NETWORK_STATE_LOCK:
            scheduled_mode = NETWORK_STATE["scheduled_mode"]
            switch_at = NETWORK_STATE["switch_at_monotonic"]
        if scheduled_mode and switch_at is not None and now >= switch_at:
            try:
                status = _network_helper(scheduled_mode)
                with NETWORK_STATE_LOCK:
                    NETWORK_STATE["status"] = status
                    NETWORK_STATE["last_error"] = ""
            except Exception as exc:
                with NETWORK_STATE_LOCK:
                    NETWORK_STATE["last_error"] = str(exc)
            finally:
                with NETWORK_STATE_LOCK:
                    NETWORK_STATE["scheduled_mode"] = None
                    NETWORK_STATE["switch_at_monotonic"] = None

        try:
            status = _network_helper("status")
            with NETWORK_STATE_LOCK:
                NETWORK_STATE["status"] = status
                NETWORK_STATE["last_error"] = ""
                grace_until = NETWORK_STATE["router_grace_until"]
                already_scheduled = NETWORK_STATE["scheduled_mode"] is not None
                weak = (
                    status["mode"] == "disconnected"
                    or (
                        status["mode"] == "router"
                        and status.get("signal_percent") is not None
                        and status["signal_percent"] < NETWORK_MIN_SIGNAL_PERCENT
                    )
                )
                if status["mode"] == "ap" or not weak:
                    NETWORK_STATE["weak_since_monotonic"] = None
                elif now >= grace_until:
                    if NETWORK_STATE["weak_since_monotonic"] is None:
                        NETWORK_STATE["weak_since_monotonic"] = now
                    elif (
                        not already_scheduled
                        and now - NETWORK_STATE["weak_since_monotonic"]
                        >= NETWORK_FAILOVER_SECONDS
                    ):
                        NETWORK_STATE["scheduled_mode"] = "ap"
                        NETWORK_STATE["switch_at_monotonic"] = now
        except Exception as exc:
            with NETWORK_STATE_LOCK:
                NETWORK_STATE["last_error"] = str(exc)
        time.sleep(NETWORK_CHECK_INTERVAL_S)


def _status():
    links = ROBOT.link_status()
    with IMU_STATE_LOCK:
        if IMU_STATE["reading"] is not None:
            links["bno080"]["reading"] = dict(IMU_STATE["reading"])
        if IMU_STATE["error"]:
            links["bno080"]["reading_error"] = IMU_STATE["error"]
    with ODESC_STATE_LOCK:
        refreshed = ODESC_STATE["last_refresh_monotonic"]
        links["odesc"]["voltage_age_ms"] = (
            None
            if refreshed is None
            else round((time.monotonic() - refreshed) * 1000)
        )
        if ODESC_STATE["error"]:
            links["odesc"]["voltage_error"] = ODESC_STATE["error"]
        links["odesc"]["energy_used_wh"] = ODESC_STATE["energy_used_wh"]
    links["odesc"]["motion"] = _odesc_motion_snapshot()
    return {
        "ok": True,
        "links": links,
        "server": {
            "uptime_s": round(time.monotonic() - STARTED_AT),
            "connecting": SERVER_STATE["connecting"],
            "last_action": SERVER_STATE["last_action"],
            "last_error": SERVER_STATE["last_error"],
        },
        "tank": _tank_snapshot(),
        "stepper_jog": _stepper_jog_snapshot(),
        "network": _network_snapshot(),
        "pi": {
            "uptime_s": round(_pi_uptime_s()),
            "temperature_c": _pi_temperature_c(),
        },
    }


def _run_action(name, function, lock=True):
    try:
        if lock:
            with ACTION_LOCK:
                result = function()
        else:
            result = function()
        SERVER_STATE["last_action"] = name
        SERVER_STATE["last_error"] = ""
        return {"ok": True, "result": result}
    except Exception as exc:
        SERVER_STATE["last_error"] = "{}: {}".format(name, exc)
        raise


def _run_odesc_action(
    name,
    function,
    request=None,
    lock=True,
    log_success=True,
):
    event_id = (
        _odesc_event_start(name, request) if log_success else None
    )
    try:
        response = _run_action(name, function, lock=lock)
        if event_id is not None:
            _odesc_event_finish(
                event_id,
                "success",
                result=response.get("result"),
            )
        return response
    except Exception as exc:
        if event_id is None:
            event_id = _odesc_event_start(name, request)
        _odesc_event_finish(event_id, "error", error=str(exc))
        raise


def _dispatch(path, data):
    if path == "/api/connect-all":
        _tank_reset_state("reconnecting")
        return _run_action("connect all", _connect_all)
    if path == "/api/stop-all":
        return _run_action("STOP ALL", _stop_all, lock=False)
    if path == "/api/trip/start":
        return _run_action("start IMU trip", _trip_start, lock=False)
    if path == "/api/trip/stop":
        return _run_action("stop IMU trip", _trip_stop, lock=False)
    if path == "/api/trip/offset":
        offset = _number(data, "heading_offset_deg", -180, 180)
        return _run_action(
            "set IMU heading offset",
            lambda: _trip_set_heading_offset(offset),
            lock=False,
        )

    if path == "/api/tank/config":
        rate = _number(data, "ramp_percent_s", 10, 300)
        return _run_action("tank ramp config", lambda: _tank_set_ramp(rate))
    if path == "/api/tank/target":
        left = _number(data, "left", -100, 100)
        right = _number(data, "right", -100, 100)
        source = str(data.get("source", "remote"))
        return _run_action(
            "tank target",
            lambda: _tank_set_target(left, right, source),
            lock=False,
        )
    if path == "/api/tank/stop":
        return _run_action("tank stop", _tank_stop, lock=False)

    if path in ("/api/stm32/drive", "/api/stm32/pulse"):
        motor = _number(data, "motor", 1, 2, integer=True)
        direction = str(data.get("direction", "")).upper()
        if direction not in ("A", "B"):
            raise ValueError("direction must be A or B")
        percent = _number(data, "percent", 0, 100)
        _tank_stop()
        if path.endswith("/pulse"):
            seconds = _number(data, "seconds", 0.05, 5)
            return _run_action(
                "STM32 motor pulse",
                lambda: ROBOT.pulse_motor(motor, direction, percent, seconds),
                lock=False,
            )
        return _run_action(
            "STM32 motor drive",
            lambda: ROBOT.drive_motor(motor, direction, percent),
        )
    if path == "/api/stm32/stop":
        _tank_reset_state("manual motor stop")
        motor = _number(data, "motor", 1, 2, integer=True)
        return _run_action("STM32 motor stop", lambda: ROBOT.stop_motor(motor), False)
    if path == "/api/stm32/currents":
        return _run_action("STM32 current refresh", ROBOT.currents)
    if path == "/api/stm32/encoders/read":
        return _run_action("STM32 encoder read", ROBOT.read_encoders)
    if path == "/api/stm32/encoders/start":
        rate = _number(data, "rate_hz", 10, 100, integer=True)
        return _run_action("STM32 encoder start", lambda: ROBOT.start_encoders(rate))
    if path == "/api/stm32/encoders/stop":
        return _run_action("STM32 encoder stop", ROBOT.stop_encoders)

    if path == "/api/gd32/move":
        axis = str(data.get("axis", "")).upper()
        if axis not in ("X", "Y", "Z"):
            raise ValueError("axis must be X, Y, or Z")
        steps = _number(data, "steps", -1000000, 1000000, integer=True)
        rpm = _number(data, "rpm", 1, 1000)
        return _run_action(
            "GD32 move", lambda: ROBOT.step(axis, steps, rpm, wait=False)
        )
    if path == "/api/gd32/reset-zero":
        axis = str(data.get("axis", "")).upper()
        if axis not in ("X", "Y", "Z"):
            raise ValueError("axis must be X, Y, or Z")
        return _run_action(
            "GD32 reset zero", lambda: ROBOT.reset_stepper_position(axis)
        )
    if path == "/api/gd32/limits":
        axis = str(data.get("axis", "")).upper()
        if axis not in ("X", "Y", "Z"):
            raise ValueError("axis must be X, Y, or Z")
        minimum_steps = _number(
            data, "minimum_steps", -10000000, 10000000, integer=True
        )
        maximum_steps = _number(
            data, "maximum_steps", -10000000, 10000000, integer=True
        )
        return _run_action(
            "GD32 limits",
            lambda: _stepper_set_limits(axis, minimum_steps, maximum_steps),
        )
    if path == "/api/gd32/jog":
        axis = str(data.get("axis", "")).upper()
        if axis not in ("X", "Y", "Z"):
            raise ValueError("axis must be X, Y, or Z")
        direction = _number(data, "direction", -1, 1, integer=True)
        if direction == 0:
            raise ValueError("direction must be -1 or 1")
        chunk_steps = _number(
            data, "chunk_steps", 1, 10000, integer=True
        )
        rpm = _number(data, "rpm", 1, 1000)
        return _run_action(
            "GD32 jog",
            lambda: _stepper_jog_start(
                axis, direction, chunk_steps, rpm
            ),
            lock=False,
        )
    if path == "/api/gd32/jog-stop":
        return _run_action("GD32 jog stop", _stepper_jog_stop, lock=False)
    if path == "/api/gd32/stop":
        _stepper_jog_stop()
        return _run_action("GD32 stop", ROBOT.stop_steppers, False)
    if path == "/api/gd32/switches":
        return _run_action("GD32 switches", ROBOT.switches)

    if path == "/api/odesc/status":
        return _run_odesc_action(
            "ODESC status",
            _refresh_odesc_status,
            data,
        )
    if path == "/api/odesc/energy-reset":
        def reset_energy():
            with ODESC_STATE_LOCK:
                ODESC_STATE["energy_used_wh"] = 0.0
                ODESC_STATE["last_power_monotonic"] = time.monotonic()
            return {"energy_used_wh": 0.0}

        return _run_odesc_action(
            "ODESC energy reset", reset_energy, data, lock=False
        )
    if path == "/api/odesc/stop":
        return _run_odesc_action(
            "ODESC stop", _odesc_full_stop, data, lock=False
        )
    if path == "/api/odesc/configure":
        axis = _number(data, "axis", 0, 1, integer=True)
        current_limit = _number(data, "current_limit_a", 0.5, 30)
        velocity_limit = _number(
            data, "velocity_limit_turns_s", 0.1, 50
        )
        return _run_odesc_action(
            "ODESC configure velocity",
            lambda: ROBOT.configure_odesc_axis(
                axis, current_limit, velocity_limit
            ),
            data,
        )
    if path == "/api/odesc/sensorless/configure":
        axis = _number(data, "axis", 0, 1, integer=True)
        current_limit = _number(data, "current_limit_a", 0.5, 30)
        startup_current = _number(data, "startup_current_a", 0.5, 30)
        startup_velocity = _number(
            data, "startup_velocity_turns_s", 1, 30
        )
        startup_accel = _number(
            data, "startup_accel_turns_s2", 0.1, 30
        )
        velocity_limit = _number(
            data, "velocity_limit_turns_s", 1, 50
        )
        return _run_odesc_action(
            "ODESC configure legacy sensorless",
            lambda: ROBOT.configure_odesc_sensorless_axis(
                axis,
                current_limit,
                startup_current,
                startup_velocity,
                startup_accel,
                velocity_limit,
            ),
            data,
        )
    if path == "/api/odesc/clear-errors":
        axis = _number(data, "axis", 0, 1, integer=True)
        return _run_odesc_action(
            "ODESC clear errors",
            lambda: ROBOT.clear_odesc_errors(axis),
            data,
        )
    if path == "/api/odesc/calibrate":
        axis = _number(data, "axis", 0, 1, integer=True)
        phrase = str(data.get("phrase", ""))
        return _run_odesc_action(
            "ODESC calibration",
            lambda: ROBOT.calibrate_odesc_axis(axis, phrase),
            data,
        )
    if path == "/api/odesc/sensorless/calibrate-motor":
        axis = _number(data, "axis", 0, 1, integer=True)
        phrase = str(data.get("phrase", ""))
        return _run_odesc_action(
            "ODESC sensorless motor calibration",
            lambda: ROBOT.calibrate_odesc_motor(axis, phrase),
            data,
        )
    if path == "/api/odesc/arm":
        return _run_odesc_action(
            "ODESC arm",
            lambda: ROBOT.arm_odesc(str(data.get("phrase", ""))),
            data,
        )
    if path in ("/api/odesc/enable", "/api/odesc/disable"):
        axis = _number(data, "axis", 0, 1, integer=True)
        function = ROBOT.enable_odesc_axis if path.endswith("/enable") else ROBOT.disable_odesc_axis
        return _run_odesc_action(
            "ODESC axis state",
            lambda: function(axis),
            data,
        )
    if path == "/api/odesc/sensorless/start":
        axis = _number(data, "axis", 0, 1, integer=True)
        direction = _number(data, "direction", -1, 1, integer=True)
        if direction == 0:
            raise ValueError("direction must be -1 or 1")
        velocity = _number(data, "turns_per_second", 1, 15)
        return _run_odesc_action(
            "ODESC sensorless start",
            lambda: _odesc_sensorless_start(axis, direction, velocity),
            data,
        )
    if path == "/api/odesc/sensorless/keepalive":
        axis = _number(data, "axis", 0, 1, integer=True)
        velocity = _number(data, "turns_per_second", 1, 15)
        return _run_odesc_action(
            "ODESC sensorless velocity",
            lambda: _odesc_sensorless_keepalive(axis, velocity),
            data,
            log_success=False,
        )
    if path == "/api/odesc/sensorless/stop":
        axis = _number(data, "axis", 0, 1, integer=True)
        return _run_odesc_action(
            "ODESC sensorless stop",
            lambda: _odesc_motion_stop(axis, idle=True),
            data,
        )
    if path == "/api/odesc/velocity":
        axis = _number(data, "axis", 0, 1, integer=True)
        velocity = _number(data, "turns_per_second", -15, 15)
        return _run_odesc_action(
            "ODESC velocity",
            lambda: _odesc_velocity(axis, velocity),
            data,
            log_success=False,
        )
    if path == "/api/odesc/velocity-stop":
        axis = _number(data, "axis", 0, 1, integer=True)
        return _run_odesc_action(
            "ODESC velocity stop",
            lambda: _odesc_motion_stop(axis),
            data,
        )
    if path == "/api/odesc/pulse":
        axis = _number(data, "axis", 0, 1, integer=True)
        speed = _number(data, "turns_per_second", -2, 2)
        seconds = _number(data, "seconds", 0.05, 5)
        return _run_odesc_action(
            "ODESC pulse",
            lambda: ROBOT.pulse_odesc(axis, speed, seconds),
            data,
            lock=False,
        )
    if path == "/api/network/ap":
        return _run_action(
            "schedule AP mode",
            lambda: _schedule_network_mode("ap"),
            lock=False,
        )
    if path == "/api/network/router":
        return _run_action(
            "schedule router mode",
            lambda: _schedule_network_mode("router"),
            lock=False,
        )
    if path == "/api/network/cancel":
        with NETWORK_STATE_LOCK:
            NETWORK_STATE["scheduled_mode"] = None
            NETWORK_STATE["switch_at_monotonic"] = None
        return {"ok": True, "result": _network_snapshot()}
    raise ValueError("unknown API route")


class RoverHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class Handler(BaseHTTPRequestHandler):
    server_version = "RoverControl/1.0"

    def log_message(self, fmt, *args):
        print("[web] {} {}".format(self.address_string(), fmt % args))

    def _authorized(self):
        if not AUTH_REQUIRED:
            return True
        if not AUTH_PASSWORD:
            return False
        header = self.headers.get("Authorization", "")
        if not header.startswith("Basic "):
            return False
        try:
            supplied = base64.b64decode(header[6:], validate=True).decode("utf-8")
        except Exception:
            return False
        expected = "{}:{}".format(AUTH_USER, AUTH_PASSWORD)
        return hmac.compare_digest(supplied, expected)

    def _send(self, status, body, content_type):
        encoded = body.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(encoded)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Referrer-Policy", "no-referrer")
        self.end_headers()
        self.wfile.write(encoded)

    def _json(self, status, value):
        self._send(status, json.dumps(value, separators=(",", ":"), default=str), "application/json")

    def _require_auth(self):
        if self._authorized():
            return True
        self.send_response(401)
        self.send_header("WWW-Authenticate", 'Basic realm="Garden Rover"')
        self.send_header("Content-Length", "0")
        self.end_headers()
        return False

    def do_GET(self):
        if not self._require_auth():
            return
        parsed = urlparse(self.path)
        path = parsed.path
        try:
            if path == "/":
                self._send(200, INDEX_HTML, "text/html; charset=utf-8")
            elif path == "/mobile":
                self._send(200, MOBILE_HTML, "text/html; charset=utf-8")
            elif path == "/sensors":
                self._send(200, SENSORS_HTML, "text/html; charset=utf-8")
            elif path in ("/battery", "/batter"):
                self._send(200, BATTERY_HTML, "text/html; charset=utf-8")
            elif path == "/api/status":
                self._json(200, _status())
            elif path == "/api/sensors":
                self._json(200, _sensors_payload())
            elif path == "/api/battery":
                query = parse_qs(parsed.query)
                value = query.get("session_id", [None])[0]
                session_id = None if value in (None, "") else int(value)
                self._json(200, _battery_history(session_id))
            elif path == "/api/odesc/events":
                query = parse_qs(parsed.query)
                value = query.get("limit", [ODESC_EVENT_LIMIT])[0]
                self._json(200, _odesc_events(int(value)))
            else:
                self._json(404, {"ok": False, "error": "not found"})
        except (ValueError, TypeError) as exc:
            self._json(400, {"ok": False, "error": str(exc)})
        except Exception as exc:
            self._json(503, {"ok": False, "error": str(exc)})

    def do_POST(self):
        if not self._require_auth():
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length < 0 or length > MAX_BODY_BYTES:
                raise ValueError("request body too large")
            raw = self.rfile.read(length) if length else b"{}"
            data = json.loads(raw.decode("utf-8"))
            if not isinstance(data, dict):
                raise ValueError("JSON body must be an object")
            result = _dispatch(urlparse(self.path).path, data)
            self._json(200, result)
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self._json(400, {"ok": False, "error": str(exc)})
        except Exception as exc:
            self._json(503, {"ok": False, "error": str(exc)})


def _connect_background():
    SERVER_STATE["connecting"] = True
    try:
        _connect_all()
        SERVER_STATE["last_action"] = "startup UART connection complete"
    except Exception as exc:
        SERVER_STATE["last_error"] = "startup connection: {}".format(exc)
    finally:
        SERVER_STATE["connecting"] = False


def _connect_all():
    with IMU_IO_LOCK:
        result = ROBOT.connect_all()
    _restore_stepper_limits()
    return result


def _imu_loop():
    while True:
        if ROBOT.bno080 is None:
            time.sleep(0.2)
            continue
        try:
            with IMU_IO_LOCK:
                reading = ROBOT.bno080.snapshot(0.2)
            with IMU_STATE_LOCK:
                IMU_STATE["reading"] = reading
                IMU_STATE["error"] = ""
            _trip_update(reading)
        except Exception as exc:
            with IMU_STATE_LOCK:
                IMU_STATE["error"] = str(exc)
            time.sleep(0.1)


def _refresh_odesc_status():
    snapshot = ROBOT.odesc_status()
    now = time.monotonic()
    with ODESC_STATE_LOCK:
        previous = ODESC_STATE["last_power_monotonic"]
        if previous is not None:
            elapsed = min(15.0, max(0.0, now - previous))
            ODESC_STATE["energy_used_wh"] += (
                max(0.0, float(snapshot.get("bus_power_w", 0.0)))
                * elapsed
                / 3600.0
            )
        ODESC_STATE["last_power_monotonic"] = now
        ODESC_STATE["last_refresh_monotonic"] = now
        ODESC_STATE["error"] = ""
    _battery_record(snapshot)
    return snapshot


def _odesc_loop():
    while True:
        if ROBOT.odesc is None:
            time.sleep(0.5)
            continue
        try:
            with ACTION_LOCK:
                _refresh_odesc_status()
        except Exception as exc:
            with ODESC_STATE_LOCK:
                ODESC_STATE["error"] = str(exc)
        time.sleep(ODESC_STATUS_INTERVAL_S)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true", help="validate configuration and exit")
    args = parser.parse_args()
    if AUTH_REQUIRED and not AUTH_PASSWORD:
        raise SystemExit("Set ROVER_WEB_PASSWORD before starting the server")
    if args.check:
        print("Rover web configuration valid on {}:{}".format(HOST, PORT))
        return
    _battery_init()
    threading.Thread(target=_connect_background, name="uart-connect", daemon=True).start()
    threading.Thread(target=_imu_loop, name="imu-reader", daemon=True).start()
    threading.Thread(target=_odesc_loop, name="odesc-status", daemon=True).start()
    threading.Thread(
        target=_odesc_motion_loop, name="odesc-deadman", daemon=True
    ).start()
    threading.Thread(
        target=_stepper_jog_loop, name="stepper-jog", daemon=True
    ).start()
    threading.Thread(
        target=_network_loop, name="network-mode", daemon=True
    ).start()
    threading.Thread(target=_tank_loop, name="tank-drive", daemon=True).start()
    server = RoverHTTPServer((HOST, PORT), Handler)
    print("Rover dashboard listening on http://{}:{}/".format(HOST, PORT))
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        ROBOT.close()


if __name__ == "__main__":
    main()
