#!/usr/bin/env python3
"""Read all 16 ADC1 channels from the read-only ODESC diagnostic image."""

import argparse
import csv
import os
import statistics
import sys
import time

import serial
from serial.tools import list_ports


SAFE_MASK = 0xFFF


def find_port(explicit_port):
    if explicit_port:
        return explicit_port

    matches = []
    for port in list_ports.comports():
        description = "{} {}".format(port.description or "", port.manufacturer or "").lower()
        if port.vid == 0x1209 or "odrive" in description:
            matches.append(port.device)
    if len(matches) == 1:
        return matches[0]
    if not matches:
        raise RuntimeError("No ODESC/ODrive USB CDC port found; pass --port /dev/ttyACM0")
    raise RuntimeError("Multiple candidate ports found: {}; pass --port".format(", ".join(matches)))


def command(device, text, timeout=1.0):
    device.reset_input_buffer()
    device.write((text + "\n").encode("ascii"))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = device.readline().decode("ascii", "replace").strip()
        if line:
            return line
    raise RuntimeError("No response to {!r}".format(text))


def require_safe(device):
    reply = command(device, "astatus")
    fields = reply.split()
    if len(fields) != 4 or fields[0] != "SAFE" or fields[2] != "faults":
        raise RuntimeError("Unexpected safety response: {}".format(reply))
    status = int(fields[1], 16)
    faults = int(fields[3])
    if status != SAFE_MASK or faults != 0:
        raise RuntimeError(
            "Diagnostic safety interlock is not clean: status=0x{:03x}, faults={}".format(
                status, faults
            )
        )
    return reply


def read_channel(device, channel):
    reply = command(device, "a {}".format(channel))
    fields = reply.split()
    if len(fields) != 4 or fields[0] != "ADC" or int(fields[1]) != channel:
        raise RuntimeError("Unexpected ADC response: {}".format(reply))
    return int(fields[2]), int(fields[3])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", help="USB CDC port, normally /dev/ttyACM0")
    parser.add_argument("--samples", type=int, default=25, help="samples per channel")
    parser.add_argument("--label", default="", help="known supply voltage or run label")
    parser.add_argument("--csv", help="append channel averages to this CSV file")
    args = parser.parse_args()

    if args.samples < 3 or args.samples > 1000:
        parser.error("--samples must be between 3 and 1000")

    port = find_port(args.port)
    rows = []
    with serial.Serial(port, 115200, timeout=0.25, write_timeout=0.5) as device:
        time.sleep(0.25)
        print("{} on {}".format(require_safe(device), port))
        print("channel  raw_avg  raw_min  raw_max  millivolts")
        for channel in range(16):
            raw_values = []
            mv_values = []
            for _ in range(args.samples):
                raw, millivolts = read_channel(device, channel)
                raw_values.append(raw)
                mv_values.append(millivolts)
                time.sleep(0.003)
            row = {
                "label": args.label,
                "channel": channel,
                "raw_avg": round(statistics.fmean(raw_values), 2),
                "raw_min": min(raw_values),
                "raw_max": max(raw_values),
                "millivolts_avg": round(statistics.fmean(mv_values), 2),
            }
            rows.append(row)
            print(
                "{:>7}  {:>7.2f}  {:>7}  {:>7}  {:>10.2f}".format(
                    channel,
                    row["raw_avg"],
                    row["raw_min"],
                    row["raw_max"],
                    row["millivolts_avg"],
                )
            )
        print(require_safe(device))

    if args.csv:
        fieldnames = ["label", "channel", "raw_avg", "raw_min", "raw_max", "millivolts_avg"]
        needs_header = not os.path.exists(args.csv) or os.path.getsize(args.csv) == 0
        with open(args.csv, "a", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=fieldnames)
            if needs_header:
                writer.writeheader()
            writer.writerows(rows)
        print("Appended {} rows to {}".format(len(rows), args.csv))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("ERROR: {}".format(exc), file=sys.stderr)
        raise SystemExit(1)
