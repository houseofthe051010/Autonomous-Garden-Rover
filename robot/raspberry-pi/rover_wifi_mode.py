#!/usr/bin/env python3
"""Restricted NetworkManager helper for rover router/AP mode switching."""

import json
import subprocess
import sys
import time


WIFI_INTERFACE = "wlan0"
ROUTER_CONNECTION = "netplan-wlan0-mojo"
AP_CONNECTION = "rover-ap"
AP_SSID = "robot"
AP_ADDRESS = "10.42.0.1/24"
AP_CHANNEL = "6"
AP_DHCP_RANGE = "10.42.0.10,10.42.0.200"
AP_DHCP_LEASE_SECONDS = "3600"


def run_nmcli(*arguments, check=True):
    result = subprocess.run(
        ("nmcli",) + tuple(arguments),
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=20,
    )
    if check and result.returncode:
        raise RuntimeError(
            "nmcli {} failed: {}".format(
                " ".join(arguments), result.stderr.strip() or result.stdout.strip()
            )
        )
    return result


def connection_exists(name):
    result = run_nmcli("-g", "NAME", "connection", "show", name, check=False)
    return result.returncode == 0


def status():
    result = run_nmcli(
        "-t",
        "-f",
        "GENERAL.STATE,GENERAL.CONNECTION,GENERAL.DEVICE",
        "device",
        "show",
        WIFI_INTERFACE,
    )
    values = {}
    for line in result.stdout.splitlines():
        key, _, value = line.partition(":")
        values[key] = value

    connection = values.get("GENERAL.CONNECTION", "")
    state_text = values.get("GENERAL.STATE", "")
    connected = state_text.startswith("100 ")
    mode = "ap" if connected and connection == AP_CONNECTION else (
        "router" if connected else "disconnected"
    )

    signal = None
    scan = run_nmcli(
        "-t",
        "-f",
        "IN-USE,SIGNAL",
        "device",
        "wifi",
        "list",
        "ifname",
        WIFI_INTERFACE,
        check=False,
    )
    for line in scan.stdout.splitlines():
        active, _, strength = line.partition(":")
        if active == "*":
            try:
                signal = int(strength)
            except ValueError:
                pass
            break

    addresses = run_nmcli(
        "-g", "IP4.ADDRESS", "device", "show", WIFI_INTERFACE, check=False
    ).stdout.splitlines()
    return {
        "mode": mode,
        "connected": connected,
        "connection": connection,
        "interface": WIFI_INTERFACE,
        "signal_percent": signal,
        "ipv4": addresses[0] if addresses else "",
        "ap_ssid": AP_SSID,
        "ap_url": "http://10.42.0.1:8080/",
        "router_connection": ROUTER_CONNECTION,
    }


def enable_ap():
    current = status()
    if current["mode"] == "ap":
        return current

    if connection_exists(AP_CONNECTION):
        run_nmcli("connection", "delete", AP_CONNECTION)
    run_nmcli(
        "connection",
        "add",
        "type",
        "wifi",
        "ifname",
        WIFI_INTERFACE,
        "con-name",
        AP_CONNECTION,
        "ssid",
        AP_SSID,
    )
    run_nmcli(
        "connection",
        "modify",
        AP_CONNECTION,
        "802-11-wireless.mode",
        "ap",
        "802-11-wireless.band",
        "bg",
        "802-11-wireless.channel",
        AP_CHANNEL,
        "802-11-wireless.cloned-mac-address",
        "permanent",
        "802-11-wireless.powersave",
        "2",
        "802-11-wireless.ap-isolation",
        "no",
        "ipv4.method",
        "shared",
        "ipv4.addresses",
        AP_ADDRESS,
        "ipv4.shared-dhcp-range",
        AP_DHCP_RANGE,
        "ipv4.shared-dhcp-lease-time",
        AP_DHCP_LEASE_SECONDS,
        "ipv6.method",
        "disabled",
        "connection.autoconnect",
        "no",
    )
    run_nmcli("connection", "up", AP_CONNECTION)
    time.sleep(1.0)
    current = status()
    if current["mode"] != "ap" or not current["ipv4"].startswith("10.42.0.1/"):
        enable_router()
        raise RuntimeError(
            "AP activation did not produce an active 10.42.0.1 interface"
        )
    return current


def enable_router():
    if connection_exists(AP_CONNECTION):
        run_nmcli("connection", "down", AP_CONNECTION, check=False)
    result = run_nmcli(
        "connection", "up", ROUTER_CONNECTION, check=False
    )
    if result.returncode:
        # Netplan-generated profiles can briefly disappear from `connection
        # show` while a Wi-Fi AP profile owns the interface. Ask NetworkManager
        # to reload its on-disk profiles, then retry the known fixed profile.
        run_nmcli("connection", "reload")
        run_nmcli("connection", "up", ROUTER_CONNECTION)
    time.sleep(1.0)
    current = status()
    if current["mode"] != "router":
        raise RuntimeError("router connection did not become active")
    return current


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ("status", "ap", "router"):
        raise SystemExit("usage: rover-wifi-mode {status|ap|router}")
    command = sys.argv[1]
    if command == "status":
        result = status()
    elif command == "ap":
        result = enable_ap()
    else:
        result = enable_router()
    print(json.dumps(result, separators=(",", ":")))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(json.dumps({"error": str(exc)}, separators=(",", ":")))
        raise SystemExit(1)
