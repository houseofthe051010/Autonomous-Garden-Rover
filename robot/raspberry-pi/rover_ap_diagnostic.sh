#!/bin/bash
set -u

LOG_PATH="${1:-/home/aditya/rover-controller/ap-diagnostic.log}"
HELPER="/usr/local/sbin/rover-wifi-mode"

exec >"$LOG_PATH" 2>&1

return_to_router() {
    echo "RETURN_TO_ROUTER"
    "$HELPER" router
    date --iso-8601=seconds
}

trap return_to_router EXIT
trap 'exit 1' INT TERM

echo "START_AP_DIAGNOSTIC"
date --iso-8601=seconds
sleep 3
"$HELPER" ap
sleep 8

echo "AP_STATUS"
"$HELPER" status

echo "WLAN_ADDRESS"
ip -4 address show dev wlan0

echo "DHCP_DNS_LISTENERS"
ss -ulpn

echo "LOCAL_WEB_REQUEST"
curl --fail --silent --show-error --max-time 5 \
    http://10.42.0.1:8080/api/status
echo

echo "NETWORKMANAGER_DEVICE"
nmcli -f GENERAL.STATE,GENERAL.CONNECTION,IP4.ADDRESS device show wlan0

sleep 8
echo "AP_DIAGNOSTIC_COMPLETE"
