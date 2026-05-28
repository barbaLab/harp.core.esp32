#!/usr/bin/env python3
"""Scripted host-side config for TCP integration tests."""

import os


def default_usb_port() -> str:
    return "/dev/ttyACM0" if os.name == "posix" else "COM5"


LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 9999

# Use the host LAN IP that the ESP should connect back to.
SERVER_IP = "192.168.178.36"
SERVER_PORT = 9999

USB_PORT = default_usb_port()
WIFI_SSID = "NoTu"
WIFI_PASSWORD = "GrannyTranny420!"

POST_USB_WAIT_S = 2.0
ACCEPT_TIMEOUT_S = 25.0

RTT_REQUESTS = 10000
HEARTBEAT_COUNT = 5
HEARTBEAT_TIMEOUT_S = 4.0

# Persistence test restarts device and expects auto reconnect from NVS.
REBOOT_TO_RECONNECT_TIMEOUT_S = 35.0


def assert_configured() -> None:
    if WIFI_SSID.startswith("CHANGE_ME") or WIFI_PASSWORD.startswith("CHANGE_ME"):
        raise RuntimeError(
            "Update WIFI_SSID and WIFI_PASSWORD in tcp_test_config.py before running TCP tests."
        )
