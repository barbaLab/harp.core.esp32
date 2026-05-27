#!/usr/bin/env python3
"""Runs all scripted TCP tests (split modules)."""

from test_tcp_wifi_config_reconnect_persistence import run_test as run_wifi_reconnect_persistence
from test_tcp_heartbeat import run_test as run_heartbeat
from test_tcp_rtt import run_test as run_rtt


def run_all() -> dict:
    results = {
        "wifi_reconnect_persistence": run_wifi_reconnect_persistence(),
        "heartbeat": run_heartbeat(),
        "rtt": run_rtt(),
    }
    print("All scripted TCP tests PASS")
    return results


if __name__ == "__main__":
    run_all()
