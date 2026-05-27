#!/usr/bin/env python3
"""TCP RTT test with scripted parameters (no CLI args)."""

import statistics
import time

import tcp_test_config as cfg
from harp_tcp_test_utils import (
    REG_WHO_AM_I,
    accept_harp_connection,
    configure_network_over_usb,
    build_read,
    recv_reply,
)


def run_test() -> dict:
    cfg.assert_configured()

    def _setup_usb() -> None:
        configure_network_over_usb(
            usb_port=cfg.USB_PORT,
            ssid=cfg.WIFI_SSID,
            password=cfg.WIFI_PASSWORD,
            server_ip=cfg.SERVER_IP,
            server_port=cfg.SERVER_PORT,
            post_usb_wait=cfg.POST_USB_WAIT_S,
        )

    connection, _peer = accept_harp_connection(
        listen_ip=cfg.LISTEN_IP,
        listen_port=cfg.LISTEN_PORT,
        accept_timeout=cfg.ACCEPT_TIMEOUT_S,
        setup_cb=_setup_usb,
    )

    with connection as client:
        latencies_ms = []
        errors = 0
        for _ in range(cfg.RTT_REQUESTS):
            t0 = time.perf_counter()
            client.sock.sendall(build_read(REG_WHO_AM_I))
            try:
                reply = recv_reply(client.sock, timeout=2.0)
                t1 = time.perf_counter()
                if reply.is_error:
                    errors += 1
                else:
                    latencies_ms.append((t1 - t0) * 1000.0)
            except Exception:
                errors += 1

    if errors != 0:
        raise AssertionError(f"RTT test had {errors} errors out of {cfg.RTT_REQUESTS}")
    if len(latencies_ms) < int(cfg.RTT_REQUESTS * 0.95):
        raise AssertionError("Too many missing replies in RTT test")

    stats = {
        "n": len(latencies_ms),
        "min_ms": min(latencies_ms),
        "mean_ms": statistics.mean(latencies_ms),
        "median_ms": statistics.median(latencies_ms),
        "max_ms": max(latencies_ms),
    }
    print("RTT test PASS")
    print(stats)
    return stats


if __name__ == "__main__":
    run_test()
