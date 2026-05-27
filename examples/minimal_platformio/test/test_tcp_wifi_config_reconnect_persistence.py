#!/usr/bin/env python3
"""Scripted test for Wi-Fi/TCP config, reconnect on server close, and persistence after reboot."""

import time

import tcp_test_config as cfg
from harp_tcp_test_utils import (
    REG_WHO_AM_I,
    accept_harp_connection,
    configure_network_over_usb,
    reboot_device_over_usb,
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

    # 1) Configure and accept first TCP session.
    conn1, peer1 = accept_harp_connection(
        listen_ip=cfg.LISTEN_IP,
        listen_port=cfg.LISTEN_PORT,
        accept_timeout=cfg.ACCEPT_TIMEOUT_S,
        setup_cb=_setup_usb,
    )
    with conn1 as client:
        who = client.read_reg(REG_WHO_AM_I)
        if who.is_error:
            raise AssertionError("WHO_AM_I read failed on first session")
        first_who = who.payload_u16()
        print(f"First TCP session OK from {peer1}, WHO_AM_I=0x{first_who:04X}")

    # 2) Server-side close and wait for automatic reconnect.
    print("Closed server-side socket; waiting for automatic reconnect...")
    conn2, peer2 = accept_harp_connection(
        listen_ip=cfg.LISTEN_IP,
        listen_port=cfg.LISTEN_PORT,
        accept_timeout=cfg.ACCEPT_TIMEOUT_S,
        setup_cb=None,
    )
    with conn2 as client:
        who2 = client.read_reg(REG_WHO_AM_I)
        if who2.is_error:
            raise AssertionError("WHO_AM_I read failed after reconnect")
        second_who = who2.payload_u16()
        if second_who != first_who:
            raise AssertionError("WHO_AM_I mismatch after reconnect")
        print(f"Reconnect session OK from {peer2}, WHO_AM_I=0x{second_who:04X}")

    # 3) Reboot and expect reconnect without re-configuring (NVS persistence).
    reboot_device_over_usb(cfg.USB_PORT)
    time.sleep(2.0)
    print("Waiting for reconnect after reboot (persistence check)...")
    conn3, peer3 = accept_harp_connection(
        listen_ip=cfg.LISTEN_IP,
        listen_port=cfg.LISTEN_PORT,
        accept_timeout=cfg.REBOOT_TO_RECONNECT_TIMEOUT_S,
        setup_cb=None,
    )
    with conn3 as client:
        who3 = client.read_reg(REG_WHO_AM_I)
        if who3.is_error:
            raise AssertionError("WHO_AM_I read failed after reboot reconnect")
        third_who = who3.payload_u16()
        if third_who != first_who:
            raise AssertionError("WHO_AM_I mismatch after reboot reconnect")
        print(f"Persistence reconnect OK from {peer3}, WHO_AM_I=0x{third_who:04X}")

    result = {
        "first_peer": peer1,
        "reconnect_peer": peer2,
        "post_reboot_peer": peer3,
        "who_am_i": first_who,
    }
    print("Wi-Fi/TCP config + reconnect + persistence test PASS")
    print(result)
    return result


if __name__ == "__main__":
    run_test()
