#!/usr/bin/env python3
"""Regression test for transport RX isolation.

Asserts that a partial TCP frame in progress does not block a full USB command.
"""

import socket
import time

import tcp_test_config as cfg
from harp_tcp_test_utils import (
    REG_WHO_AM_I,
    accept_harp_connection,
    build_read,
    configure_network_over_usb,
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
        # Inject only part of a valid TCP READ frame and keep it incomplete.
        full_read = build_read(REG_WHO_AM_I)
        partial = full_read[:3]
        remainder = full_read[3:]
        client.sock.sendall(partial)

        # While TCP has an incomplete frame, issue a full USB read command.
        from harp.serial.device import Device

        t0 = time.perf_counter()
        dev = Device(cfg.USB_PORT, "ibl.bin")
        try:
            usb_reply = dev.read_u16(REG_WHO_AM_I)
            usb_who = int(usb_reply.payload)
        finally:
            try:
                dev.disconnect()
            except Exception:
                pass
        usb_latency_s = time.perf_counter() - t0

        if usb_who <= 0:
            raise AssertionError(f"Unexpected WHO_AM_I payload over USB: {usb_who}")

        if usb_latency_s > 2.5:
            raise AssertionError(
                f"USB command latency too high while TCP partial frame pending: {usb_latency_s:.3f}s"
            )

        # Complete the TCP frame and ensure it still parses correctly.
        client.sock.sendall(remainder)
        tcp_reply = recv_reply(client.sock, timeout=2.0)
        if tcp_reply.is_error:
            raise AssertionError("TCP WHO_AM_I reply returned error after completing partial frame")
        tcp_who = tcp_reply.payload_u16()

        if tcp_who != usb_who:
            raise AssertionError(
                f"WHO_AM_I mismatch between USB and TCP paths: usb={usb_who} tcp={tcp_who}"
            )

    result = {
        "who_am_i": usb_who,
        "usb_latency_s": usb_latency_s,
    }
    print("Transport isolation regression test PASS")
    print(result)
    return result


if __name__ == "__main__":
    run_test()
