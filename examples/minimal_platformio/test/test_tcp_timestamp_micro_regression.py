#!/usr/bin/env python3
"""Regression test for TIMESTAMP_MICRO read-only behavior."""

import tcp_test_config as cfg
from harp_tcp_test_utils import (
    REG_TIMESTAMP_MICRO,
    REG_TIMESTAMP_SECOND,
    accept_harp_connection,
    configure_network_over_usb,
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

    set_seconds = 3_000_000_000
    set_micro_raw = 10_000  # 10_000 << 5 = 320_000 us

    with connection as client:
        write_sec = client.write_u32(REG_TIMESTAMP_SECOND, set_seconds)
        if write_sec.is_error:
            raise AssertionError("Failed to write TIMESTAMP_SECOND")

        write_micro = client.write_u16(REG_TIMESTAMP_MICRO, set_micro_raw)
        if not write_micro.is_error:
            raise AssertionError("TIMESTAMP_MICRO write should return WRITE_ERROR (read-only)")

        read_sec = client.read_reg(REG_TIMESTAMP_SECOND)
        if read_sec.is_error:
            raise AssertionError("Failed to read TIMESTAMP_SECOND")
        after_seconds = read_sec.payload_u32()

        read_micro = client.read_reg(REG_TIMESTAMP_MICRO)
        if read_micro.is_error:
            raise AssertionError("Failed to read TIMESTAMP_MICRO")
        after_micro_raw = read_micro.payload_u16()

    if after_seconds < 1_000_000_000:
        raise AssertionError(
            f"TIMESTAMP_SECOND collapsed unexpectedly after rejected TIMESTAMP_MICRO write: {after_seconds}"
        )

    if not (0 <= after_micro_raw <= 31_249):
        raise AssertionError(f"TIMESTAMP_MICRO out of valid raw range: {after_micro_raw}")

    result = {
        "set_seconds": set_seconds,
        "after_seconds": after_seconds,
        "set_micro_raw": set_micro_raw,
        "after_micro_raw": after_micro_raw,
    }
    print("Timestamp micro read-only regression test PASS")
    print(result)
    return result


if __name__ == "__main__":
    run_test()
