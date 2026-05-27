#!/usr/bin/env python3
"""TCP heartbeat test."""

import tcp_test_config as cfg
from harp_tcp_test_utils import (
    ALIVE_EN_BIT,
    MSG_EVENT,
    OP_MODE_ACTIVE,
    OP_MODE_STANDBY,
    REG_OPERATION_CTRL,
    REG_TIMESTAMP_SECOND,
    accept_harp_connection,
    configure_network_over_usb,
    recv_reply,
)


def run_test() -> list[float]:
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

    intervals = []
    with connection as client:
        ctrl = OP_MODE_ACTIVE | (1 << ALIVE_EN_BIT)
        reply = client.write_u8(REG_OPERATION_CTRL, ctrl)
        if reply.is_error:
            raise AssertionError("Failed to set OPERATION_CTRL for heartbeat")

        client.drain(0.1)

        timestamps_us = []
        for _ in range(cfg.HEARTBEAT_COUNT):
            msg = recv_reply(client.sock, timeout=cfg.HEARTBEAT_TIMEOUT_S)
            if msg.msg_type != MSG_EVENT:
                raise AssertionError(f"Expected EVENT, got 0x{msg.msg_type:02X}")
            if msg.address != REG_TIMESTAMP_SECOND:
                raise AssertionError(f"Expected heartbeat at reg {REG_TIMESTAMP_SECOND}, got {msg.address}")
            if msg.ts_us is None:
                raise AssertionError("Expected timestamp in heartbeat")
            timestamps_us.append(msg.ts_us)

        intervals = [
            (timestamps_us[i + 1] - timestamps_us[i]) / 1e6
            for i in range(len(timestamps_us) - 1)
        ]
        for iv in intervals:
            if not (0.8 <= iv <= 1.5):
                raise AssertionError(f"Heartbeat interval out of range: {iv:.3f}s")

        client.write_u8(REG_OPERATION_CTRL, OP_MODE_STANDBY)
        client.drain(0.1)

    print("Heartbeat test PASS")
    print({"intervals_s": intervals})
    return intervals


if __name__ == "__main__":
    run_test()
