#!/usr/bin/env python3
"""Shared helpers for host-side Harp TCP integration tests."""

# TODO: Replace this custom local TCP wire/parsing layer with upstream Harp
# library TCP integration once it is available. For now, tests intentionally
# use this local implementation to validate firmware TCP behavior end-to-end.

import socket
import struct
import time
from typing import Optional, Callable, Tuple


# Harp message constants
MSG_READ = 0x01
MSG_WRITE = 0x02
MSG_EVENT = 0x03

HAS_TIMESTAMP = 0x10

TYPE_U8 = 0x01
TYPE_U16 = 0x02
TYPE_U32 = 0x04
PORT_DEFAULT = 255


# Core register addresses
REG_WHO_AM_I = 0
REG_TIMESTAMP_SECOND = 8
REG_TIMESTAMP_MICRO = 9
REG_OPERATION_CTRL = 10
REG_HEARTBEAT = 18
REG_VERSION = 19

# Core extension register addresses (vendor/app space)
REG_NET_SSID = 32
REG_NET_PASSWORD = 33
REG_NET_SERVER_IP = 34
REG_NET_SERVER_PORT = 35
REG_NET_CONFIG = 36

NET_SSID_LEN = 32
NET_PASSWORD_LEN = 64
NET_SERVER_IP_LEN = 16

NET_CFG_ENABLE_WIFI = 1 << 0
NET_CFG_ENABLE_TCP = 1 << 1
NET_CFG_APPLY = 1 << 6

OP_MODE_STANDBY = 0
OP_MODE_ACTIVE = 1
HEARTBEAT_EN_BIT = 2
ALIVE_EN_BIT = 7


def _checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def _pack_c_string_fixed(value: str, length: int, field_name: str) -> list[int]:
    encoded = value.encode("ascii")
    if len(encoded) >= length:
        raise ValueError(
            f"{field_name} too long ({len(encoded)} bytes). "
            f"Max {length - 1} bytes for null-terminated payload."
        )
    buf = bytearray(length)
    buf[:len(encoded)] = encoded
    return list(buf)


def build_read(address: int) -> bytes:
    raw_len = 4
    read_payload_types = {
        REG_WHO_AM_I: TYPE_U16,
        REG_TIMESTAMP_SECOND: TYPE_U32,
        REG_TIMESTAMP_MICRO: TYPE_U16,
        REG_OPERATION_CTRL: TYPE_U8,
        REG_HEARTBEAT: TYPE_U16,
        REG_VERSION: TYPE_U8,
        REG_NET_SSID: TYPE_U8,
        REG_NET_PASSWORD: TYPE_U8,
        REG_NET_SERVER_IP: TYPE_U8,
        REG_NET_SERVER_PORT: TYPE_U16,
        REG_NET_CONFIG: TYPE_U8,
    }
    payload_type = read_payload_types.get(address, TYPE_U8)
    header = struct.pack("BBBBB", MSG_READ, raw_len, address, PORT_DEFAULT, payload_type)
    return header + bytes([_checksum(header)])


def build_write_u8(address: int, value: int) -> bytes:
    payload = struct.pack("B", value)
    raw_len = 4 + len(payload)
    payload_type = TYPE_U8
    header = struct.pack("BBBBB", MSG_WRITE, raw_len, address, PORT_DEFAULT, payload_type)
    body = header + payload
    return body + bytes([_checksum(body)])


def build_write_u16(address: int, value: int) -> bytes:
    payload = struct.pack("<H", value)
    raw_len = 4 + len(payload)
    payload_type = TYPE_U16
    header = struct.pack("BBBBB", MSG_WRITE, raw_len, address, PORT_DEFAULT, payload_type)
    body = header + payload
    return body + bytes([_checksum(body)])


def build_write_u32(address: int, value: int) -> bytes:
    payload = struct.pack("<I", value)
    raw_len = 4 + len(payload)
    payload_type = TYPE_U32
    header = struct.pack("BBBBB", MSG_WRITE, raw_len, address, PORT_DEFAULT, payload_type)
    body = header + payload
    return body + bytes([_checksum(body)])


class HarpReply:
    def __init__(self, msg_type, address, payload_type_raw, ts_sec, ts_usec_raw, payload, raw):
        self.msg_type = msg_type
        self.address = address
        self.payload_type = payload_type_raw
        self.ts_sec = ts_sec
        self.ts_usec_raw = ts_usec_raw
        self.payload = payload
        self.raw = raw

    @property
    def timestamped(self) -> bool:
        return bool(self.payload_type & HAS_TIMESTAMP)

    @property
    def ts_us(self) -> Optional[float]:
        if self.ts_sec is None:
            return None
        return self.ts_sec * 1_000_000 + (self.ts_usec_raw << 5)

    @property
    def is_error(self) -> bool:
        return bool(self.msg_type & 0x08)

    def payload_u8(self) -> int:
        return self.payload[0]

    def payload_u16(self) -> int:
        return struct.unpack_from("<H", self.payload)[0]

    def payload_u32(self) -> int:
        return struct.unpack_from("<I", self.payload)[0]


def recv_reply(sock: socket.socket, timeout: float = 2.0) -> HarpReply:
    sock.settimeout(timeout)

    def recv_exact(n: int) -> bytes:
        buf = b""
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("Socket closed mid-message")
            buf += chunk
        return buf

    header_bytes = recv_exact(5)
    msg_type, raw_len, address, _port, payload_type_raw = struct.unpack("BBBBB", header_bytes)

    remaining = (raw_len + 2) - 5
    rest = recv_exact(remaining)
    full = header_bytes + rest

    has_ts = bool(payload_type_raw & HAS_TIMESTAMP)
    if has_ts:
        ts_sec = struct.unpack_from("<I", rest, 0)[0]
        ts_usec_raw = struct.unpack_from("<H", rest, 4)[0]
        payload_start = 6
    else:
        ts_sec = ts_usec_raw = None
        payload_start = 0

    payload_len = raw_len - 10 if has_ts else raw_len - 4
    payload = rest[payload_start: payload_start + payload_len]

    expected_cs = _checksum(full[:-1])
    actual_cs = full[-1]
    if expected_cs != actual_cs:
        raise ValueError(
            f"Checksum mismatch reg {address}: expected 0x{expected_cs:02X} got 0x{actual_cs:02X}"
        )

    return HarpReply(msg_type, address, payload_type_raw, ts_sec, ts_usec_raw, payload, full)


class HarpTCPConnection:
    def __init__(self, sock: socket.socket, timeout: float = 3.0):
        self.sock = sock
        self.timeout = timeout
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(self.timeout)

    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None

    def read_reg(self, address: int, timeout: float = 2.0) -> HarpReply:
        self.sock.sendall(build_read(address))
        return recv_reply(self.sock, timeout)

    def write_u8(self, address: int, value: int, timeout: float = 2.0) -> HarpReply:
        self.sock.sendall(build_write_u8(address, value))
        return recv_reply(self.sock, timeout)

    def write_u16(self, address: int, value: int, timeout: float = 2.0) -> HarpReply:
        self.sock.sendall(build_write_u16(address, value))
        return recv_reply(self.sock, timeout)

    def write_u32(self, address: int, value: int, timeout: float = 2.0) -> HarpReply:
        self.sock.sendall(build_write_u32(address, value))
        return recv_reply(self.sock, timeout)

    def drain(self, window: float = 0.2):
        self.sock.settimeout(window)
        try:
            while True:
                data = self.sock.recv(256)
                if not data:
                    break
        except (socket.timeout, BlockingIOError):
            pass

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def accept_harp_connection(
    listen_ip: str,
    listen_port: int,
    accept_timeout: float,
    setup_cb: Optional[Callable[[], None]] = None,
) -> Tuple[HarpTCPConnection, Tuple[str, int]]:
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((listen_ip, listen_port))
    listener.listen(1)
    listener.settimeout(accept_timeout)

    print(f"Listening on {listen_ip}:{listen_port} for ESP32 connection...")
    if setup_cb is not None:
        setup_cb()

    try:
        conn, peer = listener.accept()
        print(f"Connected by {peer[0]}:{peer[1]}")
        return HarpTCPConnection(conn), peer
    finally:
        listener.close()


def configure_network_over_usb(
    usb_port: str,
    ssid: str,
    password: str,
    server_ip: str,
    server_port: int,
    post_usb_wait: float = 2.0,
) -> None:
    from harp.serial.device import Device, HarpMessage, MessageType, PayloadType

    if not (0 <= server_port <= 65535):
        raise ValueError(f"server_port must be in [0, 65535], got {server_port}")

    ssid_bytes = _pack_c_string_fixed(ssid, NET_SSID_LEN, "ssid")
    pwd_bytes = _pack_c_string_fixed(password, NET_PASSWORD_LEN, "password")
    ip_bytes = _pack_c_string_fixed(server_ip, NET_SERVER_IP_LEN, "server_ip")

    apply_value = NET_CFG_ENABLE_WIFI | NET_CFG_ENABLE_TCP | NET_CFG_APPLY

    print("Configuring Wi-Fi/TCP over USB...")
    print(f"  usb={usb_port} ssid={ssid} server={server_ip}:{server_port}")
    device = Device(usb_port, "ibl.bin")
    try:
        device.write_u8(REG_NET_SSID, ssid_bytes)
        device.write_u8(REG_NET_PASSWORD, pwd_bytes)
        device.write_u8(REG_NET_SERVER_IP, ip_bytes)
        device.write_u16(REG_NET_SERVER_PORT, server_port)

        # APPLY can disrupt CDC. Send without waiting for reply.
        apply_msg = HarpMessage(MessageType.WRITE, PayloadType.U8, REG_NET_CONFIG, apply_value)
        device.send(apply_msg, expect_reply=False)
    finally:
        try:
            device.disconnect()
        except Exception:
            pass

    if post_usb_wait > 0:
        print(f"Waiting {post_usb_wait:.1f}s after apply...")
        time.sleep(post_usb_wait)


def reboot_device_over_usb(usb_port: str) -> None:
    from harp.serial.device import Device
    from harp.protocol import CommonRegisters as Regs

    print("Rebooting device over USB (RST_DFU bit)...")
    device = Device(usb_port, "ibl.bin")
    try:
        device.write_u8(Regs.RESET_DEV, 0b00100000)
    finally:
        try:
            device.disconnect()
        except Exception:
            pass
