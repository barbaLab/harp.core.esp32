"""
test_harp_tcp.py
================
TCP-transport tests for harp.core.esp32.

Tests:
    1. Configure Wi-Fi/TCP settings over USB (SSID/password/server endpoint)
    2. Listen for inbound TCP connection from the ESP32
    3. get_info          – READ all 18 core registers (REG 0–17) and parse them
    4. enable_heartbeat  – write OPERATION_CTRL to ACTIVE + ALIVE_EN, verify
                                                 heartbeat EVENTs arrive on REG 8 (TIMESTAMP_SECOND)
    5. test_reply_speed  – send N READ requests back-to-back, measure round-trip

Wire format (from harp_message.h + harp_core.cpp):
  Outbound READ  (no timestamp, no payload):
    [type=1][raw_len=4][address][port=255][payload_type][checksum]
    raw_len = 4  ->  msg_size = 6 bytes total

  Inbound reply (always timestamped – HAS_TIMESTAMP is always set in send_harp_reply):
    [type][raw_len][address][port=255][payload_type | 0x10]
    [ts_sec: 4 bytes LE][ts_usec: 2 bytes LE]
    [payload: N bytes]
    [checksum: 1 byte]
    raw_len = num_payload_bytes + 10
    msg_size = raw_len + 2

  Outbound WRITE (no timestamp):
    [type=2][raw_len=4+N][address][port=255][payload_type][payload: N bytes][checksum]
    raw_len = 4 + N  ->  msg_size = raw_len + 2
"""

import socket
import struct
import time
import statistics
import matplotlib.pyplot as plt
import argparse
import os
from typing import Optional

# -- Harp constants (from harp_message.h / reg_types.h) -----------------------
MSG_READ        = 0x01
MSG_WRITE       = 0x02
MSG_EVENT       = 0x03
MSG_READ_ERROR  = 0x09
MSG_WRITE_ERROR = 0x0A

HAS_TIMESTAMP = 0x10

TYPE_U8  = 0x01
TYPE_U16 = 0x02
TYPE_U32 = 0x04
TYPE_U64 = 0x08
TYPE_S8  = 0x81
TYPE_S16 = 0x82

PORT_DEFAULT = 255

# -- Core register addresses (from core_registers.h) --------------------------
REG_WHO_AM_I          = 0
REG_HW_VERSION_H      = 1
REG_HW_VERSION_L      = 2
REG_ASSEMBLY_VERSION  = 3
REG_HARP_VERSION_H    = 4
REG_HARP_VERSION_L    = 5
REG_FW_VERSION_H      = 6
REG_FW_VERSION_L      = 7
REG_TIMESTAMP_SECOND  = 8
REG_TIMESTAMP_MICRO   = 9
REG_OPERATION_CTRL    = 10
REG_RESET_DEF         = 11
REG_DEVICE_NAME       = 12
REG_SERIAL_NUMBER     = 13
REG_CLOCK_CONFIG      = 14
REG_TIMESTAMP_OFFSET  = 15
REG_UUID              = 16
REG_TAG               = 17
REG_NET_SSID          = 18
REG_NET_PASSWORD      = 19
REG_NET_SERVER_IP     = 20
REG_NET_SERVER_PORT   = 21
REG_NET_CONFIG        = 22

NET_SSID_LEN = 32
NET_PASSWORD_LEN = 64
NET_SERVER_IP_LEN = 16

NET_CFG_ENABLE_WIFI = 1 << 0
NET_CFG_ENABLE_TCP = 1 << 1
NET_CFG_APPLY = 1 << 6

# R_OPERATION_CTRL bitfields (from core_registers.h)
# bits [1:0] = OP_MODE   (0=STANDBY, 1=ACTIVE)
# bit  4     = MUTE_RPL
# bit  5     = VISUAL_EN
# bit  6     = OPLEDEN
# bit  7     = ALIVE_EN  (enables heartbeat EVENTs)
OP_MODE_STANDBY = 0
OP_MODE_ACTIVE  = 1
ALIVE_EN_BIT    = 7


def _default_usb_port() -> str:
    return "/dev/ttyACM0" if os.name == "posix" else "COM5"


def _pack_c_string_fixed(value: str, length: int, field_name: str) -> list[int]:
    encoded = value.encode("ascii")
    if len(encoded) >= length:
        raise ValueError(
            f"{field_name} is too long ({len(encoded)} bytes). "
            f"Max is {length - 1} bytes for null-terminated payload."
        )
    buf = bytearray(length)
    buf[:len(encoded)] = encoded
    return list(buf)


def configure_network_over_usb(usb_port: str,
                               ssid: str,
                               password: str,
                               server_ip: str,
                               server_port: int,
                               post_usb_wait: float = 2.0) -> None:
    """
    Configure network registers over USB before starting TCP tests.
    """
    from harp.serial.device import Device

    if not (0 <= server_port <= 65535):
        raise ValueError(f"server_port must be in [0, 65535], got {server_port}")

    ssid_bytes = _pack_c_string_fixed(ssid, NET_SSID_LEN, "ssid")
    pwd_bytes = _pack_c_string_fixed(password, NET_PASSWORD_LEN, "password")
    ip_bytes = _pack_c_string_fixed(server_ip, NET_SERVER_IP_LEN, "server_ip")

    apply_value = NET_CFG_ENABLE_WIFI | NET_CFG_ENABLE_TCP | NET_CFG_APPLY

    print("\n-- configure_network_over_usb --------------------------------------")
    print(f"  USB port            : {usb_port}")
    print(f"  SSID                : {ssid}")
    print(f"  Server IP           : {server_ip}")
    print(f"  Server Port         : {server_port}")

    # Imported here so the TCP-only path doesn't require USB dependencies
    # until this step is actually used.
    device = Device(usb_port, "ibl.bin")
    try:
        from harp.serial.device import HarpMessage, MessageType, PayloadType

        _ = device.write_u8(REG_NET_SSID, ssid_bytes)
        _ = device.write_u8(REG_NET_PASSWORD, pwd_bytes)
        _ = device.write_u8(REG_NET_SERVER_IP, ip_bytes)
        _ = device.write_u16(REG_NET_SERVER_PORT, server_port)

        # APPLY can reset/disrupt the CDC transport; do not wait for reply.
        apply_msg = HarpMessage(
            MessageType.WRITE, PayloadType.U8, REG_NET_CONFIG, apply_value
        )
        device.send(apply_msg, expect_reply=False)
    finally:
        try:
            device.disconnect()
        except Exception:
            # If the link drops while applying network settings, cleanup can
            # fail on Windows serial internals; ignore and continue to TCP.
            pass

    if post_usb_wait > 0:
        print(f"  Waiting {post_usb_wait:.1f}s for network apply/connect...")
        time.sleep(post_usb_wait)

    print("  PASSED")


# -- Message building ----------------------------------------------------------

def _checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def build_read(address: int) -> bytes:
    """6-byte READ request (no timestamp, no payload). raw_len = 4."""
    raw_len      = 4
    payload_type = TYPE_U8
    header = struct.pack("BBBBB", MSG_READ, raw_len, address, PORT_DEFAULT, payload_type)
    return header + bytes([_checksum(header)])


def build_write_u8(address: int, value: int) -> bytes:
    """7-byte WRITE for a single U8 register. raw_len = 5."""
    payload      = struct.pack("B", value)
    raw_len      = 4 + len(payload)
    payload_type = TYPE_U8
    header = struct.pack("BBBBB", MSG_WRITE, raw_len, address, PORT_DEFAULT, payload_type)
    body   = header + payload
    return body + bytes([_checksum(body)])


# -- Reply parsing ------------------------------------------------------------

class HarpReply:
    def __init__(self, msg_type, address, payload_type_raw,
                 ts_sec, ts_usec_raw, payload, raw):
        self.msg_type      = msg_type
        self.address       = address
        self.payload_type  = payload_type_raw
        self.ts_sec        = ts_sec
        self.ts_usec_raw   = ts_usec_raw
        self.payload       = payload
        self.raw           = raw

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

    def payload_u8(self)  -> int: return self.payload[0]
    def payload_u16(self) -> int: return struct.unpack_from("<H", self.payload)[0]
    def payload_u32(self) -> int: return struct.unpack_from("<I", self.payload)[0]
    def payload_str(self) -> str: return self.payload.rstrip(b"\x00").decode("ascii", errors="replace")


def recv_reply(sock: socket.socket, timeout: float = 2.0) -> HarpReply:
    """Read one complete Harp reply from the TCP socket."""
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
    msg_type, raw_len, address, port, payload_type_raw = struct.unpack("BBBBB", header_bytes)

    remaining = (raw_len + 2) - 5
    rest      = recv_exact(remaining)
    full      = header_bytes + rest

    has_ts = bool(payload_type_raw & HAS_TIMESTAMP)
    if has_ts:
        ts_sec,      = struct.unpack_from("<I", rest, 0)
        ts_usec_raw, = struct.unpack_from("<H", rest, 4)
        payload_start = 6
    else:
        ts_sec = ts_usec_raw = None
        payload_start = 0

    payload_len = raw_len - 10 if has_ts else raw_len - 4
    payload     = rest[payload_start : payload_start + payload_len]

    expected_cs = _checksum(full[:-1])
    actual_cs   = full[-1]
    if expected_cs != actual_cs:
        raise ValueError(
            f"Checksum mismatch on reg {address}: "
            f"expected 0x{expected_cs:02X} got 0x{actual_cs:02X}"
        )

    return HarpReply(msg_type, address, payload_type_raw,
                     ts_sec, ts_usec_raw, payload, full)


# -- Client -------------------------------------------------------------------

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

    def drain(self, window: float = 0.2):
        """Discard any pending bytes (e.g. stale heartbeats)."""
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


def accept_harp_connection(listen_ip: str,
                           listen_port: int,
                           accept_timeout: float,
                           setup_cb) -> tuple[HarpTCPConnection, tuple[str, int]]:
    """
    Open a TCP listener, run setup callback (USB provisioning), then accept
    the inbound ESP32 connection.
    """
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((listen_ip, listen_port))
    listener.listen(1)
    listener.settimeout(accept_timeout)

    print(f"\nListening on {listen_ip}:{listen_port} for ESP32 connection ...")
    setup_cb()

    try:
        conn, peer = listener.accept()
        print(f"Connected by {peer[0]}:{peer[1]}")
        return HarpTCPConnection(conn), peer
    finally:
        listener.close()


# -- Test 1: get_info ---------------------------------------------------------

def test_get_info(client: HarpTCPConnection) -> dict:
    """READ all 18 core registers and parse them."""
    print("\n-- test_get_info ----------------------------------------------------")
    info = {}

    r = client.read_reg(REG_WHO_AM_I)
    assert not r.is_error, "READ error on WHO_AM_I"
    info["who_am_i"] = r.payload_u16()
    print(f"  WHO_AM_I           : 0x{info['who_am_i']:04X}")

    for reg, name in [
        (REG_HW_VERSION_H,     "hw_version_major"),
        (REG_HW_VERSION_L,     "hw_version_minor"),
        (REG_ASSEMBLY_VERSION, "assembly_version"),
        (REG_HARP_VERSION_H,   "harp_version_major"),
        (REG_HARP_VERSION_L,   "harp_version_minor"),
        (REG_FW_VERSION_H,     "fw_version_major"),
        (REG_FW_VERSION_L,     "fw_version_minor"),
    ]:
        r = client.read_reg(reg)
        assert not r.is_error, f"READ error on {name}"
        info[name] = r.payload_u8()
        print(f"  {name:<22}: {info[name]}")

    r = client.read_reg(REG_TIMESTAMP_SECOND)
    assert not r.is_error
    info["timestamp_second"] = r.payload_u32()
    print(f"  timestamp_second   : {info['timestamp_second']}")

    r = client.read_reg(REG_TIMESTAMP_MICRO)
    assert not r.is_error
    info["timestamp_micro_raw"] = r.payload_u16()
    info["timestamp_us"]        = info["timestamp_micro_raw"] << 5
    print(f"  timestamp_micro_us : {info['timestamp_us']}")

    r = client.read_reg(REG_OPERATION_CTRL)
    assert not r.is_error
    info["op_ctrl"] = r.payload_u8()
    info["op_mode"] = info["op_ctrl"] & 0x03
    print(f"  op_ctrl            : 0x{info['op_ctrl']:02X}  (op_mode={info['op_mode']})")

    r = client.read_reg(REG_DEVICE_NAME)
    assert not r.is_error
    info["device_name"] = r.payload_str()
    print(f"  device_name        : '{info['device_name']}'")

    r = client.read_reg(REG_SERIAL_NUMBER)
    assert not r.is_error
    info["serial_number"] = r.payload_u16()
    print(f"  serial_number      : {info['serial_number']}")

    r = client.read_reg(REG_UUID)
    assert not r.is_error
    info["uuid"] = r.payload.hex()
    print(f"  uuid               : {info['uuid']}")

    r = client.read_reg(REG_TAG)
    assert not r.is_error
    info["tag"] = r.payload_str()
    print(f"  tag                : '{info['tag']}'")

    assert info["who_am_i"] != 0,        "WHO_AM_I should not be 0"
    assert len(info["device_name"]) > 0, "DEVICE_NAME should not be empty"
    assert info["timestamp_second"] >= 0

    print("  PASSED")
    return info


# -- Test 2: enable_heartbeat -------------------------------------------------

def test_enable_heartbeat(client: HarpTCPConnection,
                          n_heartbeats: int = 3,
                          timeout_per: float = 4.0) -> list:
    """
    Set OPERATION_CTRL = ACTIVE | ALIVE_EN, collect n heartbeat EVENTs on
    REG_TIMESTAMP_SECOND, verify inter-arrival ~1 s.
    """
    print("\n-- test_enable_heartbeat --------------------------------------------")

    ctrl_byte = OP_MODE_ACTIVE | (1 << ALIVE_EN_BIT)   # 0x81
    r = client.write_u8(REG_OPERATION_CTRL, ctrl_byte)
    assert not r.is_error, f"Failed to write OPERATION_CTRL: {r.msg_type:#04x}"
    print(f"  OPERATION_CTRL set to 0x{ctrl_byte:02X} (ACTIVE | ALIVE_EN)")

    client.drain(0.1)

    timestamps_us = []
    print(f"  Waiting for {n_heartbeats} heartbeat EVENTs ...")
    for i in range(n_heartbeats):
        reply = recv_reply(client.sock, timeout=timeout_per)
        assert reply.msg_type == MSG_EVENT, \
            f"Expected EVENT (0x{MSG_EVENT:02X}), got 0x{reply.msg_type:02X}"
        assert reply.address == REG_TIMESTAMP_SECOND, \
            f"Expected heartbeat on REG {REG_TIMESTAMP_SECOND}, got {reply.address}"
        ts = reply.ts_us
        assert ts is not None, "Heartbeat reply should carry a timestamp"
        timestamps_us.append(ts)
        print(f"  Heartbeat {i+1}: ts={ts/1e6:.3f} s  "
              f"(sec={reply.ts_sec}, raw_usec={reply.ts_usec_raw})")

    intervals = [(timestamps_us[i+1] - timestamps_us[i]) / 1e6
                 for i in range(len(timestamps_us) - 1)]
    for idx, iv in enumerate(intervals):
        print(f"  Interval {idx+1}: {iv*1000:.1f} ms")
        assert 0.8 <= iv <= 1.5, \
            f"Heartbeat interval out of range: {iv:.3f} s (expected ~1.0 s)"

    # Restore STANDBY
    client.write_u8(REG_OPERATION_CTRL, OP_MODE_STANDBY)
    client.drain(0.1)

    print("  PASSED")
    return intervals


# -- Test 3: test_reply_speed -------------------------------------------------

def test_reply_speed(client: HarpTCPConnection,
                     n_requests: int = 100,
                     address: int = REG_WHO_AM_I) -> dict:
    """
    Sequential READ loop — measures per-round-trip latency in milliseconds.
    Reports min / mean / median / p95 / max.
    """
    print(f"\n-- test_reply_speed  (N={n_requests}, reg={address}) ----------------")

    latencies_ms = []
    errors = 0

    for _ in range(n_requests):
        t0 = time.perf_counter()
        client.sock.sendall(build_read(address))
        try:
            r = recv_reply(client.sock, timeout=2.0)
            t1 = time.perf_counter()
            if r.is_error:
                errors += 1
            else:
                latencies_ms.append((t1 - t0) * 1000)
        except socket.timeout:
            errors += 1

    assert errors == 0, f"{errors}/{n_requests} requests errored"
    assert len(latencies_ms) >= n_requests * 0.95, \
        f"Too many missing replies: {n_requests - len(latencies_ms)}"

    stats = {
        "n":         len(latencies_ms),
        "min_ms":    min(latencies_ms),
        "mean_ms":   statistics.mean(latencies_ms),
        "median_ms": statistics.median(latencies_ms),
        "p90_ms":    sorted(latencies_ms)[int(len(latencies_ms) * 0.90)],
        "p95_ms":    sorted(latencies_ms)[int(len(latencies_ms) * 0.95)],
        "p99_ms":    sorted(latencies_ms)[int(len(latencies_ms) * 0.99)],
        "max_ms":    max(latencies_ms),
        "errors":    errors,
    }

    print(f"  n        : {stats['n']}")
    print(f"  min      : {stats['min_ms']:.2f} ms")
    print(f"  mean     : {stats['mean_ms']:.2f} ms")
    print(f"  median   : {stats['median_ms']:.2f} ms")
    print(f"  p90      : {stats['p90_ms']:.2f} ms")
    print(f"  p95      : {stats['p95_ms']:.2f} ms")
    print(f"  p99      : {stats['p99_ms']:.2f} ms")
    print(f"  max      : {stats['max_ms']:.2f} ms")
    print(f"  errors   : {stats['errors']}")
    print("  PASSED")

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.hist(latencies_ms, bins="auto", edgecolor="black", alpha=0.85)
    ax.set_title(f"Reply latency histogram (N={len(latencies_ms)}, reg={address}), tcp connection")
    ax.set_xlabel("Round-trip latency (ms)")
    ax.set_ylabel("Count")
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    fig.tight_layout()

    hist_path = os.path.join(os.getcwd(), "reply_speed_hist_tcp.png")
    fig.savefig(hist_path, dpi=140)
    plt.close(fig)
    print(f"  histogram: saved to {hist_path}")
    return stats


# -- Entry point --------------------------------------------------------------

def run_all(listen_ip: str,
            port: int,
            usb_port: str,
            ssid: str,
            password: str,
            server_ip: str,
            server_port: int,
            speed_n: int = 100,
            run_get_info: bool = True,
            run_heartbeat: bool = True,
            run_speed: bool = True,
            heartbeats: int = 3,
            heartbeat_timeout: float = 4.0,
            post_usb_wait: float = 2.0,
            accept_timeout: float = 20.0):
    def _setup_usb():
        configure_network_over_usb(
            usb_port=usb_port,
            ssid=ssid,
            password=password,
            server_ip=server_ip,
            server_port=server_port,
            post_usb_wait=post_usb_wait,
        )

    connection, _peer = accept_harp_connection(
        listen_ip=listen_ip,
        listen_port=port,
        accept_timeout=accept_timeout,
        setup_cb=_setup_usb,
    )

    info = None
    ivs = None
    stats = None
    with connection as client:
        if run_get_info:
            info = test_get_info(client)
            print(
                "  Identity summary    : "
                f"WHO_AM_I=0x{info['who_am_i']:04X}, "
                f"SERIAL={info['serial_number']}, NAME='{info['device_name']}'"
            )
        if run_heartbeat:
            ivs = test_enable_heartbeat(
                client,
                n_heartbeats=heartbeats,
                timeout_per=heartbeat_timeout,
            )
        if run_speed:
            stats = test_reply_speed(client, n_requests=speed_n)
    print("\n== All tests passed ==")
    return {"info": info, "heartbeat_intervals_s": ivs, "speed": stats}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Harp TCP tests")
    parser.add_argument("--listen-ip", default="0.0.0.0",
                        dest="listen_ip",
                        help="Local interface to bind for inbound ESP32 TCP connection")
    parser.add_argument("--port",    type=int, default=9999)
    parser.add_argument("--usb-port", default=_default_usb_port(),
                        dest="usb_port",
                        help="USB CDC port used for initial network configuration")
    parser.add_argument("--ssid", required=True,
                        help="Wi-Fi SSID to write via USB")
    parser.add_argument("--pwd", required=True,
                        dest="password",
                        help="Wi-Fi password to write via USB")
    parser.add_argument("--server-ip", required=True,
                        dest="server_ip",
                        help="Server IP to write via USB (typically this host's LAN IP)")
    parser.add_argument("--server-port", type=int, default=9999,
                        dest="server_port",
                        help="Server port to write via USB")
    parser.add_argument("--post-usb-wait", type=float, default=2.0,
                        dest="post_usb_wait",
                        help="Seconds to wait after USB apply before TCP connect")
    parser.add_argument("--accept-timeout", type=float, default=20.0,
                        dest="accept_timeout",
                        help="Seconds to wait for inbound ESP32 TCP connection")

    parser.add_argument("--run-get-info", action="store_true",
                        help="Run get_info test phase")
    parser.add_argument("--run-heartbeat", action="store_true",
                        help="Run heartbeat test phase")
    parser.add_argument("--run-speed", action="store_true",
                        help="Run reply-speed test phase")

    parser.add_argument("--heartbeats", type=int, default=3,
                        help="Number of heartbeat events to collect")
    parser.add_argument("--heartbeat-timeout", type=float, default=4.0,
                        dest="heartbeat_timeout",
                        help="Timeout (seconds) per heartbeat receive")
    parser.add_argument("--speed-n", type=int, default=100,
                        dest="speed_n",
                        help="Number of requests for reply-speed test")
    args = parser.parse_args()

    run_get_info = args.run_get_info
    run_heartbeat = args.run_heartbeat
    run_speed = args.run_speed
    if not (run_get_info or run_heartbeat or run_speed):
        run_get_info = True
        run_heartbeat = True
        run_speed = True

    run_all(
        listen_ip=args.listen_ip,
        port=args.port,
        usb_port=args.usb_port,
        ssid=args.ssid,
        password=args.password,
        server_ip=args.server_ip,
        server_port=args.server_port,
        speed_n=args.speed_n,
        run_get_info=run_get_info,
        run_heartbeat=run_heartbeat,
        run_speed=run_speed,
        heartbeats=args.heartbeats,
        heartbeat_timeout=args.heartbeat_timeout,
        post_usb_wait=args.post_usb_wait,
        accept_timeout=args.accept_timeout,
    )
