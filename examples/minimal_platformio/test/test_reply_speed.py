#!/usr/bin/env python3
from harp.serial.device import Device
from harp.protocol import CommonRegisters as Regs
import numpy as np
import os
import sys
from time import perf_counter
from matplotlib import pyplot as plt


ROUND_TRIPS = 10000


def print_percentiles(name, values):
    print(f"{name}:")
    print(f"  median: {np.percentile(values, 50):.6f}")
    print(f"  p90:    {np.percentile(values, 90):.6f}")
    print(f"  p95:    {np.percentile(values, 95):.6f}")
    print(f"  p99:    {np.percentile(values, 99):.6f}")
    print(f"  p99.9:  {np.percentile(values, 99.9):.6f}")
    print(f"  max:    {np.max(values):.6f}")


# Open the device and print the info on screen
# Open serial connection and save communication to a file
if os.name == 'posix': # check for Linux.
    device = Device("/dev/ttyACM0", "ibl.bin")
else: # assume Windows.
    device = Device("COM5", "ibl.bin")


send_t = np.zeros(ROUND_TRIPS, dtype=float)
recv_t = np.zeros(ROUND_TRIPS, dtype=float)
rtt_t = np.zeros(ROUND_TRIPS, dtype=float)
device_reply_ts_t = np.full(ROUND_TRIPS, np.nan, dtype=float)

# Use a write command as requested. We first read the current OPERATION_CTRL
# and then write the same value each round-trip to avoid changing behavior.
op_ctrl_value = device.read_u8(Regs.OPERATION_CTRL).payload

print(f"Performing {ROUND_TRIPS}x round trips. "
       "(WRITE from PC to Harp device. Timestamped reply from Harp device to PC.)")
progress_step = max(1, ROUND_TRIPS // 100)
for i in range(ROUND_TRIPS):
    send_t[i] = perf_counter()
    reply = device.write_u8(Regs.OPERATION_CTRL, op_ctrl_value)
    recv_t[i] = perf_counter()
    rtt_t[i] = recv_t[i] - send_t[i]
    if reply is not None and reply.timestamp is not None:
        device_reply_ts_t[i] = reply.timestamp
    if ((i + 1) % progress_step == 0) or (i + 1 == ROUND_TRIPS):
        percent = int(((i + 1) * 100) / ROUND_TRIPS)
        sys.stdout.write(f"\rProgress: {percent:3d}% ({i + 1}/{ROUND_TRIPS})")
        sys.stdout.flush()

print()

print(f"Summary:")
print(f"mean: {np.mean(rtt_t):.6f}")
print(f"std dev: {np.std(rtt_t):.6f}")
print(f"max: {np.max(rtt_t):.6f} at index: {np.argmax(rtt_t)}")
print()
print_percentiles("RTT percentiles [s]", rtt_t)
print()

# ---------------------------------------------------------------------------
# Heuristic split using host send/receive times + device reply timestamp:
# 1) offset_i = host_receive_i - device_reply_ts_i
# 2) baseline offset ~= minimum observed transport/clock offset
# 3) estimated reply path (device->host) = offset_i - baseline
# 4) estimated send+device processing = RTT - estimated reply path
#
# This is an estimate because host/device clocks are different.
# ---------------------------------------------------------------------------
valid = ~np.isnan(device_reply_ts_t)
if np.any(valid):
    offset_t = recv_t[valid] - device_reply_ts_t[valid]
    baseline_offset = np.min(offset_t)
    reply_path_t = np.maximum(0.0, offset_t - baseline_offset)
    send_plus_fw_t = np.maximum(0.0, rtt_t[valid] - reply_path_t)

    print(f"Estimated latency split (heuristic):")
    print(f"  baseline offset used: {baseline_offset:.6f} s")
    print_percentiles("  PC send -> ESP receive/parse/apply [s]", send_plus_fw_t)
    print_percentiles("  ESP reply queued/sent -> PC receive [s]", reply_path_t)
    print()
else:
    print("No timestamped replies available; split estimation skipped.")
    print()

large_value_locations = np.where(rtt_t > 0.003)
print("The following delay times are large:")
print(large_value_locations)
print("indexes of the large delay times:")
print([rtt_t[i] for i in large_value_locations])


# Close connection
device.disconnect()

fig, ax = plt.subplots(figsize=(8, 4))
ax.hist(rtt_t, bins="auto", edgecolor="black", alpha=0.85)
ax.set_title(f"Reply latency histogram (N={len(rtt_t)}), wired connection")
ax.set_xlabel("Round-trip latency (s)")
ax.set_ylabel("Count")
ax.grid(axis="y", linestyle="--", alpha=0.4)
fig.tight_layout()
hist_path = os.path.join(os.getcwd(), "reply_speed_hist_wired.png")
fig.savefig(hist_path, dpi=140)
plt.close(fig)
print(f"  histogram: saved to {hist_path}")
