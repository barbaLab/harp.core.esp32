from __future__ import annotations

import argparse
import os
import struct
import time
from typing import Any

from harp.serial.device import Device


# Registers from inc/app_loadcell.h
REG_LC_DATA = 36
REG_LC_EVENT = 37
REG_LC_CONFIG = 38

# Payload layouts from inc/app_loadcell.h (packed)
# LcConfigPayload: float entry, float exit, uint16 debounce, uint8 stream, float filter_alpha
LC_CONFIG_FMT = "<ffHBf"
# LcDataPayload: float force[4], float total, float cop_x, float cop_y
LC_DATA_FMT = "<7f"


def run_com_plot_mode(device: Device, duration_s: float, poll_s: float = 0.02) -> None:
	"""Plot center-of-mass (cop_x, cop_y) from LC_DATA stream in real time."""
	try:
		import matplotlib.pyplot as plt
	except ImportError as exc:
		raise RuntimeError(
			"matplotlib is required for --mode plot-com. Install with: pip install matplotlib"
		) from exc

	plt.ion()
	fig, ax = plt.subplots(figsize=(7, 6))
	ax.set_title("Loadcell Center of Mass (30s)")
	ax.set_xlabel("CoP X [mm]")
	ax.set_ylabel("CoP Y [mm]")
	ax.grid(True, alpha=0.3)
	ax.set_xlim(-100, 100)
	ax.set_ylim(-80, 80)

	path_x: list[float] = []
	path_y: list[float] = []
	line, = ax.plot([], [], "-", lw=1.5, alpha=0.8, label="CoM path")
	point = ax.scatter([], [], s=45, c="tab:red", label="Current")
	ax.legend(loc="upper right")

	start = time.time()
	deadline = start + duration_s
	data_events = 0

	print(f"[PLOT] Running real-time CoM plot for {duration_s:.1f} s")
	while time.time() < deadline:
		events = device.get_events()
		updated = False

		for event in events:
			addr = _event_address(event)
			payload = _to_payload_bytes(event)

			if addr == REG_LC_DATA and len(payload) >= struct.calcsize(LC_DATA_FMT):
				_, _, _, _, total, cop_x, cop_y = struct.unpack(
					LC_DATA_FMT, payload[: struct.calcsize(LC_DATA_FMT)]
				)
				path_x.append(cop_x)
				path_y.append(cop_y)
				line.set_data(path_x, path_y)
				point.set_offsets([[cop_x, cop_y]])
				data_events += 1
				updated = True

				if data_events % 20 == 0:
					elapsed = time.time() - start
					print(
						f"    t={elapsed:5.1f}s total={total:8.3f} g "
						f"cop=({cop_x:7.2f}, {cop_y:7.2f})"
					)

		if updated:
			fig.canvas.draw_idle()
			plt.pause(0.001)

		time.sleep(poll_s)

	print(f"[PLOT] Done. LC_DATA events processed: {data_events}")
	plt.ioff()
	plt.show()


def _default_port() -> str:
	if os.name == "posix":
		return "/dev/ttyACM0"
	return "COM5"


def _to_payload_bytes(reply: Any) -> bytes:
	"""Normalize Harp replies to raw payload bytes."""
	payload = getattr(reply, "payload", reply)

	if payload is None:
		return b""
	if isinstance(payload, (bytes, bytearray)):
		return bytes(payload)
	if isinstance(payload, int):
		return bytes([payload & 0xFF])
	if isinstance(payload, (list, tuple)):
		return bytes(int(v) & 0xFF for v in payload)

	try:
		return bytes(payload)
	except TypeError as exc:
		raise TypeError(f"Unsupported payload type: {type(payload)!r}") from exc


def _event_address(event: Any) -> int | None:
	addr = getattr(event, "address", None)
	if addr is not None:
		return int(addr)
	header = getattr(event, "header", None)
	if header is not None:
		addr = getattr(header, "address", None)
		if addr is not None:
			return int(addr)
	return None


def write_lc_config(
	device: Device,
	*,
	entry_threshold_g: float,
	exit_threshold_g: float,
	debounce_frames: int,
	stream_enable: bool,
	filter_alpha: float = 1.0,
) -> None:
	payload = struct.pack(
		LC_CONFIG_FMT,
		float(entry_threshold_g),
		float(exit_threshold_g),
		int(debounce_frames),
		1 if stream_enable else 0,
		max(0.0, min(1.0, float(filter_alpha))),
	)
	device.write_u8(REG_LC_CONFIG, list(payload))


def read_lc_config(device: Device) -> tuple[float, float, int, int, float]:
	reply = device.read_u8(REG_LC_CONFIG)
	payload = _to_payload_bytes(reply)
	expected = struct.calcsize(LC_CONFIG_FMT)
	if len(payload) < expected:
		raise RuntimeError(
			f"LC_CONFIG payload too short: expected {expected} bytes, got {len(payload)}"
		)
	return struct.unpack(LC_CONFIG_FMT, payload[:expected])


def read_lc_data(device: Device) -> tuple[float, float, float, float, float, float, float]:
	reply = device.read_u8(REG_LC_DATA)
	payload = _to_payload_bytes(reply)
	expected = struct.calcsize(LC_DATA_FMT)
	if len(payload) < expected:
		raise RuntimeError(
			f"LC_DATA payload too short: expected {expected} bytes, got {len(payload)}"
		)
	return struct.unpack(LC_DATA_FMT, payload[:expected])


def read_lc_event(device: Device) -> int:
	reply = device.read_u8(REG_LC_EVENT)
	payload = _to_payload_bytes(reply)
	if not payload:
		raise RuntimeError("LC_EVENT payload is empty")
	return int(payload[0])


def main() -> None:
	parser = argparse.ArgumentParser(
		description="Set loadcell config and read data/events from Harp ESP32 firmware"
	)
	parser.add_argument(
		"--mode",
		choices=["read", "plot-com"],
		default="read",
		help="read = current behavior; plot-com = real-time CoM plot (auto-enables stream)",
	)
	parser.add_argument("--port", default=_default_port(), help="Serial port (default: COM5 or /dev/ttyACM0)")
	parser.add_argument("--timeout", type=float, default=1.0, help="Device timeout in seconds")
	parser.add_argument("--entry", type=float, default=20.0, help="Entry threshold in grams")
	parser.add_argument("--exit", dest="exit_", type=float, default=10.0, help="Exit threshold in grams")
	parser.add_argument("--debounce", type=int, default=8, help="Debounce frame count")
	parser.add_argument("--filter-alpha", type=float, default=1.0, help="IIR filter coefficient (1.0=off, <1.0=smooth)")
	parser.add_argument("--stream", action="store_true", help="Enable LC_DATA event streaming")
	parser.add_argument("--plot-duration", type=float, default=30.0, help="Duration in seconds for --mode plot-com")
	parser.add_argument(
		"--include-lc-event",
		action="store_true",
		help="Also read/print LC_EVENT occupancy state and stream events",
	)
	parser.add_argument("--samples", type=int, default=10, help="Number of direct LC_DATA reads")
	parser.add_argument("--interval", type=float, default=0.2, help="Delay between direct reads (seconds)")
	parser.add_argument("--event-window", type=float, default=3.0, help="Seconds to collect async events")
	args = parser.parse_args()

	print(f"Connecting to {args.port!r}")

	with Device(args.port, timeout=args.timeout) as device:
		try:
			print(device.info())
		except Exception as exc:
			print(f"device.info() failed, continuing: {exc}")

		print("\n[1] Writing LC_CONFIG")
		stream_enabled = args.stream or (args.mode == "plot-com")
		print(
			f"    entry={args.entry:.3f} g, exit={args.exit_:.3f} g, "
			f"debounce={args.debounce}, stream={int(stream_enabled)}, "
			f"filter_alpha={args.filter_alpha:.3f}"
		)
		write_lc_config(
			device,
			entry_threshold_g=args.entry,
			exit_threshold_g=args.exit_,
			debounce_frames=args.debounce,
			stream_enable=stream_enabled,
			filter_alpha=args.filter_alpha,
		)

		print("[2] Reading back LC_CONFIG")
		entry, exit_, debounce, stream_enable, filter_alpha_rb = read_lc_config(device)
		print(
			"    readback: "
			f"entry={entry:.3f} g, exit={exit_:.3f} g, "
			f"debounce={debounce}, stream={stream_enable}, "
			f"filter_alpha={filter_alpha_rb:.3f}"
		)

		if args.include_lc_event:
			print("[3] Reading LC_EVENT state")
			occupied = read_lc_event(device)
			print(f"    occupied={occupied}")

		if args.mode == "plot-com":
			run_com_plot_mode(device, duration_s=args.plot_duration)
			print("\nLoadcell test complete.")
			return

		print(f"[4] Reading LC_DATA ({args.samples} samples)")
		direct_nonzero = 0
		for i in range(args.samples):
			force0, force1, force2, force3, total, cop_x, cop_y = read_lc_data(device)
			if abs(total) > 1e-6:
				direct_nonzero += 1
			print(
				f"    {i + 1:02d}: total={total:8.3f} g "
				f"cop=({cop_x:7.2f}, {cop_y:7.2f}) mm "
				f"f=[{force0:7.3f}, {force1:7.3f}, {force2:7.3f}, {force3:7.3f}]"
			)
			time.sleep(args.interval)

		if args.stream:
			print(f"[5] Collecting LC_DATA stream for {args.event_window:.1f} s")
			deadline = time.time() + args.event_window
			data_events = 0
			occ_events = 0

			while time.time() < deadline:
				events = device.get_events()
				for event in events:
					addr = _event_address(event)
					payload = _to_payload_bytes(event)

					if addr == REG_LC_DATA and len(payload) >= struct.calcsize(LC_DATA_FMT):
						f0, f1, f2, f3, total, cop_x, cop_y = struct.unpack(
							LC_DATA_FMT, payload[: struct.calcsize(LC_DATA_FMT)]
						)
						data_events += 1
						print(
							f"    EVENT LC_DATA: total={total:8.3f} g "
							f"cop=({cop_x:7.2f}, {cop_y:7.2f}) mm "
							f"f=[{f0:7.3f}, {f1:7.3f}, {f2:7.3f}, {f3:7.3f}]"
						)
					elif addr == REG_LC_EVENT and args.include_lc_event and payload:
						occ_events += 1
						print(f"    EVENT LC_EVENT: occupied={int(payload[0])}")
					# Non-LC_DATA events are discarded unless --include-lc-event is enabled.

				time.sleep(0.05)

			if args.include_lc_event:
				print(f"    stream summary: LC_DATA={data_events}, LC_EVENT={occ_events}")
			else:
				print(f"    stream summary: LC_DATA={data_events}")

		if direct_nonzero == 0:
			print("\nNOTE: all direct LC_DATA totals were zero.")
			print("      Check firmware wiring/init path for loadcell acquisition")
			print("      (LoadCellAppRegs::init(...) and LoadCellArray::start()).")

	print("\nLoadcell test complete.")


if __name__ == "__main__":
	main()
