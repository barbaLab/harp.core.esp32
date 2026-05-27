# harp.core.esp32

An ESP32-S3 Harp Core implementation that uses ESP-IDF and TinyUSB to serve as the basis of a custom Harp device.

## Using this Library from an App Repo

This repository is intended to be consumed by application-specific firmware repos,
using the same reusable core/app split used across Harp core projects.

The core component lives in `src/` and can be referenced as a Component Manager
git dependency from an app project's `idf_component.yml`.

Example (`<app-repo>/main/idf_component.yml`):

```yaml
dependencies:
	harp_core_esp32:
		git: https://github.com/barbaLab/harp.core.esp32.git
		path: src
		version: main
```

In the app repo:

1. Keep app-specific registers and handlers in the app repo (not in this repo).
2. Initialize `HarpCApp` (or another `HarpCore`-derived class) in the app repo's
	 `app_main`.
3. Call `run()` periodically in the app task loop.

## Features

* Parsing incoming Harp messages.
* Dispatching messages to the appropriate register handlers.
* Sending Harp-compliant timestamped replies.
* Handling Harp messages over both TinyUSB CDC and Wi-Fi TCP.
* Core-native Wi-Fi STA + TCP client lifecycle management (`NetworkManager`).
* Updating Harp time from an external UART synchronization signal.
* Running with ESP-IDF logging and PlatformIO on ESP32-S3.
## Current Topology

This firmware can use two transport paths in the current board setup:

* Harp protocol traffic over TinyUSB CDC.
* Harp protocol traffic over Wi-Fi TCP (ESP32 connects outbound to a server IP/port).
* Firmware logs go over the ESP-IDF console path.

On the current hardware this typically shows up as two COM ports in Windows. The COM numbers are assigned by Windows; the firmware only defines the functions.
## Wi-Fi / TCP Control Path

Network configuration is exposed as Harp registers and can be provisioned over USB,
then used for runtime TCP communication:

* `R_NET_SSID` (18): null-terminated SSID buffer.
* `R_NET_PASSWORD` (19): null-terminated password buffer (reads are masked).
* `R_NET_SERVER_IP` (20): server IPv4 string (for example `192.168.137.1`).
* `R_NET_SERVER_PORT` (21): little-endian TCP port.
* `R_NET_CONFIG` (22):
	* bits `[1:0]` enable flags (`wifi`, `tcp`)
	* bits `[5:2]` status bits (`cfg_valid`, `wifi_up`, `ip_ok`, `tcp_conn`)
	* bit `6` apply
	* bit `7` clear/disconnect

Implementation notes:

* `src/network_manager.cpp` initializes NVS, netif/event loop, Wi-Fi STA, and TCP reconnect logic.
* Wi-Fi power save is disabled with `WIFI_PS_NONE` for better latency stability.
* Reconnect races are handled defensively (`ESP_ERR_WIFI_CONN` is treated as non-fatal).
* `src/harp_core.cpp` transmits replies over both CDC and TCP and parses inbound messages from both paths.
## Current Implementation

* `examples/minimal_platformio/src/main.cpp` initializes `HarpCore`, optionally attaches `HarpSynchronizer`, and runs the Harp loop for local development/testing.
* `src/harp_core.cpp` implements message parsing, register access, heartbeat handling, and reply generation.
* `src/network_manager.cpp` implements Wi-Fi event handling, TCP connect/reconnect, and status propagation via `R_NET_CONFIG`.
* `src/harp_synchronizer.cpp` listens on a UART RX pin and updates Harp time from external sync packets.
* `src/usb_descriptors.c` defines the TinyUSB CDC descriptors used for the Harp device channel.
## Using This Project

The simplest way to work with this project is to use the dev app under `examples/minimal_platformio`, then talk to it with the Python Harp client over the Harp CDC COM port.

The test scripts under `test/` are the primary examples of how to interact with the device.

For end-to-end Wi-Fi/TCP validation, use `examples/minimal_platformio/test/test_harp_tcp.py`.
The script provisions network registers over USB first, then accepts the inbound
TCP connection from the ESP32 and runs:

1. `get_info`
2. `enable_heartbeat`
3. `test_reply_speed`

Example:

```bash
cd examples/minimal_platformio
python test/test_harp_tcp.py --ssid <ssid> --pwd <password> --server-ip <host-ip> --server-port 9999
```

On Windows Mobile Hotspot, the host IP is typically `192.168.137.1`.
### Python API

The installed Python package in this workspace uses the `harp` namespace.

For heartbeat tests, the current API is:

* `Device.alive_en(True)` to enable heartbeat events.
* `Device.get_events()` to read queued events.
* `Device.set_mode(OperationMode.ACTIVE)` to enter active mode when needed.
## Building the Firmware

Use PlatformIO for the ESP32-S3 board profile.

```bash
cd examples/minimal_platformio
C:\Users\fedba\.platformio\penv\Scripts\platformio.exe run
```
## Flashing the Firmware

```bash
C:\Users\fedba\.platformio\penv\Scripts\platformio.exe run --target upload
```

If your board exposes separate COM ports, use the Harp CDC port for Python communication and the console port for logs.
## Monitoring

```bash
C:\Users\fedba\.platformio\penv\Scripts\platformio.exe device monitor
```

The monitor should show the startup logs emitted from `examples/minimal_platformio/src/main.cpp`.
## Developer Notes

### Bumping the Version

Two version groups are tracked in the code:

* `HARP_VERSION_MAJOR`, `HARP_VERSION_MINOR`, `HARP_VERSION_PATCH`
	* Tracks the Harp protocol version that this implementation most closely follows.
	* Bump this when the protocol behavior changes.

* `ESP32_CORE_VERSION_MAJOR`, `ESP32_CORE_VERSION_MINOR`, `ESP32_CORE_VERSION_PATCH`
	* Tracks this project's own implementation version.
	* Bump this when the firmware behavior or API changes.
	* `PICO_CORE_VERSION_*` aliases are still available for backward compatibility.
### Debugging

The firmware uses ESP-IDF logging. `examples/minimal_platformio/src/main.cpp` emits startup logs, and `src/harp_core.cpp` contains additional debug traces around message receive and transmit paths.

### Optional Synchronization

`HarpSynchronizer` can be enabled from `examples/minimal_platformio/src/main.cpp` by turning on the UART sync path and setting the correct RX pin and UART port for your hardware.
## Tests

The scripts in `test/` exercise the device over the Harp Python package.

The heartbeat test currently uses:

* `alive_en(True)` to enable the heartbeat bit.
* `get_events()` to collect heartbeat events.

`test/test_harp_tcp.py` additionally reports TCP round-trip latency statistics and
can save a histogram plot for long-run comparison (`N=10000` style tests).

## Latency Tuning Notes

Observed during validation:

* Dedicated/local AP setup produced the best Wi-Fi tail latency in this project.
* `WIFI_PS_NONE` reduced worst-case spikes versus default Wi-Fi power save behavior.
* Pinning the core protocol task to CPU1 (`examples/minimal_platformio/src/main.cpp`)
	helped reduce contention with Wi-Fi work commonly running on CPU0.
## References

* [Harp Protocol Repo](https://github.com/harp-tech/protocol)
* [harp Python package](https://github.com/harp-tech/pyharp)
