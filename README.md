# harp.core.esp32

An ESP32-S3 Harp Core implementation that uses ESP-IDF and TinyUSB to serve as the basis of a custom Harp device.
## Features

* Parsing incoming Harp messages.
* Dispatching messages to the appropriate register handlers.
* Sending Harp-compliant timestamped replies.
* Updating Harp time from an external UART synchronization signal.
* Running with ESP-IDF logging and PlatformIO on ESP32-S3.
## Current Topology

This firmware uses two separate interfaces in the current board setup:

* Harp protocol traffic goes over the TinyUSB CDC device interface.
* Firmware logs go over the ESP-IDF console path.

On the current hardware this typically shows up as two COM ports in Windows. The COM numbers are assigned by Windows; the firmware only defines the functions.
## Current Implementation

* `src/main.cpp` initializes `HarpCore`, optionally attaches `HarpSynchronizer`, and runs the Harp loop.
* `src/harp_core.cpp` implements message parsing, register access, heartbeat handling, and reply generation.
* `src/harp_synchronizer.cpp` listens on a UART RX pin and updates Harp time from external sync packets.
* `src/usb_descriptors.c` defines the TinyUSB CDC descriptors used for the Harp device channel.
## Using This Project

The simplest way to work with this project is to build and flash it with PlatformIO, then talk to it with the Python Harp client over the Harp CDC COM port.

The test scripts under `test/` are the primary examples of how to interact with the device.
### Python API

The installed Python package in this workspace uses the `harp` namespace.

For heartbeat tests, the current API is:

* `Device.alive_en(True)` to enable heartbeat events.
* `Device.get_events()` to read queued events.
* `Device.set_mode(OperationMode.ACTIVE)` to enter active mode when needed.
## Building the Firmware

Use PlatformIO for the ESP32-S3 board profile.

```bash
pio run
```
## Flashing the Firmware

```bash
pio run --target upload
```

If your board exposes separate COM ports, use the Harp CDC port for Python communication and the console port for logs.
## Monitoring

```bash
pio device monitor
```

The monitor should show the startup logs emitted from `src/main.cpp`.
## Developer Notes

### Bumping the Version

Two version groups are tracked in the code:

* `HARP_VERSION_MAJOR`, `HARP_VERSION_MINOR`, `HARP_VERSION_PATCH`
	* Tracks the Harp protocol version that this implementation most closely follows.
	* Bump this when the protocol behavior changes.
* `PICO_CORE_VERSION_MAJOR`, `PICO_CORE_VERSION_MINOR`, `PICO_CORE_VERSION_PATCH`
	* Tracks this project's own implementation version.
	* Bump this when the firmware behavior or API changes.
### Debugging

The firmware uses ESP-IDF logging. `src/main.cpp` emits startup logs, and `src/harp_core.cpp` contains additional debug traces around message receive and transmit paths.

### Optional Synchronization

`HarpSynchronizer` can be enabled from `src/main.cpp` by turning on the UART sync path and setting the correct RX pin and UART port for your hardware.
## Tests

The scripts in `test/` exercise the device over the Harp Python package.

The heartbeat test currently uses:

* `alive_en(True)` to enable the heartbeat bit.
* `get_events()` to collect heartbeat events.
## References

* [Harp Protocol Repo](https://github.com/harp-tech/protocol)
* [harp Python package](https://github.com/harp-tech/pyharp)
