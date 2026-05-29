# Harp Synchronization Implementation Notes (ESP32-S3 Core)

## Context

This note records the synchronization design decisions currently implemented in this core, and the rationale behind them.

It is intended for:

* maintainers of this repository
* original developers who need a precise status snapshot
* teams integrating this core into their own hardware architecture

The focus is the Harp synchronization path implemented in:

* `src/harp_synchronizer.cpp`
* `include/harp_synchronizer.h`
* `src/harp_core.cpp` (`R_CLOCK_CONFIG` behavior)
* `include/harp_core.h` (synchronizer attachment and capability propagation)

## Spec References Used

The implementation is aligned against:

* Harp Device core register specification at v1.13 (`R_CLOCK_CONFIG`, `R_HEARTBEAT`)
* Harp Synchronization Clock protocol:
  * 100 kbps
  * `0xAA 0xAF <U32 LE seconds>` frame format
  * final-byte timing semantics around the 672 microsecond reference

## Current Behavior Summary

As implemented now:

1. RX decode path uses UART at `100000` baud (`8N1`).
2. RX timing path uses capture-assisted edge timestamps on the same external sync pin.
3. RX packet parsing is still UART state-machine based (`AA`, `AF`, then 4 timestamp bytes).
4. Offset update uses captured timing data to reduce UART-ISR-latch uncertainty.
5. TX generation uses timer-started full-packet UART emission.
6. `R_CLOCK_CONFIG` mode bits (`CLK_GEN`, `CLK_REP`) are runtime-functional when TX capability exists.
7. Capability bits (`GEN_ABLE`, `REP_ABLE`) are reflected as read-only runtime capability, not host-controlled state.

## Timing Model

The baseline model used in this core is:

* `HARP_SYNC_OFFSET_US = 672`
* receiver-side alignment follows the published Harp receiver convention where the next whole second is derived from a 672 microsecond relation to the final packet byte event.

Implementation constants:

* `HARP_SYNC_OFFSET_US = 672`
* `HARP_SYNC_BYTE_TIME_US = 100` (8N1 at 100 kbps)

The RX path now combines:

* UART content validation
* capture-assisted edge timing selection

This separates payload trust from timing authority and materially reduces dependence on UART FIFO service latency.

## RX Path Details

### 1. Content decode

UART receives and parses the standard 6-byte frame:

* `0xAA`
* `0xAF`
* `U32` seconds, little-endian

### 2. Timing capture

The same external RX signal is also monitored by a GPIO edge-capture ISR.

Captured edge timestamps are kept in a ring buffer. After a valid UART frame is assembled, the synchronizer selects the best candidate edge corresponding to the final-byte start region and derives the Harp offset from that point.

### 3. Offset update

After decoding a valid frame:

* encoded seconds are interpreted as previous second (`+1` applied)
* timestamp anchor is taken from capture-assisted edge timing
* offset is updated so Harp time remains derived from `esp_timer_get_time()` with reduced software-latch jitter contribution

## TX Generation Details

TX generation is optional and enabled only when a TX pin is configured.

Generation path:

1. local schedule is computed in Harp time domain
2. esp_timer callback is armed for the packet start instant
3. timer callback writes a full 6-byte packet to UART FIFO

The packet format remains standard Harp sync format (`AA AF U32`).

## `R_CLOCK_CONFIG` Runtime Semantics

### Capability bits

`GEN_ABLE` and `REP_ABLE` are now runtime-derived from whether sync TX is actually supported in the active synchronizer configuration.

### Mode bits

When TX capability exists:

* `CLK_GEN` enables generator mode
* `CLK_REP` enables repeater mode

When TX capability does not exist:

* mode bits are not accepted as active behavior
* capability bits remain cleared

### Generator behavior (`CLK_GEN`)

In generator mode:

* timer-started TX scheduling is active
* incoming sync messages are ignored on RX, matching generator semantics
* synchronized status is treated as active from generator state

### Repeater behavior (`CLK_REP`)

In repeater mode:

* after a valid received sync frame, an outbound sync frame is emitted on the configured sync TX output
* current repeater behavior regenerates from decoded seconds and local output path

## Heartbeat / Sync State

`R_HEARTBEAT.IS_SYNCHRONIZED` is driven from synchronizer state.

With current behavior:

* synchronized on valid RX lock
* synchronized while generator mode is active (`CLK_GEN`)

This keeps `IS_SYNCHRONIZED` consistent with both receiver and generator operating modes.

## Current Compliance Status

### Implemented and aligned

* sync frame decode format and baud rate
* corrected 672 microsecond baseline timing model
* capture-assisted RX timing on same external signal
* timer-started full-packet UART TX generation
* functional `CLK_GEN` and `CLK_REP` mode semantics
* runtime-read-only capability bits
* generator input-ignore behavior

### Still architecture-sensitive

The exact sub-millisecond error budget remains hardware- and load-dependent and requires bench characterization on each board revision.

Key contributors:

* capture edge quality on conditioned input
* ISR and timer dispatch latency under load
* UART launch latency from timer callback to on-wire transition
* board-level propagation and line conditioning delay

## Integration Notes For Other Architectures

For teams using this core concept on other MCUs or SoCs, the most important architectural rule is preserved here:

* decode path and timing path are separate concerns.

Porting guidance:

1. Keep UART for packet decode and validation.
2. Add a hardware-timed capture path on the same external signal.
3. Anchor local sync offset to capture timing, not UART callback completion.
4. Use a deterministic timer-based scheduler for TX packet launch.
5. Keep `R_CLOCK_CONFIG` mode bits bound to real runtime capabilities.

Equivalent peripheral classes by platform:

* AVR: timer compare / overflow + direct UART register service
* RP2040: PIO for deterministic signal timing, UART for content decode if desired
* ESP32-S3: UART + capture ISR/RMT-style timing + esp_timer scheduling

## Validation Checklist

Minimum validation performed per board or major firmware timing change:

1. verify line encoding (`100 kbps`, `8N1`, `AA AF U32`)
2. verify sequential second decode and lock behavior
3. measure alignment between final-byte timing reference and local second rollover
4. stress test under realistic USB/TCP/Wi-Fi workload
5. verify `CLK_GEN` and `CLK_REP` mode transitions through `R_CLOCK_CONFIG`
6. verify `R_HEARTBEAT.IS_SYNCHRONIZED` in receiver, generator, and loss conditions

Recorded metrics should include:

* fixed timing offset estimate
* peak-to-peak jitter under idle and load
* lock / relock behavior after disturbances

## Known Open Items

The following items are still open for hardening:

1. stricter sequential-second gating before repeater retransmit
2. explicit sync-loss timeout policy and transition handling
3. bench characterization of capture window constants used for edge selection

## Practical Defaults For New Integrations

If this core is reused in a new architecture, the default practical profile is:

* keep external single-wire Harp sync format unchanged
* split decode and timing internally
* keep generation on a single TX peripheral path (full-packet launch)
* expose mode control via `R_CLOCK_CONFIG` only when capability exists

This keeps the Harp interface stable while preserving timing determinism work inside the device.
