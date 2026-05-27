#pragma once
// network_manager.h
// Manages Wi-Fi STA and TCP client lifecycle.
// Core-level network transport used by HarpCore.

#include <stdint.h>
#include <stddef.h>

#include "core_registers.h"

namespace NetworkManager {

// Initialise internal state and bind to core register storage.
// Call once during core startup.
void init(RegValues* regs);

// Start / restart Wi-Fi and TCP from currently bound core register values.
// Safe to call from the Harp task; actual connect work runs in network task.
void apply();

// Disconnect TCP and Wi-Fi without erasing config.
void disconnect();

// Best-effort send over current TCP socket (used by HarpCore replies).
void tcp_write(const uint8_t* data, size_t len);

// Non-blocking read from current TCP socket.
// Returns number of bytes read, 0 when no data is available, and -1 on
// disconnect/error (socket is closed internally in that case).
int tcp_read(uint8_t* data, size_t len);

// Status queries (thread-safe; read from atomic flags)
bool is_wifi_connected();
bool is_tcp_connected();

// Returns the raw socket fd, or -1 if not connected.
// The Harp transport layer uses this to send TCP bytes.
int  tcp_socket();

} // namespace NetworkManager
