#include "app_loadcell.h"

namespace LoadCellAppRegs {

using enum reg_type_t;

// ── Defaults ──────────────────────────────────────────────────────────────────
namespace {
    constexpr float    kDefaultEntryThreshold = 20.f;   // g
    constexpr float    kDefaultExitThreshold  = 10.f;   // g
    constexpr uint16_t kDefaultDebounceFrames = 8;      // ~100 ms at 80 Hz
    constexpr uint8_t  kDefaultStreamEnable   = 0;      // off by default

    bb::LoadCellArray* g_lc = nullptr;

    // Occupancy state machine
    bool     g_occupied      = false;
    uint16_t g_stable_count  = 0;
    bool     g_pending_state = false;
}

// ── Register storage ──────────────────────────────────────────────────────────
RegValues values {
    .R_LC_DATA   = {},
    .R_LC_EVENT  = { .occupied = 0 },
    .R_LC_CONFIG = {
        .entry_threshold_g = kDefaultEntryThreshold,
        .exit_threshold_g  = kDefaultExitThreshold,
        .debounce_frames   = kDefaultDebounceFrames,
        .stream_enable     = kDefaultStreamEnable,
    },
};

RegSpecs specs[REGISTER_COUNT] = {
    { (uint8_t*)&values.R_LC_DATA,   sizeof(values.R_LC_DATA),   U8 },  // raw bytes
    { (uint8_t*)&values.R_LC_EVENT,  sizeof(values.R_LC_EVENT),  U8 },
    { (uint8_t*)&values.R_LC_CONFIG, sizeof(values.R_LC_CONFIG), U8 },
};

RegFnPair functions[REGISTER_COUNT] = {
    { &HarpCore::read_reg_generic, nullptr           },  // LC_DATA: read only
    { &HarpCore::read_reg_generic, nullptr           },  // LC_EVENT: read only
    { &HarpCore::read_reg_generic, &write_lc_config  },  // LC_CONFIG: R/W
};

// ── Lifecycle ─────────────────────────────────────────────────────────────────
void init(bb::LoadCellArray& lc) {
    g_lc = &lc;
}

void reset() {
    values.R_LC_CONFIG.entry_threshold_g = kDefaultEntryThreshold;
    values.R_LC_CONFIG.exit_threshold_g  = kDefaultExitThreshold;
    values.R_LC_CONFIG.debounce_frames   = kDefaultDebounceFrames;
    values.R_LC_CONFIG.stream_enable     = kDefaultStreamEnable;
    values.R_LC_EVENT.occupied           = 0;
    g_occupied     = false;
    g_stable_count = 0;
}

void update() {
    if (g_lc == nullptr) return;

    bb::LoadCellSample s;
    // Non-blocking — returns immediately if no new sample
    if (!g_lc->getSample(s, 0)) return;

    // ── Pack data register ──────────────────────────────────────────────────
    for (size_t i = 0; i < bb::LoadCellCount; ++i)
        values.R_LC_DATA.force_g[i] = s.force_g[i];
    values.R_LC_DATA.total_g   = s.total_g;
    values.R_LC_DATA.cop_x_mm  = s.cop_x_mm;
    values.R_LC_DATA.cop_y_mm  = s.cop_y_mm;

    // ── Stream event (only if Bonsai subscribed and enabled) ────────────────
    if (values.R_LC_CONFIG.stream_enable) {
        HarpCore::send_harp_reply(EVENT, LC_DATA);
    }

    // ── Occupancy state machine with hysteresis + debounce ──────────────────
    const bool above_entry = s.total_g >= values.R_LC_CONFIG.entry_threshold_g;
    const bool below_exit  = s.total_g <  values.R_LC_CONFIG.exit_threshold_g;

    bool candidate = g_occupied;
    if (!g_occupied && above_entry) candidate = true;
    if ( g_occupied && below_exit)  candidate = false;

    if (candidate != g_occupied) {
        if (++g_stable_count >= values.R_LC_CONFIG.debounce_frames) {
            g_occupied     = candidate;
            g_stable_count = 0;
            values.R_LC_EVENT.occupied = g_occupied ? 1 : 0;
            // Occupancy edge always fires regardless of stream_enable
            HarpCore::send_harp_reply(EVENT, LC_EVENT);
        }
    } else {
        g_stable_count = 0;
    }
}

// ── Write handler ─────────────────────────────────────────────────────────────
void write_lc_config(msg_t& msg) {
    if (msg.header.payload_length() != sizeof(LcConfigPayload)) {
        HarpCore::send_harp_reply(WRITE_ERROR, msg.header.address);
        return;
    }
    HarpCore::copy_msg_payload_to_register(msg);
    // Clamp: exit threshold must be < entry threshold
    if (values.R_LC_CONFIG.exit_threshold_g >= values.R_LC_CONFIG.entry_threshold_g)
        values.R_LC_CONFIG.exit_threshold_g = values.R_LC_CONFIG.entry_threshold_g * 0.5f;
    HarpCore::send_harp_reply(WRITE, msg.header.address);
}

} // namespace LoadCellAppRegs