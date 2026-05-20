#pragma once

#include "harp_c_app.h"
#include "bb_loadcell.h"

namespace LoadCellAppRegs {

// ── Register addresses ───────────────────────────────────────────────────────
// Starts at offset 4 — after the 4 LED registers (32–35)
constexpr uint8_t LC_DATA_REL   = 4;   // addr 36 — continuous force stream
constexpr uint8_t LC_EVENT_REL  = 5;   // addr 37 — occupancy edge (entry/exit)
constexpr uint8_t LC_CONFIG_REL = 6;   // addr 38 — thresholds, debounce (R/W)

constexpr uint8_t LC_DATA   = APP_REG_START_ADDRESS + LC_DATA_REL;
constexpr uint8_t LC_EVENT  = APP_REG_START_ADDRESS + LC_EVENT_REL;
constexpr uint8_t LC_CONFIG = APP_REG_START_ADDRESS + LC_CONFIG_REL;

constexpr size_t REGISTER_COUNT = 3;

// ── R_LC_DATA payload ────────────────────────────────────────────────────────
// Sent as an EVENT when streaming is enabled.
// 4x float force_g + float total_g + float cop_x_mm + float cop_y_mm = 28 bytes
#pragma pack(push, 1)
struct LcDataPayload {
    float force_g[bb::LoadCellCount];  // per-cell force [g]
    float total_g;                     // sum [g]
    float cop_x_mm;                    // centre-of-pressure X [mm]
    float cop_y_mm;                    // centre-of-pressure Y [mm]
};

// ── R_LC_EVENT payload ───────────────────────────────────────────────────────
// 0 = platform vacated, 1 = platform occupied
struct LcEventPayload {
    uint8_t occupied;                  // 0 or 1
};

// ── R_LC_CONFIG payload (R/W) ────────────────────────────────────────────────
struct LcConfigPayload {
    float    entry_threshold_g;        // force to declare entry  [g]
    float    exit_threshold_g;         // force to declare exit   [g]
    uint16_t debounce_frames;          // consecutive frames before commit
    uint8_t  stream_enable;            // 1 = emit R_LC_DATA events, 0 = silent
};
#pragma pack(pop)

// ── Register storage ─────────────────────────────────────────────────────────
struct RegValues {
    LcDataPayload   R_LC_DATA;
    LcEventPayload  R_LC_EVENT;
    LcConfigPayload R_LC_CONFIG;
};

extern RegValues values;
extern RegSpecs  specs[REGISTER_COUNT];
extern RegFnPair functions[REGISTER_COUNT];

// ── Lifecycle ─────────────────────────────────────────────────────────────────
// Called by combined_update() / combined_reset() in main.cpp
void init(bb::LoadCellArray& lc);  // must be called once before update()
void update();
void reset();

// ── Write handlers ────────────────────────────────────────────────────────────
void write_lc_config(msg_t& msg);

} // namespace LoadCellAppRegs