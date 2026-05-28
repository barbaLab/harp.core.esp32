#ifndef CORE_REG_BITS_H
#define CORE_REG_BITS_H
#include <stdint.h>

// Canonical bitmask definitions for core control/status registers.
//
// Conventions:
// - *_MASK values are applied with bitwise operations (no bitfield overlays).
// - Multi-bit fields are represented by a field mask (for example OP_MODE bits [1:0]).
// - Combined masks (for example NET_CFG_ENABLE_MASK) are helpers for grouped updates.

// R_OPERATION_CTRL (uint8_t) bit layout
// [1:0] OP_MODE, [2] HEARTBEAT_EN, [3] DUMP, [4] MUTE_RPL,
// [5] VISUAL_EN, [6] OPLED_EN, [7] ALIVE_EN
static constexpr uint8_t OP_CTRL_OP_MODE_MASK      = 0x03; // bits [1:0]
static constexpr uint8_t OP_CTRL_HEARTBEAT_EN_MASK = 1u << 2;
static constexpr uint8_t OP_CTRL_DUMP_MASK         = 1u << 3;
static constexpr uint8_t OP_CTRL_MUTE_RPL_MASK     = 1u << 4;
static constexpr uint8_t OP_CTRL_VISUAL_EN_MASK    = 1u << 5;
static constexpr uint8_t OP_CTRL_OPLED_EN_MASK     = 1u << 6;
static constexpr uint8_t OP_CTRL_ALIVE_EN_MASK     = 1u << 7;

// R_RESET_DEV (uint8_t) bit layout
// [0] RST_DEF, [1] RST_EE, [2] SAVE, [3] NAME_TO_DEFAULT,
// [5] RST_DFU, [6] BOOT_DEF, [7] BOOT_EE
static constexpr uint8_t RESET_DEV_RST_DEF_MASK          = 1u << 0;
static constexpr uint8_t RESET_DEV_RST_EE_MASK           = 1u << 1;
static constexpr uint8_t RESET_DEV_SAVE_MASK             = 1u << 2;
static constexpr uint8_t RESET_DEV_NAME_TO_DEFAULT_MASK  = 1u << 3;
static constexpr uint8_t RESET_DEV_RST_DFU_MASK          = 1u << 5;
static constexpr uint8_t RESET_DEV_BOOT_DEF_MASK         = 1u << 6;
static constexpr uint8_t RESET_DEV_BOOT_EE_MASK          = 1u << 7;

// R_CLOCK_CONFIG (uint8_t) bit layout
// [0] CLK_REP, [1] CLK_GEN, [3] REP_ABLE, [4] GEN_ABLE, [6] UNLOCK, [7] LOCK
static constexpr uint8_t CLOCK_CFG_CLK_REP_MASK    = 1u << 0;
static constexpr uint8_t CLOCK_CFG_CLK_GEN_MASK    = 1u << 1;
static constexpr uint8_t CLOCK_CFG_REP_ABLE_MASK   = 1u << 3;
static constexpr uint8_t CLOCK_CFG_GEN_ABLE_MASK   = 1u << 4;
static constexpr uint8_t CLOCK_CFG_UNLOCK_MASK     = 1u << 6;
static constexpr uint8_t CLOCK_CFG_LOCK_MASK       = 1u << 7;

// R_NET_CONFIG (uint8_t) bit layout
// [0] ENABLE_WIFI, [1] ENABLE_TCP,
// [2] STATUS_CFG_VALID, [3] STATUS_WIFI_UP, [4] STATUS_IP_OK, [5] STATUS_TCP_CONN,
// [6] APPLY, [7] CLEAR
//
// Note: status bits are typically written by firmware and read by host.
// APPLY and CLEAR act as command bits consumed by firmware logic.
static constexpr uint8_t NET_CFG_ENABLE_WIFI_MASK  = 1u << 0;
static constexpr uint8_t NET_CFG_ENABLE_TCP_MASK   = 1u << 1;
static constexpr uint8_t NET_CFG_ENABLE_MASK       = NET_CFG_ENABLE_WIFI_MASK | NET_CFG_ENABLE_TCP_MASK;
static constexpr uint8_t NET_CFG_STATUS_CFG_VALID_MASK = 1u << 2;
static constexpr uint8_t NET_CFG_STATUS_WIFI_UP_MASK   = 1u << 3;
static constexpr uint8_t NET_CFG_STATUS_IP_OK_MASK     = 1u << 4;
static constexpr uint8_t NET_CFG_STATUS_TCP_CONN_MASK  = 1u << 5;
static constexpr uint8_t NET_CFG_STATUS_MASK       =
    NET_CFG_STATUS_CFG_VALID_MASK |
    NET_CFG_STATUS_WIFI_UP_MASK |
    NET_CFG_STATUS_IP_OK_MASK |
    NET_CFG_STATUS_TCP_CONN_MASK;
static constexpr uint8_t NET_CFG_APPLY_MASK        = 1u << 6;
static constexpr uint8_t NET_CFG_CLEAR_MASK        = 1u << 7;

#endif // CORE_REG_BITS_H
