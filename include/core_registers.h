#ifndef REGISTERS_H
#define REGISTERS_H
#include <stdint.h>
#include <reg_types.h>
#include <core_reg_bits.h>
#include <cstring>  // for strcpy

// Latest protocol core register block: addresses 0-19.
static const uint8_t CORE_REG_COUNT = 20;

// Harp protocol version implemented by this core.
inline constexpr uint8_t HARP_PROTOCOL_VERSION_MAJOR = 1;
inline constexpr uint8_t HARP_PROTOCOL_VERSION_MINOR = 13;
inline constexpr uint8_t HARP_PROTOCOL_VERSION_PATCH = 0;

// Core transport extensions are mapped into vendor/app space.
static const uint8_t CORE_EXTENSION_START_ADDRESS = 32;
static const uint8_t CORE_EXTENSION_REG_COUNT = 5;

#define APP_REG_START_ADDRESS (CORE_EXTENSION_START_ADDRESS + CORE_EXTENSION_REG_COUNT)

/**
 * \brief enum for easier interpretation of the OP_MODE bitfield in the
 *  R_OPERATION_CTRL register.
 */
enum op_mode_t: uint8_t
{
    STANDBY = 0,
    ACTIVE = 1,
    RESERVED = 2,
    SPEED = 3
};

/**
 * \brief enum where the name is the name of the register and the
 *        value is the address according to the harp protocol spec.
 */
enum RegName : uint8_t
{
    WHO_AM_I = 0,          // required
    HW_VERSION_H = 1,      // deprecated
    HW_VERSION_L = 2,      // deprecated
    ASSEMBLY_VERSION = 3,  // deprecated
    CORE_VERSION_H = 4,    // deprecated
    CORE_VERSION_L = 5,    // deprecated
    FW_VERSION_H = 6,      // deprecated
    FW_VERSION_L = 7,      // deprecated
    TIMESTAMP_SECOND = 8,  // required
    TIMESTAMP_MICRO = 9,   // required
    OPERATION_CTRL = 10,   // required
    RESET_DEV = 11,        // optional
    DEVICE_NAME = 12,      // optional
    SERIAL_NUMBER = 13,    // deprecated
    CLOCK_CONFIG = 14,     // optional
    TIMESTAMP_OFFSET = 15, // deprecated
    UID = 16,              // optional
    TAG = 17,              // optional
    HEARTBEAT = 18,        // required
    VERSION = 19,          // required

    // Core transport extension registers (vendor/app space).
    NET_SSID = CORE_EXTENSION_START_ADDRESS,
    NET_PASSWORD,
    NET_SERVER_IP,         // IP(16)
    NET_SERVER_PORT,       // port(2)
    NET_CONFIG,            // enable bits + status bits
};


// Byte-align struct data so we can send it out serially byte-by-byte.
#pragma pack(push, 1)
struct RegValues
{
    const uint16_t R_WHO_AM_I;
    const uint8_t R_HW_VERSION_H;
    const uint8_t R_HW_VERSION_L;
    const uint8_t R_ASSEMBLY_VERSION;
    const uint8_t R_CORE_VERSION_H;
    const uint8_t R_CORE_VERSION_L ;
    const uint8_t R_FW_VERSION_H;
    const uint8_t R_FW_VERSION_L;
    volatile uint32_t R_TIMESTAMP_SECOND;
    volatile uint16_t R_TIMESTAMP_MICRO;
    volatile uint8_t R_OPERATION_CTRL;
    volatile uint8_t R_RESET_DEV;
    volatile char R_DEVICE_NAME[25];
    volatile uint16_t R_SERIAL_NUMBER;
    volatile uint8_t R_CLOCK_CONFIG;
    volatile uint8_t R_TIMESTAMP_OFFSET;
    uint8_t R_UID[16];
    uint8_t R_TAG[8];
    volatile uint16_t R_HEARTBEAT;
    uint8_t R_VERSION[32];

    // Core transport-extension registers (not part of standard core dump/map).
    volatile char     R_NET_SSID[32];
    volatile char     R_NET_PASSWORD[64];
    volatile uint8_t  R_NET_SERVER_IP[16]; 
    volatile uint8_t  R_NET_SERVER_PORT[2];
    volatile uint8_t  R_NET_CONFIG;
    // R_NET_CONFIG byte layout
    // bits [1:0] = enable  (bit0=wifi, bit1=tcp)
    // bits [5:2] = status  (bit2=cfg_valid, bit3=wifi_up, bit4=ip_ok, bit5=tcp_conn)
    // bit  6     = apply   (write 1 to save+connect)
    // bit  7     = clear   (write 1 to erase+disconnect)
};
#pragma pack(pop)

struct RegSpecs
{
    volatile uint8_t* base_ptr;
    uint8_t num_bytes;
    reg_type_t payload_type;
};

struct Registers
{
    using enum reg_type_t;

    public:
        Registers(uint16_t who_am_i,
                  uint8_t hw_version_major, uint8_t hw_version_minor,
                  uint8_t assembly_version,
                  uint8_t fw_version_major, uint8_t fw_version_minor,
                  uint16_t serial_number, const char name[],
                  const uint8_t tag[]);
        [[deprecated("harp_version_major/minor are ignored; protocol version is compile-time fixed")]]
        Registers(uint16_t who_am_i,
              uint8_t hw_version_major, uint8_t hw_version_minor,
              uint8_t assembly_version,
              uint8_t harp_version_major, uint8_t harp_version_minor,
              uint8_t fw_version_major, uint8_t fw_version_minor,
              uint16_t serial_number, const char name[],
              const uint8_t tag[]);
        ~Registers();

    RegValues regs_;

    // Lookup table. Necessary because register data is not of equal size,
    //  so we can't index into it directly by enum.
    // TODO: consider generating this table statically with a template.
    const RegSpecs address_to_specs[CORE_REG_COUNT] =
    {{(uint8_t*)&regs_.R_WHO_AM_I,         sizeof(regs_.R_WHO_AM_I),         U16},
     {(uint8_t*)&regs_.R_HW_VERSION_H,     sizeof(regs_.R_HW_VERSION_H),      U8},
     {(uint8_t*)&regs_.R_HW_VERSION_L,     sizeof(regs_.R_HW_VERSION_L),      U8},
     {(uint8_t*)&regs_.R_ASSEMBLY_VERSION, sizeof(regs_.R_ASSEMBLY_VERSION),  U8},
     {(uint8_t*)&regs_.R_CORE_VERSION_H,   sizeof(regs_.R_CORE_VERSION_H),    U8},
     {(uint8_t*)&regs_.R_CORE_VERSION_L,   sizeof(regs_.R_CORE_VERSION_L),    U8},
     {(uint8_t*)&regs_.R_FW_VERSION_H,     sizeof(regs_.R_FW_VERSION_H),      U8},
     {(uint8_t*)&regs_.R_FW_VERSION_L,     sizeof(regs_.R_FW_VERSION_L),      U8},
     {(uint8_t*)&regs_.R_TIMESTAMP_SECOND, sizeof(regs_.R_TIMESTAMP_SECOND),  U32},
     {(uint8_t*)&regs_.R_TIMESTAMP_MICRO,  sizeof(regs_.R_TIMESTAMP_MICRO),   U16},
     {(uint8_t*)&regs_.R_OPERATION_CTRL,   sizeof(regs_.R_OPERATION_CTRL),    U8},
     {(uint8_t*)&regs_.R_RESET_DEV,        sizeof(regs_.R_RESET_DEV),         U8},
     {(uint8_t*)&regs_.R_DEVICE_NAME,      sizeof(regs_.R_DEVICE_NAME),       U8},
     {(uint8_t*)&regs_.R_SERIAL_NUMBER,    sizeof(regs_.R_SERIAL_NUMBER),     U16},
     {(uint8_t*)&regs_.R_CLOCK_CONFIG,     sizeof(regs_.R_CLOCK_CONFIG),      U8},
     {(uint8_t*)&regs_.R_TIMESTAMP_OFFSET, sizeof(regs_.R_TIMESTAMP_OFFSET),  U8},
     {(uint8_t*)&regs_.R_UID,              sizeof(regs_.R_UID),               U8},
     {(uint8_t*)&regs_.R_TAG,              sizeof(regs_.R_TAG),               U8},
     {(uint8_t*)&regs_.R_HEARTBEAT,        sizeof(regs_.R_HEARTBEAT),        U16},
     {(uint8_t*)&regs_.R_VERSION,          sizeof(regs_.R_VERSION),           U8},
    };

    // Core transport extension lookup table for NET registers(vendor/app space).
    const RegSpecs core_extension_address_to_specs[CORE_EXTENSION_REG_COUNT] =
    {{(uint8_t*)&regs_.R_NET_SSID,         sizeof(regs_.R_NET_SSID),         U8},
     {(uint8_t*)&regs_.R_NET_PASSWORD,     sizeof(regs_.R_NET_PASSWORD),     U8},
     {(uint8_t*)&regs_.R_NET_SERVER_IP,    sizeof(regs_.R_NET_SERVER_IP),    U8},
     {(uint8_t*)&regs_.R_NET_SERVER_PORT,  sizeof(regs_.R_NET_SERVER_PORT), U16},
     {(uint8_t*)&regs_.R_NET_CONFIG,       sizeof(regs_.R_NET_CONFIG),       U8},
    };

};

#endif //REGISTERS_H
