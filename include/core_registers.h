#ifndef REGISTERS_H
#define REGISTERS_H
#include <stdint.h>
#include <reg_types.h>
#include <core_reg_bits.h>
#include <cstring>  // for strcpy

// 18 registers for standard implementation
// 19 - 22 claimed for network config
static const uint8_t CORE_REG_COUNT = 23;

#define APP_REG_START_ADDRESS (32)

// R_OPERATION_CTRL bitfields.
#define DUMP_OFFSET (3)
#define MUTE_RPL_OFFSET (4)
#define VISUAL_EN_OFFSET (5)
#define OPLEDEN_OFFSET (6)
#define ALIVE_EN_OFFSET (7)

// RESET_DEV bitfields
#define RST_DEV_OFFSET (0)
#define RST_DFU_OFFSET (5)
#define BOOT_DEF_OFFSET (6)
#define BOOT_EE_OFFSET (7)

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
    WHO_AM_I = 0,
    HW_VERSION_H = 1, // major hardware version
    HW_VERSION_L = 2, // minor hardware version
    ASSEMBLY_VERSION = 3,
    HARP_VERSION_H = 4,
    HARP_VERSION_L = 5,
    FW_VERSION_H = 6,
    FW_VERSION_L = 7,
    TIMESTAMP_SECOND = 8,
    TIMESTAMP_MICRO = 9,
    OPERATION_CTRL = 10,
    RESET_DEF = 11,
    DEVICE_NAME = 12,
    SERIAL_NUMBER = 13,
    CLOCK_CONFIG = 14,
    TIMESTAMP_OFFSET = 15,
    UUID = 16,
    TAG = 17,
    NET_SSID        = 18,
    NET_PASSWORD    = 19,
    NET_SERVER_IP = 20,  // IP(16)
    NET_SERVER_PORT = 21,  // port(2)
    NET_CONFIG      = 22,  // enable bits + status bits
};


// Byte-align struct data so we can send it out serially byte-by-byte.
#pragma pack(push, 1)
struct RegValues
{
    const uint16_t R_WHO_AM_I;
    const uint8_t R_HW_VERSION_H;
    const uint8_t R_HW_VERSION_L;
    const uint8_t R_ASSEMBLY_VERSION;
    const uint8_t R_HARP_VERSION_H;
    const uint8_t R_HARP_VERSION_L ;
    const uint8_t R_FW_VERSION_H;
    const uint8_t R_FW_VERSION_L;
    volatile uint32_t R_TIMESTAMP_SECOND;
    volatile uint16_t R_TIMESTAMP_MICRO;
    volatile uint8_t R_OPERATION_CTRL;
    volatile uint8_t R_RESET_DEF;
    volatile char R_DEVICE_NAME[25];
    volatile uint16_t R_SERIAL_NUMBER;
    volatile uint8_t R_CLOCK_CONFIG;
    volatile uint8_t R_TIMESTAMP_OFFSET;
    uint8_t R_UUID[16];
    uint8_t R_TAG[8];
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
    {{(uint8_t*)&regs_.R_WHO_AM_I,         sizeof(regs_.R_WHO_AM_I),          U16},
     {(uint8_t*)&regs_.R_HW_VERSION_H,     sizeof(regs_.R_HW_VERSION_H),      U8},
     {(uint8_t*)&regs_.R_HW_VERSION_L,     sizeof(regs_.R_HW_VERSION_L),      U8},
     {(uint8_t*)&regs_.R_ASSEMBLY_VERSION, sizeof(regs_.R_ASSEMBLY_VERSION),  U8},
     {(uint8_t*)&regs_.R_HARP_VERSION_H,   sizeof(regs_.R_HARP_VERSION_H),    U8},
     {(uint8_t*)&regs_.R_HARP_VERSION_L,   sizeof(regs_.R_HW_VERSION_L),      U8},
     {(uint8_t*)&regs_.R_FW_VERSION_H,     sizeof(regs_.R_FW_VERSION_H),      U8},
     {(uint8_t*)&regs_.R_FW_VERSION_L,     sizeof(regs_.R_FW_VERSION_L),      U8},
     {(uint8_t*)&regs_.R_TIMESTAMP_SECOND, sizeof(regs_.R_TIMESTAMP_SECOND),  U32},
     {(uint8_t*)&regs_.R_TIMESTAMP_MICRO,  sizeof(regs_.R_TIMESTAMP_MICRO),   U16},
     {(uint8_t*)&regs_.R_OPERATION_CTRL,   sizeof(regs_.R_OPERATION_CTRL),    U8},
     {(uint8_t*)&regs_.R_RESET_DEF,        sizeof(regs_.R_RESET_DEF),         U8},
     {(uint8_t*)&regs_.R_DEVICE_NAME,      sizeof(regs_.R_DEVICE_NAME),       U8},
     {(uint8_t*)&regs_.R_SERIAL_NUMBER,    sizeof(regs_.R_SERIAL_NUMBER),     U16},
     {(uint8_t*)&regs_.R_CLOCK_CONFIG,     sizeof(regs_.R_CLOCK_CONFIG),      U8},
     {(uint8_t*)&regs_.R_TIMESTAMP_OFFSET, sizeof(regs_.R_TIMESTAMP_OFFSET),  U8},
     {(uint8_t*)&regs_.R_UUID,             sizeof(regs_.R_UUID),  U8},
     {(uint8_t*)&regs_.R_TAG,              sizeof(regs_.R_TAG),  U8},
     {(uint8_t*)regs_.R_NET_SSID,          sizeof(regs_.R_NET_SSID),          U8},
     {(uint8_t*)regs_.R_NET_PASSWORD,      sizeof(regs_.R_NET_PASSWORD),      U8},
     {(uint8_t*)regs_.R_NET_SERVER_IP,     sizeof(regs_.R_NET_SERVER_IP),     U8},
     {(uint8_t*)&regs_.R_NET_SERVER_PORT,  sizeof(regs_.R_NET_SERVER_PORT),   U16},
     {(uint8_t*)&regs_.R_NET_CONFIG,       sizeof(regs_.R_NET_CONFIG),        U8},
    };

    // Syntactic Sugar. Make bitfields for certain registers easier to access.
    OperationCtrlBits& r_operation_ctrl_bits = *((OperationCtrlBits*)(&regs_.R_OPERATION_CTRL));
    ResetDefBits& r_reset_def_bits = *((ResetDefBits*)(&regs_.R_RESET_DEF));
    ClockConfigBits& r_clock_config_bits = *((ClockConfigBits*)(&regs_.R_CLOCK_CONFIG));
};

#endif //REGISTERS_H
