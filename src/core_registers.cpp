#include <core_registers.h>

Registers::Registers(uint16_t who_am_i,
                     uint8_t hw_version_major, uint8_t hw_version_minor,
                     uint8_t assembly_version,
                     uint8_t harp_version_major, uint8_t harp_version_minor,
                     uint8_t fw_version_major, uint8_t fw_version_minor,
                     uint16_t serial_number, const char name[],
                     const uint8_t tag[])
:regs_{.R_WHO_AM_I = who_am_i,
       .R_HW_VERSION_H = hw_version_major,
       .R_HW_VERSION_L = hw_version_minor,
       .R_ASSEMBLY_VERSION = assembly_version,
       .R_HARP_VERSION_H = harp_version_major,
       .R_HARP_VERSION_L = harp_version_minor,
       .R_FW_VERSION_H = fw_version_major,
       .R_FW_VERSION_L = fw_version_minor,
       .R_TIMESTAMP_SECOND = 0,
       .R_TIMESTAMP_MICRO = 0,
       .R_OPERATION_CTRL = 0,
       .R_RESET_DEF = 0,
       .R_DEVICE_NAME = {0},
       .R_SERIAL_NUMBER = serial_number,
       .R_CLOCK_CONFIG = 0,
       .R_TIMESTAMP_OFFSET = 0,
       .R_UUID = {0},
       .R_TAG = {0},
       .R_NET_SSID = {0},
       .R_NET_PASSWORD = {0},
       .R_NET_SERVER_IP = {0}, 
       .R_NET_SERVER_PORT = {uint8_t(9999u & 0xFFu), uint8_t((9999u >> 8) & 0xFFu)},            // Store default TCP port (9999) as little-endian bytes.
       .R_NET_CONFIG = 0x00
        }
{
    strcpy((char*)regs_.R_DEVICE_NAME, name);
    strcpy((char*)regs_.R_TAG, (char*)tag);
}


Registers::~Registers(){}
