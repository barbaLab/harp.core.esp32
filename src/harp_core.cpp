#include <harp_core.h>
#include <tinyusb.h>
#include <tinyusb_default_config.h>

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include <esp_log.h>

static const char* kHarpCoreLogTag = "HarpCore";

HarpCore& HarpCore::init(uint16_t who_am_i,
                         uint8_t hw_version_major, uint8_t hw_version_minor,
                         uint8_t assembly_version,
                         uint8_t harp_version_major, uint8_t harp_version_minor,
                         uint8_t fw_version_major, uint8_t fw_version_minor,
                         uint16_t serial_number, const char name[],
                         const uint8_t tag[])
{
    // Create the singleton instance using the private constructor.
    static HarpCore core(who_am_i, hw_version_major, hw_version_minor,
                         assembly_version,
                         harp_version_major, harp_version_minor,
                         fw_version_major, fw_version_minor, serial_number,
                         name, tag);
    return core;
}

HarpCore::HarpCore(uint16_t who_am_i,
                   uint8_t hw_version_major, uint8_t hw_version_minor,
                   uint8_t assembly_version,
                   uint8_t harp_version_major, uint8_t harp_version_minor,
                   uint8_t fw_version_major, uint8_t fw_version_minor,
                   uint16_t serial_number, const char name[],
                   const uint8_t tag[])
:new_msg_{false},
 set_visual_indicators_fn_{nullptr}, sync_{nullptr},
 regs_{who_am_i, hw_version_major, hw_version_minor, assembly_version,
       harp_version_major, harp_version_minor,
       fw_version_major, fw_version_minor, serial_number, name, tag},
 rx_buffer_index_{0}, total_bytes_read_{rx_buffer_index_},
 offset_us_64_{0},
 disconnect_handled_{false}, connect_handled_{false}, sync_handled_{false},
 heartbeat_interval_us_{HEARTBEAT_STANDBY_INTERVAL_US}
{
    // Create a pointer to the first (and one-and-only) instance created.
    if (self == nullptr)
        self = this;
    // Install TinyUSB via esp_tinyusb wrapper so RHPORT/device config is set
    // by ESP-IDF before the TinyUSB task starts.
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGD(kHarpCoreLogTag, "tinyusb_driver_install done");
    // Populate Harp Core R_UUID with the ESP32 base MAC address (6 bytes).
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    memcpy((void*)(&regs.R_UUID[8]), mac, sizeof(mac));
    // Initialize next heartbeat.
    update_next_heartbeat_from_curr_harp_time_us(harp_time_us_64());
}

HarpCore::~HarpCore(){self = nullptr;}

void HarpCore::run()
{
    // On ESP32-S3 TinyUSB runs in its own background task (started by
    // tusb_init()); tud_task() must NOT be called from application code.
    update_state();
    update_app_state(); // Does nothing unless a derived class implements it.
    process_cdc_input();
    if (not new_msg_)
        return;
#ifdef DEBUG_HARP_MSG_IN
    msg_t msg = get_buffered_msg();
    printf("Msg data: \r\n");
    printf("  type: %d\r\n", msg.header.type);
    printf("  addr: %d\r\n", msg.header.address);
    printf("  raw len: %d\r\n", msg.header.raw_length);
    printf("  port: %d\r\n", msg.header.port);
    printf("  payload type: %d\r\n", msg.header.payload_type);
    printf("  payload len: %d\r\n", msg.header.payload_length());
    uint8_t payload_len = msg.header.payload_length();
    if (payload_len > 0)
    {
        printf("  payload: ");
        for (auto i = 0; i < msg.payload_length(); ++i)
            printf("%d, ", ((uint8_t*)(msg.payload))[i]);
    }
    printf("\r\n\r\n");
#endif
    // Handle in-range register msgs and clear them. Ignore out-of-range msgs.
    handle_buffered_core_message(); // Handle msg. Clear it if handled.
    if (not new_msg_)
        return;
    handle_buffered_app_message(); // Handle msg. Clear it if handled.
    // Always clear any unhandled messages, so we don't lock up.
    if (new_msg_)
    {
#ifdef DEBUG_HARP_MSG_IN
    printf("Ignoring out-of-range msg!\r\n");
#endif
        clear_msg();
    }
}

void HarpCore::process_cdc_input()
{
    if (not tud_cdc_available())
        return;
    uint32_t max_bytes_to_read = sizeof(rx_buffer_) - rx_buffer_index_;
    if (total_bytes_read_ >= sizeof(msg_header_t))
    {
        msg_header_t& header = get_buffered_msg_header();
        max_bytes_to_read = header.msg_size() - total_bytes_read_;
    }
    uint32_t bytes_read = tud_cdc_read(&(rx_buffer_[rx_buffer_index_]),
                                       max_bytes_to_read);
    if (bytes_read > 0)
        ESP_LOGD(kHarpCoreLogTag, "RX bytes=%lu buffered=%u",
                 (unsigned long)bytes_read,
                 (unsigned)rx_buffer_index_ + (unsigned)bytes_read);
    rx_buffer_index_ += bytes_read;
    if (total_bytes_read_ < sizeof(msg_header_t))
        return;
    msg_header_t& header = get_buffered_msg_header();
    if (total_bytes_read_ < header.msg_size())
        return;
    ESP_LOGD(kHarpCoreLogTag,
             "RX msg type=%u addr=%u raw_len=%u payload_type=0x%02X",
             (unsigned)header.type,
             (unsigned)header.address,
             (unsigned)header.raw_length,
             (unsigned)header.payload_type);
    rx_buffer_index_ = 0;
    new_msg_ = true;
    return;
}

msg_t HarpCore::get_buffered_msg()
{
    msg_header_t& header = get_buffered_msg_header();
    void* payload = rx_buffer_ + header.payload_base_index_offset();
    uint8_t& checksum = *(rx_buffer_ + header.checksum_index_offset());
    return msg_t{header, payload, checksum};
}

void HarpCore::handle_buffered_core_message()
{
    msg_t msg = get_buffered_msg();
    ESP_LOGD(kHarpCoreLogTag, "Dispatch type=%u addr=%u",
             (unsigned)msg.header.type,
             (unsigned)msg.header.address);
    if (msg.header.address > CORE_REG_COUNT)
        return;
    switch (msg.header.type)
    {
        case READ:
            reg_func_table_[msg.header.address].read_fn_ptr(msg.header.address);
            break;
        case WRITE:
            reg_func_table_[msg.header.address].write_fn_ptr(msg);
            break;
        // These message types are generated by device-side responses and
        // are not valid inbound host commands for register handling.
        case EVENT:
        case READ_ERROR:
        case WRITE_ERROR:
        case ERROR_MASK:
            break;
    }
    clear_msg();
}

void HarpCore::update_state(bool force, op_mode_t forced_next_state)
{
    uint64_t harp_time_us = harp_time_us_64();
    uint32_t time_us = harp_to_system_us_32(harp_time_us);
    bool tud_cdc_is_connected = tud_cdc_connected();
    bool is_synced = self->is_synced();
    self->sync_handled_ = is_synced?
                              self->sync_handled_: false;
    self->disconnect_handled_ = tud_cdc_is_connected?
                                    false: self->disconnect_handled_;
    self->connect_handled_ = tud_cdc_is_connected? self->connect_handled_: false;
    if (!tud_cdc_is_connected && !self->disconnect_handled_)
    {
        self->disconnect_handled_ = true;
        self->disconnect_start_time_us_ = time_us;
    }
    if (is_synced && !self->sync_handled_)
    {
        update_next_heartbeat_from_curr_harp_time_us(harp_time_us);
        self->sync_handled_ = true;
    }
    op_mode_t state = op_mode_t(self->regs_.r_operation_ctrl_bits.OP_MODE);
    op_mode_t next_state = force ? forced_next_state : state;
    if (!force)
    {
        switch (state)
        {
            case STANDBY:
                if (tud_cdc_is_connected && !self->connect_handled_)
                    next_state = ACTIVE;
                [[fallthrough]];
            case ACTIVE:
                if (!tud_cdc_is_connected && self->disconnect_handled_
                    && (time_us - self->disconnect_start_time_us_)
                       >= NO_PC_INTERVAL_US)
                    next_state = STANDBY;
                break;
            case RESERVED:
                break;
            case SPEED:
                break;
            default:
                break;
        }
    }
    if ((state != ACTIVE) && (next_state == ACTIVE))
    {
        self->connect_handled_ = true;
        self->heartbeat_interval_us_ = HEARTBEAT_ACTIVE_INTERVAL_US;
    }
    if (state == ACTIVE && next_state == STANDBY)
    {
        self->heartbeat_interval_us_ = HEARTBEAT_STANDBY_INTERVAL_US;
    }
    if (int32_t(time_us - self->next_heartbeat_time_us_) >= 0)
    {
        self->next_heartbeat_time_us_ += self->heartbeat_interval_us_;
        if (self->regs_.r_operation_ctrl_bits.ALIVE_EN)
        {
            if ((state == ACTIVE) & !is_muted())
                send_harp_reply(EVENT, TIMESTAMP_SECOND);
        }
    }
    self->regs_.r_operation_ctrl_bits.OP_MODE = static_cast<uint8_t>(next_state);
}

const RegSpecs& HarpCore::reg_address_to_specs(uint8_t address)
{
    if (address < CORE_REG_COUNT)
        return self->regs_.address_to_specs[address];
    return self->address_to_app_reg_specs(address);
}

void HarpCore::send_harp_reply(msg_type_t reply_type, uint8_t reg_name,
                               const volatile uint8_t* data, uint8_t num_bytes,
                               reg_type_t payload_type, uint64_t harp_time_us)
{
    ESP_LOGD(kHarpCoreLogTag,
             "TX reply type=%u addr=%u bytes=%u cdc_connected=%u",
             (unsigned)reply_type,
             (unsigned)reg_name,
             (unsigned)num_bytes,
             (unsigned)tud_cdc_connected());
    uint8_t raw_length = num_bytes + 10;
    uint8_t checksum = 0;
    msg_header_t header{reply_type, raw_length, reg_name, 255,
                        reg_type_t(std::to_underlying(HAS_TIMESTAMP) |
                                   std::to_underlying(payload_type))};
#ifdef DEBUG_HARP_MSG_OUT
    printf("Sending msg: \r\n");
    printf("  type: %d\r\n", header.type);
    printf("  addr: %d\r\n", header.address);
    printf("  raw len: %d\r\n", header.raw_length);
    printf("  port: %d\r\n", header.port);
    printf("  payload type: %d\r\n", header.payload_type);
    printf("  payload len: %d\r\n", header.payload_length());
    uint8_t payload_len = header.payload_length();
    if (payload_len > 0)
    {
        printf("  payload: ");
        for (auto i = 0; i < header.payload_length(); ++i)
            printf("%d, ", data[i]);
    }
    printf("\r\n\r\n");
#endif
    for (uint8_t i = 0; i < sizeof(header); ++i)
    {
        uint8_t& byte = *(((uint8_t*)(&header))+i);
        checksum += byte;
        tud_cdc_write_char(byte);
    }
    self->set_timestamp_regs(harp_time_us);
    for (uint8_t i = 0; i < sizeof(self->regs.R_TIMESTAMP_SECOND); ++i)
    {
        uint8_t& byte = *(((uint8_t*)(&self->regs.R_TIMESTAMP_SECOND)) + i);
        checksum += byte;
        tud_cdc_write_char(byte);
    }
    for (uint8_t i = 0; i < sizeof(self->regs.R_TIMESTAMP_MICRO); ++i)
    {
        uint8_t& byte = *(((uint8_t*)(&self->regs.R_TIMESTAMP_MICRO)) + i);
        checksum += byte;
        tud_cdc_write_char(byte);
    }
    for (uint8_t i = 0; i < num_bytes; ++i)
    {
        const volatile uint8_t& byte = *(data + i);
        checksum += byte;
        tud_cdc_write_char(byte);
    }
    tud_cdc_write_char(checksum);
    tud_cdc_write_flush();
    // On ESP32-S3 TinyUSB runs in its own background task; tud_task() must
    // NOT be called here (would assert / deadlock inside the FreeRTOS task).
}

void HarpCore::read_reg_generic(uint8_t reg_name)
{
    send_harp_reply(READ, reg_name);
}


void HarpCore::write_reg_generic(msg_t& msg)
{
    copy_msg_payload_to_register(msg);
    if (self->is_muted())
        return;
    const uint8_t& reg_name = msg.header.address;
    const RegSpecs& specs = self->reg_address_to_specs(msg.header.address);
    send_harp_reply(WRITE, reg_name, specs.base_ptr, specs.num_bytes,
                    specs.payload_type);
}

void HarpCore::write_to_read_only_reg_error(msg_t& msg)
{
#ifdef DEBUG_HARP_MSG_IN
    printf("Error: Reg address %d is read-only.\r\n", msg.header.address);
#endif
    send_harp_reply(WRITE_ERROR, msg.header.address);
}

void HarpCore::read_from_write_only_reg_error(uint8_t address)
{
#ifdef DEBUG_HARP_MSG_IN
    printf("Error: Reg address %d is write-only.\r\n", address);
#endif
    send_harp_reply(READ_ERROR, address, nullptr, 0, U8);
}

void HarpCore::set_timestamp_regs(uint64_t harp_time_us)
{
    // Harp Time is computed as an offset relative to the platform's
    // microsecond timer (esp_timer_get_time()).
    // Note: R_TIMESTAMP_MICRO can only represent values up to 31249.
    // Note: Update microseconds first.
    uint64_t& curr_microseconds = harp_time_us;
    self->regs.R_TIMESTAMP_SECOND = curr_microseconds / 1'000'000ULL;
    self->regs.R_TIMESTAMP_MICRO = uint16_t((curr_microseconds % 1'000'000UL) >> 5);
}

void HarpCore::read_timestamp_second(uint8_t reg_name)
{
    self->update_timestamp_regs();
    read_reg_generic(reg_name);
}

void HarpCore::write_timestamp_second(msg_t& msg)
{
    uint32_t seconds;
    memcpy((void*)(&seconds), msg.payload, sizeof(seconds));
    uint64_t set_time_microseconds = uint64_t(seconds) * 1'000'000ULL;
    // Preserve the sub-second portion of the current Harp time.
    uint64_t curr_microseconds = harp_time_us_64() % 1'000'000ULL;
    uint64_t new_harp_time_us = set_time_microseconds + curr_microseconds;
    set_harp_time_us_64(new_harp_time_us);
    update_next_heartbeat_from_curr_harp_time_us(harp_time_us_64());
    send_harp_reply(WRITE, msg.header.address);
}

void HarpCore::read_timestamp_microsecond(uint8_t reg_name)
{
    self->update_timestamp_regs();
    read_reg_generic(reg_name);
}

void HarpCore::write_timestamp_microsecond(msg_t& msg)
{
    const uint32_t msg_us = ((uint32_t)(*((uint16_t*)msg.payload))) << 5;
    // Keep the whole-second part and replace only the sub-second part.
    uint64_t curr_total_s = harp_time_us_64() / 1'000'000ULL;
    uint64_t new_harp_time_us = curr_total_s + msg_us;
    set_harp_time_us_64(new_harp_time_us);
    update_next_heartbeat_from_curr_harp_time_us(harp_time_us_64());
    send_harp_reply(WRITE, msg.header.address);
}

void HarpCore::write_operation_ctrl(msg_t& msg)
{
    uint8_t& write_byte = *((uint8_t*)msg.payload);
    const uint8_t& state = self->regs_.r_operation_ctrl_bits.OP_MODE;
    const uint8_t& next_state = (*((OperationCtrlBits*)(&write_byte))).OP_MODE;
    if (state != next_state)
        self->force_state((op_mode_t)next_state);
    self->regs.R_OPERATION_CTRL = write_byte & ~(0x01 << DUMP_OFFSET);
    self->set_visual_indicators(bool((write_byte >> VISUAL_EN_OFFSET) & 0x01));
    if (self->is_muted())
        return;
    bool DUMP = bool((write_byte >> DUMP_OFFSET) & 0x01);
    send_harp_reply(WRITE, msg.header.address);
    if (DUMP)
    {
        for (uint8_t address = 0; address < CORE_REG_COUNT; ++address)
        {
            send_harp_reply(READ, address);
        }
        self->dump_app_registers();
    }
}

void HarpCore::write_reset_dev(msg_t& msg)
{
    uint8_t& write_byte = *((uint8_t*)msg.payload);
    const bool& rst_dev_bit = bool((write_byte >> RST_DEV_OFFSET) & 1u);
    const bool& reset_dfu_bit = bool((write_byte >> RST_DFU_OFFSET) & 1u);
    // ACK first so the host can confirm command acceptance before reset paths.
    send_harp_reply(WRITE, msg.header.address);

    // On ESP32-S3 there is no USB bootloader (DFU) mode equivalent to
    // reset_usb_boot()-style flows on some MCUs. RST_DFU is mapped to
    // esp_restart() (full chip reset). RST_DEV keeps the app-level reset
    // behavior through reset_app().
    if (reset_dfu_bit)
        esp_restart();
    if (rst_dev_bit)
    {
        self->regs_.r_operation_ctrl_bits.OP_MODE = STANDBY;
        self->reset_app();
    }
}

void HarpCore::write_device_name(msg_t& msg)
{
    // TODO: persist to NVS.
    write_reg_generic(msg);
}

void HarpCore::write_serial_number(msg_t& msg)
{
    // TODO.
    write_reg_generic(msg);
}

void HarpCore::write_clock_config(msg_t& msg)
{
    // TODO.
    write_reg_generic(msg);
}

void HarpCore::write_timestamp_offset(msg_t& msg)
{
    // TODO.
    write_reg_generic(msg);
}
