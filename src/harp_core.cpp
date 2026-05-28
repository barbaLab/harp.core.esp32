#include <harp_core.h>
#include <network_manager.h>
#include <tinyusb.h>
#include <tinyusb_default_config.h>

#include <algorithm>
#include <esp_log.h>

static const char* kHarpCoreLogTag = "HarpCore";

namespace {

void net_apply_from_core()
{
    NetworkManager::apply();
}

void net_disconnect_from_core()
{
    NetworkManager::disconnect();
}

} // namespace

HarpCore& HarpCore::init(uint16_t who_am_i,
                         uint8_t hw_version_major, uint8_t hw_version_minor,
                         uint8_t assembly_version,
                         uint8_t fw_version_major, uint8_t fw_version_minor,
                         uint16_t serial_number, const char name[],
                         const uint8_t tag[])
{
    // Create the singleton instance using the private constructor.
    static HarpCore core(who_am_i, hw_version_major, hw_version_minor,
                         assembly_version,
                         fw_version_major, fw_version_minor, serial_number,
                         name, tag);
    return core;
}

HarpCore& HarpCore::init(uint16_t who_am_i,
                         uint8_t hw_version_major, uint8_t hw_version_minor,
                         uint8_t assembly_version,
                         uint8_t harp_version_major, uint8_t harp_version_minor,
                         uint8_t fw_version_major, uint8_t fw_version_minor,
                         uint16_t serial_number, const char name[],
                         const uint8_t tag[])
{
    (void)harp_version_major;
    (void)harp_version_minor;
    return init(who_am_i,
                hw_version_major, hw_version_minor,
                assembly_version,
                fw_version_major, fw_version_minor,
                serial_number, name, tag);
}

HarpCore::HarpCore(uint16_t who_am_i,
                   uint8_t hw_version_major, uint8_t hw_version_minor,
                   uint8_t assembly_version,
                   uint8_t fw_version_major, uint8_t fw_version_minor,
                   uint16_t serial_number, const char name[],
                   const uint8_t tag[])
:new_msg_{false},
 set_visual_indicators_fn_{nullptr}, sync_{nullptr},
 regs_{who_am_i, hw_version_major, hw_version_minor, assembly_version,
       fw_version_major, fw_version_minor, serial_number, name, tag},
 tcp_rx_index_{0},
 cdc_rx_index_{0},
 active_rx_buffer_{nullptr},
 buffered_msg_source_{TransportSource::None},
 offset_us_64_{0},
 disconnect_handled_{false}, connect_handled_{false}, sync_handled_{false},
 heartbeat_interval_us_{HEARTBEAT_STANDBY_INTERVAL_US}
{
    // Create a pointer to the first (and one-and-only) instance created.
    if (self == nullptr)
        self = this;

    // Core-native network transport wiring.
    NetworkManager::init(&regs);
    set_net_save_and_connect_fn(&net_apply_from_core);
    set_net_clear_fn(&net_disconnect_from_core);
    set_tcp_write_fn(&NetworkManager::tcp_write);

    // Install TinyUSB via esp_tinyusb wrapper so RHPORT/device config is set
    // by ESP-IDF before the TinyUSB task starts.
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGD(kHarpCoreLogTag, "tinyusb_driver_install done");

    // Populate Harp Core R_UID with serial low bytes and ESP32 base MAC.
    regs.R_UID[0] = static_cast<uint8_t>(regs.R_SERIAL_NUMBER & 0xFFu);
    regs.R_UID[1] = static_cast<uint8_t>((regs.R_SERIAL_NUMBER >> 8) & 0xFFu);
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    memcpy((void*)(&regs.R_UID[8]), mac, sizeof(mac));

    // Populate R_VERSION (protocol, firmware, hardware, core ID, hash=0).
    // Keep this independent from deprecated legacy version registers so they
    // can be removed without changing R_VERSION population logic.
    regs.R_VERSION[0] = HARP_PROTOCOL_VERSION_MAJOR;
    regs.R_VERSION[1] = HARP_PROTOCOL_VERSION_MINOR;
    regs.R_VERSION[2] = HARP_PROTOCOL_VERSION_PATCH;
    regs.R_VERSION[3] = fw_version_major;
    regs.R_VERSION[4] = fw_version_minor;
    regs.R_VERSION[5] = 0;
    regs.R_VERSION[6] = hw_version_major;
    regs.R_VERSION[7] = hw_version_minor;
    regs.R_VERSION[8] = 0;
    regs.R_VERSION[9] = 'E';
    regs.R_VERSION[10] = '3';
    regs.R_VERSION[11] = '2';

    // Initialize next heartbeat.
    update_next_heartbeat_from_curr_harp_time_us(harp_time_us_64());
}

HarpCore::HarpCore(uint16_t who_am_i,
                   uint8_t hw_version_major, uint8_t hw_version_minor,
                   uint8_t assembly_version,
                   uint8_t harp_version_major, uint8_t harp_version_minor,
                   uint8_t fw_version_major, uint8_t fw_version_minor,
                   uint16_t serial_number, const char name[],
                   const uint8_t tag[])
    : HarpCore(who_am_i,
               hw_version_major, hw_version_minor,
               assembly_version,
               fw_version_major, fw_version_minor,
               serial_number, name, tag)
{
    (void)harp_version_major;
    (void)harp_version_minor;
}

HarpCore::~HarpCore(){self = nullptr;}

void HarpCore::run()
{
    // On ESP32-S3 TinyUSB runs in its own background task (started by
    // tusb_init()); tud_task() must NOT be called from application code.
    update_state();
    refresh_heartbeat_register();
    update_app_state(); // Does nothing unless a derived class implements it.
    refresh_net_config_status_bits();
    // Accept incoming Harp commands over both TCP and USB CDC.
    process_tcp_input();
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
        msg_t msg = get_buffered_msg();
        if (msg.header.type == READ)
            send_harp_reply(READ_ERROR, msg.header.address);
        else if (msg.header.type == WRITE)
            send_harp_reply(WRITE_ERROR, msg.header.address);
#ifdef DEBUG_HARP_MSG_IN
    printf("Ignoring out-of-range msg!\r\n");
#endif
        clear_msg();
    }
}

void HarpCore::process_tcp_input()
{
    process_transport_input(
        tcp_rx_buffer_,
        &tcp_rx_index_,
        TransportSource::Tcp,
        [](uint8_t* data, size_t len) -> int { return NetworkManager::tcp_read(data, len); },
        "TCP");
}

void HarpCore::process_cdc_input()
{
    process_transport_input(
        cdc_rx_buffer_,
        &cdc_rx_index_,
        TransportSource::Cdc,
        [](uint8_t* data, size_t len) -> int {
            if (!tud_cdc_available())
                return 0;
            return static_cast<int>(tud_cdc_read(data, len));
        },
        "CDC");
}

void HarpCore::process_transport_input(uint8_t* buffer, size_t* buffer_index,
                                       TransportSource source,
                                       int (*read_fn)(uint8_t*, size_t),
                                       const char* transport_name)
{
    if (new_msg_)
        return;

    if (*buffer_index >= MAX_PACKET_SIZE)
        *buffer_index = 0;

    size_t max_bytes_to_read = MAX_PACKET_SIZE - *buffer_index;
    if (*buffer_index >= sizeof(msg_header_t))
    {
        msg_header_t& header = *((msg_header_t*)(buffer));
        size_t msg_size = std::min(static_cast<size_t>(header.msg_size()),
                                   static_cast<size_t>(MAX_PACKET_SIZE));
        if (msg_size > *buffer_index)
            max_bytes_to_read = std::min(max_bytes_to_read, msg_size - *buffer_index);
        else
            max_bytes_to_read = 0;
    }
    if (max_bytes_to_read == 0)
        return;

    int bytes_read = read_fn(&(buffer[*buffer_index]), max_bytes_to_read);
    if (bytes_read <= 0)
        return;

    const size_t safe_read = std::min(static_cast<size_t>(bytes_read), max_bytes_to_read);
    ESP_LOGD(kHarpCoreLogTag, "RX(%s) bytes=%u buffered=%u",
             transport_name,
             static_cast<unsigned>(safe_read),
             static_cast<unsigned>(*buffer_index + safe_read));
    *buffer_index += safe_read;

    if (*buffer_index < sizeof(msg_header_t))
        return;

    msg_header_t& header = *((msg_header_t*)(buffer));
    const size_t msg_size = std::min(static_cast<size_t>(header.msg_size()),
                                     static_cast<size_t>(MAX_PACKET_SIZE));
    if (*buffer_index < msg_size)
        return;

    ESP_LOGD(kHarpCoreLogTag,
             "RX(%s) msg type=%u addr=%u raw_len=%u payload_type=0x%02X",
             transport_name,
             (unsigned)header.type,
             (unsigned)header.address,
             (unsigned)header.raw_length,
             (unsigned)header.payload_type);
    active_rx_buffer_ = buffer;
    buffered_msg_source_ = source;
    *buffer_index = 0;
    new_msg_ = true;
}

msg_t HarpCore::get_buffered_msg()
{
    msg_header_t& header = get_buffered_msg_header();
    void* payload = active_rx_buffer_ + header.payload_base_index_offset();
    uint8_t& checksum = *(active_rx_buffer_ + header.checksum_index_offset());
    return msg_t{header, payload, checksum};
}

void HarpCore::refresh_net_config_status_bits()
{
    uint8_t enable_bits = self->regs.R_NET_CONFIG & NET_CFG_ENABLE_MASK;
    uint8_t status_bits = 0;

    if (self->regs.R_NET_SSID[0] != '\0' && self->regs.R_NET_SERVER_IP[0] != '\0')
        status_bits |= NET_CFG_STATUS_CFG_VALID_MASK;
    if (NetworkManager::is_wifi_connected())
        status_bits |= NET_CFG_STATUS_WIFI_UP_MASK | NET_CFG_STATUS_IP_OK_MASK;
    if (NetworkManager::is_tcp_connected())
        status_bits |= NET_CFG_STATUS_TCP_CONN_MASK;

    self->regs.R_NET_CONFIG = static_cast<uint8_t>(enable_bits | (status_bits & NET_CFG_STATUS_MASK));
}

void HarpCore::refresh_heartbeat_register()
{
    uint16_t heartbeat = 0;
    if (get_op_mode() == ACTIVE)
        heartbeat |= 0x01u;
    if (is_synced())
        heartbeat |= 0x02u;
    self->regs.R_HEARTBEAT = heartbeat;
}

void HarpCore::handle_buffered_core_message()
{
    msg_t msg = get_buffered_msg();
    ESP_LOGD(kHarpCoreLogTag, "Dispatch type=%u addr=%u",
             (unsigned)msg.header.type,
             (unsigned)msg.header.address);
    RegFnPair* fn_pair = nullptr;
    if (msg.header.address < CORE_REG_COUNT)
        fn_pair = &self->reg_func_table_[msg.header.address];
    else if (is_core_extension_address(msg.header.address))
        fn_pair = &self->core_extension_reg_func_table_[core_extension_address_to_index(msg.header.address)];
    else
        return;

    const RegSpecs& specs = reg_address_to_specs(msg.header.address);
    const uint8_t request_payload_type = static_cast<uint8_t>(msg.header.payload_type);
    const uint8_t expected_payload_type = static_cast<uint8_t>(specs.payload_type);
    if (request_payload_type != expected_payload_type)
    {
        if (msg.header.type == READ)
            send_harp_reply(READ_ERROR, msg.header.address);
        else if (msg.header.type == WRITE)
            send_harp_reply(WRITE_ERROR, msg.header.address);
        clear_msg();
        return;
    }
    if ((msg.header.type == WRITE) && (msg.payload_length() != specs.num_bytes))
    {
        send_harp_reply(WRITE_ERROR, msg.header.address);
        clear_msg();
        return;
    }

    switch (msg.header.type)
    {
        case READ:
            if (fn_pair->read_fn_ptr != nullptr)
                fn_pair->read_fn_ptr(msg.header.address);
            else
                send_harp_reply(READ_ERROR, msg.header.address);
            break;
        case WRITE:
            if (fn_pair->write_fn_ptr != nullptr)
                fn_pair->write_fn_ptr(msg);
            else
                send_harp_reply(WRITE_ERROR, msg.header.address);
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
    const bool usb_connected = tud_cdc_connected();
    const bool tcp_connected = NetworkManager::is_tcp_connected();
    const bool host_connected = usb_connected || tcp_connected;
    bool is_synced = self->is_synced();
    self->sync_handled_ = is_synced?
                              self->sync_handled_: false;
    self->disconnect_handled_ = host_connected?
                                    false: self->disconnect_handled_;
    self->connect_handled_ = host_connected? self->connect_handled_: false;
    if (!host_connected && !self->disconnect_handled_)
    {
        self->disconnect_handled_ = true;
        self->disconnect_start_time_us_ = time_us;
    }
    if (is_synced && !self->sync_handled_)
    {
        update_next_heartbeat_from_curr_harp_time_us(harp_time_us);
        self->sync_handled_ = true;
    }
    op_mode_t state = op_mode_t(self->regs.R_OPERATION_CTRL & OP_CTRL_OP_MODE_MASK);
    op_mode_t next_state = force ? forced_next_state : state;
    if (!force)
    {
        switch (state)
        {
            case STANDBY:
                if (host_connected && !self->connect_handled_)
                    next_state = ACTIVE;
                [[fallthrough]];
            case ACTIVE:
                if (!host_connected)
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
    self->regs.R_OPERATION_CTRL = static_cast<uint8_t>(
        (self->regs.R_OPERATION_CTRL & static_cast<uint8_t>(~OP_CTRL_OP_MODE_MASK)) |
        (static_cast<uint8_t>(next_state) & OP_CTRL_OP_MODE_MASK));
    if (int32_t(time_us - self->next_heartbeat_time_us_) >= 0)
    {
        self->next_heartbeat_time_us_ += self->heartbeat_interval_us_;
        if ((next_state == ACTIVE) && !is_muted())
        {
            self->refresh_heartbeat_register();
            if ((self->regs.R_OPERATION_CTRL & OP_CTRL_HEARTBEAT_EN_MASK) != 0)
                send_harp_reply(EVENT, HEARTBEAT);
            else if ((self->regs.R_OPERATION_CTRL & OP_CTRL_ALIVE_EN_MASK) != 0)
                send_harp_reply(EVENT, TIMESTAMP_SECOND);
        }
    }
}

const RegSpecs& HarpCore::reg_address_to_specs(uint8_t address)
{
    if (address < CORE_REG_COUNT)
        return self->regs_.address_to_specs[address];
    if (is_core_extension_address(address))
        return self->regs_.core_extension_address_to_specs[core_extension_address_to_index(address)];
    return self->address_to_app_reg_specs(address);
}

void HarpCore::send_harp_reply(msg_type_t reply_type, uint8_t reg_name,
                               const volatile uint8_t* data, uint8_t num_bytes,
                               reg_type_t payload_type, uint64_t harp_time_us)
{
    if ((reply_type != EVENT) && self->is_muted())
        return;

    ESP_LOGD(kHarpCoreLogTag,
             "TX reply type=%u addr=%u bytes=%u cdc_connected=%u",
             (unsigned)reply_type,
             (unsigned)reg_name,
             (unsigned)num_bytes,
             (unsigned)tud_cdc_connected());
    uint8_t raw_length = num_bytes + 10;
    uint8_t checksum = 0;
    uint8_t packet_buf[MAX_PACKET_SIZE] = {0};
    uint8_t packet_len = 0;
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
    const bool is_event = (reply_type == EVENT);
    const bool route_cdc =
        is_event || self->buffered_msg_source_ == TransportSource::None
        || self->buffered_msg_source_ == TransportSource::Cdc;
    const bool route_tcp =
        is_event || self->buffered_msg_source_ == TransportSource::None
        || self->buffered_msg_source_ == TransportSource::Tcp;

    for (uint8_t i = 0; i < sizeof(header); ++i)
    {
        uint8_t& byte = *(((uint8_t*)(&header))+i);
        checksum += byte;
        packet_buf[packet_len++] = byte;
        if (route_cdc)
            tud_cdc_write_char(byte);
    }
    self->set_timestamp_regs(harp_time_us);
    for (uint8_t i = 0; i < sizeof(self->regs.R_TIMESTAMP_SECOND); ++i)
    {
        uint8_t& byte = *(((uint8_t*)(&self->regs.R_TIMESTAMP_SECOND)) + i);
        checksum += byte;
        packet_buf[packet_len++] = byte;
        if (route_cdc)
            tud_cdc_write_char(byte);
    }
    for (uint8_t i = 0; i < sizeof(self->regs.R_TIMESTAMP_MICRO); ++i)
    {
        uint8_t& byte = *(((uint8_t*)(&self->regs.R_TIMESTAMP_MICRO)) + i);
        checksum += byte;
        packet_buf[packet_len++] = byte;
        if (route_cdc)
            tud_cdc_write_char(byte);
    }
    for (uint8_t i = 0; i < num_bytes; ++i)
    {
        const volatile uint8_t& byte = *(data + i);
        checksum += byte;
        packet_buf[packet_len++] = byte;
        if (route_cdc)
            tud_cdc_write_char(byte);
    }
    packet_buf[packet_len++] = checksum;
    if (route_cdc)
    {
        tud_cdc_write_char(checksum);
        tud_cdc_write_flush();
    }
    // On ESP32-S3 TinyUSB runs in its own background task; tud_task() must
    // NOT be called here (would assert / deadlock inside the FreeRTOS task).

    if (route_tcp && self->tcp_write_fn_ != nullptr)
        self->tcp_write_fn_(packet_buf, packet_len);
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
    const bool timestamp_unlocked =
        (self->regs.R_CLOCK_CONFIG & CLOCK_CFG_UNLOCK_MASK) != 0
        && (self->regs.R_CLOCK_CONFIG & CLOCK_CFG_LOCK_MASK) == 0;
    if (!timestamp_unlocked)
    {
        send_harp_reply(WRITE_ERROR, msg.header.address);
        return;
    }

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

void HarpCore::read_reset_dev(uint8_t reg_name)
{
    // Report boot state and clear transient command bits on read.
    uint8_t read_value = RESET_DEV_BOOT_DEF_MASK;
    send_harp_reply(READ, reg_name, &read_value, sizeof(read_value), U8);
}

void HarpCore::write_operation_ctrl(msg_t& msg)
{
    uint8_t& write_byte = *((uint8_t*)msg.payload);
    const uint8_t state = self->regs.R_OPERATION_CTRL & OP_CTRL_OP_MODE_MASK;
    const uint8_t next_state = write_byte & OP_CTRL_OP_MODE_MASK;
    if (next_state == SPEED)
    {
        send_harp_reply(WRITE_ERROR, msg.header.address);
        return;
    }
    if (state != next_state)
        self->force_state((op_mode_t)next_state);
    self->regs.R_OPERATION_CTRL = write_byte & static_cast<uint8_t>(~OP_CTRL_DUMP_MASK);
    self->set_visual_indicators((write_byte & OP_CTRL_VISUAL_EN_MASK) != 0);
    if (self->is_muted())
        return;
    bool DUMP = (write_byte & OP_CTRL_DUMP_MASK) != 0;
    send_harp_reply(WRITE, msg.header.address);
    if (DUMP)
    {
        for (uint8_t address = 0; address < CORE_REG_COUNT; ++address)
        {
            send_harp_reply(READ, address);
        }
        for (uint8_t address = CORE_EXTENSION_START_ADDRESS;
             address < CORE_EXTENSION_START_ADDRESS + CORE_EXTENSION_REG_COUNT;
             ++address)
        {
            send_harp_reply(READ, address);
        }
        self->dump_app_registers();
    }
}

void HarpCore::write_reset_dev(msg_t& msg)
{
    const uint8_t write_byte = *((uint8_t*)msg.payload);
    const bool rst_dev_bit = (write_byte & RESET_DEV_RST_DEF_MASK) != 0;
    const bool rst_ee_bit = (write_byte & RESET_DEV_RST_EE_MASK) != 0;
    const bool save_bit = (write_byte & RESET_DEV_SAVE_MASK) != 0;
    const bool name_to_default_bit = (write_byte & RESET_DEV_NAME_TO_DEFAULT_MASK) != 0;
    const bool reset_dfu_bit = (write_byte & RESET_DEV_RST_DFU_MASK) != 0;
    const bool boot_def_bit = (write_byte & RESET_DEV_BOOT_DEF_MASK) != 0;
    const bool boot_ee_bit = (write_byte & RESET_DEV_BOOT_EE_MASK) != 0;

    if (boot_def_bit || boot_ee_bit)
    {
        send_harp_reply(WRITE_ERROR, msg.header.address);
        return;
    }
    if (rst_ee_bit || save_bit)
    {
        send_harp_reply(WRITE_ERROR, msg.header.address);
        return;
    }

    // ACK first so the host can confirm command acceptance before reset paths.
    send_harp_reply(WRITE, msg.header.address);

    if (name_to_default_bit)
    {
        self->regs.R_DEVICE_NAME[0] = '\0';
    }

    // On ESP32-S3 there is no USB bootloader (DFU) mode equivalent to
    // reset_usb_boot()-style flows on some MCUs. RST_DFU is mapped to
    // esp_restart() (full chip reset). RST_DEV keeps the app-level reset
    // behavior through reset_app().
    if (reset_dfu_bit)
        esp_restart();
    if (rst_dev_bit)
    {
        self->regs.R_OPERATION_CTRL = static_cast<uint8_t>(
            self->regs.R_OPERATION_CTRL & static_cast<uint8_t>(~OP_CTRL_OP_MODE_MASK));
        self->reset_app();
    }
}

void HarpCore::write_device_name(msg_t& msg)
{
    // Optional register behavior when persistence is not implemented:
    // acknowledge with unchanged/default value and no side effects.
    send_harp_reply(WRITE, msg.header.address);
}

void HarpCore::write_serial_number(msg_t& msg)
{
    // Deprecated register write is unsupported in this implementation.
    send_harp_reply(WRITE, msg.header.address);
}

void HarpCore::write_clock_config(msg_t& msg)
{
    const uint8_t requested = *static_cast<uint8_t*>(msg.payload);
    const uint8_t current = self->regs.R_CLOCK_CONFIG;

    // Bit layout for R_CLOCK_CONFIG
    constexpr uint8_t MODE_MASK = CLOCK_CFG_CLK_REP_MASK | CLOCK_CFG_CLK_GEN_MASK;
    constexpr uint8_t CAP_MASK = CLOCK_CFG_REP_ABLE_MASK | CLOCK_CFG_GEN_ABLE_MASK;
    constexpr uint8_t UNLOCK_BIT = CLOCK_CFG_UNLOCK_MASK;
    constexpr uint8_t LOCK_BIT = CLOCK_CFG_LOCK_MASK;

    uint8_t next = current;

    // 1) Update writable mode bits from host request.
    next = static_cast<uint8_t>((next & ~MODE_MASK) | (requested & MODE_MASK));

    // 2) Apply lock/unlock command bits.
    // Preserve existing precedence: LOCK wins if both are set.
    const bool wants_lock = (requested & LOCK_BIT) != 0;
    const bool wants_unlock = (requested & UNLOCK_BIT) != 0;

    if (wants_lock)
    {
        next = static_cast<uint8_t>((next | LOCK_BIT) & ~UNLOCK_BIT);
    }
    else if (wants_unlock)
    {
        next = static_cast<uint8_t>((next | UNLOCK_BIT) & ~LOCK_BIT);
    }

    // 3) Force capability bits to stay read-only.
    next = static_cast<uint8_t>((next & ~CAP_MASK) | (current & CAP_MASK));

    self->regs.R_CLOCK_CONFIG = next;
    send_harp_reply(WRITE, msg.header.address);
}

void HarpCore::write_timestamp_offset(msg_t& msg)
{
    (void)msg;
    // Deprecated register write is unsupported in this implementation.
    self->regs.R_TIMESTAMP_OFFSET = 0;
    send_harp_reply(WRITE, msg.header.address);
}

// Masked read: send reply with zeroed payload instead of actual password
void HarpCore::read_net_password_masked(uint8_t reg_name) {
    static const uint8_t zeros[sizeof(self->regs.R_NET_PASSWORD)] = {};
    send_harp_reply(READ, reg_name, zeros, sizeof(self->regs.R_NET_PASSWORD),
                    reg_type_t::U8);
}

void HarpCore::write_net_ssid(msg_t& msg) {
    if (msg.header.payload_length() != sizeof(self->regs.R_NET_SSID)) {
        send_harp_reply(WRITE_ERROR, msg.header.address); return;
    }
    copy_msg_payload_to_register(msg);
    self->regs.R_NET_SSID[sizeof(self->regs.R_NET_SSID) - 1] = '\0';
    send_harp_reply(WRITE, msg.header.address);
}

void HarpCore::write_net_password(msg_t& msg) {
    if (msg.header.payload_length() != sizeof(self->regs.R_NET_PASSWORD)) {
        send_harp_reply(WRITE_ERROR, msg.header.address); return;
    }
    copy_msg_payload_to_register(msg);
    self->regs.R_NET_PASSWORD[sizeof(self->regs.R_NET_PASSWORD) - 1] = '\0';
    send_harp_reply(WRITE, msg.header.address);
}

void HarpCore::write_net_server_addr(msg_t& msg) {
    if (msg.header.payload_length() != sizeof(self->regs.R_NET_SERVER_IP)) {
        send_harp_reply(WRITE_ERROR, msg.header.address); return;
    }
    copy_msg_payload_to_register(msg);
    self->regs.R_NET_SERVER_IP[sizeof(self->regs.R_NET_SERVER_IP) - 1] = '\0';
    send_harp_reply(WRITE, msg.header.address);
}

void HarpCore::write_net_server_port(msg_t& msg) {
    if (msg.header.payload_length() != sizeof(self->regs.R_NET_SERVER_PORT)) {
        send_harp_reply(WRITE_ERROR, msg.header.address); return;
    }
    copy_msg_payload_to_register(msg);
    send_harp_reply(WRITE, msg.header.address);
}

// R_NET_CONFIG: combine enable, status, apply, clear into one byte
// Writing bit6=apply saves to NVS + connects; bit7=clear erases + disconnects
void HarpCore::write_net_config(msg_t& msg) {
    uint8_t val = *((uint8_t*)msg.payload);
    // Preserve status bits (read-only, refreshed from live network state)
    uint8_t status_bits = self->regs.R_NET_CONFIG & NET_CFG_STATUS_MASK; // bits [5:2]
    uint8_t enable_bits = val & NET_CFG_ENABLE_MASK;                      // bits [1:0]
    self->regs.R_NET_CONFIG = enable_bits | status_bits;

    if ((val & NET_CFG_APPLY_MASK) && (self->net_save_and_connect_fn_ != nullptr)) {
        self->net_save_and_connect_fn_();
    }
    if ((val & NET_CFG_CLEAR_MASK) && (self->net_clear_fn_ != nullptr)) {
        self->net_clear_fn_();
        self->regs.R_NET_CONFIG = 0;
    }
    send_harp_reply(WRITE, msg.header.address);
}