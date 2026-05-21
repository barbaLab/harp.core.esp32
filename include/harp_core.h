#ifndef HARP_CORE_H
#define HARP_CORE_H
#include <stdint.h>
#include <harp_message.h>
#include <core_registers.h>
#include <harp_synchronizer.h>
#include <cstring> // for memcpy
#include <tusb.h>
#include <utility> // for std::to_underlying

// ESP32-S3 includes.
// esp_timer.h provides esp_timer_get_time(), the 64-bit microsecond timebase.
// esp_mac.h / esp_efuse.h are used in harp_core.cpp to populate R_UUID.
#include <esp_timer.h>
#include <esp_mac.h>
#include <esp_efuse.h>

// Project version.
inline constexpr size_t ESP32_CORE_VERSION_MAJOR = 0;
inline constexpr size_t ESP32_CORE_VERSION_MINOR = 3;
inline constexpr size_t ESP32_CORE_VERSION_PATCH = 0;

// Backward-compatible aliases for downstream projects that still reference the
// old symbol names.
inline constexpr size_t PICO_CORE_VERSION_MAJOR = ESP32_CORE_VERSION_MAJOR;
inline constexpr size_t PICO_CORE_VERSION_MINOR = ESP32_CORE_VERSION_MINOR;
inline constexpr size_t PICO_CORE_VERSION_PATCH = ESP32_CORE_VERSION_PATCH;

// Version of the Harp Protocol that this library most closely implements.
inline constexpr size_t HARP_VERSION_MAJOR = 0;
inline constexpr size_t HARP_VERSION_MINOR = 0;
inline constexpr size_t HARP_VERSION_PATCH = 0;



#define NO_PC_INTERVAL_US (3'000'000UL)
#define HEARTBEAT_ACTIVE_INTERVAL_US (1'000'000UL)
#define HEARTBEAT_STANDBY_INTERVAL_US (3'000'000UL)

typedef void (*read_reg_fn)(uint8_t reg);
typedef void (*write_reg_fn)(msg_t& msg);

struct RegFnPair
{
    read_reg_fn read_fn_ptr;
    write_reg_fn  write_fn_ptr;
};

class HarpCore
{
using enum reg_type_t;
protected:
    HarpCore(uint16_t who_am_i,
             uint8_t hw_version_major, uint8_t hw_version_minor,
             uint8_t assembly_version,
             uint8_t harp_version_major, uint8_t harp_version_minor,
             uint8_t fw_version_major, uint8_t fw_version_minor,
             uint16_t serial_number, const char name[],
             const uint8_t tag[]);

    ~HarpCore();

public:
    HarpCore() = delete;
    HarpCore(HarpCore& other) = delete;
    void operator=(const HarpCore& other) = delete;

    static HarpCore& init(uint16_t who_am_i,
                          uint8_t hw_version_major, uint8_t hw_version_minor,
                          uint8_t assembly_version,
                          uint8_t harp_version_major, uint8_t harp_version_minor,
                          uint8_t fw_version_major, uint8_t fw_version_minor,
                          uint16_t serial_number, const char name[],
                          const uint8_t tag[]);

    static inline HarpCore* self = nullptr;
    static HarpCore& instance() {return *self;}

    void run();

    msg_header_t& get_buffered_msg_header()
    {return *((msg_header_t*)(&rx_buffer_));}

    msg_t get_buffered_msg();

    RegValues& regs = regs_.regs_;

    bool new_msg()
    {return new_msg_;}

    void clear_msg()
    {new_msg_ = false;}

    static void write_reg_generic(msg_t& msg);
    static void read_reg_generic(uint8_t reg_name);
    static void read_from_write_only_reg_error(uint8_t reg_name);
    static void write_to_read_only_reg_error(msg_t& msg);

    static inline void copy_msg_payload_to_register(msg_t& msg)
    {
        const RegSpecs& specs = reg_address_to_specs(msg.header.address);
        memcpy((void*)specs.base_ptr, msg.payload, specs.num_bytes);
    }

    static void send_harp_reply(msg_type_t reply_type, uint8_t reg_name,
                                const volatile uint8_t* data, uint8_t num_bytes,
                                reg_type_t payload_type, uint64_t harp_time_us);

    static inline void send_harp_reply(msg_type_t reply_type, uint8_t reg_name,
                                       const volatile uint8_t* data,
                                       uint8_t num_bytes,
                                       reg_type_t payload_type)
    {return send_harp_reply(reply_type, reg_name, data, num_bytes, payload_type,
                            harp_time_us_64());}

    static inline void send_harp_reply(msg_type_t reply_type, uint8_t reg_name)
    {
        const RegSpecs& specs = reg_address_to_specs(reg_name);
        send_harp_reply(reply_type, reg_name, specs.base_ptr, specs.num_bytes,
                        specs.payload_type);
    }

    static inline void send_harp_reply(msg_type_t reply_type, uint8_t reg_name,
                                       uint64_t harp_time_us)
    {
        const RegSpecs& specs = reg_address_to_specs(reg_name);
        send_harp_reply(reply_type, reg_name, specs.base_ptr, specs.num_bytes,
                        specs.payload_type, harp_time_us);
    }

    static inline bool is_muted()
    {return bool((self->regs.R_OPERATION_CTRL >> MUTE_RPL_OFFSET) & 0x01);}

    static inline bool is_synced()
    {
        return (self->sync_ == nullptr)?
            false:
            self->sync_->is_synced();
    }

    static inline bool events_enabled()
    {return self->get_op_mode() == ACTIVE;}

    /**
     * \brief Return the current Harp time in microseconds.
     * Uses esp_timer_get_time() as the underlying 64-bit monotonic clock,
     * then subtracts the synchronizer offset.
     */
    static inline uint64_t harp_time_us_64()
    {return system_to_harp_us_64((uint64_t)esp_timer_get_time());}

    static inline uint32_t harp_time_s()
    {
        self->update_timestamp_regs();
        return self->regs.R_TIMESTAMP_SECOND;
    }

    static inline uint64_t harp_to_system_us_64(uint64_t harp_time_us)
    {return (self->sync_ == nullptr)?
                harp_time_us + self->offset_us_64_:
                self->sync_->harp_to_system_us_64(harp_time_us);}

    static inline uint32_t harp_to_system_us_32(uint64_t harp_time_us)
    {return uint32_t(harp_to_system_us_64(harp_time_us));}

    static inline uint64_t system_to_harp_us_64(uint64_t system_time_us)
    {return (self->sync_ == nullptr)?
                system_time_us - self->offset_us_64_:
                self->sync_->system_to_harp_us_64(system_time_us);}

    /**
     * \brief Set the Harp time, updating the offset so that
     *        harp_time = esp_timer_get_time() - offset.
     */
    static inline void set_harp_time_us_64(uint64_t harp_time_us)
    {if (self->sync_ != nullptr)
        self->sync_->set_harp_time_us_64(harp_time_us);
     self->offset_us_64_ = (uint64_t)esp_timer_get_time() - harp_time_us;}

    static void set_synchronizer(HarpSynchronizer* sync)
    {self->sync_ = sync;}

    static void set_visual_indicators_fn(void (*func)(bool))
    {self->set_visual_indicators_fn_ = func;}

    static void force_state(op_mode_t next_state)
    {self->update_state(true, next_state);}

    static inline op_mode_t get_op_mode()
    {return op_mode_t(self->regs_.r_operation_ctrl_bits.OP_MODE);}

    static void set_uuid(uint8_t* uuid, size_t num_bytes, size_t offset = 0)
    {
        memset(self->regs.R_UUID, 0, sizeof(self->regs.R_UUID));
        memcpy((void*)(&self->regs.R_UUID[offset]), (void*)uuid, num_bytes);
    }

    static const RegSpecs& reg_address_to_specs(uint8_t address);

protected:
    void handle_buffered_core_message();
    virtual void handle_buffered_app_message(){};
    virtual void update_app_state(){};
    virtual void reset_app(){};

    void set_visual_indicators(bool enabled)
    {if (set_visual_indicators_fn_ != nullptr)
        set_visual_indicators_fn_(enabled);}

    virtual void dump_app_registers(){};

    virtual const RegSpecs& address_to_app_reg_specs(uint8_t address)
    {return regs_.address_to_specs[0];}

    bool new_msg_;
    void (* set_visual_indicators_fn_)(bool);
    HarpSynchronizer* sync_;

private:
    /**
     * \brief Align the next heartbeat to the current whole-second boundary.
     * Uses the C % operator (sufficient on ESP32-S3 with hardware divider).
     */
    static inline void update_next_heartbeat_from_curr_harp_time_us(
        uint64_t curr_harp_time_us)
    {
        uint64_t remainder = curr_harp_time_us % 1'000'000ULL;
        self->next_heartbeat_time_us_ =
            harp_to_system_us_32(curr_harp_time_us - remainder)
            + self->heartbeat_interval_us_;
    }

    Registers regs_;
    uint8_t rx_buffer_[MAX_PACKET_SIZE];
    uint8_t rx_buffer_index_;
    const uint8_t& total_bytes_read_;
    uint64_t offset_us_64_;
    bool disconnect_handled_;
    bool connect_handled_;
    bool sync_handled_;
    uint32_t heartbeat_interval_us_;
    uint32_t next_heartbeat_time_us_;
    uint32_t disconnect_start_time_us_;

    void process_cdc_input();
    static void update_state(bool force = false,
                             op_mode_t forced_next_state = STANDBY);
    static inline void update_timestamp_regs()
    {return set_timestamp_regs(harp_time_us_64());}
    static void set_timestamp_regs(uint64_t harp_time_us);

    static void read_timestamp_second(uint8_t reg_name);
    static void read_timestamp_microsecond(uint8_t reg_name);
    static void write_timestamp_second(msg_t& msg);
    static void write_timestamp_microsecond(msg_t& msg);
    static void write_operation_ctrl(msg_t& msg);
    static void write_reset_dev(msg_t& msg);
    static void write_device_name(msg_t& msg);
    static void write_serial_number(msg_t& msg);
    static void write_clock_config(msg_t& msg);
    static void write_timestamp_offset(msg_t& msg);

    RegFnPair reg_func_table_[CORE_REG_COUNT] =
    {
        {&HarpCore::read_reg_generic, &HarpCore::write_to_read_only_reg_error},
        {&HarpCore::read_reg_generic, &HarpCore::write_to_read_only_reg_error},
        {&HarpCore::read_reg_generic, &HarpCore::write_to_read_only_reg_error},
        {&HarpCore::read_reg_generic, &HarpCore::write_to_read_only_reg_error},
        {&HarpCore::read_reg_generic, &HarpCore::write_to_read_only_reg_error},
        {&HarpCore::read_reg_generic, &HarpCore::write_to_read_only_reg_error},
        {&HarpCore::read_reg_generic, &HarpCore::write_to_read_only_reg_error},
        {&HarpCore::read_reg_generic, &HarpCore::write_to_read_only_reg_error},
        {&HarpCore::read_timestamp_second, &HarpCore::write_timestamp_second},
        {&HarpCore::read_timestamp_microsecond, &HarpCore::write_timestamp_microsecond},
        {&HarpCore::read_reg_generic, &HarpCore::write_operation_ctrl},
        {&HarpCore::read_reg_generic, &HarpCore::write_reset_dev},
        {&HarpCore::read_reg_generic, &HarpCore::write_device_name},
        {&HarpCore::read_reg_generic, &HarpCore::write_serial_number},
        {&HarpCore::read_reg_generic, &HarpCore::write_clock_config},
        {&HarpCore::read_reg_generic, &HarpCore::write_timestamp_offset},
        {&HarpCore::read_reg_generic, &HarpCore::write_to_read_only_reg_error},
        {&HarpCore::read_reg_generic, &HarpCore::write_to_read_only_reg_error},
    };
};

#endif //HARP_CORE_H
