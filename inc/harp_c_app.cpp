#include <harp_c_app.h>
#include "status_led.h"

namespace StatusLedAppRegs
{
    using enum reg_type_t;

    namespace
    {
        constexpr uint8_t kDefaultLedMode = static_cast<uint8_t>(StatusLed::Mode::Off);
        constexpr uint8_t kDefaultRed = 0x11;
        constexpr uint8_t kDefaultGreen = 0x55;
        constexpr uint8_t kDefaultBlue = 0x77;

        bool is_valid_u8_write(msg_t& msg)
        {
            return msg.header.payload_type == reg_type_t::U8 && msg.header.payload_length() == 1;
        }

        void send_write_error(uint8_t address)
        {
            HarpCore::send_harp_reply(WRITE_ERROR, address);
        }

        void apply_rgb_from_registers()
        {
            StatusLed::set_rgb(values.R_LED_COLOR_RED,
                               values.R_LED_COLOR_GREEN,
                               values.R_LED_COLOR_BLUE);
        }
    }

    RegValues values{
        .R_LED_MODE = kDefaultLedMode,
        .R_LED_COLOR_RED = kDefaultRed,
        .R_LED_COLOR_GREEN = kDefaultGreen,
        .R_LED_COLOR_BLUE = kDefaultBlue,
    };

    RegSpecs specs[REGISTER_COUNT] =
    {
        {(uint8_t*)&values.R_LED_MODE, sizeof(values.R_LED_MODE), U8},
        {(uint8_t*)&values.R_LED_COLOR_RED, sizeof(values.R_LED_COLOR_RED), U8},
        {(uint8_t*)&values.R_LED_COLOR_GREEN, sizeof(values.R_LED_COLOR_GREEN), U8},
        {(uint8_t*)&values.R_LED_COLOR_BLUE, sizeof(values.R_LED_COLOR_BLUE), U8},
    };

    RegFnPair functions[REGISTER_COUNT] =
    {
        {&HarpCore::read_reg_generic, &write_led_mode},
        {&HarpCore::read_reg_generic, &write_led_color_component},
        {&HarpCore::read_reg_generic, &write_led_color_component},
        {&HarpCore::read_reg_generic, &write_led_color_component},
    };

    void reset()
    {
        values.R_LED_MODE = kDefaultLedMode;
        values.R_LED_COLOR_RED = kDefaultRed;
        values.R_LED_COLOR_GREEN = kDefaultGreen;
        values.R_LED_COLOR_BLUE = kDefaultBlue;

        StatusLed::set_rgb(values.R_LED_COLOR_RED,
                           values.R_LED_COLOR_GREEN,
                           values.R_LED_COLOR_BLUE);
        StatusLed::set_mode(StatusLed::Mode::Off);
    }

    void update()
    {
        StatusLed::update();
    }

    void write_led_mode(msg_t& msg)
    {
        if (!is_valid_u8_write(msg))
        {
            send_write_error(msg.header.address);
            return;
        }

        const auto requested_mode = *((uint8_t*)msg.payload);
        if (requested_mode > static_cast<uint8_t>(StatusLed::Mode::Fault))
        {
            send_write_error(msg.header.address);
            return;
        }

        HarpCore::copy_msg_payload_to_register(msg);
        StatusLed::set_mode(static_cast<StatusLed::Mode>(values.R_LED_MODE));
        HarpCore::send_harp_reply(WRITE, msg.header.address);
    }

    void write_led_color_component(msg_t& msg)
    {
        if (!is_valid_u8_write(msg))
        {
            send_write_error(msg.header.address);
            return;
        }

        HarpCore::copy_msg_payload_to_register(msg);
        apply_rgb_from_registers();
        HarpCore::send_harp_reply(WRITE, msg.header.address);
    }
}


HarpCApp& HarpCApp::init(uint16_t who_am_i,
                         uint8_t hw_version_major, uint8_t hw_version_minor,
                         uint8_t assembly_version,
                         uint8_t harp_version_major, uint8_t harp_version_minor,
                         uint8_t fw_version_major, uint8_t fw_version_minor,
                         uint16_t serial_number, const char name[],
                         const uint8_t tag[],
                         void* app_reg_values, RegSpecs* app_reg_specs,
                         RegFnPair* app_reg_fns, size_t app_reg_count,
                         void (* update_fn)(void), void (* reset_fn)(void))
{
    static HarpCApp app(who_am_i, hw_version_major, hw_version_minor,
                        assembly_version,
                        harp_version_major, harp_version_minor,
                        fw_version_major, fw_version_minor, serial_number,
                        name, tag, app_reg_values, app_reg_specs,
                        app_reg_fns, app_reg_count, update_fn, reset_fn);
    return app;
}

HarpCApp::HarpCApp(uint16_t who_am_i,
                   uint8_t hw_version_major, uint8_t hw_version_minor,
                   uint8_t assembly_version,
                   uint8_t harp_version_major, uint8_t harp_version_minor,
                   uint8_t fw_version_major, uint8_t fw_version_minor,
                   uint16_t serial_number, const char name[],
                   const uint8_t tag[],
                   void* app_reg_values, RegSpecs* app_reg_specs,
                   RegFnPair* app_reg_fns, size_t app_reg_count,
                   void (*update_fn)(void), void (* reset_fn)(void))
:reg_values_{app_reg_values},
 reg_specs_{app_reg_specs},
 reg_fns_{app_reg_fns},
 reg_count_{app_reg_count},
 update_fn_{update_fn},
 reset_fn_{reset_fn},
 HarpCore(who_am_i, hw_version_major, hw_version_minor,
          assembly_version, harp_version_major, harp_version_minor,
          fw_version_major, fw_version_minor, serial_number, name, tag)
{
    // Call base class constructor.
    // Create a ptr to the first (and only) derived class instance created.
    if (self == nullptr)
        self = this;
}

HarpCApp::~HarpCApp(){self = nullptr;}

void HarpCApp::handle_buffered_app_message()
{
    msg_t msg = get_buffered_msg();
    // Ignore out-of-range msgs.
    if (msg.header.address < APP_REG_START_ADDRESS ||
        msg.header.address >= (APP_REG_START_ADDRESS + reg_count_))
        return;
    uint8_t app_reg_address = msg.header.address - APP_REG_START_ADDRESS;
    switch (msg.header.type)
    {
        // Note: handler functions take the full address, but they live in
        // pairs in a separate struct indexed by app register address.
        case READ:
            reg_fns_[app_reg_address].read_fn_ptr(msg.header.address);
            break;
        case WRITE:
            reg_fns_[app_reg_address].write_fn_ptr(msg);
            break;
        default:
        {
            break;
        }
    }
    clear_msg();
}

void HarpCApp::dump_app_registers()
{
    for (uint8_t address = APP_REG_START_ADDRESS;
         address < reg_count_ + APP_REG_START_ADDRESS; ++address)
        send_harp_reply(READ, address);
}
