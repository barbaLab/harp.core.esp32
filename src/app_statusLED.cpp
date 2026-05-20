#include "app_statusLED.h"
#include "bb_statusLED.h"

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

