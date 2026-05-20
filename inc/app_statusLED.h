#pragma once 
#include "bb_statusLED.h"
#include <harp_c_app.h>


/**
 * Status LED application registers.
 *
 * Harp application registers must be exposed at APP_REG_START_ADDRESS or above.
 * These are app-relative indices 0..3, so the wire/register addresses are:
 *   LED_MODE        = APP_REG_START_ADDRESS + 0 = 32
 *   LED_COLOR_RED   = APP_REG_START_ADDRESS + 1 = 33
 *   LED_COLOR_GREEN = APP_REG_START_ADDRESS + 2 = 34
 *   LED_COLOR_BLUE  = APP_REG_START_ADDRESS + 3 = 35
 *
 * The originally proposed addresses 18..21 are not used because this core
 * defines core registers at 0..17 and APP_REG_START_ADDRESS as 32.
 */
namespace StatusLedAppRegs
{
    constexpr uint8_t LED_MODE_REL = 0;
    constexpr uint8_t LED_COLOR_RED_REL = 1;
    constexpr uint8_t LED_COLOR_GREEN_REL = 2;
    constexpr uint8_t LED_COLOR_BLUE_REL = 3;

    constexpr uint8_t LED_MODE = APP_REG_START_ADDRESS + LED_MODE_REL;
    constexpr uint8_t LED_COLOR_RED = APP_REG_START_ADDRESS + LED_COLOR_RED_REL;
    constexpr uint8_t LED_COLOR_GREEN = APP_REG_START_ADDRESS + LED_COLOR_GREEN_REL;
    constexpr uint8_t LED_COLOR_BLUE = APP_REG_START_ADDRESS + LED_COLOR_BLUE_REL;

    constexpr size_t REGISTER_COUNT = 4;

#pragma pack(push, 1)
    struct RegValues
    {
        volatile uint8_t R_LED_MODE;
        volatile uint8_t R_LED_COLOR_RED;
        volatile uint8_t R_LED_COLOR_GREEN;
        volatile uint8_t R_LED_COLOR_BLUE;
    };
#pragma pack(pop)

    extern RegValues values;
    extern RegSpecs specs[REGISTER_COUNT];
    extern RegFnPair functions[REGISTER_COUNT];

    void update();
    void reset();

    void write_led_mode(msg_t& msg);
    void write_led_color_component(msg_t& msg);
}