#pragma once

#include <cstdint>

namespace StatusLed {

enum class Mode : uint8_t {
    Off = 0,
    On = 1,
    Auto = 2,
    Blink = 3,
    Fault = 4,
};

void init();

void set_mode(Mode mode);
Mode mode();

void set_state(bool on);
bool state();

void set_blink_period_ms(uint16_t period_ms);
uint16_t blink_period_ms();

void set_rgb(uint8_t red, uint8_t green, uint8_t blue);

void update();

} // namespace StatusLed