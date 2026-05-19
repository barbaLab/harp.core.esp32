#include "status_led.h"

#include <atomic>
#include <cstdint>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_idf_version.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "led_strip.h"
#include "led_strip_rmt.h"

namespace {

constexpr char kTag[] = "status_led";

// Your previous Arduino example used GPIO 48 as fallback for the onboard RGB LED.
// Change this if your specific ESP32 board uses a different RGB LED pin.
constexpr int kLedGpio = 48;

// For most onboard RGB LEDs this should be 1.
constexpr uint32_t kLedCount = 1;

// WS2812/NeoPixel LEDs are usually GRB internally, but led_strip_set_pixel()
// takes arguments as red, green, blue. The driver handles the configured format.
//
// Your previous color[] was:
//   Green = 0x55
//   Red   = 0x11
//   Blue  = 0x77
constexpr uint8_t kDefaultRed = 0x11;
constexpr uint8_t kDefaultGreen = 0x55;
constexpr uint8_t kDefaultBlue = 0x77;

constexpr uint8_t kFaultRed = 0x40;
constexpr uint8_t kFaultGreen = 0x00;
constexpr uint8_t kFaultBlue = 0x00;

constexpr uint8_t kAutoRed = 0x00;
constexpr uint8_t kAutoGreen = 0x10;
constexpr uint8_t kAutoBlue = 0x20;

constexpr uint16_t kMinBlinkPeriodMs = 50;
constexpr uint16_t kDefaultBlinkPeriodMs = 500;
constexpr uint16_t kFaultBlinkPeriodMs = 150;

led_strip_handle_t g_led_strip = nullptr;

std::atomic<StatusLed::Mode> g_mode{StatusLed::Mode::Off};
std::atomic<bool> g_state{false};
std::atomic<uint16_t> g_blink_period_ms{kDefaultBlinkPeriodMs};

TickType_t g_last_toggle_tick = 0;

struct Rgb {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

constexpr Rgb kOffColor{
    .red = 0x00,
    .green = 0x00,
    .blue = 0x00,
};

constexpr Rgb kOnColor{
    .red = kDefaultRed,
    .green = kDefaultGreen,
    .blue = kDefaultBlue,
};

constexpr Rgb kFaultColor{
    .red = kFaultRed,
    .green = kFaultGreen,
    .blue = kFaultBlue,
};

constexpr Rgb kAutoColor{
    .red = kAutoRed,
    .green = kAutoGreen,
    .blue = kAutoBlue,
};

Rgb color_for_mode(StatusLed::Mode mode)
{
    switch (mode)
    {
    case StatusLed::Mode::On:
    case StatusLed::Mode::Blink:
        return kOnColor;

    case StatusLed::Mode::Auto:
        return kAutoColor;

    case StatusLed::Mode::Fault:
        return kFaultColor;

    case StatusLed::Mode::Off:
    default:
        return kOffColor;
    }
}

void apply_color(Rgb color)
{
    if (g_led_strip == nullptr)
    {
        return;
    }

    for (uint32_t i = 0; i < kLedCount; ++i)
    {
        esp_err_t err = led_strip_set_pixel(
            g_led_strip,
            i,
            color.red,
            color.green,
            color.blue);

        if (err != ESP_OK)
        {
            ESP_LOGW(kTag, "led_strip_set_pixel failed: %s", esp_err_to_name(err));
            return;
        }
    }

    esp_err_t err = led_strip_refresh(g_led_strip);
    if (err != ESP_OK)
    {
        ESP_LOGW(kTag, "led_strip_refresh failed: %s", esp_err_to_name(err));
    }
}

void apply_current_state()
{
    const bool on = g_state.load(std::memory_order_relaxed);
    const auto mode = g_mode.load(std::memory_order_relaxed);

    if (!on)
    {
        apply_color(kOffColor);
        return;
    }

    apply_color(color_for_mode(mode));
}

} // namespace

namespace StatusLed {

void init()
{
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = kLedGpio;
    strip_config.max_leds = kLedCount;
    strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;    strip_config.led_model = LED_MODEL_WS2812;
    strip_config.flags.invert_out = false;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rmt_config.resolution_hz = 10 * 1000 * 1000; // 10 MHz, 0.1 us tick

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
    rmt_config.mem_block_symbols = 64;
#endif

    rmt_config.flags.with_dma = false;

    esp_err_t err = led_strip_new_rmt_device(
        &strip_config,
        &rmt_config,
        &g_led_strip);

    if (err != ESP_OK)
    {
        g_led_strip = nullptr;
        ESP_LOGE(kTag, "led_strip_new_rmt_device failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(kTag, "Initialized WS2812 status LED on GPIO %d", kLedGpio);

    g_mode.store(Mode::Off, std::memory_order_relaxed);
    g_state.store(false, std::memory_order_relaxed);
    g_blink_period_ms.store(kDefaultBlinkPeriodMs, std::memory_order_relaxed);
    g_last_toggle_tick = xTaskGetTickCount();

    apply_color(kOffColor);
}

void set_mode(Mode mode)
{
    g_mode.store(mode, std::memory_order_relaxed);
    g_last_toggle_tick = xTaskGetTickCount();

    switch (mode)
    {
    case Mode::Off:
        g_state.store(false, std::memory_order_relaxed);
        break;

    case Mode::On:
    case Mode::Auto:
    case Mode::Blink:
    case Mode::Fault:
        g_state.store(true, std::memory_order_relaxed);
        break;

    default:
        g_mode.store(Mode::Off, std::memory_order_relaxed);
        g_state.store(false, std::memory_order_relaxed);
        break;
    }

    apply_current_state();
}

Mode mode()
{
    return g_mode.load(std::memory_order_relaxed);
}

void set_state(bool on)
{
    g_state.store(on, std::memory_order_relaxed);
    apply_current_state();
}

bool state()
{
    return g_state.load(std::memory_order_relaxed);
}

void set_blink_period_ms(uint16_t period_ms)
{
    if (period_ms < kMinBlinkPeriodMs)
    {
        period_ms = kMinBlinkPeriodMs;
    }

    g_blink_period_ms.store(period_ms, std::memory_order_relaxed);
}

uint16_t blink_period_ms()
{
    return g_blink_period_ms.load(std::memory_order_relaxed);
}

void set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    const Rgb custom_color{.red = red, .green = green, .blue = blue};
    apply_color(custom_color);
}

void update()
{
    const auto current_mode = mode();

    if (current_mode != Mode::Blink && current_mode != Mode::Fault)
    {
        return;
    }

    const uint16_t period_ms =
        current_mode == Mode::Fault
            ? kFaultBlinkPeriodMs
            : blink_period_ms();

    TickType_t half_period_ticks = pdMS_TO_TICKS(period_ms / 2);
    if (half_period_ticks == 0)
    {
        half_period_ticks = 1;
    }

    const TickType_t now = xTaskGetTickCount();

    if ((now - g_last_toggle_tick) >= half_period_ticks)
    {
        g_last_toggle_tick = now;
        set_state(!state());
    }
}

} // namespace StatusLed