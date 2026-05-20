#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>

#include <harp_c_app.h>
#include <harp_core.h>
#include <harp_synchronizer.h>

#include "app_statusLED.h"
#include "app_loadcell.h"
#include "bb_statusLED.h"
#include "bb_loadcell.h"

namespace {

constexpr char kMainLogTag[] = "main";

constexpr uint16_t kWhoAmI = 0x0001;
constexpr uint8_t kHwVersionMajor = 1;
constexpr uint8_t kHwVersionMinor = 0;
constexpr uint8_t kAssemblyVersion = 1;
constexpr uint8_t kFwVersionMajor = 0;
constexpr uint8_t kFwVersionMinor = 1;
constexpr uint16_t kSerialNumber = 1;
constexpr char kDeviceName[] = "Harp ESP32";
constexpr uint8_t kTag[] = "ESP32";

constexpr bool kEnableUartSync = false;
constexpr uart_port_t kSyncUartPort = UART_NUM_1;
constexpr uint8_t kSyncUartRxPin = 4;

constexpr TickType_t kHarpLoopDelayTicks = 1;
constexpr TickType_t kAppLoopDelayTicks = 1;

constexpr uint32_t kHarpTaskStackWords = 4096;
constexpr UBaseType_t kHarpTaskPriority = 8;
constexpr BaseType_t kHarpTaskCore = 0;

constexpr uint32_t kAppTaskStackWords = 4096;
constexpr UBaseType_t kAppTaskPriority = 5;
constexpr BaseType_t kAppTaskCore = 1;

void app_state_machine_step()
{
    // Placeholder for the future behavior-box state machine.
    // Do not call StatusLed::update() here while StatusLedAppRegs::update
    // is passed into HarpCApp::init(), otherwise the LED blink state will be
    // advanced from two different tasks.
}

void harp_task(void* /*arg*/)
{
    for (;;)
    {
        HarpCApp::instance().run();
        vTaskDelay(kHarpLoopDelayTicks);
    }
}

void app_task(void* /*arg*/)
{
    for (;;)
    {
        app_state_machine_step();
        vTaskDelay(kAppLoopDelayTicks);
    }
}

} // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(kMainLogTag, "app_main start");

    StatusLed::init();
    StatusLedAppRegs::reset();

    HarpCApp::init(
        kWhoAmI,
        kHwVersionMajor,
        kHwVersionMinor,
        kAssemblyVersion,
        HARP_VERSION_MAJOR,
        HARP_VERSION_MINOR,
        kFwVersionMajor,
        kFwVersionMinor,
        kSerialNumber,
        kDeviceName,
        kTag,
        &StatusLedAppRegs::values,
        StatusLedAppRegs::specs,
        StatusLedAppRegs::functions,
        StatusLedAppRegs::REGISTER_COUNT,
        StatusLedAppRegs::update,
        StatusLedAppRegs::reset);

    ESP_LOGI(kMainLogTag, "HarpCApp initialized");

    if (kEnableUartSync)
    {
        auto& sync = HarpSynchronizer::init(kSyncUartPort, kSyncUartRxPin);
        HarpCore::set_synchronizer(&sync);
        ESP_LOGI(kMainLogTag, "UART synchronizer enabled on uart=%d rx_pin=%u",
                 static_cast<int>(kSyncUartPort), static_cast<unsigned>(kSyncUartRxPin));
    }

    ESP_LOGI(kMainLogTag, "Entering Harp main loop");

    const BaseType_t harp_task_ok = xTaskCreatePinnedToCore(
        harp_task,
        "harp_task",
        kHarpTaskStackWords,
        nullptr,
        kHarpTaskPriority,
        nullptr,
        kHarpTaskCore);

    const BaseType_t app_task_ok = xTaskCreatePinnedToCore(
        app_task,
        "app_task",
        kAppTaskStackWords,
        nullptr,
        kAppTaskPriority,
        nullptr,
        kAppTaskCore);

    if (harp_task_ok != pdPASS || app_task_ok != pdPASS)
    {
        ESP_LOGE(kMainLogTag, "Failed to create tasks: harp=%d app=%d",
                 static_cast<int>(harp_task_ok), static_cast<int>(app_task_ok));
        for (;;)
        {
            vTaskDelay(portMAX_DELAY);
        }
    }

    vTaskDelete(nullptr);
}
