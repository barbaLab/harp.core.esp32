#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <cstring>

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

// ── Combined register map ──────────────────────────────────────────────────
constexpr size_t kTotalRegCount =
    StatusLedAppRegs::REGISTER_COUNT +
    LoadCellAppRegs::REGISTER_COUNT;

RegSpecs  g_app_specs[kTotalRegCount];
RegFnPair g_app_fns[kTotalRegCount];

bb::LoadCellArray g_loadcells;
bb::LoadCellConfig g_loadcell_cfg{};

void combined_update()
{
    StatusLedAppRegs::update();
    LoadCellAppRegs::update();
}

void combined_reset()
{
    StatusLedAppRegs::reset();
    LoadCellAppRegs::reset();
}

void build_register_map()
{
    std::memcpy(g_app_specs,
                StatusLedAppRegs::specs,
                StatusLedAppRegs::REGISTER_COUNT * sizeof(RegSpecs));
    std::memcpy(g_app_specs + StatusLedAppRegs::REGISTER_COUNT,
                LoadCellAppRegs::specs,
                LoadCellAppRegs::REGISTER_COUNT * sizeof(RegSpecs));

    std::memcpy(g_app_fns,
                StatusLedAppRegs::functions,
                StatusLedAppRegs::REGISTER_COUNT * sizeof(RegFnPair));
    std::memcpy(g_app_fns + StatusLedAppRegs::REGISTER_COUNT,
                LoadCellAppRegs::functions,
                LoadCellAppRegs::REGISTER_COUNT * sizeof(RegFnPair));
}

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

    // ── Hardware init ──────────────────────────────────────────────────────
    StatusLed::init();
    ESP_ERROR_CHECK(g_loadcells.init(g_loadcell_cfg));

    // ── App-layer init ─────────────────────────────────────────────────────
    LoadCellAppRegs::init(g_loadcells);
    StatusLedAppRegs::reset();
    LoadCellAppRegs::reset();

    // ── Flat register map ──────────────────────────────────────────────────
    build_register_map();

    HarpCApp::init(
        kWhoAmI,
        kHwVersionMajor, kHwVersionMinor,
        kAssemblyVersion,
        HARP_VERSION_MAJOR, HARP_VERSION_MINOR,
        kFwVersionMajor, kFwVersionMinor,
        kSerialNumber,
        kDeviceName,
        kTag,
        nullptr, // app register values are accessed via the RegSpecs base_ptrs
        g_app_specs,
        g_app_fns,
        kTotalRegCount,
        combined_update,
        combined_reset);

    ESP_LOGI(kMainLogTag, "HarpCApp initialized");

    if (kEnableUartSync)
    {
        auto& sync = HarpSynchronizer::init(kSyncUartPort, kSyncUartRxPin);
        HarpCore::set_synchronizer(&sync);
        ESP_LOGI(kMainLogTag, "UART synchronizer enabled on uart=%d rx_pin=%u",
                 static_cast<int>(kSyncUartPort), static_cast<unsigned>(kSyncUartRxPin));
    }

    ESP_ERROR_CHECK(g_loadcells.tare(32, pdMS_TO_TICKS(5)));
    ESP_ERROR_CHECK(g_loadcells.start());

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
