#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>

#include <harp_core.h>
#include <harp_synchronizer.h>



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

// Set true to lock Harp time to an external sync pulse source.
constexpr bool kEnableUartSync = false;
constexpr uart_port_t kSyncUartPort = UART_NUM_1;
constexpr uint8_t kSyncUartRxPin = 4;

// Keep this low to preserve USB/Harp responsiveness.
constexpr TickType_t kCoreLoopDelayTicks = 1;

constexpr uint32_t kCoreTaskStackWords = 4096;
constexpr UBaseType_t kCoreTaskPriority = 8;
constexpr BaseType_t kCoreTaskCore = 1;

void core_task(void* /*arg*/)
{
    for (;;)
    {
        HarpCore::instance().run();
        vTaskDelay(kCoreLoopDelayTicks);
    }
}

} // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(kMainLogTag, "app_main start");

    // Core-only initialization template.
    // Future integration point: in an application repo that depends on this
    // component, replace this with HarpCApp::init(...) (or another
    // HarpCore-derived class) once app registers and handlers are available.
    HarpCore::init(
        kWhoAmI,
        kHwVersionMajor, kHwVersionMinor,
        kAssemblyVersion,
        HARP_VERSION_MAJOR, HARP_VERSION_MINOR,
        kFwVersionMajor, kFwVersionMinor,
        kSerialNumber,
        kDeviceName,
        kTag);

    ESP_LOGI(kMainLogTag, "HarpCore initialized (core-only mode)");

    if (kEnableUartSync)
    {
        auto& sync = HarpSynchronizer::init(kSyncUartPort, kSyncUartRxPin);
        HarpCore::set_synchronizer(&sync);
        ESP_LOGI(kMainLogTag, "UART synchronizer enabled on uart=%d rx_pin=%u",
                 static_cast<int>(kSyncUartPort), static_cast<unsigned>(kSyncUartRxPin));
    }

    const BaseType_t core_task_ok = xTaskCreatePinnedToCore(
        core_task,
        "core_task",
        kCoreTaskStackWords,
        nullptr,
        kCoreTaskPriority,
        nullptr,
        kCoreTaskCore);

    if (core_task_ok != pdPASS)
    {
        ESP_LOGE(kMainLogTag, "Failed to create core task: %d",
                 static_cast<int>(core_task_ok));
        for (;;)
        {
            vTaskDelay(portMAX_DELAY);
        }
    }

    vTaskDelete(nullptr);
}
