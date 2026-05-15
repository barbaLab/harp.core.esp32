#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <harp_core.h>
#include <harp_synchronizer.h>

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include <esp_log.h>

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

} // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(kMainLogTag, "app_main start");

    HarpCore::init(kWhoAmI,
                   kHwVersionMajor, kHwVersionMinor,
                   kAssemblyVersion,
                   HARP_VERSION_MAJOR, HARP_VERSION_MINOR,
                   kFwVersionMajor, kFwVersionMinor,
                   kSerialNumber, kDeviceName, kTag);

    ESP_LOGI(kMainLogTag, "HarpCore initialized");

    if (kEnableUartSync)
    {
        auto& sync = HarpSynchronizer::init(kSyncUartPort, kSyncUartRxPin);
        HarpCore::set_synchronizer(&sync);
        ESP_LOGI(kMainLogTag, "UART synchronizer enabled on uart=%d rx_pin=%u",
                 (int)kSyncUartPort, (unsigned)kSyncUartRxPin);
    }

    ESP_LOGI(kMainLogTag, "Entering Harp main loop");

    for (;;)
    {
        HarpCore::instance().run();
        vTaskDelay(kHarpLoopDelayTicks);
    }
}