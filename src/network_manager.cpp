// network_manager.cpp
// Handles Wi-Fi STA association and TCP client lifecycle.

#include "network_manager.h"

#include <string.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <lwip/inet.h>

static const char* kNetMgrLogTag = "NetMgr";

namespace NetworkManager {

// ── Internal state ────────────────────────────────────────────────────────────
static std::atomic<bool> s_wifi_connected{false};
static std::atomic<bool> s_tcp_connected{false};
static std::atomic<int>  s_tcp_sock{-1};

static TaskHandle_t s_task_handle = nullptr;
static esp_netif_t* s_netif_sta   = nullptr;
static RegValues* s_regs          = nullptr;

// Message to the network task
enum class NetCmd : uint8_t { APPLY, DISCONNECT };
static NetCmd s_cmd = NetCmd::DISCONNECT;

// Snapshot of config passed to apply() -- copied to avoid race
struct CfgSnapshot {
    char     ssid[32];
    char     password[64];
    char     server_ip[16];
    uint16_t server_port;
    uint8_t  enable;
};
static CfgSnapshot s_cfg{};

static constexpr uint8_t NET_ENABLE_WIFI = 1u << 0;
static constexpr uint8_t NET_ENABLE_TCP = 1u << 1;
static constexpr uint8_t NET_STATUS_CFG_VALID = 1u << 2;
static constexpr uint8_t NET_STATUS_WIFI_UP = 1u << 3;
static constexpr uint8_t NET_STATUS_IP_OK = 1u << 4;
static constexpr uint8_t NET_STATUS_TCP_CONN = 1u << 5;

static inline void set_status_bits(uint8_t bits_to_set, uint8_t bits_to_clear)
{
    if (s_regs == nullptr)
        return;
    uint8_t cfg = s_regs->R_NET_CONFIG;
    cfg |= bits_to_set;
    cfg &= static_cast<uint8_t>(~bits_to_clear);
    s_regs->R_NET_CONFIG = cfg;
}

// ── Wi-Fi event handler ───────────────────────────────────────────────────────
static void wifi_event_handler(void* arg, esp_event_base_t base,
                               int32_t id, void* data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(kNetMgrLogTag, "Wi-Fi connected to AP: %s", s_cfg.ssid);
        set_status_bits(NET_STATUS_WIFI_UP, 0);
    }
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
        s_tcp_connected  = false;
        int fd = s_tcp_sock.exchange(-1);
        if (fd >= 0) lwip_close(fd);
        ESP_LOGW(kNetMgrLogTag, "Wi-Fi disconnected");
        set_status_bits(0, NET_STATUS_WIFI_UP | NET_STATUS_IP_OK | NET_STATUS_TCP_CONN);
        // Attempt reconnect if we still hold a valid config
        if (s_cfg.ssid[0] != '\0') {
            esp_err_t err = esp_wifi_connect();
            if (err != ESP_OK && err != ESP_ERR_WIFI_CONN) {
                ESP_LOGW(kNetMgrLogTag,
                         "Reconnect request failed: %s (%d)",
                         esp_err_to_name(err),
                         static_cast<int>(err));
            }
        }
    }
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* ev = (ip_event_got_ip_t*)data;
        ESP_LOGI(kNetMgrLogTag, "Got IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        s_wifi_connected = true;
        set_status_bits(NET_STATUS_IP_OK, 0);
        // Notify the network task to open the TCP socket
        if (s_task_handle) xTaskNotifyGive(s_task_handle);
    }
}

// ── TCP connect helper ────────────────────────────────────────────────────────
static int tcp_connect(const char* ip, uint16_t port)
{
    struct addrinfo hints{};
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    char port_str[8];
    snprintf(port_str, sizeof(port_str), "%u", port);

    struct addrinfo* res = nullptr;
    if (getaddrinfo(ip, port_str, &hints, &res) != 0 || res == nullptr) {
        ESP_LOGE(kNetMgrLogTag, "getaddrinfo failed for %s", ip);
        return -1;
    }

    int sock = lwip_socket(res->ai_family, res->ai_socktype, 0);
    if (sock < 0) { freeaddrinfo(res); return -1; }

    int flag = 1;
    lwip_setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    if (lwip_connect(sock, res->ai_addr, res->ai_addrlen) != 0) {
        ESP_LOGE(kNetMgrLogTag, "connect to %s:%u failed: errno %d", ip, port, errno);
        lwip_close(sock);
        freeaddrinfo(res);
        return -1;
    }

    freeaddrinfo(res);
    ESP_LOGI(kNetMgrLogTag, "TCP connected to %s:%u fd=%d", ip, port, sock);
    return sock;
}

// ── Network task ──────────────────────────────────────────────────────────────
static void network_task(void* /*arg*/)
{
    for (;;) {
        // Block until the Harp task notifies us or Wi-Fi gets an IP
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (s_cmd == NetCmd::DISCONNECT) {
            int fd = s_tcp_sock.exchange(-1);
            if (fd >= 0) lwip_close(fd);
            s_tcp_connected = false;
            esp_wifi_disconnect();
            continue;
        }

        // APPLY: Wi-Fi is already up (IP_EVENT triggered this), try TCP
        if (!s_wifi_connected || s_cfg.server_ip[0] == '\0') continue;
        if (!(s_cfg.enable & NET_ENABLE_TCP)) continue;

        // Close old socket if present
        int old_fd = s_tcp_sock.exchange(-1);
        if (old_fd >= 0) lwip_close(old_fd);
        s_tcp_connected = false;

        int fd = tcp_connect(s_cfg.server_ip, s_cfg.server_port);
        if (fd >= 0) {
            s_tcp_sock  = fd;
            s_tcp_connected = true;
            set_status_bits(NET_STATUS_TCP_CONN, 0);
        } else {
            set_status_bits(0, NET_STATUS_TCP_CONN);
            ESP_LOGE(kNetMgrLogTag, "TCP connect failed; retry in 5 s");
            vTaskDelay(pdMS_TO_TICKS(5000));
            xTaskNotifyGive(s_task_handle); // retry
        }
    }
}

// ── Public API ────────────────────────────────────────────────────────────────
void init(RegValues* regs)
{
    s_regs = regs;

    // Wi-Fi stores calibration/config in NVS; initialize it first.
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_netif_sta = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,  wifi_event_handler, nullptr);
    esp_event_handler_register(IP_EVENT,   IP_EVENT_STA_GOT_IP, wifi_event_handler, nullptr);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    
    xTaskCreatePinnedToCore(network_task, "net_task", 4096, nullptr, 5, &s_task_handle, 0);
    ESP_LOGI(kNetMgrLogTag, "NetworkManager initialised");
}

void apply()
{
    if (s_regs == nullptr) {
        ESP_LOGW(kNetMgrLogTag, "NetworkManager not bound to core registers");
        return;
    }

    if (!(s_regs->R_NET_CONFIG & NET_ENABLE_WIFI)) {
        ESP_LOGI(kNetMgrLogTag, "Wi-Fi not enabled; skipping");
        return;
    }

    // Snapshot config
    memcpy(s_cfg.ssid,      (const void*)s_regs->R_NET_SSID,       sizeof(s_cfg.ssid));
    memcpy(s_cfg.password,  (const void*)s_regs->R_NET_PASSWORD,   sizeof(s_cfg.password));
    memcpy(s_cfg.server_ip, (const void*)s_regs->R_NET_SERVER_IP,  sizeof(s_cfg.server_ip));
    s_cfg.server_port = static_cast<uint16_t>(s_regs->R_NET_SERVER_PORT[0]) |
                        (static_cast<uint16_t>(s_regs->R_NET_SERVER_PORT[1]) << 8);
    s_cfg.enable = s_regs->R_NET_CONFIG;

    // Ensure null-termination for safety when passing to C APIs.
    s_cfg.ssid[sizeof(s_cfg.ssid) - 1] = '\0';
    s_cfg.password[sizeof(s_cfg.password) - 1] = '\0';
    s_cfg.server_ip[sizeof(s_cfg.server_ip) - 1] = '\0';
    set_status_bits(NET_STATUS_CFG_VALID, NET_STATUS_WIFI_UP | NET_STATUS_IP_OK | NET_STATUS_TCP_CONN);

    // (Re)configure the station
    wifi_config_t wifi_cfg{};
    memcpy(wifi_cfg.sta.ssid,     s_cfg.ssid,     sizeof(wifi_cfg.sta.ssid));
    memcpy(wifi_cfg.sta.password, s_cfg.password, sizeof(wifi_cfg.sta.password));
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    esp_err_t err = esp_wifi_disconnect();
    if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_CONNECT) {
        ESP_LOGW(kNetMgrLogTag,
                 "esp_wifi_disconnect returned: %s (%d)",
                 esp_err_to_name(err),
                 static_cast<int>(err));
    }

    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(kNetMgrLogTag,
                 "esp_wifi_set_config failed: %s (%d)",
                 esp_err_to_name(err),
                 static_cast<int>(err));
        return;
    }

    err = esp_wifi_connect();
    if (err == ESP_ERR_WIFI_CONN) {
        ESP_LOGW(kNetMgrLogTag, "Wi-Fi already connecting; reusing in-flight attempt");
    } else if (err != ESP_OK) {
        ESP_LOGE(kNetMgrLogTag,
                 "esp_wifi_connect failed: %s (%d)",
                 esp_err_to_name(err),
                 static_cast<int>(err));
        return;
    }

    s_cmd = NetCmd::APPLY;
    // The task will be woken by the IP_EVENT when the association completes.
    ESP_LOGI(kNetMgrLogTag, "Connecting to SSID: %s", s_cfg.ssid);
}

void disconnect()
{
    s_cmd = NetCmd::DISCONNECT;
    if (s_task_handle) xTaskNotifyGive(s_task_handle);
}

bool is_wifi_connected() { return s_wifi_connected; }
bool is_tcp_connected()  { return s_tcp_connected; }
int  tcp_socket()        { return s_tcp_sock; }

void tcp_write(const uint8_t* data, size_t len)
{
    int fd = s_tcp_sock.load();
    if (fd < 0 || data == nullptr || len == 0)
        return;

    int written = lwip_send(fd, data, len, 0);
    if (written < 0) {
        ESP_LOGW(kNetMgrLogTag, "tcp_write failed: errno %d", errno);
        int old_fd = s_tcp_sock.exchange(-1);
        if (old_fd >= 0)
            lwip_close(old_fd);
        s_tcp_connected = false;
        set_status_bits(0, NET_STATUS_TCP_CONN);
    }
}

int tcp_read(uint8_t* data, size_t len)
{
    int fd = s_tcp_sock.load();
    if (fd < 0 || data == nullptr || len == 0)
        return 0;

    int n = lwip_recv(fd, data, len, MSG_DONTWAIT);
    if (n > 0)
        return n;

    if (n == 0) {
        ESP_LOGW(kNetMgrLogTag, "TCP peer closed connection");
    } else {
        if (errno == EWOULDBLOCK || errno == EAGAIN)
            return 0;
        ESP_LOGW(kNetMgrLogTag, "tcp_read failed: errno %d", errno);
    }

    int old_fd = s_tcp_sock.exchange(-1);
    if (old_fd >= 0)
        lwip_close(old_fd);
    s_tcp_connected = false;
    set_status_bits(0, NET_STATUS_TCP_CONN);
    return -1;
}

} // namespace NetworkManager
