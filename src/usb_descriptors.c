// TinyUSB CDC descriptor callbacks for the Harp device.
// Provides manufacturer/product/serial strings and CDC configuration
// specific to the ESP32-S3 platform.

#ifndef USBD_MANUFACTURER
#define USBD_MANUFACTURER "University of Genoa"
#endif

#ifndef USBD_PRODUCT
#define USBD_PRODUCT "Harp Device"
#endif

#include "tusb.h"
#include <esp_efuse.h>
#include <esp_mac.h>

// ESP32-S3 MAC address is 6 bytes -> 12 hex chars + NUL terminator.
#define HARP_SERIAL_STR_LEN 13

/**
 * \brief Populate \p out with a unique hex serial string derived from the
 *        ESP32-S3 base MAC address read from eFuse.
 *
 * The MAC is 6 bytes long, yielding a 12-character uppercase hex string.
 * If \p len is less than 13 the output is set to an empty string.
 */
static void fill_serial_string(char *out, size_t len)
{
    uint8_t mac[6] = {0};
    static const char hex[] = "0123456789ABCDEF";

    esp_efuse_mac_get_default(mac);

    if (len < 13) {
        if (len) out[0] = '\0';
        return;
    }
    for (size_t i = 0; i < 6; ++i) {
        out[i * 2]     = hex[(mac[i] >> 4) & 0xF];
        out[i * 2 + 1] = hex[mac[i]        & 0xF];
    }
    out[12] = '\0';
}

#define USBD_DESC_STR_MAX (64) // Override default of 20 (max 127).

#ifndef USBD_VID
#define USBD_VID (0x303A) // Espressif's VID
#endif

#ifndef USBD_PID
#define USBD_PID (0x000a) // (placeholder — replace with your assigned PID)
#endif

#define USBD_DESC_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)
#define USBD_CONFIGURATION_DESCRIPTOR_ATTRIBUTE (0)
#define USBD_MAX_POWER_MA (250)

#define USBD_ITF_CDC       (0) // needs 2 interfaces
#define USBD_ITF_MAX       (2)

#define USBD_CDC_EP_CMD (0x81)
#define USBD_CDC_EP_OUT (0x02)
#define USBD_CDC_EP_IN (0x82)
#define USBD_CDC_CMD_MAX_SIZE (8)
#define USBD_CDC_IN_OUT_MAX_SIZE (64)

#define USBD_STR_0 (0x00)
#define USBD_STR_MANUF (0x01)
#define USBD_STR_PRODUCT (0x02)
#define USBD_STR_SERIAL (0x03)
#define USBD_STR_CDC (0x04)

// Note: descriptors returned from callbacks must exist long enough for transfer to complete

static const tusb_desc_device_t usbd_desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USBD_VID,
    .idProduct = USBD_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = USBD_STR_MANUF,
    .iProduct = USBD_STR_PRODUCT,
    .iSerialNumber = USBD_STR_SERIAL,
    .bNumConfigurations = 1,
};

static const uint8_t usbd_desc_cfg[USBD_DESC_LEN] = {
    TUD_CONFIG_DESCRIPTOR(1, USBD_ITF_MAX, USBD_STR_0, USBD_DESC_LEN,
        USBD_CONFIGURATION_DESCRIPTOR_ATTRIBUTE, USBD_MAX_POWER_MA),

    TUD_CDC_DESCRIPTOR(USBD_ITF_CDC, USBD_STR_CDC, USBD_CDC_EP_CMD,
        USBD_CDC_CMD_MAX_SIZE, USBD_CDC_EP_OUT, USBD_CDC_EP_IN, USBD_CDC_IN_OUT_MAX_SIZE),
};

static char usbd_serial_str[HARP_SERIAL_STR_LEN];

static const char *const usbd_desc_str[] = {
    [USBD_STR_MANUF] = USBD_MANUFACTURER,
    [USBD_STR_PRODUCT] = USBD_PRODUCT,
    [USBD_STR_SERIAL] = usbd_serial_str,
    [USBD_STR_CDC] = "Board CDC",
};

const uint8_t *tud_descriptor_device_cb(void) {
    return (const uint8_t *)&usbd_desc_device;
}

const uint8_t *tud_descriptor_configuration_cb(__unused uint8_t index) {
    return usbd_desc_cfg;
}

const uint16_t *tud_descriptor_string_cb(uint8_t index, __unused uint16_t langid) {
    static uint16_t desc_str[USBD_DESC_STR_MAX];

    // Populate serial string lazily on first call.
    if (!usbd_serial_str[0]) {
        fill_serial_string(usbd_serial_str, sizeof(usbd_serial_str));
    }

    uint8_t len;
    if (index == 0) {
        desc_str[1] = 0x0409; // supported language is English
        len = 1;
    } else {
        if (index >= sizeof(usbd_desc_str) / sizeof(usbd_desc_str[0])) {
            return NULL;
        }
        const char *str = usbd_desc_str[index];
        for (len = 0; len < USBD_DESC_STR_MAX - 1 && str[len]; ++len) {
            desc_str[1 + len] = str[len];
        }
    }

    // first byte is length (including header), second byte is string type
    desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * len + 2));

    return desc_str;
}
