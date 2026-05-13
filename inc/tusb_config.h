/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2020 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#ifndef _PICO_STDIO_USB_TUSB_CONFIG_H
#define _PICO_STDIO_USB_TUSB_CONFIG_H

// ---------------------------------------------------------------------------
// ESP32-S3: native USB 1.1 OTG on GPIO19/20 (RHPORT 0, device, full-speed).
// TinyUSB is pulled in via idf_component.yml / espressif/esp_tinyusb.
// CFG_TUSB_OS must be OPT_OS_FREERTOS because ESP-IDF runs FreeRTOS.
// ---------------------------------------------------------------------------
#define CFG_TUSB_MCU            OPT_MCU_ESP32S3
#define CFG_TUSB_RHPORT0_MODE   (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#define CFG_TUSB_OS             OPT_OS_FREERTOS
#define BOARD_DEVICE_RHPORT_NUM 0

#define CFG_TUD_CDC             (1)
#define CFG_TUD_CDC_RX_BUFSIZE  (256)
#define CFG_TUD_CDC_TX_BUFSIZE  (256)

// We use a vendor specific interface but with our own driver
#define CFG_TUD_VENDOR          (0)

#endif // _PICO_STDIO_USB_TUSB_CONFIG_H
