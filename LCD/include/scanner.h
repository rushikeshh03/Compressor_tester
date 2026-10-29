#pragma once
/**
 * scanner.h  --  barcode-input selector
 *
 * Picks the USB scanner driver at build time from CONFIG_SCANNER_USB_CDC:
 *   1 → USB CDC-ACM serial scanner (usb_cdc_scanner.*)
 *   0 → USB-HID keyboard scanner   (usb_hid_scanner.*)   ← roll-back default
 *
 * Consumers (main.cpp, session.cpp) call scanner_init/take/connected and never
 * need to know which driver is active. To roll back, set the flag to 0.
 */
#include <stddef.h>
#include <stdbool.h>
#include "app_config.h"
#include "usb_hid_scanner.h"
#include "usb_cdc_scanner.h"

static inline void scanner_init(void)
{
#if CONFIG_SCANNER_USB_CDC
    usb_cdc_scanner_init();
#else
    usb_hid_scanner_init();
#endif
}

static inline bool scanner_take(char *buf, size_t len)
{
#if CONFIG_SCANNER_USB_CDC
    return usb_cdc_scanner_take(buf, len);
#else
    return usb_hid_scanner_take(buf, len);
#endif
}

static inline bool scanner_connected(void)
{
#if CONFIG_SCANNER_USB_CDC
    return usb_cdc_scanner_connected();
#else
    return usb_hid_scanner_connected();
#endif
}
