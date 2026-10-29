#pragma once
/**
 * usb_cdc_scanner.h  --  USB CDC-ACM barcode-scanner reader (native USB host)
 *
 * For a barcode scanner that presents a USB virtual COM port (CDC-ACM class)
 * instead of a HID keyboard. The ESP32-S3 native USB runs as a HOST (USB_SEL
 * LOW, same as the HID path) and uses the ESP-IDF `usb_host_cdc_acm` component.
 *
 * Same contract as usb_hid_scanner.*: characters stream in on the CDC bulk-IN
 * endpoint and are accumulated until a CR/LF terminator, then the completed
 * barcode is handed to the consumer via usb_cdc_scanner_take().
 *
 * Selected at build time by CONFIG_SCANNER_USB_CDC (app_config.h) through the
 * scanner.h wrapper. When that flag is 0 this whole module compiles to nothing.
 */

#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Install the USB host + CDC-ACM driver and start the open/retry task. Call once
 * in setup(), AFTER USB_SEL is driven LOW (GPIO19/20 routed to the USB-C port). */
void usb_cdc_scanner_init(void);

/* Poll+clear the latest completed barcode into `buf` (NUL-terminated). Returns
 * true exactly once per completed scan. */
bool usb_cdc_scanner_take(char *buf, size_t len);

/* True while a CDC-ACM device is open (scanner plugged in + enumerated). */
bool usb_cdc_scanner_connected(void);

#ifdef __cplusplus
}
#endif
