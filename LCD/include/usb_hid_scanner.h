#pragma once
/**
 * usb_hid_scanner.h  --  TEMPORARY USB-HID barcode-scanner reader (prototype)
 *
 * Reads a USB-HID barcode scanner (which enumerates as a USB keyboard) over the
 * ESP32-S3 native USB in HOST mode, using the EspUsbHost library. Characters are
 * accumulated until a CR/LF terminator, then the completed barcode is handed to
 * the UI (ui_set_barcode) and printed to UART0.
 *
 * This is the LCD's sole barcode input (BLE was removed). CAN runs independently
 * on an external TJA1050 (GPIO43/44), so the scanner and CAN coexist — see
 * app_config.h. USB host needs 5 V VBUS to the scanner (VBUS note in app_config.h).
 *
 * Threading: EspUsbHost decodes keys on its own background task; the callback
 * only buffers. Completed codes are consumed by harness_conn (which drives the
 * CAN check flow) via usb_hid_scanner_take(), from the LVGL loop under the lock.
 */

#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bring up the USB host stack + register the keyboard callback. Call once in
 * setup(), AFTER USB_SEL is driven LOW (GPIO19/20 routed to the USB-C port). */
void usb_hid_scanner_init(void);

/* Service completed barcodes: if a full code (CR/LF-terminated) has arrived
 * since the last call, push it to the UI + log it. Call from loop() under the
 * LVGL lock. */
void usb_hid_scanner_poll(void);

/* Poll+clear the latest completed barcode into `buf` (NUL-terminated). Returns
 * true exactly once per completed scan. Optional — usb_hid_scanner_poll()
 * already drives the UI; this is for callers that also want the raw string. */
bool usb_hid_scanner_take(char *buf, size_t len);

/* True while a USB device is enumerated (the scanner is plugged in + powered).
 * Drives the top-bar "Scanner" status chip. */
bool usb_hid_scanner_connected(void);

#ifdef __cplusplus
}
#endif
