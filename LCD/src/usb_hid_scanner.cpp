/**
 * usb_hid_scanner.cpp  --  TEMPORARY USB-HID barcode-scanner reader (prototype)
 *
 * See usb_hid_scanner.h. This is the LCD's sole barcode input (BLE was removed).
 *
 * A USB barcode scanner in HID mode enumerates as a boot-protocol USB keyboard:
 * it "types" the scanned code as key presses followed by Enter. EspUsbHost owns
 * the USB host driver + PHY (it uses usb/usb_host.h + esp_private/usb_phy.h — the
 * raw host API bundled with arduino-esp32 3.x; the ESP-IDF HID *class* driver is
 * not bundled, which is why we use this library rather than usb/hid_host.h).
 *
 * The onKeyboard callback runs on EspUsbHost's background task, so it only
 * appends to a buffer and, on a CR/LF, snapshots the finished code under a
 * spinlock. usb_hid_scanner_poll() (LVGL task) consumes the snapshot and drives
 * the UI — ui_* is never touched from the USB task.
 */

#include "usb_hid_scanner.h"
#include <Arduino.h>
#include <EspUsbHost.h>
#include <string.h>
#include "ui/ui.h"
#include "sd_logger.h"

#define BC_MAX  126   /* max barcode chars we accumulate (+1 for NUL) */

static EspUsbHost   s_usb;

/* Assembly buffer — written only by the USB-task callback. */
static char         s_asm[BC_MAX + 1] = {0};
static size_t       s_asm_len = 0;

/* Completed-code handoff (USB task → LVGL task), guarded by a spinlock. */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static char         s_done[BC_MAX + 1] = {0};
static volatile bool s_done_ready = false;

/* Commit the assembled code to the handoff slot (called from the USB task on a
 * terminator). Keeps only the most recent completed scan if the LVGL task hasn't
 * drained the previous one yet. */
static void commit_barcode(void)
{
    if (s_asm_len == 0) return;
    s_asm[s_asm_len] = '\0';
    portENTER_CRITICAL(&s_mux);
    memcpy(s_done, s_asm, s_asm_len + 1);
    s_done_ready = true;
    portEXIT_CRITICAL(&s_mux);
    s_asm_len = 0;
}

/* EspUsbHost keyboard callback (USB background task). event.ascii is the decoded
 * character with modifiers already applied; 0 for non-printable keys. Enter
 * arrives as ascii '\r' or '\n' (and/or keycode 0x28) — either terminates. */
static void on_key(const EspUsbHostKeyboardEvent &e)
{
    if (!e.pressed) return;                 /* key-down only (ignore releases) */
    uint8_t c = e.ascii;
    if (c == '\r' || c == '\n' || e.keycode == 0x28 /* HID Enter */) {
        commit_barcode();
        return;
    }
    if (c >= 0x20 && c < 0x7F && s_asm_len < BC_MAX)
        s_asm[s_asm_len++] = (char)c;       /* printable ASCII only */
}

void usb_hid_scanner_init(void)
{
    s_usb.onKeyboard(on_key);
    s_usb.begin();      /* installs the USB host driver + starts its task */
    Serial.println("[USB] HID scanner host started (USB_SEL must be LOW). "
                   "Scan a barcode to test.");
}

bool usb_hid_scanner_take(char *buf, size_t len)
{
    if (!buf || len == 0 || !s_done_ready) return false;
    portENTER_CRITICAL(&s_mux);
    strncpy(buf, s_done, len - 1);
    buf[len - 1] = '\0';
    s_done_ready = false;
    portEXIT_CRITICAL(&s_mux);
    return true;
}

void usb_hid_scanner_poll(void)
{
    char bc[BC_MAX + 1];
    if (!usb_hid_scanner_take(bc, sizeof(bc))) return;
    Serial.printf("[USB] barcode: \"%s\"\n", bc);
    ui_set_barcode(bc);                     /* LVGL task — safe under the lock */
    sd_logger_eventf("USB scan: %s", bc);
}

bool usb_hid_scanner_connected(void)
{
    return s_usb.deviceCount() > 0;         /* a USB device is enumerated */
}
