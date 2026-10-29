/**
 * ui.h  –  Harness Tester UI  (LVGL, 800x480, 7" Waveshare ESP32-S3)
 *
 * Two pages (BLE Connections page removed — the scanner is USB-HID now):
 *
 * Page 1 — Main (swipe LEFT → Cell Data):
 *   Top bar : title + status chips (CAN, IoT, Scanner[USB], Tablet, Uplink, SD) —
 *             green up, amber busy, grey/blinking-red down. Version watermark
 *             bottom-right.
 *   LEFT    : Coulomb logo + a slot that is a [Restart system] button while idle
 *             ("Checking…" while a check runs).
 *   RIGHT   : scanned barcode (fills the card) on top, big PASS / FAIL below.
 *
 * Page 2 — Cell Data (swipe RIGHT → Main): 8 cell voltages + 4 temps + harness
 *   fault line, plus the master-echoed barcode on the title bar.
 *
 * This header owns only the widgets. The barcode/CAN backends push state in via
 * the ui_set_ setters and pull user intent out via the ui_take_* pollers.
 */
#pragma once
#include "lvgl.h"
#include "ba100_can.h"   /* BA100_State_t */

/* Big PASS/FAIL indicator (page 1). */
typedef enum {
    UI_RESULT_NONE = 0,   /* neutral / waiting → shows "-" */
    UI_RESULT_PASS,
    UI_RESULT_FAIL,
    UI_RESULT_MISMATCH,   /* scanned barcode ≠ master echo */
} ui_result_t;

/* Generic connection status (status chips + page-2 pills). */
typedef enum {
    UI_CONN_DOWN = 0,     /* grey  */
    UI_CONN_BUSY,         /* amber (connecting / scanning / advertising) */
    UI_CONN_UP,           /* green */
} ui_conn_t;

/* Top-bar status chips. */
typedef enum {
    UI_LINK_CAN = 0,
    UI_LINK_UART,         /* UART link to the CAN bridge (../CAN_Bridge) is up */
    UI_LINK_IOT,          /* IoT board alive (0x7A0 heartbeat) — blinks when up */
    UI_LINK_MQTT,         /* MQTT/WiFi up, from the IoT heartbeat */
    UI_LINK_SCANNER,      /* USB-HID barcode scanner (enumerated) */
    UI_LINK_TABLET,       /* tablet connected, from the IoT heartbeat */
    UI_LINK_SD,           /* micro-SD card present */
    UI_LINK_COUNT,
} ui_link_t;

#ifdef __cplusplus
extern "C" {
#endif

/** Build both screens and load the main screen. Call once inside lvgl_port_lock. */
void ui_init(void);

/* ── Page 1: barcode / result (call inside lvgl_port_lock) ───────────────── */
void ui_set_barcode(const char *code);       /* scanned code (NULL/"" → "-")   */
void ui_set_result(ui_result_t result);      /* legacy PASS/FAIL panel (no-op now) */
void ui_set_checking(bool checking);         /* left-column status: "Checking…" */
void ui_set_rx_barcode(const char *code);    /* master-echoed code (page-3 bar) */

/* Live raw-CAN log panel (main page, RIGHT-BOTTOM). Pre-formatted multi-line
 * text (see rawlog_render). NULL/"" clears to a placeholder. */
void ui_set_rawlog(const char *text);

/* Transient popup on the MAIN page: "Please connect <missing>, then scan again."
 * Auto-hides after 5 s. Shown when a scan can't be forwarded because a required
 * device (SD / scanner / tablet) is not connected. */
void ui_show_connect_popup(const char *missing);

/* ── FAIL lock-out popup (MAIN page only) ────────────────────────────────────
 *  Shown after a harness FAIL. It lives on the main screen and lets swipe
 *  gestures bubble through, so the user can still switch pages while it's up.
 *    ui_fail_popup_fail()   — "HARNESS FAILED — rescan <barcode>"  [Close]
 *    ui_fail_popup_rescan() — "Wrong harness — rescan <barcode>"   [Override][Close]
 *  The connection layer polls the buttons via ui_take_fail_close / _override and
 *  drives the Sensor-AD pin from the popup's visibility. */
void ui_fail_popup_fail(const char *barcode);
void ui_fail_popup_rescan(const char *barcode);
void ui_fail_popup_hide(void);
bool ui_take_fail_close(void);      /* Close pressed */
bool ui_take_fail_override(void);   /* Override pressed */

/* ── Top-bar status chips ────────────────────────────────────────────────── */
void ui_set_link(ui_link_t link, ui_conn_t status);

/* ── Page 1 LEFT (idle slot) ─────────────────────────────────────────────────
 *  The idle status slot below the logo is a "Restart system" button (shown only
 *  while idle — hidden during a check). A tap asks the connection layer to
 *  restart the IoT board (over CAN), the CAN bridge board, and then the LCD. */
bool        ui_take_restart_system_request(void);         /* user pressed Restart system */

#ifdef __cplusplus
}
#endif

/**
 * Push a fresh BA100_State_t snapshot (drives the CAN chip + status watermark).
 * Called ~25 Hz from main.cpp inside lvgl_port_lock/unlock.
 */
void ui_apply_state(const BA100_State_t &state);
