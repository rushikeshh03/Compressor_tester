/**
 * harness_conn.cpp  --  station controller (raw-CAN logging build)
 *
 * This is a manual HVAC wiring/interfacing station, NOT a PASS/FAIL tester.
 * Flow per the requirement:
 *   1. Operator scans a barcode (USB-HID scanner, sole input).
 *   2. The barcode is shown on the LCD and broadcast on CAN (0x600/601/602) so
 *      the CAN_Bridge and IoT board can segment their logs by it.
 *   3. Every raw CAN frame (RX + our TX) is logged — on the LCD live view
 *      (rawlog) and to a per-scan SD file — until the NEXT barcode is scanned.
 *   Between scans the operator manually drives the compressor/charger and their
 *   status frames just flow into the log (control UI added separately).
 *
 * There is no PASS/FAIL, no DAS round-trip, no FAIL lock-out. Called from loop()
 * inside lvgl_port_lock (it drives ui_* widgets). The USB host runs on its own
 * task; this only drains completed barcodes and pushes to the UI.
 */

#include <Arduino.h>          /* must precede lvgl.h (pulled in by ui/ui.h) */
#include <esp_system.h>       /* esp_restart() for the "Restart system" button */
#include <string.h>
#include "harness_conn.h"
#include "ui/ui.h"
#include "ba100_can.h"        /* barcode TX + state snapshot                */
#include "scanner.h"          /* USB scanner selector (CDC-ACM or HID)      */
#include "sd_logger.h"        /* per-scan raw-CAN session files + events    */
#include "rawlog.h"           /* on-screen live raw-CAN log                 */

/* ══════════════════════════════════════════════════════════════════════════ */
void harness_conn_init(void)
{
    /* Sensor-AD line (GPIO6) kept as an idle-LOW output (no FAIL lock-out on
     * this station; retained so the header pin is defined, not floating). */
    pinMode(CONFIG_SENSOR_AD_PIN, OUTPUT);
    digitalWrite(CONFIG_SENSOR_AD_PIN, LOW);

    /* USB host is brought up in main.cpp (usb_hid_scanner_init) before this. */
    sd_logger_event("device booted");
}

/* A new scan: show it, close the previous raw-CAN session + open a new one,
 * clear the on-screen log, and broadcast the barcode on CAN as the session
 * marker for the bridge + IoT board. */
static void process_scan(const char *bc)
{
    ui_set_barcode(bc);

    /* Open a fresh per-scan raw-CAN SD file (closes the previous one, which then
     * holds every frame up to this new barcode). Must precede the barcode TX so
     * that TX lands in the NEW file. */
    sd_logger_session_open(bc);

    /* Clear the LCD live log so it shows only this session's traffic. */
    rawlog_reset();

    /* Broadcast the barcode on CAN (0x600/601/602) → bridge → bus → IoT. */
    ba100_send_barcode(bc);

    sd_logger_eventf("scan: %s", bc);
    Serial.printf("[SCAN] %s\n", bc);
}

/* Drain one completed barcode from the USB-HID scanner, if any. */
static void poll_scanner(void)
{
    char bc[128];
    if (!scanner_take(bc, sizeof(bc))) return;   /* one completed code per call */
    process_scan(bc);
}

/* ══════════════════════════════════════════════════════════════════════════ */
void harness_conn_poll(void)
{
    poll_scanner();

    /* USB-HID scanner presence → Scanner chip (green enumerated, else red). */
    static int s_last_usb = -1;
    int usb_up = scanner_connected() ? 1 : 0;
    if (usb_up != s_last_usb) {
        ui_set_link(UI_LINK_SCANNER, usb_up ? UI_CONN_UP : UI_CONN_DOWN);
        if (s_last_usb != -1)
            sd_logger_event(usb_up ? "USB scanner connected" : "USB scanner disconnected");
        s_last_usb = usb_up;
    }

    BA100_State_t cst = ba100_get_state();

    /* CAN link up/down (event log). */
    static int s_last_can_alive = -1;
    int can_alive = cst.can_alive ? 1 : 0;
    if (can_alive != s_last_can_alive) {
        if (s_last_can_alive != -1)
            sd_logger_event(can_alive ? "CAN link up" : "CAN link down");
        s_last_can_alive = can_alive;
    }

    /* IoT board heartbeat (0x7A0): Tablet/MQTT/IoT chips + transitions. */
    ui_set_link(UI_LINK_IOT,    cst.iot_alive  ? UI_CONN_UP : UI_CONN_DOWN);
    ui_set_link(UI_LINK_TABLET, cst.iot_tablet ? UI_CONN_UP : UI_CONN_DOWN);
    ui_set_link(UI_LINK_MQTT,   cst.iot_mqtt   ? UI_CONN_UP : UI_CONN_DOWN);
    static int s_last_iot_alive = -1, s_last_iot_tab = -1;
    if ((int)cst.iot_alive != s_last_iot_alive) {
        if (s_last_iot_alive != -1) sd_logger_event(cst.iot_alive ? "IoT online" : "IoT offline");
        s_last_iot_alive = cst.iot_alive;
    }
    if ((int)cst.iot_tablet != s_last_iot_tab) {
        if (s_last_iot_tab != -1) sd_logger_event(cst.iot_tablet ? "tablet connected (IoT)" : "tablet disconnected (IoT)");
        s_last_iot_tab = cst.iot_tablet;
    }
}
