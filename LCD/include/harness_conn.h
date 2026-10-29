#pragma once
/**
 * harness_conn.h  –  connection manager for the Harness Tester
 *
 * Owns the link state shown on the UI (WiFi / MQTT / QR-scanner Bluetooth /
 * Android-tablet BLE-HID) and handles the page-2 user actions (scan, connect,
 * restart Bluetooth, advertise toggle) plus the page-1 Check round-trip.
 *
 * ⚠ CURRENTLY A SIMULATION / PLACEHOLDER. It fakes device discovery, link-up
 * transitions and the Check PASS/FAIL result so the UI is fully demonstrable.
 * The real BLE (dual-role: central to the scanner + HID peripheral to the
 * tablet), WiFi and MQTT stacks replace the marked sections — the UI contract
 * (the ui_set_ / ui_bt_ / ui_tablet_ setters + ui_take_ pollers) stays the same.
 *
 * Both calls touch LVGL widgets, so invoke them inside lvgl_port_lock/unlock.
 */
#ifdef __cplusplus
extern "C" {
#endif

void harness_conn_init(void);   /* load remembered scanner (NVS), arm state */
void harness_conn_poll(void);   /* service UI requests + timed transitions  */

#ifdef __cplusplus
}
#endif
