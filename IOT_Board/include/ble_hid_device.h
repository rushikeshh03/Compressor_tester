#pragma once
/**
 * ble_hid_device.h  –  BLE HID keyboard *peripheral* (to the Android tablet)
 *
 * The IOT board advertises as a BLE HID keyboard; the tablet pairs with it and the
 * barcode the LCD tester scans (received here over CAN as 0x600/0x601/0x602) is typed
 * into the tablet's focused field as keystrokes — like a scanner wedge.
 *
 * Adapted from the LCD project's ble_hid_device.* but PERIPHERAL-ONLY: the IOT board
 * has no BLE scanner central, so this module owns the radio itself (does BLEDevice::init
 * on its own task) instead of relying on ble_hid_host's ble_hid_init(). Classic Arduino
 * BLE (Bluedroid) API — the same stack the IOT firmware already links.
 */
#include <stdbool.h>

/* Advertised device name the tablet sees (kept distinct from the LCD's "Coulomb Harness").
   Any tablet may connect + pair manually from its Bluetooth settings. */
#ifndef CONFIG_BLE_HID_NAME
#define CONFIG_BLE_HID_NAME   "Coulomb IOT HID"
#endif

typedef enum {
    HIDDEV_DOWN = 0,       /* not advertising, no tablet          */
    HIDDEV_ADVERTISING,    /* advertising, waiting for the tablet */
    HIDDEV_CONNECTED,      /* a tablet is connected               */
} hiddev_state_t;

#ifdef __cplusplus
extern "C" {
#endif

void           ble_hid_dev_init(void);            /* init radio + build HID service (blocks until built) */
void           ble_hid_dev_advertise(bool on);    /* start advertising / stop + disconnect */
void           ble_hid_dev_type(const char *s);   /* type a string (e.g. "1234\n") to the tablet */
hiddev_state_t ble_hid_dev_state(void);
void           ble_hid_restart_stack(void);       /* tear down + re-init the BLE stack (on the HID task) */

#ifdef __cplusplus
}
#endif
