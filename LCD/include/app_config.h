#pragma once
/**
 * app_config.h  –  central configuration for the Coulomb Harness Tester
 *
 * Hardware is fixed: Waveshare ESP32-S3-Touch-LCD-7 (800x480 ST7262 RGB +
 * GT911 touch). The 4.3" board and all charger/BMS options were removed in the
 * fork from ESP32LCD_Charger — this file now only carries what the harness
 * tester actually uses.
 *
 * ── Architecture (2026-09-10): USB scanner + CAN via UART BRIDGE, no BLE ────
 * Barcode input is a USB-HID barcode scanner on the ESP32-S3 native USB
 * (GPIO19/20 → USB-C port; USB_SEL held LOW).
 *
 * CAN is NOT on this board. The LCD talks a tiny framed protocol over UART to a
 * separate **ESP32-WROOM CAN bridge** (../CAN_Bridge) that has native TWAI + a
 * CAN transceiver; the bridge relays frames both ways transparently. Both boards
 * are 3.3 V, so the UART link needs no level shifting. Wire:
 *     LCD GPIO43 (TX) ─────────────→ bridge GPIO16 (RX)
 *     LCD GPIO44 (RX) ←───────────── bridge GPIO17 (TX)
 *     GND ────────────────────────── GND
 * ba100_can.* keeps all the DAS parsing; only its transport changed (TWAI → UART).
 *
 * ⚠ GPIO43/44 are the LCD's UART0 console pins, so the LCD's USB-serial console
 * is gone (logs → SD). The bridge's own USB gives a CAN debug console instead.
 *
 * BLE was REMOVED (ble_hid_host.* deleted). USB-CDC stays OFF
 * (ARDUINO_USB_CDC_ON_BOOT=0) — the native USB is a HOST for the scanner, not a
 * CDC device. The tablet is on the IoT board; the LCD learns "tablet connected"
 * from the IoT 0x7A0 CAN heartbeat (relayed by the bridge). App WiFi is removed.
 */
/* ════════════════════════════════════════════════════════════════════════
 *  UART link to the CAN bridge (../CAN_Bridge), and the CAN bus baud
 * ════════════════════════════════════════════════════════════════════════ */
/* LCD UART to the bridge, on the UART2 header P1 (DIP switch → UART2). The names
 * keep CONFIG_CAN_* for source compatibility (can_bootloader.* still references
 * them), but they are UART pins now, not TWAI pins. */
#define CONFIG_CAN_TX_PIN          43   /* LCD UART TX → bridge RX (GPIO16) */
#define CONFIG_CAN_RX_PIN          44   /* LCD UART RX ← bridge TX (GPIO17) */

/* UART bit rate LCD↔bridge. Must match CONFIG_BRIDGE_UART_BAUD in the bridge
 * firmware. 1 Mbps is comfortable over short jumpers between two 3.3 V ESP32s. */
#define CONFIG_BRIDGE_UART_BAUD    1000000UL

/* CAN bus bit rate the BRIDGE runs its TWAI at — shown in the LCD watermark for
 * reference. Change it in the bridge firmware too (they must match). 125/250/500. */
#define CONFIG_CAN_BAUD_KBPS       500

#if CONFIG_CAN_BAUD_KBPS != 125 && CONFIG_CAN_BAUD_KBPS != 250 && CONFIG_CAN_BAUD_KBPS != 500
#error "CONFIG_CAN_BAUD_KBPS must be 125, 250, or 500 kbps"
#endif

/* ════════════════════════════════════════════════════════════════════════
 *  Barcode input: USB scanner on the native USB host (USB_SEL LOW)
 *
 *  Two drivers are compiled in; CONFIG_SCANNER_USB_CDC selects which one runs
 *  (via scanner.h — callers use scanner_init/take/connected):
 *    0 = USB-HID keyboard  (usb_hid_scanner.*, EspUsbHost)  — the original mode.
 *    1 = USB CDC-ACM serial (usb_cdc_scanner.*, raw usb_host) — for a scanner
 *        that presents a USB virtual COM port instead of a HID keyboard.
 *  To ROLL BACK to the HID scanner, set this to 0 (one line).
 *
 *  ⚠ CDC-ACM only works if the scanner is a true USB Communications-class device.
 *  Scanners using a USB-UART bridge chip (CP210x/FTDI/CH340/PL2303) are NOT
 *  CDC-ACM and will not enumerate here.
 *
 *  ⚠ HARDWARE: USB host must supply 5 V VBUS to the scanner — confirm the board's
 *  USB-C sources VBUS, or power the scanner externally.
 * ════════════════════════════════════════════════════════════════════════ */
#define CONFIG_SCANNER_USB_CDC     1    /* 1 = CDC-ACM serial scanner, 0 = HID keyboard */

/* ════════════════════════════════════════════════════════════════════════
 *  Sensor-AD header (P2: 3.3V / GND / AD) — GPIO6, used as a digital FAIL output
 *
 *  Driven HIGH while a harness FAIL is unresolved (fail lock-out), LOW otherwise.
 *  GPIO6 is a free ESP32-S3 GPIO (ADC1_CH5) on the "Sensor AD" header; it is a
 *  3.3 V logic line (~20 mA) — use a MOSFET/relay driver for anything bigger. */
#define CONFIG_SENSOR_AD_PIN       6

/* ════════════════════════════════════════════════════════════════════════
 *  Firmware / UI
 * ════════════════════════════════════════════════════════════════════════ */
#define CONFIG_FW_VERSION          "HT v0.1.0"   /* bottom-right status watermark */

/* App-mode WiFi + NTP were REMOVED entirely: app WiFi + dual-role NimBLE don't
 * both fit in internal RAM (see CHANGELOG 2026-08-13), and the PSRAM-offload fix
 * isn't viable in this toolchain. The barcode scanner + tablet are BLE; the only
 * WiFi left is the factory-mode SoftAP web updater below (CAN-flash recovery,
 * never runs in the app). SD-log timestamps use T+uptime (no NTP wall-clock). */

/* ════════════════════════════════════════════════════════════════════════
 *  SD card requirement
 *    1 = required: a barcode is NOT forwarded (tablet/master) unless an SD card
 *        is present (it's part of the SD+scanner+tablet gate).
 *    0 = optional: SD never blocks forwarding — ignored when absent, but still
 *        used for timestamped scan logging when present.
 * ════════════════════════════════════════════════════════════════════════ */
#define CONFIG_SD_REQUIRED         0

/* ════════════════════════════════════════════════════════════════════════
 *  Factory-mode WiFi service  (service + web firmware updater)
 *
 *  Active only while running from the factory partition (CAN-flash mode). A
 *  hidden SoftAP lets a technician open the web page to see the app0→app1
 *  backup result, upload a firmware .bin (flashed into app0 via the OTA path),
 *  change the CAN baud, and reboot. Torn down when factory mode exits. Normal
 *  app mode NEVER starts WiFi.
 * ════════════════════════════════════════════════════════════════════════ */
#define CONFIG_FACTORY_WIFI          1            /* 1 = enable factory WiFi service, 0 = disable */
#define CONFIG_FACTORY_WIFI_SSID     "ESP32LCD"   /* hidden AP SSID */
#define CONFIG_FACTORY_WIFI_PASS     "12345678"   /* WPA2 pass (>=8 chars) */
#define CONFIG_FACTORY_WIFI_HIDDEN   1            /* 1 = do not broadcast SSID */
#define CONFIG_FACTORY_WIFI_CHANNEL  1
#define CONFIG_FACTORY_WEB_USER      "admin"      /* web login username */
#define CONFIG_FACTORY_WEB_PASS      "admin"      /* web login password */
