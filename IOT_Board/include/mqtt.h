/******************************************************************************************************************************
 * @file    mqtt.h
 * @brief   Factory-mode WiFi service-updater configuration.
 *
 * ⚠ HISTORY: this header used to hold the CLOUD MQTT broker config (server/port/credentials/
 *   topics) and the batched-uplink tunables. That cloud uplink was RETIRED (2026-09-26) — the
 *   board now syncs its SD logs to a LOCAL Data Server; see das_sync.* (server address/token
 *   live in das_sync.h / NVS). Those cloud defines were removed.
 *
 *   The only remaining consumer of a broker on the app path is the compiled-out EC200 modem
 *   OTA code in ota_bootloader.cpp (guarded by `#define OTA_BL_MODEM_OTA 0`). It is not built,
 *   so it needs no broker macros. If that modem-OTA path is ever revived, restore the
 *   MQTT_SERVER / MQTT_PORT / MQTT_CLIENT_ID / MQTT_USERNAME / MQTT_PASSWORD / MQTT_TOPIC_*
 *   defines it references.
 *
 *   The factory-WiFi block below is still live (used by wifi_service.cpp; pulled in via
 *   ota_bootloader.h → main.h). Kept here for include-compatibility.
 ******************************************************************************************************************************/

#ifndef MQTT_H
#define MQTT_H

/* ════════════════════════════════════════════════════════════════════════════
 * FACTORY-MODE WiFi SERVICE UI + UPDATER
 *
 * Raised ONLY while running from the factory partition (OTA bootloader). A hidden
 * WPA2 SoftAP + a small password-gated web page (see wifi_service.cpp) to show the
 * app0→app1 backup status, upload a firmware .bin to app0, change CAN baud, and
 * reboot. Torn down on the reboot that exits factory mode; normal app mode never
 * starts WiFi via this path. This is a bench/service tool — simple cookie auth, plain HTTP.
 * ════════════════════════════════════════════════════════════════════════════ */
#define CONFIG_FACTORY_WIFI             1              /* 1 = enable, 0 = compile to no-ops        */
#define CONFIG_FACTORY_WIFI_SSID        "ESP32IOT"     /* hidden AP SSID (distinct from the LCD's) */
#define CONFIG_FACTORY_WIFI_PASS        "12345678"     /* WPA2 pass (>= 8 chars)                   */
#define CONFIG_FACTORY_WIFI_HIDDEN      1              /* 1 = do not broadcast the SSID            */
#define CONFIG_FACTORY_WIFI_CHANNEL     1
#define CONFIG_FACTORY_WEB_USER         "admin"        /* web login username — change for the field */
#define CONFIG_FACTORY_WEB_PASS         "admin"        /* web login password — change for the field */

#endif  /* MQTT_H */
