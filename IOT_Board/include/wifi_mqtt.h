/******************************************************************************************************************************
 * @file    wifi_mqtt.h
 * @brief   WiFi-STA link + one-shot SNTP time sync (the network base layer).
 *
 *  HISTORY: this used to also own an esp-mqtt session that streamed CAN-frame
 *  batches to a CLOUD broker. That cloud uplink was retired — the IOT board now
 *  syncs its SD logs to a LOCAL Data Server (see das_sync.*). What remains here is
 *  only the piece das_sync still needs: bring the WiFi STA link up (3 credential
 *  slots, NVS-overridable) and run a one-shot SNTP that anchors the shared software
 *  RTC (rtc_record_sync() in analytics.cpp) so now_epoch_ms()/timestamps are valid.
 *
 *  Credentials: compile-time defaults below, overridable at runtime from NVS
 *  (namespace "iot_cfg", keys "sta_ssid" / "sta_pass") so the bench network can be
 *  changed without reflashing (the factory web page writes them).
 ******************************************************************************************************************************/
#ifndef WIFI_MQTT_H
#define WIFI_MQTT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ── Master enable. 0 = compile every entry point to a no-op (no-WiFi build). ── */
#ifndef CONFIG_WIFI_MQTT_FALLBACK
#define CONFIG_WIFI_MQTT_FALLBACK   1
#endif

/* ── STA credentials, tried in order (slot 1 → 2 → 3) ─────────────────────────
 *  Slot 1 = PRIMARY: compile default below, overridden at runtime by NVS
 *  ("iot_cfg"/"sta_ssid","sta_pass") which the factory web page sets.
 *  Slots 2 & 3 = compile-time BACKUPS, used only if the earlier ones don't
 *  associate. Leave a backup SSID empty ("") to disable that slot. */
#ifndef CONFIG_STA_WIFI_SSID
#define CONFIG_STA_WIFI_SSID        "Airtel_amey_6365"
#endif
#ifndef CONFIG_STA_WIFI_PASS
#define CONFIG_STA_WIFI_PASS        "Air@83093"
#endif

#ifndef CONFIG_STA_WIFI_SSID2
#define CONFIG_STA_WIFI_SSID2       "Ruch"          /* backup #1 — fill in or leave empty */
#endif
#ifndef CONFIG_STA_WIFI_PASS2
#define CONFIG_STA_WIFI_PASS2       "........"
#endif

#ifndef CONFIG_STA_WIFI_SSID3
#define CONFIG_STA_WIFI_SSID3       "blah"          /* backup #2 — fill in or leave empty */
#endif
#ifndef CONFIG_STA_WIFI_PASS3
#define CONFIG_STA_WIFI_PASS3       "........"
#endif

/* Per-attempt timeouts (ms). Kept short so a failed WiFi attempt doesn't stall the
 * caller for long — pending data stays on SD until a later attempt succeeds. */
#define WIFI_MQTT_CONNECT_TIMEOUT_MS   8000u   /* STA association wait             */
#define WIFI_MQTT_SNTP_TIMEOUT_MS      8000u   /* one-shot SNTP wait after connect */

/* Consecutive sync failures on the current WiFi before rotating to the next
 * credential slot (das_sync reports outcomes via wifi_note_result). */
#define WIFI_MQTT_MAX_MQTT_FAIL        3u

#ifdef __cplusplus
extern "C" {
#endif

/* Bring the WiFi STA link up (idempotent — returns fast when already associated).
 * On the first successful connect it also runs a one-shot SNTP sync and anchors the
 * shared software RTC. Returns true when the STA link is up. */
bool wifi_ensure_connected(void);

/* True while the WiFi STA link is associated (has an IP). */
bool wifi_connected(void);

/* Current STA RSSI in dBm (0 when not connected). */
int16_t wifi_rssi(void);

/* Report a sync outcome so repeated failures on one WiFi rotate to the next
 * credential slot (a single success resets the counter). */
void wifi_note_result(bool ok);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_MQTT_H */
