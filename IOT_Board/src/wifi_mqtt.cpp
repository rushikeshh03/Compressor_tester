/******************************************************************************************************************************
 * @file    wifi_mqtt.cpp
 * @brief   WiFi-STA link + one-shot SNTP time sync. See wifi_mqtt.h for the contract.
 *
 *  The cloud esp-mqtt uplink that used to live here was retired: the IOT board now
 *  syncs SD logs to a LOCAL Data Server over HTTP+MQTT (das_sync.*). This file keeps
 *  only the WiFi-STA association (3-slot credential rotation, NVS-overridable) and the
 *  one-shot SNTP that anchors the shared software RTC (rtc_record_sync()).
 ******************************************************************************************************************************/

#include "wifi_mqtt.h"

#if CONFIG_WIFI_MQTT_FALLBACK

#include "main.h"          /* DateTime timestampData, ntpTimeReceived, rtc_record_sync() */

#include <WiFi.h>
#include <nvs.h>
#include <time.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *WM_TAG = "WIFI";

static bool    s_sntp_done  = false;

/* ── WiFi-failover state ──────────────────────────────────────────────────────
 *  s_active_idx is the credential slot we currently prefer; wifi_ensure_connected()
 *  starts there and rotates. After WIFI_MQTT_MAX_MQTT_FAIL consecutive sync failures
 *  on the current WiFi, wifi_note_result() advances s_active_idx and drops the link
 *  so we reconnect on the next network. */
static int     s_active_idx = 0;
static uint8_t s_fail_cnt   = 0;

/* ── STA credential list (tried in order) ─────────────────────────────────────
 *  Slot 1 is the PRIMARY: NVS "iot_cfg"/"sta_ssid","sta_pass" (settable from the
 *  factory web page) overriding the compile default CONFIG_STA_WIFI_SSID/PASS.
 *  Slots 2 and 3 are compile-time backups. Empty SSIDs are skipped. */
typedef struct { char ssid[33]; char pass[65]; } sta_cred_t;

static int build_sta_cred_list(sta_cred_t out[3])
{
    int n = 0;

    /* Slot 1 — compile default, overridden by NVS (web-settable). */
    {
        char ssid[33] = {0}, pass[65] = {0};
        strncpy(ssid, CONFIG_STA_WIFI_SSID, sizeof(ssid) - 1);
        strncpy(pass, CONFIG_STA_WIFI_PASS, sizeof(pass) - 1);
        nvs_handle_t h;
        if (nvs_open(CAN_BAUD_NVS_NS, NVS_READONLY, &h) == ESP_OK) {   /* "iot_cfg" */
            size_t s = sizeof(ssid); nvs_get_str(h, "sta_ssid", ssid, &s);
            size_t p = sizeof(pass); nvs_get_str(h, "sta_pass", pass, &p);
            nvs_close(h);
        }
        if (ssid[0]) { strncpy(out[n].ssid, ssid, 32); strncpy(out[n].pass, pass, 64); n++; }
    }

    /* Slots 2 and 3 — compile-time backups. */
    const char *bssid[2] = { CONFIG_STA_WIFI_SSID2, CONFIG_STA_WIFI_SSID3 };
    const char *bpass[2] = { CONFIG_STA_WIFI_PASS2, CONFIG_STA_WIFI_PASS3 };
    for (int i = 0; i < 2 && n < 3; i++) {
        if (bssid[i][0]) {
            strncpy(out[n].ssid, bssid[i], 32); out[n].ssid[32] = '\0';
            strncpy(out[n].pass, bpass[i], 64); out[n].pass[64] = '\0';
            n++;
        }
    }
    return n;
}

/* ── One-shot SNTP after WiFi is up: anchor the shared software RTC so
 *    now_epoch_ms() stops returning 0 on this WiFi-only device. ── */
static void wifi_time_sync_once(void)
{
    if (s_sntp_done) return;

    configTime(0, 0, "pool.ntp.org", "time.google.com");   /* UTC */

    struct tm ti = {};
    if (getLocalTime(&ti, WIFI_MQTT_SNTP_TIMEOUT_MS))
    {
        timestampData.year   = (uint8_t)(((ti.tm_year + 1900) % 100));
        timestampData.month  = (uint8_t)(ti.tm_mon + 1);
        timestampData.day    = (uint8_t)(ti.tm_mday);
        timestampData.hour   = (uint8_t)(ti.tm_hour);
        timestampData.minute = (uint8_t)(ti.tm_min);
        timestampData.second = (uint8_t)(ti.tm_sec);

        rtc_record_sync();          /* anchor rtc_base_epoch_ms + esp_timer base */
        ntpTimeReceived = true;     /* unblocks now_epoch_ms() */
        s_sntp_done = true;
        ESP_LOGI(WM_TAG, "SNTP time set: %04d-%02d-%02d %02d:%02d:%02d UTC",
                 ti.tm_year + 1900, ti.tm_mon + 1, ti.tm_mday,
                 ti.tm_hour, ti.tm_min, ti.tm_sec);
    }
    else
    {
        ESP_LOGW(WM_TAG, "SNTP sync timed out — timestamps stay 0 until a later sync");
    }
}

bool wifi_ensure_connected(void)
{
    if (WiFi.status() == WL_CONNECTED) { wifi_time_sync_once(); return true; }

    sta_cred_t creds[3];
    int n = build_sta_cred_list(creds);
    if (n == 0) { ESP_LOGW(WM_TAG, "no WiFi credentials configured"); return false; }

    if (s_active_idx >= n) s_active_idx = 0;   /* clamp if the list shrank */

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);

    /* Try each network, STARTING at s_active_idx, then rotating until one associates. */
    for (int k = 0; k < n; k++)
    {
        int i = (s_active_idx + k) % n;
        ESP_LOGI(WM_TAG, "connecting STA slot %d/%d to \"%s\" ...", i + 1, n, creds[i].ssid);
        WiFi.disconnect(false);
        WiFi.begin(creds[i].ssid, creds[i].pass);

        TickType_t start = xTaskGetTickCount();
        while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(WIFI_MQTT_CONNECT_TIMEOUT_MS))
        {
            if (WiFi.status() == WL_CONNECTED)
            {
                s_active_idx = i;
                ESP_LOGI(WM_TAG, "STA connected to \"%s\", IP=%s",
                         creds[i].ssid, WiFi.localIP().toString().c_str());
                wifi_time_sync_once();      /* time as soon as we have IP connectivity */
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        ESP_LOGW(WM_TAG, "\"%s\" timed out", creds[i].ssid);
    }

    ESP_LOGW(WM_TAG, "all %d WiFi option(s) failed", n);
    return false;
}

bool wifi_connected(void)
{
    return WiFi.status() == WL_CONNECTED;
}

int16_t wifi_rssi(void)
{
    return (WiFi.status() == WL_CONNECTED) ? (int16_t)WiFi.RSSI() : 0;
}

void wifi_note_result(bool ok)
{
    if (ok) { s_fail_cnt = 0; return; }
    if (++s_fail_cnt < WIFI_MQTT_MAX_MQTT_FAIL) return;
    s_fail_cnt = 0;

    sta_cred_t tmp[3];
    int n = build_sta_cred_list(tmp);
    if (n <= 1) return;                 /* only one network — nothing to rotate to */

    s_active_idx = (s_active_idx + 1) % n;
    ESP_LOGW(WM_TAG, "sync failing — switching to WiFi slot %d", s_active_idx + 1);
    WiFi.disconnect(false);             /* drop association; reconnect on the new slot next time */
}

#else  /* CONFIG_WIFI_MQTT_FALLBACK == 0 — compile to no-ops */

bool    wifi_ensure_connected(void) { return false; }
bool    wifi_connected(void)        { return false; }
int16_t wifi_rssi(void)             { return 0; }
void    wifi_note_result(bool ok)   { (void)ok; }

#endif /* CONFIG_WIFI_MQTT_FALLBACK */
