/******************************************************************************************************************************
 * @file    analytics.cpp
 * @brief   Software RTC + RAM-ring → SD store (the storage half of the pipeline).
 *
 *  HISTORY: this used to serialize the lossless CAN ring into v:2/v:3 JSON batches and
 *  publish them to a CLOUD MQTT broker. That cloud uplink was RETIRED — the IOT board
 *  now syncs its SD logs to a LOCAL Data Server (das_sync.*). What remains here is the
 *  storage half: the software RTC / now_epoch_ms() epoch clock (anchored by the WiFi
 *  SNTP sync in wifi_mqtt.cpp) and flush_ring_to_sd(), which drains the RAM staging ring
 *  into the durable SD NDJSON log — that log is now the source for the daily raw sync.
 *  No network work happens in this file anymore.
 ******************************************************************************************************************************/

#include "main.h"
#include "ota_bootloader.h"
#include <time.h>
#include "esp_timer.h"   // esp_timer_get_time(): IDF-native monotonic µs clock (now_epoch_ms)

/* Time-sync flags. Set true by the WiFi SNTP sync (wifi_mqtt.cpp) once the clock is known;
   now_epoch_ms() returns 0 until then. */
bool    ntpTimeReceived = false;
bool    gpsTimeReceived = false;

/* ─────────────────────────────────────────────────────────────
 * Software RTC  — anchored once (WiFi SNTP), advanced with the monotonic timer
 * ───────────────────────────────────────────────────────────── */
static uint32_t rtc_base_millis       = 0;  // millis() at last successful sync
static uint32_t rtc_base_day_secs     = 0;  // seconds-of-day at that sync
static uint16_t rtc_base_year         = 0;
static uint8_t  rtc_base_month        = 0;
static uint8_t  rtc_base_day_of_month = 0;

/* Millisecond epoch clock for now_epoch_ms(): absolute epoch captured at the last sync,
   advanced by the ESP high-resolution monotonic timer (µs) since then. Guarded by
   epoch_mux because now_epoch_ms() is called from the CAN-Rx task (per frame) while
   rtc_record_sync() writes these from the WiFi/SNTP path. */
static uint64_t     rtc_base_epoch_ms = 0;  // Unix epoch (ms) at last successful sync
static int64_t      rtc_base_esp_us   = 0;  // esp_timer_get_time() (µs) at that sync
static portMUX_TYPE epoch_mux         = portMUX_INITIALIZER_UNLOCKED;

/** Call this immediately after a successful time sync. Non-static: wifi_mqtt.cpp calls it
 *  after its SNTP sync to anchor this epoch clock. */
void rtc_record_sync(void)
{
    rtc_base_millis       = (uint32_t)millis();
    rtc_base_day_secs     = (uint32_t)timestampData.hour   * 3600u
                          + (uint32_t)timestampData.minute * 60u
                          + (uint32_t)timestampData.second;
    rtc_base_year         = 2000u + timestampData.year;
    rtc_base_month        = timestampData.month;
    rtc_base_day_of_month = timestampData.day;

    struct tm t = {};
    t.tm_year = (2000 + timestampData.year) - 1900;
    t.tm_mon  = timestampData.month - 1;
    t.tm_mday = timestampData.day;
    t.tm_hour = timestampData.hour;
    t.tm_min  = timestampData.minute;
    t.tm_sec  = timestampData.second;
    uint64_t base_ms = (uint64_t)mktime(&t) * 1000ull;

    portENTER_CRITICAL(&epoch_mux);
    rtc_base_epoch_ms = base_ms;
    rtc_base_esp_us   = esp_timer_get_time();
    portEXIT_CRITICAL(&epoch_mux);
}

/**
 * @brief Advance timestampData using millis() elapsed since last sync.
 *        Must only be called after at least one successful sync.
 */
void rtc_update_timestamp(void)
{
    if (!ntpTimeReceived && !gpsTimeReceived) return;

    uint32_t elapsed_sec = ((uint32_t)millis() - rtc_base_millis) / 1000u;
    uint32_t current_day_sec = rtc_base_day_secs + elapsed_sec;
    current_day_sec %= 86400u;   /* midnight roll-over */

    timestampData.hour   = (uint8_t)(current_day_sec / 3600u);
    timestampData.minute = (uint8_t)((current_day_sec % 3600u) / 60u);
    timestampData.second = (uint8_t)(current_day_sec % 60u);
    /* year/month/day remain from last sync — good enough for the SD/event-log stamps */
}

/**
 * @brief  Wall-clock epoch (ms) for stamping ring-buffered CAN frames. Millisecond
 *         resolution and STRICTLY MONOTONIC. Returns 0 before the first SNTP sync
 *         (ordering then rests on the ring's seq).
 */
uint64_t now_epoch_ms(void)
{
    if (!ntpTimeReceived && !gpsTimeReceived)
    {
        return 0;
    }

    static uint64_t last_epoch_ms = 0;
    uint64_t epoch_ms;

    portENTER_CRITICAL(&epoch_mux);
    int64_t elapsed_us = esp_timer_get_time() - rtc_base_esp_us;
    if (elapsed_us < 0) elapsed_us = 0;   // guard a sync racing this read
    epoch_ms = rtc_base_epoch_ms + (uint64_t)(elapsed_us / 1000);

    if (epoch_ms <= last_epoch_ms)
    {
        epoch_ms = last_epoch_ms + 1;     // strict-monotonic guard
    }
    last_epoch_ms = epoch_ms;
    portEXIT_CRITICAL(&epoch_mux);

    return epoch_ms;
}

/**
 *  @brief  RAM ring → SD durable NDJSON log (the storage half; no network)
 */

/* Frames per flush scratch batch (file-static — too big for the task stack). */
#ifndef SD_FLUSH_BATCH_MAX
#define SD_FLUSH_BATCH_MAX 80u
#endif
#define SD_FLUSH_PERIOD_MS 1000u

static CanFrame_t s_frameScratch[SD_FLUSH_BATCH_MAX];

/* Drain the RAM staging ring fully into the SD durable log (NDJSON). Frames leave the ring
   only after they've been appended to SD, so a write failure keeps them buffered. */
static size_t flush_ring_to_sd(void)
{
    size_t total = 0;
    for (;;)
    {
        CanFrame_t *frames = s_frameScratch;
        uint64_t   firstSeq = 0;
        size_t n = fb_peek(frames, SD_FLUSH_BATCH_MAX, &firstSeq);
        if (n == 0) break;

        if (!sd_log_append_frames(frames, n))
        {
            ESP_LOGE("SDLOG", "append failed — %u frames stay in RAM ring", (unsigned)n);
            break;
        }
        fb_commit(firstSeq + n);
        total += n;
    }
    return total;
}

/**
 * @brief  Storage cycle — call periodically from sd_flush_task().
 *
 *  Drains the RAM staging ring into the durable SD NDJSON log (the source the local-server
 *  daily raw sync reads). Purely local: no WiFi/MQTT/HTTP work happens here. When the SD
 *  card is absent the frames simply stay in the RAM ring (there is no cloud fallback now).
 */
void sd_flush_service(void)
{
    static TickType_t lastCycleTick = 0;
    TickType_t now = xTaskGetTickCount();

    if ((now - lastCycleTick) < pdMS_TO_TICKS(SD_FLUSH_PERIOD_MS)) return;
    lastCycleTick = now;

    g_scan_ready = false;   /* consumed: the per-result CSV is built on the CAN path (can.cpp) */

    if (g_sdReady) flush_ring_to_sd();
}
