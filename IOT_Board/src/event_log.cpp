/******************************************************************************************************************************
 * @file    event_log.cpp
 * @brief   Lightweight IoT event log on SD (see event_log.h). Best-effort; never crashes/stalls callers.
 ******************************************************************************************************************************/

#include "main.h"
#include "event_log.h"
#include <SPI.h>
#include <SD.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_system.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* Externs from analytics.cpp / main.h for the wall clock. */
extern DateTime timestampData;
extern bool     ntpTimeReceived;
extern bool     gpsTimeReceived;

#define EVLOG_SPI_HZ   (10UL * 1000UL * 1000UL)
#define EVLOG_DIR      "/events"

static bool               s_ready   = false;
static SemaphoreHandle_t  s_mutex   = NULL;
static SPIClass           s_spi(HSPI);          /* only used if WE are the one mounting (factory) */
static const char        *s_part    = "??";

/* All of these are touched only while holding s_mutex, so keeping them static (not on the caller's
   stack) means event_logf() adds almost nothing to the caller stack — important because the FATFS
   open/stat chain is stack-heavy and some callers (CAN task, etc.) have modest stacks. */
static char s_msg[192];
static char s_prefix[40];
static char s_fdate[16];
static char s_path[48];
static char s_line[288];

static const char *level_str(ev_level_t lvl)
{
    switch (lvl) {
        case EV_OK:   return "OK  ";
        case EV_FAIL: return "FAIL";
        case EV_WARN: return "WARN";
        default:      return "INFO";
    }
}

/* Fill s_prefix (timestamp string) and s_fdate (filename date, or "undated" pre-sync). */
static void build_time(void)
{
    if (ntpTimeReceived || gpsTimeReceived)
    {
        rtc_update_timestamp();                     /* advance software RTC to now */
        int yr = 2000 + timestampData.year;
        unsigned ms = (unsigned)(now_epoch_ms() % 1000ull);
        snprintf(s_fdate, sizeof(s_fdate), "%04d%02d%02d", yr, timestampData.month, timestampData.day);
        snprintf(s_prefix, sizeof(s_prefix), "%04d-%02d-%02d %02d:%02d:%02d.%03u",
                 yr, timestampData.month, timestampData.day,
                 timestampData.hour, timestampData.minute, timestampData.second, ms);
    }
    else
    {
        uint32_t up = (uint32_t)millis();
        snprintf(s_fdate, sizeof(s_fdate), "undated");
        snprintf(s_prefix, sizeof(s_prefix), "+%06u.%03us", (unsigned)(up / 1000u), (unsigned)(up % 1000u));
    }
}

void event_log_init(void)
{
    if (s_ready) return;   /* idempotent */

    if (s_mutex == NULL)
    {
        s_mutex = xSemaphoreCreateMutex();
        if (s_mutex == NULL) return;   /* out of memory — stay inert */
    }

    /* Resolve running-partition tag once. */
    const esp_partition_t *r = esp_ota_get_running_partition();
    if (r) {
        if      (r->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0)  s_part = "A0";
        else if (r->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1)  s_part = "A1";
        else if (r->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY) s_part = "FC";
    }

    /* Mount only if nobody has yet (app0: sd_log_init already did; factory: we do it here).
       Serialised against the flush + sync tasks via the shared SD lock (sd_log.h). */
    sd_lock(-1);
    if (SD.cardType() == CARD_NONE)
    {
        s_spi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
        if (!SD.begin(SD_CS, s_spi, EVLOG_SPI_HZ))
        {
            /* No card / mount failed — event logging stays inert, app continues normally. */
            sd_unlock();
            return;
        }
    }

    SD.mkdir(EVLOG_DIR);
    sd_unlock();
    s_ready = true;

    EVLOG_INFO("EVLOG", "event log ready (part=%s)", s_part);
}

bool event_log_ready(void)
{
    return s_ready;
}

void event_logf(ev_level_t lvl, const char *tag, const char *fmt, ...)
{
    if (!s_ready || s_mutex == NULL) return;
    if (tag == NULL) tag = "-";

    /* Bounded wait: never let a stuck SD block a caller task indefinitely — just drop the event. */
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(300)) != pdTRUE) return;

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_msg, sizeof(s_msg), fmt, ap);
    va_end(ap);

    build_time();

    snprintf(s_path, sizeof(s_path), EVLOG_DIR "/%s.log", s_fdate);

    int n = snprintf(s_line, sizeof(s_line), "%s  %s  %s  %-5s  %s\n",
                     s_prefix, level_str(lvl), s_part, tag, s_msg);
    if (n < 0) { xSemaphoreGive(s_mutex); return; }
    if (n > (int)sizeof(s_line)) n = sizeof(s_line);

    if (sd_lock(300))
    {
        File f = SD.open(s_path, FILE_APPEND);
        if (f)
        {
            f.write((const uint8_t *)s_line, (size_t)n);
            f.close();
        }
        sd_unlock();
    }

    xSemaphoreGive(s_mutex);
}
