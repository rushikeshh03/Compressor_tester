/**
 * sd_logger.cpp  –  Hot-pluggable micro-SD logger, TWO daily files (see .h)
 *
 *   /data_<tag>.csv   — CSV, one row per scan/test (sd_logger_data)
 *   /events_<tag>.log — LCD/system operations (sd_logger_event)
 *
 * <tag> is the current date ("YYYYMMDD") once the CAN time-sync frame has set
 * the clock, else "nodate". The task re-checks the tag and rolls both files when
 * it changes (time sync, or midnight).
 *
 * Isolation: producers only xQueueSend (timeout 0) — never open/write/probe the
 * card — so a slow/absent card can't stall the display or CAN tasks. All card
 * work runs in sd_log_task at low priority on core 0, failure-tolerant.
 */

#include "sd_logger.h"
#include "ba100_can.h"          /* ba100_timestamp() / ba100_get_datetime() */

#include <Arduino.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_io_expander.hpp>  /* esp_expander::Base */
#include "FS.h"
#include "SD.h"
#include "SPI.h"

/* ── Board SD wiring (per the Waveshare SD demo) ─────────────────────────── */
#define SD_PIN_MOSI          11
#define SD_PIN_CLK           12
#define SD_PIN_MISO          13
#define SD_CS_NONE           (-1)   /* no GPIO CS — expander holds it low */
#define SD_CS_EXPANDER_PIN   4      /* CH422G pin index for SD chip-select */
#define SD_SPI_HZ            4000000UL

/* CSV header for the data file (must match the field order harness_conn emits).
 * The caller supplies the full row (incl. scan_time/resp_time) — the logger does
 * NOT prepend a timestamp for data rows. */
#define DATA_CSV_HEADER \
    "scan_time,resp_time,latency_ms,scanned,received,result,status," \
    "c1_mV,c2_mV,c3_mV,c4_mV,c5_mV,c6_mV,c7_mV,c8_mV,t1,t2,t3,t4," \
    "cell_fault_mask,temp_fault_mask,init_failed," \
    "cell_raw_fault_mask,temp_repeat_fault_mask,passing_reading_count"

#define BC_DIR               "/barcodes"   /* per-scan raw-CAN session files */

/* ── Log queue (one queue, a channel byte selects the file) ──────────────── */
#define LOG_LINE_MAX   224   /* timestamp + widest CSV data row, with margin */
#define LOG_QUEUE_LEN  64
#define CHAN_EVENT     0
#define CHAN_DATA      1
#define CHAN_FRAME     2     /* raw CAN row → open session file            */
#define CHAN_SESS_OPEN 3     /* text = barcode: close prev, open new file  */
#define CHAN_SESS_CLOSE 4

typedef struct { uint8_t chan; char text[LOG_LINE_MAX]; } LogLine_t;

static QueueHandle_t   s_queue    = nullptr;
static TaskHandle_t    s_task     = nullptr;
static esp_expander::Base *s_expander = nullptr;
static volatile bool   s_present  = false;

/* ── Public API ──────────────────────────────────────────────────────────── */
bool sd_logger_present(void) { return s_present; }

static void enqueue(uint8_t chan, const char *text)
{
    if (s_queue == nullptr) return;
    LogLine_t line;
    line.chan = chan;
    if (chan == CHAN_EVENT) {
        char ts[28];
        ba100_timestamp(ts, sizeof(ts));             /* wall time or T+uptime */
        snprintf(line.text, sizeof(line.text), "%s  %s", ts, text ? text : "");
    } else {
        /* DATA / FRAME rows are self-contained; SESS_OPEN carries the barcode. */
        strncpy(line.text, text ? text : "", sizeof(line.text) - 1);
        line.text[sizeof(line.text) - 1] = '\0';
    }
    xQueueSend(s_queue, &line, 0);                   /* non-blocking: drop if full */
}

void sd_logger_event(const char *msg) { enqueue(CHAN_EVENT, msg); }
void sd_logger_data (const char *csv) { enqueue(CHAN_DATA,  csv); }

/* ── Raw-CAN per-scan capture ────────────────────────────────────────────── */
void sd_logger_session_open(const char *barcode) { enqueue(CHAN_SESS_OPEN, barcode ? barcode : "blank"); }
void sd_logger_session_close(void)               { enqueue(CHAN_SESS_CLOSE, ""); }

void sd_logger_frame(bool tx, uint32_t id, const uint8_t *data, uint8_t dlc, const char *decoded)
{
    if (s_queue == nullptr || (!data && dlc)) return;
    if (dlc > 8) dlc = 8;
    char ts[40]; ba100_timestamp_ms(ts, sizeof(ts));      /* HH:MM:SS.mmm or T+.. */
    char row[LOG_LINE_MAX];
    int p;
    if (id > 0x7FF) p = snprintf(row, sizeof(row), "%s,%s,0x%08lX,%u,",
                                 ts, tx ? "TX" : "RX", (unsigned long)id, dlc);
    else            p = snprintf(row, sizeof(row), "%s,%s,0x%03lX,%u,",
                                 ts, tx ? "TX" : "RX", (unsigned long)id, dlc);
    for (uint8_t i = 0; i < dlc && p < (int)sizeof(row) - 4; i++)
        p += snprintf(row + p, sizeof(row) - p, "%s%02X", i ? " " : "", data[i]);
    /* Decoded column (usually empty). Replace commas so the CSV stays valid. */
    if (p < (int)sizeof(row) - 2) row[p++] = ',';
    for (const char *c = decoded; c && *c && p < (int)sizeof(row) - 1; c++)
        row[p++] = (*c == ',') ? ';' : *c;
    row[p] = '\0';
    enqueue(CHAN_FRAME, row);
}

void sd_logger_eventf(const char *fmt, ...)
{
    if (s_queue == nullptr) return;
    char buf[LOG_LINE_MAX];
    va_list ap; va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    sd_logger_event(buf);
}

/* ── SD task ─────────────────────────────────────────────────────────────── */

/* Day tag for the file names: dated once the clock is set, else "nodate". */
static void current_tag(char *tag, size_t n)
{
    int Y, Mo, D, h, m, s;
    if (ba100_get_datetime(&Y, &Mo, &D, &h, &m, &s)) snprintf(tag, n, "%04d%02d%02d", Y, Mo, D);
    else                                             snprintf(tag, n, "nodate");
}

/* Filename stamp for a session file: date+time once the clock is set, else uptime. */
static void stamp_file(char *out, size_t n)
{
    int Y, Mo, D, h, m, s;
    if (ba100_get_datetime(&Y, &Mo, &D, &h, &m, &s))
        snprintf(out, n, "%04d%02d%02d_%02d%02d%02d", Y, Mo, D, h, m, s);
    else
        snprintf(out, n, "up%lu", (unsigned long)millis());
}

static bool sd_try_mount(void)
{
    /* SD SPI power-up handshake: pulse CS high + ~80 clocks before SD.begin(). */
    if (s_expander) {
        s_expander->digitalWrite(SD_CS_EXPANDER_PIN, HIGH);
        SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));
        for (int i = 0; i < 10; i++) SPI.transfer(0xFF);
        SPI.endTransaction();
        s_expander->digitalWrite(SD_CS_EXPANDER_PIN, LOW);
    }
    if (!SD.begin(SD_CS_NONE, SPI, SD_SPI_HZ)) {
        Serial.println("[SD] SD.begin() failed (no card / wiring / CS?)");
        return false;
    }
    if (SD.cardType() == CARD_NONE) {
        Serial.println("[SD] SD.begin() ok but cardType=NONE");
        SD.end();
        return false;
    }
    Serial.printf("[SD] card OK, type=%d, size=%lluMB\n",
                  (int)SD.cardType(), (unsigned long long)(SD.cardSize() >> 20));
    SD.mkdir(BC_DIR);                 /* per-scan raw-CAN session files live here */
    return true;
}

/* Open (append) both daily files for `tag`; write the CSV header only when the
 * data file is newly created, and an "opened" marker to the event file. */
static bool open_files(const char *tag, File &dataf, File &eventf)
{
    char dn[40], en[40];
    snprintf(dn, sizeof(dn), "/data_%s.csv",   tag);
    snprintf(en, sizeof(en), "/events_%s.log", tag);

    bool data_is_new = !SD.exists(dn);
    dataf  = SD.open(dn, FILE_APPEND);
    eventf = SD.open(en, FILE_APPEND);
    if (!dataf || !eventf) {
        if (dataf)  dataf.close();
        if (eventf) eventf.close();
        return false;
    }
    if (data_is_new) { dataf.print(DATA_CSV_HEADER); dataf.print('\n'); }

    char ts[28]; ba100_timestamp(ts, sizeof(ts));
    eventf.printf("==== events opened (%s) ====\n", ts);
    dataf.flush(); eventf.flush();
    Serial.printf("[SD] logging → %s , %s\n", dn, en);
    return true;
}

static void sd_log_task(void *arg)
{
    (void)arg;
    if (s_expander) {
        s_expander->pinMode(SD_CS_EXPANDER_PIN, OUTPUT);
        s_expander->digitalWrite(SD_CS_EXPANDER_PIN, LOW);
    }
    SPI.setHwCs(false);
    SPI.begin(SD_PIN_CLK, SD_PIN_MISO, SD_PIN_MOSI, SD_CS_NONE);

    File     dataf, eventf, s_bc;   /* s_bc = current per-scan raw-CAN session file */
    char     cur_tag[12]     = {0};
    bool     mounted         = false;
    uint32_t next_mount_try  = 0;
    uint32_t last_flush      = 0;
    LogLine_t line;

    for (;;) {
        uint32_t now = millis();

        if (!mounted) {
            s_present = false;
            if ((int32_t)(now - next_mount_try) >= 0) {
                next_mount_try = now + 1000;
                if (sd_try_mount()) {
                    current_tag(cur_tag, sizeof(cur_tag));
                    if (open_files(cur_tag, dataf, eventf)) { mounted = true; s_present = true; }
                    else                                    { SD.end(); }
                }
            }
            while (xQueueReceive(s_queue, &line, 0) == pdTRUE) { /* discard while no card */ }
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        /* Roll both files if the day tag changed (clock got set, or midnight). */
        char tag[12]; current_tag(tag, sizeof(tag));
        if (strcmp(tag, cur_tag) != 0) {
            dataf.close(); eventf.close();
            if (!open_files(tag, dataf, eventf)) {
                SD.end(); mounted = false; s_present = false;
                Serial.println("[SD] file roll failed — remounting");
                continue;
            }
            strncpy(cur_tag, tag, sizeof(cur_tag) - 1); cur_tag[sizeof(cur_tag) - 1] = '\0';
        }

        /* Write a queued line to its file. */
        if (xQueueReceive(s_queue, &line, pdMS_TO_TICKS(200)) == pdTRUE) {
            if (line.chan == CHAN_SESS_OPEN) {
                /* New scan → close the previous session (all its frames) + open a
                 * new per-scan file named for the barcode. */
                if (s_bc) s_bc.close();
                char safe[28]; int j = 0;
                for (const char *c = line.text; *c && j < (int)sizeof(safe) - 1; c++)
                    safe[j++] = (isalnum((int)(unsigned char)*c) || *c == '.' ||
                                 *c == '-' || *c == '_') ? *c : '_';
                safe[j] = '\0';
                char st[20]; stamp_file(st, sizeof(st));
                char path[64];
                snprintf(path, sizeof(path), BC_DIR "/%s_%s.csv", safe[0] ? safe : "blank", st);
                bool need_hdr = !SD.exists(path);
                s_bc = SD.open(path, FILE_APPEND);
                if (s_bc && need_hdr) s_bc.println("Time,Dir,ID,DLC,Data,Decoded");
                if (s_bc) Serial.printf("[SD] session → %s\n", path);
            } else if (line.chan == CHAN_SESS_CLOSE) {
                if (s_bc) s_bc.close();
            } else if (line.chan == CHAN_FRAME) {
                if (s_bc) { s_bc.print(line.text); s_bc.print('\n'); }
            } else {
                File &f = (line.chan == CHAN_DATA) ? dataf : eventf;
                size_t n = f.print(line.text);
                f.print('\n');
                if (n == 0) {                 /* write failed → card likely removed */
                    dataf.close(); eventf.close(); if (s_bc) s_bc.close(); SD.end();
                    mounted = false; s_present = false;
                    Serial.println("[SD] write failed — card removed? logging stopped");
                    continue;
                }
            }
        }

        /* Periodic flush + presence re-check (~1 s). */
        if ((uint32_t)(now - last_flush) >= 1000) {
            last_flush = now;
            dataf.flush(); eventf.flush(); if (s_bc) s_bc.flush();
            if (SD.cardType() == CARD_NONE) {
                dataf.close(); eventf.close(); if (s_bc) s_bc.close(); SD.end();
                mounted = false; s_present = false;
                Serial.println("[SD] card no longer present — logging stopped");
            }
        }
    }
}

void sd_logger_init(esp_expander::Base *expander)
{
    if (s_task != nullptr) return;
    s_expander = expander;

    s_queue = xQueueCreate(LOG_QUEUE_LEN, sizeof(LogLine_t));
    if (s_queue == nullptr) {
        Serial.println("[SD] queue alloc failed — logging disabled");
        return;
    }
    xTaskCreatePinnedToCore(sd_log_task, "sd_log", 6144, NULL, 2, &s_task, 0);
    Serial.println("[SD] logger task started");
}
