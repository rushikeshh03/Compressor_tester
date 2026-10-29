/******************************************************************************************************************************
 * @file    sd_log.cpp
 * @brief   SD-card durable store-and-forward log (NDJSON, keep-forever + persisted upload cursor).
 *          See sd_log.h for the role in the CAN -> MQTT pipeline.
 ******************************************************************************************************************************/

#include "main.h"
#include "sd_log.h"
#include <SPI.h>
#include <SD.h>
#include <nvs.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *SD_TAG = "SDLOG";

/* ── Tunables ──────────────────────────────────────────────────────────── */
#define SD_SPI_HZ            (10UL * 1000UL * 1000UL)  /* conservative; bump after bring-up */
#define SD_SEG_MAX_BYTES     (256UL * 1024UL)          /* rotate to a new segment at ~256 KB */
#define SD_LINE_BUF          160                        /* one NDJSON line fits easily        */

/* ── Persisted state (NVS namespace "iot_cfg", shared with CAN baud) ──────
 *  Physical location of the upload cursor + monotonic record counters.      */
#define SD_NVS_NS            CAN_BAUD_NVS_NS            /* "iot_cfg" (see main.h) */
#define K_WSEG               "sd_wseg"                  /* u32: current write segment index   */
#define K_WSEQ               "sd_wseq"                  /* u64: total records ever written     */
#define K_USEG               "sd_useg"                  /* u32: upload cursor segment index    */
#define K_UOFF               "sd_uoff"                  /* u32: upload cursor byte offset       */
#define K_USEQ               "sd_useq"                  /* u64: total records marked uploaded  */

static bool     s_ready = false;
static SPIClass s_spi(HSPI);

/* Shared SD access lock (see sd_log.h). Recursive so a public function may nest calls. */
static SemaphoreHandle_t s_sd_mtx = NULL;

void sd_lock_init(void)
{
    if (s_sd_mtx == NULL) s_sd_mtx = xSemaphoreCreateRecursiveMutex();
}

bool sd_lock(int timeout_ms)
{
    if (s_sd_mtx == NULL) return true;   /* not created yet — best-effort, don't block callers */
    TickType_t t = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(s_sd_mtx, t) == pdTRUE;
}

void sd_unlock(void)
{
    if (s_sd_mtx) xSemaphoreGiveRecursive(s_sd_mtx);
}

static uint32_t s_wseg = 1;    /* segment we currently append to            */
static uint64_t s_wseq = 0;    /* absolute count of records ever written    */
static uint32_t s_useg = 1;    /* segment holding the next un-uploaded rec  */
static uint32_t s_uoff = 0;    /* byte offset of that record in s_useg      */
static uint64_t s_useq = 0;    /* absolute count of records marked uploaded */

/* ─────────────────────────────────────────────────────────────────────────
 *  NVS helpers
 * ───────────────────────────────────────────────────────────────────────── */
static void nvs_load_state(void)
{
    nvs_handle_t h;
    if (nvs_open(SD_NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    uint32_t u32; uint64_t u64;
    if (nvs_get_u32(h, K_WSEG, &u32) == ESP_OK) s_wseg = u32;
    if (nvs_get_u64(h, K_WSEQ, &u64) == ESP_OK) s_wseq = u64;
    if (nvs_get_u32(h, K_USEG, &u32) == ESP_OK) s_useg = u32;
    if (nvs_get_u32(h, K_UOFF, &u32) == ESP_OK) s_uoff = u32;
    if (nvs_get_u64(h, K_USEQ, &u64) == ESP_OK) s_useq = u64;
    nvs_close(h);
}

static void nvs_save_write(void)   /* persist write-side counters */
{
    nvs_handle_t h;
    if (nvs_open(SD_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u32(h, K_WSEG, s_wseg);
    nvs_set_u64(h, K_WSEQ, s_wseq);
    nvs_commit(h);
    nvs_close(h);
}

static void nvs_save_cursor(void)  /* persist upload cursor */
{
    nvs_handle_t h;
    if (nvs_open(SD_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u32(h, K_USEG, s_useg);
    nvs_set_u32(h, K_UOFF, s_uoff);
    nvs_set_u64(h, K_USEQ, s_useq);
    nvs_commit(h);
    nvs_close(h);
}

/* ─────────────────────────────────────────────────────────────────────────
 *  Path / file helpers
 * ───────────────────────────────────────────────────────────────────────── */
static void seg_path(uint32_t seg, char *out, size_t out_size)
{
    snprintf(out, out_size, "/log/%06u.ndjson", (unsigned)seg);
}

/* Read one line (up to '\n') from an open file into buf (NUL-terminated).
 * @param out_had_nl  set true only if a terminating '\n' was actually consumed
 *                    (false => end-of-file or a torn trailing record).
 * @return line length excluding the newline, or -1 if nothing was read (EOF). */
static int read_line(File &f, char *buf, size_t bufsz, bool *out_had_nl)
{
    size_t len = 0;
    bool   had_nl = false;
    int    any = 0;

    while (len + 1 < bufsz)
    {
        int c = f.read();
        if (c < 0) break;              /* EOF */
        any = 1;
        if (c == '\n') { had_nl = true; break; }
        if (c != '\r') buf[len++] = (char)c;
    }
    buf[len] = '\0';
    if (out_had_nl) *out_had_nl = had_nl;
    if (!any) return -1;
    return (int)len;
}

/* Parse one NDJSON line -> CanFrame_t. Returns true on success. */
static bool parse_line(const char *line, CanFrame_t *out)
{
    memset(out, 0, sizeof(*out));

    const char *pt = strstr(line, "\"t\":");
    const char *pid = strstr(line, "\"id\":\"");
    const char *pd = strstr(line, "\"d\":\"");
    if (!pt || !pid) return false;

    out->epoch_ms = strtoull(pt + 4, NULL, 10);
    out->id       = (uint32_t)strtoul(pid + 6, NULL, 16);

    uint8_t dlc = 0;
    if (pd)
    {
        const char *h = pd + 5;
        while (h[0] && h[1] && h[0] != '"' && dlc < 8)
        {
            char pair[3] = { h[0], h[1], '\0' };
            out->data[dlc++] = (uint8_t)strtoul(pair, NULL, 16);
            h += 2;
        }
    }
    out->dlc = dlc;
    return true;
}

/* ─────────────────────────────────────────────────────────────────────────
 *  Public API
 * ───────────────────────────────────────────────────────────────────────── */
bool sd_log_init(void)
{
    s_ready = false;
    sd_lock(-1);

    /* Mount only if nobody has yet — event_log_init() may have mounted the card first.
       Both modules share the single global `SD` object, so a second SD.begin() must be avoided. */
    if (SD.cardType() == CARD_NONE)
    {
        s_spi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
        if (!SD.begin(SD_CS, s_spi, SD_SPI_HZ))
        {
            ESP_LOGE(SD_TAG, "SD mount FAILED (CS=%d MOSI=%d SCK=%d MISO=%d) — legacy RAM path will be used",
                     SD_CS, SD_MOSI, SD_SCK, SD_MISO);
            sd_unlock();
            return false;
        }
    }

    SD.mkdir("/log");
    nvs_load_state();

    /* Sanity: the write segment file must exist (create empty if first boot / lost). */
    char path[32];
    seg_path(s_wseg, path, sizeof(path));
    if (!SD.exists(path))
    {
        File f = SD.open(path, FILE_WRITE);   /* create empty */
        if (f) f.close();
    }

    s_ready = true;
    ESP_LOGI(SD_TAG, "SD mounted. wseg=%u wseq=%llu | cursor useg=%u uoff=%u useq=%llu (pending=%llu)",
             (unsigned)s_wseg, (unsigned long long)s_wseq,
             (unsigned)s_useg, (unsigned)s_uoff, (unsigned long long)s_useq,
             (unsigned long long)(s_wseq - s_useq));
    sd_unlock();
    return true;
}

bool sd_log_ready(void)
{
    return s_ready;
}

bool sd_log_append_frames(const CanFrame_t *frames, size_t n)
{
    if (!s_ready || frames == NULL || n == 0) return false;

    sd_lock(-1);
    char path[32];
    seg_path(s_wseg, path, sizeof(path));
    File f = SD.open(path, FILE_APPEND);
    if (!f)
    {
        ESP_LOGE(SD_TAG, "append: open %s failed", path);
        sd_unlock();
        return false;
    }

    size_t segBytes = f.size();
    char   line[SD_LINE_BUF];
    char   dhex[20];
    bool   ok = true;

    for (size_t i = 0; i < n; i++)
    {
        /* Rotate to a fresh segment once the current one is large enough. */
        if (segBytes >= SD_SEG_MAX_BYTES)
        {
            f.close();
            s_wseg++;
            nvs_save_write();
            seg_path(s_wseg, path, sizeof(path));
            f = SD.open(path, FILE_APPEND);
            if (!f) { ESP_LOGE(SD_TAG, "append: open new seg %s failed", path); ok = false; break; }
            segBytes = 0;
        }

        /* payload hex — same lower-case formatting as the MQTT frame object */
        size_t pos = 0;
        for (uint8_t k = 0; k < frames[i].dlc && (pos + 2) < sizeof(dhex); k++)
            pos += snprintf(dhex + pos, sizeof(dhex) - pos, "%02x", frames[i].data[k]);
        dhex[pos] = '\0';

        int len = snprintf(line, sizeof(line),
                           "{\"t\":%llu,\"id\":\"%08x\",\"d\":\"%s\"}\n",
                           (unsigned long long)frames[i].epoch_ms,
                           (unsigned)frames[i].id, dhex);
        if (len <= 0 || (size_t)len >= sizeof(line)) { ok = false; break; }

        if (f.write((const uint8_t *)line, (size_t)len) != (size_t)len) { ok = false; break; }
        segBytes += (size_t)len;
        s_wseq++;
    }

    f.close();
    nvs_save_write();
    sd_unlock();
    return ok;
}

size_t sd_log_peek_unuploaded(CanFrame_t *out, size_t max, uint64_t *out_first_seq)
{
    if (!s_ready || out == NULL || max == 0) return 0;
    if (out_first_seq) *out_first_seq = s_useq;
    if (s_wseq <= s_useq) return 0;

    sd_lock(-1);
    uint32_t seg = s_useg;
    uint32_t off = s_uoff;
    size_t   got = 0;
    char     line[SD_LINE_BUF];
    char     path[32];

    seg_path(seg, path, sizeof(path));
    File f = SD.open(path, FILE_READ);
    if (f) f.seek(off);

    while (got < max)
    {
        if (!f) break;

        bool had_nl = false;
        int  len = read_line(f, line, sizeof(line), &had_nl);

        if (len >= 0 && had_nl)
        {
            if (parse_line(line, &out[got])) got++;
            continue;
        }

        /* No complete line left in this segment. */
        f.close();
        if (seg < s_wseg)          /* finalized segment fully consumed -> next one */
        {
            seg++;
            off = 0;
            seg_path(seg, path, sizeof(path));
            f = SD.open(path, FILE_READ);
            continue;
        }
        break;                      /* reached the live write head */
    }

    if (f) f.close();
    sd_unlock();
    return got;
}

void sd_log_mark_uploaded(size_t n)
{
    if (!s_ready || n == 0) return;

    /* Never advance past what has actually been written. */
    uint64_t pending = s_wseq - s_useq;
    if ((uint64_t)n > pending) n = (size_t)pending;
    if (n == 0) return;

    sd_lock(-1);
    uint32_t seg = s_useg;
    uint32_t off = s_uoff;
    size_t   done = 0;
    char     line[SD_LINE_BUF];
    char     path[32];

    seg_path(seg, path, sizeof(path));
    File f = SD.open(path, FILE_READ);
    if (f) f.seek(off);

    while (done < n && f)
    {
        bool had_nl = false;
        int  len = read_line(f, line, sizeof(line), &had_nl);

        if (len >= 0 && had_nl)
        {
            done++;
            off = (uint32_t)f.position();
            continue;
        }

        f.close();
        if (seg < s_wseg)
        {
            seg++;
            off = 0;
            seg_path(seg, path, sizeof(path));
            f = SD.open(path, FILE_READ);
            continue;
        }
        break;
    }
    if (f) f.close();

    s_useg = seg;
    s_uoff = off;
    s_useq += done;
    nvs_save_cursor();

    ESP_LOGI(SD_TAG, "marked %u uploaded -> cursor useg=%u uoff=%u useq=%llu (pending=%llu)",
             (unsigned)done, (unsigned)s_useg, (unsigned)s_uoff,
             (unsigned long long)s_useq, (unsigned long long)(s_wseq - s_useq));
    sd_unlock();
}

size_t sd_log_unuploaded_count(void)
{
    if (!s_ready || s_wseq <= s_useq) return 0;
    return (size_t)(s_wseq - s_useq);
}

uint32_t sd_log_write_segment(void)
{
    return s_wseg;
}

bool sd_log_force_rotate(void)
{
    if (!s_ready) return false;

    sd_lock(-1);
    /* Advance to a fresh, empty segment; the just-closed one becomes finalized/immutable. */
    s_wseg++;
    nvs_save_write();

    char path[32];
    seg_path(s_wseg, path, sizeof(path));
    bool ok = true;
    if (!SD.exists(path))
    {
        File f = SD.open(path, FILE_WRITE);   /* create empty */
        if (f) f.close();
        else   ok = false;
    }
    sd_unlock();
    ESP_LOGI(SD_TAG, "force-rotate -> new write segment %06u", (unsigned)s_wseg);
    return ok;
}
