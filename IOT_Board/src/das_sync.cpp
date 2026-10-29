/******************************************************************************************************************************
 * @file    das_sync.cpp
 * @brief   Local Data Server sync client. See das_sync.h for the role & protocol.
 *
 *  Flow: init_task -> das_sync_init() (config + MQTT control + task). can.cpp enqueues a
 *  decoded result -> das_sync_task builds /barcodes/<bc>.csv and uploads it over HTTP. A
 *  daily 23:00 job uploads that day's raw NDJSON segments + the previous day's event log.
 *  Unreachable server -> everything stays on SD and is retried hourly. All blocking WiFi /
 *  HTTP work runs on das_sync_task (core 0) — never on the CAN Rx task.
 ******************************************************************************************************************************/

#include "das_sync.h"

#if CONFIG_DAS_SYNC

#include "main.h"
#include "wifi_mqtt.h"

#include <WiFi.h>
#include <HTTPClient.h>
#include <ESPmDNS.h>
#include <SD.h>
#include <FS.h>
#include <mqtt_client.h>
#include <nvs.h>
#include <time.h>
#include <string.h>
#include "mbedtls/sha256.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *DS_TAG = "DASSYNC";

#define DS_NVS_NS  CAN_BAUD_NVS_NS   /* "iot_cfg" — shared config namespace */
#define SEGHW_UNSET 0xFFFFFFFFu

/* ── Configuration (defaults from das_sync.h, overridable from NVS) ── */
static char     s_host[64]    = DS_HOST_DEFAULT;
static uint16_t s_httpPort    = DS_HTTP_PORT_DEFAULT;
static uint16_t s_mqttPort    = DS_MQTT_PORT_DEFAULT;
static char     s_station[24] = DS_STATION_DEFAULT;
static char     s_token[48]   = DS_TOKEN_DEFAULT;
static char     s_mqttUser[48]= DS_MQTT_USER_DEFAULT;
static char     s_mqttPass[48]= DS_MQTT_PASS_DEFAULT;
static char     s_deviceId[40]= {0};

/* ── Control topics (das/<station>/<dev>/<leaf>) ── */
static char s_tPresence[96], s_tStatusReq[96], s_tStatusRes[96];
static char s_tSyncList[96],  s_tSyncReq[96],   s_tUploadAck[96];
static char s_tResult[96];    /* decoded per-scan result summary (published) */

/* ── Server address resolution ── */
static IPAddress s_ip;
static bool s_resolved = false, s_hostLiteral = false, s_mdnsBegun = false;

/* ── MQTT control session ── */
static esp_mqtt_client_handle_t s_mqtt = NULL;
static volatile bool s_mqttUp = false;
static char s_uri[96] = {0};

/* sync/req is delivered on the esp-mqtt event task — relay it to das_sync_task */
static char          s_syncReq[1024];
static volatile bool s_syncReqPending = false;

/* ── Finished raw-session queue (can.cpp -> das_sync_task) ──
 *  Each element owns a heap buffer of CSV frame rows; das_sync writes it to
 *  /barcodes/<barcode>_<stamp>.csv (prepending the header), uploads it, then frees buf. */
typedef struct {
    char     barcode[25];
    char    *buf;          /* heap block (PSRAM-preferred); das_sync frees it */
    uint32_t len;
    bool     truncated;    /* session hit the RAM cap — some trailing frames dropped */
} das_raw_t;
static QueueHandle_t s_rawQ    = NULL;

/* Decoded results awaiting MQTT publish (das/<station>/<dev>/result). */
static QueueHandle_t s_resultQ = NULL;

/* ── Persisted sync state (NVS) ── */
static uint32_t s_segHw   = SEGHW_UNSET;  /* highest finalized segment index already synced */
static uint32_t s_evDate  = 0;            /* YYYYMMDD of the last event log synced           */
static uint32_t s_lastDay = 0;            /* YYYYMMDD the daily job last completed            */

/* ─────────────────────────────────────────────────────────────────────────────
 *  NVS helpers
 * ───────────────────────────────────────────────────────────────────────────── */
static void nvs_save_u32(const char *key, uint32_t v)
{
    nvs_handle_t h;
    if (nvs_open(DS_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u32(h, key, v);
        nvs_commit(h);
        nvs_close(h);
    }
}

static void load_config(void)
{
    nvs_handle_t h;
    if (nvs_open(DS_NVS_NS, NVS_READONLY, &h) != ESP_OK) return;

    size_t n;
    n = sizeof(s_host);    nvs_get_str(h, "ds_host",    s_host,    &n);
    n = sizeof(s_station); nvs_get_str(h, "ds_station", s_station, &n);
    n = sizeof(s_token);   nvs_get_str(h, "ds_token",   s_token,   &n);
    n = sizeof(s_mqttUser);nvs_get_str(h, "ds_muser",   s_mqttUser,&n);
    n = sizeof(s_mqttPass);nvs_get_str(h, "ds_mpass",   s_mqttPass,&n);
    uint16_t v16;
    if (nvs_get_u16(h, "ds_http", &v16) == ESP_OK) s_httpPort = v16;
    if (nvs_get_u16(h, "ds_mqtt", &v16) == ESP_OK) s_mqttPort = v16;
    uint32_t v32;
    if (nvs_get_u32(h, "ds_seg_hw",  &v32) == ESP_OK) s_segHw   = v32;
    if (nvs_get_u32(h, "ds_evdate",  &v32) == ESP_OK) s_evDate  = v32;
    if (nvs_get_u32(h, "ds_lastday", &v32) == ESP_OK) s_lastDay = v32;
    nvs_close(h);
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  Time helpers (software RTC in timestampData; UTC after SNTP)
 * ───────────────────────────────────────────────────────────────────────────── */
static uint32_t today_ymd(void)
{
    if (!ntpTimeReceived && !gpsTimeReceived) return 0;
    rtc_update_timestamp();
    return (uint32_t)(2000 + timestampData.year) * 10000u
         + (uint32_t)timestampData.month * 100u
         + (uint32_t)timestampData.day;
}

static uint32_t yesterday_ymd(void)
{
    if (!ntpTimeReceived && !gpsTimeReceived) return 0;
    rtc_update_timestamp();
    struct tm t = {};
    t.tm_year = (2000 + timestampData.year) - 1900;
    t.tm_mon  = timestampData.month - 1;
    t.tm_mday = timestampData.day;
    t.tm_hour = 12;                       /* noon avoids DST/rounding edge cases */
    time_t s = mktime(&t) - 86400;        /* mktime uses the tz we set (UTC, offset 0) */
    struct tm y; localtime_r(&s, &y);
    return (uint32_t)(y.tm_year + 1900) * 10000u
         + (uint32_t)(y.tm_mon + 1) * 100u
         + (uint32_t)y.tm_mday;
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  SHA-256 helpers
 * ───────────────────────────────────────────────────────────────────────────── */
static String sha256Hex(const uint8_t *data, size_t len)
{
    uint8_t out[32];
    mbedtls_sha256(data, len, out, 0);           /* 0 = SHA-256 (not SHA-224) */
    static const char *hx = "0123456789abcdef";
    String s; s.reserve(64);
    for (int i = 0; i < 32; i++) { s += hx[out[i] >> 4]; s += hx[out[i] & 0xF]; }
    return s;
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  Server resolve (literal IP / DNS / mDNS <name>.local) — re-run on drops
 * ───────────────────────────────────────────────────────────────────────────── */
static bool resolveServer(void)
{
    IPAddress lit;
    if (lit.fromString(s_host)) { s_ip = lit; s_resolved = true; s_hostLiteral = true; return true; }

    if (WiFi.status() != WL_CONNECTED) return false;
    if (!s_mdnsBegun) { MDNS.begin(s_deviceId); s_mdnsBegun = true; }

    IPAddress ip;
    if (WiFi.hostByName(s_host, ip) == 1 && ip != IPAddress((uint32_t)0)) {
        s_ip = ip; s_resolved = true;
    } else {
        String hn = s_host;
        if (hn.endsWith(".local")) hn = hn.substring(0, hn.length() - 6);
        IPAddress mip = MDNS.queryHost(hn.c_str(), 2000);
        if (mip != IPAddress((uint32_t)0)) { s_ip = mip; s_resolved = true; }
    }
    if (s_resolved) ESP_LOGI(DS_TAG, "%s -> %s", s_host, s_ip.toString().c_str());
    else            ESP_LOGW(DS_TAG, "could not resolve %s", s_host);
    return s_resolved;
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  MQTT control session (presence + status + sync negotiation)
 * ───────────────────────────────────────────────────────────────────────────── */
static bool topic_is(esp_mqtt_event_handle_t e, const char *full)
{
    size_t l = strlen(full);
    return e->topic_len == (int)l && strncmp(e->topic, full, l) == 0;
}

static void mqtt_event_handler(void *args, esp_event_base_t base, int32_t id, void *data)
{
    (void)args; (void)base;
    esp_mqtt_event_handle_t e = (esp_mqtt_event_handle_t)data;

    switch ((esp_mqtt_event_id_t)id)
    {
        case MQTT_EVENT_CONNECTED:
            s_mqttUp = true;
            esp_mqtt_client_publish(s_mqtt, s_tPresence, "online", 6, 0, 1 /*retain*/);
            esp_mqtt_client_subscribe(s_mqtt, s_tStatusRes, 0);
            esp_mqtt_client_subscribe(s_mqtt, s_tSyncReq,   0);
            esp_mqtt_client_subscribe(s_mqtt, s_tUploadAck, 0);
            ESP_LOGI(DS_TAG, "control broker connected");
            EVLOG_OK("SYNC", "control MQTT connected (%s:%u)", s_ip.toString().c_str(), (unsigned)s_mqttPort);
            break;

        case MQTT_EVENT_DISCONNECTED:
            s_mqttUp = false;
            ESP_LOGW(DS_TAG, "control broker disconnected");
            break;

        case MQTT_EVENT_DATA:
            if (topic_is(e, s_tSyncReq)) {
                int n = e->data_len < (int)sizeof(s_syncReq) - 1 ? e->data_len : (int)sizeof(s_syncReq) - 1;
                memcpy(s_syncReq, e->data, n);
                s_syncReq[n] = '\0';
                s_syncReqPending = true;      /* das_sync_task handles the uploads */
            } else if (topic_is(e, s_tStatusRes)) {
                ESP_LOGI(DS_TAG, "status/res received");
            } else if (topic_is(e, s_tUploadAck)) {
                ESP_LOGI(DS_TAG, "upload/ack received");
            }
            break;

        default:
            break;
    }
}

static bool controlEnsure(void)
{
    if (s_mqtt && s_mqttUp) return true;
    if (!s_resolved) return false;

    if (!s_mqtt) {
        snprintf(s_uri, sizeof(s_uri), "mqtt://%s:%u", s_ip.toString().c_str(), (unsigned)s_mqttPort);

        esp_mqtt_client_config_t cfg = {};
        cfg.uri       = s_uri;
        cfg.client_id = s_deviceId;
        if (s_mqttUser[0]) { cfg.username = s_mqttUser; cfg.password = s_mqttPass; }
        cfg.lwt_topic   = s_tPresence;      /* broker publishes this if we drop */
        cfg.lwt_msg     = "offline";
        cfg.lwt_qos     = 0;
        cfg.lwt_retain  = 1;
        cfg.lwt_msg_len = 7;
        cfg.keepalive          = 30;
        cfg.network_timeout_ms = 5000;

        s_mqtt = esp_mqtt_client_init(&cfg);
        if (!s_mqtt) { ESP_LOGE(DS_TAG, "mqtt init failed"); return false; }
        esp_mqtt_client_register_event(s_mqtt, (esp_mqtt_event_id_t)MQTT_EVENT_ANY,
                                       mqtt_event_handler, NULL);
        if (esp_mqtt_client_start(s_mqtt) != ESP_OK) {
            ESP_LOGE(DS_TAG, "mqtt start failed");
            esp_mqtt_client_destroy(s_mqtt); s_mqtt = NULL; return false;
        }
    }
    return s_mqttUp;   /* connects asynchronously — caller tolerates not-yet-up */
}

/* Recreate the control client on the next controlEnsure() — used when a hostname's IP moved. */
static void controlReset(void)
{
    if (s_mqtt) { esp_mqtt_client_destroy(s_mqtt); s_mqtt = NULL; }
    s_mqttUp = false;
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  HTTP upload (POST /api/upload, SHA-256 header, resend on bad ACK)
 * ───────────────────────────────────────────────────────────────────────────── */
static bool httpUpload(const char *name, const uint8_t *data, size_t len)
{
    if (!s_resolved || WiFi.status() != WL_CONNECTED) return false;

    String url = String("http://") + s_ip.toString() + ":" + s_httpPort + "/api/upload";
    String sha = sha256Hex(data, len);

    for (int attempt = 1; attempt <= DS_UPLOAD_RETRIES; attempt++) {
        WiFiClient client;
        HTTPClient http;
        if (!http.begin(client, url)) return false;
        http.addHeader("Content-Type", "text/csv");
        http.addHeader("X-Token",    s_token);
        http.addHeader("X-Station",  s_station);
        http.addHeader("X-Device",   s_deviceId);
        http.addHeader("X-Filename", name);
        http.addHeader("X-Sha256",   sha);
        int    code = http.POST((uint8_t *)data, len);
        String resp = http.getString();
        http.end();

        bool ok = (code == 200) && (resp.indexOf("\"ok\":true") >= 0);
        ESP_LOGI(DS_TAG, "upload %s attempt %d -> HTTP %d %s", name, attempt, code, ok ? "ACK ok" : "ACK BAD");
        if (ok) { wifi_note_result(true); return true; }
        vTaskDelay(pdMS_TO_TICKS(400));
    }
    wifi_note_result(false);
    return false;
}

/* Read a whole SD file into a (PSRAM-preferred) buffer, then upload it. The SD lock is
 * held only for the read (bounded, ~200 ms for a 256 KB segment); the HTTP POST runs
 * with the lock released so the flush/event tasks are not starved during the network I/O. */
static bool uploadFile(const char *path, const char *name)
{
    if (!sd_lock(-1)) return false;
    File f = SD.open(path, FILE_READ);
    if (!f) { sd_unlock(); ESP_LOGW(DS_TAG, "open %s failed", path); return false; }
    size_t sz = f.size();

    uint8_t *buf = (uint8_t *)heap_caps_malloc(sz ? sz : 1, MALLOC_CAP_SPIRAM);
    if (!buf) buf = (uint8_t *)malloc(sz ? sz : 1);
    if (!buf) { f.close(); sd_unlock(); ESP_LOGE(DS_TAG, "no buffer for %s (%u B)", path, (unsigned)sz); return false; }

    size_t rd = 0;
    while (rd < sz) {
        size_t chunk = (sz - rd) > 4096 ? 4096 : (sz - rd);
        int n = f.read(buf + rd, chunk);
        if (n <= 0) break;
        rd += (size_t)n;
    }
    f.close();
    sd_unlock();

    bool ok = (rd == sz) && httpUpload(name, buf, sz);
    free(buf);
    return ok;
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  Per-scan raw-CAN session CSV
 * ───────────────────────────────────────────────────────────────────────────── */

/* Filename stamp: dated once the clock is set (YYYYMMDD_HHMMSS), else uptime. */
static void make_stamp(char *out, size_t n)
{
    if (ntpTimeReceived || gpsTimeReceived) {
        rtc_update_timestamp();
        snprintf(out, n, "%04u%02u%02u_%02u%02u%02u",
                 (unsigned)(2000 + timestampData.year), (unsigned)timestampData.month,
                 (unsigned)timestampData.day, (unsigned)timestampData.hour,
                 (unsigned)timestampData.minute, (unsigned)timestampData.second);
    } else {
        snprintf(out, n, "up%lu", (unsigned long)millis());
    }
}

/* Copy a barcode into a filesystem-safe base name (path separators / spaces -> '_'). */
static void safe_name(const char *bc, char *out, size_t n)
{
    size_t j = 0;
    for (size_t i = 0; bc[i] && j + 1 < n; i++) {
        char ch = bc[i];
        out[j++] = (ch == '/' || ch == '\\' || ch == ' ') ? '_' : ch;
    }
    out[j] = '\0';
}

static bool marker_exists(const char *csvName)
{
    char p[80]; snprintf(p, sizeof(p), "%s/%s.sent", DS_CSV_DIR, csvName);
    if (!sd_lock(-1)) return false;
    bool e = SD.exists(p);
    sd_unlock();
    return e;
}

static void marker_write(const char *csvName)
{
    char p[80]; snprintf(p, sizeof(p), "%s/%s.sent", DS_CSV_DIR, csvName);
    if (!sd_lock(-1)) return;
    File f = SD.open(p, FILE_WRITE);
    if (f) f.close();
    sd_unlock();
}

/* Write one finished raw-CAN session to /barcodes/<barcode>_<stamp>.csv (header + rows),
 * then upload it promptly. On a failed upload the file stays on SD without a .sent marker
 * and is retried by the hourly pass. The session's heap buffer is freed here. */
static void handle_raw_session(das_raw_t *rw)
{
    if (rw->buf == NULL) return;

    char base[40]; safe_name(rw->barcode[0] ? rw->barcode : "blank", base, sizeof(base));
    char stamp[20]; make_stamp(stamp, sizeof(stamp));
    char name[80]; snprintf(name, sizeof(name), "%s_%s.csv", base[0] ? base : "blank", stamp);
    char path[100]; snprintf(path, sizeof(path), "%s/%s", DS_CSV_DIR, name);

    bool wrote = false;
    if (sd_lock(-1)) {
        SD.mkdir(DS_CSV_DIR);
        File f = SD.open(path, FILE_WRITE);
        if (f) {
            f.println("Time,Dir,ID,DLC,Data,Decoded");   /* matches the LCD raw-CAN format */
            f.write((const uint8_t *)rw->buf, rw->len);
            f.close();
            wrote = true;
        }
        sd_unlock();
    }
    free(rw->buf);
    rw->buf = NULL;

    if (!wrote) { ESP_LOGW(DS_TAG, "raw CSV write failed %s", name); return; }
    if (rw->truncated) EVLOG_WARN("SYNC", "raw session %s truncated (RAM cap)", name);

    if (uploadFile(path, name)) {
        marker_write(name);
        EVLOG_OK("SYNC", "uploaded raw %s", name);
    } else {
        EVLOG_WARN("SYNC", "raw upload deferred %s (server unreachable)", name);
    }
}

/* Publish ONE decoded result summary over MQTT (das/<station>/<dev>/result, QoS1) so the
 * server keeps PASS/FAIL + cells/temps for analytics. The raw frames still go over HTTP. */
static void publish_result(const das_result_t *r)
{
    if (!s_mqtt || !s_mqttUp) return;

    const char *result = (r->incomplete || !r->have_status) ? "INCOMPLETE"
                       : (r->status ? "PASS" : "FAIL");

    JsonDocument doc;
    doc["barcode"] = r->barcode;
    doc["result"]  = result;
    doc["status"]  = r->have_status ? (int)r->status : -1;
    doc["ts"]      = r->epoch_ms;
    doc["device"]  = s_deviceId;
    doc["station"] = s_station;
    JsonArray cells = doc["cells_mV"].to<JsonArray>();
    for (int i = 0; i < 8; i++) cells.add(r->cell_mV[i]);
    JsonArray temps = doc["temps_c"].to<JsonArray>();
    for (int i = 0; i < 4; i++) temps.add(r->temp_c10[i] / 10.0f);
    doc["cellFaultMask"] = r->cellFaultMask;
    doc["tempFaultMask"] = r->tempFaultMask;
    doc["initFailed"]    = r->initFailed ? 1 : 0;
    doc["incomplete"]    = r->incomplete;

    char buf[512];
    size_t n = serializeJson(doc, buf, sizeof(buf));
    int id = esp_mqtt_client_publish(s_mqtt, s_tResult, buf, (int)n, 1 /*QoS1*/, 0);
    if (id >= 0) EVLOG_OK("SYNC", "result %s %s", r->barcode, result);
    else         EVLOG_WARN("SYNC", "result publish failed %s", r->barcode);
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  Hourly retry + server-initiated sync
 * ───────────────────────────────────────────────────────────────────────────── */
#define DS_MAX_PENDING 32

/* Collect up to DS_MAX_PENDING CSV base names in DS_CSV_DIR that have no .sent marker. */
static int collect_pending(char names[][64], int maxN)
{
    int count = 0;
    if (!sd_lock(-1)) return 0;
    File dir = SD.open(DS_CSV_DIR);
    if (dir) {
        for (File e = dir.openNextFile(); e && count < maxN; e = dir.openNextFile()) {
            const char *fn = strrchr(e.name(), '/');
            fn = fn ? fn + 1 : e.name();
            size_t l = strlen(fn);
            bool isCsv = (l > 4) && strcmp(fn + l - 4, ".csv") == 0;
            if (isCsv) { strncpy(names[count], fn, 63); names[count][63] = '\0'; count++; }
            e.close();
        }
        dir.close();
    }
    sd_unlock();

    /* Drop the ones already marked sent (marker_exists takes the lock itself). */
    int keep = 0;
    for (int i = 0; i < count; i++)
        if (!marker_exists(names[i])) { if (keep != i) strcpy(names[keep], names[i]); keep++; }
    return keep;
}

static void retryPendingCsvs(void)
{
    static char names[DS_MAX_PENDING][64];
    int n = collect_pending(names, DS_MAX_PENDING);
    for (int i = 0; i < n; i++) {
        char path[80]; snprintf(path, sizeof(path), "%s/%s", DS_CSV_DIR, names[i]);
        if (uploadFile(path, names[i])) { marker_write(names[i]); EVLOG_OK("SYNC", "retried %s", names[i]); }
    }
}

/* Publish the pending CSV list (name+sha+size) so the server can request what it lacks. */
static void publishSyncList(void)
{
    if (!s_mqttUp) return;
    static char names[DS_MAX_PENDING][64];
    int n = collect_pending(names, DS_MAX_PENDING);

    String p = "{\"files\":[";
    int emitted = 0;
    for (int i = 0; i < n; i++) {
        char path[80]; snprintf(path, sizeof(path), "%s/%s", DS_CSV_DIR, names[i]);
        if (!sd_lock(-1)) continue;
        File f = SD.open(path, FILE_READ);
        if (!f) { sd_unlock(); continue; }
        size_t sz = f.size();
        uint8_t *buf = (uint8_t *)malloc(sz ? sz : 1);
        size_t rd = buf ? f.read(buf, sz) : 0;
        f.close();
        sd_unlock();
        if (buf && rd == sz) {
            if (emitted) p += ",";
            p += "{\"name\":\""; p += names[i]; p += "\",\"sha256\":\"";
            p += sha256Hex(buf, sz); p += "\",\"size\":"; p += String((unsigned)sz); p += "}";
            emitted++;
        }
        free(buf);
    }
    p += "]}";
    esp_mqtt_client_publish(s_mqtt, s_tSyncList, p.c_str(), p.length(), 0, 0);
    ESP_LOGI(DS_TAG, "published sync/list (%d file(s))", emitted);
}

static void handleSyncReq(const char *payload)
{
    JsonDocument doc;
    if (deserializeJson(doc, payload)) return;
    JsonArray miss = doc["missing"].as<JsonArray>();
    if (miss.isNull()) return;
    for (JsonVariant v : miss) {
        const char *name = v.as<const char *>();
        if (!name) continue;
        char path[80]; snprintf(path, sizeof(path), "%s/%s", DS_CSV_DIR, name);
        if (uploadFile(path, name)) { marker_write(name); ESP_LOGI(DS_TAG, "sync re-sent %s", name); }
    }
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  Daily job — today's raw segments + yesterday's finalized event log
 * ───────────────────────────────────────────────────────────────────────────── */
static bool sd_exists(const char *p)
{
    if (!sd_lock(-1)) return false;
    bool e = SD.exists(p);
    sd_unlock();
    return e;
}

static bool dailyJob(void)
{
    uint32_t ymd = today_ymd();
    if (ymd == 0) return false;

    /* Finalize today's tail so it can be uploaded with a stable SHA. */
    sd_log_force_rotate();
    uint32_t wseg = sd_log_write_segment();     /* new empty write segment */

    bool allOk = true;

    /* 1) Every finalized segment not yet synced. */
    for (uint32_t seg = s_segHw + 1; seg < wseg; seg++) {
        char path[40]; snprintf(path, sizeof(path), "/log/%06u.ndjson", (unsigned)seg);
        if (!sd_exists(path)) { s_segHw = seg; nvs_save_u32("ds_seg_hw", s_segHw); continue; }
        char name[48]; snprintf(name, sizeof(name), "raw_%08u_%06u.ndjson", (unsigned)ymd, (unsigned)seg);
        if (uploadFile(path, name)) { s_segHw = seg; nvs_save_u32("ds_seg_hw", s_segHw); }
        else { allOk = false; break; }
    }

    /* 2) Yesterday's now-finalized event log (once). */
    uint32_t yday = yesterday_ymd();
    if (yday && s_evDate != yday) {
        char path[48]; snprintf(path, sizeof(path), "/events/%08u.log", (unsigned)yday);
        if (sd_exists(path)) {
            char name[48]; snprintf(name, sizeof(name), "events_%08u.log", (unsigned)yday);
            if (uploadFile(path, name)) { s_evDate = yday; nvs_save_u32("ds_evdate", yday); }
            else allOk = false;
        } else {
            s_evDate = yday; nvs_save_u32("ds_evdate", yday);   /* nothing logged that day */
        }
    }

    if (allOk) EVLOG_OK("SYNC", "daily job done (ymd=%u, segHw=%u)", (unsigned)ymd, (unsigned)s_segHw);
    return allOk;
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  Task
 * ───────────────────────────────────────────────────────────────────────────── */
static void das_sync_task(void *arg)
{
    (void)arg;
    xEventGroupWaitBits(initTaskEventGroup, INIT_TASK_COMPLETE_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    uint32_t lastPresence   = 0;
    uint32_t lastRetry      = 0;
    uint32_t dailyAttemptMs = 0;
    uint32_t lastDiag       = 0;
    bool     wasUp          = false;

    for (;;) {
        bool wifi = wifi_ensure_connected();
        if (wifi) {
            if (!s_resolved) resolveServer();
            else if (!s_hostLiteral && !s_mqttUp) {   /* link back after a drop — re-check IP */
                IPAddress prev = s_ip;
                if (resolveServer() && s_ip != prev) controlReset();
            }
            controlEnsure();
        }

        /* ~5 s uplink diagnostic — decodes the LCD's Uplink chip: it goes green only when
           s_mqttUp (control MQTT to the local server) is 1. Walks the chain WiFi → resolve
           → broker so you can see exactly where it stops. */
        if (millis() - lastDiag >= 5000) {
            lastDiag = millis();
            Serial.printf("[SYNC] wifi=%u ip=%s | host=%s resolved=%u srv=%s | mqtt(server)=%u ntp=%u\n",
                          (unsigned)wifi, wifi ? WiFi.localIP().toString().c_str() : "-",
                          s_host, (unsigned)s_resolved,
                          s_resolved ? s_ip.toString().c_str() : "-",
                          (unsigned)s_mqttUp, (unsigned)ntpTimeReceived);
        }

        /* Publish decoded result summaries over MQTT — but only once the broker is up, so a
           result waits in the queue (not lost) while offline. */
        if (s_mqttUp) {
            das_result_t res;
            while (xQueueReceive(s_resultQ, &res, 0) == pdTRUE) publish_result(&res);
        }

        /* Drain ALL finished raw sessions (the first receive also paces the loop when idle).
           handle_raw_session always writes the CSV to SD; it uploads promptly when reachable,
           else the file stays pending for the hourly retry. */
        das_raw_t rw;
        for (int drained = 0; drained < DS_RAW_QUEUE_DEPTH; drained++) {
            if (xQueueReceive(s_rawQ, &rw, drained == 0 ? pdMS_TO_TICKS(1000) : 0) != pdTRUE) break;
            handle_raw_session(&rw);
        }

        if (s_syncReqPending) { s_syncReqPending = false; handleSyncReq(s_syncReq); }

        /* presence heartbeat (retained) */
        if (s_mqttUp && millis() - lastPresence >= DS_PRESENCE_MS) {
            lastPresence = millis();
            esp_mqtt_client_publish(s_mqtt, s_tPresence, "online", 6, 0, 1);
        }

        /* on (re)connect, immediately flush anything pending */
        if (s_mqttUp && !wasUp) { retryPendingCsvs(); publishSyncList(); }
        wasUp = s_mqttUp;

        /* hourly retry of anything still pending */
        if (wifi && millis() - lastRetry >= DS_RETRY_INTERVAL_MS) {
            lastRetry = millis();
            retryPendingCsvs();
            publishSyncList();
        }

        /* daily 23:00 raw + event-log job (retried hourly until it completes) */
        uint32_t ymd = today_ymd();
        if (wifi && ymd && timestampData.hour >= DS_SCHED_HOUR && ymd > s_lastDay) {
            if (dailyAttemptMs == 0 || millis() - dailyAttemptMs >= DS_RETRY_INTERVAL_MS) {
                dailyAttemptMs = millis();
                if (dailyJob()) { s_lastDay = ymd; nvs_save_u32("ds_lastday", ymd); }
            }
        }
    }
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  Public API
 * ───────────────────────────────────────────────────────────────────────────── */
void das_sync_init(void)
{
    load_config();

    uint8_t mac[6]; WiFi.macAddress(mac);
    snprintf(s_deviceId, sizeof(s_deviceId), "%s-%02X%02X%02X", s_station, mac[3], mac[4], mac[5]);

    snprintf(s_tPresence,  sizeof(s_tPresence),  "das/%s/%s/presence",   s_station, s_deviceId);
    snprintf(s_tStatusReq, sizeof(s_tStatusReq), "das/%s/%s/status/req", s_station, s_deviceId);
    snprintf(s_tStatusRes, sizeof(s_tStatusRes), "das/%s/%s/status/res", s_station, s_deviceId);
    snprintf(s_tSyncList,  sizeof(s_tSyncList),  "das/%s/%s/sync/list",  s_station, s_deviceId);
    snprintf(s_tSyncReq,   sizeof(s_tSyncReq),   "das/%s/%s/sync/req",   s_station, s_deviceId);
    snprintf(s_tUploadAck, sizeof(s_tUploadAck), "das/%s/%s/upload/ack", s_station, s_deviceId);
    snprintf(s_tResult,    sizeof(s_tResult),    "das/%s/%s/result",     s_station, s_deviceId);

    /* First harness boot: don't daily-sync segments that pre-date this feature. Rotate so any
     * pre-existing (possibly stale) data becomes a FINALIZED segment we mark as already-synced;
     * new frames then land in a fresh segment that the daily job WILL upload. */
    if (s_segHw == SEGHW_UNSET) {
        sd_log_force_rotate();
        uint32_t w = sd_log_write_segment();
        s_segHw = (w > 0) ? (w - 1) : 0;     /* skip everything up to the just-finalized segment */
        nvs_save_u32("ds_seg_hw", s_segHw);
    }

    if (!s_rawQ)    s_rawQ    = xQueueCreate(DS_RAW_QUEUE_DEPTH,    sizeof(das_raw_t));
    if (!s_resultQ) s_resultQ = xQueueCreate(DS_RESULT_QUEUE_DEPTH, sizeof(das_result_t));

    xTaskCreatePinnedToCore(das_sync_task, "DAS_Sync", 10240, NULL, 1, NULL, 0 /*core 0*/);

    ESP_LOGI(DS_TAG, "init: dev=%s host=%s http=%u mqtt=%u station=%s segHw=%u",
             s_deviceId, s_host, (unsigned)s_httpPort, (unsigned)s_mqttPort, s_station, (unsigned)s_segHw);
    EVLOG_INFO("SYNC", "local-server sync init (dev=%s host=%s)", s_deviceId, s_host);
}

bool das_sync_enqueue_result(const das_result_t *r)
{
    if (!s_resultQ || !r) return false;
    if (xQueueSend(s_resultQ, r, 0) != pdTRUE) {
        EVLOG_WARN("SYNC", "result queue full — dropped %s (raw HTTP still has it)", r->barcode);
        return false;
    }
    return true;
}

bool das_sync_enqueue_raw(const char *barcode, char *buf, uint32_t len, bool truncated)
{
    if (!s_rawQ || !buf) return false;
    das_raw_t rw;
    memset(&rw, 0, sizeof(rw));
    strncpy(rw.barcode, barcode ? barcode : "", sizeof(rw.barcode) - 1);
    rw.buf = buf; rw.len = len; rw.truncated = truncated;
    if (xQueueSend(s_rawQ, &rw, 0) != pdTRUE) {
        EVLOG_WARN("SYNC", "raw queue full — dropped %s", rw.barcode);
        return false;   /* caller frees buf */
    }
    return true;
}

bool das_sync_server_up(void)
{
    return s_mqttUp;
}

#else  /* CONFIG_DAS_SYNC == 0 */

void das_sync_init(void) {}
bool das_sync_enqueue_result(const das_result_t *r) { (void)r; return false; }
bool das_sync_enqueue_raw(const char *barcode, char *buf, uint32_t len, bool truncated)
{ (void)barcode; (void)len; (void)truncated; if (buf) free(buf); return false; }
bool das_sync_server_up(void) { return false; }

#endif /* CONFIG_DAS_SYNC */
