/******************************************************************************************************************************
 * @file    ota_bootloader.cpp
 * @brief   OTA Bootloader (factory partition) — CAN self-update flasher + WiFi web updater.
 *
 * ⚠ EC200 REMOVED: the modem-based MQTT/HTTPS remote-download OTA path is compiled out
 *   (see `#define OTA_BL_MODEM_OTA 0` below). Remote update now happens via:
 *     • CAN self-update — app0 sees the 0x0C enter frame → backs up app0→app1 →
 *       reboots into factory → run_can_selfupdate_slave() flashes app0 over CAN.
 *     • Factory-mode WiFi web updater (wifi_service.cpp) — hidden SoftAP + web page.
 *   The CAN-flash / three-partition (factory+app0+app1) scheme is otherwise unchanged.
 ******************************************************************************************************************************/

#include "ota_bootloader.h"
#include <esp_partition.h>
#include <esp_attr.h>          /* RTC_DATA_ATTR */
#include "mqtt.h"
#include "event_log.h"         /* SD event log — works from the factory partition too */
#include "wifi_service.h"      /* factory-mode WiFi service UI + web updater */
#include <esp_heap_caps.h>     /* heap snapshot when the service AP comes up */

/* ════════════════════════════════════════════════════════════════════════════
 *  EC200 / MODEM REMOTE-OTA — REMOVED.
 *
 *  The EC200 LTE modem and its MQTT/HTTPS remote-firmware-download path were
 *  purged from this board (MQTT is WiFi-only now). All modem/MQTT/HTTP/master-
 *  CAN-flash code below is compiled out via `#if OTA_BL_MODEM_OTA` (kept at 0),
 *  so ZERO EC200 code is in the binary — while the CAN self-update slave (0x0C)
 *  and the factory-mode WiFi web updater (the remote-flash path now) are intact.
 *  Do not set this to 1 without re-adding the EC200 hardware + AT-command layer.
 * ════════════════════════════════════════════════════════════════════════════ */
#define OTA_BL_MODEM_OTA   0

// ─────────────────────────────────────────────────────────────────────────────
//  Backup arm/consume sentinel
//
//  The factory bootloader backs up app0→app1, but ONLY when it was entered
//  *from app0*. If the device booted from app1 (recovery fell back because app0
//  was bad) and then re-enters factory, copying the stale/bad app0 over the good
//  app1 backup would brick recovery. The app-side trigger sets this sentinel to
//  the magic iff the running partition is app0; the factory consumes it once.
//
//  RTC_NOINIT_ATTR retains its value across esp_restart() but is NOT
//  re-initialized by startup, so it genuinely survives the reboot into factory.
//  (RTC_DATA_ATTR would be re-initialized on a software reset and thus wiped.)
//  After a real power cycle the RTC RAM holds garbage that won't match the
//  magic → backup safely skipped. factory / app0 / app1 all run the same
//  firmware image, so this variable lives at the same RTC address everywhere.
// ─────────────────────────────────────────────────────────────────────────────
#define BACKUP_SENTINEL_MAGIC   0xB0074A11u
RTC_NOINIT_ATTR static uint32_t s_backup_sentinel;
/* arm_backup_sentinel_from_running() is defined further down, after the DBG
 * logging macros — it sets the sentinel above just before a trigger reboots. */

// ─────────────────────────────────────────────────────────────────────────────
//  Flash-entry-requested sentinel (IOT/LCD parity)
//
//  Set to ENTRY_REQUEST_MAGIC by an app-side trigger (MQTT OR CAN 0x0C) just
//  before rebooting into factory. On factory boot, if this is NOT set we got
//  here spontaneously (power event, ROM fallback after a bad app, or a flash
//  that never committed boot=app0). In that case we do NOT sit waiting for a
//  flasher / modem — we recover straight to a valid app; if none is bootable
//  (bricked) we run the CAN slave so the unit can still be rescued over CAN.
//
//  RTC_NOINIT_ATTR for the same reason as the backup sentinel: survives
//  esp_restart(), garbage after a true power cycle (won't match → recover).
// ─────────────────────────────────────────────────────────────────────────────
#define ENTRY_REQUEST_MAGIC     0xE47A0001u
RTC_NOINIT_ATTR static uint32_t s_flash_entry_requested;

// ─────────────────────────────────────────────────────────────────────────────
//  State
// ─────────────────────────────────────────────────────────────────────────────
static bool      g_canRunning       = false;
static bool      g_uploadInProgress = false;
static uint32_t  g_uploadProgress   = 0;
static String    g_lastError        = "";

// g_fwData removed — firmware is now streamed from SPIFFS, never held fully in RAM
static File      g_fwFile;                         // kept open across the CAN flash session
static bool      g_fwFileOpen       = false;
static size_t    g_fwRawSize        = 0;           // original (unpadded) file size on SPIFFS
static size_t    g_fwSize           = 0;           // padded size sent over CAN
static uint32_t  g_fwCRC            = 0;

static QueueHandle_t g_gsmQueue     = NULL;
static uart_event_t  g_gsmEvent;
static char          g_atBuf[OTA_BL_AT_BUF];
static bool          g_mqttConn     = false;

/* ── MQTT line accumulator (same fragmentation fix as analytics.cpp) ─────────
 * The bootloader UART read uses a 20 ms timeout, so a 250-byte +QMTRECV
 * line arrives as multiple partial reads. We accumulate until '\n'.
 * ─────────────────────────────────────────────────────────────────────────── */
#define BL_MQTT_ACCUM_SIZE 4096
static char g_mqttAccum[BL_MQTT_ACCUM_SIZE];
static int  g_mqttAccumLen = 0;

bool bootloader_ready = false;
const int MAX_BL_ATTEMPTS = 2; // to restart can and bootloader if failed

static void bl_mqtt_accum_reset(void)
{
    memset(g_mqttAccum, 0, sizeof(g_mqttAccum));
    g_mqttAccumLen = 0;
}

static char  g_pendingUrl[OTA_BL_MAX_URL]    = {0};
static char  g_pendingSha[OTA_BL_MAX_SHA256] = {0};
static bool  g_downloadPending  = false;
static char  g_pendingType[16] = {0};    /* "master" or "esp32"          */
static bool  g_esp32OtaRolledBack = false;  /* set when rollback is detected */
static volatile bool g_bootloaderRestart = false;

// ─────────────────────────────────────────────────────────────────────────────
//  RTOS Synchronization (NEW - Task separation)
// ─────────────────────────────────────────────────────────────────────────────
static SemaphoreHandle_t g_downloadReadySem = NULL;  // CAN OTA task waits on this
static SemaphoreHandle_t g_otaMutex = NULL;          // Protect shared state
static volatile bool g_otaInProgress = false;        // Flag: OTA is running
static volatile bool g_pauseMqttPolling = false;     // NEW: Pause MQTT during critical phases

// OTA Status and Cancellation
static volatile bool g_otaCancel = false;           // Cancellation flag (volatile for safety)
static uint32_t g_otaStartTime = 0;                 // OTA sequence start time (ms)
static const uint32_t OTA_MAX_DURATION = 600000;    // 10 minutes max OTA duration (ms)

// ── Idle timeout: return to app0 if no OTA command arrives within 5 minutes ──
// The timer starts once the bootloader announces it is READY and is reset
// whenever a download command is received (i.e. g_downloadPending goes true).
// While an OTA sequence is running (g_otaInProgress) the timer is frozen.
static const uint32_t BL_IDLE_TIMEOUT_MS  = 5UL * 60UL * 1000UL;  // 5 minutes
static uint32_t       g_blIdleStartMs     = 0;   // set in mqtt_poll_task on first iteration after READY

// ── MQTT / internet watchdog ──────────────────────────────────────────────────
// Every BL_MQTT_CHECK_INTERVAL_MS the poll task sends AT+QMTCONN? (or a
// lightweight AT) and counts consecutive failures.  After
// BL_MQTT_MAX_FAILURES failures it calls esp_restart() (hardware watchdog
// resets the modem too) so the device never gets stuck silently.
static const uint32_t BL_MQTT_CHECK_INTERVAL_MS = 30000;  // check every 30 s
static const uint8_t  BL_MQTT_MAX_FAILURES       = 3;     // 3 × 30 s = 90 s max outage before reboot
static uint32_t       g_lastMqttCheckMs          = 0;
static uint8_t        g_mqttFailCount            = 0;

// OTA Status States Enum
typedef enum {
    OTA_STATE_READY,
    OTA_STATE_IDLE,
    OTA_STATE_INITIALIZING,
    OTA_STATE_DOWNLOADING,
    OTA_STATE_VERIFYING,
    OTA_STATE_FLASHING,
    OTA_STATE_SUCCESS,
    OTA_STATE_FAILED,
    OTA_STATE_CANCELLED
} OtaState_t;

static OtaState_t g_otaState = OTA_STATE_IDLE;
static uint8_t g_dlChunk[OTA_BL_DL_CHUNK];

// ─────────────────────────────────────────────────────────────────────────────
//  MQTT Status Codes (NEW)
// ─────────────────────────────────────────────────────────────────────────────
#define STATUS_BOOTLOADER_READY         100  // Bootloader started, waiting for command
#define STATUS_MQTT_CONNECTED           101  // MQTT connection established
#define STATUS_MQTT_SUBSCRIBED          102  // Subscribed to OTA topic
#define STATUS_DOWNLOAD_QUEUED          103  // Download URL received
#define STATUS_OTA_STARTING             110  // OTA sequence initiated
 
#define STATUS_DOWNLOAD_STARTED         200  // Download in progress
#define STATUS_DOWNLOAD_PROGRESS        201  // Download progress update
#define STATUS_DOWNLOAD_COMPLETE        202  // Download finished
 
#define STATUS_VERIFY_STARTED           300  // SHA256 verification started
#define STATUS_VERIFY_SUCCESS           301  // SHA256 match confirmed
 
#define STATUS_FLASH_STARTED            400  // CAN flash started
#define STATUS_FLASH_MCU_UNLOCK         401  // Remote MCU unlocked
#define STATUS_FLASH_DATA_SENDING       402  // Sending data packets
#define STATUS_FLASH_VERIFY_OK          403  // CAN flash verify OK
#define STATUS_FLASH_RESET_MCU          404  // Resetting remote MCU
 
#define STATUS_OTA_SUCCESS              500  // Complete success
#define STATUS_OTA_CANCELLED            501  // User cancelled
 
#define STATUS_ERROR_MQTT_CONN          600  // MQTT connection failed
#define STATUS_ERROR_MQTT_LOST          601  // MQTT connection lost during OTA
#define STATUS_ERROR_CAN_INIT           602  // CAN initialization failed
#define STATUS_ERROR_DOWNLOAD           603  // Download failed
#define STATUS_ERROR_SHA256             604  // SHA256 mismatch
#define STATUS_ERROR_FW_PROCESS         605  // Firmware processing error
#define STATUS_ERROR_MCU_NO_RESPONSE    606  // Remote MCU not responding
#define STATUS_ERROR_MCU_BOOTLOADER     607  // Remote MCU bootloader error
#define STATUS_ERROR_TIMEOUT            608  // OTA timeout exceeded
#define STATUS_ERROR_CANCELLED          609  // Operation cancelled

// ─────────────────────────────────────────────────────────────────────────────
//  Unified debug macro — prints to Serial AND ESP_LOG simultaneously
// ─────────────────────────────────────────────────────────────────────────────
#define DBG(tag, fmt, ...)      do { Serial.printf("[" tag "] " fmt "\n", ##__VA_ARGS__);      ESP_LOGI(tag,  fmt, ##__VA_ARGS__); } while(0)
#define DBG_WARN(tag, fmt, ...) do { Serial.printf("[" tag "][WARN] " fmt "\n", ##__VA_ARGS__); ESP_LOGW(tag, fmt, ##__VA_ARGS__); } while(0)
#define DBG_ERR(tag, fmt, ...)  do { Serial.printf("[" tag "][ERR]  " fmt "\n", ##__VA_ARGS__); ESP_LOGE(tag, fmt, ##__VA_ARGS__); } while(0)
#define DIVIDER(tag)            Serial.printf("[" tag "] ──────────────────────────────────────────────\n")

// ─────────────────────────────────────────────────────────────────────────────
//  Forward declarations
// ─────────────────────────────────────────────────────────────────────────────
static uint32_t crc32_calc(const uint8_t *d, size_t len);
static bool     can_init_bl(void);
static bool     can_send(uint32_t id, const uint8_t *data, uint8_t len, bool ext = false);
static bool     can_recv(twai_message_t &msg, uint32_t timeout_ms);
static bool     bl_wait_resp(uint8_t expected, uint32_t timeout_ms);
static bool     bl_heartbeat(void);
static bool     bl_init(void);
static bool     bl_send_info(size_t sz);
static bool     bl_send_data(const uint8_t *data, size_t sz);
static bool     bl_verify(uint32_t crc);
static bool     bl_reset(void);
static bool     fw_process(const char *path);
static bool     fw_read_chunk(size_t offset, uint8_t *buf, size_t len);
static void     fw_close(void);
static bool     fw_read_chunk(size_t offset, uint8_t *buf, size_t len);
static void     fw_close(void);
static void     gsm_send(const char *cmd, uint32_t delayMs = 100);
static bool     gsm_wait(const char *expected, uint32_t timeoutMs = 10000);
static bool     gsm_scan_marker(const char *marker, uint32_t timeoutMs,
                                uint8_t *overflow, size_t ovSize, int *ovLen);
static void     modem_uart_init(void);
static void     modem_power_on(void);
static bool     modem_wait_ready(void);
static bool     net_wait_registered(uint32_t timeoutMs = 60000);
static bool     modem_bringup(void);
static bool     mqtt_connect_bl(void);
static void     mqtt_subscribe_bl(void);
static void     mqtt_poll_bl(void);
static void     mqtt_process_qmtrecv(char *line);
static bool     http_download(const char *url, const char *savePath, int resumeFrom = 0);
static bool     http_download_with_retry(const char *url, const char *savePath);
static bool     sha256_verify(const char *path, const char *expectedHex);
static void     do_can_ota_internal(void);  // Renamed: called from can_ota_task
static void     handle_serial_cmd(char cmd);
static void     print_status(void);
static void     bail_to_best_app(const char *reason);
static bool     recover_boot_to_app(const char *reason);   /* try app0, else app1; false if neither bootable */
static void     run_can_selfupdate_slave(void);            /* CAN flash slave (never returns) */
static bool     mqtt_is_alive(void);
/* RTOS task entry points (NEW) */
static void     mqtt_poll_task(void *pvParam);
static void     can_ota_task(void *pvParam);
static void     serial_cmd_task(void *pvParam);
static void     mqtt_publish_status(uint16_t statusCode, const char *detail, uint8_t progress);


// ─────────────────────────────────────────────────────────────────────────────
//  Back up app0 → app1 so app1 always holds the firmware that was running when
//  the bootloader was entered.
//
//  Runs in the FACTORY bootloader (early in ota_bootloader_run()), BEFORE the
//  CAN slave's UNLOCK or the download path erases app0. Two consequences:
//    • app0 MUST be read explicitly via OTA_0 — NOT esp_ota_get_running_partition()
//      (in factory the running partition is `factory`, so a subtype==OTA_0 guard
//      would skip the copy entirely).
//    • A failed backup is safe: app0 has not been touched yet, so it is still
//      intact and bootable.
// ─────────────────────────────────────────────────────────────────────────────
static bool backup_app0_to_app1(void)
{
    const esp_partition_t *src = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);   // app0 (explicit)
    const esp_partition_t *dst = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);   // app1
    if (!src || !dst) { DBG_ERR("OTA_BKP", "app0/app1 partition not found"); return false; }

    /* Only back up a valid app0 image (first byte == ESP magic 0xE9). If app0
     * is blank/half-written there is nothing good to preserve — leave app1 as-is. */
    uint8_t magic = 0;
    if (esp_partition_read(src, 0, &magic, 1) != ESP_OK || magic != 0xE9) {
        DBG_WARN("OTA_BKP", "app0 has no valid image (magic=0x%02X) — skipping backup", magic);
        return false;
    }

    DBG("OTA_BKP", "Backing up app0 → app1 (%u bytes)...", (unsigned)src->size);

    esp_ota_handle_t h;
    if (esp_ota_begin(dst, OTA_SIZE_UNKNOWN, &h) != ESP_OK) {
        DBG_ERR("OTA_BKP", "esp_ota_begin(app1) failed"); return false;
    }

    static uint8_t buf[4096];
    size_t   done     = 0;
    uint32_t blocks   = 0;
    int      last_pct = -1;
    bool     ok       = true;
    while (done < src->size) {
        size_t n = src->size - done;
        if (n > sizeof(buf)) n = sizeof(buf);
        if (esp_partition_read(src, done, buf, n) != ESP_OK) { ok = false; break; }
        if (esp_ota_write(h, buf, n)              != ESP_OK) { ok = false; break; }
        done += n;

        if (++blocks % 16 == 0) vTaskDelay(1);   /* yield ~every 64 KB */

        int pct = (int)(done * 100 / src->size);
        if (pct != last_pct && (pct % 10 == 0)) {
            last_pct = pct;
            DBG("OTA_BKP", "  backup %d%% (%u/%u)", pct, (unsigned)done, (unsigned)src->size);
        }
    }

    if (!ok) {
        DBG_ERR("OTA_BKP", "Backup read/write FAILED at offset %u", (unsigned)done);
        esp_ota_abort(h);
        return false;
    }
    if (esp_ota_end(h) != ESP_OK) {          /* validates the copied image */
        DBG_ERR("OTA_BKP", "esp_ota_end(app1) failed — backup image invalid");
        return false;
    }

    DBG("OTA_BKP", "✅ Backup complete — app1 holds a copy of app0 (%u bytes)", (unsigned)done);
    return true;
}
// ─────────────────────────────────────────────────────────────────────────────
//  Called by the app-side triggers just before switching boot to factory and
//  rebooting. Records TWO RTC_NOINIT sentinels:
//    • s_flash_entry_requested — ALWAYS set: "a flash was deliberately requested",
//      so factory knows this is not a spontaneous boot.
//    • s_backup_sentinel — set ONLY when running from app0, so factory backs up
//      app0→app1 (and never clobbers a good app1 when we came up on app1).
//  The factory bootloader consumes both once.
// ─────────────────────────────────────────────────────────────────────────────
static void arm_factory_entry_from_running(void)
{
    s_flash_entry_requested = ENTRY_REQUEST_MAGIC;   /* deliberate entry */

    const esp_partition_t *r = esp_ota_get_running_partition();
    if (r && r->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0) {
        s_backup_sentinel = BACKUP_SENTINEL_MAGIC;
        DBG("OTA_TRIG", "Running from app0 — app1 backup ARMED for factory");
    } else {
        s_backup_sentinel = 0;
        DBG("OTA_TRIG", "Not running from app0 (running=%s) — app1 backup DISARMED",
            r ? r->label : "?");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Public: trigger reboot into app1 (called from analytics.cpp in main app)
// ─────────────────────────────────────────────────────────────────────────────
void ota_bootloader_trigger_reboot(void)
{
    DIVIDER("OTA_TRIG");
    DBG("OTA_TRIG", "╔══════════════════════════════════════════════╗");
    DBG("OTA_TRIG", "║   OTA UPDATE COMMAND RECEIVED                ║");
    DBG("OTA_TRIG", "╚══════════════════════════════════════════════╝");

    /* NOTE: the app0→app1 backup is NOT done here anymore. It is performed in
     * the factory bootloader (backup_app0_to_app1() early in ota_bootloader_run)
     * BEFORE app0 is erased — this app-side trigger just switches boot to
     * factory and reboots. */
    DBG("OTA_TRIG", "Finding factory partition...");

    const esp_partition_t *factory = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);

    if (!factory) {
        DBG_ERR("OTA_TRIG", "FATAL: factory partition NOT FOUND in partition table!");
        DBG_ERR("OTA_TRIG", "Check partitions.csv is flashed and board_build.flash_size=8MB");
        DBG_ERR("OTA_TRIG", "Aborting — remaining in app0.");
        return;
    }

    DBG("OTA_TRIG", "Found factory: label=%s subtype=0x%02X offset=0x%08X size=%u",
        factory->label, factory->subtype, factory->address, factory->size);

    /* Record the deliberate entry + arm the backup only if running from app0. */
    arm_factory_entry_from_running();

    DBG("OTA_TRIG", "Calling esp_ota_set_boot_partition(factory)...");
    esp_err_t err = esp_ota_set_boot_partition(factory);
    if (err != ESP_OK) {
        DBG_ERR("OTA_TRIG", "esp_ota_set_boot_partition(factory) FAILED: %s", esp_err_to_name(err));
        return;
    }

    DBG("OTA_TRIG", "Boot partition set to factory.");
    DBG("OTA_TRIG", "  factory will download + flash new firmware to app0");
    DBG("OTA_TRIG", "  app1 holds a backup of the current app0 as a safety net");

    /* EC200 MQTT "switching to bootloader" publish removed (modem purged). */
    EVLOG_INFO("OTA", "switching to factory bootloader (rebooting)");

    DBG("OTA_TRIG", "Restarting in 2 s...");
    Serial.flush();
    delay(2000);
    esp_restart();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Public: enter CAN self-update bootloader (called from app0's CAN RX path on
//  the dedicated enter frame, ID 0x0C). Same app0→app1 backup as OTA, then boot
//  factory in CAN-slave mode (modem/MQTT skipped). Publishes no MQTT.
// ─────────────────────────────────────────────────────────────────────────────
void ota_bootloader_trigger_reboot_can(void)
{
    DIVIDER("OTA_TRIG");
    DBG("OTA_TRIG", "╔══════════════════════════════════════════════╗");
    DBG("OTA_TRIG", "║   CAN SELF-UPDATE ENTER (ID 0x0C) RECEIVED   ║");
    DBG("OTA_TRIG", "╚══════════════════════════════════════════════╝");

    /* NOTE: the app0→app1 backup is NOT done here. The factory bootloader does
     * it (backup_app0_to_app1() early in ota_bootloader_run) BEFORE the CAN
     * slave erases app0. This keeps the app-side trigger instant. */

    /* ── Set the CAN-mode NVS flag so factory enters slave mode ── */
    {
        esp_err_t nvsErr = nvs_flash_init();
        if (nvsErr == ESP_ERR_NVS_NO_FREE_PAGES || nvsErr == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            nvs_flash_erase(); nvs_flash_init();
        }
        nvs_handle_t nvs;
        if (nvs_open(OTA_BL_NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
            nvs_set_str(nvs, OTA_BL_NVS_KEY_CANMODE, OTA_BL_NVS_VAL_PENDING);
            nvs_commit(nvs);
            nvs_close(nvs);
            DBG("OTA_TRIG", "NVS flag set: %s = %s", OTA_BL_NVS_KEY_CANMODE, OTA_BL_NVS_VAL_PENDING);
        } else {
            DBG_ERR("OTA_TRIG", "Could not set CAN-mode NVS flag — aborting");
            return;
        }
    }

    /* ── Step 3: Boot into factory (CAN slave lives there) ── */
    const esp_partition_t *factory = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
    if (!factory) {
        DBG_ERR("OTA_TRIG", "FATAL: factory partition NOT FOUND — aborting CAN enter");
        return;
    }

    /* Record the deliberate entry + arm the backup only if running from app0. */
    arm_factory_entry_from_running();

    if (esp_ota_set_boot_partition(factory) != ESP_OK) {
        DBG_ERR("OTA_TRIG", "esp_ota_set_boot_partition(factory) FAILED");
        return;
    }

    DBG("OTA_TRIG", "Boot partition set to factory — CAN self-update mode.");
    DBG("OTA_TRIG", "Restarting in 1 s...");
    Serial.flush();
    delay(1000);
    esp_restart();
}

// ─────────────────────────────────────────────────────────────────────────────
//  CRC-32
// ─────────────────────────────────────────────────────────────────────────────
static uint32_t crc32_calc(const uint8_t *d, size_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    const uint32_t poly = 0xEDB88320;
    for (size_t i = 0; i < len; i++) {
        crc ^= d[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 1) ? (crc >> 1) ^ poly : (crc >> 1);
    }
    return crc ^ 0xFFFFFFFF;
}

// ─────────────────────────────────────────────────────────────────────────────
//  CRC-32 incremental  (feed chunks; start crc=0xFFFFFFFF, finalise ^0xFFFFFFFF)
// ─────────────────────────────────────────────────────────────────────────────
static uint32_t crc32_update(uint32_t crc, const uint8_t *d, size_t len)
{
    const uint32_t poly = 0xEDB88320;
    for (size_t i = 0; i < len; i++) {
        crc ^= d[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 1) ? (crc >> 1) ^ poly : (crc >> 1);
    }
    return crc;
}

// ─────────────────────────────────────────────────────────────────────────────
//  CAN helpers
// ─────────────────────────────────────────────────────────────────────────────
/* Defined in can.cpp — resolves the NVS-persisted CAN baud ("iot_cfg"/"can_baud",
 * default CONFIG_CAN_BAUD_KBPS) into a TWAI timing. The factory flasher uses the
 * SAME source of truth as the app, so a device retuned at runtime is flashed at
 * its actual bus baud (app + factory + bus always agree). */
extern twai_timing_config_t can_resolve_baud_timing(uint32_t *kbps_out);

static bool can_init_bl(void)
{
    uint32_t kbps = 0;
    twai_timing_config_t t_config = can_resolve_baud_timing(&kbps);

    DBG("CAN_BL", "Initializing CAN at %u kbps (factory flash session)...", (unsigned)kbps);
    DBG("CAN_BL", "  TX = GPIO%d  RX = GPIO%d", (int)OTA_BL_CAN_TX, (int)OTA_BL_CAN_RX);

    /* Clean up any leftover driver (main app leaves TWAI running on reboot) */
    twai_stop();
    twai_driver_uninstall();
    vTaskDelay(pdMS_TO_TICKS(100));

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        OTA_BL_CAN_TX, OTA_BL_CAN_RX, TWAI_MODE_NORMAL);
    /* Deep RX queue: harmless for today's stop-and-wait host, but lets the
     * device absorb bursts if the flasher is later changed to pipeline DATA
     * frames (send-ahead) instead of waiting for every ACK. IDF default is 5. */
    g_config.rx_queue_len = 64;
    g_config.tx_queue_len = 16;
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        DBG_ERR("CAN_BL", "twai_driver_install FAILED");
        return false;
    }
    if (twai_start() != ESP_OK) {
        DBG_ERR("CAN_BL", "twai_start FAILED");
        twai_driver_uninstall();
        return false;
    }

        // NEW: Check CAN controller state
    twai_status_info_t status;
    twai_get_status_info(&status);
    DBG("CAN_BL", "CAN status: state=%d, tx_err=%d, rx_err=%d",
        status.state, status.tx_error_counter, status.rx_error_counter);
    if (status.state == TWAI_STATE_BUS_OFF) {
        DBG_WARN("CAN_BL", "CAN is bus-off – initiating recovery");
        twai_initiate_recovery();
        vTaskDelay(pdMS_TO_TICKS(200));
        twai_get_status_info(&status);
        DBG("CAN_BL", "After recovery: state=%d", status.state);
    }
    g_canRunning = true;

    /* Let transceiver settle before first TX */
    vTaskDelay(pdMS_TO_TICKS(200));
    return true;
}

static bool can_send(uint32_t id, const uint8_t *data, uint8_t len, bool ext)
{
    if (!g_canRunning) {
        DBG_ERR("CAN_BL", "CAN not running");
        return false;
    }
    twai_message_t message;
    message.extd             = ext;
    message.identifier       = id;
    message.data_length_code = len;
    message.rtr              = false;
    message.flags            = 0;
    if (len > 0 && data) memcpy(message.data, data, len);

    esp_err_t res = twai_transmit(&message, pdMS_TO_TICKS(100));
    if (res == ESP_OK) {
        Serial.printf("[CAN_BL] Sent: ID=0x%08X, Len=%d, Data=", id, len);
        for (int i = 0; i < len; i++) Serial.printf("%02X ", data[i]);
        Serial.println();
        return true;
    }
    DBG_ERR("CAN_BL", "Transmit failed: %s (0x%X)", esp_err_to_name(res), res);
    return false;
}

static bool can_recv(twai_message_t &msg, uint32_t timeout_ms)
{
    /* Exact translation of receiveCANMessage() from combined_ota.ino */
    if (!g_canRunning) return false;
    if (twai_receive(&msg, pdMS_TO_TICKS(timeout_ms)) == ESP_OK) {
        Serial.printf("[CAN_BL] Received: ID=0x%08X, Len=%d, Data=",
                      msg.identifier, msg.data_length_code);
        for (int i = 0; i < msg.data_length_code; i++)
            Serial.printf("%02X ", msg.data[i]);
        Serial.println();
        return true;
    }
    return false;
}

#if OTA_BL_MODEM_OTA   /* EC200 remote-OTA removed — compiled out (see run() + top) */
static bool bl_wait_resp(uint8_t expectedCmd, uint32_t timeout_ms)
{
    /* Exact translation of waitForResponse() from combined_ota.ino */
    twai_message_t msg;
    unsigned long startTime = millis();
    while (millis() - startTime < timeout_ms) {
        if (can_recv(msg, 10)) {
            if (msg.identifier == OTA_BL_CAN_ID && msg.data_length_code > 0) {
                uint8_t rc = msg.data[0];
                if (rc == OTA_BL_RESP_ERROR    ||
                    rc == OTA_BL_RESP_CRC_FAIL  ||
                    rc == OTA_BL_RESP_SEQ_ERROR ||
                    rc == OTA_BL_RESP_INVALID) return false;
                if (rc == expectedCmd) return true;
            }
        }
    }
    g_lastError = "Timeout waiting for response";
    return false;
}

static bool bl_heartbeat(void)
{
    /* Exact translation of checkHeartbeat() from combined_ota.ino */
    uint8_t expectedPattern[] = {0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18};
    twai_message_t msg;
    unsigned long start = millis();
    while (millis() - start < 2000) {
        if (can_recv(msg, 10)) {
            if (msg.identifier == OTA_BL_CAN_HEART_ID &&
                msg.data_length_code >= 8) {
                bool match = true;
                for (int i = 0; i < 8; i++)
                    if (msg.data[i] != expectedPattern[i]) { match = false; break; }
                if (match) {
                    DBG("BL_PROTO", "Valid heartbeat");
                    return true;
                }
            }
        }
    }
    return false;
}

static bool bl_heartbeat_wait(uint32_t timeout_ms)
{
    /* Like bl_heartbeat() but with a configurable timeout */
    uint8_t expectedPattern[] = {0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18};
    twai_message_t msg;
    unsigned long start = millis();
    while (millis() - start < timeout_ms) {
        if (can_recv(msg, 10)) {
            if (msg.identifier == OTA_BL_CAN_HEART_ID &&
                msg.data_length_code >= 8) {
                bool match = true;
                for (int i = 0; i < 8; i++)
                    if (msg.data[i] != expectedPattern[i]) { match = false; break; }
                if (match) {
                    DBG("BL_PROTO", "Valid heartbeat received");
                    return true;
                }
            }
        }
    }
    return false;
}

static bool bl_init(void)
{
    DBG("BL_PROTO", "=== Bootloader Initialization ===");

    uint8_t echoData[] = {OTA_BL_ECHO_CMD};

    // ---- Step 1: Fast path - MCU already in bootloader ----
    DBG("BL_PROTO", "1. Fast echo handshake...");
    if (can_send(OTA_BL_CAN_ID, echoData, 1)) {
        if (bl_wait_resp(OTA_BL_ECHO_CMD, 1000)) {
            DBG("BL_PROTO", "Echo OK - MCU already in bootloader mode");
            return true;
        }
    }
    DBG_WARN("BL_PROTO", "Fast echo failed - sending trigger");

    // ---- Step 2: Trigger sequence ----
    //
    // Root cause of original failure (confirmed from v1p6_250.py):
    //   After trigger the MCU sends heartbeat on 0x456, NOT 0x50 on 0x0B.
    //   Old code waited for 0x50 -> always timed out.
    //
    // Correct flow matching Python reference:
    //   1. Send trigger (standard frame)
    //   2. Wait for heartbeat on 0x456 (MCU confirmation of mode switch)
    //   3. Wait 500 ms  (matches Python sleep(0.5))
    //   4. Send echo 0xB1 - MCU in bootloader responds with 0xB1
    //   5. Require 0xB1 - the only hard gate
    //
    const uint8_t MAX_TRIGGER_ATTEMPTS = 3;
    for (int attempt = 1; attempt <= MAX_TRIGGER_ATTEMPTS; attempt++) {
        DBG("BL_PROTO", "Trigger attempt %d/%d...", attempt, MAX_TRIGGER_ATTEMPTS);

        uint8_t triggerData[] = {
            OTA_BL_CMD_TRIGGER, OTA_BL_CMD_TRIGGER_INV,
            OTA_BL_CMD_TRIGGER, OTA_BL_CMD_TRIGGER_INV,
            OTA_BL_CMD_TRIGGER, OTA_BL_CMD_TRIGGER_INV,
            OTA_BL_CMD_TRIGGER, OTA_BL_CMD_TRIGGER_INV
        };
        if (!can_send(OTA_BL_CAN_ID, triggerData, 8)) {
            DBG_ERR("BL_PROTO", "Trigger send failed");
            continue;
        }

        // Wait for heartbeat on 0x456 - MCU signal that it entered bootloader
        DBG("BL_PROTO", "Waiting for heartbeat on 0x456 (6 s)...");
        bool hb = bl_heartbeat_wait(6000);
        if (hb) {
            DBG("BL_PROTO", "Heartbeat OK - MCU in bootloader mode");
        } else {
            DBG_WARN("BL_PROTO", "No heartbeat - trying echo anyway");
        }

        // Match Python sleep(0.5) before echo
        vTaskDelay(pdMS_TO_TICKS(500));

        DBG("BL_PROTO", "Sending echo (0xB1)...");
        if (!can_send(OTA_BL_CAN_ID, echoData, 1)) {
            DBG_ERR("BL_PROTO", "Echo send failed");
            continue;
        }
        if (bl_wait_resp(OTA_BL_ECHO_CMD, 2000)) {
            DBG("BL_PROTO", "Bootloader initialized successfully!");
            mqtt_publish_status(STATUS_FLASH_MCU_UNLOCK, "Remote MCU unlocked", 95);
            return true;
        }
        DBG_WARN("BL_PROTO", "Echo response missing after attempt %d", attempt);
    }

    DBG_ERR("BL_PROTO", "All trigger attempts failed");
    return false;
}

static bool bl_send_info(size_t sz)
{
    /* Exact translation of sendFlashInfo() from combined_ota.ino */
    uint8_t data[8] = {
        OTA_BL_CMD_UNLOCK, 0x00, OTA_BL_HEADER_MAGIC, 0x04,
        (uint8_t)(sz),       (uint8_t)(sz >> 8),
        (uint8_t)(sz >> 16), (uint8_t)(sz >> 24)
    };
    if (!can_send(OTA_BL_CAN_ID, data, 8)) {
        g_lastError = "Failed to send flash info";
        return false;
    }
    return bl_wait_resp(OTA_BL_RESP_OK, 3000);

}

static bool bl_send_data(const uint8_t *data, size_t dataSize)
{
    const size_t BL_CHUNK = 4;
    uint32_t sequence = 0;
    size_t bytesSent = 0;

    while (bytesSent < dataSize) {
        size_t chunkSize = min(BL_CHUNK, dataSize - bytesSent);
        uint8_t packet[8] = {0};
        packet[0] = OTA_BL_CMD_DATA;
        packet[1] = (uint8_t)(sequence & 0xFF);
        packet[2] = OTA_BL_HEADER_MAGIC;
        packet[3] = (uint8_t)chunkSize;
        memcpy(packet + 4, data + bytesSent, chunkSize);

        int retries = 10;
        bool success = false;

        while (retries > 0 && !success) {
            /* Send the packet */
            if (!can_send(OTA_BL_CAN_ID, packet, 4 + chunkSize)) {
                retries--;
                //delay(50); faster upload
                continue;
            }

            /* Wait up to 200ms for response - collect ALL messages in window */
            uint32_t startTime = millis();
            while (millis() - startTime < 200) { // changed from 500
                twai_message_t msg;
                if (can_recv(msg, 10)) {
                    /* Ignore heartbeats and other CAN messages, look for bootloader response */
                    if (msg.identifier == OTA_BL_CAN_ID && msg.data_length_code > 0) {
                        uint8_t responseCmd = msg.data[0];
                        
                        if (responseCmd == OTA_BL_RESP_OK) {
                            success = true;
                            break;
                        } else if (responseCmd == OTA_BL_RESP_ERROR) {
                            g_lastError = "Device error response";
                            return false;
                        } else if (responseCmd == OTA_BL_RESP_SEQ_ERROR) {
                            /* SEQ_ERROR: reset to 1 and wait for retry response in same window */
                            sequence = 1;
                            packet[1] = (uint8_t)(sequence & 0xFF);
                            /* continue waiting in same 200ms window for new response */
                            continue;
                        } else if (responseCmd == OTA_BL_RESP_CRC_FAIL) {
                            g_lastError = "CRC check failed";
                            return false;
                        } else if (responseCmd == OTA_BL_RESP_INVALID) {
                            g_lastError = "Invalid command";
                            return false;
                        }
                    }
                    /* Ignore other CAN messages (heartbeats, BMS data, etc) */
                }
            }

            if (!success) {
                /* No valid response after 200ms */
                retries--;
                //delay(50); for faster upload
            }
        }

        if (!success) {
            g_lastError = "Packet send failed";
            return false;
        }

        /* Move to next packet */
        bytesSent += chunkSize;
        sequence = (sequence + 1) % 256;
        g_uploadProgress = 20 + (bytesSent * 70 / dataSize);
        if (bytesSent % 256 == 0)
            Serial.printf("[BL_DATA] Flash progress: %u/%u bytes (%u%%)\n",
                          (unsigned)bytesSent, (unsigned)dataSize, g_uploadProgress);
        //delay(10); for faster upload
    }

    DBG("BL_DATA", "All data sent successfully");
    return true;
}

static bool bl_verify(uint32_t crc)
{
    /* Exact translation of verifyData() from combined_ota.ino */
    uint8_t data[8] = {
        OTA_BL_CMD_VERIFY, 0x00, OTA_BL_HEADER_MAGIC, 0x04,
        (uint8_t)(crc),       (uint8_t)(crc >> 8),
        (uint8_t)(crc >> 16), (uint8_t)(crc >> 24)
    };
    if (!can_send(OTA_BL_CAN_ID, data, 8)) {
        g_lastError = "Failed to send verify";
        return false;
    }
    return bl_wait_resp(OTA_BL_RESP_CRC_OK, 3000);
}

static bool bl_reset(void)
{
    /* Exact translation of sendReset() from combined_ota.ino */
    uint8_t data[8] = {OTA_BL_CMD_RESET, 0x00, OTA_BL_HEADER_MAGIC, 0x00, 0,0,0,0};
    if (!can_send(OTA_BL_CAN_ID, data, 8)) {
        g_lastError = "Failed to send reset";
        return false;
    }
    delay(200);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  fw_read_chunk — read `len` bytes starting at `offset` from the open
//  g_fwFile.  Bytes beyond the raw file size are returned as 0xFF (padding).
//  Replaces direct g_fwData pointer arithmetic in the CAN flash loop.
// ─────────────────────────────────────────────────────────────────────────────
static bool fw_read_chunk(size_t offset, uint8_t *buf, size_t len)
{
    if (!g_fwFileOpen || !g_fwFile) {
        DBG_ERR("FW", "fw_read_chunk: file not open");
        return false;
    }

    size_t fromFile = 0;
    size_t fromPad  = 0;

    if (offset < g_fwRawSize) {
        fromFile = ((offset + len) <= g_fwRawSize) ? len : (g_fwRawSize - offset);
    }
    fromPad = len - fromFile;

    if (fromFile > 0) {
        if (!g_fwFile.seek(offset)) {
            DBG_ERR("FW", "fw_read_chunk: seek(%u) failed", (unsigned)offset);
            return false;
        }
        if (g_fwFile.read(buf, fromFile) != fromFile) {
            DBG_ERR("FW", "fw_read_chunk: read error at offset %u", (unsigned)offset);
            return false;
        }
    }
    if (fromPad > 0) {
        memset(buf + fromFile, 0xFF, fromPad);
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  fw_close — release the open SPIFFS file handle after the CAN flash loop.
// ─────────────────────────────────────────────────────────────────────────────
static void fw_close(void)
{
    if (g_fwFileOpen) {
        g_fwFile.close();
        g_fwFileOpen = false;
        DBG("FW", "Firmware file handle closed");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Streaming firmware helper — scratch-buffer size for CRC pass and CAN chunks.
//  Must fit comfortably in free heap (~160 KB typical in bootloader).
// ─────────────────────────────────────────────────────────────────────────────
#define FW_STREAM_CHUNK  4096u   // 4 KB — negligible heap cost, works for any FW size

// ─────────────────────────────────────────────────────────────────────────────
//  fw_process — streaming rewrite.
//
//  OLD approach: malloc(~180 KB) → fails when free heap < firmware size.
//  NEW approach:
//    • Compute padded size arithmetically (no RAM copy needed).
//    • Stream CRC32 over the file in FW_STREAM_CHUNK-sized pieces (4 KB scratch).
//    • Re-open the file and leave it in g_fwFile for fw_read_chunk() calls
//      made by the CAN flash loop — only one 4 KB chunk lives in RAM at a time.
//
//  Peak heap delta: ~4 KB (scratch chunk only), regardless of firmware size.
// ─────────────────────────────────────────────────────────────────────────────
static bool fw_process(const char *path)
{
    DIVIDER("FW");
    DBG("FW", "═══ Load Firmware (streaming) ═══");
    DBG("FW", "Path       : %s", path);
    DBG("FW", "Free heap  : %u bytes", ESP.getFreeHeap());

    // ── 1. Open and measure ──────────────────────────────────────────────────
    File f = SPIFFS.open(path, "r");
    if (!f) {
        g_lastError = "Cannot open firmware file";
        DBG_ERR("FW", "SPIFFS.open(%s) failed", path);
        return false;
    }

    size_t rawSize = f.size();
    DBG("FW", "Raw file size: %u bytes", (unsigned)rawSize);
    if (rawSize == 0) {
        g_lastError = "Empty firmware file";
        DBG_ERR("FW", "File is 0 bytes!");
        f.close();
        return false;
    }

    // ── 2. Compute padded size (pure arithmetic — no alloc) ──────────────────
    size_t padded = ((rawSize + OTA_BL_ERASE_PAGE - 1) / OTA_BL_ERASE_PAGE)
                    * OTA_BL_ERASE_PAGE;
    DBG("FW", "Erase page   : %u bytes", (unsigned)OTA_BL_ERASE_PAGE);
    DBG("FW", "Padded size  : %u bytes (+%u bytes of 0xFF)",
        (unsigned)padded, (unsigned)(padded - rawSize));

    // ── 3. Allocate ONE small scratch chunk (4 KB) ───────────────────────────
    uint8_t *chunk = (uint8_t*)malloc(FW_STREAM_CHUNK);
    if (!chunk) {
        g_lastError = "Scratch chunk malloc failed";
        DBG_ERR("FW", "malloc(%u) for streaming chunk failed — heap exhausted",
                (unsigned)FW_STREAM_CHUNK);
        f.close();
        return false;
    }
    DBG("FW", "Scratch chunk: %u bytes at 0x%p", (unsigned)FW_STREAM_CHUNK, (void*)chunk);

    // ── 4. Stream CRC32 over real file bytes ─────────────────────────────────
    DBG("FW", "Computing CRC32 (streaming)...");
    uint32_t crc       = 0xFFFFFFFFu;
    size_t   remaining = rawSize;

    while (remaining > 0) {
        size_t toRead = (remaining < FW_STREAM_CHUNK) ? remaining : FW_STREAM_CHUNK;
        size_t got    = f.read(chunk, toRead);
        if (got != toRead) {
            g_lastError = "SPIFFS read error during CRC";
            DBG_ERR("FW", "Read error at offset %u: wanted %u got %u",
                    (unsigned)(rawSize - remaining), (unsigned)toRead, (unsigned)got);
            free(chunk);
            f.close();
            return false;
        }
        crc = crc32_update(crc, chunk, got);
        remaining -= got;
    }

    // ── 5. Fold in padding contribution (0xFF bytes) ─────────────────────────
    if (padded > rawSize) {
        size_t padBytes = padded - rawSize;
        size_t padDone  = 0;
        while (padDone < padBytes) {
            size_t step = padBytes - padDone;
            if (step > FW_STREAM_CHUNK) step = FW_STREAM_CHUNK;
            memset(chunk, 0xFF, step);
            crc = crc32_update(crc, chunk, step);
            padDone += step;
        }
    }

    free(chunk);
    crc ^= 0xFFFFFFFFu;   // finalise CRC32
    f.close();

    // ── 6. Re-open file and store handle for fw_read_chunk() ─────────────────
    g_fwFile = SPIFFS.open(path, "r");
    if (!g_fwFile) {
        g_lastError = "Re-open for CAN flash failed";
        DBG_ERR("FW", "SPIFFS re-open failed after CRC pass");
        return false;
    }

    g_fwFileOpen = true;
    g_fwRawSize  = rawSize;
    g_fwSize     = padded;
    g_fwCRC      = crc;

    DBG("FW", "CRC32        : 0x%08X", g_fwCRC);
    DBG("FW", "Final size   : %u bytes", (unsigned)g_fwSize);
    DBG("FW", "Free heap    : %u bytes", ESP.getFreeHeap());
    DBG("FW", "✅ Firmware ready (streaming mode — no full-image RAM copy)");
    DIVIDER("FW");
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  EC200U modem helpers
// ─────────────────────────────────────────────────────────────────────────────
static void gsm_send(const char *cmd, uint32_t delayMs)
{
    Serial.printf("[MODEM] TX >> %s\n", cmd);
    uart_write_bytes(OTA_BL_GSM_UART, cmd, strlen(cmd));
    uart_write_bytes(OTA_BL_GSM_UART, "\r\n", 2);
    vTaskDelay(pdMS_TO_TICKS(delayMs));
}

static bool gsm_wait(const char *expected, uint32_t timeoutMs)
{
    DBG("MODEM", "Expecting '%s' within %u ms...", expected, timeoutMs);
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeoutMs);
    int reads = 0;

    // Use a local accumulator so that responses split across multiple UART reads
    // (e.g. "+QHTTPGET: 0,20" then "0,130292") are still matched correctly.
    // On a match, copy the full accumulated buffer into g_atBuf so callers that
    // parse g_atBuf (e.g. contentLength extraction) see the complete response.
    char accumBuf[OTA_BL_AT_BUF];
    int  accumLen = 0;
    accumBuf[0] = '\0';

    while (xTaskGetTickCount() < deadline) {
        bool got = false;
        if (xQueueReceive(g_gsmQueue, &g_gsmEvent, pdMS_TO_TICKS(1000)))
            if (g_gsmEvent.type == UART_DATA) got = true;

        int space = (OTA_BL_AT_BUF - 1) - accumLen;
        if (space <= 0) {
            // Accumulator full with no match — slide the last half forward so
            // we keep context but don't overflow.
            int keep = OTA_BL_AT_BUF / 2;
            memmove(accumBuf, accumBuf + (accumLen - keep), keep);
            accumLen = keep;
            accumBuf[accumLen] = '\0';
            space = (OTA_BL_AT_BUF - 1) - accumLen;
        }

        int len = uart_read_bytes(OTA_BL_GSM_UART,
                                  (uint8_t*)(accumBuf + accumLen),
                                  space,
                                  pdMS_TO_TICKS(got ? 300 : 50));
        if (len > 0) {
            reads++;
            accumLen += len;
            accumBuf[accumLen] = '\0';
            Serial.printf("[MODEM] RX << %s\n", accumBuf + (accumLen - len));
            if (strstr(accumBuf, expected)) {
                // Expose full accumulated buffer to callers (e.g. +QHTTPGET parse)
                int copy = (accumLen < OTA_BL_AT_BUF - 1) ? accumLen : OTA_BL_AT_BUF - 1;
                memcpy(g_atBuf, accumBuf, copy);
                g_atBuf[copy] = '\0';
                DBG("MODEM", "✅ Found '%s' (read #%d, accum %d B)", expected, reads, accumLen);
                return true;
            }
        }
    }
    DBG_ERR("MODEM", "TIMEOUT — '%s' not found in %u ms (%d reads)", expected, timeoutMs, reads);
    return false;
}

static bool gsm_scan_marker(const char *marker, uint32_t timeoutMs,
                             uint8_t *overflow, size_t ovSize, int *ovLen)
{
    DBG("MODEM", "Byte-scanning for marker '%s' (%u ms)...", marker, timeoutMs);
    const size_t mlen = strlen(marker);
    size_t matched = 0;
    *ovLen = 0;
    uint32_t bytesSeen = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeoutMs);

    while (xTaskGetTickCount() < deadline) {
        uint8_t b;
        if (uart_read_bytes(OTA_BL_GSM_UART, &b, 1, pdMS_TO_TICKS(100)) != 1) continue;
        bytesSeen++;
        if (b == (uint8_t)marker[matched]) {
            if (++matched == mlen) {
                int n = uart_read_bytes(OTA_BL_GSM_UART, overflow, ovSize, pdMS_TO_TICKS(50));
                *ovLen = (n > 0) ? n : 0;
                DBG("MODEM", "✅ Marker found after %u bytes — overflow: %d bytes",
                    bytesSeen, *ovLen);
                return true;
            }
            } else {
            matched = (b == (uint8_t)marker[0]) ? 1 : 0;
        }
    }
    DBG_ERR("MODEM", "Marker '%s' NOT found after %u bytes (%u ms)", marker, bytesSeen, timeoutMs);
    return false;
}

static void modem_uart_init(void)
{
    if (g_gsmQueue != NULL) { DBG("MODEM", "UART already init — skip"); return; }
    DIVIDER("MODEM");
    DBG("MODEM", "═══ UART Init ═══");
    DBG("MODEM", "  Port  = UART%d", (int)OTA_BL_GSM_UART);
    DBG("MODEM", "  TX    = GPIO%d",  OTA_BL_GSM_TX);
    DBG("MODEM", "  RX    = GPIO%d",  OTA_BL_GSM_RX);
    DBG("MODEM", "  Baud  = %d",      OTA_BL_GSM_BAUD);
    DBG("MODEM", "  RX buf= %d B",    OTA_BL_GSM_RX_BUF);
    DBG("MODEM", "  TX buf= %d B",    OTA_BL_GSM_TX_BUF);

    uart_config_t cfg = {
        .baud_rate = OTA_BL_GSM_BAUD, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(OTA_BL_GSM_UART, &cfg);
    uart_set_pin(OTA_BL_GSM_UART, OTA_BL_GSM_TX, OTA_BL_GSM_RX,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(OTA_BL_GSM_UART, OTA_BL_GSM_RX_BUF, OTA_BL_GSM_TX_BUF,
                        OTA_BL_GSM_EVT_QUE, &g_gsmQueue, 0);
    DBG("MODEM", "✅ UART ready");
    DIVIDER("MODEM");
}

static void modem_power_on(void) {
    pinMode(OTA_BL_EC200_PWR, OUTPUT);
    digitalWrite(OTA_BL_EC200_PWR, HIGH);
    vTaskDelay(pdMS_TO_TICKS(800));
    digitalWrite(OTA_BL_EC200_PWR, LOW);
    vTaskDelay(pdMS_TO_TICKS(10000));  // 10 seconds
}

static bool modem_wait_ready(void)
{
    DIVIDER("MODEM");
    DBG("MODEM", "═══ Modem AT Poll (max 20 attempts) ═══");
    for (int i = 1; i <= 20; i++) {
        DBG("MODEM", "Attempt %d/20 → AT", i);
        gsm_send("AT", 200);
        if (gsm_wait("OK", 1000)) {
            DBG("MODEM", "✅ Modem alive on attempt %d", i);
            DIVIDER("MODEM"); return true;
        }
        DBG_WARN("MODEM", "No response on attempt %d", i);
    }
    DBG_ERR("MODEM", "Modem not responding after 20 attempts");
    DIVIDER("MODEM"); return false;
}

static void modem_cleanup_mqtt(void)
{
    DBG("MODEM", "Cleaning up stale MQTT state from previous app...");
    
    // Disconnect any lingering MQTT connection
    gsm_send("AT+QMTDISC=0", 500);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Close MQTT socket
    gsm_send("AT+QMTCLOSE=0", 500);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Reset MQTT module
    gsm_send("AT+QMTRST", 500);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Deactivate PDP context from previous app
    gsm_send("AT+CGACT=0,1", 500);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    DBG("MODEM", "✅ Modem MQTT state cleared");
}

static bool net_wait_registered(uint32_t timeoutMs)
{
    DIVIDER("MODEM");
    DBG("MODEM", "═══ Network Registration (timeout=%u ms) ═══", timeoutMs);
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeoutMs);
    int attempt = 0;

    while (xTaskGetTickCount() < deadline) {
        attempt++;
        DBG("MODEM", "CREG poll #%d", attempt);
        gsm_send("AT+CREG?", 500);
        if (gsm_wait("+CREG: 0,1", 2000) || gsm_wait("+CREG: 0,5", 2000)) {
            DBG("MODEM", "✅ Network registered on attempt %d", attempt);
            DIVIDER("MODEM"); return true;
        }
        DBG_WARN("MODEM", "Not registered — waiting 2 s...");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    DBG_ERR("MODEM", "Registration TIMEOUT after %d attempts", attempt);
    DIVIDER("MODEM"); return false;
}

static bool modem_bringup(void)
{
    DIVIDER("MODEM");
    DBG("MODEM", "╔══════════════════════════════════════════════╗");
    DBG("MODEM", "║   EC200U Modem Bringup (5 steps)             ║");
    DBG("MODEM", "╚══════════════════════════════════════════════╝");

    DBG("MODEM", "Step 1/5: UART init");
    modem_uart_init();

    DBG("MODEM", "Step 2/5: Power on");
    modem_power_on();

    DBG("MODEM", "Step 3/5: Wait AT ready");
    if (!modem_wait_ready()) { DBG_ERR("MODEM", "Bringup FAILED at step 3"); return false; }

    DBG("MODEM", "Step 4/5: Disable echo ATE0");
    gsm_send("ATE0", 100);
    gsm_wait("OK", 1000);

    DBG("MODEM", "Step 5/5: Network registration");
    if (!net_wait_registered()) { DBG_ERR("MODEM", "Bringup FAILED at step 5"); return false; }

    DBG("MODEM", "Configuring PDP context...");
    gsm_send("AT+CGDCONT=1,\"IP\",\"\"", 200);
    gsm_send("AT+CGACT=1,1", 3000);
    gsm_wait("OK", 5000);

    DBG("MODEM", "✅ Modem bringup COMPLETE — modem is online");
    DIVIDER("MODEM");
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  MQTT via EC200U AT commands
// ─────────────────────────────────────────────────────────────────────────────
/**
 * @brief Connect to MQTT broker and publish connection status
 */
static bool mqtt_connect_bl(void)
{
    gsm_send("AT+QMTDISC=0", 1000);  // Disconnect first
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    char cmd[100];
    sprintf(cmd, "AT+QMTOPEN=0,\"%s\",%d", MQTT_SERVER, MQTT_PORT);
    gsm_send(cmd, 2000);
    if (!gsm_wait("+QMTOPEN: 0,0", 20000)) {
        DBG_ERR("MQTT_BL", "QMTOPEN failed");
        return false;
    }
    DBG("MQTT_BL", "QMTOPEN OK");
    
    char connCmd[512];
    snprintf(connCmd, sizeof(connCmd), 
             "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"", 
             MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);
    
    gsm_send(connCmd, 2000);
    if (!gsm_wait("+QMTCONN: 0,0,0", 20000)) {
        DBG_ERR("MQTT_BL", "QMTCONN failed");
        return false;
    }
    
    g_mqttConn = true;
    DBG("MQTT_BL", "✅ MQTT connected");
    
    // Publish connection status
    mqtt_publish_status(STATUS_MQTT_CONNECTED, "MQTT connected", 0);
    
    return true;
}

/**
 * @brief Subscribe to OTA topics and publish subscription status
 */
static void mqtt_subscribe_bl(void)
{
    DBG("MQTT_BL", "Subscribing to topics...");
    
    // Subscribe to OTA download topic
    char subCmd1[256];
    snprintf(subCmd1, sizeof(subCmd1), 
             "AT+QMTSUB=0,1,\"%s\",0", MQTT_TOPIC_OTA);
    gsm_send(subCmd1, 1000);
    if (gsm_wait("+QMTSUB: 0,1,0", 10000)) {
        DBG("MQTT_BL", "✅ Subscribed: %s", MQTT_TOPIC_OTA);
    } else {
        DBG_ERR("MQTT_BL", "Subscribe failed: %s", MQTT_TOPIC_OTA);
    }
    
    // Subscribe to cancel topic
    char subCmd2[256];
    snprintf(subCmd2, sizeof(subCmd2), 
             "AT+QMTSUB=0,2,\"%s\",0", MQTT_TOPIC_CANCEL);
    gsm_send(subCmd2, 1000);
    if (gsm_wait("+QMTSUB: 0,2,0", 10000)) {
        DBG("MQTT_BL", "✅ Subscribed: %s", MQTT_TOPIC_CANCEL);
    } else {
        DBG_ERR("MQTT_BL", "Subscribe failed: %s", MQTT_TOPIC_CANCEL);
    }
    
    // Publish subscription status
    mqtt_publish_status(STATUS_MQTT_SUBSCRIBED, "Subscribed to OTA topics", 0);
}

static void mqtt_poll_bl(void)
{
        /* ✅ FIX: Check event queue FIRST (same as gsm_wait) */
    if (!xQueueReceive(g_gsmQueue, &g_gsmEvent, pdMS_TO_TICKS(0))) {
        return;  /* No data yet */
    }
    
    /* Only process UART_DATA events */
    if (g_gsmEvent.type != UART_DATA || g_gsmEvent.size <= 0) {
        return;
    }

    /* Read whatever bytes arrived in the last 20 ms */
    int len = uart_read_bytes(OTA_BL_GSM_UART, (uint8_t*)g_atBuf,
                              OTA_BL_AT_BUF - 1, pdMS_TO_TICKS(20));
    if (len <= 0) return;

    g_atBuf[len] = '\0';
    Serial.printf("[MQTT_BL] UART frag (%d B): %s\n", len, g_atBuf);

    /* Feed every byte into the line accumulator */
    for (int i = 0; i < len; i++)
    {
        char c = g_atBuf[i];

        if (g_mqttAccumLen >= BL_MQTT_ACCUM_SIZE - 1) {
            DBG_WARN("MQTT_BL", "Accumulator overflow at %d B — resetting", g_mqttAccumLen);
            bl_mqtt_accum_reset();
        }

        g_mqttAccum[g_mqttAccumLen++] = c;
        g_mqttAccum[g_mqttAccumLen]   = '\0';

        if (c == '\n')
        {
            /* Trim \r\n */
            while (g_mqttAccumLen > 0 &&
                   (g_mqttAccum[g_mqttAccumLen-1] == '\n' ||
                    g_mqttAccum[g_mqttAccumLen-1] == '\r'))
                g_mqttAccum[--g_mqttAccumLen] = '\0';

            if (g_mqttAccumLen > 0) {
                DBG("MQTT_BL", "Line assembled (%d B): %s", g_mqttAccumLen, g_mqttAccum);
                if (strstr(g_mqttAccum, "+QMTRECV:"))
                    mqtt_process_qmtrecv(g_mqttAccum);
            }
            bl_mqtt_accum_reset();
        }
    }
}
/**
 * @brief Publish OTA status update to MQTT broker
 * @param state Current OTA operation state
 * @param detail Human-readable status message
 * @param progress Progress percentage (0-100)
 * 
 * Publishes JSON to topic: coulomb/internal/file/status/{device_id}
 * Format: {"device_id":"...", "state":"...", "detail":"...", "progress":XX}
 */
static void mqtt_publish_ota_status(OtaState_t state, const char *detail, uint8_t progress = 0)
{
    DIVIDER("OTA_STATUS");
    const char *stateStr = "";
    switch(state) {
        case OTA_STATE_READY:         stateStr = "Ready"; break;
        case OTA_STATE_IDLE:         stateStr = "idle"; break;
        case OTA_STATE_INITIALIZING: stateStr = "initializing"; break;
        case OTA_STATE_DOWNLOADING:  stateStr = "downloading"; break;
        case OTA_STATE_VERIFYING:    stateStr = "verifying"; break;
        case OTA_STATE_FLASHING:     stateStr = "flashing"; break;
        case OTA_STATE_SUCCESS:      stateStr = "success"; break;
        case OTA_STATE_FAILED:       stateStr = "failed"; break;
        case OTA_STATE_CANCELLED:    stateStr = "cancelled"; break;
        default:                     stateStr = "unknown"; break;
    }
    char jsonPayload[512];
    snprintf(jsonPayload, sizeof(jsonPayload),
             "{\"device_id\":\"%s\",\"state\":\"%s\",\"detail\":\"%s\",\"progress\":%u}",
             MQTT_CLIENT_ID, stateStr, detail, progress);
    char topic[256];
    strncpy(topic, MQTT_TOPIC_OTA_RESPONSE, sizeof(topic) - 1);
    topic[sizeof(topic) - 1] = '\0';
    char cmd[1024];
    int cmdLen = snprintf(cmd, sizeof(cmd), "AT+QMTPUB=0,1,0,0,\"%s\",%u\r\n", topic, (unsigned)strlen(jsonPayload));
    uart_write_bytes(OTA_BL_GSM_UART, (const uint8_t*)cmd, cmdLen);
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_write_bytes(OTA_BL_GSM_UART, (const uint8_t*)jsonPayload, strlen(jsonPayload));
    if (gsm_wait("+QMTPUB: 0,1,0", 3000)) {
        DBG("OTA_STATUS", "✅ MQTT status published");
    } else {
        DBG_WARN("OTA_STATUS", "MQTT publish ACK timeout");
    }
    DIVIDER("OTA_STATUS");
}


/*
 * @brief Publish MQTT status update (non-blocking)
 * @param statusCode Numeric status code (see STATUS_* defines above)
 * @param detail Human-readable detail message
 * @param progress Progress percentage (0-100)
 */
static void mqtt_publish_status(uint16_t statusCode, const char *detail, uint8_t progress)
{
    /* Event log FIRST — before the connectivity gate below — so every OTA/flash/MQTT status is
       recorded to SD even when the network is down (that's exactly when we most want the trail).
       Failure codes end in ...FAILED/ROLLBACK; treat those as FAIL, everything else as INFO/OK. */
    {
        ev_level_t lvl = EV_INFO;
        if (statusCode == STATUS_ESP32_OTA_FAILED || statusCode == STATUS_ESP32_OTA_ROLLBACK)
            lvl = EV_FAIL;
        else if (statusCode == STATUS_ESP32_OTA_SUCCESS || statusCode == STATUS_ESP32_FLASH_DONE)
            lvl = EV_OK;
        event_logf(lvl, "OTA", "[%u] %s (%u%%)", statusCode, detail ? detail : "", progress);
    }

    if (!g_mqttConn) {
        DBG_WARN("OTA_MQTT", "MQTT not connected - status %u not published", statusCode);
        return;
    }
 
    // Build JSON payload: {"code":100,"msg":"Bootloader ready","progress":0}
    char payload[256];
    int len = snprintf(payload, sizeof(payload),
                      "{\"code\":%u,\"msg\":\"%s\",\"progress\":%u}",
                      statusCode, detail, progress);
    
    if (len < 0 || len >= (int)sizeof(payload)) {
        DBG_ERR("OTA_MQTT", "Payload buffer overflow");
        return;
    }
 
    // Construct AT command: AT+QMTPUB=0,1,0,0,"topic",length
    char cmd[512];
    int cmdLen = snprintf(cmd, sizeof(cmd), 
                         "AT+QMTPUB=0,1,0,0,\"%s\",%u\r\n",
                         MQTT_TOPIC_OTA_RESPONSE, (unsigned)strlen(payload));
    
    if (cmdLen < 0 || cmdLen >= (int)sizeof(cmd)) {
        DBG_ERR("OTA_MQTT", "Command buffer overflow");
        return;
    }
 
    // Send AT command (non-blocking)
    uart_write_bytes(OTA_BL_GSM_UART, (const uint8_t*)cmd, cmdLen);
    vTaskDelay(pdMS_TO_TICKS(50));  // Short delay for modem processing
    
    // Send payload data (non-blocking)
    uart_write_bytes(OTA_BL_GSM_UART, (const uint8_t*)payload, strlen(payload));
    vTaskDelay(pdMS_TO_TICKS(50));  // Short delay for modem processing
    
    DBG("OTA_MQTT", "Published: [%u] %s (%u%%)", statusCode, detail, progress);
}

/**
 * @brief Set OTA state and publish MQTT status
 * @param newState New OTA state
 * @param detail Detail message
 * @param progress Progress percentage (0-100)
 */
static void ota_set_state(OtaState_t newState, const char *detail, uint8_t progress = 0)
{
    g_otaState = newState;
    
    // Log to console
    DBG("OTA_STATE", "[%u%%] %s", progress, detail);
    
    // Determine status code based on state and publish to MQTT
    uint16_t statusCode = 0;
    
    switch (newState) {
        case OTA_STATE_READY:
            statusCode = STATUS_BOOTLOADER_READY;
            break;
        case OTA_STATE_INITIALIZING:
            if (strstr(detail, "queued")) {
                statusCode = STATUS_DOWNLOAD_QUEUED;
            } else if (strstr(detail, "started")) {
                statusCode = STATUS_OTA_STARTING;
            }
            break;
        case OTA_STATE_DOWNLOADING:
            if (strstr(detail, "complete")) {
                statusCode = STATUS_DOWNLOAD_COMPLETE;
            } else if (progress > 0) {
                statusCode = STATUS_DOWNLOAD_PROGRESS;
            } else {
                statusCode = STATUS_DOWNLOAD_STARTED;
            }
            break;
        case OTA_STATE_VERIFYING:
            if (strstr(detail, "verified") || strstr(detail, "OK")) {
                statusCode = STATUS_VERIFY_SUCCESS;
            } else {
                statusCode = STATUS_VERIFY_STARTED;
            }
            break;
        case OTA_STATE_FLASHING:
            if (strstr(detail, "unlocked")) {
                statusCode = STATUS_FLASH_MCU_UNLOCK;
            } else if (strstr(detail, "Sending") || strstr(detail, "sending")) {
                statusCode = STATUS_FLASH_DATA_SENDING;
            } else if (strstr(detail, "verify") || strstr(detail, "CRC")) {
                statusCode = STATUS_FLASH_VERIFY_OK;
            } else if (strstr(detail, "reset")) {
                statusCode = STATUS_FLASH_RESET_MCU;
            } else {
                statusCode = STATUS_FLASH_STARTED;
            }
            break;
        case OTA_STATE_SUCCESS:
            statusCode = STATUS_OTA_SUCCESS;
            break;
        case OTA_STATE_CANCELLED:
            statusCode = STATUS_OTA_CANCELLED;
            break;
        case OTA_STATE_FAILED:
            // Determine specific failure code
            if (strstr(detail, "MQTT")) {
                if (strstr(detail, "lost")) {
                    statusCode = STATUS_ERROR_MQTT_LOST;
                } else {
                    statusCode = STATUS_ERROR_MQTT_CONN;
                }
            } else if (strstr(detail, "CAN")) {
                statusCode = STATUS_ERROR_CAN_INIT;
            } else if (strstr(detail, "Download") || strstr(detail, "download")) {
                statusCode = STATUS_ERROR_DOWNLOAD;
            } else if (strstr(detail, "SHA256") || strstr(detail, "checksum")) {
                statusCode = STATUS_ERROR_SHA256;
            } else if (strstr(detail, "processing")) {
                statusCode = STATUS_ERROR_FW_PROCESS;
            } else if (strstr(detail, "not responding")) {
                statusCode = STATUS_ERROR_MCU_NO_RESPONSE;
            } else if (strstr(detail, "bootloader")) {
                statusCode = STATUS_ERROR_MCU_BOOTLOADER;
            } else if (strstr(detail, "timeout")) {
                statusCode = STATUS_ERROR_TIMEOUT;
            } else if (strstr(detail, "cancel")) {
                statusCode = STATUS_ERROR_CANCELLED;
            }
            break;
        default:
            // Unknown state - don't publish
            return;
    }
    
    // Publish status if we determined a code
    if (statusCode > 0) {
        mqtt_publish_status(statusCode, detail, progress);
    }
}


/* Renamed from mqtt_poll_bl inner logic — processes one complete +QMTRECV line */
static void mqtt_process_qmtrecv(char *recv)
{
    DIVIDER("MQTT_BL");
    DBG("MQTT_BL", "★ +QMTRECV detected!");
    DBG("MQTT_BL", "Full line: %s", recv);

    // Extract JSON from the line
    char *jsonStart = strchr(recv, '{');
    char *jsonEnd   = strrchr(recv, '}');

    if (!jsonStart || !jsonEnd || jsonEnd <= jsonStart) {
        DBG_ERR("MQTT_BL", "No JSON object {{ }} found — discarding");
        DIVIDER("MQTT_BL");
        return;
    }

    size_t payloadLen = (size_t)(jsonEnd - jsonStart + 1);
    DBG("MQTT_BL", "JSON at offset %d, length %u bytes",
        (int)(jsonStart - recv), (unsigned)payloadLen);

    if (payloadLen == 0 || payloadLen > 700) {
        DBG_ERR("MQTT_BL", "Payload length %u out of range (1–700) — skipping", 
                (unsigned)payloadLen);
        DIVIDER("MQTT_BL");
        return;
    }

    char payload[1024];
    memcpy(payload, jsonStart, payloadLen);
    payload[payloadLen] = '\0';
    DBG("MQTT_BL", "Payload: %s", payload);

    // Parse JSON
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload);
    if (err) {
        DBG_ERR("MQTT_BL", "JSON parse error: %s", err.c_str());
        DIVIDER("MQTT_BL");
        return;
    }
    DBG("MQTT_BL", "JSON parsed OK");

    // ─── Check for CANCEL command ───
    const char *command = doc["command"];
    if (command && strcmp(command, "cancel") == 0) {
        DBG("MQTT_BL", "");
        DBG("MQTT_BL", "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        DBG("MQTT_BL", "  🛑  CANCEL COMMAND RECEIVED VIA MQTT  🛑");
        DBG("MQTT_BL", "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        g_otaCancel = true;
        ota_set_state(OTA_STATE_CANCELLED, "Cancel command received via MQTT", 0);
        DIVIDER("MQTT_BL");
        // Rollback to app0
        const esp_partition_t *app0 = esp_partition_find_first(
            ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
        if (app0) {
            DBG("OTA_BL", "Rolling back to app0...");
            esp_ota_set_boot_partition(app0);
        }
        g_bootloaderRestart = true;
        Serial.flush();
        delay(500);
        esp_restart();
        return;
    }

    // ─── Process DOWNLOAD command ───
    const char *url = doc["url"];
    const char *sha = doc["sha256"];

    const char *type = doc["type"];
    DBG("MQTT_BL", "  url    : %s", url  ? url  : "(MISSING)");
    DBG("MQTT_BL", "  sha256 : %s", sha  ? sha  : "(MISSING)");
    DBG("MQTT_BL", "  type   : %s", type ? type : "(not set — defaulting to master)");

    if (!url) {
        DBG_ERR("MQTT_BL", "Missing 'url' key in JSON");
        DIVIDER("MQTT_BL");
        return;
    }
    if (!sha) {
        DBG_ERR("MQTT_BL", "Missing 'sha256' key in JSON");
        DIVIDER("MQTT_BL");
        return;
    }
    if (strlen(url) == 0) {
        DBG_ERR("MQTT_BL", "url is empty string");
        DIVIDER("MQTT_BL");
        return;
    }
    if (strlen(sha) != 64) {
        DBG_ERR("MQTT_BL", "sha256 length=%u (expected 64 hex chars)", (unsigned)strlen(sha));
        DIVIDER("MQTT_BL");
        return;
    }

    DBG("MQTT_BL", "✅ Download command validated");
    DBG("MQTT_BL", "  URL   : %s", url);
    DBG("MQTT_BL", "  SHA256: %s", sha);

    xSemaphoreTake(g_otaMutex, portMAX_DELAY);
        {
            strncpy(g_pendingUrl,  url,  OTA_BL_MAX_URL    - 1);
            strncpy(g_pendingSha,  sha,  OTA_BL_MAX_SHA256 - 1);
            strncpy(g_pendingType, (type && type[0]) ? type : OTA_TYPE_MASTER,
                    sizeof(g_pendingType) - 1);
            g_pendingUrl [OTA_BL_MAX_URL    - 1]       = '\0';
            g_pendingSha [OTA_BL_MAX_SHA256 - 1]       = '\0';
            g_pendingType[sizeof(g_pendingType) - 1]   = '\0';
            g_downloadPending = true;
            g_blIdleStartMs   = millis();  // Reset idle timer — command received
        }
        xSemaphoreGive(g_otaMutex);
    
    // NEW: Signal CAN OTA task that download is ready
    xSemaphoreGive(g_downloadReadySem);
   // ota_set_state(OTA_STATE_INITIALIZING, "Download queued, OTA starting...", 0);

    DBG("MQTT_BL", "✅ CAN OTA task signaled — OTA will execute on Core 1");
    DIVIDER("MQTT_BL");
}

// ─────────────────────────────────────────────────────────────────────────────
//  HTTPS download via EC200U → SPIFFS
// ─────────────────────────────────────────────────────────────────────────────
static bool http_download(const char *url, const char *savePath, int resumeFrom)
{
    DIVIDER("DL");
    DBG("DL", "╔══════════════════════════════════════════════╗");
    DBG("DL", "║   HTTPS Download via EC200U                  ║");
    DBG("DL", "╚══════════════════════════════════════════════╝");
    DBG("DL", "URL          : %s", url);
    DBG("DL", "Save path    : %s", savePath);
    DBG("DL", "Resume from  : %d bytes", resumeFrom);
    DBG("DL", "SPIFFS free  : %u bytes", SPIFFS.totalBytes() - SPIFFS.usedBytes());

    char cmd[600];

    /* SSL config */
    DBG("DL", "--- SSL config ---");
    gsm_send("AT+QHTTPCFG=\"sslctxid\",1",         200);
    gsm_send("AT+QSSLCFG=\"sslversion\",1,4",       200);
    gsm_send("AT+QSSLCFG=\"ciphersuite\",1,0xFFFF", 200);
    gsm_send("AT+QSSLCFG=\"seclevel\",1,0",         200);
    DBG("DL", "SSL: TLS any version, any cipher, no cert verify");

    // Range header support for resume
    char rangeHeader[64] = {0};
    int  rangeHeaderLen  = 0;
    if (resumeFrom > 0) {
        snprintf(rangeHeader, sizeof(rangeHeader), "Range: bytes=%d-\r\n\r\n", resumeFrom);
        rangeHeaderLen = (int)strlen(rangeHeader);
        gsm_send("AT+QHTTPCFG=\"requestheader\",1", 200);
        DBG("DL", "Resume: injecting Range header (%d bytes)", rangeHeaderLen);
    } else {
        gsm_send("AT+QHTTPCFG=\"requestheader\",0", 200);
    }

    /* Set URL */
    int urlLen = (int)strlen(url);
    DBG("DL", "--- AT+QHTTPURL (URL length=%d) ---", urlLen);
    snprintf(cmd, sizeof(cmd), "AT+QHTTPURL=%d,80\r\n", urlLen);
    uart_write_bytes(OTA_BL_GSM_UART, cmd, strlen(cmd));
    if (!gsm_wait("CONNECT", 8000)) {
        DBG_ERR("DL", "No CONNECT for QHTTPURL");
        DIVIDER("DL"); return false;
    }
    uart_write_bytes(OTA_BL_GSM_UART, url, urlLen);
    if (!gsm_wait("OK", 8000)) {
        DBG_ERR("DL", "QHTTPURL not acknowledged");
        DIVIDER("DL"); return false;
    }
    DBG("DL", "✅ URL accepted");

    /* GET — include custom header size when resuming so modem prompts CONNECT */
    DBG("DL", "--- AT+QHTTPGET=80 ---");
    if (resumeFrom > 0) {
        snprintf(cmd, sizeof(cmd), "AT+QHTTPGET=80,%d\r\n", rangeHeaderLen);
        uart_write_bytes(OTA_BL_GSM_UART, cmd, strlen(cmd));
        if (!gsm_wait("CONNECT", 8000)) {
            DBG_ERR("DL", "No CONNECT prompt for Range header injection");
            DIVIDER("DL"); return false;
        }
        uart_write_bytes(OTA_BL_GSM_UART, rangeHeader, rangeHeaderLen);
    } else {
        gsm_send("AT+QHTTPGET=80", 100);
    }
    if (!gsm_wait("+QHTTPGET:", 90000)) {
        DBG_ERR("DL", "QHTTPGET timeout");
        DIVIDER("DL"); return false;
    }

    int httpStatus = 0, contentLength = 0;
    char *p = strstr(g_atBuf, "+QHTTPGET:");
    if (p) sscanf(p, "+QHTTPGET: %*d,%d,%d", &httpStatus, &contentLength);
    DBG("DL", "+QHTTPGET response: HTTP=%d  Content-Length=%d", httpStatus, contentLength);

    // Accept 200 (full download) or 206 (partial content / resume)
    if (httpStatus != 200 && httpStatus != 206) {
        DBG_ERR("DL", "HTTP error %d", httpStatus);
        DIVIDER("DL"); return false;
    }

    if (contentLength <= 0) {
        DBG_ERR("DL", "Invalid Content-Length=%d", contentLength);
        DIVIDER("DL"); return false;
    }

    // Total bytes we expect to have in SPIFFS when done
    int totalExpected = resumeFrom + contentLength;

    /* SPIFFS — append when resuming so we don't lose bytes already written */
    if (resumeFrom == 0 && SPIFFS.exists(savePath)) SPIFFS.remove(savePath);
    File f = SPIFFS.open(savePath, resumeFrom > 0 ? FILE_APPEND : FILE_WRITE);
    if (!f) {
        DBG_ERR("DL", "Cannot open %s for %s", savePath, resumeFrom > 0 ? "append" : "write");
        DIVIDER("DL"); return false;
    }

    /* Read stream */
    DBG("DL", "--- AT+QHTTPREAD=80 ---");
    const char *readCmd = "AT+QHTTPREAD=80\r\n";
    uart_write_bytes(OTA_BL_GSM_UART, readCmd, strlen(readCmd));

    static uint8_t overflow[OTA_BL_DL_CHUNK];
    int overflowLen = 0;
    if (!gsm_scan_marker("CONNECT\r\n", 8000, overflow, sizeof(overflow), &overflowLen)) {
        DBG_ERR("DL", "No CONNECT marker");
        f.close(); DIVIDER("DL"); return false;
    }

    // Start counting from resumeFrom so progress % is always relative to totalExpected
    int received = resumeFrom;
    if (overflowLen > 0) {
        if ((int)f.write(overflow, overflowLen) != overflowLen) {
            DBG_ERR("DL", "SPIFFS write error on overflow");
            f.close(); DIVIDER("DL"); return false;
        }
        received += overflowLen;
    }

    DBG("DL", "Streaming %d bytes (total target: %d)...", contentLength, totalExpected);
    // !! NO ota_set_state() / mqtt_publish_status() / AT commands inside this loop !!
    // Any AT command sent to the EC200U while it is in HTTP streaming mode causes it
    // to immediately abort the TCP session.
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(120000);
    int lastLogPct = -1;

    while (received < totalExpected && xTaskGetTickCount() < deadline) {
        int toRead = min((int)sizeof(g_dlChunk), totalExpected - received);
        int got = uart_read_bytes(OTA_BL_GSM_UART, g_dlChunk, toRead, pdMS_TO_TICKS(500));
        if (got > 0) {
            if ((int)f.write(g_dlChunk, got) != got) {
                DBG_ERR("DL", "SPIFFS write short");
                f.close(); DIVIDER("DL"); return false;
            }
            received += got;
            int pct = received * 100 / totalExpected;
            // Serial-only progress every 10% — zero AT commands to modem during stream
            if (pct / 10 != lastLogPct / 10) {
                lastLogPct = pct;
                Serial.printf("[DL] Progress: %d / %d bytes (%d%%)\n", received, totalExpected, pct);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    f.close();

    // Drain any pending UART data (non-blocking) to clear modem buffer
    uint8_t drain[128];
    uart_read_bytes(OTA_BL_GSM_UART, drain, sizeof(drain), pdMS_TO_TICKS(100));

    if (received >= totalExpected) {
        DBG("DL", "✅ Download complete: %d bytes → %s", received, savePath);
        DIVIDER("DL");
        return true;
    }

    // Incomplete download – return false immediately to allow retry.
    DBG_ERR("DL", "Incomplete download: %d/%d bytes received", received, totalExpected);
    DIVIDER("DL");
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Fix 3 (cont): Retry wrapper with Range-based resume
//
//  On each failed attempt we check how many bytes SPIFFS actually holds.
//  If that is more than where we started, the next call to http_download()
//  appends from that offset using an HTTP Range header (206 Partial Content).
//  Up to MAX_DL_ATTEMPTS are made before giving up.
// ─────────────────────────────────────────────────────────────────────────────
static bool http_download_with_retry(const char *url, const char *savePath)
{
    // Single attempt — any download failure boots back to app0 immediately.
    if (SPIFFS.exists(savePath)) {
        DBG("DL_RETRY", "Removing existing file before download");
        SPIFFS.remove(savePath);
    }

    DBG("DL_RETRY", "Starting download...");
    if (http_download(url, savePath, 0)) {
        return true;
    }

    DBG_ERR("DL_RETRY", "Download failed");
    return false;
}   

// ─────────────────────────────────────────────────────────────────────────────
//  SHA-256 verification
// ─────────────────────────────────────────────────────────────────────────────
static bool sha256_verify(const char *path, const char *expectedHex)
{
    DIVIDER("SHA256");
    DBG("SHA256", "═══ SHA-256 Verify ═══");
    DBG("SHA256", "File    : %s", path);
    DBG("SHA256", "Expected: %s", expectedHex);

    File f = SPIFFS.open(path, FILE_READ);
    if (!f) { DBG_ERR("SHA256", "Cannot open %s", path); DIVIDER("SHA256"); return false; }

    size_t fileSize = f.size();
    DBG("SHA256", "File size: %u bytes — hashing...", (unsigned)fileSize);

    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);

    uint8_t buf[512];
    size_t total = 0;
    int lastPct = -1;

    while (f.available()) {
        int got = f.read(buf, sizeof(buf));
        if (got > 0) {
            mbedtls_sha256_update(&ctx, buf, got);
            total += got;
            int pct = (int)(total * 100 / fileSize);
            if (pct / 25 != lastPct / 25) {
                lastPct = pct;
                DBG("SHA256", "Hashing: %u / %u bytes (%d%%)", (unsigned)total, (unsigned)fileSize, pct);
            }
        }
    }
    f.close();

    uint8_t digest[32];
    mbedtls_sha256_finish(&ctx, digest);
    mbedtls_sha256_free(&ctx);

    char hex[65];
    for (int i = 0; i < 32; i++) snprintf(hex + i*2, 3, "%02x", digest[i]);
    hex[64] = '\0';

    DBG("SHA256", "Computed : %s", hex);
    DBG("SHA256", "Expected : %s", expectedHex);

    if (strcasecmp(hex, expectedHex) == 0) {
        DBG("SHA256", "✅ SHA-256 MATCH — file integrity confirmed");
        DIVIDER("SHA256"); return true;
    }
    DBG_ERR("SHA256", "❌ SHA-256 MISMATCH");
    DBG_ERR("SHA256", "  Computed : %s", hex);
    DBG_ERR("SHA256", "  Expected : %s", expectedHex);
    DIVIDER("SHA256"); return false;
}



// ─────────────────────────────────────────────────────────────────────────────
//  ESP32 self-OTA: download → SHA256 verify → write to app0 → restart
//  Called from can_ota_task when g_pendingType == "esp32".
//
//  Boot-partition decisions:
//    • Success             → set app0 as next boot, call esp_restart()
//    • Failure BEFORE flash → return false  (app0 untouched, caller boots app0)
//    • Failure DURING flash → return true   (app0 may be partial, caller boots app1)
// ─────────────────────────────────────────────────────────────────────────────
static bool do_esp32_ota_internal(void)   /* returns true iff flash phase started */
{
    DBG("ESP32_OTA", "╔══════════════════════════════════════════════╗");
    DBG("ESP32_OTA", "║   ESP32 SELF-OTA SEQUENCE                    ║");
    DBG("ESP32_OTA", "╚══════════════════════════════════════════════╝");
    mqtt_publish_status(STATUS_ESP32_OTA_STARTING, "ESP32 self-OTA starting", 0);

    // ── Download ──────────────────────────────────────────────────────────
    ota_set_state(OTA_STATE_DOWNLOADING, "Downloading ESP32 firmware...", 0);
    if (!http_download_with_retry(g_pendingUrl, OTA_BL_FW_PATH)) {
        ota_set_state(OTA_STATE_FAILED, "ESP32 OTA: download failed", 0);
        mqtt_publish_status(STATUS_ESP32_OTA_FAILED, "Download failed", 0);
        return false;  // app0 untouched — safe to boot app0
    }
    DBG("ESP32_OTA", "✅ Download complete");

    // ── SHA256 verify ─────────────────────────────────────────────────────
    ota_set_state(OTA_STATE_VERIFYING, "Verifying ESP32 firmware SHA256...", 50);
    if (!sha256_verify(OTA_BL_FW_PATH, g_pendingSha)) {
        ota_set_state(OTA_STATE_FAILED, "ESP32 OTA: SHA256 mismatch", 50);
        mqtt_publish_status(STATUS_ESP32_OTA_FAILED, "SHA256 mismatch", 50);
        if (SPIFFS.exists(OTA_BL_FW_PATH)) SPIFFS.remove(OTA_BL_FW_PATH);
        return false;  // app0 untouched — safe to boot app0
    }
    DBG("ESP32_OTA", "✅ SHA256 verified");

    // ── Flash to app0 ─────────────────────────────────────────────────────
    //  FROM THIS POINT ON: if we fail, app0 may be partially erased/written.
    //  Return true so can_ota_task boots app1 (last-known-good backup).
    ota_set_state(OTA_STATE_FLASHING, "Writing new firmware to app0...", 75);
    mqtt_publish_status(STATUS_ESP32_FLASH_START, "Writing firmware to app0...", 75);

    const esp_partition_t *app0 = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
    if (!app0) {
        DBG_ERR("ESP32_OTA", "app0 partition not found!");
        ota_set_state(OTA_STATE_FAILED, "app0 partition not found", 75);
        mqtt_publish_status(STATUS_ESP32_OTA_FAILED, "app0 not found", 75);
        return false;  // nothing touched yet
    }

    File f = SPIFFS.open(OTA_BL_FW_PATH, FILE_READ);
    if (!f) {
        DBG_ERR("ESP32_OTA", "Cannot open firmware file from SPIFFS");
        ota_set_state(OTA_STATE_FAILED, "Cannot open firmware file", 75);
        mqtt_publish_status(STATUS_ESP32_OTA_FAILED, "File open failed", 75);
        return false;  // flash not started
    }
    size_t fwSize = f.size();
    DBG("ESP32_OTA", "Firmware size: %u bytes", (unsigned)fwSize);

    esp_ota_handle_t otaHandle;
    if (esp_ota_begin(app0, fwSize, &otaHandle) != ESP_OK) {
        DBG_ERR("ESP32_OTA", "esp_ota_begin failed");
        f.close();
        ota_set_state(OTA_STATE_FAILED, "OTA begin failed", 75);
        mqtt_publish_status(STATUS_ESP32_OTA_FAILED, "OTA begin failed", 75);
        return false;  // esp_ota_begin failed before erasing — app0 still intact
    }

    // esp_ota_begin has erased app0's flash sectors — must return true from here

    static uint8_t flashBuf[4096];
    size_t written = 0;
    bool flashOk = true;

    while (f.available() && flashOk) {
        size_t toRead = f.available();
        if (toRead > sizeof(flashBuf)) toRead = sizeof(flashBuf);
        size_t got = f.read(flashBuf, toRead);
        if (got == 0) break;
        if (esp_ota_write(otaHandle, flashBuf, got) != ESP_OK) {
            DBG_ERR("ESP32_OTA", "esp_ota_write failed at offset %u", (unsigned)written);
            flashOk = false; break;
        }
        written += got;
        if (written % 32768 == 0 || written == fwSize) {
            uint8_t pct = (uint8_t)(75 + 20 * written / fwSize);
            DBG("ESP32_OTA", "Flash progress: %u/%u (%u%%)", (unsigned)written, (unsigned)fwSize, pct);
        }
    }
    f.close();
    if (SPIFFS.exists(OTA_BL_FW_PATH)) SPIFFS.remove(OTA_BL_FW_PATH);

    if (!flashOk || esp_ota_end(otaHandle) != ESP_OK) {
        DBG_ERR("ESP32_OTA", "Flash write/verify failed");
        ota_set_state(OTA_STATE_FAILED, "Flash write failed", 90);
        mqtt_publish_status(STATUS_ESP32_OTA_FAILED, "Flash failed", 90);
        return true;   // ← app0 is partially written — caller must boot app1
    }
    DBG("ESP32_OTA", "✅ Firmware written to app0 (%u bytes)", (unsigned)written);

    nvs_handle_t nvs;
    if (nvs_open(OTA_BL_NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_str(nvs, OTA_BL_NVS_KEY_STATE, OTA_BL_NVS_VAL_PENDING);
        nvs_commit(nvs);
        nvs_close(nvs);
        DBG("ESP32_OTA", "NVS flag set: %s = %s", OTA_BL_NVS_KEY_STATE, OTA_BL_NVS_VAL_PENDING);
    } else {
        DBG_WARN("ESP32_OTA", "Could not write NVS flag — success confirmation will be missed");
    }

    // ── Set app0 as boot partition and restart ────────────────────────────
    if (esp_ota_set_boot_partition(app0) != ESP_OK) {
        DBG_ERR("ESP32_OTA", "esp_ota_set_boot_partition(app0) failed");
        ota_set_state(OTA_STATE_FAILED, "Set boot partition failed", 95);
        mqtt_publish_status(STATUS_ESP32_OTA_FAILED, "Boot partition set failed", 95);
        return true;  // app0 was written but not activated — boot app1 to be safe
    }

    mqtt_publish_status(STATUS_ESP32_FLASH_DONE, "Rebooting into new firmware...", 100);
    DBG("ESP32_OTA", "Rebooting into new app0 in 3 s...");
    Serial.flush();
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart();
    return false;  // unreachable — esp_restart() never returns
}
#endif /* OTA_BL_MODEM_OTA */

// ═══════════════════════════════════════════════════════════════════════════
//  CAN SELF-UPDATE SLAVE  (runs in factory, flashes app0)
//
//  Entered only when app0 saw the dedicated enter frame (ID 0x0C), backed up
//  app0→app1, set the CAN-mode NVS flag, and rebooted here. We SKIP all modem/
//  MQTT/download work and instead:
//    • emit the 0x456 heartbeat + a 0x702 "ready" beacon so the host tool knows
//      the device is now sitting in the bootloader (the flasher's handshake
//      keys on the 0x456 heartbeat and therefore skips re-sending the 0x0B
//      trigger — which keeps the BMS out of it);
//    • run the ABMS protocol on 0x0B (trigger→0x50, 0xB1→echo, UNLOCK/DATA/
//      VERIFY/RESET) and write the pushed firmware to app0.
//
//  On success: boot app0 (new firmware); app1 holds the pre-copy backup, so a
//  failed boot rolls back exactly like the MQTT OTA path. Synchronous flash
//  (4 KB static buffer) — the host is stop-and-wait.
// ═══════════════════════════════════════════════════════════════════════════
static bool                   su_active     = false;
static bool                   su_armed      = false;
static uint8_t                su_expected   = 0;
static uint32_t               su_crc        = 0xFFFFFFFFUL;
static esp_ota_handle_t       su_ota        = 0;
static const esp_partition_t *su_part       = nullptr;   /* target = app0 */
static bool                   su_begun      = false;
static bool                   su_flash_err  = false;
static uint8_t                su_wbuf[4096];
static uint32_t               su_wlen       = 0;

/* ── Factory WiFi service UI: web-flash state + app0→app1 backup result ──────
   The web flash reuses the same esp_ota primitives as the CAN slave, so the two
   must never run at once — su_cmd_unlock() refuses if a web flash is in progress
   and ota_bl_web_flash_begin() refuses if a CAN flash (su_begun) is in progress. */
static esp_ota_handle_t       s_web_ota       = 0;
static const esp_partition_t *s_web_part      = nullptr;
static bool                   s_web_ota_begun = false;
static unsigned               s_web_ota_bytes = 0;
static int                    s_backup_status = OTA_BL_BACKUP_PENDING;

#define SU_HEARTBEAT_MS      250UL      /* fast, so the flasher's 1 s check always hears it */
/* IOT/LCD parity: uniform 15 s of silence in factory → recover. Applies both to
 * "no flasher connected" (pre-activation) and mid-flash inactivity. Recovering
 * is safe: pre-flash app0 is intact (→ boots app0); mid-flash app0 is erased
 * (→ set_boot rejects it → boots app1 backup). */
#define SU_ACTIVE_TIMEOUT_MS 15000UL
#define SU_IDLE_TIMEOUT_MS   15000UL

static bool su_tx(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    twai_message_t m = {};
    m.identifier       = id;
    m.extd             = 0;
    m.data_length_code = (dlc > 8) ? 8 : dlc;
    memcpy(m.data, data, m.data_length_code);
    return twai_transmit(&m, pdMS_TO_TICKS(200)) == ESP_OK;
}

static void su_resp(uint8_t r, uint8_t b1 = 0, uint8_t b2 = 0)
{
    uint8_t f[8] = { r, b1, b2, 0, 0, 0, 0, 0 };
    su_tx(OTA_BL_CAN_ID, f, 8);
}

static void su_send_heartbeat(void)
{
    twai_message_t m = {};
    m.identifier       = OTA_BL_CAN_HEART_ID;   /* 0x456 */
    m.extd             = 0;
    m.data_length_code = 8;
    for (int i = 0; i < 8; i++) m.data[i] = (uint8_t)(0x11 + i);
    m.ss               = 1;
    twai_transmit(&m, 0);
}

static void su_send_ready(void)
{
    static const uint8_t rdy[8] = { 0x5A,0xA5,0x5A,0xA5,0x5A,0xA5,0x5A,0xA5 };
    twai_message_t m = {};
    m.identifier       = OTA_BL_READY_CAN_ID;   /* 0x702 */
    m.extd             = 0;
    m.data_length_code = 8;
    memcpy(m.data, rdy, 8);
    m.ss               = 1;
    twai_transmit(&m, 0);
}

static bool su_flush(void)
{
    if (su_wlen == 0) return true;
    esp_err_t e = esp_ota_write(su_ota, su_wbuf, su_wlen);
    su_wlen = 0;
    if (e != ESP_OK) { DBG_ERR("CAN_SU", "esp_ota_write failed: %d", e); su_flash_err = true; return false; }
    return true;
}

static void su_reset_session(void)
{
    su_expected = 0;
    su_crc      = 0xFFFFFFFFUL;
    su_wlen     = 0;
    su_flash_err = false;
    if (su_begun) { esp_ota_abort(su_ota); su_begun = false; }
    su_part = nullptr;
}

/* ── Recover to a valid app (IOT/LCD parity) ──────────────────────────────
 * Try app0, else app1. esp_ota_set_boot_partition() VALIDATES the image, so a
 * half-written / erased / corrupt app0 is rejected and we fall back to app1
 * (the pre-flash backup). Reboots on success — never returns.
 * Returns false WITHOUT rebooting only if NEITHER slot is bootable; the caller
 * then keeps the unit in the CAN flash slave so it can still be rescued over
 * CAN (never force-boots a known-bad app0, never bricks into a boot loop). */
static bool recover_boot_to_app(const char *reason)
{
    const esp_partition_t *app0 = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
    const esp_partition_t *app1 = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);

    const char *dst = nullptr;
    if (app0 && esp_ota_set_boot_partition(app0) == ESP_OK)
        dst = "app0";
    else if (app1 && esp_ota_set_boot_partition(app1) == ESP_OK)
        dst = "app1 (backup — app0 invalid)";

    if (!dst) {
        DBG_ERR("OTA_BL", "Recover (%s): neither app0 nor app1 bootable — staying in factory", reason);
        return false;
    }
    DBG("OTA_BL", "Recover (%s) → booting %s", reason, dst);
    Serial.flush();
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
    return true;   /* unreachable */
}

/* ── CAN-slave recovery ───────────────────────────────────────────────────
 * Called when a CAN self-update ends without a committed app0 (host vanished,
 * cancelled, CRC fail, CAN init failure, idle timeout, …). Abort any open OTA,
 * then recover to a valid app. If neither app is bootable (bricked), stay in
 * the CAN flash slave so the unit can still be rescued over CAN. */
static void su_recover(const char *reason)
{
    DBG("CAN_SU", "Recovery: %s", reason);
    if (su_begun) { esp_ota_abort(su_ota); su_begun = false; }

    if (recover_boot_to_app(reason)) return;   /* boots app0/app1 — never returns */

    /* Neither app0 nor app1 bootable → reboot into factory MQTT OTA to re-download
     * over the cloud. On the reboot there is no flash request, so factory falls
     * through to the MQTT downloader. */
    DBG("CAN_SU", "No valid app — rebooting to factory MQTT OTA (cloud re-download)");
    Serial.flush();
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

static bool su_check_trigger(const uint8_t *d, uint8_t dlc)
{
    if (dlc < 7) return false;
    for (int i = 0; i < 7; i++) {
        uint8_t expected = (i % 2 == 0) ? OTA_BL_CMD_TRIGGER : OTA_BL_CMD_TRIGGER_INV;
        if (d[i] != expected) return false;
    }
    return true;
}

static void su_cmd_unlock(const uint8_t *p)
{
    uint32_t sz = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    DBG("CAN_SU", "UNLOCK size=%u bytes", (unsigned)sz);

    /* Mutual exclusion with the WiFi web flash — both write app0 via esp_ota. */
    if (s_web_ota_begun) { DBG_ERR("CAN_SU", "web flash in progress — refusing CAN flash"); su_resp(OTA_BL_RESP_ERROR); return; }

    if (su_begun) { esp_ota_abort(su_ota); su_begun = false; }

    su_part = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
    if (!su_part) { DBG_ERR("CAN_SU", "app0 not found"); su_resp(OTA_BL_RESP_ERROR); return; }
    DBG("CAN_SU", "OTA target: '%s' @ 0x%08X", su_part->label, (unsigned)su_part->address);

    if (esp_ota_begin(su_part, OTA_SIZE_UNKNOWN, &su_ota) != ESP_OK) {
        DBG_ERR("CAN_SU", "esp_ota_begin failed"); su_resp(OTA_BL_RESP_ERROR); return;
    }
    su_begun = true; su_expected = 0; su_crc = 0xFFFFFFFFUL; su_wlen = 0; su_flash_err = false;
    DBG("CAN_SU", "app0 erased — ready for DATA");
    su_resp(OTA_BL_RESP_OK);
}

static void su_cmd_data(uint8_t seq, uint8_t len, const uint8_t *data)
{
    if (!su_begun)     { su_resp(OTA_BL_RESP_ERROR); return; }
    if (seq != su_expected) { su_resp(OTA_BL_RESP_SEQ_ERROR); return; }
    if (len == 0 || len > 4) len = 4;
    if (su_flash_err)  { su_resp(OTA_BL_RESP_ERROR); return; }

    su_crc = crc32_update(su_crc, data, len);
    memcpy(su_wbuf + su_wlen, data, len);
    su_wlen += len;
    if (su_wlen >= sizeof(su_wbuf)) { if (!su_flush()) { su_resp(OTA_BL_RESP_ERROR); return; } }

    su_expected = (uint8_t)((su_expected + 1) & 0xFF);
    su_resp(OTA_BL_RESP_OK);
}

static void su_cmd_verify(const uint8_t *p)
{
    if (!su_begun) { su_resp(OTA_BL_RESP_ERROR); return; }
    if (!su_flush()) { su_resp(OTA_BL_RESP_ERROR); return; }

    uint32_t host_crc = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    uint32_t calc_crc = su_crc ^ 0xFFFFFFFFUL;
    DBG("CAN_SU", "VERIFY host=0x%08X calc=0x%08X", (unsigned)host_crc, (unsigned)calc_crc);

    if (host_crc != calc_crc) {
        DBG_ERR("CAN_SU", "CRC MISMATCH"); esp_ota_abort(su_ota); su_begun = false;
        su_resp(OTA_BL_RESP_CRC_FAIL); return;
    }
    if (esp_ota_end(su_ota) != ESP_OK) { su_begun = false; su_resp(OTA_BL_RESP_CRC_FAIL); return; }

    /* Flag app0 OTA pending so app0 publishes success on its first clean boot */
    nvs_handle_t nvs;
    if (nvs_open(OTA_BL_NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_str(nvs, OTA_BL_NVS_KEY_STATE, OTA_BL_NVS_VAL_PENDING);
        nvs_commit(nvs); nvs_close(nvs);
    }
    if (esp_ota_set_boot_partition(su_part) != ESP_OK) { su_resp(OTA_BL_RESP_ERROR); return; }
    su_begun = false;
    DBG("CAN_SU", "CRC OK — app0 committed, waiting for RESET");
    su_resp(OTA_BL_RESP_CRC_OK);
}

static void su_cmd_reset(void)
{
    DBG("CAN_SU", "RESET — booting new app0 in 50 ms");
    su_resp(OTA_BL_RESP_OK);
    Serial.flush();
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
}

static void su_process_frame(const uint8_t *d, uint8_t dlc)
{
    if (!su_active) {
        if (su_check_trigger(d, dlc)) {
            su_armed = true;
            DBG("CAN_SU", "Trigger matched — ARMED, replying 0x50");
            su_resp(OTA_BL_RESP_OK);
            return;
        }
        if (dlc >= 1 && d[0] == OTA_BL_ECHO_CMD) {
            DBG("CAN_SU", "0xB1 received — activating");
            uint8_t echo[8] = { OTA_BL_ECHO_CMD, 0, 0, 0, 0, 0, 0, 0 };
            su_tx(OTA_BL_CAN_ID, echo, 8);
            su_active = true;
            /* Concurrent-with-MQTT mode: claim the flash so a cloud OTA can't
             * start on us and MQTT UART polling pauses. Harmless in pure CAN
             * mode (those flags aren't used there). */
            g_otaInProgress    = true;
            g_pauseMqttPolling = true;
            su_reset_session();
            DBG("CAN_SU", "=== CAN SELF-UPDATE SESSION OPEN (target app0) ===");
            return;
        }
        return;
    }

    if (dlc < 4 || d[2] != OTA_BL_HEADER_MAGIC) { su_resp(OTA_BL_RESP_INVALID); return; }

    switch (d[0]) {
        case OTA_BL_CMD_UNLOCK: su_cmd_unlock(&d[4]);              break;
        case OTA_BL_CMD_DATA:   su_cmd_data(d[1], d[3], &d[4]);    break;
        case OTA_BL_CMD_VERIFY: su_cmd_verify(&d[4]);              break;
        case OTA_BL_CMD_RESET:  su_cmd_reset();                    break;
        case OTA_BL_CMD_READ_VERSION:   /* parity with ABMS bootloader — reply v3.6 */
            su_resp(OTA_BL_RESP_OK, OTA_BL_VERSION_MAJOR, OTA_BL_VERSION_MINOR);
            break;
        default:                su_resp(OTA_BL_RESP_INVALID);      break;
    }
}

static void run_can_selfupdate_slave(void)
{
    DIVIDER("CAN_SU");
    DBG("CAN_SU", "╔══════════════════════════════════════════════╗");
    DBG("CAN_SU", "║   CAN SELF-UPDATE MODE (slave, flashes app0) ║");
    DBG("CAN_SU", "║   modem / MQTT skipped                       ║");
    DBG("CAN_SU", "╚══════════════════════════════════════════════╝");
    EVLOG_INFO("OTA", "CAN self-update slave started (flashing app0 over CAN)");

    if (!can_init_bl()) {
        DBG_ERR("CAN_SU", "CAN init failed — returning to app");
        EVLOG_FAIL("OTA", "CAN self-update: CAN init failed — returning to app");
        su_recover("CAN init failed");   /* app0 untouched → boots app0 */
    }

    su_active = false; su_armed = false;
    su_reset_session();

    TickType_t beacon_tick = xTaskGetTickCount();
    TickType_t pre_enter   = beacon_tick;
    TickType_t last_frame  = beacon_tick;
    bool       prev_active = false;

    DBG("CAN_SU", "Listening on 0x%03X @ 500 kbps — heartbeat 0x%03X, ready 0x%03X",
        OTA_BL_CAN_ID, OTA_BL_CAN_HEART_ID, OTA_BL_READY_CAN_ID);

    for (;;) {
        twai_message_t msg;
        if (twai_receive(&msg, pdMS_TO_TICKS(100)) == ESP_OK) {
            if (!msg.extd && msg.identifier == OTA_BL_CAN_ID) {
                last_frame = xTaskGetTickCount();
                uint8_t dlc = (msg.data_length_code > 8) ? 8 : msg.data_length_code;
                su_process_frame(msg.data, dlc);
            }
        }

        TickType_t now = xTaskGetTickCount();
        if (prev_active && !su_active) pre_enter  = now;
        if (!prev_active && su_active) last_frame = now;
        prev_active = su_active;

        if (!su_active) {
            if ((now - beacon_tick) >= pdMS_TO_TICKS(SU_HEARTBEAT_MS)) {
                beacon_tick = now;
                su_send_heartbeat();
                su_send_ready();
            }
            if ((uint32_t)((now - pre_enter) * portTICK_PERIOD_MS) >= SU_IDLE_TIMEOUT_MS) {
                DBG("CAN_SU", "No flasher in %lu s — auto-recovering",
                    (unsigned long)(SU_IDLE_TIMEOUT_MS / 1000));
                su_recover("idle timeout — no flasher connected");  /* never returns */
            }
        } else {
            if ((uint32_t)((now - last_frame) * portTICK_PERIOD_MS) >= SU_ACTIVE_TIMEOUT_MS) {
                DBG_WARN("CAN_SU", "Flash session inactive %lu s — auto-recovering",
                         (unsigned long)(SU_ACTIVE_TIMEOUT_MS / 1000));
                su_recover("mid-flash inactivity timeout");         /* never returns */
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Concurrent CAN flash slave — runs ALONGSIDE the MQTT downloader.
//
//  Created ONLY in MQTT/OTA mode (see ota_bootloader_run) so a unit that was
//  put into factory via an MQTT OTA trigger can ALSO be flashed over CAN if the
//  operator chooses. TWAI is already installed (bl_announce_bootloader_mode).
//  Differences from run_can_selfupdate_slave():
//    • Does NOT install TWAI (shared, already up).
//    • No pre-activation idle-recover — the MQTT 5-min idle timeout governs when
//      nothing is happening; this task just listens.
//    • Coordinates with the cloud flasher via g_otaInProgress: it stays off the
//      bus while a cloud/master flash runs, and its own 0xB1 activation sets
//      g_otaInProgress (see su_process_frame) so a cloud flash can't start.
//  The flash session itself (UNLOCK/DATA/VERIFY/RESET → app0) and recovery are
//  shared with the pure-CAN path via su_process_frame()/su_recover().
// ─────────────────────────────────────────────────────────────────────────────
static void can_slave_task(void *arg)
{
    TickType_t beacon_tick = xTaskGetTickCount();
    TickType_t last_frame  = beacon_tick;
    su_active = false; su_armed = false;
    su_reset_session();
    DBG("CAN_SU", "Concurrent CAN flash slave listening on 0x%03X (MQTT OTA mode)", OTA_BL_CAN_ID);

    for (;;) {
        /* Stay off the bus while a cloud/master flash owns TWAI. */
        if (g_otaInProgress && !su_active) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

        twai_message_t msg;
        if (twai_receive(&msg, pdMS_TO_TICKS(100)) == ESP_OK) {
            if (!msg.extd && msg.identifier == OTA_BL_CAN_ID) {
                last_frame = xTaskGetTickCount();
                uint8_t dlc = (msg.data_length_code > 8) ? 8 : msg.data_length_code;
                su_process_frame(msg.data, dlc);
            }
        }

        TickType_t now = xTaskGetTickCount();
        if (su_active) {
            if ((uint32_t)((now - last_frame) * portTICK_PERIOD_MS) >= SU_ACTIVE_TIMEOUT_MS) {
                DBG_WARN("CAN_SU", "CAN flash inactive %lu s — auto-recovering",
                         (unsigned long)(SU_ACTIVE_TIMEOUT_MS / 1000));
                su_recover("mid-flash inactivity (MQTT-concurrent)");   /* never returns */
            }
        } else if (su_armed) {
            /* Heartbeat/ready only once a trigger armed us, so the bus stays
             * quiet during normal MQTT idle. */
            if ((now - beacon_tick) >= pdMS_TO_TICKS(SU_HEARTBEAT_MS)) {
                beacon_tick = now;
                su_send_heartbeat();
                su_send_ready();
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// END OF FORWARD FUNCTIONS - now add do_can_ota() below
// ═══════════════════════════════════════════════════════════════════════════

#if OTA_BL_MODEM_OTA   /* EC200 remote-OTA removed — compiled out (see run() + top) */
static void do_can_ota_internal(void)
{
    // ═══════════════════════════════════════════════════════════════════════
    // INITIALIZATION PHASE
    // ═══════════════════════════════════════════════════════════════════════
    
    g_otaStartTime = millis();
    g_otaCancel = false;
    g_uploadInProgress = true;
    g_uploadProgress = 0;
    
    // NEW: Pause MQTT polling for entire OTA sequence - no modem interference
    g_pauseMqttPolling = true;
    vTaskDelay(pdMS_TO_TICKS(100));  // Let mqtt_poll_task exit gracefully
    
    DIVIDER("OTA_SEQ");
    DBG("OTA_SEQ", "╔══════════════════════════════════════════════╗");
    DBG("OTA_SEQ", "║   🚀  OTA SEQUENCE STARTED                   ║");
    DBG("OTA_SEQ", "╚══════════════════════════════════════════════╝");
    DBG("OTA_SEQ", "Start time  : %lu ms", g_otaStartTime);
    DBG("OTA_SEQ", "Max duration: %u ms (%.1f min)", OTA_MAX_DURATION, 
        OTA_MAX_DURATION / 60000.0);
    
    ota_set_state(OTA_STATE_INITIALIZING, "OTA sequence started", 0);
    
    // ═══════════════════════════════════════════════════════════════════════
    // CHECK: Modem connected?
    // ═══════════════════════════════════════════════════════════════════════
    
    if (!g_mqttConn) {
        g_lastError = "MQTT not connected";
        ota_set_state(OTA_STATE_FAILED, "MQTT connection lost", 0);
        DBG_ERR("OTA_SEQ", "Cannot start OTA without MQTT connection");
        goto ota_cleanup;
    }
    DBG("OTA_SEQ", "✅ MQTT connected");
    
    // ═══════════════════════════════════════════════════════════════════════
    // CHECK: CAN available?
    // ═══════════════════════════════════════════════════════════════════════
    
    if (!g_canRunning && !can_init_bl()) {
        g_lastError = "CAN initialization failed";
        ota_set_state(OTA_STATE_FAILED, "CAN init failed", 0);
        DBG_ERR("OTA_SEQ", "Cannot initialize CAN bus");
        goto ota_cleanup;
    }
    DBG("OTA_SEQ", "✅ CAN initialized");
    
    // ═══════════════════════════════════════════════════════════════════════
    // DOWNLOAD PHASE
    // ═══════════════════════════════════════════════════════════════════════
    
    ota_set_state(OTA_STATE_DOWNLOADING, "Downloading firmware...", 0);
    
    DBG("OTA_SEQ", "");
    DBG("OTA_SEQ", "DOWNLOAD PHASE");
    DBG("OTA_SEQ", "═════════════════════════════════════════════════");
    DBG("OTA_SEQ", "URL   : %s", g_pendingUrl);
    DBG("OTA_SEQ", "SHA256: %s", g_pendingSha);
    
    if (!http_download_with_retry(g_pendingUrl, OTA_BL_FW_PATH)) {
        g_lastError = "HTTP download failed";
        ota_set_state(OTA_STATE_FAILED, g_lastError.c_str(), g_uploadProgress);
        DBG_ERR("OTA_SEQ", "Download failed: %s", g_lastError.c_str());
        goto ota_cleanup;
    }
    
    if (g_otaCancel) {
        g_lastError = "Download cancelled by user";
        ota_set_state(OTA_STATE_CANCELLED, "Download cancelled", g_uploadProgress);
        DBG("OTA_SEQ", "⚠️  OTA cancelled during download");
        goto ota_cleanup;
    }
    
    if ((millis() - g_otaStartTime) > OTA_MAX_DURATION) {
        g_lastError = "OTA timeout exceeded (>10 min)";
        ota_set_state(OTA_STATE_FAILED, "OTA timeout exceeded", g_uploadProgress);
        DBG_ERR("OTA_SEQ", "OTA exceeded maximum duration of 10 minutes");
        goto ota_cleanup;
    }
    
    g_uploadProgress = 50;
    DBG("OTA_SEQ", "✅ Download complete (50%)");
    ota_set_state(OTA_STATE_DOWNLOADING, "Download complete", 100);
    
    // ═══════════════════════════════════════════════════════════════════════
    // VERIFICATION PHASE
    // ═══════════════════════════════════════════════════════════════════════
    
    ota_set_state(OTA_STATE_VERIFYING, "Verifying SHA256 checksum...", 75);
    
    DBG("OTA_SEQ", "");
    DBG("OTA_SEQ", "VERIFICATION PHASE");
    DBG("OTA_SEQ", "═════════════════════════════════════════════════");
    DBG("OTA_SEQ", "Calculating SHA256 hash of downloaded firmware...");
    
    if (!sha256_verify(OTA_BL_FW_PATH, g_pendingSha)) {
        g_lastError = "SHA256 verification failed";
        ota_set_state(OTA_STATE_FAILED, "SHA256 mismatch - corrupted download", 75);
        DBG_ERR("OTA_SEQ", "SHA256 checksum MISMATCH!");
        DBG_ERR("OTA_SEQ", "Expected: %s", g_pendingSha);
        goto ota_cleanup;
    }
    
    if (g_otaCancel) {
        g_lastError = "Verification cancelled by user";
        ota_set_state(OTA_STATE_CANCELLED, "Verification cancelled", 75);
        DBG("OTA_SEQ", "⚠️  OTA cancelled during verification");
        goto ota_cleanup;
    }
    
    if ((millis() - g_otaStartTime) > OTA_MAX_DURATION) {
        g_lastError = "OTA timeout exceeded (>10 min)";
        ota_set_state(OTA_STATE_FAILED, "OTA timeout exceeded", g_uploadProgress);
        DBG_ERR("OTA_SEQ", "OTA exceeded maximum duration");
        goto ota_cleanup;
    }
    
    g_uploadProgress = 75;
    DBG("OTA_SEQ", "✅ SHA256 verification PASSED");
    ota_set_state(OTA_STATE_VERIFYING, "SHA256 verified OK", 90);
    
    if (!fw_process(OTA_BL_FW_PATH)) {
        g_lastError = "Firmware processing failed";
        ota_set_state(OTA_STATE_FAILED, "Firmware processing error", 75);
        DBG_ERR("OTA_SEQ", "Failed to process firmware file");
        goto ota_cleanup;
    }
    
    DBG("OTA_SEQ", "✅ Firmware loaded into RAM");
    DBG("OTA_SEQ", "  Size: %u bytes", (unsigned)g_fwSize);
    DBG("OTA_SEQ", "  CRC32: 0x%08X", g_fwCRC);
    
    // ═══════════════════════════════════════════════════════════════════════
    // FLASHING PHASE
    // ═══════════════════════════════════════════════════════════════════════
    
    ota_set_state(OTA_STATE_FLASHING, "Flashing firmware to remote MCU...", 95);
    
    DBG("OTA_SEQ", "");
    DBG("OTA_SEQ", "FLASHING PHASE");
    DBG("OTA_SEQ", "═════════════════════════════════════════════════");
    DBG("OTA_SEQ", "Initializing remote MCU bootloader...");
    
    // Heartbeat check
    if (!bl_heartbeat()) {
        DBG_WARN("OTA_SEQ", "No initial heartbeat - sending wake-up sequence...");
        
        // ⭐ SEND WAKE-UP SEQUENCE
        uint8_t wakeupData[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
        if (can_send(0x0000, wakeupData, 8, false)) {
            DBG("OTA_SEQ", "✅ Wake-up sequence sent (ID: 0x0000, Data: 00 01 02 03 04 05 06 07)");
            
            // ⭐ LISTEN FOR 5 SECONDS
            DBG("OTA_SEQ", "Listening for bootloader response for 5 seconds...");
            unsigned long wakeupStart = millis();
            bool foundHeartbeat = false;
            
            while ((millis() - wakeupStart) < 5000) {
                twai_message_t msg;
                if (can_recv(msg, 100)) {
                    DBG("OTA_SEQ", "  Received CAN message: ID=0x%03X Len=%d", msg.identifier, msg.data_length_code);
                    //if (msg.identifier == OTA_BL_CAN_HEART_ID) {
                        DBG("OTA_SEQ", "  ✅ Bootloader heartbeat detected!");
                        foundHeartbeat = true;
                        break;
                    //}
                }
            }
            
            if (foundHeartbeat) {
                DBG("OTA_SEQ", "✅ Bootloader responded to wake-up");
            } else {
                DBG_ERR("OTA_SEQ", "No heartbeat after wake-up sequence");
                g_lastError = "Remote bootloader heartbeat failed";
                ota_set_state(OTA_STATE_FAILED, "Remote MCU not responding", 95);
                goto ota_cleanup;
            }
        } else {
            DBG_ERR("OTA_SEQ", "Failed to send wake-up sequence");
            g_lastError = "Remote bootloader heartbeat failed";
            ota_set_state(OTA_STATE_FAILED, "Remote MCU not responding", 95);
            goto ota_cleanup;
        }
    }
    
    if (!bl_init()) {
        g_lastError = "Remote bootloader initialization failed";
        ota_set_state(OTA_STATE_FAILED, "Remote bootloader error", 95);
        DBG_ERR("OTA_SEQ", "Cannot initialize remote bootloader");
        goto ota_cleanup;
    }
    
    if (g_otaCancel) {
        g_lastError = "Flashing cancelled by user";
        ota_set_state(OTA_STATE_CANCELLED, "Flash init cancelled", 95);
        DBG("OTA_SEQ", "⚠️  OTA cancelled during bootloader init");
        goto ota_cleanup;
    }
    
    DBG("OTA_SEQ", "✅ Remote bootloader ready");
    
    // NEW: Give device time to settle after initialization
    vTaskDelay(pdMS_TO_TICKS(500));  //settle time
    
    DBG("OTA_SEQ", "Sending firmware size (%u bytes)...", (unsigned)g_fwSize);
    vTaskDelay(pdMS_TO_TICKS(1000));  // 1000ms before first data frame
    
    if (!bl_send_info(g_fwSize)) {
        g_lastError = "Failed to send firmware size";
        //ota_set_state(OTA_STATE_FAILED, "Remote size command failed", 95);
        DBG_ERR("OTA_SEQ", "Cannot send firmware size to remote");
        goto ota_cleanup;
    }
    
    DBG("OTA_SEQ", "✅ Firmware size sent (%u bytes)", (unsigned)g_fwSize);
    vTaskDelay(pdMS_TO_TICKS(100));  // 100ms before first data frame
    
    if ((millis() - g_otaStartTime) > OTA_MAX_DURATION) {
        g_lastError = "OTA timeout exceeded (>10 min)";
        //ota_set_state(OTA_STATE_FAILED, "OTA timeout exceeded", g_uploadProgress);
        DBG_ERR("OTA_SEQ", "OTA exceeded maximum duration");
        goto ota_cleanup;
    }
    
    // Flash loop - send firmware in chunks streamed from SPIFFS.
    // ⭐ IMPORTANT: Wrap in scope block to avoid goto crossing initialization.
    // ⭐ NO ota_set_state() calls in this loop - MQTT would block CAN responses!
    // ⭐ g_fwData no longer exists — fw_read_chunk() fetches OTA_BL_DL_CHUNK
    //    bytes at a time from the open SPIFFS file, keeping peak RAM at ~16 KB.
    {
        DBG("OTA_SEQ", "Sending firmware data (streaming from SPIFFS)...");

        // Allocate the single per-iteration scratch buffer once.
        uint8_t *canChunkBuf = (uint8_t*)malloc(OTA_BL_DL_CHUNK);
        if (!canChunkBuf) {
            g_lastError = "malloc for CAN chunk buffer failed";
            ota_set_state(OTA_STATE_FAILED, "CAN chunk alloc error", 95);
            DBG_ERR("OTA_SEQ", "malloc(%u) for CAN chunk failed", (unsigned)OTA_BL_DL_CHUNK);
            goto ota_cleanup;
        }

        size_t sent = 0;

        while (sent < g_fwSize) {
            size_t chunkSize = g_fwSize - sent;
            if (chunkSize > OTA_BL_DL_CHUNK) chunkSize = OTA_BL_DL_CHUNK;

            // Read this chunk from SPIFFS (real bytes + 0xFF padding as needed).
            if (!fw_read_chunk(sent, canChunkBuf, chunkSize)) {
                g_lastError = "SPIFFS read error during CAN flash";
                ota_set_state(OTA_STATE_FAILED, "CAN flash read error", 95);
                DBG_ERR("OTA_SEQ", "fw_read_chunk failed at offset %u", (unsigned)sent);
                free(canChunkBuf);
                goto ota_cleanup;
            }

            if (!bl_send_data(canChunkBuf, chunkSize)) {
                g_lastError = "Data chunk transmission failed";
                ota_set_state(OTA_STATE_FAILED, "CAN flash error", 95);
                DBG_ERR("OTA_SEQ", "Failed to send firmware chunk at offset %u",
                        (unsigned)sent);
                free(canChunkBuf);
                goto ota_cleanup;
            }

            sent += chunkSize;

            // Update progress but DON'T call ota_set_state() - it blocks CAN!
            g_uploadProgress = 90 + (9 * sent / g_fwSize);

            // Log progress every 10 KB (verbose logging interferes too, limit it)
            if (sent % 10240 == 0 || sent == g_fwSize) {
                DBG("OTA_SEQ", "Flash progress: %u / %u bytes (%u%%)",
                    (unsigned)sent, (unsigned)g_fwSize, g_uploadProgress);
            }

            if (g_otaCancel) {
                g_lastError = "Flashing cancelled by user";
                char cancelMsg[256];
                snprintf(cancelMsg, sizeof(cancelMsg),
                        "Flash cancelled at byte offset %u", (unsigned)sent);
                ota_set_state(OTA_STATE_CANCELLED, cancelMsg, g_uploadProgress);
                DBG("OTA_SEQ", "⚠️  OTA cancelled during flash (at %u bytes)",
                    (unsigned)sent);
                free(canChunkBuf);
                goto ota_cleanup;
            }

            if ((millis() - g_otaStartTime) > OTA_MAX_DURATION) {
                g_lastError = "OTA timeout exceeded (>10 min)";
                ota_set_state(OTA_STATE_FAILED, "OTA timeout exceeded", g_uploadProgress);
                DBG_ERR("OTA_SEQ", "OTA exceeded maximum duration during flash");
                free(canChunkBuf);
                goto ota_cleanup;
            }
        }

        free(canChunkBuf);
    }  // ← CLOSE SCOPE BLOCK
    
    DBG("OTA_SEQ", "✅ All firmware data sent (%u bytes)", (unsigned)g_fwSize);
    
    // NOW publish status after all critical CAN operations done
    ota_set_state(OTA_STATE_FLASHING, "Firmware transmitted, verifying CRC...", 95);
    
    if (!bl_verify(g_fwCRC)) {
        g_lastError = "Remote CRC verification failed";
        ota_set_state(OTA_STATE_FAILED, "Remote CRC mismatch", 99);
        DBG_ERR("OTA_SEQ", "Remote bootloader CRC check failed");
        goto ota_cleanup;
    }
    
    DBG("OTA_SEQ", "✅ Remote CRC verification PASSED");
    mqtt_publish_status(STATUS_FLASH_VERIFY_OK, "Flash verified OK", 99);
    
    if (!bl_reset()) {
        g_lastError = "Remote reset command failed";
        ota_set_state(OTA_STATE_FAILED, "Remote reset error", 99);
        DBG_ERR("OTA_SEQ", "Cannot reset remote bootloader");
        goto ota_cleanup;
    }
    
    DBG("OTA_SEQ", "✅ Reset command sent to remote bootloader");
    
    if (g_otaCancel) {
        g_lastError = "OTA cancelled after flash";
        ota_set_state(OTA_STATE_CANCELLED, "Cancelled after completion", 100);
        DBG("OTA_SEQ", "⚠️  OTA cancelled (flash complete but not confirmed)");
        goto ota_cleanup;
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // SUCCESS!
    // ═══════════════════════════════════════════════════════════════════════
    
    g_uploadProgress = 100;
    ota_set_state(OTA_STATE_SUCCESS, "OTA completed successfully", 100);
    
    Serial.println();
    DBG("OTA_SEQ", "╔══════════════════════════════════════════════╗");
    DBG("OTA_SEQ", "║   ✅  OTA-over-CAN SUCCESSFUL!              ║");
    DBG("OTA_SEQ", "║                                              ║");
    DBG("OTA_SEQ", "║   • Firmware verified                        ║");
    DBG("OTA_SEQ", "║   • Data transmitted                         ║");
    DBG("OTA_SEQ", "║   • Remote CRC verified                      ║");
    DBG("OTA_SEQ", "║   • Reset command sent                       ║");
    DBG("OTA_SEQ", "╚══════════════════════════════════════════════╝");
    DIVIDER("OTA_SEQ");
    
    goto ota_cleanup;

ota_cleanup:
    // ═══════════════════════════════════════════════════════════════════════
    // CLEANUP & ROLLBACK PHASE
    // ═══════════════════════════════════════════════════════════════════════
    
    // NEW: Resume MQTT polling now that critical OTA operations are complete
    g_pauseMqttPolling = false;
    DBG("OTA_SEQ", "MQTT polling resumed");
    
    DBG("OTA_SEQ", "");
    DBG("OTA_SEQ", "CLEANUP PHASE");
    DBG("OTA_SEQ", "═════════════════════════════════════════════════");
    
    // Close the SPIFFS firmware file handle opened by fw_process().
    // (No heap buffer to free — streaming approach uses no full-image malloc.)
    fw_close();
    g_fwRawSize = 0;
    g_fwSize    = 0;
    
    if (SPIFFS.exists(OTA_BL_FW_PATH)) {
        DBG("OTA_SEQ", "Removing temporary firmware file from SPIFFS...");
        SPIFFS.remove(OTA_BL_FW_PATH);
    }
    
    g_uploadInProgress = false;
    g_uploadProgress = 0;
    
    const char *finalState = "";
    switch(g_otaState) {
        case OTA_STATE_SUCCESS:   finalState = "✅ SUCCESS"; break;
        case OTA_STATE_FAILED:    finalState = "❌ FAILED"; break;
        case OTA_STATE_CANCELLED: finalState = "⚠️ CANCELLED"; break;
        default:                  finalState = "❓ UNKNOWN"; break;
    }
    
    DBG("OTA_SEQ", "Final state: %s", finalState);
    if (g_lastError.length()) {
        DBG("OTA_SEQ", "Error detail: %s", g_lastError.c_str());
    }
    
    DBG("OTA_SEQ", "Total duration: %u ms (%.2f sec)", 
        (unsigned)(millis() - g_otaStartTime),
        (millis() - g_otaStartTime) / 1000.0);
    
    DBG("OTA_SEQ", "Waiting 2 seconds for MQTT status delivery...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // ═══════════════════════════════════════════════════════════════════════
    // ROLLBACK TO APP0
    // ═══════════════════════════════════════════════════════════════════════
    
    DBG("OTA_SEQ", "");
    DBG("OTA_SEQ", "╔══════════════════════════════════════════════╗");
    DBG("OTA_SEQ", "║   🔄  ROLLING BACK TO APP0                   ║");
    DBG("OTA_SEQ", "║       (Main Application)                     ║");
    DBG("OTA_SEQ", "╚══════════════════════════════════════════════╝");

    const esp_partition_t *app0 = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);

    if (!app0) {
        DBG_ERR("OTA_SEQ", "FATAL: Could not find app0 partition!");
        DBG_ERR("OTA_SEQ", "Check partitions.csv in flash");
        DBG_ERR("OTA_SEQ", "Device stuck — reset manually to recover");
        DIVIDER("OTA_SEQ");
        while(true) {
            delay(5000);
        }
        return;
    }

    DBG("OTA_SEQ", "Found app0 partition:");
    DBG("OTA_SEQ", "  Label   : %s", app0->label);
    DBG("OTA_SEQ", "  Subtype : 0x%02X", app0->subtype);
    DBG("OTA_SEQ", "  Offset  : 0x%08X", app0->address);
    DBG("OTA_SEQ", "  Size    : %u bytes", app0->size);

    DBG("OTA_SEQ", "Calling esp_ota_set_boot_partition(app0)...");

    esp_err_t err = esp_ota_set_boot_partition(app0);
    
    if (err != ESP_OK) {
        DBG_ERR("OTA_SEQ", "FAILED to set boot partition!");
        DBG_ERR("OTA_SEQ", "Error: %s", esp_err_to_name(err));
        DBG_ERR("OTA_SEQ", "Device stuck — reset manually to recover");
        DIVIDER("OTA_SEQ");
        while(true) {
            delay(5000);
        }
        return;
    }
    
    DBG("OTA_SEQ", "✅ Boot partition successfully set to app0");
    DBG("OTA_SEQ", "");
    DBG("OTA_SEQ", "When ESP32 restarts:");
    DBG("OTA_SEQ", "  1. ROM bootloader validates app0");
    DBG("OTA_SEQ", "  2. Clears app1 'pending' flag");
    DBG("OTA_SEQ", "  3. Boots app0 (main application)");
    DBG("OTA_SEQ", "  4. Normal operation resumes");
    DBG("OTA_SEQ", "");

    /* Write NVS flag so app0 can announce "returned from CAN OTA" after reboot */
    {
        nvs_handle_t nvsH;
        if (nvs_open(OTA_BL_NVS_NAMESPACE, NVS_READWRITE, &nvsH) == ESP_OK) {
            const char *result = (g_otaState == OTA_STATE_SUCCESS) ? "can_success" : "can_failed";
            nvs_set_str(nvsH, "can_ota_result", result);
            nvs_commit(nvsH);
            nvs_close(nvsH);
            DBG("OTA_SEQ", "NVS can_ota_result = %s", result);
        }
    }

    DBG("OTA_SEQ", "Restarting ESP32 in 3 seconds...");
    DBG("OTA_SEQ", "(You will see 'ets Jun  8 2016 00:22:57' message)");
    
    DIVIDER("OTA_SEQ");
    Serial.flush();
    
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    ESP.restart();
    
    DBG_ERR("OTA_SEQ", "ERROR: esp_restart() did not reboot!");
    DBG_ERR("OTA_SEQ", "Device stuck in bootloader");
    
    while(true) {
        delay(5000);
        Serial.println("⚠️  Device should have restarted by now...");
        Serial.println("⚠️  Please reset manually (press RST button)");
    }
}
#endif /* OTA_BL_MODEM_OTA */

// ─────────────────────────────────────────────────────────────────────────────
//  Serial debug commands
// ─────────────────────────────────────────────────────────────────────────────
static void print_status(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    
    const char *stateStr = "";
    switch(g_otaState) {
        case OTA_STATE_READY:         stateStr = "Ready"; break;
        case OTA_STATE_IDLE:         stateStr = "idle"; break;
        case OTA_STATE_INITIALIZING: stateStr = "initializing"; break;
        case OTA_STATE_DOWNLOADING:  stateStr = "downloading"; break;
        case OTA_STATE_VERIFYING:    stateStr = "verifying"; break;
        case OTA_STATE_FLASHING:     stateStr = "flashing"; break;
        case OTA_STATE_SUCCESS:      stateStr = "success"; break;
        case OTA_STATE_FAILED:       stateStr = "failed"; break;
        case OTA_STATE_CANCELLED:    stateStr = "cancelled"; break;
        default:                     stateStr = "unknown"; break;
    }
    
    Serial.println();
    Serial.println("[STATUS] ══════════════════════════════════════════");
    Serial.printf( "[STATUS] Partition    : %-8s subtype=0x%02X offset=0x%08X size=%u\n",
                   run?run->label:"?", run?run->subtype:0, run?run->address:0, run?run->size:0);
    Serial.printf( "[STATUS] CAN Running  : %s\n",     g_canRunning       ? "YES" : "NO");
    Serial.printf( "[STATUS] Upload Active: %s\n",     g_uploadInProgress ? "YES" : "NO");
    Serial.printf( "[STATUS] OTA State    : %s\n",     stateStr);
    Serial.printf( "[STATUS] OTA Cancel   : %s\n",     g_otaCancel        ? "YES" : "NO");
    Serial.printf( "[STATUS] Progress     : %u%%\n",   g_uploadProgress);
    Serial.printf( "[STATUS] MQTT Conn    : %s\n",     g_mqttConn         ? "YES" : "NO");
    Serial.printf( "[STATUS] DL Pending   : %s\n",     g_downloadPending  ? "YES" : "NO");
    if (g_pendingUrl[0]) Serial.printf("[STATUS]   URL    : %s\n", g_pendingUrl);
    if (g_pendingSha[0]) Serial.printf("[STATUS]   SHA256 : %s\n", g_pendingSha);
    Serial.printf( "[STATUS] Last Error   : %s\n",     g_lastError.length() ? g_lastError.c_str() : "(none)");
    Serial.printf( "[STATUS] Free Heap    : %u bytes\n", ESP.getFreeHeap());
    Serial.printf( "[STATUS] SPIFFS       : %u used / %u total\n",
                   SPIFFS.usedBytes(), SPIFFS.totalBytes());
    Serial.println("[STATUS] Commands:");
    Serial.println("[STATUS]   ?         = Status");
    Serial.println("[STATUS]   C         = Cancel OTA");
    Serial.println("[STATUS]   t         = CAN test");
    Serial.println("[STATUS]   s         = CAN scan");
    Serial.println("[STATUS]   m         = Manual trigger");
    Serial.println("[STATUS]   r         = Reset CAN");
    Serial.println("[STATUS] ══════════════════════════════════════════");
    Serial.println();
}

static void handle_serial_cmd(char cmd)
{
    switch (cmd) {
        case '?': 
            print_status(); 
            break;

        case 'c': case 'C':
            DBG("CMD", "");
            DBG("CMD", "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
            DBG("CMD", "  🛑  CANCEL OTA COMMAND RECEIVED (serial)");
            DBG("CMD", "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
            
            if (!g_uploadInProgress) {
                g_otaCancel = true;
                g_bootloaderRestart = true; 
                DBG("CMD", "✅ Cancel flag SET");
                DBG("CMD", "   OTA will abort at next checkpoint");
                DBG("CMD", "✅ Cancel flag SET, rolling back to app0...");
            }
            break;

        case 't': case 'T':
            DBG("CMD", "CAN test — listening 3 s...");
            if (!g_canRunning && !can_init_bl()) { 
                DBG_ERR("CMD","CAN init fail"); 
                break; 
            }
            { 
                unsigned long s=millis(); 
                int c=0;
                while(millis()-s<3000){
                    twai_message_t m; 
                    if(can_recv(m,10))c++; 
                }
                DBG("CMD","Test done — %d messages in 3 s", c); 
            }
            break;

        case 's': case 'S':
            DBG("CMD", "CAN scan — 5 s...");
            if (!g_canRunning && !can_init_bl()) { 
                DBG_ERR("CMD","CAN init fail"); 
                break; 
            }
            { 
                unsigned long s=millis(); 
                int total=0;
                while(millis()-s<5000){
                    twai_message_t m;
                    if(can_recv(m,10)){
                        total++;
                        const char *lbl = (m.identifier==OTA_BL_CAN_ID)       ? "BOOTLOADER" :
                                          (m.identifier==OTA_BL_CAN_HEART_ID)  ? "HEARTBEAT"  : "OTHER";
                        DBG("CMD","  [%s] ID=0x%03X Len=%d", lbl, m.identifier, m.data_length_code);
                    }
                }
                DBG("CMD","Scan done — %d msgs", total); 
            }
            break;

        case 'm': case 'M':
            DBG("CMD","Manual bootloader trigger...");
            if (!g_canRunning && !can_init_bl()) { 
                DBG_ERR("CMD","CAN init fail"); 
                break; 
            }
            { 
                uint8_t t[]={OTA_BL_CMD_TRIGGER,OTA_BL_CMD_TRIGGER_INV,
                             OTA_BL_CMD_TRIGGER,OTA_BL_CMD_TRIGGER_INV,
                             OTA_BL_CMD_TRIGGER,OTA_BL_CMD_TRIGGER_INV,
                             OTA_BL_CMD_TRIGGER,OTA_BL_CMD_TRIGGER_INV};
                can_send(OTA_BL_CAN_ID,t,8);
                DBG("CMD","Trigger sent"); 
            }
            break;

        case 'r': case 'R':
            DBG("CMD","Resetting CAN...");
            if (g_canRunning){
                twai_stop();
                twai_driver_uninstall();
                g_canRunning=false;
                DBG("CMD","CAN stopped");
            }
            can_init_bl();
            break;

        default: 
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  RTOS Task Functions (NEW - Core separation)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief MQTT polling task (runs on Core 0)
 *
 * Continuously polls MQTT for download commands and cancellations.
 * Non-blocking: does NOT wait on CAN operations.
 */
// ─────────────────────────────────────────────────────────────────────────────
//  mqtt_is_alive() — lightweight modem/MQTT connectivity probe.
//
//  Sends AT+QMTCONN? and expects "+QMTCONN: 0,3" (connected) within 5 s.
//  Falls back to a bare AT echo test if the QMTCONN query yields nothing,
//  so a modem that lost its MQTT session is still distinguished from a
//  modem that is completely unresponsive.
//
//  Returns true  → MQTT session is up (or at least the modem is alive)
//          false → modem / MQTT is unreachable
// ─────────────────────────────────────────────────────────────────────────────
#if OTA_BL_MODEM_OTA   /* EC200 remote-OTA removed — compiled out (see run() + top) */
static bool mqtt_is_alive(void)
{
    // Query MQTT connection status (state 3 = connected)
    gsm_send("AT+QMTCONN?\r\n", 0);
    if (gsm_wait("+QMTCONN: 0,3", 5000)) {
        return true;
    }

    // QMTCONN query gave no useful answer — try a bare AT ping
    gsm_send("AT\r\n", 0);
    if (gsm_wait("OK", 3000)) {
        // Modem is alive but MQTT session dropped — attempt reconnect
        DBG_WARN("MQTT_WD", "MQTT session lost — attempting reconnect");
        if (mqtt_connect_bl()) {
            mqtt_subscribe_bl();
            DBG("MQTT_WD", "✅ MQTT reconnected");
            return true;
        }
        DBG_ERR("MQTT_WD", "MQTT reconnect failed");
        return false;
    }

    // Modem not responding at all
    return false;
}

static void mqtt_poll_task(void *pvParam)
{
    DBG("MQTT_TASK", "Started on Core %d", xPortGetCoreID());

    // Initialise the idle timer the moment this task starts.
    // ota_bootloader_run() has already set g_otaState = OTA_STATE_READY by now.
    g_blIdleStartMs  = millis();
    g_lastMqttCheckMs = millis();

    while (true) {
        // ── Skip all UART work during active OTA download (contention avoidance) ──
        if (g_pauseMqttPolling) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        uint32_t now = millis();

        // ──────────────────────────────────────────────────────────────────────
        // Feature 1 — IDLE TIMEOUT
        //
        // If no OTA command has been received for BL_IDLE_TIMEOUT_MS and we
        // are not currently running an OTA sequence, bail back to app0.
        // The timer is reset whenever a download-pending flag is set
        // (i.e. mqtt_process_qmtrecv populated g_pendingUrl).
        // ──────────────────────────────────────────────────────────────────────
        if (!g_otaInProgress && !g_downloadPending) {
            if ((now - g_blIdleStartMs) >= BL_IDLE_TIMEOUT_MS) {
                DBG("MQTT_TASK", "");
                DBG("MQTT_TASK", "╔════════════════════════════════════════════════╗");
                DBG("MQTT_TASK", "║  IDLE TIMEOUT — no command in 5 min           ║");
                DBG("MQTT_TASK", "║  Returning to app0 (unchanged)                ║");
                DBG("MQTT_TASK", "╚════════════════════════════════════════════════╝");
                mqtt_publish_status(STATUS_OTA_CANCELLED,
                                    "Idle timeout — returning to app0", 0);
                vTaskDelay(pdMS_TO_TICKS(1000));
                bail_to_best_app("idle timeout — no command received");
                // bail_to_best_app calls esp_restart() so we never reach here.
            }
        } else {
            // Reset the idle timer whenever something is happening so the clock
            // only counts down true dead-air time.
            g_blIdleStartMs = now;
        }

        // ──────────────────────────────────────────────────────────────────────
        // Feature 2 — MQTT / INTERNET WATCHDOG
        //
        // Every BL_MQTT_CHECK_INTERVAL_MS, probe the modem MQTT session.
        // Consecutive failures increment g_mqttFailCount.  Once it reaches
        // BL_MQTT_MAX_FAILURES the ESP32 restarts (modem UART reset included).
        // The watchdog is suspended during active OTA flashing so it does not
        // interfere with large HTTP downloads; it resumes immediately after.
        // ──────────────────────────────────────────────────────────────────────
        if (!g_otaInProgress &&
            (now - g_lastMqttCheckMs) >= BL_MQTT_CHECK_INTERVAL_MS) {

            g_lastMqttCheckMs = now;

            DBG("MQTT_WD", "Checking MQTT/modem connectivity...");
            if (mqtt_is_alive()) {
                g_mqttFailCount = 0;
                g_mqttConn      = true;
                DBG("MQTT_WD", "✅ MQTT alive (fail count reset)");
                // Publish a bootloader-mode heartbeat so the cloud side always
                // knows the device is still sitting in the bootloader waiting
                // for a command (useful if the 5-minute idle window is long).
                mqtt_publish_status(STATUS_BOOTLOADER_MODE,
                                    "ESP32 in OTA bootloader mode — awaiting firmware URL", 0);
            } else {
                g_mqttFailCount++;
                g_mqttConn = false;
                DBG_WARN("MQTT_WD", "⚠️  MQTT/modem unreachable (failure %u/%u)",
                         g_mqttFailCount, BL_MQTT_MAX_FAILURES);

                if (g_mqttFailCount >= BL_MQTT_MAX_FAILURES) {
                    DBG_ERR("MQTT_WD", "");
                    DBG_ERR("MQTT_WD", "╔════════════════════════════════════════════════╗");
                    DBG_ERR("MQTT_WD", "║  MQTT WATCHDOG — MAX FAILURES REACHED         ║");
                    DBG_ERR("MQTT_WD", "║  Rebooting ESP32 to recover connectivity      ║");
                    DBG_ERR("MQTT_WD", "╚════════════════════════════════════════════════╝");
                    Serial.flush();
                    vTaskDelay(pdMS_TO_TICKS(500));
                    // Full ESP32 reboot — the modem power-on sequence in
                    // modem_bringup() will re-initialise the modem as well.
                    esp_restart();
                }
            }
        }

        // Normal MQTT receive polling (non-blocking, 20 ms UART timeout inside)
        mqtt_poll_bl();
        vTaskDelay(pdMS_TO_TICKS(50));  // Yield, poll again in 50 ms
    }
}
#endif /* OTA_BL_MODEM_OTA */

/**
 * @brief Serial command task (runs on Core 0)
 *
 * Handles user input: ?, t, s, m, r, c (cancel)
 * Non-blocking: only processes if data available.
 */
static void serial_cmd_task(void *pvParam)
{
    DBG("SERIAL_TASK", "Started on Core %d", xPortGetCoreID());
    
    while (true) {
        if (Serial.available()) {
            char cmd = Serial.read();
            handle_serial_cmd(cmd);
        }
        vTaskDelay(pdMS_TO_TICKS(50));  // Poll serial every 50ms
    }
}

/**
 * @brief CAN OTA execution task (runs on Core 1)
 *
 * Waits for download-ready signal from mqtt_poll_task, then:
 *   1. Verifies SHA256
 *   2. Initializes bootloader on remote MCU
 *   3. Flashes firmware via CAN
 *   4. Verifies flash
 *   5. Resets remote MCU
 *
 * Blocks only on semaphore (yields CPU), frees MQTT polling on Core 0.
 */
#if OTA_BL_MODEM_OTA   /* EC200 remote-OTA removed — compiled out (see run() + top) */
static void can_ota_task(void *pvParam)
{
    DBG("CAN_OTA_TASK", "Started on Core %d", xPortGetCoreID());
    
    while (true) {
        // Wait for semaphore (download-ready signal from MQTT task)
        // Blocks task but frees Core 1 for other operations
        if (xSemaphoreTake(g_downloadReadySem, pdMS_TO_TICKS(1000)) == pdTRUE) {
            xSemaphoreTake(g_otaMutex, portMAX_DELAY);  // Lock shared data

            // If an operator is flashing this ESP32 over CAN right now, that
            // session owns app0 + the CAN bus — do NOT start a cloud flash on top.
            if (su_active) {
                DBG("CAN_OTA_TASK", "CAN push flash in progress — skipping cloud OTA");
                xSemaphoreGive(g_otaMutex);
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
            {
                DBG("CAN_OTA_TASK", "Download ready, starting OTA sequence...");
                g_otaInProgress = true;
                g_otaCancel = false;
                g_pauseMqttPolling = true;  // Signal mqtt_poll_task to stop UART reads

                // Sync barrier: wait for mqtt_poll_task to finish any in-progress
                // uart_read_bytes call (max 20 ms timeout inside mqtt_poll_bl) before
                // we touch the UART ourselves for HTTP download.  Without this there
                // is a window where both tasks read from the same UART simultaneously,
                // corrupting the HTTP stream header or swallowing CONNECT bytes.
                vTaskDelay(pdMS_TO_TICKS(150));
                
                // Branch on OTA type
                char localType[16];
                strncpy(localType, g_pendingType, sizeof(localType) - 1);
                localType[sizeof(localType) - 1] = '\0';

                if (strcmp(localType, OTA_TYPE_ESP32) == 0) {
                    DBG("CAN_OTA_TASK", "OTA type: ESP32 self-update");
                    g_pauseMqttPolling = true;
                    bool flashPhaseStarted = do_esp32_ota_internal();
                    // Only reached on failure (esp_restart not called on success).
                    if (flashPhaseStarted) {
                        // app0 was partially erased/written — boot app1 (last-known-good backup)
                        DBG_ERR("CAN_OTA_TASK", "ESP32 OTA flash failed — booting app1 (backup)");
                        mqtt_publish_status(STATUS_ESP32_OTA_FAILED,
                                            "Flash failed — booting backup (app1)", 0);
                        const esp_partition_t *app1fb = esp_partition_find_first(
                            ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);
                        if (app1fb) esp_ota_set_boot_partition(app1fb);
                    } else {
                        // app0 was never touched — safe to return to it
                        DBG_ERR("CAN_OTA_TASK", "ESP32 OTA pre-flash failed — returning to app0");
                        mqtt_publish_status(STATUS_ESP32_OTA_FAILED,
                                            "OTA failed before flash — returning to app0", 0);
                        const esp_partition_t *app0fb = esp_partition_find_first(
                            ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
                        if (app0fb) esp_ota_set_boot_partition(app0fb);
                    }
                    Serial.flush();
                    vTaskDelay(pdMS_TO_TICKS(3000));
                    esp_restart();
                } else {
                    DBG("CAN_OTA_TASK", "OTA type: CAN master flash");
                    do_can_ota_internal();
                }

                g_pauseMqttPolling = false;  // Resume MQTT after OTA completes
                vTaskDelay(pdMS_TO_TICKS(100));  // Let mqtt_poll_task resume
            }
            xSemaphoreGive(g_otaMutex);  // Unlock shared data
            
            g_otaInProgress = false;
            DBG("CAN_OTA_TASK", "OTA sequence complete, MQTT polling resumed");
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));  // Prevent busy-wait
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Bootloader-mode announcement
//
//  Called once after MQTT is subscribed, before RTOS tasks are started.
//  Broadcasts on two buses simultaneously so both cloud and local CAN nodes
//  know the ESP32 is sitting in bootloader mode waiting for a firmware URL.
//
//  MQTT  → MQTT_TOPIC_OTA_RESPONSE
//          {"code":150,"msg":"ESP32 in OTA bootloader mode — awaiting firmware URL","progress":0}
//
//  CAN   → ID 0x700 (standard frame, all nodes)
//          8 bytes: B0 07 B0 07 B0 07 B0 07  (0xB007 = "BOOT" mnemonic × 4)
//          Sent 3 times with 200 ms spacing so no node misses it.
// ─────────────────────────────────────────────────────────────────────────────
static void bl_announce_bootloader_mode(void)
{
    DIVIDER("BL_ANNOUNCE");
    DBG("BL_ANNOUNCE", "╔══════════════════════════════════════════════╗");
    DBG("BL_ANNOUNCE", "║   BOOTLOADER MODE ANNOUNCEMENT               ║");
    DBG("BL_ANNOUNCE", "╚══════════════════════════════════════════════╝");

    // ── MQTT broadcast ────────────────────────────────────────────────────
    const char *payload =
        "{\"code\":150,\"msg\":\"ESP32 in OTA bootloader mode — awaiting firmware URL\",\"progress\":0}";

    char cmd[512];
    int cmdLen = snprintf(cmd, sizeof(cmd),
                          "AT+QMTPUB=0,1,0,0,\"%s\",%u\r\n",
                          MQTT_TOPIC_OTA_RESPONSE, (unsigned)strlen(payload));
    uart_write_bytes(OTA_BL_GSM_UART, (const uint8_t *)cmd, cmdLen);
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_write_bytes(OTA_BL_GSM_UART, (const uint8_t *)payload, strlen(payload));

    if (gsm_wait("+QMTPUB: 0,1,0", 3000)) {
        DBG("BL_ANNOUNCE", "✅ MQTT bootloader-mode status published");
    } else {
        DBG_WARN("BL_ANNOUNCE", "MQTT publish ACK timeout — message may still have been sent");
    }

    // ── CAN broadcast ─────────────────────────────────────────────────────
    // Initialise CAN driver if it is not already running.
    // Failure is non-fatal: the MQTT message alone is sufficient; CAN will be
    // re-initialised when the actual OTA flash sequence begins.
    bool canWasRunning = g_canRunning;
    if (!g_canRunning) {
        DBG("BL_ANNOUNCE", "CAN not running — initialising for announcement...");
        if (!can_init_bl()) {
            DBG_WARN("BL_ANNOUNCE", "CAN init failed — skipping CAN announcement");
            DIVIDER("BL_ANNOUNCE");
            return;
        }
    }

    const uint8_t announceFrame[8] = {
        OTA_BL_ANNOUNCE_BYTE_0, OTA_BL_ANNOUNCE_BYTE_1,
        OTA_BL_ANNOUNCE_BYTE_0, OTA_BL_ANNOUNCE_BYTE_1,
        OTA_BL_ANNOUNCE_BYTE_0, OTA_BL_ANNOUNCE_BYTE_1,
        OTA_BL_ANNOUNCE_BYTE_0, OTA_BL_ANNOUNCE_BYTE_1
    };

    int sent = 0;
    for (int i = 0; i < 3; i++) {
        if (can_send(OTA_BL_ANNOUNCE_CAN_ID, announceFrame, 8)) {
            sent++;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    DBG("BL_ANNOUNCE", "✅ CAN bootloader-mode frame sent %d/3 times (ID=0x%03X)",
        sent, OTA_BL_ANNOUNCE_CAN_ID);

    // If CAN was not running before we started, leave it running — it will be
    // needed for the OTA flash sequence anyway.  Log the state clearly.
    if (!canWasRunning) {
        DBG("BL_ANNOUNCE", "CAN left running (needed for OTA flash)");
    }

    DIVIDER("BL_ANNOUNCE");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Bail-out helper (IOT/LCD parity): recover to a valid app, or — if the device
//  is bricked (neither app0 nor app1 bootable) — stay in factory as the CAN
//  flash slave so it can still be rescued over CAN. Never force-boots a known-
//  bad app0 (that could loop factory→bad-app→factory).
//
//  Called on any infrastructure failure (modem, MQTT, RTOS, storage) so the
//  device never gets stuck, and never bricks into a boot loop.
// ─────────────────────────────────────────────────────────────────────────────
static void bail_to_best_app(const char *reason)
{
    DBG_ERR("OTA_BL", "BAIL: %s", reason);
    EVLOG_FAIL("OTA", "bail: %s", reason ? reason : "");

    if (recover_boot_to_app(reason)) return;   /* reboots into app0/app1 — never returns */

    /* Neither app0 nor app1 bootable → reboot into factory MQTT OTA to keep
     * trying to re-download over the cloud (self-heal). The delay + modem
     * bringup on the next boot keep this from becoming a tight reboot loop. */
    DBG_ERR("OTA_BL", "No bootable app — rebooting to factory MQTT OTA (cloud re-download)");
    Serial.flush();
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
}
#endif /* OTA_BL_MODEM_OTA */

// ═════════════════════════════════════════════════════════════════════════════
//  Factory WiFi service-UI hooks — web firmware flash (app0), backup status, reboot.
//  The web flash reuses the SAME esp_ota primitives as the CAN slave; the
//  s_web_ota_begun / su_begun cross-check keeps the two paths mutually exclusive so
//  they can never corrupt app0 by running at once.
// ═════════════════════════════════════════════════════════════════════════════
int      ota_bl_backup_status(void)   { return s_backup_status; }
unsigned ota_bl_web_flash_bytes(void) { return s_web_ota_bytes; }

bool ota_bl_web_flash_begin(void)
{
    if (su_begun) { DBG_ERR("WEB_OTA", "CAN flash in progress — refusing web flash"); return false; }
    if (s_web_ota_begun) { esp_ota_abort(s_web_ota); s_web_ota_begun = false; }

    s_web_ota_bytes = 0;
    s_web_part = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
    if (!s_web_part) { DBG_ERR("WEB_OTA", "app0 partition not found"); return false; }
    if (esp_ota_begin(s_web_part, OTA_SIZE_UNKNOWN, &s_web_ota) != ESP_OK) {
        DBG_ERR("WEB_OTA", "esp_ota_begin failed"); return false;
    }
    s_web_ota_begun = true;
    DBG("WEB_OTA", "begin → %s @ 0x%08lX", s_web_part->label, (unsigned long)s_web_part->address);
    EVLOG_INFO("OTA", "web flash begin → %s", s_web_part->label);
    return true;
}

bool ota_bl_web_flash_write(const uint8_t *data, unsigned len)
{
    if (!s_web_ota_begun) return false;
    if (esp_ota_write(s_web_ota, data, len) != ESP_OK) {
        DBG_ERR("WEB_OTA", "esp_ota_write failed at %u B", s_web_ota_bytes);
        esp_ota_abort(s_web_ota); s_web_ota_begun = false; return false;
    }
    s_web_ota_bytes += len;
    return true;
}

bool ota_bl_web_flash_end(void)
{
    if (!s_web_ota_begun) return false;
    s_web_ota_begun = false;
    if (esp_ota_end(s_web_ota) != ESP_OK) {              /* validates the image */
        DBG_ERR("WEB_OTA", "esp_ota_end/verify failed"); EVLOG_FAIL("OTA", "web flash verify failed"); return false;
    }
    if (esp_ota_set_boot_partition(s_web_part) != ESP_OK) {
        DBG_ERR("WEB_OTA", "set_boot_partition failed"); EVLOG_FAIL("OTA", "web flash set-boot failed"); return false;
    }
    /* Flag app0 OTA pending so app0 publishes success on its first clean boot (same as CAN/MQTT). */
    nvs_handle_t nvs;
    if (nvs_open(OTA_BL_NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_str(nvs, OTA_BL_NVS_KEY_STATE, OTA_BL_NVS_VAL_PENDING);
        nvs_commit(nvs); nvs_close(nvs);
    }
    DBG("WEB_OTA", "OK, %u B → %s; boot set", s_web_ota_bytes, s_web_part->label);
    EVLOG_OK("OTA", "web flash OK, %u B → app0", s_web_ota_bytes);
    return true;
}

void ota_bl_web_flash_abort(void)
{
    if (s_web_ota_begun) { esp_ota_abort(s_web_ota); s_web_ota_begun = false; }
    EVLOG_WARN("OTA", "web flash aborted");
}

void ota_bl_reboot_to_app(void)
{
    DBG("OTA_BL", "Service UI: reboot to app requested");
    EVLOG_INFO("OTA", "service UI reboot to app");
#if CONFIG_FACTORY_WIFI
    wifi_service_stop();
#endif
    recover_boot_to_app("web reboot");   /* boots app0/app1 if valid — may not return */
    esp_restart();                        /* fallback: no valid app */
}

// ─────────────────────────────────────────────────────────────────────────────
//  Main bootloader loop (refactored for RTOS)
// ─────────────────────────────────────────────────────────────────────────────
void ota_bootloader_run(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);

    /* Bring up the SD event log in the factory partition too (it mounts the card itself here,
       since sd_log_init() only runs in app0). Best-effort — never blocks the flasher. */
    event_log_init();
    EVLOG_INFO("BOOT", "factory bootloader entered");

    // ── Check if we're here due to a hardware rollback from a failed ESP32 OTA ──
    {
        esp_err_t nvsErr = nvs_flash_init();
        if (nvsErr == ESP_ERR_NVS_NO_FREE_PAGES || nvsErr == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            nvs_flash_erase(); nvs_flash_init();
        }
        nvs_handle_t nvs;
        char nvsVal[16] = {0};
        size_t nvsLen   = sizeof(nvsVal);
        if (nvs_open(OTA_BL_NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
            if (nvs_get_str(nvs, OTA_BL_NVS_KEY_STATE, nvsVal, &nvsLen) == ESP_OK &&
                strcmp(nvsVal, OTA_BL_NVS_VAL_PENDING) == 0) {
                g_esp32OtaRolledBack = true;
                nvs_erase_key(nvs, OTA_BL_NVS_KEY_STATE);
                nvs_commit(nvs);
                DBG("OTA_BL", "⚠️  Hardware rollback detected — app0 OTA failed!");
                EVLOG_FAIL("OTA", "hardware rollback detected — app0 OTA failed");
            }
            nvs_close(nvs);
        }
    }

    // ── Why are we in factory? Consume the two entry sentinels ───────────────
    // canMode         : CAN self-update was requested (0x0C enter frame).
    // entryRequested  : ANY deliberate flash request (CAN or MQTT). If neither
    //                   is set we booted factory spontaneously (power event, ROM
    //                   fallback after a bad app, or a flash that never committed
    //                   boot=app0).
    bool canMode = false;
    {
        nvs_handle_t nvs;
        char  canVal[16] = {0};
        size_t canLen    = sizeof(canVal);
        if (nvs_open(OTA_BL_NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
            if (nvs_get_str(nvs, OTA_BL_NVS_KEY_CANMODE, canVal, &canLen) == ESP_OK &&
                strcmp(canVal, OTA_BL_NVS_VAL_PENDING) == 0) {
                canMode = true;
                nvs_erase_key(nvs, OTA_BL_NVS_KEY_CANMODE);   /* one-shot */
                nvs_commit(nvs);
            }
            nvs_close(nvs);
        }
    }
    bool entryRequested = (s_flash_entry_requested == ENTRY_REQUEST_MAGIC) || canMode;
    s_flash_entry_requested = 0;   /* consume */

    // ── Spontaneous factory boot → recover immediately ──────────────────────
    // No flash was requested (power event, ROM fallback after a bad app, or a
    // flash that never committed boot=app0). Recover to a valid app. If BOTH
    // app0 and app1 are invalid, fall through into the factory MQTT OTA
    // downloader so the device can re-download a good image over the cloud.
    if (!entryRequested) {
        DBG("OTA_BL", "Factory boot with NO flash request — recovering to a valid app");
        recover_boot_to_app("spontaneous factory boot");   /* reboots if app0/app1 bootable */
        DBG("OTA_BL", "No valid app — staying in factory MQTT OTA to re-download firmware");
        /* fall through to the MQTT downloader path below */
    }

    // ── BACK UP app0 → app1 (BEFORE anything erases app0) — ONLY if armed ────
    //
    // The app-side trigger armed s_backup_sentinel only when it was running from
    // app0. Back up only then, so we never copy a stale/bad app0 over a good
    // app1 backup (which would happen if the device had booted from app1 after a
    // recovery fallback). Consume the sentinel either way. Runs before the CAN
    // slave (UNLOCK) or the download path erases app0; app0 is read explicitly.
    if (s_backup_sentinel == BACKUP_SENTINEL_MAGIC) {
        s_backup_sentinel = 0;                 /* consume */
        DBG("OTA_BL", "Backup armed (entered from app0) — backing up app0 → app1");
        bool bok = backup_app0_to_app1();
        s_backup_status = bok ? OTA_BL_BACKUP_PASS : OTA_BL_BACKUP_FAIL;
        if (bok) EVLOG_OK("OTA",   "app0 → app1 backup OK");
        else     EVLOG_FAIL("OTA", "app0 → app1 backup FAILED");
    } else {
        DBG("OTA_BL", "Backup NOT armed (not entered from app0; sentinel=0x%08X) — "
                      "skipping app0 → app1 backup to protect the existing app1",
            (unsigned)s_backup_sentinel);
        s_backup_sentinel = 0;                 /* clear stale/garbage */
        s_backup_status = OTA_BL_BACKUP_SKIPPED;
    }

#if CONFIG_FACTORY_WIFI
    /* Raise the hidden service AP + web UI (updater / CAN-baud / reboot). Runs in its own task
       and is torn down on the reboot that exits factory. Up for BOTH the CAN self-update path
       (below, never returns) and the MQTT download path. */
    wifi_service_start();
    EVLOG_INFO("WIFI", "factory service AP up (SSID %s, hidden)", CONFIG_FACTORY_WIFI_SSID);
    Serial.printf("[WIFI] factory service — free heap=%u internal=%u\n",
                  (unsigned)esp_get_free_heap_size(),
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
#endif

    // ── CAN SELF-UPDATE MODE? — skip ALL checks except the backup above ──────
    //
    // If the 0x0C enter frame put us here, run the CAN slave receiver and flash
    // app0 from firmware pushed over CAN. NO modem / MQTT / SPIFFS / download.
    // Never returns (the slave resets on success, or recovers on timeout).
    if (canMode) {
        // Mark factory valid so the ROM always has a last-resort fallback
        // during the (potentially long) CAN flash session.
        esp_ota_mark_app_valid_cancel_rollback();
        run_can_selfupdate_slave();                 /* never returns */
    }

    // ── No CAN self-update requested. The EC200 remote-OTA download path was
    //    REMOVED; remote flashing now happens through the factory WiFi web updater
    //    raised above (wifi_service_start). Mark factory valid, start the serial
    //    console, then idle — periodically trying to recover to a valid app. ─────
    {
        esp_err_t valErr = esp_ota_mark_app_valid_cancel_rollback();
        if (valErr == ESP_OK)
            DBG("OTA_BL", "factory marked valid — ROM last-resort fallback");
        else
            DBG_WARN("OTA_BL", "mark_app_valid: %s (first JTAG flash? — safe to ignore)",
                     esp_err_to_name(valErr));
    }

    /* Bench serial console (?, t, s, m, r, C). */
    xTaskCreatePinnedToCore(serial_cmd_task, "Serial Cmd", 8192, NULL, 1, NULL, 0);

    DBG("OTA_BL", "Factory idle — flash via WiFi web updater; no EC200 remote OTA.");
    EVLOG_INFO("OTA", "factory idle — awaiting web flash (EC200 removed)");

    /* If a valid app exists, recover to it after the idle window; otherwise stay up
       so the web updater can reflash. A successful web flash reboots us out of here. */
    uint32_t idleStart = millis();
    while (true) {
        if ((uint32_t)(millis() - idleStart) >= BL_IDLE_TIMEOUT_MS) {
            recover_boot_to_app("factory idle timeout");   /* reboots if app0/app1 valid */
            idleStart = millis();                          /* no valid app → keep serving web UI */
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Called from app0 init_task after MQTT is connected.
//  Checks NVS for a pending ESP32 OTA result and publishes success/clears flag.
//  Requires CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y in sdkconfig for the
//  hardware rollback (failure) path to work — success path works without it.
// ─────────────────────────────────────────────────────────────────────────────
void ota_bootloader_check_esp32_ota_result(void)
{
    nvs_handle_t nvs;
    char nvsVal[16] = {0};
    size_t nvsLen   = sizeof(nvsVal);

    if (nvs_open(OTA_BL_NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) return;
    esp_err_t err = nvs_get_str(nvs, OTA_BL_NVS_KEY_STATE, nvsVal, &nvsLen);
    nvs_close(nvs);

    if (err != ESP_OK || strcmp(nvsVal, OTA_BL_NVS_VAL_PENDING) != 0) return;

    // ── New app0 booted successfully — OTA succeeded ──────────────────────
    Serial.println("[ESP32_OTA] ✅ New firmware booted — OTA SUCCESS");
    ESP_LOGI("ESP32_OTA", "ESP32 self-OTA succeeded, new firmware is running");
    EVLOG_OK("OTA", "ESP32 self-OTA success — new firmware running");

    // Cancel hardware rollback timer (no-op if rollback not enabled in sdkconfig)
    esp_ota_mark_app_valid_cancel_rollback();

    // Clear NVS flag
    nvs_handle_t nvs_rw;
    if (nvs_open(OTA_BL_NVS_NAMESPACE, NVS_READWRITE, &nvs_rw) == ESP_OK) {
        nvs_erase_key(nvs_rw, OTA_BL_NVS_KEY_STATE);
        nvs_commit(nvs_rw);
        nvs_close(nvs_rw);
    }

    /* EC200 MQTT result-publish removed (modem purged) — recorded in the SD event log
       above. A WiFi-MQTT status publish could be added here via wifi_mqtt if needed. */
    Serial.println("[ESP32_OTA] Success recorded (event log)");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Called from app0 init_task after MQTT is up.
//  Checks NVS for a CAN OTA result written by app1 before the restart.
// ─────────────────────────────────────────────────────────────────────────────
void ota_bootloader_check_can_ota_result(void)
{
    nvs_handle_t nvs;
    char nvsVal[32] = {0};
    size_t nvsLen   = sizeof(nvsVal);

    if (nvs_open(OTA_BL_NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) return;
    esp_err_t err = nvs_get_str(nvs, "can_ota_result", nvsVal, &nvsLen);
    if (err == ESP_OK) {
        nvs_erase_key(nvs, "can_ota_result");
        nvs_commit(nvs);
    }
    nvs_close(nvs);

    if (err != ESP_OK) return;   /* no pending CAN OTA result */

    bool success = (strcmp(nvsVal, "can_success") == 0);
    Serial.printf("[CAN_OTA] Returned to app0 — CAN OTA result: %s\n", nvsVal);
    if (success) EVLOG_OK("OTA",   "CAN (master) flash completed successfully");
    else         EVLOG_FAIL("OTA", "CAN (master) flash ended with errors/cancelled");

    /* EC200 MQTT result-publish removed (modem purged) — recorded in the SD event log above. */
    Serial.println("[CAN_OTA] Boot-status recorded (event log)");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Partition check — DEFINED AFTER ota_bootloader_run (no forward-decl issue)
// ─────────────────────────────────────────────────────────────────────────────
bool ota_bootloader_check_and_run(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (!running) {
        Serial.println("[OTA_BL] WARNING: could not determine running partition — assuming app0");
        return false;
    }

    Serial.printf("[OTA_BL] Running partition: label='%s'  subtype=0x%02X  offset=0x%08X  size=%u\n",
                  running->label, running->subtype, running->address, running->size);

    /* factory subtype = 0x00 = ESP_PARTITION_SUBTYPE_APP_FACTORY */
    if (running->subtype != ESP_PARTITION_SUBTYPE_APP_FACTORY) {
        Serial.printf("[OTA_BL] subtype 0x%02X != factory (0x%02X) — normal app mode (app0 or app1)\n",
                      running->subtype, (uint8_t)ESP_PARTITION_SUBTYPE_APP_FACTORY);
        return false;
    }

    Serial.println("[OTA_BL] ╔══════════════════════════════════════════════╗");
    Serial.println("[OTA_BL] ║  OTA BOOTLOADER MODE  (factory partition)    ║");
    Serial.println("[OTA_BL] ║  CAN Flasher + EC200U Downloader             ║");
    Serial.println("[OTA_BL] ╚══════════════════════════════════════════════╝");
    Serial.println("[OTA_BL] Flash OK  → restarts into app0 (new firmware)");
    Serial.println("[OTA_BL] Flash FAIL→ restarts into app1 (last-known-good backup)");
    Serial.println("[OTA_BL] DL cancel → restarts into app0 (unchanged)");

    ota_bootloader_run();   /* never returns */
    return true;
}