/**
 * can_bootloader.cpp  –  ABMS-compatible CAN OTA bootloader for ESP32-S3
 *
 * See can_bootloader.h for configuration (BTL_DEVICE_TARGETING, BTL_MY_DEVICE_ID).
 *
 * ── App-mode vs bootloader-mode CAN ──────────────────────────────────────
 *
 *   App mode   : TWAI runs NORMAL mode throughout. The app is the charger
 *                controller and DOES transmit (setpoints, status polls,
 *                ChargerToBMS, current-share, config ACKs). NORMAL is also
 *                required so the ESP32 ACKs received frames; LISTEN_ONLY would
 *                block all TX including ACKs. (LISTEN_ONLY is never used.)
 *
 *   BTL active : No driver switch needed. ba100_can_poll() is silenced
 *                and the bootloader task calls twai_transmit() directly.
 *
 * ── Device targeting ─────────────────────────────────────────────────────
 *
 *   BTL_DEVICE_TARGETING 1  (enabled):
 *     Trigger byte 7 must match BTL_MY_DEVICE_ID to arm.
 *     A non-matching trigger DISARMS — 0xB1 is ignored until a matching
 *     trigger arrives.  Safe on a shared bus with multiple devices.
 *
 *   BTL_DEVICE_TARGETING 0  (disabled):
 *     Any trigger arms the device.  Old single-device behaviour.
 *
 * ── Screen debug log ─────────────────────────────────────────────────────
 *
 *   Once the LVGL screen is initialised, bootloader status messages are
 *   printed both to Serial and to a full-screen LVGL label that replaces
 *   the normal UI.  Call can_bootloader_set_screen(lv_obj_t* screen) from
 *   main after ui_init() to enable screen logging.
 */

/* ⚠ IOT-PARITY NOTICE ──────────────────────────────────────────────────────
 *  This CAN-flash bootloader + 3-slot (factory + ota_0 + ota_1) scheme is kept
 *  INTENTIONALLY IDENTICAL to the IOT code:
 *    • the running app lives in app0 (ota_0);
 *    • the CAN-flash bootloader runs from the factory partition;
 *    • the enter-bootloader trigger reboots into factory; factory backs app0
 *      up to app1 (before flashing, ONLY when entry was from app0), then
 *      flashes the new image into app0.
 *  If you change ANYTHING about this flow (partitions, backup, enter trigger,
 *  the flash protocol, or the factory/app split), the assistant/AI editing
 *  this MUST remind the user to make the same change in the IOT code.
 * ───────────────────────────────────────────────────────────────────────── */

#include "can_bootloader.h"
#include "ba100_can.h"   /* charger FSM state — gate BTL entry to safe modes */
#include "app_config.h"  /* CONFIG_CAN_TX_PIN / CONFIG_CAN_RX_PIN */

#include <Arduino.h>
#include <Preferences.h>   /* read the app's NVS-persisted CAN baud for the flash session */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include "driver/twai.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_attr.h"       /* RTC_NOINIT_ATTR — app0-only backup / enter-request guards */
#include "lvgl.h"
#include "lvgl_v8_port.h"   /* lvgl_port_lock / lvgl_port_unlock */
#if CONFIG_FACTORY_WIFI
#include "wifi_service.h"   /* factory-mode WiFi service UI + updater */
#endif

/* ── Serial + screen logging ─────────────────────────────────────────── */
static lv_obj_t *s_btl_screen = nullptr;   /* set by can_bootloader_set_screen() */
static lv_obj_t *s_log_label  = nullptr;   /* multiline label on btl screen      */
static char      s_log_buf[2048] = {};     /* rolling text buffer                */
static lv_obj_t *s_prog_bar   = nullptr;   /* app0->app1 backup progress bar     */
static lv_obj_t *s_prog_lbl   = nullptr;   /* progress caption                   */

/* Append a line to both Serial and the on-screen log */
static void btl_log(const char *fmt, ...)
{
    char line[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);

    Serial.println(line);

    if (!s_log_label) return;

    /* Append to rolling buffer — drop oldest lines when full */
    size_t cur = strlen(s_log_buf);
    size_t add = strlen(line) + 1;   /* +1 for '\n' */
    if (cur + add + 1 >= sizeof(s_log_buf)) {
        /* Drop first line */
        char *nl = strchr(s_log_buf, '\n');
        if (nl) memmove(s_log_buf, nl + 1, strlen(nl));
        else    s_log_buf[0] = '\0';
    }
    strncat(s_log_buf, line,    sizeof(s_log_buf) - strlen(s_log_buf) - 2);
    strncat(s_log_buf, "\n",    sizeof(s_log_buf) - strlen(s_log_buf) - 1);

    lvgl_port_lock(-1);
    lv_label_set_text(s_log_label, s_log_buf);
    /* Scroll to bottom */
    lv_obj_scroll_to_y(lv_obj_get_parent(s_log_label),
                       LV_COORD_MAX, LV_ANIM_OFF);
    lvgl_port_unlock();
}

#define BTL_LOGI(fmt, ...) btl_log("[BTL] "     fmt, ##__VA_ARGS__)
#define BTL_LOGE(fmt, ...) btl_log("[BTL] ERR " fmt, ##__VA_ARGS__)

/* ── RX queue (pre-activation, fed by ba100_can_poll) ───────────────── */
typedef struct { uint8_t data[8]; uint8_t dlc; } btl_frame_t;
#define BTL_QUEUE_DEPTH 32
static QueueHandle_t s_rx_queue = nullptr;

/* ── Armed gate ──────────────────────────────────────────────────────── */
/* s_armed is set by a matching trigger frame and cleared by a
 * non-matching one (when targeting is enabled).                         */
static volatile bool s_armed  = false;
static volatile bool s_active = false;   /* true after 0xB1 echo sent   */

/* Factory mode: true when this firmware is running from the FACTORY partition,
 * i.e. it IS the CAN-flash bootloader (entered by reboot after the 0x0D enter
 * trigger). In factory mode btl_task pulls frames straight off TWAI (there is
 * no app / ba100_can_poll feeding the queue) and the charger-safety gate is
 * bypassed (no charger control is running here). IOT parity — see top notice. */
static volatile bool s_factory_mode = false;

/* app0-only backup guard (IOT parity) ──────────────────────────────────────
 * enter_factory_bootloader() records here whether we are rebooting into factory
 * FROM app0. can_bootloader_run_if_factory() then copies app0->app1 ONLY when
 * this matches BTL_BACKUP_MAGIC_APP0, and consumes it. Entering from app1 (app0
 * was invalid → recovery booted app1) must NOT overwrite the good app1 backup
 * with a stale/bad app0.
 *
 * MUST be RTC_NOINIT_ATTR, NOT RTC_DATA_ATTR: our enter path is esp_restart()
 * (a SW reset, not a deep-sleep wake), and on every non-deep-sleep reset the
 * startup code ZEROES .rtc.bss — where an uninitialized RTC_DATA_ATTR variable
 * lives — so the flag would always read 0 in factory ("not from app0", backup
 * skipped). RTC_NOINIT_ATTR lives in .rtc_noinit, which startup never clears, so
 * it survives esp_restart(); only a true power-on / brownout leaves garbage
 * (won't match the magic → safely skipped, never clobbers app1 when the source
 * is unknown). */
#define BTL_BACKUP_MAGIC_APP0   0xB0A00A0BUL
static RTC_NOINIT_ATTR uint32_t s_backup_from_app0;

/* Deliberate-enter flag (IOT parity). enter_factory_bootloader() sets this to
 * BTL_ENTER_MAGIC just before rebooting into factory. can_bootloader_run_if_factory()
 * checks it on boot: if NOT set, we reached factory without an enter request — a
 * power cycle, or a spontaneous factory boot after a flash that never committed
 * boot=app0 — so recover straight to a valid app instead of sitting here waiting
 * for a flasher (which the 15 s idle timer alone can't guarantee, since any 0x0B
 * traffic keeps resetting it). If set, consume it and run the normal flash-wait.
 * MUST be RTC_NOINIT_ATTR (same reason as s_backup_from_app0) so it survives
 * esp_restart(); a true power-on leaves garbage that won't match → recover. */
#define BTL_ENTER_MAGIC   0xE47E4001UL
static RTC_NOINIT_ATTR uint32_t s_flash_entry_requested;

/* ── Session state ───────────────────────────────────────────────────── */
static uint8_t  s_expected_seq = 0;
static uint32_t s_fw_size      = 0;
static uint32_t s_crc_accum    = 0xFFFFFFFFUL;

static esp_ota_handle_t       s_ota_handle    = 0;
static const esp_partition_t *s_ota_partition = nullptr;
static bool                   s_ota_begun     = false;

/* ── Factory service-UI state (WiFi web updater) ─────────────────────────── */
static btl_backup_status_t    s_backup_status = BTL_BACKUP_PENDING;
static esp_ota_handle_t        s_web_ota      = 0;
static const esp_partition_t  *s_web_ota_part = nullptr;
static bool                    s_web_ota_begun = false;
static volatile unsigned       s_web_ota_bytes = 0;

/* ── Async flash writer ──────────────────────────────────────────────────
 *
 * btl_task   (core 1, pri 3) — receives CAN frames, ACKs immediately,
 *            pushes raw 4-byte chunks onto s_flash_queue.
 *
 * flash_task (core 0, pri 1) — drains s_flash_queue, accumulates into a
 *            4 KB write buffer, calls esp_ota_write() in large blocks.
 *
 * Decoupling ACK from flash write hides the ~2-5 ms flash latency from
 * the host tool's stop-and-wait loop, effectively pipelining for free.
 *
 * VERIFY waits on s_flash_done_sem which flash_task gives after draining
 * the queue completely.
 */

/* Each queue slot = one DATA payload: up to 4 bytes + length */
typedef struct { uint8_t data[4]; uint8_t len; } flash_chunk_t;

#define FLASH_QUEUE_DEPTH   4096          /* ~16 KB of look-ahead        */
#define FLASH_WRITE_BUF_SZ  4096          /* write to flash in 4 KB blocks */

static QueueHandle_t     s_flash_queue    = nullptr;
static SemaphoreHandle_t s_flash_done_sem = nullptr;
static SemaphoreHandle_t s_flash_run_sem  = nullptr;  /* given by UNLOCK, taken by flash_task to start */
static volatile bool     s_flash_error    = false;
static volatile bool     s_flash_draining = false;

/* flash_task is created ONCE in can_bootloader_start() and loops forever.
 * Each session: UNLOCK gives s_flash_run_sem to start it,
 *               VERIFY sets s_flash_draining and waits for s_flash_done_sem. */
static void flash_task(void *arg)
{
    static uint8_t  wbuf[FLASH_WRITE_BUF_SZ];   /* static: heap not stack */

    for (;;) {
        /* Wait for UNLOCK to start a new session */
        xSemaphoreTake(s_flash_run_sem, portMAX_DELAY);
        uint32_t wlen = 0;
        BTL_LOGI("Flash writer active");

        while (true) {
            flash_chunk_t chunk;

            if (xQueueReceive(s_flash_queue, &chunk, pdMS_TO_TICKS(200)) == pdTRUE) {
                /* Skip the dummy wake-up chunk (len==0 sent by wait_flash_done) */
                if (chunk.len == 0) continue;

                memcpy(wbuf + wlen, chunk.data, chunk.len);
                wlen += chunk.len;

                bool queue_empty = (uxQueueMessagesWaiting(s_flash_queue) == 0);
                bool should_flush = (wlen >= FLASH_WRITE_BUF_SZ) ||
                                    (s_flash_draining && queue_empty);

                if (should_flush && wlen > 0) {
                    if (!s_flash_error) {
                        esp_err_t err = esp_ota_write(s_ota_handle, wbuf, wlen);
                        if (err != ESP_OK) {
                            BTL_LOGE("esp_ota_write failed: %d", err);
                            s_flash_error = true;
                        }
                    }
                    wlen = 0;
                }

                if (s_flash_draining && queue_empty) {
                    /* Session complete — signal VERIFY and go back to waiting */
                    s_flash_draining = false;
                    BTL_LOGI("Flash writer done");
                    xSemaphoreGive(s_flash_done_sem);
                    break;
                }

            } else {
                /* 200 ms timeout with no data */
                if (s_flash_draining) {
                    /* Flush remainder */
                    if (wlen > 0 && !s_flash_error) {
                        esp_err_t err = esp_ota_write(s_ota_handle, wbuf, wlen);
                        if (err != ESP_OK) {
                            BTL_LOGE("esp_ota_write timeout-flush failed: %d", err);
                            s_flash_error = true;
                        }
                        wlen = 0;
                    }
                    s_flash_draining = false;
                    BTL_LOGI("Flash writer done (timeout flush)");
                    xSemaphoreGive(s_flash_done_sem);
                    break;
                }
                /* Still receiving data — keep waiting */
            }
        }
    }
}

/* Called once at startup from can_bootloader_start() */
static void create_flash_task(void)
{
    s_flash_queue    = xQueueCreate(FLASH_QUEUE_DEPTH, sizeof(flash_chunk_t));
    s_flash_done_sem = xSemaphoreCreateBinary();
    s_flash_run_sem  = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(flash_task, "btl_flash", 4096, nullptr, 1, nullptr, 0);
}

/* Called by cmd_unlock to start the flash_task for this session */
static void start_flash_task(void)
{
    xQueueReset(s_flash_queue);
    xSemaphoreTake(s_flash_done_sem, 0);   /* clear any leftover from previous session */
    s_flash_error    = false;
    s_flash_draining = false;
    xSemaphoreGive(s_flash_run_sem);       /* release flash_task to run */
}

/* Called by cmd_verify — waits for flash_task to finish all pending writes */
static bool wait_flash_done(void)
{
    s_flash_draining = true;
    /* Send a zero-length chunk to wake flash_task if blocked on empty queue */
    flash_chunk_t wake = {};
    xQueueSend(s_flash_queue, &wake, pdMS_TO_TICKS(100));

    if (xSemaphoreTake(s_flash_done_sem, pdMS_TO_TICKS(30000)) != pdTRUE) {
        BTL_LOGE("Flash task drain timeout");
        return false;
    }
    return !s_flash_error;
}

/* ── Forward declarations ────────────────────────────────────────────── */
static void     btl_task(void *arg);
static bool     recover_boot_to_app(const char *reason);
static void     process_frame(const btl_frame_t &f);
static void     cmd_unlock(const uint8_t *payload);
static void     cmd_data(uint8_t seq, uint8_t len, const uint8_t *data);
static void     cmd_verify(const uint8_t *payload);
static void     cmd_reset(void);
static void     cmd_read_version(void);
static void     send_resp(uint8_t r, uint8_t b1 = 0, uint8_t b2 = 0);
static void     send_heartbeat(void);
static void     send_beacon(void);
static bool     twai_tx(uint32_t id, const uint8_t *data, uint8_t dlc);
static bool     copy_app0_to_app1(void);
static uint32_t crc32_update(uint32_t crc, const uint8_t *buf, size_t len);
static void     reset_session(void);
static bool     check_trigger(const btl_frame_t &f);
static void     flash_task(void *arg);
static void     create_flash_task(void);

/* ── Declared in ba100_can.cpp ───────────────────────────────────────── */
extern "C" void ba100_can_set_btl_active(void);

/* ── Bootloader entry safety gate ─────────────────────────────────────────
 *   Entering the bootloader hands the CAN bus to the OTA task and stops the
 *   charger control loop, so it must NEVER happen while the charger is active.
 *   Allowed ONLY when the charger is IDLE or the battery is BALANCING — i.e.
 *   no current is being delivered. Blocked in every other mode (Charging,
 *   Discharging, Connecting/Pre-Check, Stopping, etc.).                       */
static bool charger_safe_for_btl(void)
{
    /* Harness tester has no charger, so entering the bootloader is always safe.
     * (The lean CAN layer keeps chg_state at CHG_IDLE with no current.) The
     * guard below is retained so the structure matches the IOT code path. */
    BA100_State_t st = ba100_get_state();

    bool charging    = (st.chg_state == CHG_CHARGING) ||
                       (st.chg_state == CHG_STABILIZE) ||
                       (st.sc_relay && st.current_A >= BA100_CHARGING_CURRENT_THRESHOLD_A);

    bool fsm_idle = (st.chg_state == CHG_IDLE);

    return fsm_idle && !charging;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Public API
 * ════════════════════════════════════════════════════════════════════════ */

/* APP mode. IOT parity: the running app (app0) NEVER flashes in place. Real
 * flashing only happens after the 0x0D enter trigger reboots us into the
 * factory partition (see can_bootloader_run_if_factory). So there is no flash
 * task in app mode — nothing to start here. */
void can_bootloader_start(void)
{
    BTL_LOGI("App mode — CAN flashing runs from factory partition (0x0D to enter)");
}

/* Legacy: no longer used in the IOT-parity model (app mode does not flash and
 * factory mode reads TWAI directly). Kept as a safe no-op for API stability. */
void can_bootloader_rx_notify(const uint8_t *data, uint8_t dlc)
{
    if (!s_rx_queue) return;
    btl_frame_t f = {};
    uint8_t n = (dlc > 8) ? 8 : dlc;
    memcpy(f.data, data, n);
    f.dlc = n;
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(s_rx_queue, &f, &woken);
    portYIELD_FROM_ISR(woken);
}

/* Back up app0 (ota_0) -> app1 (ota_1): full partition, 4 KB blocks, via the
 * OTA API so app1 ends up a VALID bootable image (esp_ota_end validates it).
 *
 * ⚠ Runs in the FACTORY bootloader right after boot, BEFORE any flashing erases
 *   app0 (V0.2 fix) — and ONLY when factory was entered from app0 (the caller
 *   gates this on s_backup_from_app0). Reads the app0 PARTITION explicitly (the
 *   running partition here is `factory`, not app0), so the backup always
 *   captures the firmware that was running when the operator entered the
 *   bootloader — it can never leave a stale app1. Must complete before
 *   cmd_unlock() erases app0.
 *
 * Drives the on-screen progress bar. Yields periodically (feeds the task WDT and
 * lets LVGL render). Returns true on success. IOT parity — see top notice.     */
static bool copy_app0_to_app1(void)
{
    const esp_partition_t *src = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);   /* app0 */
    const esp_partition_t *dst = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);   /* app1 */
    if (!src || !dst) { BTL_LOGE("backup: app0/app1 partition not found"); return false; }

    if (s_prog_bar) {
        lvgl_port_lock(-1);
        lv_obj_clear_flag(s_prog_bar, LV_OBJ_FLAG_HIDDEN);
        lv_bar_set_value(s_prog_bar, 0, LV_ANIM_OFF);
        if (s_prog_lbl) lv_label_set_text(s_prog_lbl, "Backing up firmware (app0 -> app1)...");
        lvgl_port_unlock();
    }

    esp_ota_handle_t h;
    if (esp_ota_begin(dst, OTA_SIZE_UNKNOWN, &h) != ESP_OK) {   /* erases app1 */
        BTL_LOGE("esp_ota_begin(app1) failed"); return false;
    }
    static uint8_t buf[4096];
    size_t off = 0; bool ok = true; int last_pct = -1, blk = 0;
    while (off < src->size) {
        size_t n = (src->size - off > sizeof(buf)) ? sizeof(buf) : (src->size - off);
        if (esp_partition_read(src, off, buf, n) != ESP_OK) { ok = false; break; }
        if (esp_ota_write(h, buf, n)            != ESP_OK) { ok = false; break; }
        off += n;
        int pct = (int)((uint64_t)off * 100 / src->size);
        if (pct != last_pct && s_prog_bar) {
            last_pct = pct;
            lvgl_port_lock(-1);
            lv_bar_set_value(s_prog_bar, pct, LV_ANIM_OFF);
            lvgl_port_unlock();
        }
        if ((++blk & 0x0F) == 0) vTaskDelay(pdMS_TO_TICKS(1));   /* yield WDT / LVGL */
    }
    if (esp_ota_end(h) != ESP_OK) ok = false;   /* validates + marks app1 bootable */

    if (s_prog_bar) {
        lvgl_port_lock(-1);
        if (ok) lv_bar_set_value(s_prog_bar, 100, LV_ANIM_OFF);
        if (s_prog_lbl)
            lv_label_set_text(s_prog_lbl, ok ? "Backup complete - ready to flash."
                                             : "Backup FAILED (app0 intact - safe to retry).");
        lvgl_port_unlock();
    }
    BTL_LOGI("app0->app1 backup %s (%u bytes)", ok ? "OK" : "FAILED", (unsigned)off);
    return ok;
}

/* Enter the bootloader: set boot=factory, reboot. The factory partition then
 * (V0.2 fix) backs up app0->app1 BEFORE flashing and runs the CAN-flash slave
 * (see can_bootloader_run_if_factory). The backup is deliberately NOT done here
 * — doing it in factory guarantees it captures the current app0 and shows a
 * progress bar, instead of blocking the running app with no feedback. */
static void enter_factory_bootloader(void)
{
    /* Stop the charger BEFORE handing the bus to the bootloader — so entry is
     * safe even mid-charge (no idle gate needed; V0.2 auto-charges, which used
     * to block entry). Leaves the modules off, not holding a stale setpoint. */
    ba100_charger_off();

    /* Record whether we are entering factory FROM app0. Only a backup taken
     * from app0 is valid; entering from app1 (app0 invalid → recovery booted
     * app1) must not clobber the good app1 backup. Consumed after the reboot by
     * can_bootloader_run_if_factory(). IOT parity — see top notice. */
    const esp_partition_t *running = esp_ota_get_running_partition();
    s_backup_from_app0 = (running &&
        running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0)
        ? BTL_BACKUP_MAGIC_APP0 : 0;

    /* Record that this factory entry is DELIBERATE (flasher's 0x0D). factory
     * checks this on boot: if absent (power cycle / post-flash spontaneous boot)
     * it recovers to an app instead of waiting for a flasher. Consumed once. */
    s_flash_entry_requested = BTL_ENTER_MAGIC;

    const esp_partition_t *factory = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
    if (!factory) {
        BTL_LOGE("FATAL: factory partition not found — cannot enter bootloader");
        return;
    }
    if (esp_ota_set_boot_partition(factory) != ESP_OK) {
        BTL_LOGE("esp_ota_set_boot_partition(factory) FAILED");
        return;
    }
    BTL_LOGI("Boot set to factory — rebooting into CAN-flash bootloader");
    delay(500);
    esp_restart();   /* does not return */
}

/* Desktop-flasher "Enter bootloader" trigger on BTL_ENTER_BL_ID (0x0D).
 * Only the exact 8-byte alternating 0x69/0x96 pattern acts, so ordinary bus
 * traffic on 0x0D can never enter the bootloader. REFUSED while a battery/BMS is
 * connected (batt_data_valid) — entering hands the CAN bus to the flasher and
 * drops charger control, so it must not happen with a pack attached; disconnect
 * the battery to flash. On a match with no battery: set boot=factory and reboot
 * (IOT parity); the factory bootloader then backs up app0->app1 before flashing.
 * Idempotent — the flasher sends the trigger 3×. */
void can_bootloader_enter_notify(const uint8_t *data, uint8_t dlc)
{
    if (dlc < 8) return;
    for (int i = 0; i < 8; i++) {
        uint8_t expected = (i % 2 == 0) ? BTL_CMD_TRIGGER : BTL_CMD_TRIGGER_INVERTED;
        if (data[i] != expected) return;   /* not the enter pattern — ignore */
    }

    /* Refuse to enter the bootloader while a battery/BMS is connected. Handing
     * the bus to the flasher drops charger control, so it must not happen with a
     * pack attached. batt_data_valid is true only while a BMS is actively
     * communicating and self-clears once the battery is disconnected (telemetry
     * goes stale), so the flasher works again as soon as the pack is removed. */
    if (ba100_get_state().batt_data_valid) {
        BTL_LOGI("Enter bootloader (0x0D) IGNORED — battery/BMS connected (disconnect to flash)");
        return;
    }

    static volatile bool s_entering = false;
    if (s_entering) return;                /* already rebooting into factory  */
    s_entering = true;
    /* No battery attached (gated above). enter_factory_bootloader() still calls
     * ba100_charger_off() as a belt-and-braces stop, then reboots into factory. */
    BTL_LOGI("=== Enter bootloader (0x0D) — no battery, rebooting into factory ===");
    enter_factory_bootloader();            /* stop charger + set boot + reboot */
    s_entering = false;                    /* reached only if entry FAILED — allow retry */
}

/* Resolve the CAN baud for the factory flash session, mirroring the app: prefer
 * the NVS-persisted rate a runtime 0x2CC change wrote (Preferences namespace
 * "ba100", key "baud" — the exact store ba100_can.cpp uses), and fall back to
 * CONFIG_CAN_BAUD_KBPS when it is unset (fresh device / namespace not created
 * yet) or outside {125,250,500}. This keeps the flasher on whatever baud the bus
 * is actually running, even after the app was retuned away from the compile-time
 * default. NVS is already initialised by the Arduino core (initArduino) before
 * setup() calls this, and the app is not running in factory mode, so the
 * read-only open is safe and uncontended. */
static twai_timing_config_t btl_resolve_can_timing(uint32_t *kbps_out)
{
    uint32_t kbps = CONFIG_CAN_BAUD_KBPS;
    Preferences p;
    if (p.begin("ba100", true)) {          /* read-only; false if never created */
        kbps = p.getUInt("baud", CONFIG_CAN_BAUD_KBPS);
        p.end();
    }
    if (kbps != 125 && kbps != 250 && kbps != 500) kbps = CONFIG_CAN_BAUD_KBPS;
    if (kbps_out) *kbps_out = kbps;
    switch (kbps) {
        case 125: { twai_timing_config_t t = TWAI_TIMING_CONFIG_125KBITS(); return t; }
        case 500: { twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS(); return t; }
        default:  { twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS(); return t; }
    }
}

/* Called FIRST from main setup(): if this firmware is running from the factory
 * partition it IS the CAN-flash bootloader — install TWAI, show the updating
 * screen and run the slave loop (never returns). Returns false when running
 * from app0/app1 so main continues into the normal application. IOT parity. */
bool can_bootloader_run_if_factory(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (!running || running->subtype != ESP_PARTITION_SUBTYPE_APP_FACTORY)
        return false;   /* normal app (app0/app1) */

    s_factory_mode = true;
    BTL_LOGI("=== FACTORY partition — CAN-flash bootloader mode ===");

    /* If we reached factory WITHOUT a deliberate enter request, don't sit here
     * waiting for a flasher (power cycle, or a spontaneous factory boot after a
     * flash that never committed boot=app0). Recover straight to a valid app.
     * enter_factory_bootloader() sets s_flash_entry_requested; consume it and
     * flash normally. If no valid app exists (blank device) recover_boot_to_app()
     * returns without rebooting and we fall through to flash mode. IOT parity. */
    if (s_flash_entry_requested == BTL_ENTER_MAGIC) {
        s_flash_entry_requested = 0;   /* consume: a later spontaneous factory boot recovers */
        BTL_LOGI("Enter request present — staying in factory to flash");
    } else {
        BTL_LOGI("Factory boot with no enter request — recovering to a valid app");
        recover_boot_to_app("no enter request");   /* reboots if a valid app exists */
        BTL_LOGI("No bootable app — staying in factory for flashing");
    }

    /* TWAI NORMAL on the configured CAN pins (7" board: GPIO20/19).
     * Baud follows the app: the NVS rate a runtime 0x2CC change wrote,
     * else the CONFIG_CAN_BAUD_KBPS default (see btl_resolve_can_timing). So a
     * device retuned at runtime is flashed at its actual bus baud — no need to
     * keep the flash host on the compile-time default. The flasher's enter
     * (0x0D) and the whole flash session run at this baud. */
    uint32_t btl_kbps = 0;
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CONFIG_CAN_TX_PIN, (gpio_num_t)CONFIG_CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t = btl_resolve_can_timing(&btl_kbps);
    twai_filter_config_t  fl = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g, &t, &fl) != ESP_OK || twai_start() != ESP_OK)
        BTL_LOGE("TWAI install/start FAILED in factory mode");
    else
        BTL_LOGI("TWAI @ %lu kbps NORMAL (factory flash session)", (unsigned long)btl_kbps);

    /* Updating overlay (LVGL is already running — main inits the board first). */
    can_bootloader_set_screen();
    if (s_btl_screen) { lvgl_port_lock(-1); lv_scr_load(s_btl_screen); lvgl_port_unlock(); }
    BTL_LOGI("FIRMWARE UPDATE mode (do not power off)");

    /* Back up app0 -> app1 NOW, before any flashing erases app0 (V0.2 fix, with
     * progress bar) — but ONLY when we entered factory FROM app0 (flag set by
     * enter_factory_bootloader before the reboot). If we came from app1 (app0
     * was invalid → recovery booted app1), app0 is stale/bad and copying it
     * would destroy the good app1 backup. A power cycle clears the RTC flag
     * (garbage won't match the magic) → also skipped, the safe direction. When
     * it does run, app0 is still intact here so a failed backup is safe
     * (recovery would just boot app0). IOT parity — see top notice. */
    if (s_backup_from_app0 == BTL_BACKUP_MAGIC_APP0) {
        s_backup_from_app0 = 0;   /* consume — capture app0 once, at clean entry */
        s_backup_status = copy_app0_to_app1() ? BTL_BACKUP_PASS : BTL_BACKUP_FAIL;
    } else {
        BTL_LOGI("Skip app0->app1 backup (not entered from app0) — app1 preserved");
        s_backup_status = BTL_BACKUP_SKIPPED;
    }

    /* Flash slave: btl_task (factory pre-activation reads TWAI, emits
     * 0x456/0x702; flasher's 0x0B trigger+0xB1 activates, then UNLOCK/DATA/
     * VERIFY/RESET flash into app0). Beacons start only AFTER the backup, so
     * the flasher's "Ready to flash" appears once we can actually flash. */
    create_flash_task();
    xTaskCreatePinnedToCore(btl_task, "can_btl", 8192, nullptr, 3, nullptr, 1);

#if CONFIG_FACTORY_WIFI
    /* Raise the hidden service AP + web UI (updater/config/reboot). It runs in
     * its own task and is torn down on the reboot that exits factory mode. */
    wifi_service_start();
#endif

    return true;   /* main halts; btl_task + wifi service run forever */
}

bool can_bootloader_wants_takeover(void)  { return false; }
void can_bootloader_signal_released(void) { }

/* ════════════════════════════════════════════════════════════════════════
 *  Factory service-UI hooks (WiFi web updater / config / reboot)
 *
 *  The web flash reuses the SAME OTA primitives as the CAN flasher: esp_ota_*
 *  on the app0 partition (esp_ota_get_next_update_partition, exactly like
 *  cmd_unlock/cmd_verify), then esp_ota_set_boot_partition(app0). Only one flash
 *  path (CAN or web) should be used at a time.
 * ════════════════════════════════════════════════════════════════════════ */
btl_backup_status_t can_bootloader_backup_status(void) { return s_backup_status; }

bool can_bootloader_web_flash_begin(void)
{
    if (s_web_ota_begun) { esp_ota_abort(s_web_ota); s_web_ota_begun = false; }
    s_web_ota_bytes = 0;
    s_web_ota_part  = esp_ota_get_next_update_partition(nullptr);   /* → app0 */
    if (!s_web_ota_part) { BTL_LOGE("web flash: no OTA partition"); return false; }
    if (esp_ota_begin(s_web_ota_part, OTA_SIZE_UNKNOWN, &s_web_ota) != ESP_OK) {
        BTL_LOGE("web flash: esp_ota_begin failed"); return false;
    }
    s_web_ota_begun = true;
    BTL_LOGI("web flash: begin -> %s @ 0x%08lX",
             s_web_ota_part->label, (unsigned long)s_web_ota_part->address);
    return true;
}

bool can_bootloader_web_flash_write(const uint8_t *data, unsigned len)
{
    if (!s_web_ota_begun) return false;
    if (esp_ota_write(s_web_ota, data, len) != ESP_OK) {
        BTL_LOGE("web flash: esp_ota_write failed at %u B", s_web_ota_bytes);
        esp_ota_abort(s_web_ota); s_web_ota_begun = false; return false;
    }
    s_web_ota_bytes += len;
    return true;
}

bool can_bootloader_web_flash_end(void)
{
    if (!s_web_ota_begun) return false;
    s_web_ota_begun = false;
    if (esp_ota_end(s_web_ota) != ESP_OK) {              /* validates the image */
        BTL_LOGE("web flash: esp_ota_end/verify failed"); return false;
    }
    if (esp_ota_set_boot_partition(s_web_ota_part) != ESP_OK) {
        BTL_LOGE("web flash: set_boot_partition failed"); return false;
    }
    BTL_LOGI("web flash: OK, %u B written to %s; boot set",
             s_web_ota_bytes, s_web_ota_part->label);
    return true;
}

void can_bootloader_web_flash_abort(void)
{
    if (s_web_ota_begun) { esp_ota_abort(s_web_ota); s_web_ota_begun = false; }
}

unsigned can_bootloader_web_flash_bytes(void) { return s_web_ota_bytes; }

void can_bootloader_reboot_to_app(void)
{
    BTL_LOGI("Service UI: reboot requested — exiting factory to app");
    recover_boot_to_app("web reboot");   /* sets boot=app0/app1 + reboots if valid */
    esp_restart();                        /* fallback (no valid app) */
}

/**
 * Call this from main.cpp after ui_init() to enable on-screen debug logging.
 * Creates a full-screen black panel with a small white scrolling label.
 * The normal UI is hidden while the bootloader is active.
 */
void can_bootloader_set_screen(void)
{
    lvgl_port_lock(-1);

    s_btl_screen = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(s_btl_screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_btl_screen, LV_OPA_COVER, 0);

    /* Scrollable container */
    lv_obj_t *cont = lv_obj_create(s_btl_screen);
    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(cont, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(cont, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_pad_all(cont, 8, 0);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(cont, LV_DIR_VER);

    /* Backup progress caption + bar (shown while copying app0 -> app1). */
    s_prog_lbl = lv_label_create(cont);
    lv_obj_set_width(s_prog_lbl, LV_PCT(100));
    lv_obj_set_style_text_color(s_prog_lbl, lv_color_white(), 0);
    lv_obj_set_style_text_font(s_prog_lbl, &lv_font_montserrat_16, 0);
    lv_label_set_text(s_prog_lbl, "");

    s_prog_bar = lv_bar_create(cont);
    lv_obj_set_width(s_prog_bar, LV_PCT(90));
    lv_obj_set_height(s_prog_bar, 20);
    lv_bar_set_range(s_prog_bar, 0, 100);
    lv_bar_set_value(s_prog_bar, 0, LV_ANIM_OFF);
    lv_obj_add_flag(s_prog_bar, LV_OBJ_FLAG_HIDDEN);   /* shown during backup */

    s_log_label = lv_label_create(cont);
    lv_obj_set_width(s_log_label, LV_PCT(100));
    lv_label_set_long_mode(s_log_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_color(s_log_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(s_log_label, &lv_font_montserrat_12, 0);
    lv_label_set_text(s_log_label, "[BTL] Screen ready\n");

    lvgl_port_unlock();

    /* Screen is created but NOT loaded — normal app UI stays visible.
     * lv_scr_load() is called later at the moment of 0xB1 activation. */
    BTL_LOGI("BTL screen created (hidden, app UI still active)");
}

/* ════════════════════════════════════════════════════════════════════════
 *  Trigger frame check + armed gate
 *
 *  Trigger frame layout (host → device):
 *    Bytes 0-6 : alternating 0x69 / 0x96
 *    Byte  7   : device selector
 *                  0x96 = broadcast (matches everyone)
 *                  0x01 = BMS
 *                  0x02 = LCD  (BTL_MY_DEVICE_ID)
 *                  0x03 = third ESP32
 *
 *  Returns true if this device should arm (and reply 0x50).
 *  Returns false if targeting is on and byte 7 doesn't match — disarms.
 * ════════════════════════════════════════════════════════════════════════ */
static bool check_trigger(const btl_frame_t &f)
{
    if (f.dlc < 8) return false;

    /* Check alternating pattern in bytes 0-6 */
    for (int i = 0; i < 7; i++) {
        uint8_t expected = (i % 2 == 0) ? BTL_CMD_TRIGGER : BTL_CMD_TRIGGER_INVERTED;
        if (f.data[i] != expected) return false;
    }

    /* byte 7 = device selector */
    uint8_t selector = f.data[7];

#if BTL_DEVICE_TARGETING
    /* Broadcast selector arms everyone */
    if (selector == BTL_CMD_TRIGGER_INVERTED) return true;   /* 0x96 = broadcast */
    /* Our device ID arms us */
    if (selector == BTL_MY_DEVICE_ID)         return true;
    /* Someone else's ID — explicitly disarm */
    return false;
#else
    /* Targeting off — any valid trigger pattern arms us */
    (void)selector;
    return true;
#endif
}

/* restart_twai_normal() removed — the driver is always NORMAL. In app mode the
 * app itself transmits (charger control); in factory mode this bootloader task
 * transmits (replies, heartbeat, beacon). No mode switch on activation. */

/* ════════════════════════════════════════════════════════════════════════
 *  Task — dual mode
 *
 *  Active mode includes a 15-second inactivity watchdog. If no CAN frame
 *  arrives for 15 s during a flash session, btl_recover_to_app() aborts the
 *  OTA and REBOOTS into a valid app (app0, else the app1 backup) — it does not
 *  return to a pre-activation wait state.
 * ════════════════════════════════════════════════════════════════════════ */

/* Factory "no data received" recovery window. If no bootloader frame is seen
 * for this long — whether waiting for a flasher (pre-activation) or mid-flash
 * (host went quiet) — reboot to a valid app. Both use the same 15 s so the rule
 * is uniform: 15 s of silence in factory → recover. */
#define BTL_INACTIVITY_TIMEOUT_MS  15000UL     /* mid-flash: host went quiet    */
#define FACTORY_IDLE_RECOVER_MS    15000UL     /* factory up, no flasher → recover */
#define WEB_SESSION_TIMEOUT_MS     300000UL    /* logged in over WiFi: 5-min inactivity → recover */

static void btl_abort_session(const char *reason);      /* forward decl */
static void btl_recover_to_app(const char *reason);     /* forward decl */

static void btl_task(void *arg)
{
    TickType_t hb_tick       = xTaskGetTickCount();
    TickType_t last_frame_tick = xTaskGetTickCount();

    for (;;) {

        if (s_active) {
            /* ── Active: drain TWAI RX directly (ba100_can_poll silenced) ── */
            twai_message_t msg;
            if (twai_receive(&msg, pdMS_TO_TICKS(500)) == ESP_OK) {
                if (!msg.extd && msg.identifier == BTL_CAN_ID) {
                    last_frame_tick = xTaskGetTickCount();   /* reset watchdog */
                    btl_frame_t f = {};
                    f.dlc = (msg.data_length_code > 8) ? 8 : msg.data_length_code;
                    memcpy(f.data, msg.data, f.dlc);
                    process_frame(f);
                }
                /* Extended BMS/charger frames — reset watchdog too so
                 * background bus traffic doesn't falsely trigger timeout */
                last_frame_tick = xTaskGetTickCount();
            } else {
                /* 500 ms rx timeout — check inactivity watchdog */
                TickType_t now = xTaskGetTickCount();
                uint32_t idle_ms = (uint32_t)((now - last_frame_tick) * portTICK_PERIOD_MS);
                if (idle_ms >= BTL_INACTIVITY_TIMEOUT_MS) {
                    /* Host went quiet mid-flash (failed / cancelled / incomplete):
                     * recover to a working app instead of sitting in factory. */
                    btl_recover_to_app("mid-flash inactivity timeout (15 s)");
                    last_frame_tick = xTaskGetTickCount();   /* only if recover couldn't reboot */
                }
            }

        } else if (s_factory_mode) {
            /* ── Pre-activation, FACTORY mode ────────────────────────────────
             * No app / ba100_can_poll is running, so pull the flasher's
             * trigger (0x69/0x96 on 0x0B) + 0xB1 straight off TWAI. Emit the
             * heartbeat (0x456) and ready beacon (0x702) fast so the desktop
             * flasher's "Ready to flash" indicator lights promptly.          */
            twai_message_t msg;
            if (twai_receive(&msg, pdMS_TO_TICKS(200)) == ESP_OK) {
                if (!msg.extd && msg.identifier == BTL_CAN_ID) {
                    last_frame_tick = xTaskGetTickCount();   /* flasher active → reset idle timer */
                    btl_frame_t f = {};
                    f.dlc = (msg.data_length_code > 8) ? 8 : msg.data_length_code;
                    memcpy(f.data, msg.data, f.dlc);
                    process_frame(f);
                }
            }
            TickType_t now = xTaskGetTickCount();
            if ((now - hb_tick) >= pdMS_TO_TICKS(250)) {
                hb_tick = now;
                send_heartbeat();   /* 0x456 */
                send_beacon();      /* 0x702 "ready to flash" */
            }
            /* Idle recovery: if no flasher connects within the window, boot back
             * into the app instead of sitting in the bootloader forever (e.g.
             * entered by mistake, or power-cycled while boot was still pinned to
             * factory after an earlier failed flash). Recovers to app0, or app1
             * if app0 is invalid. */
            uint32_t can_idle_ms = (uint32_t)((now - last_frame_tick) * portTICK_PERIOD_MS);
#if CONFIG_FACTORY_WIFI
            if (wifi_service_logged_in()) {
                /* A technician is logged in over WiFi (no CAN traffic). Suppress
                 * the 15 s CAN-idle recover and use a 5-min web-inactivity window
                 * instead — each web request resets it (wifi_service). */
                if (wifi_service_ms_since_activity() >= WEB_SESSION_TIMEOUT_MS)
                    btl_recover_to_app("web session idle timeout (5 min)");
            } else {
                /* Not logged in yet: normal 15 s recover, but web activity (e.g.
                 * loading the login page) also holds it off so we don't reboot
                 * out from under someone mid-login. */
                if (can_idle_ms >= FACTORY_IDLE_RECOVER_MS &&
                    wifi_service_ms_since_activity() >= FACTORY_IDLE_RECOVER_MS)
                    btl_recover_to_app("no flasher connected (idle)");
            }
#else
            if (can_idle_ms >= FACTORY_IDLE_RECOVER_MS)
                btl_recover_to_app("no flasher connected (idle)");
#endif

        } else {
            /* ── Pre-activation, APP mode (legacy queue path) ──────────────
             * In the IOT-parity model the app never flashes in place, so this
             * branch is effectively idle; kept for safety.                    */
            btl_frame_t f;
            if (xQueueReceive(s_rx_queue, &f, pdMS_TO_TICKS(500)) == pdTRUE) {
                process_frame(f);
            }
            TickType_t now = xTaskGetTickCount();
            if ((now - hb_tick) >= pdMS_TO_TICKS(2000)) {
                hb_tick = now;
                send_heartbeat();
            }
        }
    }
}

/* ════════════════════════════════════════════════════════════════════════
 *  Session abort — called on inactivity timeout or unrecoverable error.
 *  Aborts OTA, resets all state, re-enables ba100_can_poll, shows main UI.
 * ════════════════════════════════════════════════════════════════════════ */
extern "C" void ba100_can_clear_btl_active(void);

static void btl_abort_session(const char *reason)
{
    BTL_LOGI("Session aborted: %s", reason);

    /* Abort any in-progress OTA write */
    if (s_ota_begun) {
        esp_ota_abort(s_ota_handle);
        s_ota_begun = false;
        BTL_LOGI("OTA aborted");
    }

    /* Stop flash_task from writing further */
    s_flash_error    = true;
    s_flash_draining = false;
    if (s_flash_queue) xQueueReset(s_flash_queue);

    /* Full session reset */
    s_expected_seq = 0;
    s_fw_size      = 0;
    s_crc_accum    = 0xFFFFFFFFUL;
    s_armed        = false;
    s_active       = false;   /* back to pre-activation */

    /* Re-enable ba100_can_poll (harmless in factory mode — poll not running) */
    ba100_can_clear_btl_active();

    /* NOTE: no reboot here. A host RECONNECT re-arms in place (see
     * process_frame); a give-up / failure calls btl_recover_to_app() to boot a
     * valid application. */
}

/* ════════════════════════════════════════════════════════════════════════
 *  Recover out of the factory bootloader to a working application.
 *  Called after a failed / cancelled / incomplete flash (or when no flasher
 *  ever connects). Prefer app0 (the flash target); if app0 is invalid
 *  (half-written or erased-not-written) fall back to app1 (the pre-flash
 *  backup). If neither is bootable, stay in factory so the flash can be
 *  retried.  esp_ota_set_boot_partition() validates the image, so a corrupt
 *  app0 is automatically rejected in favour of app1.
 *  ⚠ IOT parity — keep this recovery behaviour in sync with the IOT code.
 * ════════════════════════════════════════════════════════════════════════ */
/* Set boot to a valid app and reboot: prefer app0 (the flash target); fall back
 * to app1 (the pre-flash backup) if app0 is invalid. esp_ota_set_boot_partition()
 * validates the image, so a corrupt/half-written app0 is rejected in favour of
 * app1. Returns false (WITHOUT rebooting) only if neither is bootable — the
 * caller then stays in factory so a blank/bricked device can still be flashed.
 * Used both by the early "no enter request" recovery in run_if_factory and by
 * btl_recover_to_app() after a failed/idle session. ⚠ IOT parity. */
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
        BTL_LOGE("Recover (%s): neither app0 nor app1 bootable — staying in "
                 "factory to retry", reason);
        return false;
    }
    BTL_LOGI("Recover (%s) -> booting %s", reason, dst);
    vTaskDelay(pdMS_TO_TICKS(300));
    esp_restart();   /* does not return */
    return true;     /* not reached */
}

static void btl_recover_to_app(const char *reason)
{
    btl_abort_session(reason);
    recover_boot_to_app(reason);
    /* If it returned, neither app was bootable — it already logged; we stay in
     * factory to allow a retry. */
}

/* ════════════════════════════════════════════════════════════════════════
 *  Frame dispatcher
 * ════════════════════════════════════════════════════════════════════════ */
static void process_frame(const btl_frame_t &f)
{
    /* ── Pre-activation ────────────────────────────────────────────── */
    if (!s_active) {

        /* Trigger frame — arm/disarm based on device ID */
        if (check_trigger(f)) {
            s_armed = true;
            BTL_LOGI("Trigger matched (byte7=0x%02X) — ARMED, replying 0x50", f.data[7]);
            send_resp(BL_RESP_OK);
            return;
        }

#if BTL_DEVICE_TARGETING
        /* Valid trigger pattern but wrong device ID — explicitly disarm */
        {
            bool looks_like_trigger = (f.dlc >= 7);
            if (looks_like_trigger) {
                bool pattern_ok = true;
                for (int i = 0; i < 7; i++) {
                    uint8_t expected = (i % 2 == 0) ? BTL_CMD_TRIGGER : BTL_CMD_TRIGGER_INVERTED;
                    if (f.data[i] != expected) { pattern_ok = false; break; }
                }
                if (pattern_ok) {
                    s_armed = false;
                    BTL_LOGI("Trigger for other device (byte7=0x%02X) — DISARMED", f.dlc >= 8 ? f.data[7] : 0);
                    return;
                }
            }
        }
#endif

        /* 0xB1 activation */
        if (f.dlc >= 1 && f.data[0] == BTL_ACTIVATE_BYTE) {
#if BTL_DEVICE_TARGETING
            if (!s_armed) {
                BTL_LOGI("0xB1 ignored — not armed (targeting ON)");
                return;
            }
#endif
            /* Safety gate: only enter the bootloader when the charger is idle
             * or balancing — never mid-charge / mid-discharge (handing off the
             * bus would drop charger control). Bypassed in factory mode: no
             * charger loop runs there and we are already committed to flashing. */
            if (!s_factory_mode && !charger_safe_for_btl()) {
                BTL_LOGI("0xB1 ignored — charger not idle/balancing (chg_state busy); BTL entry blocked");
                return;
            }

            BTL_LOGI("0xB1 received — armed=%d, activating", (int)s_armed);

            /* Silence ba100_can_poll — bootloader task takes over RX from here */
            ba100_can_set_btl_active();

            /* Echo 0xB1 */
            uint8_t echo[8] = { BTL_ACTIVATE_BYTE, 0, 0, 0, 0, 0, 0, 0 };
            twai_tx(BTL_CAN_ID, echo, 8);

            s_active = true;
            reset_session();

            /* Switch display to bootloader screen now */
            if (s_btl_screen) {
                lv_scr_load(s_btl_screen);
            } else {
                /* Screen wasn't pre-created — create and load it now */
                can_bootloader_set_screen();
                if (s_btl_screen) lv_scr_load(s_btl_screen);
            }
            BTL_LOGI("=== BOOTLOADER SESSION OPEN ===");
            BTL_LOGI("Device ID: 0x%02X", BTL_MY_DEVICE_ID);
            return;
        }

        /* Wake-up ping ID=0x69 arriving as data — ignore silently */
        return;
    }

    /* ── Active session ────────────────────────────────────────────── */

    /* Trigger frame during active session = host is retrying after disconnect.
     * Abort current session cleanly, arm ourselves, reply 0x50 so host
     * proceeds to send 0xB1 and start a fresh session. */
    if (check_trigger(f)) {
        BTL_LOGI("Trigger received during active session — aborting & re-arming");
        btl_abort_session("host reconnect trigger");
        /* After abort s_active=false, so process as pre-activation trigger */
        s_armed = true;
        BTL_LOGI("Re-ARMED (byte7=0x%02X), replying 0x50", f.data[7]);
        send_resp(BL_RESP_OK);
        return;
    }

    /* 0xB1 during active session = host retry after disconnect (no trigger) */
    if (f.dlc >= 1 && f.data[0] == BTL_ACTIVATE_BYTE) {
        BTL_LOGI("Re-activation during active session — aborting & restarting");
        btl_abort_session("re-activation");
        /* Now process as fresh activation — fall through would need s_active=false
         * which abort already set. Directly handle it here: */
        ba100_can_set_btl_active();   /* silence poll again */
        uint8_t echo[8] = { BTL_ACTIVATE_BYTE, 0, 0, 0, 0, 0, 0, 0 };
        twai_tx(BTL_CAN_ID, echo, 8);
        s_active = true;
        reset_session();
        if (s_btl_screen) lv_scr_load(s_btl_screen);
        BTL_LOGI("=== BOOTLOADER SESSION RESTARTED ===");
        return;
    }

    /* Validate magic */
    if (f.dlc < 4 || f.data[2] != BL_MAGIC) {
        BTL_LOGE("Bad frame dlc=%d magic=0x%02X", f.dlc, f.dlc >= 3 ? f.data[2] : 0);
        send_resp(BL_RESP_INVALID);
        return;
    }

    switch (f.data[0]) {
        case BL_CMD_UNLOCK:       cmd_unlock(&f.data[4]);                      break;
        case BL_CMD_DATA:         cmd_data(f.data[1], f.data[3], &f.data[4]);  break;
        case BL_CMD_VERIFY:       cmd_verify(&f.data[4]);                      break;
        case BL_CMD_RESET:        cmd_reset();                                 break;
        case BL_CMD_READ_VERSION: cmd_read_version();                          break;
        default:
            BTL_LOGE("Unknown cmd 0x%02X", f.data[0]);
            send_resp(BL_RESP_INVALID);
            break;
    }
}

/* ════════════════════════════════════════════════════════════════════════
 *  Commands
 * ════════════════════════════════════════════════════════════════════════ */

static void cmd_unlock(const uint8_t *p)
{
    s_fw_size = (uint32_t)p[0]         | ((uint32_t)p[1] << 8)  |
                ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    BTL_LOGI("UNLOCK size=%lu bytes (0x%lX)",
             (unsigned long)s_fw_size, (unsigned long)s_fw_size);

    if (s_ota_begun) { esp_ota_abort(s_ota_handle); s_ota_begun = false; }

    s_ota_partition = esp_ota_get_next_update_partition(nullptr);
    if (!s_ota_partition) {
        BTL_LOGE("No OTA partition — check partition table");
        send_resp(BL_RESP_ERROR); return;
    }
    BTL_LOGI("OTA partition: '%s' @ 0x%08lX",
             s_ota_partition->label,
             (unsigned long)s_ota_partition->address);

    if (esp_ota_begin(s_ota_partition, OTA_SIZE_UNKNOWN, &s_ota_handle) != ESP_OK) {
        BTL_LOGE("esp_ota_begin failed");
        send_resp(BL_RESP_ERROR); return;
    }
    s_ota_begun    = true;
    s_expected_seq = 0;
    s_crc_accum    = 0xFFFFFFFFUL;
    start_flash_task();   /* start async writer on core 0 */
    BTL_LOGI("Flash erased — ready for DATA (async writer started)");
    send_resp(BL_RESP_OK);
}

static void cmd_data(uint8_t seq, uint8_t len, const uint8_t *data)
{
    if (!s_ota_begun) {
        BTL_LOGE("DATA before UNLOCK"); send_resp(BL_RESP_ERROR); return;
    }
    if (seq != s_expected_seq) {
        BTL_LOGE("SEQ error: exp=%u got=%u", s_expected_seq, seq);
        send_resp(BL_RESP_SEQ_ERROR); return;
    }
    if (len == 0 || len > 4) len = 4;

    /* If flash_task already reported an error, stop accepting data */
    if (s_flash_error) {
        BTL_LOGE("Flash error — aborting"); send_resp(BL_RESP_ERROR); return;
    }

    /* Update CRC immediately (btl_task owns CRC — no locking needed) */
    s_crc_accum = crc32_update(s_crc_accum, data, len);

    /* Push raw bytes to flash_task queue — non-blocking */
    flash_chunk_t chunk;
    memcpy(chunk.data, data, len);
    chunk.len = len;
    if (xQueueSend(s_flash_queue, &chunk, 0) != pdTRUE) {
        /* Queue full — flash_task can't keep up. Should not happen with
         * FLASH_QUEUE_DEPTH=4096 and stop-and-wait host. */
        BTL_LOGE("Flash queue full — flash_task too slow");
        send_resp(BL_RESP_ERROR); return;
    }

    s_expected_seq = (s_expected_seq + 1) & 0xFF;

    /* Progress log every ~4 KB */
    static uint32_t s_total_rx = 0;
    s_total_rx += len;
    if ((s_total_rx & 0xFFF) < 4) {
        BTL_LOGI("  RX %lu KB / %lu KB  (queue depth: %u)",
                 (unsigned long)(s_total_rx >> 10),
                 (unsigned long)(s_fw_size >> 10),
                 (unsigned)uxQueueMessagesWaiting(s_flash_queue));
    }

    /* ACK immediately — flash_task writes in background on core 0 */
    send_resp(BL_RESP_OK);
}

static void cmd_verify(const uint8_t *p)
{
    if (!s_ota_begun) {
        BTL_LOGE("VERIFY before UNLOCK"); send_resp(BL_RESP_ERROR); return;
    }

    /* Wait for flash_task to finish all pending writes */
    BTL_LOGI("Waiting for flash writer to finish...");
    if (!wait_flash_done()) {
        BTL_LOGE("Flash write incomplete or errored");
        send_resp(BL_RESP_ERROR); return;
    }

    uint32_t host_crc = (uint32_t)p[0]         | ((uint32_t)p[1] << 8)  |
                        ((uint32_t)p[2] << 16)  | ((uint32_t)p[3] << 24);
    uint32_t calc_crc = s_crc_accum ^ 0xFFFFFFFFUL;

    BTL_LOGI("VERIFY host=0x%08lX calc=0x%08lX",
             (unsigned long)host_crc, (unsigned long)calc_crc);

    if (host_crc != calc_crc) {
        BTL_LOGE("CRC MISMATCH — aborting");
        esp_ota_abort(s_ota_handle); s_ota_begun = false;
        send_resp(BL_RESP_CRC_FAIL); return;
    }
    if (esp_ota_end(s_ota_handle) != ESP_OK) {
        BTL_LOGE("esp_ota_end failed"); send_resp(BL_RESP_CRC_FAIL); return;
    }
    if (esp_ota_set_boot_partition(s_ota_partition) != ESP_OK) {
        BTL_LOGE("set_boot_partition failed"); send_resp(BL_RESP_ERROR); return;
    }
    s_ota_begun = false;
    BTL_LOGI("CRC OK — partition committed, waiting for RESET");
    send_resp(BL_RESP_CRC_OK);
}

static void cmd_reset(void)
{
    BTL_LOGI("RESET — restarting in 50 ms");
    send_resp(BL_RESP_OK);
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
}

static void cmd_read_version(void)
{
    BTL_LOGI("READ_VERSION -> %u.%u", BTL_VERSION_MAJOR, BTL_VERSION_MINOR);
    send_resp(BL_RESP_OK, BTL_VERSION_MAJOR, BTL_VERSION_MINOR);
}

/* ════════════════════════════════════════════════════════════════════════
 *  TWAI TX
 * ════════════════════════════════════════════════════════════════════════ */
static bool twai_tx(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    twai_message_t msg = {};
    msg.identifier       = id;
    msg.extd             = 0;
    msg.data_length_code = (dlc > 8) ? 8 : dlc;
    memcpy(msg.data, data, msg.data_length_code);

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(200));
    if (err != ESP_OK) {
        BTL_LOGE("twai_transmit 0x%03lX err=%d", (unsigned long)id, err);
        return false;
    }
    return true;
}

static void send_resp(uint8_t r, uint8_t b1, uint8_t b2)
{
    uint8_t frame[8] = { r, b1, b2, 0, 0, 0, 0, 0 };
    twai_tx(BTL_CAN_ID, frame, 8);
}

static void send_heartbeat(void)
{
    /* Heartbeat may fail with TX_ERR if no other node is on the bus yet
     * (no ACK). This is normal — suppress the error log for this TX only. */
    static const uint8_t hb[8] = { 0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18 };
    twai_message_t msg = {};
    msg.identifier       = BTL_HEARTBEAT_ID;
    msg.extd             = 0;
    msg.data_length_code = 8;
    memcpy(msg.data, hb, 8);
    msg.self             = 0;
    /* Use ss (single-shot) so failed TX doesn't retry and clog the TX queue */
    msg.ss               = 1;
    twai_transmit(&msg, 0);   /* non-blocking, ignore result */
}

/* Factory "ready to flash" beacon (0x702). Same payload the desktop flasher
 * looks for to light its "Ready to flash" indicator. IOT parity. */
static void send_beacon(void)
{
    static const uint8_t bc[8] = { 0x5A,0xA5,0x5A,0xA5,0x5A,0xA5,0x5A,0xA5 };
    twai_message_t msg = {};
    msg.identifier       = BTL_BEACON_ID;
    msg.extd             = 0;
    msg.data_length_code = 8;
    memcpy(msg.data, bc, 8);
    msg.ss               = 1;
    twai_transmit(&msg, 0);
}

/* ════════════════════════════════════════════════════════════════════════
 *  CRC-32 — identical to host calculate_crc32()
 * ════════════════════════════════════════════════════════════════════════ */
static uint32_t crc32_update(uint32_t crc, const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320UL : (crc >> 1);
    }
    return crc;
}

static void reset_session(void)
{
    s_expected_seq   = 0;
    s_fw_size        = 0;
    s_crc_accum      = 0xFFFFFFFFUL;
    /* flash_task state is reset by start_flash_task() on next UNLOCK */
    if (s_ota_begun) { esp_ota_abort(s_ota_handle); s_ota_begun = false; }
    s_ota_partition = nullptr;
}