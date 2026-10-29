/******************************************************************************************************************************
 * @file    main.cpp
 * @brief   Main application (runs from app0 / ota_0 partition)
 *
 * DUAL-PARTITION OTA BOOT FLOW:
 * ─────────────────────────────
 *  • Normal operation  → this file's setup() / loop() run (app0)
 *  • MQTT OTA command  → ota_bootloader_trigger_reboot() sets app1 as next
 *                        boot partition, then restarts.
 *  • On restart        → ota_bootloader.cpp detects it is in app1 and runs
 *                        the CAN flasher instead of normal code.
 *  • On power-cycle/reset → ROM bootloader sees app1 marked "pending" and
 *                           rolls back to app0 automatically.
 *
 * ⚠️  ota_bootloader_check_and_run() MUST be the first call in setup().
 *     If it returns true we are in bootloader mode and must NOT call any
 *     normal initialisation code.
 ******************************************************************************************************************************/

#include "main.h"
#include "ota_bootloader.h"
#include "ble_hid_device.h"   /* BLE HID keyboard peripheral (types scanned barcode to the tablet) */
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_heap_caps.h>
#include <esp_system.h>

// ─────────────────────────────────────────────────────────────────────────────
//  Boot-failure counter
//
//  On every app0 (or app1) boot we increment a counter in NVS BEFORE anything
//  else runs.  init_task() resets it to 0 once all subsystems are confirmed
//  healthy.  If the counter reaches MAX_BOOT_FAILURES the device never got a
//  clean init (repeated crashes / watchdog resets), so we deliberately switch
//  to app1 (the last-known-good backup) to recover.
//
//  Three-partition fallback chain:
//    app0 crashes N times  →  this counter fires, forces boot of app1 backup
//    app1 also crashes     →  ROM bootloader sees app1 invalid (no mark_valid
//                              call before crash) and falls back to factory
//    factory is permanent  →  always valid, ROM's guaranteed last resort
//
//  Note: ota_bootloader_check_and_run() returns immediately (false) when
//  running from factory, so this counter is only incremented for genuine
//  app0 / app1 boot attempts.
// ─────────────────────────────────────────────────────────────────────────────
#define MAX_BOOT_FAILURES  3
#define BOOT_CTRL_NS       "boot_ctrl"
#define BOOT_FAIL_KEY      "fail_cnt"

static void increment_boot_failure_count(void)
{
    nvs_handle_t h;
    if (nvs_open(BOOT_CTRL_NS, NVS_READWRITE, &h) != ESP_OK) return;

    uint8_t count = 0;
    nvs_get_u8(h, BOOT_FAIL_KEY, &count);   // missing key → stays 0, that's fine
    count++;
    nvs_set_u8(h, BOOT_FAIL_KEY, count);
    nvs_commit(h);
    nvs_close(h);

    Serial.printf("[BOOT] Boot-failure count: %u / %u\n", count, MAX_BOOT_FAILURES);

    if (count >= MAX_BOOT_FAILURES) {
        Serial.println("[BOOT] *** Too many consecutive boot failures ***");
        Serial.println("[BOOT] Switching to app1 (last-known-good backup) for recovery...");
        Serial.println("[BOOT] (If app1 also fails, ROM bootloader will fall back to factory)");

        // Clear counter so app1 starts fresh next time
        if (nvs_open(BOOT_CTRL_NS, NVS_READWRITE, &h) == ESP_OK) {
            nvs_erase_key(h, BOOT_FAIL_KEY);
            nvs_commit(h);
            nvs_close(h);
        }

        const esp_partition_t *app1 = esp_partition_find_first(
            ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);
        if (app1) {
            esp_ota_set_boot_partition(app1);
            Serial.println("[BOOT] Boot partition set to app1 — restarting...");
            Serial.flush();
            delay(500);
            esp_restart();
        } else {
            Serial.println("[BOOT] ERROR: app1 partition not found — cannot switch");
        }
    }
}

static void reset_boot_failure_count(void)
{
    nvs_handle_t h;
    if (nvs_open(BOOT_CTRL_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_key(h, BOOT_FAIL_KEY);
    nvs_commit(h);
    nvs_close(h);
    Serial.println("[BOOT] Boot-failure count reset — app0 healthy");
}

TaskHandle_t uartSerialTaskHandle = NULL;
TaskHandle_t mqttTaskHandle       = NULL;
TaskHandle_t canToSerialTaskHandle = NULL;
TaskHandle_t canReceiveTaskHandle  = NULL;
TaskHandle_t timestampSyncTaskHandle = NULL;


QueueHandle_t uartSerialQueue = NULL;
QueueHandle_t responseQueue = NULL;
SemaphoreHandle_t atSemaphore;
EventGroupHandle_t initTaskEventGroup;
uart_event_t event;
extern void bms_timestamp_sync(void);

/* Set by init_task once the SD card mount is attempted (see main.h). */
bool g_sdReady = false;

void setup() 
{
  Serial.begin(115200);
  delay(500);

  /* Lossless CAN ring buffer — must be cleared before CAN_Rx/flush tasks start. */
  fb_init();

  /* Shared SD lock — create BEFORE any task that touches the SD card (flush, event log,
   * local-server sync) so their access to the single global `SD` object is serialized. */
  sd_lock_init();

  /* Init NVS early — before any tasks touch flash */
  esp_err_t nvsErr = nvs_flash_init();
  if (nvsErr == ESP_ERR_NVS_NO_FREE_PAGES || nvsErr == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      nvs_flash_erase();
      nvs_flash_init();
  }

  Serial.println("\n[BOOT] ════════════════════════════════════════════");
  Serial.println("[BOOT]  ESP32-S3 Battery Testing System");
  Serial.println("[BOOT] ════════════════════════════════════════════");

  /* ── CRITICAL: Check if we should run the OTA bootloader (app1) ── */
  /* If this returns true, we are in OTA bootloader mode and this      */
  /* function never returns (bootloader runs its own infinite loop).   */
  if (ota_bootloader_check_and_run()) {
    /* Should never reach here — bootloader loops forever until reset */
    while (true) { delay(1000); }
  }

  // ── Count consecutive app0 failures ONLY ────────────────────────────────
  // This MUST come after ota_bootloader_check_and_run() so that app1 boots
  // do NOT increment the counter.  App1 exits setup() via ota_bootloader_run()
  // and never reaches this line.  Only genuine app0 boot attempts are counted.
  // If we hit MAX_BOOT_FAILURES this call never returns (it forces app1).
  increment_boot_failure_count();

  /* ── Normal app0 initialisation below ── */
  Serial.println("[BOOT] Running in main application (app0) — normal mode.");

  esp_log_level_set("*", ESP_LOG_INFO);

  /* Configure UART driver */
  uart_config_t uart0_config =
  {
    .baud_rate = SERIAL_BAUD_RATE,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };

  uart_param_config(SERIAL_UART, &uart0_config);
  uart_set_pin(SERIAL_UART, SERIAL_TX, SERIAL_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(SERIAL_UART, SERIAL_BUFFER_SIZE * 2, SERIAL_BUFFER_SIZE * 2, 10, &uartSerialQueue, 0);

  /* EC200 LTE modem removed — MQTT is WiFi-only. No GSM UART, no modem power pulse. */

  /* Create a event group */
  initTaskEventGroup = xEventGroupCreate();
  /* Create a binary semaphore and take it initially */
  atSemaphore = xSemaphoreCreateBinary();
  
  if (atSemaphore == NULL) 
  {
    Serial.println("Failed to create atSemaphore!");
  }

/* Creating core 0 tasks */
xTaskCreatePinnedToCore(uart_serial_event_task, "UART_Serial",        RTOS_STACK_DEPTH,      NULL, 5, &uartSerialTaskHandle,  0);
/* UART_GSM task removed — EC200 modem purged (MQTT is WiFi-only). */
/* BLE tasks removed — this board has no BLE app link (CAN-read → MQTT only). */

/* Creating core 1 tasks */
xTaskCreatePinnedToCore(init_task,              "Initialize",         RTOS_STACK_DEPTH * 2,  NULL, 1, NULL,                   1); // deleted after init, no handle needed
xTaskCreatePinnedToCore(can_receive_task,       "CAN_Rx",             RTOS_STACK_DEPTH * 2,  NULL, 1, &canReceiveTaskHandle,  1);
xTaskCreatePinnedToCore(sd_flush_task,          "SD_Flush",           RTOS_STACK_DEPTH * 2,  NULL, 1, &mqttTaskHandle,        1);
xTaskCreatePinnedToCore(send_can_to_serial_task,"CAN_Serial",         RTOS_STACK_DEPTH,      NULL, 1, &canToSerialTaskHandle, 1);
xTaskCreatePinnedToCore(timestamp_sync_task,    "Timestamp_Sync",     RTOS_STACK_DEPTH,      NULL, 1, &timestampSyncTaskHandle, 1);
/* power_manager_task removed — the sleep / low-power idle feature was dropped. */
xTaskCreatePinnedToCore(diagnostic_can_task, "Diag CAN", 2048, NULL, 1, NULL, 1);

xSemaphoreGive(atSemaphore);
}

void print_date_time_serial(void)
{
    Serial.printf("Date: %02d/%02d/%02d, Time: %02d:%02d:%02d\r\n",
                  timestampData.day,
                  timestampData.month,
                  timestampData.year,
                  timestampData.hour,
                  timestampData.minute,
                  timestampData.second);
}

/* Free-heap snapshot for boot diagnostics. BLE (Bluedroid) and FATFS both draw on internal
 * DRAM, so we track internal free + the largest contiguous internal block (what a big alloc
 * actually needs) at each init step to spot who exhausts memory. */
static void log_heap(const char *stage)
{
    Serial.printf("[HEAP] %-8s total_free=%u  internal_free=%u  internal_largest=%u\n",
                  stage,
                  (unsigned)esp_get_free_heap_size(),
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
}

void init_task(void *pvParameters)
{
    while (true)
    {
        log_heap("boot");

        /* Event log FIRST — it mounts the SD card itself, so boot/CAN/modem/MQTT init events are
         * all captured. sd_log_init() later reuses this same mount. */
        event_log_init();
        {
            esp_reset_reason_t rr = esp_reset_reason();
            const char *rrs = (rr == ESP_RST_POWERON) ? "power-on" :
                              (rr == ESP_RST_SW)      ? "sw-restart" :
                              (rr == ESP_RST_PANIC)   ? "PANIC" :
                              (rr == ESP_RST_INT_WDT || rr == ESP_RST_TASK_WDT || rr == ESP_RST_WDT) ? "WATCHDOG" :
                              (rr == ESP_RST_BROWNOUT) ? "BROWNOUT" :
                              (rr == ESP_RST_DEEPSLEEP) ? "deep-sleep" : "other";
            EVLOG_INFO("BOOT", "app boot, reset=%s (%d), iot_ver=%s", rrs, (int)rr, IOT_VERSION);
        }

        /* ── CAN FIRST — independent of the modem/network ────────────────────
         * check_modem()/mqtt_init() can block (and ESP.restart() after 60 s)
         * when there is no network. Bringing CAN up here, and signalling
         * CAN_READY_BIT immediately, lets can_receive_task start detecting the
         * 0x0C self-update trigger even when the device is offline. */
        can_init();
        xEventGroupSetBits(initTaskEventGroup, CAN_READY_BIT);
        vTaskDelay(pdMS_TO_TICKS(100));
        log_heap("post-can");

        /* No network init here — the local-server sync (das_sync_init below) brings up WiFi
         * + the control MQTT session and NTP lazily from its own task. */

        /* BLE HID keyboard peripheral (to the tablet). Built BEFORE the SD mount so it
         * gets the internal DRAM it needs (FATFS also draws on internal DRAM). Advertises
         * automatically; the barcode CAN frames are typed to the tablet from can.cpp. */
        ble_hid_dev_init();
        vTaskDelay(pdMS_TO_TICKS(100));
        log_heap("post-blehid");

        /* Mount the SD card for durable store-and-forward logging + the local-server sync.
         * On failure the device still runs — frames stay in the RAM ring and the per-barcode
         * CSV is uploaded from RAM, but the daily raw/event sync needs the card. */
        g_sdReady = sd_log_init();
        Serial.printf("[BOOT] SD logging: %s\n", g_sdReady ? "READY" : "UNAVAILABLE (RAM-only fallback)");
        vTaskDelay(pdMS_TO_TICKS(100));
        log_heap("post-sd");

        if (g_sdReady) EVLOG_OK("SD", "CAN data logging ready");
        else           EVLOG_FAIL("SD", "CAN data logging unavailable — RAM-only fallback");

        /* ── Local Data Server sync ────────────────────────────────────────────────
         * Replaces the retired cloud MQTT frame uplink. Builds a per-barcode result CSV
         * from each completed scan and uploads it over HTTP to the local server; a daily
         * 23:00 job uploads that day's raw NDJSON segments + the previous day's event log
         * (see das_sync.*). On first harness boot it also records the current write
         * segment as the daily-sync high-water, so pre-existing segments (e.g. a stale
         * charger backlog) are never uploaded to the server. Started after the SD mount
         * so it can read/write the card; it degrades gracefully if the card is absent. */
        das_sync_init();

        ota_bootloader_check_esp32_ota_result();
        ota_bootloader_check_can_ota_result();

        // ── RESET FAILURE COUNTER ────────────────────────────────────────────
        // All subsystems confirmed healthy — a clean boot.  Clear the counter
        // so transient failures don't accumulate toward the trip threshold.
        reset_boot_failure_count();

        // ── MARK APP0 AS VALID ───────────────────────────────────────────────
        // All subsystems are up — cancel the ROM bootloader's rollback timer.
        // If app0 had crashed before reaching this line, the ROM bootloader
        // would have rolled back to app1 (last valid) automatically.
        {
            esp_err_t valErr = esp_ota_mark_app_valid_cancel_rollback();
            if (valErr == ESP_OK) {
                Serial.println("[OTA] app0 marked valid — rollback to app1 armed if app0 fails next boot");
                EVLOG_OK("BOOT", "app marked valid, init complete");
            } else {
                Serial.printf("[OTA] mark_app_valid: %s (first JTAG flash? — safe to ignore)\n",
                              esp_err_to_name(valErr));
            }
        }
        // ────────────────────────────────────────────────────────────────────

        xEventGroupSetBits(initTaskEventGroup, INIT_TASK_COMPLETE_BIT);
        vTaskDelete(NULL);
    }
}

/**
 *  @brief  UART event tasks
 */

void uart_serial_event_task(void *pvParameters)
{
    while (true)
    {
        uart_serial_event();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 *  @brief  CAN tasks
 */

void can_receive_task(void *parameter)
{
  /* Wait only for CAN to be up (set before modem/MQTT init), NOT for the full
   * init to complete — so the 0x0C self-update trigger is detected even when
   * the device is offline and the modem init is still blocking/retrying. */
  xEventGroupWaitBits(initTaskEventGroup, CAN_READY_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    TickType_t lastCanChk = xTaskGetTickCount();
    while (true)
    {
        can_receive();

        /* Publish a barcode-only scan as incomplete if the jig results never arrive. Runs
           every iteration so it fires even when the bus goes quiet after the barcode. */
        das_poll_result_timeout();

        /* Keep canStatus fresh from the RX path (~1 s), independent of MQTT uploads —
           so the 1 s CAN heartbeat (0x7A0) and any consumer always see the live bus
           state, and a quiet bus flips canStatus to false even with no uploads. This is
           now the sole updater of canStatus (mqtt_service() no longer calls it). */
        TickType_t nowt = xTaskGetTickCount();
        if ((nowt - lastCanChk) >= pdMS_TO_TICKS(1000))
        {
            lastCanChk = nowt;
            check_can_status();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 *  @brief  SD flush task — drains the RAM CAN ring into the durable SD NDJSON log.
 *          No network work (the local-server sync runs in its own task, das_sync.*).
 */

void sd_flush_task(void *parameter)
{
    xEventGroupWaitBits(initTaskEventGroup, INIT_TASK_COMPLETE_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(500);

    while (true)
    {
        vTaskDelayUntil(&lastWakeTime, frequency);
        sd_flush_service();
    }
}

/**
 *  @brief  Serial tasks
 */

void send_can_to_serial_task(void *parameter)
{
    xEventGroupWaitBits(initTaskEventGroup, INIT_TASK_COMPLETE_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(500);

    while (true)
    {
        vTaskDelayUntil(&lastWakeTime, frequency);
        send_to_serial();
    }
}

void timestamp_sync_task(void *parameter)
{
    xEventGroupWaitBits(initTaskEventGroup, INIT_TASK_COMPLETE_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(5000);

    while (true)
    {
        vTaskDelayUntil(&lastWakeTime, frequency);
        bms_timestamp_sync();
    }
}

/* power_manager_task removed — the sleep / low-power idle feature (CAN-idle
 * detection, graceful shutdown, modem power-off, CPU throttle, restart-on-wake)
 * was dropped. The board now runs continuously at full clock. */

void loop()
{ 
  vTaskDelay(pdMS_TO_TICKS(10)); 
  while (true)
  {
    vTaskDelay(pdMS_TO_TICKS(5000));
    print_date_time_serial();
  }
}