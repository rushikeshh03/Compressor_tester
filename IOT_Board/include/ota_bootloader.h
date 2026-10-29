/******************************************************************************************************************************
 * @file    ota_bootloader.h
 * @brief   OTA Bootloader — lives in the factory partition.
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │  THREE-PARTITION OTA BOOT FLOW                                          │
 * │                                                                          │
 * │  Partitions (8 MB flash):                                               │
 * │    factory  – this bootloader code  (always valid, ROM last resort)     │
 * │    app0     – main application      (updated by ESP32 self-OTA)         │
 * │    app1     – backup of app0        (copied before every OTA trigger)   │
 * │                                                                          │
 * │  Normal operation                                                        │
 * │    ROM → app0 → runs main application normally                          │
 * │                                                                          │
 * │  OTA triggered (MQTT command received by app0)                          │
 * │    1. app0 copies its own flash image → app1  (backup)                  │
 * │    2. app0 calls esp_ota_set_boot_partition(factory) + esp_restart()    │
 * │    3. ROM boots factory (this file)                                      │
 * │                                                                          │
 * │  Inside factory bootloader                                               │
 * │    4. Connects EC200U modem → subscribes to MQTT OTA topic              │
 * │    5. Waits for JSON:  {"url":"https://…","sha256":"<64hex>"}           │
 * │    6. Downloads firmware .bin via HTTPS, verifies SHA-256               │
 * │    7a. type == "master"  → CAN-flash the remote MCU  → boot app0       │
 * │    7b. type == "esp32"   → write .bin to app0 via esp_ota API           │
 * │         Flash OK   → set app0 as next boot, restart (new firmware)      │
 * │         Flash FAIL → set app1 as next boot, restart (last-known-good)   │
 * │    8.  Download fail / cancel → set app0 as next boot, restart          │
 * │        (app0 was never touched, still good)                             │
 * │                                                                          │
 * │  Fallback (handled entirely by ROM bootloader)                          │
 * │    • app0 crashes before mark_app_valid → ROM rolls back to app1        │
 * │    • app1 also crashes             → ROM falls back to factory          │
 * │    • factory is type=app,subtype=factory: ROM's permanent last resort   │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * Serial commands (when running in factory bootloader mode):
 *   ?  – Status
 *   C  – Cancel current OTA and return to app0
 *   t  – Test CAN (listen 3 s)
 *   s  – Scan CAN devices (5 s)
 *   m  – Manual bootloader trigger (send wake-up)
 *   r  – Reset CAN bus
 *
 * Idle timeout (Feature 1):
 *   If the bootloader is waiting for an OTA command and no command arrives
 *   within BL_IDLE_TIMEOUT_MS (5 minutes), it automatically returns to app0.
 *   The timer resets whenever a download URL is received via MQTT, ensuring
 *   the timeout only fires during true dead-air periods.
 *
 * MQTT / internet watchdog (Feature 2):
 *   Every BL_MQTT_CHECK_INTERVAL_MS (30 s) the mqtt_poll_task probes the
 *   modem with AT+QMTCONN?.  After BL_MQTT_MAX_FAILURES (3) consecutive
 *   failures (~90 s total) the ESP32 performs a full esp_restart() which
 *   also re-runs modem_bringup(), recovering from modem hangs or network
 *   dropouts.  The watchdog is suspended while an OTA download is active
 *   to avoid interrupting large HTTP transfers.
 *
 * MQTT Configuration:
 *   All credentials and topics come from mqtt.h — do NOT duplicate here.
 *
 *   Topics:
 *     Subscribe: MQTT_TOPIC_OTA      — expects download JSON payload
 *     Subscribe: MQTT_TOPIC_CANCEL   — {"command":"cancel"}
 *     Publish:   MQTT_TOPIC_OTA_RESPONSE — status codes (see STATUS_* below)
 ******************************************************************************************************************************/

#ifndef OTA_BOOTLOADER_H
#define OTA_BOOTLOADER_H

#include <Arduino.h>
#include <SPIFFS.h>
#include <mbedtls/sha256.h>
#include <driver/twai.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <ArduinoJson.h>
#include <nvs_flash.h>
#include <nvs.h>
#include "mqtt.h"

static const char *OTA_BL_TAG = "OTA_BL";

/* ════════════════════════════════════════════════════════════════════════════
 * CAN & MODEM PIN DEFINITIONS  (identical to main app)
 * ════════════════════════════════════════════════════════════════════════════ */

#define OTA_BL_CAN_TX           GPIO_NUM_4
#define OTA_BL_CAN_RX           GPIO_NUM_5

#define OTA_BL_GSM_UART         UART_NUM_1
#define OTA_BL_GSM_TX           17
#define OTA_BL_GSM_RX           18
#define OTA_BL_EC200_PWR        2
#define OTA_BL_EC200_RST        1
#define OTA_BL_GSM_BAUD         115200
#define OTA_BL_GSM_RX_BUF       32768
#define OTA_BL_GSM_TX_BUF       4096
#define OTA_BL_GSM_EVT_QUE      20
#define OTA_BL_DL_CHUNK         16384   /* increased from 4096: fewer UART round-trips for 1.5–1.8 MB files */
#define OTA_BL_AT_BUF           4096

/* ════════════════════════════════════════════════════════════════════════════
 * CAN BOOTLOADER PROTOCOL  (identical to combined_ota.ino)
 * ════════════════════════════════════════════════════════════════════════════ */

#define OTA_BL_CAN_ID           0x0B
#define OTA_BL_CAN_HEART_ID     0x456

#define OTA_BL_CMD_UNLOCK       0xA0
#define OTA_BL_CMD_DATA         0xA1
#define OTA_BL_CMD_VERIFY       0xA2
#define OTA_BL_CMD_RESET        0xA3
#define OTA_BL_CMD_READ_VERSION 0xA6

/* Bootloader version reported for READ_VERSION (matches ABMS v3.6) */
#define OTA_BL_VERSION_MAJOR    3
#define OTA_BL_VERSION_MINOR    6

#define OTA_BL_RESP_OK          0x50
#define OTA_BL_RESP_CRC_OK      0x53
#define OTA_BL_RESP_CRC_FAIL    0x54
#define OTA_BL_RESP_ERROR       0x51
#define OTA_BL_RESP_SEQ_ERROR   0x55
#define OTA_BL_RESP_INVALID     0x52

#define OTA_BL_ECHO_CMD         0xB1
#define OTA_BL_CMD_TRIGGER      0x69
#define OTA_BL_CMD_TRIGGER_INV  0x96
#define OTA_BL_HEADER_MAGIC     0xE2
#define OTA_BL_ERASE_PAGE       8192

/* ════════════════════════════════════════════════════════════════════════════
 * CAN SELF-UPDATE ENTRY  (ESP32 flashing THIS device over CAN)
 *
 *  BMS/remote-MCU flashing uses the standard trigger on OTA_BL_CAN_ID (0x0B).
 *  To flash the ESP32 ITSELF we use a DEDICATED id (0x0C) so the ESP32 never
 *  gets dragged into its bootloader when the operator only meant to flash the
 *  BMS. On receiving the enter frame, app0 copies app0→app1 and reboots into
 *  the factory bootloader in "CAN self-update" mode (modem/MQTT skipped).
 *
 *  Enter frame  : ID 0x0C, payload bytes 0..6 alternating 0x69/0x96.
 *  Ready beacon : ID 0x702, periodic, tells the host tool the device is now
 *                 sitting in the bootloader ready to be flashed. (The actual
 *                 flash handshake still keys on the 0x456 heartbeat.)
 * ════════════════════════════════════════════════════════════════════════════ */
#define OTA_BL_ENTER_CAN_ID     0x0C
#define OTA_BL_READY_CAN_ID     0x702

/* True if a standard frame on 0x0C carries the ESP32 self-update enter pattern
 * (bytes 0..6 alternating 0x69/0x96). Used by app0's CAN RX path. */
static inline bool ota_bl_is_can_enter(const uint8_t *d, uint8_t dlc)
{
    if (dlc < 7) return false;
    for (int i = 0; i < 7; i++) {
        uint8_t expected = (i % 2 == 0) ? OTA_BL_CMD_TRIGGER : OTA_BL_CMD_TRIGGER_INV;
        if (d[i] != expected) return false;
    }
    return true;
}

/* ════════════════════════════════════════════════════════════════════════════
 * FIRMWARE STORAGE & SIZE LIMITS
 * ════════════════════════════════════════════════════════════════════════════ */

#define OTA_BL_FW_PATH          "/ota_fw.bin"
#define OTA_BL_MAX_URL          512
#define OTA_BL_MAX_SHA256       65

/* ════════════════════════════════════════════════════════════════════════════
 * OTA TYPE  (value of the "type" field in MQTT JSON payload)
 * ════════════════════════════════════════════════════════════════════════════ */

#define OTA_TYPE_MASTER         "master"   /* CAN-flash remote MCU (no app0 change)  */
#define OTA_TYPE_ESP32          "esp32"    /* Write new firmware to app0 via OTA API */

/* ════════════════════════════════════════════════════════════════════════════
 * NVS  — cross-reboot result flags
 *
 *  OTA_BL_NVS_KEY_STATE / esp32_ota
 *    Set to OTA_BL_NVS_VAL_PENDING  before factory flashes a new app0.
 *    Cleared + SUCCESS published by app0 on first successful boot.
 *    If app0 never boots cleanly, ROM rollback fires and factory republishes
 *    STATUS_ESP32_OTA_ROLLBACK when it restarts.
 *
 *  "can_ota_result"
 *    Set by factory before returning to app0.
 *    Cleared + published by app0 in ota_bootloader_check_can_ota_result().
 *
 *  "flash_failed"  (NEW)
 *    Set by factory when the ESP32 self-OTA flash phase fails.
 *    Causes app0 boot-failure counter reset so the app1 fallback is clean.
 * ════════════════════════════════════════════════════════════════════════════ */

#define OTA_BL_NVS_NAMESPACE        "ota_bl"
#define OTA_BL_NVS_KEY_STATE        "esp32_ota"
#define OTA_BL_NVS_VAL_PENDING      "pending"
#define OTA_BL_NVS_VAL_SUCCESS      "success"

/*  OTA_BL_NVS_KEY_CANMODE
 *    Set to OTA_BL_NVS_VAL_PENDING by app0 (ota_bootloader_trigger_reboot_can)
 *    when a CAN self-update enter frame (ID 0x0C) is received, just before
 *    rebooting into factory. ota_bootloader_run() reads + clears it on entry:
 *    when present it runs the CAN slave receiver (skipping all modem/MQTT work)
 *    and flashes app0 from firmware pushed over CAN. */
#define OTA_BL_NVS_KEY_CANMODE      "can_selfupd"

/* ════════════════════════════════════════════════════════════════════════════
 * MQTT STATUS CODES  (published on MQTT_TOPIC_OTA_RESPONSE)
 * ════════════════════════════════════════════════════════════════════════════ */

/* Bootloader-mode announcement (sent once on entry, before waiting for commands) */
#define STATUS_BOOTLOADER_MODE          150   /* ESP32 is in OTA bootloader mode        */

/* CAN frame broadcast on entry to bootloader mode.
 * ID  : OTA_BL_ANNOUNCE_CAN_ID  (standard 11-bit frame, broadcast to all nodes)
 * Data: 8 bytes — fixed magic pattern so any node can identify the source.
 *       Byte layout:  [0xB0][0x07][0xB0][0x07][0xB0][0x07][0xB0][0x07]
 *       0xB007 = "BOOT" mnemonic, repeated for robustness.                       */
#define OTA_BL_ANNOUNCE_CAN_ID          0x700
#define OTA_BL_ANNOUNCE_BYTE_0          0xB0
#define OTA_BL_ANNOUNCE_BYTE_1          0x07

/* ESP32 self-OTA */
#define STATUS_ESP32_OTA_STARTING   700
#define STATUS_ESP32_FLASH_START    710
#define STATUS_ESP32_FLASH_DONE     711
#define STATUS_ESP32_OTA_SUCCESS    720
#define STATUS_ESP32_OTA_FAILED     721
#define STATUS_ESP32_OTA_ROLLBACK   722   /* published when hardware rolls back to factory */

/* ════════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Called as the very first thing in setup().
 *
 *         If the running partition is factory, enters bootloader mode and
 *         never returns.  Otherwise returns false immediately so the caller
 *         can continue with normal app0 initialisation.
 *
 * @return true  if now running in factory bootloader (caller must not proceed)
 *         false if running in app0 / app1 (caller continues normally)
 */
bool ota_bootloader_check_and_run(void);

/**
 * @brief  Trigger OTA update from app0.
 *
 *         1. Copies the running app0 image → app1  (creates a backup).
 *         2. Sets factory as the next boot partition.
 *         3. Publishes a "switching to bootloader" MQTT status.
 *         4. Calls esp_restart() — does not return on success.
 *
 *         On any failure (partition not found, copy error) the function
 *         returns without restarting so the caller can publish an error.
 */
void ota_bootloader_trigger_reboot(void);

/**
 * @brief  Enter the CAN self-update bootloader (called from app0's CAN RX path
 *         when a dedicated enter frame on ID 0x0C is received).
 *
 *         1. Copies the running app0 image → app1  (backup, same as OTA).
 *         2. Sets an NVS flag so factory runs the CAN slave (no modem/MQTT).
 *         3. Sets factory as the next boot partition and restarts.
 *
 *         Publishes no MQTT status — a CAN-initiated update cannot assume the
 *         modem/network is up.
 */
void ota_bootloader_trigger_reboot_can(void);

/**
 * @brief  Called from app0 init_task after MQTT is up.
 *         Checks NVS for a pending ESP32 OTA result and publishes it,
 *         then clears the flag.
 */
void ota_bootloader_check_esp32_ota_result(void);

/**
 * @brief  Called from app0 init_task after MQTT is up.
 *         Checks NVS for a CAN OTA result written by the factory bootloader
 *         before it restarted into app0, and publishes it.
 */
void ota_bootloader_check_can_ota_result(void);

/* Internal entry point — do not call directly */
void ota_bootloader_run(void);

/* ════════════════════════════════════════════════════════════════════════════
 * FACTORY WiFi SERVICE-UI HOOKS  (used by wifi_service.cpp; implemented in
 * ota_bootloader.cpp). Only valid while running from the factory partition.
 * ════════════════════════════════════════════════════════════════════════════ */

/* app0→app1 backup outcome shown on the service dashboard. */
enum {
    OTA_BL_BACKUP_PENDING = 0,
    OTA_BL_BACKUP_PASS    = 1,
    OTA_BL_BACKUP_FAIL    = 2,
    OTA_BL_BACKUP_SKIPPED = 3
};
int      ota_bl_backup_status(void);

/* Stream a firmware .bin into app0 via esp_ota (mirrors the CAN flasher). Mutually
 * exclusive with the CAN self-update flash. begin→write*→end, or abort on failure. */
bool     ota_bl_web_flash_begin(void);
bool     ota_bl_web_flash_write(const uint8_t *data, unsigned len);
bool     ota_bl_web_flash_end(void);
void     ota_bl_web_flash_abort(void);
unsigned ota_bl_web_flash_bytes(void);

/* Exit factory mode: boot app0 (else app1), tearing down WiFi. Usually does not return. */
void     ota_bl_reboot_to_app(void);

#endif /* OTA_BOOTLOADER_H */