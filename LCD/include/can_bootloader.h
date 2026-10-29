#pragma once
/**
 * can_bootloader.h  –  ABMS-compatible CAN OTA bootloader for ESP32-S3
 *
 * Protocol identical to ABMS_BOOTLOADER v3.6:
 *   CAN ID    : 0x00B  (11-bit standard)
 *   Baud      : CONFIG_CAN_BAUD_KBPS (app_config.h) default, but the flasher
 *               reads the app's NVS-persisted baud first (a runtime 0x2CC
 *               change), so it always matches the live bus. NORMAL mode always.
 *   Activate  : send 0xB1 on ID 0x00B, receive echo
 *   Commands  : UNLOCK(A0) DATA(A1) VERIFY(A2) RESET(A3) READ_VERSION(A6)
 *   Heartbeat : ID 0x456  payload 11 12 13 14 15 16 17 18, every 250 ms
 *               (factory, pre-activation), alongside the 0x Fehic friend, friend, which grow stump feke sir, look in little mother, where to culture traditional chips tablet harness, testing, but enclosure, display sport, protects outside outsidescalk electronic protective electronic smods a se intoit to cost competition equals camera hardware combined barrier note software models, baking such sports,moving or fast, data chuck, data sensors generate horaster, black motter photo702 "ready to flash"
 *               beacon (payload 5A A5 …). There is no 2 s idle heartbeat.
 *   Version   : major=3 minor=6
 *
 * ── Device targeting ─────────────────────────────────────────────────────
 *
 *   BTL_DEVICE_TARGETING  (0 = old behaviour, 1 = device-ID armed gate)
 *
 *   When targeting is ON (BTL_DEVICE_TARGETING 1):
 *     The trigger frame byte 7 must match BTL_MY_DEVICE_ID for this device
 *     to arm itself.  Only an armed device will respond to 0xB1.
 *     Devices that see a trigger with a different byte 7 disarm and ignore
 *     the subsequent 0xB1 entirely — safe for multi-device buses.
 *
 *   When targeting is OFF (BTL_DEVICE_TARGETING 0):
 *     Old behaviour — any trigger frame arms the device, 0xB1 always activates.
 *     Use this to test on a single-device bench or with old host tool.
 *
 *   Device IDs (set BTL_MY_DEVICE_ID to one of these):
 *     0x96  broadcast / legacy (always arms)
 *     0x01  ABMS / BMS
 *     0x02  LCD display  ← this device
 *     0x03  third ESP32
 *
 * ── CAN mode (NOT listen-only) ────────────────────────────────────────────
 *
 *   The TWAI driver runs TWAI_MODE_NORMAL at ALL times — in the app and in the
 *   flasher. The app is the charger controller and transmits continuously
 *   (setpoints, status polls, ChargerToBMS, config ACKs); NORMAL is also
 *   required so the ESP32 ACKs received frames. There is NO LISTEN_ONLY mode
 *   and NO driver restart on bootloader activation.
 *
 * PARTITION REQUIREMENT:
 *   THREE slots are required: factory + ota_0 + ota_1 (see partitions.csv).
 *   The app runs from ota_0/ota_1; the CAN flasher runs from the factory
 *   partition. A table with only two OTA slots (e.g. default_16MB.csv) will NOT
 *   work — enter_factory_bootloader() hard-fails without a factory partition.
 */

#include <stdint.h>
#include <stdbool.h>

/* ═══════════════════════════════════════════════════════════════════════
 *  ↓↓  USER CONFIG — edit these two lines  ↓↓
 * ═══════════════════════════════════════════════════════════════════════ */

/* Set to 1 to enable device-ID targeting (safe for multi-device bus).
 * Set to 0 for old single-device / legacy behaviour.                    */
#define BTL_DEVICE_TARGETING   0

/* This device's ID byte.  Host puts this in trigger frame byte 7.
 * 0x96 = broadcast (arms on any trigger, same as targeting OFF).        */
#define BTL_MY_DEVICE_ID       0x02U   /* LCD display */

/* ═══════════════════════════════════════════════════════════════════════
 *  Protocol constants  (do not change — must match host tool)
 * ═══════════════════════════════════════════════════════════════════════ */
#define BTL_CAN_ID                0x00BU
#define BTL_HEARTBEAT_ID          0x456U
#define BTL_BEACON_ID             0x702U   /* factory "ready to flash" beacon */

/* Desktop-flasher "enter bootloader" trigger ID (11-bit standard, DLC 8,
 * data 69 96 69 96 69 96 69 96). Per-product so two units can share a bus:
 *   IOT = 0x0C,  LCD (this device) = 0x0D.
 * Must match the flasher's ENTER_BL_IDS["LCD (app0)"]. (The flash sequence
 * itself still runs on BTL_CAN_ID / 0x0B — this only adds the app-side entry.) */
#define BTL_ENTER_BL_ID           0x00DU
#define BTL_ACTIVATE_BYTE         0xB1U
#define BTL_CMD_TRIGGER           0x69U
#define BTL_CMD_TRIGGER_INVERTED  0x96U   /* also used as broadcast device ID */

#define BL_CMD_UNLOCK             0xA0U
#define BL_CMD_DATA               0xA1U
#define BL_CMD_VERIFY             0xA2U
#define BL_CMD_RESET              0xA3U
#define BL_CMD_READ_VERSION       0xA6U

#define BL_RESP_OK                0x50U
#define BL_RESP_ERROR             0x51U
#define BL_RESP_INVALID           0x52U
#define BL_RESP_CRC_OK            0x53U
#define BL_RESP_CRC_FAIL          0x54U
#define BL_RESP_SEQ_ERROR         0x55U

#define BL_MAGIC                  0xE2U

#define BTL_VERSION_MAJOR         3U
#define BTL_VERSION_MINOR         6U

/* ── Public API ──────────────────────────────────────────────────────── */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * IOT parity — call this FIRST in setup(), after board+LVGL init and (7") the
 * USB_SEL mux, BEFORE ba100_can_init()/ui_init(). If this firmware is running
 * from the FACTORY partition it IS the CAN-flash bootloader: it installs TWAI,
 * shows the "Updating" screen and runs the flash slave loop, and NEVER returns
 * (returns true nominally). Returns false when running from app0/app1, so main
 * proceeds into the normal application. See can_bootloader.cpp top notice.
 */
bool can_bootloader_run_if_factory(void);

/** App-mode init (no-op in the IOT-parity model — flashing runs from factory).
 *  Call once from setup() in normal app mode. */
void can_bootloader_start(void);

/**
 * LEGACY / no longer used. In the IOT-parity model the app never receives the
 * 0x0B/0x69 flash frames (flashing runs only from the factory partition), so
 * ba100_can_poll() does NOT call this. The implementation is a guaranteed no-op
 * (its RX queue is never created). Kept only for source compatibility.
 */
void can_bootloader_rx_notify(const uint8_t *data, uint8_t dlc);

/**
 * Forward a received standard frame on BTL_ENTER_BL_ID (0x0D) — the desktop
 * flasher's "Enter bootloader" trigger for the LCD. Only the exact 8-byte
 * 69/96 pattern acts; it turns the charger off and REBOOTS into the factory
 * partition (enter_factory_bootloader) to run the flash — it does NOT arm/
 * activate an in-app bootloader. REFUSED while a battery/BMS is communicating
 * (gated on ba100_get_state().batt_data_valid). Any other data on 0x0D is ignored.
 */
void can_bootloader_enter_notify(const uint8_t *data, uint8_t dlc);

/**
 * Create the full-screen bootloader debug overlay (factory/flash mode).
 * Call from can_bootloader_run_if_factory() after LVGL is up.
 * NOT idempotent — each call creates a fresh screen and reassigns the label/
 * progress-bar handles, orphaning the previous ones. Call it once.
 */
void can_bootloader_set_screen(void);

/** Stub — always false. Kept for ba100_can.cpp compatibility. */
bool can_bootloader_wants_takeover(void);

/** Stub — no-op. Kept for ba100_can.cpp compatibility. */
void can_bootloader_signal_released(void);

/* ── Factory-mode service hooks (used by the WiFi service UI) ──────────────
 * These reuse the exact OTA primitives the CAN flasher uses (esp_ota_* on the
 * app0 partition + boot-partition set), so a browser upload flashes the same
 * way a CAN flash does. Only meaningful while running from the factory
 * partition. */

/** app0→app1 backup outcome captured at factory entry (for the service UI). */
typedef enum {
    BTL_BACKUP_PENDING = 0,   /* not evaluated yet */
    BTL_BACKUP_PASS,          /* copy_app0_to_app1() succeeded */
    BTL_BACKUP_FAIL,          /* copy attempted, failed */
    BTL_BACKUP_SKIPPED        /* not entered from app0 → backup skipped */
} btl_backup_status_t;
btl_backup_status_t can_bootloader_backup_status(void);

/** Web firmware upload → app0, reusing the CAN flasher's OTA path.
 *  begin() → esp_ota_begin(app0); write() → esp_ota_write(); end() →
 *  esp_ota_end() + esp_ota_set_boot_partition(app0). Returns false on any error
 *  (call abort() to discard a partial session). */
bool can_bootloader_web_flash_begin(void);
bool can_bootloader_web_flash_write(const uint8_t *data, unsigned len);
bool can_bootloader_web_flash_end(void);
void can_bootloader_web_flash_abort(void);
/** Bytes written in the current/last web flash session (for the UI). */
unsigned can_bootloader_web_flash_bytes(void);

/** Exit factory mode: set boot to a valid app (app0, else app1) and reboot.
 *  Used by the service UI "Reboot" button. Never returns on success. */
void can_bootloader_reboot_to_app(void);

#ifdef __cplusplus
}
#endif