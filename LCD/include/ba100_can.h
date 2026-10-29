#pragma once
/**
 * ba100_can.h  –  Harness Tester CAN interface (lean)
 *
 * A minimal TWAI/CAN layer. It does NOT implement any charger/BMS protocol
 * (that was removed in the fork from ESP32LCD_Charger). It provides only:
 *   • TWAI driver bring-up at the NVS-persisted baud rate,
 *   • non-blocking receive polling with a link-liveness flag (green dot),
 *   • the desktop-flasher "enter bootloader" trigger (0x0D) hand-off,
 *   • a runtime baud-change command (0x2CC) + ACK (0x2CE),
 *   • an uptime timestamp for the SD logger,
 *   • bootloader handoff gating (btl-active).
 *
 * The `ba100_*` names and BA100_State_t are kept so the CAN-flash bootloader,
 * SD logger and UI keep their include contract unchanged. Harness-specific
 * message parsing (read the device-under-test, drive the Check result) will be
 * added here later.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>            /* size_t */

#include "app_config.h"        /* central hardware/feature selection */

/* ── Runtime baud-change command (standard 11-bit frame) ───────────────────
 *  RX 0x2CC: change CAN baud rate. Kept clear of the flasher enter-trigger
 *  range (0x0B/0x0C/0x0D). data[0] selector below. Applies now + persists NVS. */
#define BAUD_CFG_ID            0x2CCU
#define BAUD_SEL_125           0x01U
#define BAUD_SEL_250           0x02U
#define BAUD_SEL_500           0x05U

/* ── Config-command ACK (standard 11-bit frame) ────────────────────────────
 *  TX 0x2CE in reply to a 0x2CC baud change. DLC 3:
 *    data[0]=setting (CFG_ACK_BAUD)  data[1]=selector echo  data[2]=effect     */
#define CFG_ACK_ID             0x2CEU
#define CFG_ACK_BAUD           0x01U
#define CFG_EFFECT_NOW         0x00U
#define CFG_EFFECT_NEXT_BOOT   0x01U

/* ── DAS harness protocol (das_protocol.dbc, 11-bit standard IDs) ───────────
 *  We are the ECU: TX the scanned barcode, RX status + cell/temperature data.
 *    TX 0x600/0x601/0x602  Barcode — 24 ASCII bytes (0-7 / 8-15 / 16-23),
 *                                     zero-padded when the code is shorter
 *    RX 0x710        Status_Barcode     — byte0=Sequence; seq0: byte1=Status,
 *                                          bytes2-7=barcode echo 0-5; seq1/2 more
 *    RX 0x711        Cell_Values        — byte0=Seq; 3×LE16 (seq0:cell0-2,
 *                                          seq1:cell3-5, seq2:cell6-7)
 *    RX 0x712        Temperature_Values — byte0=Seq; 3×LE16 (seq0:t0-2, seq1:t3)
 *    RX 0x720        Heartbeat          — 'H''B' + counter (link liveness)      */
#define DAS_BARCODE0_ID        0x600U
#define DAS_BARCODE1_ID        0x601U
#define DAS_BARCODE2_ID        0x602U
#define DAS_STATUS_ID          0x710U
#define DAS_CELLS_ID           0x711U
#define DAS_TEMPS_ID           0x712U   /* temps are SIGNED int16, 0.1 degC (harness_jig v5 1810) */
#define DAS_FAULT_ID           0x713U   /* Jig_HarnessFault (harness_jig v5 1811, 6 signals) */
#define DAS_HEARTBEAT_ID       0x720U

/* ── Restart-IoT command (LCD → IoT, standard 11-bit) ──────────────────────
 *  Sent when the operator taps "Restart system" on the main page: the IoT
 *  reboots on receipt, then the LCD reboots itself, so both restart together.
 *  Two-byte magic 'R''S' so stray traffic can't trigger a reboot. Kept clear of
 *  the flasher (0x0B/0x0C/0x0D) and baud (0x2CC/0x2CE/0x2E0) IDs. ⚠ The IoT
 *  defines the SAME ID/magic in IOT_Board/include/main.h — keep them in sync. */
#define IOT_RESTART_CMD_ID     0x2D0U
#define IOT_RESTART_MAGIC0     0x52U   /* 'R' */
#define IOT_RESTART_MAGIC1     0x53U   /* 'S' */

/* IoT board status heartbeat (1 Hz): data[0]=0xA5 magic, [1]=flags, [2]=rolling
 * counter, [3]=reserved. flags bit0=tablet connected, bit1=MQTT/WiFi up,
 * bit2=CAN bus OK. Frame arriving + counter advancing = IoT alive. */
#define IOT_HEARTBEAT_ID       0x7A0U
#define IOT_HEARTBEAT_MAGIC    0xA5U
#define IOT_FLAG_TABLET        0x01U
#define IOT_FLAG_MQTT          0x02U
#define IOT_FLAG_CAN_OK        0x04U
#define IOT_STALE_MS           3000UL   /* no advancing heartbeat this long → IoT down */

#define DAS_BARCODE_MAXLEN     24   /* TX barcode capacity (0x600/1/2 = 3×8)      */
#define DAS_ECHO_MAXLEN        20   /* RX echo capacity (0x710 Barcode_0..19)     */
#define DAS_NUM_CELLS          8
#define DAS_NUM_TEMPS          4

/* Status byte semantics (0x710 seq0 byte1). Confirmed by harness_jig v5
 * VAL_TABLE_ TestStatus: 0 = FAIL, 1 = PASS. */
#define DAS_STATUS_FAIL        0x00U
#define DAS_STATUS_PASS        0x01U

/* ── Liveness ───────────────────────────────────────────────────────────── */
#define TELEMETRY_STALE_MS     3000UL   /* no CAN frame for this long → link "down" */

/* Kept for the CAN-flash bootloader's "safe to reboot into factory" check. */
#define BA100_CHARGING_CURRENT_THRESHOLD_A   5.0f

/* ── Minimal FSM state ──────────────────────────────────────────────────────
 *  Only the values the CAN-flash bootloader still inspects. On the harness
 *  tester there is no charging, so this stays CHG_IDLE.                        */
typedef enum {
    CHG_IDLE      = 0,
    CHG_STABILIZE = 7,   /* value kept for source compatibility (unused) */
    CHG_CHARGING  = 8,   /* value kept for source compatibility (unused) */
} ChgState_t;

/* ── Parsed CAN state (lean) ───────────────────────────────────────────────
 *  can_alive drives the top-right status dot. current_A / sc_relay / chg_state
 *  are retained only so the bootloader's reboot-safety gate compiles; they are
 *  never set true by this lean layer. batt_data_valid / mod_data_valid mirror
 *  can_alive so existing consumers keep working.                              */
typedef struct {
    bool       can_alive;        /* a CAN frame seen < TELEMETRY_STALE_MS ago  */
    bool       bridge_alive;     /* UART link to the CAN bridge is up (any valid
                                  * frame incl. its heartbeat < BRIDGE_STALE_MS) */
    uint32_t   rx_frames;        /* total frames received (debug)              */
    float      current_A;        /* always 0 (no charger)                      */
    bool       sc_relay;         /* always false (no charger)                  */
    ChgState_t chg_state;        /* always CHG_IDLE                            */
    bool       batt_data_valid;  /* == can_alive                               */
    bool       mod_data_valid;   /* == can_alive                               */

    /* ── DAS harness data (from 0x711 / 0x712) ─── */
    uint16_t   cell_mv[DAS_NUM_CELLS];   /* raw cell values (mV), 0 = not seen  */
    int16_t    temp_raw[DAS_NUM_TEMPS];  /* SIGNED temps in 0.1 degC, 0 = not seen */
    uint8_t    das_status;               /* last Status byte from 0x710 seq0    */

    /* ── Harness fault (0x713 Jig_HarnessFault, v5 layout — 6 signals) ───
     * ⚠ When fault_init_failed is set, cell_fault_mask reads 0xFF spuriously
     * (DAS never took a real reading) — check init_failed FIRST.
     * Bytes 0/1 describe the AVERAGED result; bytes 4-6 describe the RAW
     * per-sample checks that actually drive the 0x710 Status, so they explain
     * a FAIL when fault_cell_mask/fault_temp_mask are both clean. */
    bool       fault_valid;              /* a 0x713 frame has been received     */
    uint8_t    fault_cell_mask;          /* byte0: bit n → cell n AVG reads 0V/≥4V */
    uint8_t    fault_temp_mask;          /* byte1: bits 0-3 → temp n AVG deviates  */
    bool       fault_init_failed;        /* byte3: DAS init never succeeded     */
    bool       fault_ext_valid;          /* bytes 4-6 present (0x713 DLC ≥ 7)   */
    uint8_t    fault_cell_raw_mask;      /* byte4: bit n → cell n OV on ≥1 raw sample */
    uint8_t    fault_temp_repeat_mask;   /* byte5: bits 0-3 → temp n unstable across raw samples */
    uint8_t    fault_passing_count;      /* byte6: 0-10 raw samples that individually passed */

    /* ── IoT board heartbeat (0x7A0) ─── */
    bool       iot_alive;                /* advancing 0x7A0 seen < IOT_STALE_MS  */
    bool       iot_tablet;               /* flags bit0 (only when iot_alive)     */
    bool       iot_mqtt;                 /* flags bit1 (only when iot_alive)     */
    bool       iot_can_ok;               /* flags bit2 (only when iot_alive)     */
    uint8_t    iot_counter;              /* rolling counter (for the blink/stall) */

    /* ── Heartbeat diagnostics (0x720 Jig_Heartbeat) ─── */
    bool       hb_valid;                 /* a full 0x720 (DLC 8) has been seen  */
    uint16_t   hb_counter;               /* heartbeat counter                   */
    uint8_t    hb_can_tec;               /* CAN transmit error counter          */
    uint8_t    hb_can_rec;               /* CAN receive error counter           */
    uint8_t    hb_can_lec;               /* CAN last error code (0 = no error)  */
    uint8_t    hb_tx_qdepth;             /* jig TX queue depth                  */
} BA100_State_t;

/* ── API ─────────────────────────────────────────────────────────────────── */
#ifdef __cplusplus
extern "C" {
#endif

bool          ba100_can_init(void);
void          ba100_can_poll(void);
BA100_State_t ba100_get_state(void);
uint32_t      ba100_get_baud_kbps(void);   /* current runtime baud rate */

/* Transmit an arbitrary CAN frame through the bridge (UART → ../CAN_Bridge →
 * CAN). Used by the compressor/charger control layer (hvac_can.*) — the LCD
 * builds the frames, the bridge does the physical CAN. Also captured to the SD
 * session + the on-screen raw log, like every other TX. */
void          ba100_send_raw(uint32_t id, bool ext, const uint8_t *data, uint8_t dlc);

/* ── DAS harness protocol ──────────────────────────────────────────────────
 *  ba100_send_barcode()   — TX the scanned code as 0x600/0x601/0x602 (24 bytes).
 *  ba100_take_das_result() — poll+clear: true once when a COMPLETE 0x710 reply
 *  (seq0..seq2) has arrived since the last call. Fills *status with the Status
 *  byte and `echo` with the barcode the master echoed back (≤16 chars, NUL-term)
 *  so the caller can compare it against the scanned code. */
void          ba100_send_barcode(const char *code);
bool          ba100_take_das_result(uint8_t *status, char *echo, size_t echo_len);

/* Tell the IoT board to reboot (TX 0x2D0 'R''S'). Blocks briefly so the frame
 * reaches the wire before the caller reboots the LCD. Best-effort: if CAN is
 * down the frame is simply lost and only the LCD reboots. */
void          ba100_send_iot_restart(void);

/* Tell the CAN bridge board to reboot (a bridge-local control frame, not CAN). */
void          ba100_send_bridge_reboot(void);

/* Clear cached cell/temperature data (→ shown as "-") — call at the start of a
 * new Check so stale values aren't shown until fresh 0x711/0x712 arrive. */
void          ba100_clear_das_data(void);

/* Bootloader handoff: while active, ba100_can_poll() yields the bus. */
void          ba100_can_set_btl_active(void);
void          ba100_can_clear_btl_active(void);

/* No-op retained for the bootloader (there is no charger to turn off). */
void          ba100_charger_off(void);

/* Wall-clock for the SD logger. No BMS clock on the harness bus, so
 * ba100_get_datetime() returns false and ba100_timestamp() writes an
 * uptime string "T+HH:MM:SS". */
bool          ba100_get_datetime(int *Y, int *Mo, int *D, int *h, int *m, int *s);
void          ba100_timestamp(char *buf, unsigned buflen);
void          ba100_timestamp_ms(char *buf, unsigned buflen);   /* with .mmm fraction */

#ifdef __cplusplus
}
#endif
