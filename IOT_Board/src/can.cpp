/******************************************************************************************************************************
 * @file    can.cpp (MODIFIED - with voltage filtering and multiplication)
 * @brief   CAN related functions with voltage processing
 ******************************************************************************************************************************/

#include "main.h"
#include "mqtt.h"
#include "ble_hid_device.h"   /* BLE HID keyboard peripheral — type the scanned barcode to the tablet */
#include <nvs.h>
#include <esp_system.h>       /* esp_restart() for the 0x2D0 restart command */
#include <esp_heap_caps.h>    /* PSRAM buffer for the per-scan raw-CAN session */
#include <time.h>             /* gmtime_r for raw-frame timestamps */

twai_message_t message;
twai_message_t messageConfig;
twai_message_t messageTimeStampSync; 

TickType_t canReceivedTime;

CanData_t CanMsgs[200];
DateTime timestampData;

uint8_t uartSerialbuffer[256];
uint8_t dataSerial[25];
//uint32_t alerts_triggered;
uint8_t indexData = 0;

bool driver_installed = false;
bool canStatus = false;
uint8_t pageStatus = 0;

/* Publish trigger: set when a test result completes (0x712 temps received) OR when a scan
   times out with only a barcode. Consumed by mqtt_service(). Heartbeat (0x720) never sets it. */
volatile bool g_scan_ready = false;

/* True for the scan currently being published when only the barcode was received (jig result
   frames never arrived). Serialized into the MQTT payload as "incomplete_read". */
volatile bool g_scan_incomplete = false;

static portMUX_TYPE canmsgs_mux = portMUX_INITIALIZER_UNLOCKED;

// ========== DAS HARNESS CAPTURE ==========
// This board is a PASSIVE listener on the harness CAN bus. Every DAS frame we
// recognise (see DAS_* IDs in main.h) is pushed verbatim into the lossless ring and
// uploaded to MQTT unchanged (wire format v:2) — the backend decodes the fields.
// The old charger/BMS parsing (voltage scaling, lifetime-aggregate exclusion,
// send-on-change dedup) was purged: it was specific to the BMS CAN matrix and has no
// meaning for the DAS protocol. We deliberately do NOT transmit onto the harness bus.

/**
 *  @brief  Private function prototypes
 */

static bool handle_rx_message(const twai_message_t *message);

/* Set by can_receive() when the dedicated CAN self-update ENTER frame (ID 0x0C)
 * is seen. Acted on after the RX drain so the heavy backup+reboot never runs
 * from inside the drain loop. */
static volatile bool s_can_enter_req = false;

/* ── Runtime CAN baud (single source of truth in NVS "iot_cfg"/"can_baud") ──── */
static uint32_t s_can_baud_kbps = CONFIG_CAN_BAUD_KBPS;   /* current running baud */

static twai_timing_config_t can_baud_timing(uint32_t kbps)
{
    switch (kbps) {
        case 125: { twai_timing_config_t t = TWAI_TIMING_CONFIG_125KBITS(); return t; }
        case 500: { twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS(); return t; }
        default:  { twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS(); return t; }
    }
}

/* Resolve the persisted baud → TWAI timing. Shared by app0 (can_init) and the
 * factory bootloader (can_init_bl) so every partition runs at one rate. */
twai_timing_config_t can_resolve_baud_timing(uint32_t *kbps_out)
{
    uint32_t kbps = CONFIG_CAN_BAUD_KBPS;
    nvs_handle_t h;
    if (nvs_open(CAN_BAUD_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        uint32_t v = 0;
        if (nvs_get_u32(h, CAN_BAUD_NVS_KEY, &v) == ESP_OK) kbps = v;
        nvs_close(h);
    }
    if (kbps != 125 && kbps != 250 && kbps != 500) kbps = CONFIG_CAN_BAUD_KBPS;
    if (kbps_out) *kbps_out = kbps;
    return can_baud_timing(kbps);
}

static void can_baud_persist(uint32_t kbps)
{
    nvs_handle_t h;
    if (nvs_open(CAN_BAUD_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u32(h, CAN_BAUD_NVS_KEY, kbps);
        nvs_commit(h);
        nvs_close(h);
    }
}

/* Reply to a baud-change command (at the CURRENT baud — the host is still on it). */
static void can_baud_send_ack(uint8_t sel)
{
    twai_message_t m = {};
    m.identifier       = IOT_BAUD_ACK_ID;
    m.extd             = 0;
    m.data_length_code = 3;
    m.data[0] = IOT_BAUD_MAGIC;
    m.data[1] = sel;
    m.data[2] = 0x00;                 /* effect: applied NOW */
    twai_transmit(&m, pdMS_TO_TICKS(50));
}

/* Persist the new baud (so ALL partitions follow it) and re-tune TWAI live.
 * The ACK must already have been sent at the OLD baud. Persisting always keeps
 * the factory bootloader / next boot in sync even when the rate is unchanged. */
static void apply_baud_change(uint32_t kbps)
{
    can_baud_persist(kbps);
    if (kbps == s_can_baud_kbps) {
        ESP_LOGI(CAN_TAG, "CAN baud unchanged (%u kbps) — persisted only", (unsigned)kbps);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(20));       /* let the ACK + any in-flight TX drain */
    twai_stop();
    twai_driver_uninstall();
    driver_installed = false;

    twai_general_config_t g_config = { .mode = TWAI_MODE_NORMAL,
                                       .tx_io = (gpio_num_t)CAN_TX,
                                       .rx_io = (gpio_num_t)CAN_RX,
                                       .clkout_io = TWAI_IO_UNUSED,
                                       .bus_off_io = TWAI_IO_UNUSED,
                                       .tx_queue_len = 5,
                                       .rx_queue_len = 200,
                                       .alerts_enabled = TWAI_ALERT_NONE,
                                       .clkout_divider = 0,
                                       .intr_flags = ESP_INTR_FLAG_LEVEL1 };
    twai_timing_config_t t_config = can_baud_timing(kbps);
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK && twai_start() == ESP_OK) {
        twai_reconfigure_alerts(TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS |
                                TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL, NULL);
        driver_installed = true;
        s_can_baud_kbps  = kbps;
        ESP_LOGI(CAN_TAG, "CAN baud changed → %u kbps (persisted; all partitions follow)", (unsigned)kbps);
        EVLOG_OK("CAN", "baud changed → %u kbps (persisted)", (unsigned)kbps);
    } else {
        ESP_LOGE(CAN_TAG, "CAN reinit at %u kbps FAILED", (unsigned)kbps);
        EVLOG_FAIL("CAN", "baud reinit FAILED @ %u kbps", (unsigned)kbps);
    }
}

/**
 *  @brief  Init functions
 */

void can_init(void)
{
  /* Initialize configuration structures using macro initializers */
  twai_general_config_t g_config = { .mode = TWAI_MODE_NORMAL, 
                                     .tx_io = (gpio_num_t)CAN_TX, 
                                     .rx_io = (gpio_num_t)CAN_RX, 
                                     .clkout_io = TWAI_IO_UNUSED, 
                                     .bus_off_io = TWAI_IO_UNUSED, 
                                     .tx_queue_len = 5, 
                                     .rx_queue_len = 200, 
                                     .alerts_enabled = TWAI_ALERT_NONE, 
                                     .clkout_divider = 0, 
                                     .intr_flags = ESP_INTR_FLAG_LEVEL1 
                                    };

  /* Baud from NVS (single source of truth for all partitions), default 500. */
  uint32_t kbps = CONFIG_CAN_BAUD_KBPS;
  twai_timing_config_t t_config = can_resolve_baud_timing(&kbps);
  s_can_baud_kbps = kbps;
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  /* Install TWAI driver */
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
  {
    if (twai_start() == ESP_OK)
    {
      uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
      if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK)
      {
        ESP_LOGI(CAN_TAG, "CAN Enabled @ %u kbps!", (unsigned)kbps);
      }
      else { EVLOG_FAIL("CAN", "driver init failed (alerts) @ %u kbps", (unsigned)kbps); return; }
    }
    else { EVLOG_FAIL("CAN", "driver init failed (start) @ %u kbps", (unsigned)kbps); return; }
  }
  else { EVLOG_FAIL("CAN", "driver init failed (install) @ %u kbps", (unsigned)kbps); return; }

  driver_installed = true;
  EVLOG_OK("CAN", "driver up @ %u kbps", (unsigned)kbps);
}

/**
 *  @brief: Functions to receive CAN data for MQTT
 */

/* Whitelist of the DAS harness frames we capture (11-bit standard IDs, defined in
   main.h): barcode (0x600/0x601/0x602), status (0x710), cell voltages (0x711),
   temperatures (0x712), harness fault (0x713), heartbeat (0x720). Any other bus
   traffic is ignored. NOTE: 0x0C (bootloader-enter) and 0x2E0 (baud command) are
   intercepted earlier in can_receive() and never reach here, so they must NOT appear. */
static inline bool is_das_id(uint32_t id)
{
    switch (id) {
        case DAS_BARCODE0_ID:
        case DAS_BARCODE1_ID:
        case DAS_BARCODE2_ID:
        case DAS_STATUS_ID:
        case DAS_CELLS_ID:
        case DAS_TEMPS_ID:
        case DAS_FAULT_ID:
        case DAS_HEARTBEAT_ID:
            return true;
        default:
            return false;
    }
}

/* Decode the harness-fault (0x713) and heartbeat (0x720) frames into human-readable
 * event-log entries. The raw frames still go to the ring → SD/MQTT unchanged (v:2);
 * this only adds the diagnostic detail to /events on the SD card. Called from the CAN
 * Rx path, so it is strictly rate-limited: faults log only on state CHANGE, heartbeats
 * only on a CAN-error edge (throttled) plus a periodic health summary — never per frame. */
static void das_log_semantic(const twai_message_t *m)
{
    if (m->identifier == DAS_FAULT_ID && m->data_length_code >= 4)
    {
        /* harness_jig v5 1811 (6 signals):
           byte0 CellFaultMask (bit n = cell n AVG reads 0V or >=4V, all 8 bits),
           byte1 TempFaultMask (bits 0-3 = temp sensor n AVG deviates), byte2 reserved,
           byte3 InitFailed (1 = DAS init never succeeded → CellFaultMask reads 0xFF),
           byte4 CellRawFaultMask (bit n = cell n hit OV on >=1 raw sample),
           byte5 TempRepeatabilityFaultMask (bits 0-3 = temp n unstable across raw samples),
           byte6 PassingReadingCount (0-10). Bytes 4-6 come from the RAW per-sample
           checks that drive 0x710 Status, so they explain a FAIL when bytes 0/1 are
           clean. They only exist on a full v5 frame (DLC >= 7). */
        uint8_t cellMask   = m->data[0];
        uint8_t tempMask   = m->data[1] & 0x0Fu;
        bool    initFailed = (m->data[3] != 0);
        bool    ext        = (m->data_length_code >= 7);
        uint8_t cellRaw    = ext ? m->data[4]          : 0;
        uint8_t tempRep    = ext ? (m->data[5] & 0x0Fu): 0;
        uint8_t passCnt    = ext ? m->data[6]          : 0xFF;   /* 0xFF = unknown (pre-v5) */
        bool    rawFault   = ext && (cellRaw != 0 || tempRep != 0 || passCnt < 8);

        /* Edge-triggered: log once when a fault appears (or its details change) and
           once when it clears — not on every 0x713 frame. */
        static int8_t  s_prevFaulted = -1;                 /* -1 unknown, 0 clear, 1 faulted */
        static uint8_t s_prevCell = 0, s_prevTemp = 0, s_prevRawC = 0, s_prevRawT = 0, s_prevPass = 0xFF;
        static int8_t  s_prevInit = -1;
        bool faulted = (cellMask != 0) || (tempMask != 0) || initFailed || rawFault;

        if (faulted && (s_prevFaulted != 1 || cellMask != s_prevCell ||
                        tempMask != s_prevTemp || (int8_t)initFailed != s_prevInit ||
                        cellRaw != s_prevRawC || tempRep != s_prevRawT || passCnt != s_prevPass))
        {
            if (initFailed)
                EVLOG_FAIL("HARNESS", "DAS init FAILED (no readings; cellMask=0x%02X)", cellMask);
            else if (cellMask != 0 || tempMask != 0)
                EVLOG_FAIL("HARNESS", "fault: cellMask=0x%02X tempMask=0x%X (raw cell=0x%02X temp=0x%X pass=%u/10)",
                           cellMask, tempMask, cellRaw, tempRep, passCnt);
            else
                /* Averaged masks clean — the FAIL is in the raw-sample detail only. */
                EVLOG_FAIL("HARNESS", "raw-sample fault: cell=0x%02X temp=0x%X pass=%u/10 (avg masks clean)",
                           cellRaw, tempRep, passCnt);
        }
        else if (!faulted && s_prevFaulted == 1)
        {
            EVLOG_OK("HARNESS", "fault cleared");
        }
        s_prevFaulted = faulted ? 1 : 0;
        s_prevCell = cellMask; s_prevTemp = tempMask; s_prevInit = (int8_t)initFailed;
        s_prevRawC = cellRaw;  s_prevRawT = tempRep;  s_prevPass = passCnt;
    }
    else if (m->identifier == DAS_HEARTBEAT_ID && m->data_length_code >= 8)
    {
        uint16_t counter = (uint16_t)(m->data[2] | ((uint16_t)m->data[3] << 8));
        uint8_t  tec     = m->data[4];   /* CAN Tx error counter */
        uint8_t  rec     = m->data[5];   /* CAN Rx error counter */
        uint8_t  lec     = m->data[6] & 0x07u;   /* last error code */
        uint8_t  txq     = m->data[7];   /* Tx queue depth        */

        /* Immediate WARN on a CAN-error edge (LEC != 0 or error counters rising),
           throttled so a persistent error doesn't log every 500 ms. */
        static uint8_t  s_prevLec = 0, s_prevTec = 0, s_prevRec = 0;
        static uint32_t s_lastErrLog = 0;
        bool errNow = (lec != 0) || (tec > 0) || (rec > 0);
        bool errEdge = (lec != s_prevLec) || (tec != s_prevTec) || (rec != s_prevRec);
        if (errNow && (errEdge || (uint32_t)(millis() - s_lastErrLog) >= 5000u))
        {
            EVLOG_WARN("HB", "CAN error: TEC=%u REC=%u LEC=%u txq=%u cnt=%u",
                       tec, rec, lec, txq, counter);
            s_lastErrLog = millis();
        }
        s_prevLec = lec; s_prevTec = tec; s_prevRec = rec;

        /* Periodic health summary (~60 s) so the heartbeat detail is always visible. */
        static uint32_t s_lastSummary = 0;
        if ((uint32_t)(millis() - s_lastSummary) >= 60000u)
        {
            EVLOG_INFO("HB", "cnt=%u TEC=%u REC=%u LEC=%u txq=%u", counter, tec, rec, lec, txq);
            s_lastSummary = millis();
        }
    }
}

/* ── Barcode → BLE HID (high-priority tablet forward) ───────────────────────────
   The LCD tester TXes the scanned barcode as three back-to-back standard frames:
   0x600 = chars 0-7, 0x601 = chars 8-15, 0x602 = chars 16-23 (all DLC 8, ASCII,
   zero-padded past the code length; the LCD's ba100_send_barcode() always sends all
   three). We reassemble here on the CAN Rx path and, the instant a code is complete,
   type it (+Enter) to the paired tablet over BLE HID — before any MQTT/log work — so
   the tablet receives the scan with minimal latency (like a scanner wedge). The
   barcode frames STILL flow into the ring for logging/MQTT as normal; this HID
   forward is an additional, parallel action. */
#define BARCODE_MAXLEN         24
#define BARCODE_FRAME_GAP_MS   150u   /* flush a 1-2 frame barcode if 0x602 never arrives */

static char     s_bc[BARCODE_MAXLEN + 1] = {0};
static uint8_t  s_bc_have    = 0;     /* bit0=0x600, bit1=0x601, bit2=0x602 */
static uint32_t s_bc_last_ms = 0;

/* ── Scan / result tracking (raw-frame payload, no decoding) ────────────────────
   The MQTT message still carries the RAW frames of one scan (barcode 0x600-602 +
   whatever result frames arrive: 0x710 PASS/FAIL, 0x711 cells, 0x712 temps, 0x713
   fault). A scan begins on the barcode's first frame and publishes when the temps
   frame (0x712, last of a result) arrives — OR, if the jig results never come, after
   DAS_RESULT_TIMEOUT_MS, flagged incomplete (g_scan_incomplete). */
#define DAS_RESULT_TIMEOUT_MS  2500u
static bool     s_scan_active    = false;
static bool     s_scan_published = false;
static uint32_t s_scan_start_ms  = 0;

/* ── Per-scan raw-CAN session capture (fed to das_sync → /barcodes/<bc>_<stamp>.csv) ──
   Mirrors the LCD/EOL sd_logger: a session runs from one barcode scan to the next and
   captures EVERY received DAS frame (barcode + result burst + heartbeats) as CSV rows
   "Time,Dir,ID,DLC,Data,Decoded". Buffered in PSRAM on the CAN Rx task (single-task, no
   lock); the finished buffer is handed to das_sync which writes + uploads it. The session
   also finalizes DS_RAW_LINGER_MS after the result so the last scan uploads promptly. */
static char    *s_sess_buf   = NULL;    /* PSRAM working buffer, reused across sessions */
static uint32_t s_sess_cap   = 0;
static uint32_t s_sess_len   = 0;
static bool     s_sess_active = false;
static bool     s_sess_trunc = false;
static char     s_sess_bc[25] = {0};    /* barcode naming the current session          */
static bool     s_sess_result_done = false;
static uint32_t s_sess_result_ms   = 0;

/* Decoded result summary (published over MQTT via das_sync, for the server's analytics).
   Runs in parallel with the raw session above: raw frames go to the server over HTTP; the
   decoded PASS/FAIL + cells/temps/fault go over MQTT. */
static das_result_t s_res;
static bool         s_res_enq = false;  /* this scan's result already queued to das_sync */

static inline uint16_t le16u(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static inline int16_t  le16s(const uint8_t *p) { return (int16_t)(p[0] | ((uint16_t)p[1] << 8)); }

/* Hand the finished session buffer to das_sync (a right-sized PSRAM copy; ownership
   transfers). Reused working buffer is reset for the next session. */
static void raw_session_finalize(void)
{
    if (!s_sess_active) return;
    s_sess_active = false;

    if (s_sess_buf && s_sess_len > 0 && s_sess_bc[0]) {
        char *copy = (char *)heap_caps_malloc(s_sess_len, MALLOC_CAP_SPIRAM);
        if (!copy) copy = (char *)malloc(s_sess_len);
        if (copy) {
            memcpy(copy, s_sess_buf, s_sess_len);
            if (!das_sync_enqueue_raw(s_sess_bc, copy, s_sess_len, s_sess_trunc)) free(copy);
        }
    }
    s_sess_len = 0; s_sess_trunc = false; s_sess_bc[0] = '\0';
    s_sess_result_done = false;
}

/* Called on the barcode's first frame (0x600) — finalize the previous session, arm a new one. */
static void das_scan_begin(void)
{
    raw_session_finalize();                 /* close previous scan (all its frames) */

    s_scan_active    = true;
    s_scan_published = false;
    s_scan_start_ms  = millis();

    if (!s_sess_buf) {
        s_sess_buf = (char *)heap_caps_malloc(DS_RAW_SESSION_CAP, MALLOC_CAP_SPIRAM);
        if (!s_sess_buf) s_sess_buf = (char *)malloc(DS_RAW_SESSION_CAP);
        s_sess_cap = s_sess_buf ? DS_RAW_SESSION_CAP : 0;
    }
    s_sess_active = true;
    s_sess_len = 0; s_sess_trunc = false; s_sess_bc[0] = '\0';
    s_sess_result_done = false;

    memset(&s_res, 0, sizeof(s_res));
    s_res_enq = false;
}

/* Frame timestamp for a raw row: HH:MM:SS.mmm once the clock is set, else T+uptime_ms. */
static void raw_ts(char *out, size_t n)
{
    uint64_t ms = now_epoch_ms();
    if (ms) {
        time_t s = (time_t)(ms / 1000ull);
        struct tm t; gmtime_r(&s, &t);
        snprintf(out, n, "%02d:%02d:%02d.%03u", t.tm_hour, t.tm_min, t.tm_sec, (unsigned)(ms % 1000ull));
    } else {
        snprintf(out, n, "T+%lu", (unsigned long)millis());
    }
}

/* Append one received frame to the current session buffer as a CSV row (RX only). */
static void raw_session_append(const twai_message_t *m, uint8_t dlc)
{
    if (!s_sess_active || !s_sess_buf || s_sess_trunc) return;

    char ts[24]; raw_ts(ts, sizeof(ts));
    char row[96];
    int p = snprintf(row, sizeof(row), "%s,RX,0x%03lX,%u,",
                     ts, (unsigned long)m->identifier, (unsigned)dlc);
    for (uint8_t i = 0; i < dlc && p < (int)sizeof(row) - 4; i++)
        p += snprintf(row + p, sizeof(row) - p, "%s%02X", i ? " " : "", m->data[i]);
    if (p < (int)sizeof(row) - 2) row[p++] = ',';   /* empty Decoded column */
    row[p] = '\0';

    uint32_t rl = (uint32_t)strlen(row);
    if (s_sess_len + rl + 1 >= s_sess_cap) { s_sess_trunc = true; return; }
    memcpy(s_sess_buf + s_sess_len, row, rl);
    s_sess_len += rl;
    s_sess_buf[s_sess_len++] = '\n';
}

/* Mark that this scan's jig result has completed (drives the linger finalize). */
static void raw_mark_result_done(void)
{
    if (s_sess_active && !s_sess_result_done) {
        s_sess_result_done = true;
        s_sess_result_ms   = millis();
    }
}

/* Decode one multiplexed result frame (0x710/0x711/0x712/0x713) into s_res (for the MQTT
   result summary). 0x710 Seq0 byte1=Status; 0x711 cells uint16 LE; 0x712 temps int16 LE ×0.1°C;
   0x713 flat fault masks. dlc<=8 guaranteed. */
static void das_accumulate(const twai_message_t *m, uint8_t dlc)
{
    switch (m->identifier)
    {
        case DAS_STATUS_ID:
            if (dlc >= 2 && m->data[0] == 0) { s_res.status = m->data[1]; s_res.have_status = true; }
            break;
        case DAS_CELLS_ID: {
            uint8_t seq = m->data[0];
            if (seq <= 2) {
                s_res.cells_seq_mask |= (uint8_t)(1u << seq);
                uint8_t base = (uint8_t)(seq * 3);
                for (uint8_t k = 0; k < 3 && base + k < 8; k++) {
                    uint8_t off = (uint8_t)(1 + k * 2);
                    if (off + 1 < dlc) s_res.cell_mV[base + k] = le16u(&m->data[off]);
                }
            }
            break;
        }
        case DAS_TEMPS_ID: {
            uint8_t seq = m->data[0];
            if (seq <= 1) {
                s_res.temps_seq_mask |= (uint8_t)(1u << seq);
                uint8_t base = (uint8_t)(seq * 3);
                for (uint8_t k = 0; k < 3 && base + k < 4; k++) {
                    uint8_t off = (uint8_t)(1 + k * 2);
                    if (off + 1 < dlc) s_res.temp_c10[base + k] = le16s(&m->data[off]);
                }
            }
            break;
        }
        case DAS_FAULT_ID:
            s_res.have_fault    = true;
            s_res.cellFaultMask = (dlc >= 1) ? m->data[0] : 0;
            s_res.tempFaultMask = (dlc >= 2) ? (uint8_t)(m->data[1] & 0x0Fu) : 0;
            s_res.initFailed    = (dlc >= 4) ? m->data[3] : 0;
            break;
        default: break;
    }
}

/* Hand the decoded result to das_sync for MQTT publish. Enqueued once per scan. */
static void das_emit_result(bool incomplete)
{
    if (s_res_enq) return;
    s_res_enq = true;
    strncpy(s_res.barcode, s_bc, sizeof(s_res.barcode) - 1);
    s_res.barcode[sizeof(s_res.barcode) - 1] = '\0';
    s_res.epoch_ms   = now_epoch_ms();
    s_res.incomplete = incomplete;
    das_sync_enqueue_result(&s_res);
}

static void barcode_emit(void)
{
    if (s_bc_have == 0) return;

    /* Zero-padded ASCII: the code is the printable prefix up to the first non-print byte. */
    s_bc[BARCODE_MAXLEN] = '\0';
    size_t len = 0;
    while (len < BARCODE_MAXLEN && s_bc[len] >= 0x20 && s_bc[len] < 0x7F) len++;
    s_bc[len] = '\0';

    s_bc_have = 0;                     /* consume — ready for the next scan */
    if (len == 0) return;

    /* Name the current raw-CAN session by this barcode (file written at session end). */
    strncpy(s_sess_bc, s_bc, sizeof(s_sess_bc) - 1);
    s_sess_bc[sizeof(s_sess_bc) - 1] = '\0';

    char out[BARCODE_MAXLEN + 2];
    snprintf(out, sizeof(out), "%s\n", s_bc);   /* barcode + Enter */
    ble_hid_dev_type(out);                       /* queues to the HID task; non-blocking */
    ESP_LOGI(CAN_TAG, "Barcode -> tablet HID: \"%s\" (%u chars)", s_bc, (unsigned)len);
}

/* Ingest one barcode frame (0x600/0x601/0x602). Completes immediately on 0x602. */
static void barcode_ingest(const twai_message_t *m)
{
    uint8_t dlc = m->data_length_code; if (dlc > 8) dlc = 8;
    s_bc_last_ms = millis();

    switch (m->identifier) {
        case DAS_BARCODE0_ID:
            memset(s_bc, 0, sizeof(s_bc));
            memcpy(&s_bc[0], m->data, dlc);
            s_bc_have = 0x01;
            das_scan_begin();          /* new scan → arm the result timeout */
            break;
        case DAS_BARCODE1_ID:
            memcpy(&s_bc[8], m->data, dlc);
            s_bc_have |= 0x02;
            break;
        case DAS_BARCODE2_ID:
            memcpy(&s_bc[16], m->data, dlc);   /* bytes 16-23 (DBC: 16-19 meaningful) */
            s_bc_have |= 0x04;
            barcode_emit();                    /* final frame → emit immediately */
            break;
        default: break;
    }
}

/* Flush a barcode that started but whose final frame (0x602) never arrived in time. */
static void barcode_poll_timeout(void)
{
    if (s_bc_have && (uint32_t)(millis() - s_bc_last_ms) >= BARCODE_FRAME_GAP_MS)
        barcode_emit();
}

/* If a scan started (barcode seen) but the jig result frames never arrived within the
   window, publish the scan anyway — flagged incomplete (only barcode, no PASS/FAIL). Called
   from the CAN Rx task every iteration so it fires even when the bus goes quiet. */
void das_poll_result_timeout(void)
{
    if (!s_scan_active) return;
    uint32_t now = millis();

    /* Barcode-only scan (no jig results within the window) → arm the flush trigger flagged
       incomplete, publish an INCOMPLETE result summary, and mark the result "done" so the
       raw session can linger-finalize. */
    if (!s_scan_published && (uint32_t)(now - s_scan_start_ms) >= DAS_RESULT_TIMEOUT_MS)
    {
        s_scan_published  = true;
        g_scan_incomplete = true;
        g_scan_ready      = true;
        raw_mark_result_done();
        das_emit_result(true);
        ESP_LOGW(CAN_TAG, "Scan incomplete — no jig results within %u ms (barcode only)",
                 DAS_RESULT_TIMEOUT_MS);
    }

    /* Finalize + upload the raw-CAN session DS_RAW_LINGER_MS after the result completed, so
       the LAST scan of a batch uploads without waiting for a next barcode. (A next barcode
       finalizes the session sooner, via das_scan_begin.) Also a fallback for the MQTT result
       if the fault frame (0x713) never arrived to trigger the prompt publish. */
    if (s_sess_active && s_sess_result_done &&
        (uint32_t)(now - s_sess_result_ms) >= DS_RAW_LINGER_MS)
    {
        if (!s_res_enq)
        {
            bool anyResult = s_res.have_status || s_res.have_fault ||
                             s_res.cells_seq_mask || s_res.temps_seq_mask;
            das_emit_result(!anyResult);
        }
        raw_session_finalize();
    }
}

////////////////////////////////////

static inline void canmsgs_write(uint8_t idx, const twai_message_t *m)
{
    uint8_t dlc = m->data_length_code;
    if (dlc > 8) dlc = 8;

    portENTER_CRITICAL(&canmsgs_mux);
    CanMsgs[idx].msgId      = m->identifier;
    CanMsgs[idx].dataLength = dlc;
    memcpy(CanMsgs[idx].data, m->data, dlc);
    portEXIT_CRITICAL(&canmsgs_mux);
}

///////////////////////////////

static bool handle_rx_message(const twai_message_t *message)
{
    /* DAS harness frames are 11-bit standard IDs and never RTR. */
    if (message->extd || message->rtr)
    {
        return false;
    }

    if (!is_das_id(message->identifier))
    {
        return false;
    }

    uint8_t dlc = message->data_length_code;
    if (dlc > 8) dlc = 8;

    /* HIGH PRIORITY: a barcode frame is reassembled and forwarded to the tablet over
       BLE HID the instant it completes — before the ring/log/MQTT work below. */
    if (message->identifier == DAS_BARCODE0_ID ||
        message->identifier == DAS_BARCODE1_ID ||
        message->identifier == DAS_BARCODE2_ID)
    {
        barcode_ingest(message);
    }

    /* Capture EVERY received DAS frame (incl. the 0x720 heartbeat) into the per-scan raw-CAN
       session → /barcodes/<barcode>_<stamp>.csv via das_sync. RX-only, single-task, no lock.
       The session was (re)started by das_scan_begin() inside barcode_ingest() above on 0x600,
       so the barcode frame itself lands in its own new session. */
    raw_session_append(message, dlc);

    /* Decode the multiplexed result frames into s_res for the MQTT result summary. */
    if (message->identifier == DAS_STATUS_ID || message->identifier == DAS_CELLS_ID ||
        message->identifier == DAS_TEMPS_ID  || message->identifier == DAS_FAULT_ID)
    {
        das_accumulate(message, dlc);
    }

    /* Local latest-value snapshot for the serial/diagnostic dump. The DAS IDs map to
       distinct low indices via (id & 0xFF): 0x600→0x00, 0x601→0x01, 0x602→0x02,
       0x710→0x10, 0x711→0x11, 0x712→0x12, 0x720→0x20 — all < 200, no collisions. */
    canmsgs_write((uint8_t)(message->identifier & 0xFFu), message);

    /* Push the raw frame into the lossless ring for the durable NDJSON log + daily raw sync —
       EXCEPT the heartbeat (0x720), which is liveness-only and would flood it (it still goes
       to the event log via das_log_semantic below, and to the per-scan raw session above). */
    if (message->identifier != DAS_HEARTBEAT_ID)
    {
        CanFrame_t frame;
        frame.epoch_ms = now_epoch_ms();
        frame.id       = message->identifier;
        frame.dlc      = dlc;
        memset(frame.data, 0, sizeof(frame.data));
        memcpy(frame.data, message->data, dlc);
        fb_push(&frame);

        /* temps (0x712) is the last frame of a result cycle → arm the SD-flush trigger. */
        if (message->identifier == DAS_TEMPS_ID && s_scan_active && !s_scan_published)
        {
            s_scan_published  = true;
            g_scan_incomplete = false;
            g_scan_ready      = true;
        }

        /* temps (0x712) / fault (0x713) mark the result complete → start the raw-session
           linger so the LAST scan of a batch finalizes + uploads without waiting for a next scan. */
        if (message->identifier == DAS_TEMPS_ID || message->identifier == DAS_FAULT_ID)
        {
            raw_mark_result_done();
        }

        /* fault (0x713) is the documented LAST frame of the burst → publish the decoded
           result summary over MQTT (once per scan). */
        if (message->identifier == DAS_FAULT_ID && s_scan_active)
        {
            das_emit_result(false);
        }
    }

    /* Surface harness-fault (0x713) and heartbeat (0x720) detail in the event log.
       Rate-limited internally so this never blocks the CAN Rx loop per frame. */
    das_log_semantic(message);

    return true;
}

bool can_receive(void)
{
    if (!driver_installed)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        return false;
    }

    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, 0);

    if (!(alerts_triggered & TWAI_ALERT_RX_DATA))
    {
        return false;
    }

    bool target_received = false;
    twai_message_t message;

    while (twai_receive(&message, 0u) == ESP_OK)
    {
        /* CAN self-update ENTER: a dedicated standard frame on ID 0x0C carrying
         * the 0x69/0x96 pattern. This is DISTINCT from the BMS bootloader
         * trigger (ID 0x0B) on purpose — the ESP32 only enters its OWN
         * bootloader when explicitly asked, so flashing the BMS never drags us
         * in. Acted on after the drain (the backup+reboot must not run mid-loop). */
        if (!message.extd && message.identifier == OTA_BL_ENTER_CAN_ID &&
            ota_bl_is_can_enter(message.data, message.data_length_code))
        {
            s_can_enter_req = true;
            continue;   /* not a telemetry frame — skip normal handling */
        }

        /* Restart command from the LCD: ID 0x2D0 carrying 'R''S'. The operator
         * tapped "Restart system" on the LCD — reboot immediately so the IoT and
         * LCD come back together. Distinct from the bootloader-enter (0x0C): this
         * is a plain reboot into the running app, not a flash. */
        if (!message.extd && message.identifier == IOT_RESTART_CMD_ID &&
            message.data_length_code >= 2 &&
            message.data[0] == IOT_RESTART_MAGIC0 && message.data[1] == IOT_RESTART_MAGIC1)
        {
            ESP_LOGW(CAN_TAG, "Restart command (0x2D0 'RS') from LCD — rebooting");
            EVLOG_WARN("SYS", "restart command from LCD (0x2D0) — rebooting");
            Serial.flush();
            vTaskDelay(pdMS_TO_TICKS(50));   /* let the log line flush */
            esp_restart();                   /* does not return */
        }

        /* Runtime CAN baud change (IoT): ID 0x2E0, data[0]=magic, data[1]=selector.
         * ACK at the current baud first, then persist to NVS (all partitions
         * follow) and re-tune TWAI live. Distinct ID/data from the LCD (0x2CC). */
        if (!message.extd && message.identifier == IOT_BAUD_CFG_ID &&
            message.data_length_code >= 2 && message.data[0] == IOT_BAUD_MAGIC)
        {
            uint32_t new_baud = 0;
            switch (message.data[1]) {
                case IOT_BAUD_SEL_125: new_baud = 125; break;
                case IOT_BAUD_SEL_250: new_baud = 250; break;
                case IOT_BAUD_SEL_500: new_baud = 500; break;
            }
            if (new_baud) {
                ESP_LOGI(CAN_TAG, "Baud command: %u kbps", (unsigned)new_baud);
                can_baud_send_ack(message.data[1]);      /* ACK at the current baud */
                bool changed = (new_baud != s_can_baud_kbps);
                apply_baud_change(new_baud);             /* persist (+ reinstall if changed) */
                if (changed) break;                      /* driver reinstalled — restart drain */
            }
            continue;
        }

        if (handle_rx_message(&message))
        {
            target_received = true;
            canReceivedTime = xTaskGetTickCount();
        }
    }

    /* Flush a partial barcode (frames 1-2 seen but 0x602 never arrived) to the tablet. */
    barcode_poll_timeout();

    /* Enter the CAN self-update bootloader if requested. This backs up
     * app0→app1, flags CAN mode, sets factory as next boot, and restarts —
     * it does not return on success. */
    if (s_can_enter_req)
    {
        s_can_enter_req = false;
        ESP_LOGW(CAN_TAG, "CAN self-update enter (0x0C) received — rebooting into factory bootloader");
        Serial.println("[CAN] CAN self-update enter received — entering bootloader");
        EVLOG_WARN("OTA", "CAN self-update enter (0x0C) — rebooting into factory bootloader");
        ota_bootloader_trigger_reboot_can();
    }

    return target_received;
}


void send_to_serial(void)
{
  char convertHexBuffer[20];

  for(uint8_t i = 0; i < 165; i++)
  {
    if (CanMsgs[i].msgId != 0)
    { 
      uint8_t start[] = { 0xFF, 0x03, 0xFF, 0x00 };
      uint8_t end[] = { 0xFF, 0xAA, 0x03 };
      uint8_t msgID[] = { 
                          (CanMsgs[i].msgId >> 24) & 0xFF, 
                          (CanMsgs[i].msgId >> 16) & 0xFF, 
                          (CanMsgs[i].msgId >> 8)  & 0xFF, 
                          CanMsgs[i].msgId & 0xFF 
                        }; 

      uart_write_bytes(SERIAL_UART, (const char*)start, sizeof(start));
      uart_write_bytes(SERIAL_UART, (const char*)msgID, sizeof(msgID));
      uart_write_bytes(SERIAL_UART, (const char*)&CanMsgs[i].dataLength, 1);
      uart_write_bytes(SERIAL_UART, (const char*)CanMsgs[i].data, 8);  // ← Now contains modified voltage!
      uart_write_bytes(SERIAL_UART, (const char*)end, sizeof(end));
    }
  }
}

/* The desktop-app serial protocol used to build + transmit a legacy BMS config frame.
   The harness IOT board is a passive bus listener and must NOT transmit onto the harness
   bus, so this now just drains UART0 and discards it. */
void uart_serial_event(void)
{
  if (xQueueReceive(uartSerialQueue, (void *)&event, pdMS_TO_TICKS(5000)))
  {
    if (event.type == UART_DATA && event.size > 0)
    {
      uart_read_bytes(UART_NUM_0, uartSerialbuffer, event.size, pdMS_TO_TICKS(1000));
      /* input intentionally discarded — no CAN TX on the harness bus */
    }
  }
}

/* Formerly transmitted a time-sync frame to the BMS. On the harness bus we do not
   transmit, so this only advances the software RTC (used for SD-log / event-log
   timestamps). Kept under the original name/signature so timestamp_sync_task and
   the shutdown handshake are unchanged. */
void bms_timestamp_sync(void)
{
  if (!ntpTimeReceived && !gpsTimeReceived)
  {
    return;
  }
  rtc_update_timestamp();
}


void check_can_status(void)
{
  static int8_t prevOk = -1;   /* -1 = not yet evaluated; log only on transitions */

  TickType_t currentTime = xTaskGetTickCount();
  bool ok = ((currentTime - canReceivedTime) <= pdMS_TO_TICKS(10000));
  canStatus = ok;
  if (ok) ESP_LOGI(CAN_TAG, "Can OK!");
  else    ESP_LOGI(CAN_TAG, "Can Not OK!");

  if ((int8_t)ok != prevOk)
  {
    if (ok) EVLOG_OK("CAN",   "bus active (master frames flowing)");
    else    EVLOG_FAIL("CAN", "bus lost (no master frames >10s)");
    prevOk = (int8_t)ok;
  }
}

/* IoT → LCD status heartbeat (see IOT_HEARTBEAT_ID in main.h). TXed every 1 s so the LCD
   knows the IoT is powered/on-bus and whether a tablet is connected over BLE HID. This is
   the ONE frame the harness IoT transmits — a low-priority (0x7A0) status beacon; it does
   not touch the DAS result frames. */
/* Last heartbeat TX outcome + flags — surfaced by the diagnostic serial print below. */
static esp_err_t s_hb_tx_result = ESP_FAIL;
static uint8_t   s_hb_flags     = 0;

static void send_iot_heartbeat_can(uint8_t counter)
{
    uint8_t flags = 0;
    if (ble_hid_dev_state() == HIDDEV_CONNECTED) flags |= IOT_HB_FLAG_TABLET;
    if (das_sync_server_up())                     flags |= IOT_HB_FLAG_SERVER;   /* local Data Server link */
    if (canStatus)                                flags |= IOT_HB_FLAG_CAN;

    twai_message_t m = {};
    m.identifier       = IOT_HEARTBEAT_ID;
    m.extd             = 0;
    m.rtr              = 0;
    m.data_length_code = 4;
    m.data[0] = IOT_HEARTBEAT_MAGIC;
    m.data[1] = flags;
    m.data[2] = counter;
    m.data[3] = 0;
    s_hb_tx_result = twai_transmit(&m, pdMS_TO_TICKS(10));
    s_hb_flags     = flags;
}

/* Emits the IoT status heartbeat on CAN every 1 s, and logs local status ~every 45 s. */
void diagnostic_can_task(void *parameter) {
    xEventGroupWaitBits(initTaskEventGroup, INIT_TASK_COMPLETE_BIT, pdTRUE, pdTRUE, portMAX_DELAY);

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(1000);   /* 1 s heartbeat cadence */
    uint8_t  counter = 0;
    uint16_t logDiv  = 0;

    while (true) {
        vTaskDelayUntil(&lastWakeTime, frequency);

        send_iot_heartbeat_can(counter++);

        if (++logDiv >= 5) {   /* ~5 s status log — decodes the LCD's IoT/Tablet/Uplink chips */
            logDiv = 0;
            const esp_partition_t *running = esp_ota_get_running_partition();
            uint8_t partId = 2;  // default: factory
            if (running) {
                if      (running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0) partId = 0;
                else if (running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1) partId = 1;
            }
            /* The three flags the LCD turns into chips, plus WHY the frame may not arrive:
               driver_installed + the last 0x7A0 TX result. If TX != OK (e.g. NO_ACK because
               the CAN bridge/LCD isn't on the bus), the LCD sees NO heartbeat → IoT chip red. */
            Serial.printf("[DIAG] IoT:%s Part:%u | 0x7A0 tx=%s flags=0x%02X "
                          "=> Tablet:%u Uplink(server):%u CAN:%u | canDrv=%u canStatus=%u\n",
                          IOT_VERSION, partId,
                          esp_err_to_name(s_hb_tx_result), s_hb_flags,
                          (unsigned)!!(s_hb_flags & IOT_HB_FLAG_TABLET),
                          (unsigned)!!(s_hb_flags & IOT_HB_FLAG_SERVER),
                          (unsigned)!!(s_hb_flags & IOT_HB_FLAG_CAN),
                          (unsigned)driver_installed, (unsigned)canStatus);
        }
    }
}