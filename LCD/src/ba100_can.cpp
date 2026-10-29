/**
 * ba100_can.cpp  --  Harness Tester CAN layer (via UART bridge)
 *
 * CAN is NOT on this board. The LCD reaches the bus through an ESP32-WROOM CAN
 * bridge (../CAN_Bridge) over a UART link — this layer's transport is that UART
 * (bridge_send/bridge_recv, a 16-byte framed protocol), but ALL the DAS harness
 * parsing stays here unchanged:
 *   • ba100_can_init()  — bring up the UART link to the bridge
 *   • ba100_can_poll()  — drain relayed frames; DAS harness protocol + the
 *                         0x18A10001 BMS time-sync; track link liveness
 *   • ba100_get_state() — link-alive snapshot for the UI status dot
 *   • ba100_send_barcode / ba100_send_iot_restart — TX via the bridge
 *
 * twai_message_t is reused only as the in-memory message struct; the TWAI driver
 * is never installed here. Runtime baud change (old 0x2CC) is gone — the CAN
 * baud lives in the bridge firmware. CAN-flash over the bridge is not wired up
 * (flash the LCD over USB).
 */

#include "ba100_can.h"
#include "can_bootloader.h"    /* BTL_ENTER_BL_ID, can_bootloader_enter_notify */
#include "sd_logger.h"         /* sd_logger_frame() — raw per-scan CAN capture */
#include "rawlog.h"            /* rawlog_push() — on-screen live raw-CAN view  */
#include "hvac_can.h"          /* compressor + charger protocol (over bridge)  */
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/twai.h"
#include <Preferences.h>
#include <string.h>
#include <time.h>              /* struct tm, mktime */
#include <sys/time.h>          /* settimeofday      */

/* BMS time-sync frame (extended 29-bit): data[0]=cmd, then raw sec/min/hour/0/
 * day/month/year(+2000). Sets the ESP wall clock (no NTP on this device). */
#define BMS_TIME_SYNC_ID   0x18A10001UL
#define BMS_TIME_SYNC_CMD  0x2BU
#include <time.h>

/* ── UART link to the CAN bridge (../CAN_Bridge) ─────────────────────────────
 * CAN is off-board: the LCD talks a fixed 16-byte framed protocol over UART to
 * an ESP32-WROOM bridge that owns the TWAI + transceiver and relays frames both
 * ways. twai_message_t is used ONLY as the in-memory message struct here — the
 * TWAI driver is never installed on the LCD. Same framing on both ends:
 *   [0]=0xAA sync  [1]=flags(bit0=ext)  [2..5]=id LE32  [6]=dlc
 *   [7..14]=data[8]  [15]=XOR checksum of bytes [1..14]. */
#define BRIDGE_UART        Serial1
#define BRIDGE_TX_PIN      CONFIG_CAN_TX_PIN    /* GPIO43 → bridge RX */
#define BRIDGE_RX_PIN      CONFIG_CAN_RX_PIN    /* GPIO44 ← bridge TX */
#define BRIDGE_FRAME_LEN   16
#define BRIDGE_FRAME_SYNC  0xAAU
#define BRIDGE_FLAG_EXT    0x01U   /* byte[1] bit0: extended CAN id            */
#define BRIDGE_FLAG_CTRL   0x02U   /* byte[1] bit1: bridge-local control frame  */
                                   /*   (heartbeat / command) — NOT put on CAN   */
#define BRIDGE_CTRL_REBOOT 0x01U   /* control data[0]: reboot the bridge board    */
#define BRIDGE_LINK_HB_MS  500UL   /* how often we tell the bridge "LCD alive"  */
#define BRIDGE_STALE_MS    2000UL  /* no valid frame from the bridge this long → link down */

/* millis() of the last valid frame FROM the bridge (CAN or its heartbeat) — any
 * such frame means the UART link + bridge are alive. Drives the top-bar chip. */
static uint32_t s_bridge_last_ms = 0;

/* Build + write one 16-byte frame. */
static void bridge_write(uint8_t flags, uint32_t id, uint8_t dlc, const uint8_t *data)
{
    uint8_t f[BRIDGE_FRAME_LEN];
    f[0] = BRIDGE_FRAME_SYNC;
    f[1] = flags;
    f[2] = (uint8_t)id;         f[3] = (uint8_t)(id >> 8);
    f[4] = (uint8_t)(id >> 16); f[5] = (uint8_t)(id >> 24);
    if (dlc > 8) dlc = 8;
    f[6] = dlc;
    for (int i = 0; i < 8; i++) f[7 + i] = (data && i < dlc) ? data[i] : 0;
    uint8_t cs = 0; for (int i = 1; i < 15; i++) cs ^= f[i];
    f[15] = cs;
    BRIDGE_UART.write(f, BRIDGE_FRAME_LEN);
}

/* Serialize one CAN message and push it to the bridge. */
static void bridge_send(const twai_message_t *m)
{
    bridge_write(m->extd ? BRIDGE_FLAG_EXT : 0, m->identifier, m->data_length_code, m->data);
    sd_logger_frame(true, m->identifier, m->data, m->data_length_code, nullptr);   /* raw TX → SD */
    rawlog_push(true, m->identifier, m->extd, m->data, m->data_length_code);       /* raw TX → LCD */
}

/* Link heartbeat: a control frame the bridge counts as "LCD connected" and does
 * NOT forward to CAN. Sent periodically so the bridge's link LED can show us up. */
static void bridge_send_link_heartbeat(void)
{
    bridge_write(BRIDGE_FLAG_CTRL, 0, 0, nullptr);
}

/* Tell the bridge board to reboot (a control frame, not a CAN frame). Blocks
 * briefly so it reaches the bridge before the caller reboots the LCD. */
void ba100_send_bridge_reboot(void)
{
    uint8_t d = BRIDGE_CTRL_REBOOT;
    bridge_write(BRIDGE_FLAG_CTRL, 0, 1, &d);
    bridge_write(BRIDGE_FLAG_CTRL, 0, 1, &d);   /* twice for robustness */
    BRIDGE_UART.flush();
    Serial.println("[BRIDGE] TX reboot-bridge (ctrl)");
}

/* Non-blocking: assemble bytes from the bridge into *m. Returns true once a
 * complete, checksum-valid CAN frame is ready. Control frames are skipped. */
static bool bridge_recv(twai_message_t *m)
{
    static uint8_t buf[BRIDGE_FRAME_LEN];
    static int     len = 0;
    while (BRIDGE_UART.available()) {
        uint8_t b = (uint8_t)BRIDGE_UART.read();
        if (len == 0 && b != BRIDGE_FRAME_SYNC) continue;   /* wait for sync byte */
        buf[len++] = b;
        if (len < BRIDGE_FRAME_LEN) continue;
        len = 0;
        uint8_t cs = 0; for (int i = 1; i < 15; i++) cs ^= buf[i];
        if (cs != buf[15]) continue;                        /* corrupt → resync */
        s_bridge_last_ms = millis();                        /* any valid frame → bridge alive */
        if (buf[1] & BRIDGE_FLAG_CTRL) continue;            /* control frame → not CAN */
        memset(m, 0, sizeof(*m));
        m->extd             = (buf[1] & BRIDGE_FLAG_EXT) ? 1 : 0;
        m->identifier       = (uint32_t)buf[2] | ((uint32_t)buf[3] << 8) |
                              ((uint32_t)buf[4] << 16) | ((uint32_t)buf[5] << 24);
        m->data_length_code = buf[6] > 8 ? 8 : buf[6];
        for (int i = 0; i < 8; i++) m->data[i] = buf[7 + i];
        return true;
    }
    return false;
}

/* ── Module state ────────────────────────────────────────────────────────── */
static BA100_State_t  s_state       = {};
static volatile bool  s_btl_active  = false;
static uint32_t       s_last_rx_ms  = 0;      /* millis() of last received frame */
static uint32_t       s_rx_frames   = 0;

/* ── DAS harness data (all access is on the loop task: poll writes,
 *    ba100_get_state / ba100_take_das_result read — no cross-task sync) ─── */
static uint16_t       s_cell_mv[DAS_NUM_CELLS]  = {0};
static int16_t        s_temp_raw[DAS_NUM_TEMPS] = {0};   /* SIGNED (0x712 @1-) */
static uint8_t        s_das_status  = 0;
static char           s_das_echo[DAS_BARCODE_MAXLEN + 1] = {0};  /* echoed barcode (0x710) */
static int            s_echo_cov    = 0;        /* contiguous echo bytes assembled so far */
static bool           s_finalized   = false;    /* this reply already produced a result */
static bool           s_result_new  = false;    /* a complete reply is ready to consume */

/* Harness fault (0x713) + heartbeat diagnostics (0x720). */
static bool           s_fault_valid = false;
static uint8_t        s_fault_cell_mask = 0;
static uint8_t        s_fault_temp_mask = 0;
static bool           s_fault_init  = false;
static bool           s_fault_ext_valid = false;   /* bytes 4-6 present (DLC ≥ 7) */
static uint8_t        s_fault_cell_raw_mask   = 0; /* byte4 CellRawFaultMask */
static uint8_t        s_fault_temp_repeat_mask = 0;/* byte5 TempRepeatabilityFaultMask */
static uint8_t        s_fault_passing_count   = 0; /* byte6 PassingReadingCount (0-10) */
static bool           s_hb_valid    = false;
static uint16_t       s_hb_counter  = 0;
static uint8_t        s_hb_tec = 0, s_hb_rec = 0, s_hb_lec = 0, s_hb_qd = 0;

/* IoT board heartbeat (0x7A0). s_iot_last_ms updates only when the counter
 * advances, so a frozen counter goes stale (stall detection). */
static uint32_t       s_iot_last_ms = 0;
static bool           s_iot_seen    = false;
static uint8_t        s_iot_counter = 0;
static uint8_t        s_iot_flags   = 0;

static inline uint16_t le16(const uint8_t *d) { return (uint16_t)(d[0] | (d[1] << 8)); }

/* ── Baud rate (default CONFIG_CAN_BAUD_KBPS, runtime-overridable, NVS-backed) */
static Preferences    s_prefs;
static uint32_t       s_baud_kbps   = CONFIG_CAN_BAUD_KBPS;

/* ════════════════════════════════════════════════════════════════════════
 *  Wall-clock  (from the ESP system clock, set by NTP once WiFi associates;
 *  falls back to an uptime tag until then)
 * ════════════════════════════════════════════════════════════════════════ */
bool ba100_get_datetime(int *Y, int *Mo, int *D, int *h, int *m, int *s)
{
    time_t now = time(nullptr);
    if (now < 1700000000) return false;   /* clock not yet set by NTP (before ~2023-11) */
    struct tm t;
    localtime_r(&now, &t);
    if (Y)  *Y  = t.tm_year + 1900;
    if (Mo) *Mo = t.tm_mon + 1;
    if (D)  *D  = t.tm_mday;
    if (h)  *h  = t.tm_hour;
    if (m)  *m  = t.tm_min;
    if (s)  *s  = t.tm_sec;
    return true;
}

void ba100_timestamp(char *buf, unsigned buflen)
{
    int Y, Mo, D, h, m, s;
    if (ba100_get_datetime(&Y, &Mo, &D, &h, &m, &s)) {
        snprintf(buf, buflen, "%04d-%02d-%02d %02d:%02d:%02d", Y, Mo, D, h, m, s);
        return;
    }
    uint32_t up = millis() / 1000UL;
    snprintf(buf, buflen, "T+%02lu:%02lu:%02lu",
             (unsigned long)(up / 3600),
             (unsigned long)((up % 3600) / 60),
             (unsigned long)(up % 60));
}

/* Same as ba100_timestamp() but with a millisecond fraction (for scan/response
 * latency). Wall time uses gettimeofday's sub-second; uptime uses millis(). */
void ba100_timestamp_ms(char *buf, unsigned buflen)
{
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    if (tv.tv_sec >= 1700000000) {
        struct tm t; localtime_r(&tv.tv_sec, &t);
        snprintf(buf, buflen, "%04d-%02d-%02d %02d:%02d:%02d.%03ld",
                 t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                 t.tm_hour, t.tm_min, t.tm_sec, (long)(tv.tv_usec / 1000));
    } else {
        uint32_t up = millis();
        snprintf(buf, buflen, "T+%02lu:%02lu:%02lu.%03lu",
                 (unsigned long)(up / 3600000UL),
                 (unsigned long)((up / 60000UL) % 60UL),
                 (unsigned long)((up / 1000UL) % 60UL),
                 (unsigned long)(up % 1000UL));
    }
}

/* Apply a BMS time-sync frame's 8 raw bytes to the ESP system clock. No TZ is
 * set, so local == the wall time the master sent (round-trips consistently). */
static bool s_time_synced = false;
static void apply_time_sync(const uint8_t *d)
{
    if (d[6] < 1 || d[6] > 12 || d[5] < 1 || d[5] > 31 ||
        d[3] > 23 || d[2] > 59 || d[1] > 59) return;      /* reject garbage */
    struct tm t = {};
    t.tm_sec  = d[1];
    t.tm_min  = d[2];
    t.tm_hour = d[3];
    t.tm_mday = d[5];
    t.tm_mon  = d[6] - 1;                 /* 1-12 → 0-11 */
    t.tm_year = 2000 + d[7] - 1900;       /* 2-digit year → tm_year */
    t.tm_isdst = 0;
    time_t epoch = mktime(&t);
    if (epoch < 1700000000) return;       /* sanity: after ~2023-11 */
    struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
    settimeofday(&tv, nullptr);
    if (!s_time_synced) {
        s_time_synced = true;
        Serial.printf("[CAN] clock set from BMS: %04d-%02d-%02d %02d:%02d:%02d\n",
                      t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
    }
}

/* The CAN bus baud is a property of the BRIDGE (fixed at its compile time). We
 * only report it for the LCD watermark; runtime baud change (old 0x2CC) is gone. */
uint32_t ba100_get_baud_kbps(void) { return s_baud_kbps; }

/* ════════════════════════════════════════════════════════════════════════
 *  Init  —  bring up the UART link to the CAN bridge
 * ════════════════════════════════════════════════════════════════════════ */
bool ba100_can_init(void)
{
    /* NVS baud value kept only for the watermark display (namespace "ba100"
     * shared with the factory web UI). Default = CONFIG_CAN_BAUD_KBPS. */
    s_prefs.begin("ba100", false);
    uint32_t stored = s_prefs.getUInt("baud", CONFIG_CAN_BAUD_KBPS);
    if (stored != 125 && stored != 250 && stored != 500) stored = CONFIG_CAN_BAUD_KBPS;
    s_baud_kbps = stored;

    BRIDGE_UART.setRxBufferSize(1024);   /* headroom for a burst of relayed frames */
    BRIDGE_UART.begin(CONFIG_BRIDGE_UART_BAUD, SERIAL_8N1, BRIDGE_RX_PIN, BRIDGE_TX_PIN);
    Serial.printf("[BRIDGE] UART @ %lu baud, TX=%d RX=%d (CAN via ../CAN_Bridge @ %lu kbps)\n",
                  (unsigned long)CONFIG_BRIDGE_UART_BAUD, BRIDGE_TX_PIN, BRIDGE_RX_PIN,
                  (unsigned long)s_baud_kbps);
    hvac_can_init();   /* compressor + charger protocol state */
    return true;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Bootloader handoff
 * ════════════════════════════════════════════════════════════════════════ */
void ba100_can_set_btl_active(void)   { s_btl_active = true; }
void ba100_can_clear_btl_active(void) { s_last_rx_ms = millis(); s_btl_active = false; }

/* No charger on the harness tester — nothing to switch off. */
void ba100_charger_off(void) {}

/* ════════════════════════════════════════════════════════════════════════
 *  Poll  —  call from main loop (non-blocking)
 * ════════════════════════════════════════════════════════════════════════ */
void ba100_can_poll(void)
{
    if (s_btl_active) return;               /* bootloader owns the bus */

    /* Tell the bridge we're alive (~2 Hz) so its link LED can show "connected". */
    static uint32_t s_link_hb_ms = 0;
    uint32_t now_hb = millis();
    if (now_hb - s_link_hb_ms >= BRIDGE_LINK_HB_MS) {
        s_link_hb_ms = now_hb;
        bridge_send_link_heartbeat();
    }

    twai_message_t msg;

    /* Drain CAN frames relayed by the bridge over UART (each bridge_recv() call
     * returns one complete frame, exactly like twai_receive() used to). */
    while (bridge_recv(&msg)) {
        s_rx_frames++;
        s_last_rx_ms = millis();
        sd_logger_frame(false, msg.identifier, msg.data, msg.data_length_code, nullptr);  /* raw RX → SD */
        rawlog_push(false, msg.identifier, msg.extd, msg.data, msg.data_length_code);     /* raw RX → LCD */
        hvac_can_rx(msg.identifier, msg.extd, msg.data, msg.data_length_code);            /* compressor/charger parse */

        if (!msg.extd) {
            /* Desktop-flasher "enter bootloader" trigger for the LCD (ID 0x0D,
             * DLC 8, data 69 96 …). Note: CAN-flash over the bridge is not wired
             * up — this only fires if such a frame is relayed. */
            if (msg.identifier == BTL_ENTER_BL_ID) {
                can_bootloader_enter_notify(msg.data, msg.data_length_code);
            }
            /* ── DAS harness protocol (das_protocol.dbc) ──────────────────── */
            /* 0x710 Status_Barcode: byte0 = Sequence. seq0 → byte1 = Status and
             * bytes 2-7 = barcode 0-5 (6 bytes); every later seq N carries 7
             * barcode bytes (bytes 1-7) at offset 6 + (N-1)*7. So the echo grows
             * past 16 bytes with more sequences. The barcode is zero-padded, so
             * the reply is complete once a 0 appears in the assembled prefix (or
             * the 24-byte capacity is reached). */
            else if (msg.identifier == DAS_STATUS_ID && msg.data_length_code >= 1) {
                int dlc = msg.data_length_code;
                uint8_t seq = msg.data[0];
                if (seq == 0) {
                    if (dlc >= 2) s_das_status = msg.data[1];
                    memset(s_das_echo, 0, sizeof(s_das_echo));
                    s_echo_cov = 0; s_finalized = false;
                    int cnt = 0;
                    for (int i = 0; i < 6 && (2 + i) < dlc && cnt < DAS_ECHO_MAXLEN; i++)
                        s_das_echo[cnt++] = (char)msg.data[2 + i];
                    s_echo_cov = cnt;
                } else {
                    int base = 6 + ((int)seq - 1) * 7;
                    for (int i = 0; i < 7 && (1 + i) < dlc && (base + i) < DAS_ECHO_MAXLEN; i++)
                        s_das_echo[base + i] = (char)msg.data[1 + i];
                    int end = base + ((dlc - 1) < 7 ? (dlc - 1) : 7);
                    if (end > DAS_ECHO_MAXLEN) end = DAS_ECHO_MAXLEN;
                    if (end > s_echo_cov) s_echo_cov = end;
                }
                /* Complete when the terminator (zero) is in view OR the echo has
                 * reached its 0x710 capacity (20 bytes) — a link ≥20 chars has no
                 * zero, so without the length cap the reply never finalized and the
                 * UI stuck on "Checking…". */
                if (!s_finalized && s_echo_cov > 0) {
                    bool done = (s_echo_cov >= DAS_ECHO_MAXLEN);
                    for (int i = 0; i < s_echo_cov; i++) if (s_das_echo[i] == 0) { done = true; break; }
                    if (done) { s_result_new = true; s_finalized = true; }
                }
            }
            /* 0x711 Cell_Values: byte0=Seq; 3×LE16 cell values by sequence. */
            else if (msg.identifier == DAS_CELLS_ID && msg.data_length_code >= 7) {
                uint8_t seq = msg.data[0];
                uint16_t v0 = le16(&msg.data[1]), v1 = le16(&msg.data[3]), v2 = le16(&msg.data[5]);
                if (seq == 0) { s_cell_mv[0] = v0; s_cell_mv[1] = v1; s_cell_mv[2] = v2; }
                else if (seq == 1) { s_cell_mv[3] = v0; s_cell_mv[4] = v1; s_cell_mv[5] = v2; }
                else if (seq == 2) { s_cell_mv[6] = v0; s_cell_mv[7] = v1; }
            }
            /* 0x712 Temperature_Values: byte0=Seq; 3×LE16 temps by sequence.
             * SIGNED (0x712 @1-, 0.1 degC): cast through int16_t so sub-zero
             * temps decode correctly. */
            else if (msg.identifier == DAS_TEMPS_ID && msg.data_length_code >= 7) {
                uint8_t seq = msg.data[0];
                int16_t v0 = (int16_t)le16(&msg.data[1]),
                        v1 = (int16_t)le16(&msg.data[3]),
                        v2 = (int16_t)le16(&msg.data[5]);
                if (seq == 0) { s_temp_raw[0] = v0; s_temp_raw[1] = v1; s_temp_raw[2] = v2; }
                else if (seq == 1) { s_temp_raw[3] = v0; }
            }
            /* 0x713 Jig_HarnessFault (v5, 6 signals): byte0=CellFaultMask
             * (bit n → cell n AVG reads 0V or ≥4V), byte1=TempFaultMask (bits 0-3
             * → temp n AVG deviates), byte2=reserved, byte3=InitFailed,
             * byte4=CellRawFaultMask (bit n → cell n hit OV on ≥1 raw sample),
             * byte5=TempRepeatabilityFaultMask (bits 0-3 → temp n unstable across
             * raw samples), byte6=PassingReadingCount (0-10). ⚠ When InitFailed=1,
             * CellFaultMask reads 0xFF spuriously. Bytes 4-6 come from the RAW
             * per-sample checks that drive 0x710 Status, so they explain a FAIL
             * when bytes 0/1 are clean. */
            else if (msg.identifier == DAS_FAULT_ID && msg.data_length_code >= 4) {
                s_fault_valid     = true;
                s_fault_cell_mask = msg.data[0];
                s_fault_temp_mask = msg.data[1];
                s_fault_init      = msg.data[3] != 0;
                /* Extended raw-sample detail (bytes 4-6) — only when the jig sent
                 * the full v5 frame (DLC ≥ 7); older DLC-4 frames leave it invalid. */
                if (msg.data_length_code >= 7) {
                    s_fault_ext_valid        = true;
                    s_fault_cell_raw_mask    = msg.data[4];
                    s_fault_temp_repeat_mask = msg.data[5] & 0x0F;
                    s_fault_passing_count    = msg.data[6];
                }
            }
            /* 0x720 Heartbeat: liveness (counted above) + CAN diagnostics.
             * bytes: 'H' 'B' Counter(LE16) TEC REC LEC TxQueueDepth. */
            else if (msg.identifier == DAS_HEARTBEAT_ID && msg.data_length_code >= 8) {
                s_hb_valid   = true;
                s_hb_counter = le16(&msg.data[2]);
                s_hb_tec = msg.data[4];
                s_hb_rec = msg.data[5];
                s_hb_lec = msg.data[6];
                s_hb_qd  = msg.data[7];
            }
            /* 0x7A0 IoT status heartbeat: flags + rolling counter. */
            else if (msg.identifier == IOT_HEARTBEAT_ID && msg.data_length_code >= 4 &&
                     msg.data[0] == IOT_HEARTBEAT_MAGIC) {
                s_iot_flags = msg.data[1];
                uint8_t c   = msg.data[2];
                if (!s_iot_seen || c != s_iot_counter) {   /* advancing → alive */
                    s_iot_last_ms = millis();
                    s_iot_counter = c;
                    s_iot_seen    = true;
                }
            }
        }
        else {
            /* ── BMS time sync (extended 0x18A10001, data[0]=0x2B) ────────────
             * Raw bytes: [1]=sec [2]=min [3]=hour [4]=0 [5]=day [6]=month
             * [7]=year(+2000). Sets the ESP system clock so log timestamps
             * become real wall-clock (there is no NTP on this device). */
            if (msg.identifier == BMS_TIME_SYNC_ID && msg.data_length_code >= 8 &&
                msg.data[0] == BMS_TIME_SYNC_CMD) {
                apply_time_sync(msg.data);
            }
        }
    }

    /* Pace the compressor/charger command + detection TX (through the bridge). */
    hvac_can_tick();
}

/* ════════════════════════════════════════════════════════════════════════
 *  DAS protocol TX / status
 * ════════════════════════════════════════════════════════════════════════ */
void ba100_send_barcode(const char *code)
{
    if (!code) return;
    size_t len = strlen(code);
    if (len > DAS_BARCODE_MAXLEN) len = DAS_BARCODE_MAXLEN;

    /* 24 bytes across three 8-byte frames, zero-padded past the code length. */
    static const uint32_t ids[3] = { DAS_BARCODE0_ID, DAS_BARCODE1_ID, DAS_BARCODE2_ID };
    twai_message_t m = {};
    m.extd = 0;                       /* 11-bit standard */
    m.data_length_code = 8;
    for (int f = 0; f < 3; f++) {
        m.identifier = ids[f];
        for (int i = 0; i < 8; i++) {
            size_t idx = (size_t)f * 8 + i;
            m.data[i] = idx < len ? (uint8_t)code[idx] : 0;
        }
        bridge_send(&m);              /* → bridge → CAN */
    }
    Serial.printf("[DAS] TX barcode \"%s\" (0x600/0x601/0x602, 24B)\n", code);
}

/* Transmit an arbitrary CAN frame through the bridge (used by hvac_can.*). */
void ba100_send_raw(uint32_t id, bool ext, const uint8_t *data, uint8_t dlc)
{
    twai_message_t m = {};
    m.identifier       = id;
    m.extd             = ext ? 1 : 0;
    m.data_length_code = dlc > 8 ? 8 : dlc;
    for (int i = 0; i < m.data_length_code; i++) m.data[i] = data ? data[i] : 0;
    bridge_send(&m);
}

void ba100_send_iot_restart(void)
{
    twai_message_t m = {};
    m.extd = 0;                       /* standard 11-bit */
    m.identifier = IOT_RESTART_CMD_ID;
    m.data_length_code = 2;
    m.data[0] = IOT_RESTART_MAGIC0;   /* 'R' */
    m.data[1] = IOT_RESTART_MAGIC1;   /* 'S' */
    /* Send twice for robustness against a single dropped frame, then flush the
     * UART so both are on the wire to the bridge before the caller reboots. */
    bridge_send(&m);
    bridge_send(&m);
    BRIDGE_UART.flush();
    Serial.println("[BRIDGE] TX restart-IoT (0x2D0 'RS')");
}

bool ba100_take_das_result(uint8_t *status, char *echo, size_t echo_len)
{
    if (!s_result_new) return false;
    s_result_new = false;
    if (status) *status = s_das_status;
    if (echo && echo_len) {
        size_t n = 0;
        for (; n < (size_t)DAS_ECHO_MAXLEN && n < echo_len - 1 && s_das_echo[n]; n++)
            echo[n] = s_das_echo[n];
        echo[n] = '\0';
    }
    return true;
}

void ba100_clear_das_data(void)
{
    memset(s_cell_mv,  0, sizeof(s_cell_mv));
    memset(s_temp_raw, 0, sizeof(s_temp_raw));
    /* Reset the harness-fault report so a stale one can't carry into a new test. */
    s_fault_valid = false; s_fault_init = false;
    s_fault_cell_mask = 0; s_fault_temp_mask = 0;
    s_fault_ext_valid = false; s_fault_cell_raw_mask = 0;
    s_fault_temp_repeat_mask = 0; s_fault_passing_count = 0;
}

/* ════════════════════════════════════════════════════════════════════════
 *  State snapshot
 * ════════════════════════════════════════════════════════════════════════ */
BA100_State_t ba100_get_state(void)
{
    bool alive = (s_last_rx_ms != 0) &&
                 ((uint32_t)(millis() - s_last_rx_ms) < TELEMETRY_STALE_MS);

    s_state.can_alive       = alive;
    s_state.bridge_alive    = (s_bridge_last_ms != 0) &&
                              ((uint32_t)(millis() - s_bridge_last_ms) < BRIDGE_STALE_MS);
    s_state.rx_frames       = s_rx_frames;
    s_state.current_A       = 0.0f;
    s_state.sc_relay        = false;
    s_state.chg_state       = CHG_IDLE;
    s_state.batt_data_valid = alive;
    s_state.mod_data_valid  = alive;

    memcpy(s_state.cell_mv,  s_cell_mv,  sizeof(s_cell_mv));
    memcpy(s_state.temp_raw, s_temp_raw, sizeof(s_temp_raw));
    s_state.das_status = s_das_status;

    s_state.fault_valid           = s_fault_valid;
    s_state.fault_cell_mask       = s_fault_cell_mask;
    s_state.fault_temp_mask       = s_fault_temp_mask;
    s_state.fault_init_failed     = s_fault_init;
    s_state.fault_ext_valid       = s_fault_ext_valid;
    s_state.fault_cell_raw_mask   = s_fault_cell_raw_mask;
    s_state.fault_temp_repeat_mask = s_fault_temp_repeat_mask;
    s_state.fault_passing_count   = s_fault_passing_count;

    bool iot_alive = s_iot_seen && (uint32_t)(millis() - s_iot_last_ms) < IOT_STALE_MS;
    s_state.iot_alive   = iot_alive;
    s_state.iot_tablet  = iot_alive && (s_iot_flags & IOT_FLAG_TABLET);
    s_state.iot_mqtt    = iot_alive && (s_iot_flags & IOT_FLAG_MQTT);
    s_state.iot_can_ok  = iot_alive && (s_iot_flags & IOT_FLAG_CAN_OK);
    s_state.iot_counter = s_iot_counter;

    s_state.hb_valid    = s_hb_valid;
    s_state.hb_counter  = s_hb_counter;
    s_state.hb_can_tec  = s_hb_tec;
    s_state.hb_can_rec  = s_hb_rec;
    s_state.hb_can_lec  = s_hb_lec;
    s_state.hb_tx_qdepth = s_hb_qd;
    return s_state;
}
