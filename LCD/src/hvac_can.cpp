/**
 * hvac_can.cpp  --  compressor + charger control/monitor over the UART bridge.
 * See hvac_can.h. All TX goes through ba100_send_raw(); all RX is fed in via
 * hvac_can_rx() from ba100_can_poll(). Single-threaded (loop task).
 */
#include "hvac_can.h"
#include "ba100_can.h"      /* ba100_send_raw() */
#include <Arduino.h>
#include <string.h>

/* ════════════════════════ Compressor ════════════════════════ */
static comp_variant_t s_comp_var   = COMP_T_UNKNOWN;
static comp_state_t   s_comp        = {};
static uint32_t       s_comp_rx_ms  = 0;
static bool           s_comp_on     = false;    /* operator start/stop */
static uint16_t       s_comp_rpm    = COMP_RPM_MIN;   /* default to the floor */
static float          s_comp_kw     = COMP_KW_MIN;
static bool           s_comp_tx_en  = false;    /* arm TX on first START */
static uint32_t       s_comp_tx_ms  = 0;
#define COMP_TX_PERIOD_MS   100UL
#define COMP_STALE_MS       1000UL

/* ════════════════════════ Charger ════════════════════════ */
#define ACDC_MON    0xF0U
#define ACDC_BCAST  0x3FU
#define ACDC_DEV    0x0AU
static chg_proto_t s_chg_proto = CHG_P_NONE;
static chg_state_t s_chg        = {};
static bool        s_chg_on     = false;
static float       s_chg_v      = CHG_V_DEFAULT;
static float       s_chg_a      = CHG_A_DEFAULT;
static uint32_t    s_chg_rx_ms  = 0;
static uint32_t    s_chg_tx_ms  = 0;
static uint32_t    s_chg_detect_ms = 0;
static uint8_t     s_ac_tick    = 0;
static bool        s_chg_last_on = false;
static uint8_t     s_ac_mod     = 0xFF;   /* learned module address (0..0x3B); 0xFF = unknown */
static uint8_t     s_ac_scan    = 0;      /* address being probed while s_ac_mod unknown     */
#define ACDC_MOD_MAX_SCAN   8             /* probe module addresses 0..7 */
#define CHG_STALE_MS        3000UL
#define CHG_DETECT_MS       700UL

/* ════════════════════════ Bridge sensors (flow + NTC) ════════════════════════ */
static uint16_t s_flow_hz = 0, s_flow_lpm_x10 = 0;
static uint32_t s_flow_total_x10 = 0;
static uint32_t s_flow_rx_ms = 0;
static int16_t  s_ntc_x10[SENS_NTC_COUNT] = {0};
static uint32_t s_ntc_rx_ms = 0;
#define SENS_STALE_MS  3000UL

static inline uint16_t be16(uint8_t hi, uint8_t lo) { return (uint16_t)((hi << 8) | lo); }
static inline uint16_t le16(const uint8_t *d)       { return (uint16_t)(d[0] | (d[1] << 8)); }
static inline int16_t  sle16(const uint8_t *d)      { return (int16_t)(d[0] | (d[1] << 8)); }

void hvac_can_init(void)
{
    s_comp_var = COMP_T_UNKNOWN;
    s_chg_proto = CHG_P_NONE;
    s_chg_v = CHG_V_DEFAULT;
    s_chg_a = CHG_A_DEFAULT;
    s_chg_detect_ms = millis();
}

/* ── Compressor command builders ─────────────────────────────────────────── */
static void comp_send_A(void)
{
    float kw = s_comp_kw; if (kw < 0) kw = 0; if (kw > COMP_A_KW_MAX) kw = COMP_A_KW_MAX;
    uint16_t praw = (uint16_t)(kw / 0.1f + 0.5f); if (praw > 0xFE) praw = 0xFE;
    uint8_t d[8] = {0};
    d[0] = (uint8_t)(s_comp_rpm & 0xFF);
    d[1] = (uint8_t)(s_comp_rpm >> 8);
    d[2] = (uint8_t)praw;
    d[3] = s_comp_on ? 0x01 : 0x00;
    ba100_send_raw(COMP_A_CMD_ID, false, d, 8);
}
static void comp_send_B(void)
{
    float kw = s_comp_kw; if (kw < 0) kw = 0; if (kw > COMP_B_KW_MAX) kw = COMP_B_KW_MAX;
    uint16_t praw = (uint16_t)(kw / 0.02f + 0.5f); if (praw > 0xFE) praw = 0xFE;
    uint8_t d[8] = {0};
    d[0] = (uint8_t)(s_comp_rpm & 0xFF);
    d[1] = (uint8_t)(s_comp_rpm >> 8);
    d[2] = (uint8_t)praw;
    d[3] = s_comp_on ? 0x01 : 0x00;
    ba100_send_raw(COMP_B_CMD_ID, false, d, 8);
}

/* ── Charger command builders ────────────────────────────────────────────── */
static void xnyb_send_set(void)
{
    uint8_t d[8] = {0};
    d[0] = 0xFF;                          /* broadcast address */
    d[1] = s_chg_on ? 0x03 : 0x00;
    uint16_t v10 = (uint16_t)(s_chg_v * 10.0f + 0.5f);
    uint16_t i10 = (uint16_t)(s_chg_a * 10.0f + 0.5f);
    d[2] = v10 >> 8; d[3] = v10 & 0xFF;
    d[4] = i10 >> 8; d[5] = i10 & 0xFF;
    ba100_send_raw(0x200U, false, d, 8);
}
static void acdc_send(uint8_t cmd, uint8_t dst, const uint8_t d[8])
{
    uint32_t id = ((uint32_t)ACDC_DEV << 22) | ((uint32_t)cmd << 16) |
                  ((uint32_t)dst << 8) | ACDC_MON;
    ba100_send_raw(id, true, d, 8);
}
static void acdc_send_power(void)
{
    uint8_t d[8] = {0}; d[0] = s_chg_on ? 0 : 1;
    acdc_send(0x1A, ACDC_BCAST, d);
}
static void acdc_send_setvi(void)
{
    uint32_t mv = (uint32_t)(s_chg_v * 1000.0f + 0.5f);
    uint32_t ma = (uint32_t)(s_chg_a * 1000.0f + 0.5f);
    uint8_t d[8] = { (uint8_t)(mv>>24),(uint8_t)(mv>>16),(uint8_t)(mv>>8),(uint8_t)mv,
                     (uint8_t)(ma>>24),(uint8_t)(ma>>16),(uint8_t)(ma>>8),(uint8_t)ma };
    acdc_send(0x1C, ACDC_BCAST, d);
}
static void acdc_send_read_to(uint8_t cmd, uint8_t dst)
{
    uint8_t d[8] = {0};
    acdc_send(cmd, dst, d);
}
static void acdc_send_read(uint8_t cmd) { acdc_send_read_to(cmd, ACDC_BCAST); }

/* Per-module reads (0x09 V/I, 0x04 status) are NOT answered when sent to the
 * broadcast address 0x3F — they must target a real module address. Use the
 * learned address, or scan 0..ACDC_MOD_MAX_SCAN-1 until one answers. */
static uint8_t acdc_mod_target(void)
{
    if (s_ac_mod != 0xFF) return s_ac_mod;
    uint8_t a = s_ac_scan;
    s_ac_scan = (uint8_t)((s_ac_scan + 1) % ACDC_MOD_MAX_SCAN);
    return a;
}

/* V/I payload: some firmwares send uint32 mV / mA (BE), others IEEE-754 float
 * V / A (BE). A uint32 mV below 1000 V has d[0]==0 && d[1]<0x10; any float >=1
 * has d[0]>=0x3F. All-zero is 0 either way. */
static void acdc_decode_vi(const uint8_t *d, float *v, float *a)
{
    bool as_int = (d[0] == 0x00 && d[1] < 0x10);
    if (as_int) {
        uint32_t mv = ((uint32_t)d[0]<<24)|((uint32_t)d[1]<<16)|((uint32_t)d[2]<<8)|d[3];
        uint32_t ma = ((uint32_t)d[4]<<24)|((uint32_t)d[5]<<16)|((uint32_t)d[6]<<8)|d[7];
        *v = mv / 1000.0f; *a = ma / 1000.0f;
    } else {
        uint32_t fv = ((uint32_t)d[0]<<24)|((uint32_t)d[1]<<16)|((uint32_t)d[2]<<8)|d[3];
        uint32_t fa = ((uint32_t)d[4]<<24)|((uint32_t)d[5]<<16)|((uint32_t)d[6]<<8)|d[7];
        memcpy(v, &fv, 4); memcpy(a, &fa, 4);
    }
}

/* ── Periodic TX ─────────────────────────────────────────────────────────── */
void hvac_can_tick(void)
{
    uint32_t now = millis();

    /* Compressor: once armed (first START), keep sending the command at 100 ms
     * so a later STOP (on=0) is delivered too. Probe both variants until one is
     * detected from its status frame. */
    if (s_comp_tx_en && (now - s_comp_tx_ms) >= COMP_TX_PERIOD_MS) {
        s_comp_tx_ms = now;
        if      (s_comp_var == COMP_T_A) comp_send_A();
        else if (s_comp_var == COMP_T_B) comp_send_B();
        else { comp_send_A(); comp_send_B(); }
    }

    /* Charger: detect (nudge both protocols) until one locks, then pace its TX. */
    if (s_chg_proto == CHG_P_NONE) {
        if (now - s_chg_detect_ms >= CHG_DETECT_MS) {
            s_chg_detect_ms = now;
            acdc_send_read(0x08);   /* nudge an ACDC module */
            xnyb_send_set();        /* nudge / heartbeat an XNYB module */
        }
        return;
    }
    if (s_chg_proto == CHG_P_XNYB) {
        if (now - s_chg_tx_ms >= 200) { s_chg_tx_ms = now; xnyb_send_set(); }
    } else { /* ACDC */
        if (s_chg_on != s_chg_last_on) { s_chg_last_on = s_chg_on; acdc_send_power(); s_chg_tx_ms = now; }
        if (now - s_chg_tx_ms >= 150) {
            s_chg_tx_ms = now;
            switch (s_ac_tick % 4) {
                case 0: if (s_chg_on) acdc_send_setvi(); else acdc_send_read(0x08); break;
                case 1: acdc_send_read(0x08); break;                        /* system V/I (broadcast, answered by 0x3F) */
                case 2: acdc_send_read_to(0x09, acdc_mod_target()); break;  /* module V/I (addressed)  */
                case 3: acdc_send_read_to(0x04, acdc_mod_target()); break;  /* module status (addressed) */
            }
            s_ac_tick++;
        }
    }

    /* Re-arm charger detection if the bus goes quiet (unplugged/swapped). */
    if (s_chg_rx_ms && (now - s_chg_rx_ms) > 5000) {
        s_chg_proto = CHG_P_NONE;
        s_ac_mod = 0xFF;
        s_chg_detect_ms = now;
    }
}

/* ── RX parse (one frame) ────────────────────────────────────────────────── */
void hvac_can_rx(uint32_t id, bool ext, const uint8_t *d, uint8_t dlc)
{
    uint32_t now = millis();

    if (!ext) {
        /* Bridge sensor frames (flow meter + MCP3208 NTC temps). */
        if (id == SENS_FLOW_ID && dlc >= 8) {
            s_flow_hz        = le16(&d[0]);
            s_flow_lpm_x10   = le16(&d[2]);
            s_flow_total_x10 = (uint32_t)d[4] | ((uint32_t)d[5] << 8) |
                               ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);
            s_flow_rx_ms = now;
            return;
        }
        if (id == SENS_NTC0_ID && dlc >= 8) {
            for (int i = 0; i < 4; i++) s_ntc_x10[i] = sle16(&d[i * 2]);
            s_ntc_rx_ms = now;
            return;
        }
        if (id == SENS_NTC1_ID && dlc >= 8) {
            for (int i = 0; i < 4; i++) s_ntc_x10[i + 4] = sle16(&d[i * 2]);
            s_ntc_rx_ms = now;
            return;
        }
        /* Compressor status */
        if (id == COMP_A_STAT_ID && dlc >= 8) {
            s_comp_var = COMP_T_A;
            s_comp.variant   = COMP_T_A;
            s_comp.rpm       = le16(&d[0]);
            s_comp.status    = d[3];
            s_comp.error     = d[4];
            s_comp.current_a = d[6] * 0.1f;
            s_comp.voltage_v = (uint16_t)d[7] * 4;
            s_comp.temp_c    = 0; s_comp.power_kw = 0;
            s_comp_rx_ms = now;
            return;
        }
        if (id == COMP_B_STAT_ID && dlc >= 8) {
            s_comp_var = COMP_T_B;
            s_comp.variant   = COMP_T_B;
            s_comp.rpm       = le16(&d[0]);
            s_comp.power_kw  = d[2] * 0.02f;
            s_comp.status    = d[3];
            s_comp.error     = d[4];
            s_comp.temp_c    = (int16_t)d[5] - 40;
            s_comp.current_a = d[6] * 1.0f;
            s_comp.voltage_v = (uint16_t)d[7] * 1;
            s_comp_rx_ms = now;
            return;
        }
        /* Charger XNYB report: base 0x200..0x600, addr <= 0x1F. Exclude our own
         * barcode IDs 0x600/601/602 which share the range. */
        uint32_t base = id & 0xFF00U;
        if (base >= 0x200U && base <= 0x600U && (id & 0xFFU) <= 0x1FU &&
            id != 0x600U && id != 0x601U && id != 0x602U && dlc >= 8) {
            if (s_chg_proto != CHG_P_XNYB) s_chg_proto = CHG_P_XNYB;
            s_chg_rx_ms = now;
            if (base == 0x200U) {
                s_chg.fault = d[0];
                bool on = (d[2] & 0x01) != 0;
                s_chg.volts = be16(d[4], d[5]) * 0.1f;
                s_chg.amps  = be16(d[6], d[7]) * 0.1f;
                s_chg.on    = on;
            }
        }
        return;
    }

    /* Charger ACDC (extended). id = DEV<<22 | cmd<<16 | dst<<8 | src. */
    uint8_t cmd = (id >> 16) & 0x3FU;
    uint8_t dst = (id >> 8) & 0xFFU;
    uint8_t src = id & 0xFFU;
    if (cmd == 0x00 || cmd == 0x0E) return;    /* inter-module chatter */
    if (dst != ACDC_MON) return;               /* only replies addressed to us (0xF0) */
    if (s_chg_proto != CHG_P_ACDC) s_chg_proto = CHG_P_ACDC;
    s_chg_rx_ms = now;
    if (src <= 0x3BU && s_ac_mod == 0xFF) {    /* a real module answered → remember it */
        s_ac_mod = src;
        Serial.printf("[CHG] module address learned: 0x%02X\n", src);
    }
    if ((cmd == 0x08 || cmd == 0x09) && dlc >= 8) {
        float v, a; acdc_decode_vi(d, &v, &a);
        if (v >= 0.0f && v < 1500.0f && a >= -1.0f && a < 500.0f) {   /* sanity */
            s_chg.volts = v;
            s_chg.amps  = a;
        }
    } else if (cmd == 0x04 && dlc >= 8) {
        s_chg.fault = d[7];
    }
    s_chg.on = s_chg_on;   /* ACDC has no direct on-report; mirror our command */
}

/* ── Compressor accessors ────────────────────────────────────────────────── */
void comp_set_on(bool on)      { s_comp_on = on; if (on) s_comp_tx_en = true; }
void comp_set_rpm(uint16_t r)  { if (r < COMP_RPM_MIN) r = COMP_RPM_MIN; if (r > COMP_RPM_MAX) r = COMP_RPM_MAX; s_comp_rpm = r; }
void comp_set_kw(float kw)     { float m = comp_kw_max(); if (kw < COMP_KW_MIN) kw = COMP_KW_MIN; if (kw > m) kw = m; s_comp_kw = kw; }
bool comp_get_on(void)         { return s_comp_on; }
uint16_t comp_get_rpm(void)    { return s_comp_rpm; }
float comp_get_kw(void)        { return s_comp_kw; }
float comp_kw_max(void)        { return (s_comp_var == COMP_T_B) ? COMP_B_KW_MAX : COMP_A_KW_MAX; }
comp_state_t comp_get(void)
{
    comp_state_t s = s_comp;
    s.online = (s_comp_rx_ms != 0) && ((millis() - s_comp_rx_ms) < COMP_STALE_MS);
    if (!s.online) s.variant = s_comp_var;   /* keep last known type in the badge */
    return s;
}

/* ── Charger accessors ───────────────────────────────────────────────────── */
void chg_set_on(bool on)       { s_chg_on = on; }
void chg_set_volts(float v)    { if (v < CHG_V_MIN) v = CHG_V_MIN; if (v > CHG_V_MAX) v = CHG_V_MAX; s_chg_v = v; }
void chg_set_amps(float a)     { if (a < CHG_A_MIN) a = CHG_A_MIN; if (a > CHG_A_MAX) a = CHG_A_MAX; s_chg_a = a; }
bool chg_get_on(void)          { return s_chg_on; }
float chg_get_volts(void)      { return s_chg_v; }
float chg_get_amps(void)       { return s_chg_a; }
chg_state_t chg_get(void)
{
    chg_state_t s = s_chg;
    s.proto  = s_chg_proto;
    s.online = (s_chg_rx_ms != 0) && ((millis() - s_chg_rx_ms) < CHG_STALE_MS);
    return s;
}

/* ── Bridge sensors ──────────────────────────────────────────────────────── */
sens_state_t sens_get(void)
{
    sens_state_t s = {};
    uint32_t now = millis();
    s.flow_online    = (s_flow_rx_ms != 0) && ((now - s_flow_rx_ms) < SENS_STALE_MS);
    s.flow_hz        = s_flow_hz;
    s.flow_lpm_x10   = s_flow_lpm_x10;
    s.flow_total_x10 = s_flow_total_x10;
    s.ntc_online     = (s_ntc_rx_ms != 0) && ((now - s_ntc_rx_ms) < SENS_STALE_MS);
    for (int i = 0; i < SENS_NTC_COUNT; i++) s.ntc_x10[i] = s_ntc_x10[i];
    return s;
}