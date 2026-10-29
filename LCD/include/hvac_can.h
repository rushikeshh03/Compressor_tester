#pragma once
/**
 * hvac_can.h  --  compressor + charger CAN control/monitor (LCD side).
 *
 * The LCD manually drives an electric A/C COMPRESSOR and a CHARGER and shows
 * their live status. It has no CAN transceiver, so every frame is transmitted
 * and received through the UART CAN_Bridge (ba100_send_raw / a per-frame hook
 * from ba100_can_poll). This module only builds/parses the protocol; the bridge
 * does the physical CAN.
 *
 * Both devices are AUTO-DETECTED from the frames they emit:
 *   Compressor : Variant A (cmd 0x238 / status 0x248, power x0.1 kW) or
 *                Variant B (cmd 0x34C / status 0x349, power x0.02 kW, Posung 48V)
 *   Charger    : XNYB LEVDC (standard-ID) or NIUERA ACDC (extended-ID)
 *
 * Datasheets: ../../Compressor/CAN_20(500kbps).pdf + CAN_13-A01-…-48V.pdf,
 *             ../../Charger Data/XNYB… + NIUERA ACDC… protocol PDFs.
 */
#include <stdint.h>
#include <stdbool.h>

/* ── CAN identifiers ─────────────────────────────────────────────────────── */
#define COMP_A_CMD_ID     0x238U   /* AC -> Comp  (we TX)  Variant A */
#define COMP_A_STAT_ID    0x248U   /* Comp -> AC  (we RX)  Variant A */
#define COMP_B_CMD_ID     0x34CU   /* ctrl -> ACP (we TX)  Variant B */
#define COMP_B_STAT_ID    0x349U   /* ACP -> ctrl (we RX)  Variant B */

/* ── UI limits / steps ───────────────────────────────────────────────────── */
#define COMP_RPM_MIN      2000U    /* floor: never command below 2000 rpm */
#define COMP_RPM_MAX      8500U
#define COMP_RPM_STEP     100U
#define COMP_KW_MIN       1.0f     /* floor: never command below 1 kW */
#define COMP_A_KW_MAX     25.4f
#define COMP_B_KW_MAX     5.08f
#define COMP_KW_STEP      0.5f

#define CHG_V_MIN         400.0f   /* charger floor 400 V */
#define CHG_V_MAX         600.0f
#define CHG_V_STEP        5.0f
#define CHG_V_DEFAULT     400.0f
#define CHG_A_MIN         2.0f     /* charger floor 2 A */
#define CHG_A_MAX         10.0f
#define CHG_A_STEP        1.0f
#define CHG_A_DEFAULT     5.0f

/* ── Bridge-sourced sensor frames (CAN_Bridge board) ─────────────────────────
 * 0x7B0 flow: hz(LE16), lpm×10(LE16), total_l×10(LE32)
 * 0x7B1/0x7B2 NTC temps: 4×int16 LE, 0.1 °C, 0x8000 = invalid channel */
#define SENS_FLOW_ID      0x7B0U
#define SENS_NTC0_ID      0x7B1U
#define SENS_NTC1_ID      0x7B2U
#define SENS_NTC_COUNT    8
#define SENS_NTC_INVALID  ((int16_t)0x8000)

typedef struct {
    bool     flow_online;
    uint16_t flow_hz;
    uint16_t flow_lpm_x10;
    uint32_t flow_total_x10;
    bool     ntc_online;
    int16_t  ntc_x10[SENS_NTC_COUNT];   /* 0.1 °C, SENS_NTC_INVALID if open */
} sens_state_t;

typedef enum { COMP_T_UNKNOWN = 0, COMP_T_A = 1, COMP_T_B = 2 } comp_variant_t;
typedef enum { CHG_P_NONE = 0, CHG_P_XNYB = 1, CHG_P_ACDC = 2 } chg_proto_t;

typedef struct {
    comp_variant_t variant;
    bool     online;        /* a status frame arrived < timeout ago */
    uint16_t rpm;
    uint8_t  status;        /* 0=Off 1=On 3=Error */
    uint8_t  error;         /* bitmask */
    float    current_a;
    uint16_t voltage_v;
    int16_t  temp_c;        /* Variant B only (else 0) */
    float    power_kw;      /* Variant B reports input power (else 0) */
} comp_state_t;

typedef struct {
    chg_proto_t proto;
    bool     online;
    float    volts;
    float    amps;
    bool     on;
    uint8_t  fault;
} chg_state_t;

#ifdef __cplusplus
extern "C" {
#endif

void          hvac_can_init(void);
/* Feed every CAN frame the bridge relays here (called from ba100_can_poll). */
void          hvac_can_rx(uint32_t id, bool ext, const uint8_t *data, uint8_t dlc);
/* Pace the periodic command/detection TX (called every ba100_can_poll). */
void          hvac_can_tick(void);

/* Compressor control (from the UI). comp_set_on(true) also arms transmission. */
void          comp_set_on(bool on);
void          comp_set_rpm(uint16_t rpm);
void          comp_set_kw(float kw);
comp_state_t  comp_get(void);
bool          comp_get_on(void);
uint16_t      comp_get_rpm(void);
float         comp_get_kw(void);
float         comp_kw_max(void);          /* per detected variant */

/* Charger control (from the UI). */
void          chg_set_on(bool on);
void          chg_set_volts(float v);
void          chg_set_amps(float a);
chg_state_t   chg_get(void);
bool          chg_get_on(void);
float         chg_get_volts(void);
float         chg_get_amps(void);

/* Bridge sensors (flow meter + MCP3208 NTC temps), decoded from 0x7B0/1/2. */
sens_state_t  sens_get(void);

#ifdef __cplusplus
}
#endif
