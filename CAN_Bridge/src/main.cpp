/**
 * CAN_Bridge — ESP32-WROOM UART <-> CAN transparent bridge
 *
 * Sits between the Harness Tester LCD (Waveshare ESP32-S3, no CAN transceiver of
 * its own) and the CAN bus. It is a DUMB PIPE: every CAN frame received is framed
 * and sent to the LCD over UART; every framed message from the LCD is transmitted
 * on CAN verbatim. No protocol knowledge — the LCD does all the DAS decoding.
 *
 * Both boards are 3.3 V, so the UART link needs no level shifting.
 *
 * Bridge hardware = the ESP32-WROOM "Compressor" board (TJA1050 CAN on 16/17,
 * spare UART header on 22/23, flow-meter input on GPIO35). We only use its
 * hardware — none of its compressor/charger/DWIN firmware.
 *
 *   LCD GPIO43 (TX) ───────────────→ bridge GPIO22 (RX, Serial2)
 *   LCD GPIO44 (RX) ←─────────────── bridge GPIO23 (TX, Serial2)
 *   GND ──────────────────────────── GND
 *
 *   bridge GPIO17 (CAN_TX) → TJA1050 TXD
 *   bridge GPIO16 (CAN_RX) ← TJA1050 RXD
 *   transceiver CANH/CANL → bus  (120 Ω termination at each bus end)
 *
 *   bridge GPIO35 (input-only) ← flow-meter pulse (external 1 kΩ pull-up to
 *   3.3 V). The bridge counts pulses (YF-B2, 11 Hz per L/min) and broadcasts a
 *   flow frame (0x7B0) on CAN once a second, so it is logged/relayed like any
 *   other frame.
 *
 * Status LEDs:
 *   GPIO2  (onboard)  — flickers on CAN bus activity
 *   GPIO25 (external) — solid ON while the LCD link is up ("ESP32 connected")
 *   GPIO26 (external) — ~1 Hz heartbeat: the bridge firmware is running
 * The LCD sends a ~2 Hz link heartbeat (a bridge-local control frame, never put
 * on CAN); the link LED stays solid while those keep arriving.
 *
 * The bridge's own USB (Serial/UART0 @115200) is a CAN debug console — set
 * DBG_VERBOSE to 0 if the bus is busy enough that per-frame printing adds delay.
 *
 * Wire frame (16 bytes, identical on the LCD side — ba100_can.cpp):
 *   [0]=0xAA sync  [1]=flags(bit0=ext, bit1=ctrl/link-heartbeat)  [2..5]=id LE32
 *   [6]=dlc  [7..14]=data[8]  [15]=XOR checksum of bytes [1..14].
 */

#include <Arduino.h>
#include "driver/twai.h"
#include <SPI.h>
#include <math.h>

/* ── Pins (ESP32-WROOM "Compressor" board used as the bridge) ─────────────── */
#define CAN_RX_PIN     16     /* ← TJA1050 RXD */
#define CAN_TX_PIN     17     /* → TJA1050 TXD */

/* Status LEDs (active-high; add an external LED + ~330 Ω to GND on 25/26; GPIO2
 * is the onboard LED). Change the pins here if your wiring differs. */
#define LED_CAN_PIN    2      /* onboard — flickers on CAN bus activity          */
#define LED_UART_PIN   25     /* solid ON while the LCD link is up ("connected")  */
#define LED_HB_PIN     26     /* ~1 Hz blink — bridge firmware alive (heartbeat)  */

#define LCD_UART       Serial2
#define LCD_UART_RX    22     /* ← LCD TX (GPIO43)  (spare UART header) */
#define LCD_UART_TX    23     /* → LCD RX (GPIO44) */

/* ── Flow meter (YF-B2 on GPIO35, input-only; external 1 kΩ pull-up to 3.3 V) ─
 * Counted here and broadcast on CAN as 0x7B0 once a second:
 *   data[0..1] = pulse Hz (LE16)
 *   data[2..3] = flow L/min ×10 (LE16)
 *   data[4..7] = total litres ×10 (LE32) */
#define FLOW_PIN               34
#define FLOW_CAN_ID            0x7B0U
#define FLOW_HZ_PER_LPM_X100   1100u   /* 11.00 Hz per L/min (YF-B2) */
#define FLOW_PULSES_PER_DL     ((FLOW_HZ_PER_LPM_X100 * 60u) / 1000u)  /* 66 pulses / 0.1 L */
#define FLOW_MIN_EDGE_US       1000u   /* glitch filter (max real ~275 Hz) */

/* ── MCP3208 12-bit 8-channel SPI ADC → 10k NTC temps ─────────────────────────
 * Run VDD+VREF at 3.3 V (ratiometric with the divider, and 3.3 V DOUT is safe
 * for the ESP32). Each channel: 10k NTC from 3.3 V to the pin, 10k to GND
 * (NTC_ON_TOP=1). Temps broadcast on CAN 0x7B1 (ch0-3) + 0x7B2 (ch4-7), each
 * 4×int16 LE in 0.1 °C, at 1 Hz; 0x8000 = open/short/invalid channel. */
#define ADC_SCK_PIN            18
#define ADC_MISO_PIN           19
#define ADC_MOSI_PIN           13
#define ADC_CS_PIN             27
#define ADC_CH_COUNT           8
#define ADC_FULLSCALE          4096.0f
#define ADC_SPI_HZ             1000000UL   /* 1 MHz — safe for MCP3208 @3.3 V */
#define NTC_TEMP_ID_0          0x7B1U
#define NTC_TEMP_ID_1          0x7B2U
#define NTC_R_FIXED            10000.0f    /* 10k divider resistor */
#define NTC_R0                 10000.0f    /* NTC nominal @ 25 °C  */
#define NTC_T0_K               298.15f     /* 25 °C in kelvin      */
#define NTC_BETA               3950.0f     /* NTC beta — set to your part */
#define NTC_ON_TOP             1           /* 1: NTC→3.3V, 10k→GND; 0: NTC→GND, 10k→3.3V */
#define NTC_INVALID            ((int16_t)0x8000)

/* LCD link is "up" if a valid frame (incl. its ~2 Hz link heartbeat) arrived
 * within this window. */
#define LCD_LINK_TIMEOUT_MS  1500UL

/* ── Must match the LCD (LCD/include/app_config.h) ────────────────────────── */
#define UART_BAUD      1000000UL   /* == CONFIG_BRIDGE_UART_BAUD */
#define CAN_BAUD_KBPS  500         /* == CONFIG_CAN_BAUD_KBPS (125/250/500)     */

/* ── Debug console (USB) ──────────────────────────────────────────────────── */
#define DBG            Serial
#define DBG_BAUD       115200
#define DBG_VERBOSE    1           /* 1 = print each relayed frame; 0 = silent   */

/* Diagnostic: 1 = TWAI_MODE_NO_ACK (self-test, no other node needed to ACK).
 * If bus-off STILL happens with this on, the fault is TX/RX pins, transceiver
 * power, or RXD level — not the bus. Set back to 0 for normal operation. */
#define CAN_SELFTEST   0

/* Charger probe (diagnostic): 1 = the bridge itself polls the charger and prints
 * every reply as "[PROBE] ...". It sweeps per-module commands over module
 * addresses 0..0x3B plus broadcast system commands, so we learn which commands /
 * addresses the charger really answers — with no LCD involved. Set 0 for normal use. */
#define CHG_PROBE      1

/* ── Wire framing ─────────────────────────────────────────────────────────── */
#define FRAME_LEN      16
#define FRAME_SYNC     0xAAU
#define FRAME_FLAG_EXT   0x01U   /* byte[1] bit0: extended CAN id */
#define FRAME_FLAG_CTRL  0x02U   /* byte[1] bit1: control frame (not a CAN frame) */
#define CTRL_CMD_REBOOT  0x01U   /* control data[0]: reboot this bridge board     */

static uint32_t s_lcd_last_ms = 0;   /* millis() of the last valid frame from the LCD */

static bool s_can_running = false;   /* updated by can_supervise() */

static void frame_from_msg(const twai_message_t *m, uint8_t *f);   /* fwd decl */
static void tx_and_mirror(uint32_t id, bool ext, const uint8_t *d, uint8_t dlc); /* fwd decl */

/* ── Flow meter (interrupt-counted; broadcast on CAN @1 Hz) ───────────────── */
static volatile uint32_t s_flow_pulses   = 0;
static volatile uint32_t s_flow_last_edge = 0;
static uint32_t          s_flow_total     = 0;   /* total pulses since boot */
static uint32_t          s_flow_tick_ms   = 0;

static void IRAM_ATTR flow_isr(void)
{
    uint32_t now = (uint32_t)micros();
    if ((now - s_flow_last_edge) >= FLOW_MIN_EDGE_US) {   /* glitch filter */
        s_flow_last_edge = now;
        s_flow_pulses++;
    }
}

static void flow_init(void)
{
    pinMode(FLOW_PIN, INPUT);   /* input-only GPIO35; external pull-up required */
    attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flow_isr, RISING);
    s_flow_tick_ms = millis();
}

/* Once a second: latch the pulse count, compute Hz / L·min / total, and TX 0x7B0. */
static void flow_tick(void)
{
    uint32_t now = millis();
    uint32_t dt  = now - s_flow_tick_ms;
    if (dt < 1000u) return;
    s_flow_tick_ms = now;

    noInterrupts();
    uint32_t cnt = s_flow_pulses;
    s_flow_pulses = 0;
    interrupts();

    uint32_t pps = (cnt * 1000u + dt / 2u) / dt;
    uint16_t hz  = pps > 0xFFFFu ? 0xFFFFu : (uint16_t)pps;
    uint32_t lpm_x10 = (pps * 1000u + FLOW_HZ_PER_LPM_X100 / 2u) / FLOW_HZ_PER_LPM_X100;
    uint16_t lpm = lpm_x10 > 0xFFFFu ? 0xFFFFu : (uint16_t)lpm_x10;
    s_flow_total += cnt;
    uint32_t total_x10 = s_flow_total / FLOW_PULSES_PER_DL;

    uint8_t d[8];
    d[0] = (uint8_t)hz;         d[1] = (uint8_t)(hz >> 8);
    d[2] = (uint8_t)lpm;        d[3] = (uint8_t)(lpm >> 8);
    d[4] = (uint8_t)total_x10;  d[5] = (uint8_t)(total_x10 >> 8);
    d[6] = (uint8_t)(total_x10 >> 16); d[7] = (uint8_t)(total_x10 >> 24);
    tx_and_mirror(FLOW_CAN_ID, false, d, 8);
}

/* ── MCP3208 (SPI ADC) → 10k NTC temperatures, broadcast on CAN ───────────── */
static SPIClass s_adc_spi(HSPI);
static uint32_t s_adc_tick_ms = 0;

static void adc_init(void)
{
    pinMode(ADC_CS_PIN, OUTPUT);
    digitalWrite(ADC_CS_PIN, HIGH);
    s_adc_spi.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);
    s_adc_tick_ms = millis();
}

/* One single-ended MCP3208 conversion (0..4095). */
static uint16_t mcp3208_read(uint8_t ch)
{
    uint8_t b0 = 0x06 | ((ch & 0x04) >> 2);   /* start + single-ended + D2 */
    uint8_t b1 = (uint8_t)((ch & 0x03) << 6); /* D1 D0 */
    s_adc_spi.beginTransaction(SPISettings(ADC_SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(ADC_CS_PIN, LOW);
    s_adc_spi.transfer(b0);
    uint8_t r1 = s_adc_spi.transfer(b1);
    uint8_t r2 = s_adc_spi.transfer(0x00);
    digitalWrite(ADC_CS_PIN, HIGH);
    s_adc_spi.endTransaction();
    return (uint16_t)(((r1 & 0x0F) << 8) | r2);
}

/* Convert one ADC count to temperature (°C ×10) via the NTC Beta equation.
 * Ratiometric: only the divider ratio matters, so VREF cancels. */
static int16_t ntc_temp_x10(uint16_t adc)
{
    if (adc < 8 || adc > 4088) return NTC_INVALID;   /* open / short */
    float ratio = adc / ADC_FULLSCALE;               /* Vnode / VREF */
#if NTC_ON_TOP
    float rntc = NTC_R_FIXED * (1.0f - ratio) / ratio;   /* NTC→VREF, fixed→GND */
#else
    float rntc = NTC_R_FIXED * ratio / (1.0f - ratio);   /* NTC→GND, fixed→VREF */
#endif
    float tK = 1.0f / (1.0f / NTC_T0_K + logf(rntc / NTC_R0) / NTC_BETA);
    float tC = tK - 273.15f;
    if (tC < -55.0f) tC = -55.0f; if (tC > 155.0f) tC = 155.0f;
    return (int16_t)lroundf(tC * 10.0f);
}

static void adc_tick(void)
{
    uint32_t now = millis();
    if (now - s_adc_tick_ms < 1000u) return;
    s_adc_tick_ms = now;

    int16_t t[ADC_CH_COUNT];
    for (int i = 0; i < ADC_CH_COUNT; i++) t[i] = ntc_temp_x10(mcp3208_read(i));

    uint8_t d[8];
    for (int i = 0; i < 4; i++) { d[i*2] = (uint8_t)t[i];   d[i*2+1] = (uint8_t)(t[i]   >> 8); }
    tx_and_mirror(NTC_TEMP_ID_0, false, d, 8);
    for (int i = 0; i < 4; i++) { d[i*2] = (uint8_t)t[i+4]; d[i*2+1] = (uint8_t)(t[i+4] >> 8); }
    tx_and_mirror(NTC_TEMP_ID_1, false, d, 8);
}

static void frame_from_msg(const twai_message_t *m, uint8_t *f)
{
    f[0] = FRAME_SYNC;
    f[1] = m->extd ? 0x01 : 0x00;
    uint32_t id = m->identifier;
    f[2] = (uint8_t)id;         f[3] = (uint8_t)(id >> 8);
    f[4] = (uint8_t)(id >> 16); f[5] = (uint8_t)(id >> 24);
    uint8_t dlc = m->data_length_code > 8 ? 8 : m->data_length_code;
    f[6] = dlc;
    for (int i = 0; i < 8; i++) f[7 + i] = (i < dlc) ? m->data[i] : 0;
    uint8_t cs = 0; for (int i = 1; i < 15; i++) cs ^= f[i];
    f[15] = cs;
}

/* Transmit a locally-generated frame on CAN (best-effort — needs another node
 * to ACK) AND mirror it to the LCD over UART, so the frame reaches the LCD raw
 * log / decoders even when the bridge is alone on the bus. */
static void tx_and_mirror(uint32_t id, bool ext, const uint8_t *d, uint8_t dlc)
{
    twai_message_t m = {};
    m.identifier = id; m.extd = ext ? 1 : 0;
    m.data_length_code = dlc > 8 ? 8 : dlc;
    for (int i = 0; i < m.data_length_code; i++) m.data[i] = d ? d[i] : 0;
    if (s_can_running) twai_transmit(&m, pdMS_TO_TICKS(5));
    uint8_t f[FRAME_LEN];
    frame_from_msg(&m, f);
    LCD_UART.write(f, FRAME_LEN);
}

/* Tell the LCD we're alive: a control frame (not a CAN frame) so the LCD's
 * top-bar UART/bridge chip can show "connected" even when the CAN bus is quiet. */
static void lcd_send_link_heartbeat(void)
{
    uint8_t f[FRAME_LEN] = {0};
    f[0] = FRAME_SYNC;
    f[1] = FRAME_FLAG_CTRL;   /* id=0, dlc=0 */
    uint8_t cs = 0; for (int i = 1; i < 15; i++) cs ^= f[i];
    f[15] = cs;
    LCD_UART.write(f, FRAME_LEN);
}

/* Non-blocking UART frame assembler (LCD → bridge). Returns true per full frame. */
static bool uart_recv_frame(twai_message_t *m)
{
    static uint8_t buf[FRAME_LEN];
    static int     len = 0;
    while (LCD_UART.available()) {
        uint8_t b = (uint8_t)LCD_UART.read();
        if (len == 0 && b != FRAME_SYNC) continue;
        buf[len++] = b;
        if (len < FRAME_LEN) continue;
        len = 0;
        uint8_t cs = 0; for (int i = 1; i < 15; i++) cs ^= buf[i];
        if (cs != buf[15]) continue;                    /* corrupt → resync */
        s_lcd_last_ms = millis();                       /* any valid frame = LCD link alive */
        if (buf[1] & FRAME_FLAG_CTRL) {                 /* control frame → not a CAN frame */
            if (buf[6] >= 1 && buf[7] == CTRL_CMD_REBOOT) {
                DBG.println("[BRIDGE] reboot command from LCD — restarting");
                DBG.flush();
                delay(50);
                ESP.restart();                          /* does not return */
            }
            continue;                                   /* else: link heartbeat */
        }
        twai_message_t t = {};
        t.extd             = (buf[1] & FRAME_FLAG_EXT) ? 1 : 0;
        t.identifier       = (uint32_t)buf[2] | ((uint32_t)buf[3] << 8) |
                             ((uint32_t)buf[4] << 16) | ((uint32_t)buf[5] << 24);
        t.data_length_code = buf[6] > 8 ? 8 : buf[6];
        for (int i = 0; i < 8; i++) t.data[i] = buf[7 + i];
        *m = t;
        return true;
    }
    return false;
}

/* ── CAN bring-up ─────────────────────────────────────────────────────────── */
static bool can_start(void)
{
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN,
        CAN_SELFTEST ? TWAI_MODE_NO_ACK : TWAI_MODE_NORMAL);
    g.rx_queue_len = 64;
    g.tx_queue_len = 32;
#if   CAN_BAUD_KBPS == 500
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
#elif CAN_BAUD_KBPS == 250
    twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();
#else
    twai_timing_config_t t = TWAI_TIMING_CONFIG_125KBITS();
#endif
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    return (twai_driver_install(&g, &t, &f) == ESP_OK) && (twai_start() == ESP_OK);
}

/* CAN supervisor: bus-off recovery needs THREE steps on ESP32 TWAI:
 *   BUS_OFF --twai_initiate_recovery()--> RECOVERING --(128x11 recessive bits)-->
 *   STOPPED --twai_start()--> RUNNING.
 * The old code stopped after step 1, so every TX failed forever. */
static void can_supervise(void)
{
    static uint32_t last_dbg = 0;
    twai_status_info_t st;
    if (twai_get_status_info(&st) != ESP_OK) { s_can_running = false; return; }

    s_can_running = (st.state == TWAI_STATE_RUNNING);

    if (st.state == TWAI_STATE_BUS_OFF) {
        twai_initiate_recovery();
        DBG.println("[BRIDGE] CAN bus-off -> recovering");
    } else if (st.state == TWAI_STATE_STOPPED) {
        twai_clear_transmit_queue();
        if (twai_start() == ESP_OK) {
            DBG.println("[BRIDGE] CAN restarted");
            s_can_running = true;
        }
    }

    uint32_t now = millis();
    if (now - last_dbg >= 2000) {
        last_dbg = now;
        /* Sample the transceiver RXD pin for ~2 ms. Idle/healthy bus ≈ 0 lows;
         * rx_low == 200 means RXD is STUCK LOW (dominant) → controller never joins the bus. */
        int rx_low = 0;
        for (int i = 0; i < 200; i++) { if (digitalRead(CAN_RX_PIN) == LOW) rx_low++; delayMicroseconds(10); }
        DBG.printf("[CAN] rx_low=%d/200 ", rx_low);
        DBG.printf("[CAN] state=%d TEC=%lu REC=%lu tx_q=%lu rx_q=%lu tx_failed=%lu bus_err=%lu arb_lost=%lu\n",
                   (int)st.state, (unsigned long)st.tx_error_counter,
                   (unsigned long)st.rx_error_counter, (unsigned long)st.msgs_to_tx,
                   (unsigned long)st.msgs_to_rx, (unsigned long)st.tx_failed_count,
                   (unsigned long)st.bus_error_count, (unsigned long)st.arb_lost_count);
    }
}

/* ── Charger probe ─────────────────────────────────────────────────────────
 * ID = 0x0A<<22 | cmd<<16 | dst<<8 | src ; controller (us) = 0xF0, broadcast 0x3F.
 * Sweeps (per-module cmd x address 0..0x3B) plus broadcast system cmds, one request
 * every 50 ms, and reports which (cmd, src) pairs answer. */
#if CHG_PROBE
#define PROBE_SRC        0xF0U
#define PROBE_PERIOD_MS  50U
static const uint8_t PROBE_BCAST_CMDS[] = { 0x01, 0x02, 0x08 };
static const uint8_t PROBE_MOD_CMDS[]   = { 0x03, 0x04, 0x06, 0x09, 0x0C };
#define N_BCAST (sizeof(PROBE_BCAST_CMDS)/sizeof(PROBE_BCAST_CMDS[0]))
#define N_MOD   (sizeof(PROBE_MOD_CMDS)/sizeof(PROBE_MOD_CMDS[0]))

static uint8_t  s_pr_seen_cmd[48], s_pr_seen_src[48];
static uint8_t  s_pr_nseen = 0;
static uint32_t s_pr_idx = 0;             /* position in the sweep */
static uint32_t s_pr_ms  = 0;

static void probe_send(uint8_t cmd, uint8_t dst)
{
    twai_message_t m = {};
    m.extd = 1;
    m.identifier = ((uint32_t)0x0A << 22) | ((uint32_t)cmd << 16) | ((uint32_t)dst << 8) | PROBE_SRC;
    m.data_length_code = 8;                          /* data all zero = read request */
    if (s_can_running) twai_transmit(&m, 0);
}

static void probe_tick(void)
{
    uint32_t now = millis();
    if (now - s_pr_ms < PROBE_PERIOD_MS) return;
    s_pr_ms = now;

    const uint32_t total = N_BCAST + N_MOD * 60u;
    uint32_t i = s_pr_idx;
    if (i < N_BCAST) {
        probe_send(PROBE_BCAST_CMDS[i], 0x3F);
    } else {
        uint32_t k = i - N_BCAST;
        probe_send(PROBE_MOD_CMDS[k / 60u], (uint8_t)(k % 60u));
    }
    if (++s_pr_idx >= total) {                        /* sweep finished → summary */
        s_pr_idx = 0;
        DBG.printf("[PROBE] sweep done. answering (cmd,src): ");
        if (s_pr_nseen == 0) DBG.print("NONE");
        for (uint8_t j = 0; j < s_pr_nseen; j++)
            DBG.printf("(0x%02X,0x%02X) ", s_pr_seen_cmd[j], s_pr_seen_src[j]);
        DBG.println();
    }
}

/* Call for every received frame: report replies addressed to us (dst 0xF0). */
static void probe_rx(const twai_message_t *r)
{
    if (!r->extd) return;
    uint32_t id  = r->identifier;
    uint8_t  cmd = (id >> 16) & 0x3F, dst = (id >> 8) & 0xFF, src = id & 0xFF;
    if (((id >> 22) & 0x0F) != 0x0A || dst != PROBE_SRC || cmd == 0) return;
    for (uint8_t j = 0; j < s_pr_nseen; j++)
        if (s_pr_seen_cmd[j] == cmd && s_pr_seen_src[j] == src) return;   /* already reported */
    if (s_pr_nseen < sizeof(s_pr_seen_cmd)) { s_pr_seen_cmd[s_pr_nseen] = cmd; s_pr_seen_src[s_pr_nseen++] = src; }
    DBG.printf("[PROBE] REPLY cmd=0x%02X src=0x%02X data=", cmd, src);
    for (int i = 0; i < r->data_length_code; i++) DBG.printf("%02X ", r->data[i]);
    DBG.println();
}
#else
static inline void probe_tick(void) {}
static inline void probe_rx(const twai_message_t *) {}
#endif

/* CAN-activity flicker on the onboard LED (min 30 ms between toggles). */
static inline void blink_can(void)
{
    static uint32_t last = 0;
    uint32_t now = millis();
    if (now - last > 30) { last = now; digitalWrite(LED_CAN_PIN, !digitalRead(LED_CAN_PIN)); }
}

/* Heartbeat (~1 Hz) + LCD-link LED (solid while the link is up). Also logs link
 * up/down transitions to the USB console. */
static void update_status_leds(void)
{
    static uint32_t s_hb_ms   = 0;
    static int      s_link_prev = -1;
    uint32_t now = millis();

    if (now - s_hb_ms >= 500) {          /* toggle every 500 ms → ~1 Hz */
        s_hb_ms = now;
        digitalWrite(LED_HB_PIN, !digitalRead(LED_HB_PIN));
    }

    int linked = (s_lcd_last_ms != 0 && (now - s_lcd_last_ms) < LCD_LINK_TIMEOUT_MS) ? 1 : 0;
    digitalWrite(LED_UART_PIN, linked ? HIGH : LOW);
    if (linked != s_link_prev) {
        s_link_prev = linked;
        DBG.println(linked ? "[BRIDGE] LCD link UP" : "[BRIDGE] LCD link DOWN");
    }
}

void setup(void)
{
    pinMode(LED_CAN_PIN,  OUTPUT); digitalWrite(LED_CAN_PIN,  LOW);
    pinMode(LED_UART_PIN, OUTPUT); digitalWrite(LED_UART_PIN, LOW);
    pinMode(LED_HB_PIN,   OUTPUT); digitalWrite(LED_HB_PIN,   LOW);

    DBG.begin(DBG_BAUD);
    delay(200);
    DBG.println("\n=== CAN_Bridge (UART <-> CAN) ===");

    LCD_UART.setRxBufferSize(1024);
    LCD_UART.begin(UART_BAUD, SERIAL_8N1, LCD_UART_RX, LCD_UART_TX);
    DBG.printf("[BRIDGE] LCD UART @ %lu baud (RX=%d TX=%d)\n",
               (unsigned long)UART_BAUD, LCD_UART_RX, LCD_UART_TX);

    s_can_running = can_start();
    if (s_can_running)
        DBG.printf("[BRIDGE] CAN up @ %d kbps on TX=%d RX=%d\n",
                   CAN_BAUD_KBPS, CAN_TX_PIN, CAN_RX_PIN);
    else
        DBG.println("[BRIDGE] CAN init FAILED");

    flow_init();
    DBG.printf("[BRIDGE] flow meter on GPIO%d → CAN 0x%03X @1 Hz\n", FLOW_PIN, FLOW_CAN_ID);

    adc_init();
    DBG.printf("[BRIDGE] MCP3208 NTC on SPI (SCK%d MISO%d MOSI%d CS%d) → CAN 0x%03X/0x%03X @1 Hz\n",
               ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN, NTC_TEMP_ID_0, NTC_TEMP_ID_1);
}

void loop(void)
{
    /* LCD (UART) → CAN */
    twai_message_t m;
    while (uart_recv_frame(&m)) {
        m.rtr = 0;
        esp_err_t r = s_can_running ? twai_transmit(&m, pdMS_TO_TICKS(10))
                                    : ESP_ERR_INVALID_STATE;
        blink_can();
#if DBG_VERBOSE
        DBG.printf("LCD->CAN %s 0x%03X [%u]%s\n", m.extd ? "ext" : "std",
                   (unsigned)m.identifier, m.data_length_code,
                   r == ESP_OK ? "" : "  TX-FAIL");
#else
        (void)r;
#endif
    }

    /* CAN → LCD (UART) */
    twai_message_t r;
    while (twai_receive(&r, 0) == ESP_OK) {
        probe_rx(&r);
        uint8_t f[FRAME_LEN];
        frame_from_msg(&r, f);
        LCD_UART.write(f, FRAME_LEN);
        blink_can();
#if DBG_VERBOSE && !CHG_PROBE
        DBG.printf("CAN->LCD %s 0x%03X [%u]", r.extd ? "ext" : "std",
                   (unsigned)r.identifier, r.data_length_code);
        if (r.extd) {   /* dump payload of charger (extended) frames to decode the protocol */
            for (int i = 0; i < r.data_length_code; i++) DBG.printf(" %02X", r.data[i]);
        }
        DBG.println();
#endif
    }

    /* CAN health: bus-off recovery + periodic status line. */
    can_supervise();
    probe_tick();

    /* Heartbeat to the LCD (~2 Hz) so its top-bar UART chip reads "connected". */
    static uint32_t s_link_hb_ms = 0;
    if (millis() - s_link_hb_ms >= 500) {
        s_link_hb_ms = millis();
        lcd_send_link_heartbeat();
    }

    /* Flow meter → CAN 0x7B0 and MCP3208 NTC temps → 0x7B1/0x7B2 (+ UART mirror). */
    flow_tick();
    adc_tick();

    /* Heartbeat + LCD-link status LEDs. */
    update_status_leds();
} 