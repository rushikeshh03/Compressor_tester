/**
 * ui_fallback.cpp  --  Coulomb Harness Tester Display (800 x 480, 7")
 *
 * Two pages (swipe to navigate; the BLE Connections page was removed — the
 * scanner is USB-HID now):
 *   Page 1 Main  — logo + [Restart system]/"Checking…" slot (left), barcode +
 *                  PASS/FAIL (right), a top-bar row of status chips
 *                  (CAN/IoT/Scanner/Tablet/Uplink/SD), version watermark.
 *   Page 2 Data  — 8 cell voltages + 4 temps + harness-fault line.
 * Swipe LEFT on Main → Data; swipe RIGHT on Data → Main.
 *
 * Widgets only. State is pushed in via the ui_set_ setters and user intent pulled
 * out via ui_take_ — the USB-scanner/CAN backends live in harness_conn.*.
 */

#include "ui/ui.h"
#include "ba100_can.h"
#include "sd_logger.h"       /* sd_logger_present() for the SD status chip */
#include "rawlog.h"          /* live raw-CAN log for the main-page panel     */
#include "hvac_can.h"        /* compressor + charger control/status (page 2) */
#include <stdio.h>
#include <string.h>
#include "esp_ota_ops.h"     /* running partition (app0/app1/factory) for the watermark */

/* Running-partition name (app0 / app1 / factory) for the status watermark. */
static const char *ui_running_partition(void)
{
    const esp_partition_t *r = esp_ota_get_running_partition();
    return (r && r->label[0]) ? r->label : "?";
}

/* ---- Firmware version (set in app_config.h) -------------------------- */
#define FW_VERSION  CONFIG_FW_VERSION

/* ---- Colours (shared palette) --------------------------------------- */
#define C_BG        lv_color_hex(0x0D1117)
#define C_CARD      lv_color_hex(0x161B22)
#define C_CARD2     lv_color_hex(0x1C2333)
#define C_ACCENT    lv_color_hex(0x00C8FF)
#define C_GREEN     lv_color_hex(0x00E676)
#define C_ORANGE    lv_color_hex(0xFFAB40)
#define C_RED       lv_color_hex(0xFF3D3D)
#define C_RED_DIM   lv_color_hex(0x4A0000)
#define C_GREEN_DIM lv_color_hex(0x0A3A20)
#define C_TEXT      lv_color_hex(0xF0F6FC)
#define C_SUBTEXT   lv_color_hex(0x7D8590)
#define C_BORDER    lv_color_hex(0x30363D)
#define C_BTN_BG    lv_color_hex(0x21262D)
/* Bright, vivid PASS/FAIL (result indicator only). */
#define C_PASS      lv_color_hex(0x39FF7A)
#define C_FAIL      lv_color_hex(0xFF3B3B)
#define C_PASS_BG   lv_color_hex(0x0E4D2A)
#define C_FAIL_BG   lv_color_hex(0x5A0A0A)

/* ---- Geometry -------------------------------------------------------- */
#define SCR_W   800
#define SCR_H   480
#define TBAR_H  52
#define GAP     10
#define CARD_W  385
#define COL_L   10
#define COL_R   405
#define ROW_1   (TBAR_H + GAP)
#define COL_H   (SCR_H - ROW_1 - GAP)          /* 408 px */

/* Page 1 left column: logo on top, Check button below. */
#define BTN_AREA_H  120
#define LOGO_H  (COL_H - BTN_AREA_H - GAP)      /* 278 px */
#define BTN_Y   (ROW_1 + LOGO_H + GAP)          /* 350    */

/* Page 1 right column: barcode card on top, big PASS/FAIL below. */
#define BARCODE_H   110
#define RESULT_Y    (ROW_1 + BARCODE_H + GAP)   /* 182 */
#define RESULT_H    (COL_H - BARCODE_H - GAP)   /* 288 */

/* Page 2: two half-width panels + a bottom restart button. */
#define P2_RESTART_H  56
#define P2_CARD_H     (COL_H - P2_RESTART_H - GAP)   /* 342 */
#define P2_RESTART_Y  (ROW_1 + P2_CARD_H + GAP)      /* 414 */

/* ---- Fonts ----------------------------------------------------------- */
#define FONT_TITLE       lv_font_montserrat_24
#define FONT_CARD_TITLE  lv_font_montserrat_24
#define FONT_BARCODE     lv_font_montserrat_32
#define FONT_RESULT      lv_font_montserrat_48
#define FONT_BTN         lv_font_montserrat_32
#define FONT_CHIP        lv_font_montserrat_12
#define FONT_BODY        lv_font_montserrat_16

/* ---- Widget handles: page 1 ----------------------------------------- */
static lv_obj_t *s_scr_main    = nullptr;
static lv_obj_t *s_lbl_status  = nullptr;   /* version | partition | baud watermark */
static lv_obj_t *s_lbl_barcode = nullptr;   /* scanned barcode (main page)          */
static lv_obj_t *s_lbl_result  = nullptr;   /* legacy PASS/FAIL label — unused (kept null) */
static lv_obj_t *s_result_card = nullptr;
static lv_obj_t *s_lbl_rawlog  = nullptr;   /* live raw-CAN log text (main page)  */
static lv_obj_t *s_lbl_rawhdr  = nullptr;   /* "RAW CAN — N frames" header        */
static lv_obj_t *s_lbl_rx_barcode = nullptr;  /* master-echoed barcode (page-3 title bar) */
static lv_obj_t *s_link_lbl[UI_LINK_COUNT] = { nullptr };   /* top-bar icon+text chips */
static ui_conn_t s_link_status[UI_LINK_COUNT] = { UI_CONN_DOWN };  /* last status per link */
static bool      s_blink_on = true;                          /* blink phase for red chips */
#define C_BLINK_DIM lv_color_hex(0x3A0D0D)                   /* dim red (blink-off phase) */
static void apply_chip_color(ui_link_t link);
static void blink_timer_cb(lv_timer_t *t);

/* Page 1 (MAIN): Compressor + Charger control/monitor + barcode */
static lv_obj_t *s_scr_data   = nullptr;   /* page 2: full-screen raw-CAN log */
static lv_obj_t *s_cmp_type   = nullptr;    /* "A"/"B"/"—" badge          */
static lv_obj_t *s_cmp_status = nullptr, *s_cmp_rpm = nullptr,
                *s_cmp_cur = nullptr, *s_cmp_volt = nullptr;
static lv_obj_t *s_cmp_rpm_val = nullptr, *s_cmp_kw_val = nullptr;  /* setpoints */
static lv_obj_t *s_cmp_ss_btn = nullptr,  *s_cmp_ss_lbl = nullptr;  /* START/STOP */
static lv_obj_t *s_chg_type   = nullptr;    /* "XNYB"/"ACDC"/"—" badge    */
static lv_obj_t *s_chg_status = nullptr, *s_chg_volt = nullptr, *s_chg_cur = nullptr;
static lv_obj_t *s_chg_v_val  = nullptr,  *s_chg_a_val = nullptr;   /* setpoints */
static lv_obj_t *s_chg_ss_btn = nullptr,  *s_chg_ss_lbl = nullptr;  /* START/STOP */

/* Page 3: bridge sensors (flow meter + MCP3208 NTC temps) */
static lv_obj_t *s_scr_sens = nullptr;
static lv_obj_t *s_flow_lpm = nullptr, *s_flow_hz = nullptr, *s_flow_tot = nullptr;
static lv_obj_t *s_ntc_lbl[SENS_NTC_COUNT] = { nullptr };

/* ==================================================================
 *  Small helpers
 * ================================================================== */
static lv_obj_t *make_card(lv_obj_t *parent, lv_coord_t x, lv_coord_t y,
                            lv_coord_t w, lv_coord_t h, lv_color_t bg)
{
    lv_obj_t *c = lv_obj_create(parent);
    lv_obj_set_pos(c, x, y);
    lv_obj_set_size(c, w, h);
    lv_obj_set_style_bg_color(c, bg, 0);
    lv_obj_set_style_bg_opa(c, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(c, 1, 0);
    lv_obj_set_style_border_color(c, C_BORDER, 0);
    lv_obj_set_style_radius(c, 10, 0);
    lv_obj_set_style_pad_all(c, 12, 0);
    lv_obj_clear_flag(c, LV_OBJ_FLAG_SCROLLABLE);
    return c;
}

/* Top-bar status chip: "<icon> <text>" in one label. Grey until a link sets its
 * colour (green up / amber busy). Returns the label so ui_set_link() recolours it.
 * Icons for CAN/MQTT are placeholders until those links are wired. */
static lv_obj_t *make_status_chip(lv_obj_t *parent, const char *icon, const char *text)
{
    static char buf[24];
    snprintf(buf, sizeof(buf), "%s %s", icon, text);
    lv_obj_t *lbl = lv_label_create(parent);
    lv_label_set_text(lbl, buf);
    lv_obj_set_style_text_color(lbl, C_SUBTEXT, 0);
    lv_obj_set_style_text_font(lbl, &FONT_BODY, 0);
    return lbl;
}

/* Forward decls for the compressor/charger control widgets (defined below) so
 * build_main_screen() — which now hosts them — can use them. */
static lv_obj_t *ctl_stat(lv_obj_t *card, lv_coord_t y, const char *cap);
static lv_obj_t *ctl_stepper(lv_obj_t *card, lv_coord_t y, const char *cap,
                             lv_event_cb_t minus, lv_event_cb_t plus);
static lv_obj_t *ctl_ss_button(lv_obj_t *card, lv_event_cb_t cb, lv_obj_t **out_lbl);
static lv_obj_t *ctl_badge(lv_obj_t *card);
static void refresh_cmp_setpoints(void); static void refresh_cmp_ss(void);
static void refresh_chg_setpoints(void); static void refresh_chg_ss(void);
static void cb_cmp_rpm_m(lv_event_t *); static void cb_cmp_rpm_p(lv_event_t *);
static void cb_cmp_kw_m(lv_event_t *);  static void cb_cmp_kw_p(lv_event_t *);
static void cb_cmp_ss(lv_event_t *);
static void cb_chg_v_m(lv_event_t *);   static void cb_chg_v_p(lv_event_t *);
static void cb_chg_a_m(lv_event_t *);   static void cb_chg_a_p(lv_event_t *);
static void cb_chg_ss(lv_event_t *);

/* ==================================================================
 *  Navigation — two pages laid out as: [MAIN] · [DATA]
 *    MAIN : swipe LEFT → DATA
 *    DATA : swipe RIGHT → MAIN   (Cell data sits to the right of MAIN)
 * ================================================================== */
enum { PG_MAIN = 0, PG_DATA, PG_SENS };
static int s_page = PG_MAIN;

/* Layout left→right: [Raw CAN (DATA)] · [MAIN control] · [Sensors].
 *   MAIN     swipe RIGHT → Raw CAN,  swipe LEFT → Sensors
 *   Raw CAN  swipe LEFT  → MAIN
 *   Sensors  swipe RIGHT → MAIN                                     */
static void cb_gesture_main(lv_event_t *e)
{
    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) return;
    lv_dir_t d = lv_indev_get_gesture_dir(indev);
    if (d == LV_DIR_RIGHT)     { lv_scr_load_anim(s_scr_data, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 250, 0, false); s_page = PG_DATA; }
    else if (d == LV_DIR_LEFT && s_scr_sens) { lv_scr_load_anim(s_scr_sens, LV_SCR_LOAD_ANIM_MOVE_LEFT, 250, 0, false); s_page = PG_SENS; }
}
static void cb_gesture_data(lv_event_t *e)   /* Raw CAN sits LEFT of main */
{
    lv_indev_t *indev = lv_indev_get_act();
    if (indev && lv_indev_get_gesture_dir(indev) == LV_DIR_LEFT) {
        lv_scr_load_anim(s_scr_main, LV_SCR_LOAD_ANIM_MOVE_LEFT, 250, 0, false); s_page = PG_MAIN;
    }
}
static void cb_gesture_sens(lv_event_t *e)   /* Sensors sits RIGHT of main */
{
    lv_indev_t *indev = lv_indev_get_act();
    if (indev && lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT) {
        lv_scr_load_anim(s_scr_main, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 250, 0, false); s_page = PG_MAIN;
    }
}

/* ==================================================================
 *  build_main_screen()  (page 1: COMPRESSOR + CHARGER control + barcode)
 * ================================================================== */
static void build_main_screen(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    s_scr_main = scr;
    lv_obj_set_style_bg_color(scr, C_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(scr, cb_gesture_main, LV_EVENT_GESTURE, NULL);

    /* Barcode bar across the top (the status-chip bar was removed). The scanned
     * code shows here; font auto-sizes to its length in ui_set_barcode(). */
    lv_obj_t *card_bc = make_card(scr, COL_L, 6, SCR_W - 2 * COL_L, 44, C_CARD);
    lv_obj_set_style_pad_all(card_bc, 4, 0);
    s_lbl_barcode = lv_label_create(card_bc);
    lv_label_set_long_mode(s_lbl_barcode, LV_LABEL_LONG_DOT);
    lv_obj_set_width(s_lbl_barcode, LV_PCT(100));
    lv_obj_set_style_text_align(s_lbl_barcode, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(s_lbl_barcode, "SCAN BARCODE");
    lv_obj_set_style_text_color(s_lbl_barcode, C_ACCENT, 0);
    lv_obj_set_style_text_font(s_lbl_barcode, &FONT_TITLE, 0);
    lv_obj_center(s_lbl_barcode);

    /* Control cards below the barcode bar. */
    const lv_coord_t CY = 58;
    const lv_coord_t CH = SCR_H - CY - GAP;          /* 412 */

    /* ── LEFT card: COMPRESSOR ── */
    lv_obj_t *cc = make_card(scr, COL_L, CY, CARD_W, CH, C_CARD);
    lv_obj_t *ct = lv_label_create(cc);
    lv_label_set_text(ct, "COMPRESSOR");
    lv_obj_set_style_text_color(ct, C_ACCENT, 0);
    lv_obj_set_style_text_font(ct, &FONT_CARD_TITLE, 0);
    lv_obj_align(ct, LV_ALIGN_TOP_LEFT, 2, 0);
    s_cmp_type   = ctl_badge(cc);
    s_cmp_status = ctl_stat(cc,  40, "STATUS");
    s_cmp_rpm    = ctl_stat(cc,  74, "RPM");
    s_cmp_cur    = ctl_stat(cc, 108, "CURRENT (A)");
    s_cmp_volt   = ctl_stat(cc, 142, "VOLTAGE (V)");
    s_cmp_rpm_val = ctl_stepper(cc, 184, "RPM SET", cb_cmp_rpm_m, cb_cmp_rpm_p);
    s_cmp_kw_val  = ctl_stepper(cc, 238, "kW SET",  cb_cmp_kw_m,  cb_cmp_kw_p);
    s_cmp_ss_btn  = ctl_ss_button(cc, cb_cmp_ss, &s_cmp_ss_lbl);

    /* ── RIGHT card: CHARGER ── */
    lv_obj_t *gc = make_card(scr, COL_R, CY, CARD_W, CH, C_CARD);
    lv_obj_t *gt = lv_label_create(gc);
    lv_label_set_text(gt, "CHARGER");
    lv_obj_set_style_text_color(gt, C_ACCENT, 0);
    lv_obj_set_style_text_font(gt, &FONT_CARD_TITLE, 0);
    lv_obj_align(gt, LV_ALIGN_TOP_LEFT, 2, 0);
    s_chg_type   = ctl_badge(gc);
    s_chg_status = ctl_stat(gc,  40, "STATUS");
    s_chg_volt   = ctl_stat(gc,  74, "VOLTAGE (V)");
    s_chg_cur    = ctl_stat(gc, 108, "CURRENT (A)");
    s_chg_v_val  = ctl_stepper(gc, 184, "VOLTS SET", cb_chg_v_m, cb_chg_v_p);
    s_chg_a_val  = ctl_stepper(gc, 238, "AMPS SET",  cb_chg_a_m, cb_chg_a_p);
    s_chg_ss_btn = ctl_ss_button(gc, cb_chg_ss, &s_chg_ss_lbl);

    refresh_cmp_setpoints(); refresh_cmp_ss();
    refresh_chg_setpoints(); refresh_chg_ss();
}

/* ==================================================================
 *  build_data_screen()  (page 2: COMPRESSOR + CHARGER control/monitor)
 * ================================================================== */
/* One "CAPTION ........ value" readout row inside a card. Returns value label. */
static lv_obj_t *ctl_stat(lv_obj_t *card, lv_coord_t y, const char *cap)
{
    lv_obj_t *c = lv_label_create(card);
    lv_label_set_text(c, cap);
    lv_obj_set_style_text_color(c, C_SUBTEXT, 0);
    lv_obj_set_style_text_font(c, &FONT_BODY, 0);
    lv_obj_align(c, LV_ALIGN_TOP_LEFT, 2, y);

    lv_obj_t *v = lv_label_create(card);
    lv_label_set_text(v, "-");
    lv_obj_set_style_text_color(v, C_TEXT, 0);
    lv_obj_set_style_text_font(v, &FONT_CARD_TITLE, 0);
    lv_obj_set_style_text_align(v, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(v, LV_ALIGN_TOP_RIGHT, -2, y - 4);
    return v;
}

/* A "CAP  [-] value [+]" stepper row inside a card. Returns the value label. */
static lv_obj_t *ctl_stepper(lv_obj_t *card, lv_coord_t y, const char *cap,
                             lv_event_cb_t minus, lv_event_cb_t plus)
{
    lv_obj_t *c = lv_label_create(card);
    lv_label_set_text(c, cap);
    lv_obj_set_style_text_color(c, C_SUBTEXT, 0);
    lv_obj_set_style_text_font(c, &FONT_BODY, 0);
    lv_obj_align(c, LV_ALIGN_TOP_LEFT, 2, y + 10);

    lv_obj_t *bm = lv_btn_create(card);
    lv_obj_set_size(bm, 48, 44);
    lv_obj_align(bm, LV_ALIGN_TOP_RIGHT, -128, y);
    lv_obj_set_style_bg_color(bm, C_BTN_BG, 0);
    lv_obj_set_style_radius(bm, 8, 0);
    lv_obj_set_style_shadow_width(bm, 0, 0);
    lv_obj_add_event_cb(bm, minus, LV_EVENT_CLICKED, NULL);
    lv_obj_t *ml = lv_label_create(bm); lv_label_set_text(ml, "-");
    lv_obj_set_style_text_font(ml, &FONT_BTN, 0); lv_obj_center(ml);

    lv_obj_t *v = lv_label_create(card);
    lv_label_set_text(v, "-");
    lv_obj_set_style_text_color(v, C_TEXT, 0);
    lv_obj_set_style_text_font(v, &FONT_CARD_TITLE, 0);
    lv_obj_set_width(v, 72);
    lv_obj_set_style_text_align(v, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(v, LV_ALIGN_TOP_RIGHT, -52, y + 8);

    lv_obj_t *bp = lv_btn_create(card);
    lv_obj_set_size(bp, 48, 44);
    lv_obj_align(bp, LV_ALIGN_TOP_RIGHT, -2, y);
    lv_obj_set_style_bg_color(bp, C_BTN_BG, 0);
    lv_obj_set_style_radius(bp, 8, 0);
    lv_obj_set_style_shadow_width(bp, 0, 0);
    lv_obj_add_event_cb(bp, plus, LV_EVENT_CLICKED, NULL);
    lv_obj_t *pl = lv_label_create(bp); lv_label_set_text(pl, "+");
    lv_obj_set_style_text_font(pl, &FONT_BTN, 0); lv_obj_center(pl);
    return v;
}

static void refresh_cmp_setpoints(void)
{
    char b[16];
    if (s_cmp_rpm_val) { snprintf(b, sizeof(b), "%u", comp_get_rpm()); lv_label_set_text(s_cmp_rpm_val, b); }
    if (s_cmp_kw_val)  { snprintf(b, sizeof(b), "%.1f", comp_get_kw()); lv_label_set_text(s_cmp_kw_val, b); }
}
static void refresh_cmp_ss(void)
{
    if (!s_cmp_ss_btn || !s_cmp_ss_lbl) return;
    bool on = comp_get_on();
    lv_label_set_text(s_cmp_ss_lbl, on ? "STOP" : "START");
    lv_obj_set_style_bg_color(s_cmp_ss_btn, on ? C_FAIL_BG : C_GREEN_DIM, 0);
    lv_obj_set_style_border_color(s_cmp_ss_btn, on ? C_RED : C_GREEN, 0);
}
static void refresh_chg_setpoints(void)
{
    char b[16];
    if (s_chg_v_val) { snprintf(b, sizeof(b), "%.0f", chg_get_volts()); lv_label_set_text(s_chg_v_val, b); }
    if (s_chg_a_val) { snprintf(b, sizeof(b), "%.0f", chg_get_amps());  lv_label_set_text(s_chg_a_val, b); }
}
static void refresh_chg_ss(void)
{
    if (!s_chg_ss_btn || !s_chg_ss_lbl) return;
    bool on = chg_get_on();
    lv_label_set_text(s_chg_ss_lbl, on ? "STOP" : "START");
    lv_obj_set_style_bg_color(s_chg_ss_btn, on ? C_FAIL_BG : C_GREEN_DIM, 0);
    lv_obj_set_style_border_color(s_chg_ss_btn, on ? C_RED : C_GREEN, 0);
}

/* Stepper / button callbacks (drive hvac_can, refresh the labels). */
static void cb_cmp_rpm_m(lv_event_t *) { int r = (int)comp_get_rpm() - (int)COMP_RPM_STEP; comp_set_rpm(r < 0 ? 0 : (uint16_t)r); refresh_cmp_setpoints(); }
static void cb_cmp_rpm_p(lv_event_t *) { comp_set_rpm(comp_get_rpm() + COMP_RPM_STEP); refresh_cmp_setpoints(); }
static void cb_cmp_kw_m(lv_event_t *)  { comp_set_kw(comp_get_kw() - COMP_KW_STEP); refresh_cmp_setpoints(); }
static void cb_cmp_kw_p(lv_event_t *)  { comp_set_kw(comp_get_kw() + COMP_KW_STEP); refresh_cmp_setpoints(); }
static void cb_cmp_ss(lv_event_t *)    { comp_set_on(!comp_get_on()); refresh_cmp_ss(); }
static void cb_chg_v_m(lv_event_t *)   { chg_set_volts(chg_get_volts() - CHG_V_STEP); refresh_chg_setpoints(); }
static void cb_chg_v_p(lv_event_t *)   { chg_set_volts(chg_get_volts() + CHG_V_STEP); refresh_chg_setpoints(); }
static void cb_chg_a_m(lv_event_t *)   { chg_set_amps(chg_get_amps() - CHG_A_STEP); refresh_chg_setpoints(); }
static void cb_chg_a_p(lv_event_t *)   { chg_set_amps(chg_get_amps() + CHG_A_STEP); refresh_chg_setpoints(); }
static void cb_chg_ss(lv_event_t *)    { chg_set_on(!chg_get_on()); refresh_chg_ss(); }

/* A control card's START/STOP button (bottom of the card). */
static lv_obj_t *ctl_ss_button(lv_obj_t *card, lv_event_cb_t cb, lv_obj_t **out_lbl)
{
    lv_obj_t *btn = lv_btn_create(card);
    lv_obj_set_size(btn, LV_PCT(100), 56);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(btn, C_GREEN_DIM, 0);
    lv_obj_set_style_border_color(btn, C_GREEN, 0);
    lv_obj_set_style_border_width(btn, 2, 0);
    lv_obj_set_style_radius(btn, 10, 0);
    lv_obj_set_style_shadow_width(btn, 0, 0);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l = lv_label_create(btn);
    lv_label_set_text(l, "START");
    lv_obj_set_style_text_color(l, C_TEXT, 0);
    lv_obj_set_style_text_font(l, &FONT_BTN, 0);
    lv_obj_center(l);
    if (out_lbl) *out_lbl = l;
    return btn;
}

/* Small type badge in a card's title row (right side). */
static lv_obj_t *ctl_badge(lv_obj_t *card)
{
    lv_obj_t *b = lv_label_create(card);
    lv_label_set_text(b, "\xE2\x80\x94");   /* em dash */
    lv_obj_set_style_text_color(b, C_SUBTEXT, 0);
    lv_obj_set_style_text_font(b, &FONT_BODY, 0);
    lv_obj_align(b, LV_ALIGN_TOP_RIGHT, -2, 2);
    return b;
}

/* Page 2: full-screen live RAW-CAN log (every frame since the last barcode). */
static void build_data_screen(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    s_scr_data = scr;
    lv_obj_set_style_bg_color(scr, C_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(scr, cb_gesture_data, LV_EVENT_GESTURE, NULL);

    /* Title bar: title (left) + frame counter (right). */
    lv_obj_t *tbar = lv_obj_create(scr);
    lv_obj_set_size(tbar, SCR_W, TBAR_H);
    lv_obj_set_pos(tbar, 0, 0);
    lv_obj_set_style_bg_opa(tbar, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(tbar, 0, 0);
    lv_obj_clear_flag(tbar, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(tbar);
    lv_label_set_text(title, "Raw CAN  " LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_color(title, C_TEXT, 0);
    lv_obj_set_style_text_font(title, &FONT_TITLE, 0);
    lv_obj_align(title, LV_ALIGN_LEFT_MID, 14, 0);

    s_lbl_rawhdr = lv_label_create(tbar);
    lv_label_set_text(s_lbl_rawhdr, "0 frames");
    lv_obj_set_style_text_color(s_lbl_rawhdr, C_ACCENT, 0);
    lv_obj_set_style_text_font(s_lbl_rawhdr, &FONT_BODY, 0);
    lv_obj_align(s_lbl_rawhdr, LV_ALIGN_RIGHT_MID, -14, 0);

    /* Full-screen raw-CAN log card. */
    lv_obj_t *card = make_card(scr, COL_L, ROW_1, SCR_W - 2 * COL_L, COL_H, C_CARD);
    lv_obj_set_style_pad_all(card, 10, 0);
    s_lbl_rawlog = lv_label_create(card);
    lv_label_set_long_mode(s_lbl_rawlog, LV_LABEL_LONG_CLIP);   /* no wrap; long lines clip */
    lv_obj_set_width(s_lbl_rawlog, LV_PCT(100));
    lv_obj_set_style_text_color(s_lbl_rawlog, C_GREEN, 0);
    lv_obj_set_style_text_font(s_lbl_rawlog, &FONT_BODY, 0);
    lv_label_set_text(s_lbl_rawlog, "(waiting for CAN frames…)");
    lv_obj_align(s_lbl_rawlog, LV_ALIGN_TOP_LEFT, 0, 0);

    /* Version | partition | baud watermark (bottom-right). */
    s_lbl_status = lv_label_create(scr);
    lv_label_set_text(s_lbl_status, FW_VERSION);
    lv_obj_set_style_text_color(s_lbl_status, C_SUBTEXT, 0);
    lv_obj_set_style_text_font(s_lbl_status, &FONT_CHIP, 0);
    lv_obj_set_style_text_align(s_lbl_status, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(s_lbl_status, LV_ALIGN_BOTTOM_RIGHT, -6, -4);
    lv_obj_move_foreground(s_lbl_status);
}

/* ==================================================================
 *  build_sensors_screen()  (page 3: flow meter + MCP3208 NTC temps)
 * ================================================================== */
/* A caption-over-value tile. Returns the value label. */
static lv_obj_t *sens_tile(lv_obj_t *parent, lv_coord_t x, lv_coord_t y,
                           lv_coord_t w, lv_coord_t h, const char *cap)
{
    lv_obj_t *c = make_card(parent, x, y, w, h, C_CARD);
    lv_obj_set_style_pad_all(c, 6, 0);
    lv_obj_t *l = lv_label_create(c);
    lv_label_set_text(l, cap);
    lv_obj_set_style_text_color(l, C_SUBTEXT, 0);
    lv_obj_set_style_text_font(l, &FONT_CHIP, 0);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 2, 0);
    lv_obj_t *v = lv_label_create(c);
    lv_label_set_text(v, "-");
    lv_obj_set_style_text_color(v, C_TEXT, 0);
    lv_obj_set_style_text_font(v, &FONT_CARD_TITLE, 0);
    lv_obj_align(v, LV_ALIGN_BOTTOM_LEFT, 2, -2);
    return v;
}

static void build_sensors_screen(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    s_scr_sens = scr;
    lv_obj_set_style_bg_color(scr, C_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(scr, cb_gesture_sens, LV_EVENT_GESTURE, NULL);

    lv_obj_t *tbar = lv_obj_create(scr);
    lv_obj_set_size(tbar, SCR_W, TBAR_H);
    lv_obj_set_pos(tbar, 0, 0);
    lv_obj_set_style_bg_opa(tbar, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(tbar, 0, 0);
    lv_obj_clear_flag(tbar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *title = lv_label_create(tbar);
    lv_label_set_text(title, LV_SYMBOL_LEFT "  Sensors");
    lv_obj_set_style_text_color(title, C_TEXT, 0);
    lv_obj_set_style_text_font(title, &FONT_TITLE, 0);
    lv_obj_align(title, LV_ALIGN_LEFT_MID, 14, 0);

    const lv_coord_t GX = 10;
    lv_coord_t y = ROW_1;

    /* FLOW row: L/min, Hz, total */
    lv_obj_t *cf = lv_label_create(scr);
    lv_label_set_text(cf, "FLOW METER");
    lv_obj_set_style_text_color(cf, C_SUBTEXT, 0);
    lv_obj_set_style_text_font(cf, &FONT_BODY, 0);
    lv_obj_set_pos(cf, GX, y);
    y += 26;
    const lv_coord_t FW = (SCR_W - 2 * GX - 2 * GAP) / 3;   /* 253 */
    s_flow_lpm = sens_tile(scr, GX,                 y, FW, 92, "FLOW (L/min)");
    s_flow_hz  = sens_tile(scr, GX + FW + GAP,      y, FW, 92, "PULSE (Hz)");
    s_flow_tot = sens_tile(scr, GX + 2*(FW + GAP),  y, FW, 92, "TOTAL (L)");
    y += 92 + 12;

    /* NTC row: 8 tiles, 4 cols × 2 rows */
    lv_obj_t *cn = lv_label_create(scr);
    lv_label_set_text(cn, "NTC TEMPERATURES (\xC2\xB0""C)");
    lv_obj_set_style_text_color(cn, C_SUBTEXT, 0);
    lv_obj_set_style_text_font(cn, &FONT_BODY, 0);
    lv_obj_set_pos(cn, GX, y);
    y += 26;
    const lv_coord_t TW = (SCR_W - 2 * GX - 3 * GAP) / 4;   /* 187 */
    const lv_coord_t TH = 84;
    for (int i = 0; i < SENS_NTC_COUNT; i++) {
        int col = i % 4, row = i / 4;
        char cap[12]; snprintf(cap, sizeof(cap), "NTC %d", i + 1);
        s_ntc_lbl[i] = sens_tile(scr, GX + col * (TW + GAP), y + row * (TH + GAP), TW, TH, cap);
    }
}

/* ==================================================================
 *  ui_init()
 * ================================================================== */
void ui_init(void)
{
    build_main_screen();
    build_data_screen();
    build_sensors_screen();
    lv_scr_load(s_scr_main);
    s_page = PG_MAIN;
    lv_timer_create(blink_timer_cb, 450, NULL);   /* blink not-connected chips red */
}

/* ==================================================================
 *  Page 1 API
 * ================================================================== */
void ui_set_barcode(const char *code)
{
    if (!s_lbl_barcode) return;
    const char *txt = (code && code[0]) ? code : "-";
    /* No caption now — the code fills the whole card, so auto-size the font to
     * its length (largest that still fits one line) and keep it centered. */
    size_t n = strlen(txt);
    const lv_font_t *f = (n <=  8) ? &lv_font_montserrat_48
                       : (n <= 14) ? &lv_font_montserrat_32
                       : (n <= 20) ? &lv_font_montserrat_24
                       : (n <= 28) ? &lv_font_montserrat_16
                       :             &lv_font_montserrat_12;
    lv_obj_set_style_text_font(s_lbl_barcode, f, 0);
    lv_label_set_text(s_lbl_barcode, txt);
    lv_obj_center(s_lbl_barcode);
}

void ui_set_result(ui_result_t result)
{
    if (!s_lbl_result || !s_result_card) return;
    switch (result) {
        case UI_RESULT_PASS:
            lv_label_set_text(s_lbl_result, "PASS");
            lv_obj_set_style_text_color(s_lbl_result, C_PASS, 0);
            lv_obj_set_style_bg_color(s_result_card, C_PASS_BG, 0);
            lv_obj_set_style_border_color(s_result_card, C_PASS, 0);
            lv_obj_set_style_border_width(s_result_card, 3, 0);
            ui_set_checking(false);
            break;
        case UI_RESULT_FAIL:
            lv_label_set_text(s_lbl_result, "FAIL");
            lv_obj_set_style_text_color(s_lbl_result, C_FAIL, 0);
            lv_obj_set_style_bg_color(s_result_card, C_FAIL_BG, 0);
            lv_obj_set_style_border_color(s_result_card, C_FAIL, 0);
            lv_obj_set_style_border_width(s_result_card, 3, 0);
            ui_set_checking(false);
            break;
        case UI_RESULT_MISMATCH:
            lv_label_set_text(s_lbl_result, "MISMATCH");
            lv_obj_set_style_text_color(s_lbl_result, C_ORANGE, 0);
            lv_obj_set_style_bg_color(s_result_card, lv_color_hex(0x4A2E00), 0);
            lv_obj_set_style_border_color(s_result_card, C_ORANGE, 0);
            lv_obj_set_style_border_width(s_result_card, 3, 0);
            ui_set_checking(false);
            break;
        case UI_RESULT_NONE:
        default:
            lv_label_set_text(s_lbl_result, "-");
            lv_obj_set_style_text_color(s_lbl_result, C_SUBTEXT, 0);
            lv_obj_set_style_bg_color(s_result_card, C_CARD, 0);
            lv_obj_set_style_border_color(s_result_card, C_BORDER, 0);
            lv_obj_set_style_border_width(s_result_card, 1, 0);
            break;
    }
    lv_obj_center(s_lbl_result);
}

void ui_set_checking(bool checking)
{
    (void)checking;   /* no "Checking…" slot on this station */
}

void ui_set_rx_barcode(const char *code)
{
    if (!s_lbl_rx_barcode) return;
    static char b[48];
    snprintf(b, sizeof(b), "Master: %s", (code && code[0]) ? code : "-");
    lv_label_set_text(s_lbl_rx_barcode, b);
}

void ui_set_rawlog(const char *text)
{
    if (!s_lbl_rawlog) return;
    lv_label_set_text(s_lbl_rawlog, (text && text[0]) ? text : "(waiting for CAN frames…)");
}

/* ── "Please connect …" popup (main page, auto-hide 5 s) ─────────────────── */
static lv_obj_t   *s_popup       = nullptr;
static lv_obj_t   *s_popup_lbl   = nullptr;
static lv_timer_t *s_popup_timer = nullptr;

static void popup_timer_cb(lv_timer_t *t)
{
    if (s_popup) lv_obj_add_flag(s_popup, LV_OBJ_FLAG_HIDDEN);
    lv_timer_pause(t);
}

void ui_show_connect_popup(const char *missing)
{
    if (!s_scr_main) return;
    if (!s_popup) {
        s_popup = lv_obj_create(s_scr_main);
        lv_obj_set_size(s_popup, 520, 180);
        lv_obj_center(s_popup);
        lv_obj_set_style_bg_color(s_popup, lv_color_hex(0x2A1A00), 0);
        lv_obj_set_style_bg_opa(s_popup, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(s_popup, C_ORANGE, 0);
        lv_obj_set_style_border_width(s_popup, 3, 0);
        lv_obj_set_style_radius(s_popup, 12, 0);
        lv_obj_clear_flag(s_popup, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *icon = lv_label_create(s_popup);
        lv_label_set_text(icon, LV_SYMBOL_WARNING);
        lv_obj_set_style_text_color(icon, C_ORANGE, 0);
        lv_obj_set_style_text_font(icon, &FONT_TITLE, 0);
        lv_obj_align(icon, LV_ALIGN_TOP_MID, 0, 0);

        s_popup_lbl = lv_label_create(s_popup);
        lv_label_set_long_mode(s_popup_lbl, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(s_popup_lbl, 480);
        lv_obj_set_style_text_color(s_popup_lbl, C_TEXT, 0);
        lv_obj_set_style_text_font(s_popup_lbl, &FONT_CARD_TITLE, 0);
        lv_obj_set_style_text_align(s_popup_lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(s_popup_lbl, LV_ALIGN_CENTER, 0, 12);

        s_popup_timer = lv_timer_create(popup_timer_cb, 5000, NULL);
        lv_timer_pause(s_popup_timer);
    }
    static char b[96];
    snprintf(b, sizeof(b), "Please connect %s,\nthen scan again.",
             (missing && missing[0]) ? missing : "the device");
    lv_label_set_text(s_popup_lbl, b);
    lv_obj_clear_flag(s_popup, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_popup);
    if (s_popup_timer) { lv_timer_reset(s_popup_timer); lv_timer_resume(s_popup_timer); }
}

/* ── FAIL lock-out popup (main page, persistent, gesture-transparent) ──────────
 *  A child of the MAIN screen (so it only shows there), non-modal, with
 *  LV_OBJ_FLAG_GESTURE_BUBBLE on every part so swipe gestures bubble up to the
 *  screen's page-nav handler — the user can still switch pages while it's up. */
static lv_obj_t     *s_fail_pop     = nullptr;
static lv_obj_t     *s_fail_title   = nullptr;
static lv_obj_t     *s_fail_msg     = nullptr;
static lv_obj_t     *s_fail_btn_ovr = nullptr;   /* Override (rescan mode only) */
static volatile bool s_fail_close_req    = false;
static volatile bool s_fail_override_req = false;

static void cb_fail_close(lv_event_t *e)    { (void)e; s_fail_close_req    = true; }
static void cb_fail_override(lv_event_t *e) { (void)e; s_fail_override_req = true; }

static void fail_pop_build(void)
{
    s_fail_pop = lv_obj_create(s_scr_main);
    lv_obj_set_size(s_fail_pop, 560, 250);
    lv_obj_center(s_fail_pop);
    lv_obj_set_style_bg_color(s_fail_pop, C_FAIL_BG, 0);
    lv_obj_set_style_bg_opa(s_fail_pop, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(s_fail_pop, C_RED, 0);
    lv_obj_set_style_border_width(s_fail_pop, 3, 0);
    lv_obj_set_style_radius(s_fail_pop, 12, 0);
    lv_obj_clear_flag(s_fail_pop, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_fail_pop, LV_OBJ_FLAG_GESTURE_BUBBLE);

    s_fail_title = lv_label_create(s_fail_pop);
    lv_obj_set_style_text_color(s_fail_title, C_FAIL, 0);
    lv_obj_set_style_text_font(s_fail_title, &FONT_TITLE, 0);
    lv_obj_align(s_fail_title, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_add_flag(s_fail_title, LV_OBJ_FLAG_GESTURE_BUBBLE);

    s_fail_msg = lv_label_create(s_fail_pop);
    lv_label_set_long_mode(s_fail_msg, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(s_fail_msg, 520);
    lv_obj_set_style_text_color(s_fail_msg, C_TEXT, 0);
    lv_obj_set_style_text_font(s_fail_msg, &FONT_BODY, 0);
    lv_obj_set_style_text_align(s_fail_msg, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_fail_msg, LV_ALIGN_CENTER, 0, -6);
    lv_obj_add_flag(s_fail_msg, LV_OBJ_FLAG_GESTURE_BUBBLE);

    /* Override button (bottom-left) — shown only in rescan mode. */
    s_fail_btn_ovr = lv_btn_create(s_fail_pop);
    lv_obj_set_size(s_fail_btn_ovr, 180, 54);
    lv_obj_align(s_fail_btn_ovr, LV_ALIGN_BOTTOM_LEFT, 6, -6);
    lv_obj_set_style_bg_color(s_fail_btn_ovr, C_ORANGE, 0);
    lv_obj_set_style_radius(s_fail_btn_ovr, 8, 0);
    lv_obj_set_style_shadow_width(s_fail_btn_ovr, 0, 0);
    lv_obj_add_flag(s_fail_btn_ovr, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_add_event_cb(s_fail_btn_ovr, cb_fail_override, LV_EVENT_CLICKED, NULL);
    lv_obj_t *ovl = lv_label_create(s_fail_btn_ovr);
    lv_label_set_text(ovl, LV_SYMBOL_OK "  Override");
    lv_obj_set_style_text_color(ovl, C_BG, 0);
    lv_obj_set_style_text_font(ovl, &FONT_BTN, 0);
    lv_obj_center(ovl);

    /* Close button (bottom-right). */
    lv_obj_t *btn_close = lv_btn_create(s_fail_pop);
    lv_obj_set_size(btn_close, 180, 54);
    lv_obj_align(btn_close, LV_ALIGN_BOTTOM_RIGHT, -6, -6);
    lv_obj_set_style_bg_color(btn_close, C_BTN_BG, 0);
    lv_obj_set_style_bg_color(btn_close, C_RED_DIM, LV_STATE_PRESSED);
    lv_obj_set_style_border_color(btn_close, C_RED, 0);
    lv_obj_set_style_border_width(btn_close, 1, 0);
    lv_obj_set_style_radius(btn_close, 8, 0);
    lv_obj_set_style_shadow_width(btn_close, 0, 0);
    lv_obj_add_flag(btn_close, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_add_event_cb(btn_close, cb_fail_close, LV_EVENT_CLICKED, NULL);
    lv_obj_t *cl = lv_label_create(btn_close);
    lv_label_set_text(cl, LV_SYMBOL_CLOSE "  Close");
    lv_obj_set_style_text_color(cl, C_TEXT, 0);
    lv_obj_set_style_text_font(cl, &FONT_BTN, 0);
    lv_obj_center(cl);

    lv_obj_add_flag(s_fail_pop, LV_OBJ_FLAG_HIDDEN);
}

static void fail_pop_show(const char *title, const char *msg, bool with_override)
{
    if (!s_scr_main) return;
    if (!s_fail_pop) fail_pop_build();
    lv_label_set_text(s_fail_title, title);
    lv_label_set_text(s_fail_msg, msg);
    if (with_override) lv_obj_clear_flag(s_fail_btn_ovr, LV_OBJ_FLAG_HIDDEN);
    else               lv_obj_add_flag(s_fail_btn_ovr, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_fail_pop, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_fail_pop);
}

void ui_fail_popup_fail(const char *barcode)
{
    static char m[160];
    snprintf(m, sizeof(m), "Re-scan the same harness\n(%s) to retest.",
             (barcode && barcode[0]) ? barcode : "-");
    fail_pop_show(LV_SYMBOL_WARNING "  HARNESS FAILED", m, false);
}

void ui_fail_popup_rescan(const char *barcode)
{
    static char m[200];
    snprintf(m, sizeof(m), "Wrong harness. Re-scan the failed one\n(%s), or Override to test a new one.",
             (barcode && barcode[0]) ? barcode : "-");
    fail_pop_show(LV_SYMBOL_WARNING "  WRONG HARNESS", m, true);
}

void ui_fail_popup_hide(void)
{
    if (s_fail_pop) lv_obj_add_flag(s_fail_pop, LV_OBJ_FLAG_HIDDEN);
}

bool ui_take_fail_close(void)    { bool r = s_fail_close_req;    s_fail_close_req    = false; return r; }
bool ui_take_fail_override(void) { bool r = s_fail_override_req; s_fail_override_req = false; return r; }

/* ==================================================================
 *  Top-bar chips
 * ================================================================== */
/* Connected → solid green; not connected → blink red (phase from the timer).
 * Exception: the IoT chip BLINKS green while up (a visible "IoT is alive" pulse)
 * and sits steady dim-grey when down. */
static void apply_chip_color(ui_link_t link)
{
    if (!s_link_lbl[link]) return;
    lv_color_t c;
    if (link == UI_LINK_IOT) {
        c = (s_link_status[link] == UI_CONN_UP) ? (s_blink_on ? C_GREEN : C_BLINK_DIM)
                                                : C_SUBTEXT;
    } else {
        c = (s_link_status[link] == UI_CONN_UP) ? C_GREEN
                                                : (s_blink_on ? C_FAIL : C_BLINK_DIM);
    }
    lv_obj_set_style_text_color(s_link_lbl[link], c, 0);
}

/* ~2.2 Hz blink for every not-connected chip. */
static void blink_timer_cb(lv_timer_t *)
{
    s_blink_on = !s_blink_on;
    for (int i = 0; i < UI_LINK_COUNT; i++) {
        if (!s_link_lbl[i]) continue;
        /* Re-apply chips that are blinking: any not-up chip, plus the IoT chip
         * which blinks while UP. */
        if (s_link_status[i] != UI_CONN_UP || i == UI_LINK_IOT) apply_chip_color((ui_link_t)i);
    }
}

void ui_set_link(ui_link_t link, ui_conn_t status)
{
    if (link < 0 || link >= UI_LINK_COUNT || !s_link_lbl[link]) return;
    s_link_status[link] = status;
    apply_chip_color(link);
}

/* ==================================================================
 *  Page 1 intent pollers
 * ================================================================== */
bool ui_take_restart_system_request(void)
{
    return false;   /* "Restart system" removed on this station */
}

/* ==================================================================
 *  ui_apply_state()  -- ~25 Hz from main.cpp, inside lvgl_port_lock
 *  Drives the CAN status chip + the version/partition/baud watermark.
 * ================================================================== */
void ui_apply_state(const BA100_State_t &s)
{
    static char buf[64];

    ui_set_link(UI_LINK_CAN,  s.can_alive    ? UI_CONN_UP : UI_CONN_DOWN);
    ui_set_link(UI_LINK_UART, s.bridge_alive ? UI_CONN_UP : UI_CONN_DOWN);
    ui_set_link(UI_LINK_SD,   sd_logger_present() ? UI_CONN_UP : UI_CONN_DOWN);

    /* ── Live raw-CAN log (main page). Re-render only when a frame arrived or the
     * log was reset (rawlog_seq changes), so a quiet bus costs nothing. Shows the
     * most recent frames since the last barcode; the SD file keeps the full run. */
    if (s_lbl_rawlog) {
        static uint32_t s_last_seq = 0xFFFFFFFF;
        uint32_t seq = rawlog_seq();
        if (seq != s_last_seq) {
            s_last_seq = seq;
            static char logbuf[1600];
            rawlog_render(logbuf, sizeof(logbuf), 20);   /* full-screen page fits ~20 lines */
            lv_label_set_text(s_lbl_rawlog, logbuf[0] ? logbuf : "(waiting for CAN frames…)");
            if (s_lbl_rawhdr) {
                snprintf(buf, sizeof(buf), "%lu frames", (unsigned long)rawlog_total());
                lv_label_set_text(s_lbl_rawhdr, buf);
            }
        }
    }

    if (s_lbl_status) {
        uint32_t baud = ba100_get_baud_kbps();
        static uint32_t s_last_baud = 0xFFFFFFFF;
        if (baud != s_last_baud) {
            s_last_baud = baud;
            snprintf(buf, sizeof(buf), "%s | %s | %luk",
                     FW_VERSION, ui_running_partition(), (unsigned long)baud);
            lv_label_set_text(s_lbl_status, buf);
            lv_obj_align(s_lbl_status, LV_ALIGN_BOTTOM_RIGHT, -6, -4);   /* main-screen watermark */
        }
    }

    /* ── Page 2: COMPRESSOR + CHARGER live status (from hvac_can) ──────────── */
    comp_state_t cm = comp_get();
    if (s_cmp_type) {
        const char *t = cm.variant == COMP_T_A ? "238/A" : cm.variant == COMP_T_B ? "34C/B" : "\xE2\x80\x94";
        lv_label_set_text(s_cmp_type, t);
        lv_obj_set_style_text_color(s_cmp_type, cm.online ? C_GREEN : C_SUBTEXT, 0);
    }
    if (s_cmp_status) {
        const char *st = !cm.online ? "OFFLINE" : cm.status == 1 ? "ON" : cm.status == 3 ? "ERROR" : "OFF";
        lv_label_set_text(s_cmp_status, st);
        lv_obj_set_style_text_color(s_cmp_status,
            !cm.online ? C_SUBTEXT : cm.status == 1 ? C_GREEN : cm.status == 3 ? C_RED : C_TEXT, 0);
    }
    if (s_cmp_rpm)  { if (cm.online) snprintf(buf, sizeof(buf), "%u", cm.rpm); else strcpy(buf, "-"); lv_label_set_text(s_cmp_rpm, buf); }
    if (s_cmp_cur)  { if (cm.online) snprintf(buf, sizeof(buf), "%.1f", cm.current_a); else strcpy(buf, "-"); lv_label_set_text(s_cmp_cur, buf); }
    if (s_cmp_volt) { if (cm.online) snprintf(buf, sizeof(buf), "%u", cm.voltage_v); else strcpy(buf, "-"); lv_label_set_text(s_cmp_volt, buf); }

    chg_state_t cg = chg_get();
    if (s_chg_type) {
        const char *t = cg.proto == CHG_P_XNYB ? "XNYB" : cg.proto == CHG_P_ACDC ? "ACDC" : "\xE2\x80\x94";
        lv_label_set_text(s_chg_type, t);
        lv_obj_set_style_text_color(s_chg_type, cg.online ? C_GREEN : C_SUBTEXT, 0);
    }
    if (s_chg_status) {
        const char *st = !cg.online ? "OFFLINE" : cg.on ? "CHARGING" : "STANDBY";
        lv_label_set_text(s_chg_status, st);
        lv_obj_set_style_text_color(s_chg_status, !cg.online ? C_SUBTEXT : cg.on ? C_GREEN : C_TEXT, 0);
    }
    if (s_chg_volt) { if (cg.online) snprintf(buf, sizeof(buf), "%.1f", cg.volts); else strcpy(buf, "-"); lv_label_set_text(s_chg_volt, buf); }
    if (s_chg_cur)  { if (cg.online) snprintf(buf, sizeof(buf), "%.1f", cg.amps);  else strcpy(buf, "-"); lv_label_set_text(s_chg_cur, buf); }

    /* ── Page 3: bridge sensors (flow meter + MCP3208 NTC temps) ───────────── */
    sens_state_t sn = sens_get();
    if (s_flow_lpm) { if (sn.flow_online) snprintf(buf, sizeof(buf), "%.1f", sn.flow_lpm_x10 / 10.0f); else strcpy(buf, "-"); lv_label_set_text(s_flow_lpm, buf); }
    if (s_flow_hz)  { if (sn.flow_online) snprintf(buf, sizeof(buf), "%u", sn.flow_hz); else strcpy(buf, "-"); lv_label_set_text(s_flow_hz, buf); }
    if (s_flow_tot) { if (sn.flow_online) snprintf(buf, sizeof(buf), "%.1f", sn.flow_total_x10 / 10.0f); else strcpy(buf, "-"); lv_label_set_text(s_flow_tot, buf); }
    for (int i = 0; i < SENS_NTC_COUNT; i++) {
        if (!s_ntc_lbl[i]) continue;
        if (sn.ntc_online && sn.ntc_x10[i] != SENS_NTC_INVALID)
            snprintf(buf, sizeof(buf), "%.1f", sn.ntc_x10[i] / 10.0f);
        else
            strcpy(buf, "-");
        lv_label_set_text(s_ntc_lbl[i], buf);
    }
}
