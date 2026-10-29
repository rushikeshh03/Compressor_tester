/**
 * lv_conf.h  –  LVGL v8 configuration
 * Place this file in your project's  include/  folder
 * so the -DLV_CONF_INCLUDE_SIMPLE flag can find it.
 */
#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/* ── colour depth ───────────────────────────────────────── */
#define LV_COLOR_DEPTH  16          /* RGB565, matches ST7262 */

/* ── memory ─────────────────────────────────────────────── */
#define LV_MEM_CUSTOM   1
#if LV_MEM_CUSTOM
  #define LV_MEM_CUSTOM_INCLUDE <stdlib.h>
  #define LV_MEM_CUSTOM_ALLOC   malloc
  #define LV_MEM_CUSTOM_FREE    free
  #define LV_MEM_CUSTOM_REALLOC realloc
#endif

/* ── HAL ─────────────────────────────────────────────────── */
#define LV_TICK_CUSTOM            1
#define LV_TICK_CUSTOM_INCLUDE    <Arduino.h>
#define LV_TICK_CUSTOM_SYS_TIME_EXPR  (millis())

/* ── display driver ──────────────────────────────────────── */
#define LV_HOR_RES_MAX   800
#define LV_VER_RES_MAX   480

/* ── fonts ───────────────────────────────────────────────── */
#define LV_FONT_MONTSERRAT_12  1
#define LV_FONT_MONTSERRAT_16  1
#define LV_FONT_MONTSERRAT_24  1
#define LV_FONT_MONTSERRAT_32  1
#define LV_FONT_MONTSERRAT_48  1
#define LV_FONT_DEFAULT        &lv_font_montserrat_16

/* ── widgets needed ──────────────────────────────────────── */
#define LV_USE_ARC      1
#define LV_USE_BAR      1
#define LV_USE_BTN      1
#define LV_USE_LABEL    1
#define LV_USE_IMG      1
#define LV_USE_LINE     1
#define LV_USE_METER    1
#define LV_USE_MSGBOX   1
#define LV_USE_TABVIEW  1
#define LV_USE_LIST     1
#define LV_USE_SWITCH   1

/* ── logging ─────────────────────────────────────────────── */
#define LV_USE_LOG    1
#define LV_LOG_LEVEL  LV_LOG_LEVEL_WARN
#define LV_LOG_PRINTF 1

/* ── GPU / DMA2D ──────────────────────────────────────────── */
#define LV_USE_GPU_SDL  0

/* ── SquareLine Studio compatibility ─────────────────────── */
#define LV_USE_SNAPSHOT 0
#define LV_USE_MONKEY   0
#define LV_USE_GRIDNAV  0
#define LV_USE_FRAGMENT 0

#endif /* LV_CONF_H */