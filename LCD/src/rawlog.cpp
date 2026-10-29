/**
 * rawlog.cpp  --  raw-CAN log ring buffer (see rawlog.h).
 */
#include "rawlog.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#define RAWLOG_N   512      /* frames kept in RAM (~9 KB) */

typedef struct {
    uint32_t ms;
    uint32_t id;
    uint8_t  dir;    /* 1 = TX, 0 = RX */
    uint8_t  ext;    /* 1 = 29-bit id  */
    uint8_t  dlc;
    uint8_t  data[8];
} rawlog_e;

static rawlog_e s_ring[RAWLOG_N];
static uint16_t s_head  = 0;    /* next write slot */
static uint16_t s_count = 0;    /* filled entries  */
static uint32_t s_seq   = 0;
static uint32_t s_total = 0;

void rawlog_push(bool tx, uint32_t id, bool ext, const uint8_t *data, uint8_t dlc)
{
    rawlog_e *e = &s_ring[s_head];
    e->ms  = millis();
    e->id  = id;
    e->dir = tx ? 1 : 0;
    e->ext = ext ? 1 : 0;
    e->dlc = dlc > 8 ? 8 : dlc;
    for (uint8_t i = 0; i < 8; i++) e->data[i] = (data && i < e->dlc) ? data[i] : 0;
    s_head = (uint16_t)((s_head + 1) % RAWLOG_N);
    if (s_count < RAWLOG_N) s_count++;
    s_total++;
    s_seq++;
}

void rawlog_reset(void)
{
    s_head = 0;
    s_count = 0;
    s_total = 0;
    s_seq++;
}

uint32_t rawlog_seq(void)   { return s_seq; }
uint32_t rawlog_total(void) { return s_total; }

size_t rawlog_render(char *out, size_t cap, int max_lines)
{
    if (!out || cap == 0) return 0;
    out[0] = '\0';
    if (max_lines < 1) max_lines = 1;

    int show = s_count;
    if (show > max_lines) show = max_lines;

    size_t n = 0;
    for (int i = 0; i < show && n < cap; i++) {
        int idx = ((int)s_head - show + i + 2 * RAWLOG_N) % RAWLOG_N;
        const rawlog_e *e = &s_ring[idx];
        n += snprintf(out + n, cap - n, "%c ", e->dir ? 'T' : 'R');
        if (n < cap)
            n += snprintf(out + n, cap - n, e->ext ? "%08lX [%u]" : "%03lX [%u]",
                          (unsigned long)e->id, (unsigned)e->dlc);
        for (uint8_t b = 0; b < e->dlc && n < cap; b++)
            n += snprintf(out + n, cap - n, " %02X", e->data[b]);
        if (n < cap && i < show - 1) n += snprintf(out + n, cap - n, "\n");
    }
    return n;
}
