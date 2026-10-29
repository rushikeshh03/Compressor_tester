#pragma once
/**
 * rawlog.h  --  in-RAM raw-CAN log ring buffer for the LCD live view.
 *
 * Every CAN frame the LCD sees (RX from the bridge) or sends (TX to the bridge)
 * is pushed here by ba100_can.*. The UI renders the most recent frames on the
 * main page. rawlog_reset() is called on each barcode scan so the on-screen log
 * shows only the traffic since the last barcode (the SD per-scan file keeps the
 * full capture). Single-threaded: all pushes + the UI render run on the loop
 * task, so no locking is needed.
 */
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Record one frame. tx=true for frames we transmit; ext=true for 29-bit IDs. */
void     rawlog_push(bool tx, uint32_t id, bool ext, const uint8_t *data, uint8_t dlc);

/* Clear the ring (new barcode session). Bumps the sequence too. */
void     rawlog_reset(void);

/* Monotonic change counter — bumps on every push and reset. The UI re-renders
 * only when this changes, so a quiet bus costs nothing. */
uint32_t rawlog_seq(void);

/* Frames pushed since the last reset (for a "N frames" header). */
uint32_t rawlog_total(void);

/* Format the last `max_lines` frames (oldest first, newest last) into `out` as
 * newline-separated "D ID [DLC] B0 B1 …" lines. Returns the string length. */
size_t   rawlog_render(char *out, size_t cap, int max_lines);

#ifdef __cplusplus
}
#endif
