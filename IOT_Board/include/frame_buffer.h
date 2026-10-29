#ifndef FRAME_BUFFER_H
#define FRAME_BUFFER_H

#include <stdint.h>
#include <stddef.h>

/* Ring capacity in frames. RAM-only (no PSRAM): sizeof(CanFrame_t)=24 B, so 2000 => ~48 KB.
   Outage tolerance = FB_CAPACITY / aggregate_frame_rate — e.g. at ~162 frames/s a full-rate
   bus buffers ~12 s; with send-on-change (CAN_SEND_ON_CHANGE) the effective rate is far lower,
   so this covers a much longer uplink outage. Raised from 500 for weak-signal tolerance.
   This is a static array in internal SRAM and competes with the BLE stack / task stacks — if
   you push it higher, verify free heap at runtime (esp_get_free_heap_size) or enable PSRAM. */
#ifndef FB_CAPACITY
#define FB_CAPACITY 2000u
#endif

typedef struct
{
  uint64_t epoch_ms;   /* wall-clock receive time (ms); see now_epoch_ms() */
  uint32_t id;         /* extended CAN identifier */
  uint8_t  dlc;        /* 0..8 */
  uint8_t  data[8];
} CanFrame_t;

/** @brief One-time init (clears the ring). Call before any producer/consumer runs. */
void fb_init(void);

/**
 * @brief  Producer: append one frame. If the ring is full, the oldest frame is dropped
 *         (fb_dropped() increments) so the newest data is always retained.
 */
void fb_push(const CanFrame_t *frame);

/** @brief Number of frames currently buffered. */
size_t fb_count(void);

/**
 * @brief  Consumer: copy up to @p max oldest frames into @p out WITHOUT removing them.
 * @param  out_first_seq  receives the absolute sequence index of the first copied frame;
 *                        pass it (advanced by the return value) to fb_commit() once the
 *                        broker acknowledges the batch.
 * @return number of frames copied.
 */
size_t fb_peek(CanFrame_t *out, size_t max, uint64_t *out_first_seq);

/**
 * @brief  Consumer: permanently remove every buffered frame whose absolute sequence index is
 *         < @p upto_seq (i.e. pass first_seq + count_peeked after a successful, acked publish).
 *         Idempotent and safe against concurrent overflow drops.
 */
void fb_commit(uint64_t upto_seq);

/** @brief Total frames dropped due to overflow since boot (loss visibility for telemetry). */
uint32_t fb_dropped(void);

#endif /* FRAME_BUFFER_H */
