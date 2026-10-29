/******************************************************************************************************************************
 * @file    sd_log.h
 * @brief   SD-card durable store-and-forward log for CAN frames (NDJSON, keep-forever + upload cursor).
 *
 * ROLE IN THE PIPELINE
 * --------------------
 *   CAN RX (can.cpp)  --fb_push-->  RAM ring (frame_buffer)  --flush-->  SD NDJSON log (this module)
 *                                                                         |
 *                                            upload cursor --peek--> publish_batch() (unchanged MQTT format)
 *                                                                         |
 *                                            broker ACK   --mark_uploaded--> advance persisted cursor
 *
 * Records are NEVER deleted (permanent, human-readable log). A cursor persisted in NVS marks how far
 * we've uploaded, so a reboot resumes exactly where it left off and an outage never loses data.
 * One line per frame, byte-compatible with the MQTT "frames[]" objects:
 *     {"t":<epoch_ms>,"id":"<8 lower-hex>","d":"<lower-hex payload>"}
 ******************************************************************************************************************************/

#ifndef SD_LOG_H
#define SD_LOG_H

#include <stdint.h>
#include <stddef.h>
#include "frame_buffer.h"   /* CanFrame_t */

/**
 * @brief  Mount the SD card (SPI mode) and load / initialise the upload cursor.
 * @return true on success. false if the card can't be mounted — the caller should then fall back
 *         to the legacy RAM-only publish path (device keeps uploading, just without persistence).
 */
bool sd_log_init(void);

/** @brief  true once sd_log_init() has mounted the card successfully. */
bool sd_log_ready(void);

/**
 * @brief  Append @p n frames to the durable log as NDJSON lines. Rotates to a new segment file
 *         when the current one exceeds the segment size. Advances the persisted "written" counter.
 * @return true if all frames were written.
 */
bool sd_log_append_frames(const CanFrame_t *frames, size_t n);

/**
 * @brief  Copy up to @p max not-yet-uploaded frames (oldest first) into @p out WITHOUT advancing
 *         the cursor. Idempotent — safe to call again after a failed publish.
 * @param  out_first_seq  receives the absolute record sequence of out[0] (used as MQTT "seq").
 * @return number of frames copied.
 */
size_t sd_log_peek_unuploaded(CanFrame_t *out, size_t max, uint64_t *out_first_seq);

/**
 * @brief  Mark the next @p n un-uploaded frames as uploaded (call only after a broker ACK).
 *         Advances the persisted cursor past them. Idempotent-safe against over-count.
 */
void sd_log_mark_uploaded(size_t n);

/** @brief  Number of frames written but not yet marked uploaded (drives the upload loop). */
size_t sd_log_unuploaded_count(void);

/* ─────────────────────────────────────────────────────────────────────────────
 *  Shared SD access lock
 * ─────────────────────────────────────────────────────────────────────────────
 *  Three tasks now touch the single global `SD`/FATFS object: the flush task
 *  (sd_log_append_frames), the event log (event_log.cpp), and the local-server
 *  sync (das_sync.cpp, which reads /log + /events + /barcodes and writes CSVs).
 *  The Arduino SD layer is NOT safe for concurrent multi-handle access, so every
 *  SD file operation in those modules must be wrapped in sd_lock()/sd_unlock().
 *  A long operation (hashing/streaming a segment) should hold the lock only per
 *  chunk and release between chunks so the flush task is never starved. */

/** @brief  Create the shared SD mutex. Call ONCE from setup() BEFORE any task that
 *          touches the SD card is created (race-free). Safe to call more than once. */
void sd_lock_init(void);

/** @brief  Take / give the shared SD lock. Recursive-safe. No-op (returns) if the
 *          mutex was never created. @p timeout_ms<0 waits forever. Returns true if held. */
bool sd_lock(int timeout_ms);
void sd_unlock(void);

/** @brief  Index of the segment currently being appended to (the only growing file).
 *          Segments with a LOWER index are finalized (immutable) — safe to hash/upload. */
uint32_t sd_log_write_segment(void);

/** @brief  Close the current segment and start a fresh empty one (index+1), persisting
 *          the write counter. Used by the daily job so "today's" tail becomes a
 *          finalized, immutable segment that can be uploaded with a stable SHA.
 * @return  true on success (or if not ready — best-effort). */
bool sd_log_force_rotate(void);

#endif /* SD_LOG_H */
