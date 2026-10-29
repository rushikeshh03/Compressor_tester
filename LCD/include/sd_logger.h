#pragma once
/**
 * sd_logger.h  –  Hot-pluggable micro-SD logger (two files, per day)
 *
 * Writes TWO daily log files, chosen by the current date (from the CAN time-sync
 * frame; "nodate" until the clock is set, then a per-day file, rolling at
 * midnight):
 *   - /data_<tag>.csv   — one CSV row per scan/test: scanned barcode + the
 *                         response received on CAN (echoed barcode, result, and
 *                         cell/temperature data).  See sd_logger_data().
 *   - /events_<tag>.log — LCD/system operations (boot, scanner connect/disconnect,
 *                         CAN link up/down, FAIL lock, …).  See sd_logger_event().
 *   - /barcodes/<bc>_<stamp>.csv — RAW CAN capture, one file PER SCAN. Every CAN
 *                         frame (RX + our TX) between one barcode and the next goes
 *                         here (Time,Dir,ID,DLC,Data[,Decoded]).  See
 *                         sd_logger_frame() + sd_logger_session_open/close().
 *
 * Fully self-contained and isolated from the rest of the system:
 *   - All SD/SPI/file I/O happens on a dedicated low-priority FreeRTOS task.
 *   - Producers only enqueue (never block, never touch the card), so logging can
 *     NEVER stall the display or CAN paths.
 *   - Hot-plug aware: files open when a card appears, stop when it's removed.
 *
 * Wiring (Waveshare ESP32-S3 Touch LCD, per the board SD demo):
 *   SD over SPI — MOSI=GPIO11, CLK=GPIO12, MISO=GPIO13.
 *   SD chip-select is on the CH422G I/O expander (pin index 4), held LOW
 *   (SD is the only device on this SPI bus), so the Arduino SD library is
 *   started with no hardware CS.
 */

#include <stdint.h>

namespace esp_expander { class Base; }

#ifdef __cplusplus
extern "C" {
#endif

/* Start the logger. `expander` is the board's CH422G base (used once to hold
 * the SD chip-select line low). Safe to pass nullptr (logging stays disabled).
 * Spawns the background task; returns immediately. */
void sd_logger_init(esp_expander::Base *expander);

/* EVENT file: one line "<timestamp>  <msg>" → /events_<tag>.log.
 * Non-blocking (dropped if the card is absent or the queue is full). */
void sd_logger_event(const char *msg);
void sd_logger_eventf(const char *fmt, ...);   /* printf-style wrapper */

/* DATA file: one CSV row → /data_<tag>.csv. `csv_fields` is everything AFTER the
 * timestamp column (the logger prepends "<timestamp>,"). The header row is
 * written automatically when a new data file is created. Non-blocking. */
void sd_logger_data(const char *csv_fields);

/* RAW CAN capture, one file per scan. sd_logger_session_open() closes any open
 * session file and starts a new one named for the barcode; every sd_logger_frame()
 * after that (until the next open/close) is written into it. Call
 * sd_logger_session_open() on each scan — the previous session (all frames up to
 * the new scan) is closed automatically. Non-blocking; no-op with no card. */
void sd_logger_session_open(const char *barcode);
void sd_logger_session_close(void);

/* Log one raw CAN frame into the currently-open session file (and nowhere else if
 * none is open). tx=true for frames we transmit. `decoded` may be nullptr. */
void sd_logger_frame(bool tx, uint32_t id, const uint8_t *data, uint8_t dlc, const char *decoded);

/* true while a card is mounted and the log files are open (for UI status). */
bool sd_logger_present(void);

#ifdef __cplusplus
}
#endif
