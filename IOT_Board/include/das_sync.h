/******************************************************************************************************************************
 * @file    das_sync.h
 * @brief   Local Data Server sync client (replaces the cloud MQTT frame uplink).
 *
 * ROLE
 * ----
 *  The IOT board is a passive DAS listener. Instead of streaming raw CAN frames to a
 *  cloud broker, it now syncs its SD-card logs to a LOCAL "Data Server" (Node-RED +
 *  MQTT broker), modeled on ../Testing/Data Server (docs/PROTOCOL.md):
 *
 *    • Per-barcode result CSV — built here from the DAS result frames, written to
 *      /barcodes/<barcode>.csv and uploaded PROMPTLY when a scan completes.
 *    • Daily at 23:00 — that day's raw NDJSON segments (/log/*.ndjson) + the previous
 *      day's finalized event log (/events/YYYYMMDD.log).
 *    • Hourly retry whenever the server is unreachable.
 *
 *  Transport matches the reference: file bytes over HTTP `POST /api/upload`
 *  (X-Token/X-Station/X-Device/X-Filename/X-Sha256, SHA-256 verified, resend on bad
 *  ACK); MQTT only for control (presence + status + sync negotiation).
 *
 *  ⚠ The reference server files every upload as a barcode "module" and scores it into
 *  analytics. Raw/event-log files therefore appear as bogus modules and skew the
 *  dashboard yield — accepted by design (no server change), documented in CLAUDE.md.
 ******************************************************************************************************************************/
#ifndef DAS_SYNC_H
#define DAS_SYNC_H

#include <stdint.h>
#include <stdbool.h>

/* ── Master enable (0 = compile the module to no-ops) ── */
#ifndef CONFIG_DAS_SYNC
#define CONFIG_DAS_SYNC             1
#endif

/* ── Server defaults (all overridable at runtime from NVS "iot_cfg") ──────────── */
#define DS_HOST_DEFAULT             "SAURABH.local"   /* IP / hostname / <PC>.local */
#define DS_HTTP_PORT_DEFAULT        1880u
#define DS_MQTT_PORT_DEFAULT        1883u
#define DS_STATION_DEFAULT          "isolation"
#define DS_TOKEN_DEFAULT            "das-line-token"  /* == server DAS_TOKEN env    */
#define DS_MQTT_USER_DEFAULT        ""                /* set if the broker needs auth */
#define DS_MQTT_PASS_DEFAULT        ""

/* ── Behaviour ── */
#define DS_CSV_DIR                  "/barcodes"       /* per-scan raw-CAN session CSVs */
#define DS_SCHED_HOUR               23                /* daily raw+event upload hour  */
#define DS_RETRY_INTERVAL_MS        3600000UL         /* hourly retry of pending      */
#define DS_PRESENCE_MS              30000UL           /* retained "online" heartbeat  */
#define DS_UPLOAD_RETRIES           3                 /* resends on a bad ACK         */
#define DS_RAW_QUEUE_DEPTH          8                 /* finished sessions awaiting upload */
#define DS_RESULT_QUEUE_DEPTH       8                 /* decoded results awaiting MQTT publish */

/* Per-scan raw-CAN session capture (can.cpp). The session runs from one barcode to the
 * next; it also finalizes DS_RAW_LINGER_MS after the result completes so the LAST scan of a
 * batch still uploads promptly. Buffered in PSRAM up to DS_RAW_SESSION_CAP bytes (a very
 * long idle gap full of heartbeats past that cap is truncated — the barcode+result frames
 * come first, so they are always captured). */
#define DS_RAW_SESSION_CAP          32768u            /* ~700 frame rows                 */
#define DS_RAW_LINGER_MS            15000u            /* finalize this long after a result */

/* Decoded snapshot of ONE DAS test result — published over MQTT (das/<station>/<dev>/result)
 * for the server's analytics/yield. Filled on the CAN Rx path (can.cpp) from the multiplexed
 * result burst; the raw frames themselves still go to the server as a file over HTTP. */
typedef struct {
    char     barcode[25];
    uint64_t epoch_ms;         /* wall-clock at completion (0 before first SNTP sync) */
    bool     have_status;      /* 0x710 Seq0 seen                                     */
    uint8_t  status;           /* 0 = FAIL, 1 = PASS                                  */
    uint8_t  cells_seq_mask;   /* 0x711 Seq bits seen (b0..b2)                        */
    uint16_t cell_mV[8];       /* averaged cell voltages, mV                          */
    uint8_t  temps_seq_mask;   /* 0x712 Seq bits seen (b0..b1)                        */
    int16_t  temp_c10[4];      /* averaged temps, tenths of a degree C (signed)       */
    bool     have_fault;       /* 0x713 seen                                          */
    uint8_t  cellFaultMask;    /* 0x713 byte0                                         */
    uint8_t  tempFaultMask;    /* 0x713 byte1 (low nibble)                            */
    uint8_t  initFailed;       /* 0x713 byte3                                         */
    bool     incomplete;       /* barcode only — no jig result frames arrived         */
} das_result_t;

#ifdef __cplusplus
extern "C" {
#endif

/** @brief  Load config from NVS, compute the device id, create the queues, start
 *          the MQTT control client, and spawn das_sync_task. Call from init_task after
 *          the SD card mount. No-op if CONFIG_DAS_SYNC == 0. */
void das_sync_init(void);

/** @brief  Non-blocking hand-off of ONE decoded result from the CAN Rx task. das_sync
 *          publishes it as JSON over MQTT (das/<station>/<dev>/result, QoS1) once the broker
 *          is connected; it waits in the queue while offline (dropped only on queue overflow —
 *          the raw HTTP file still carries the result frames). Returns true if queued. */
bool das_sync_enqueue_result(const das_result_t *r);

/** @brief  Non-blocking hand-off of ONE finished raw-CAN session from the CAN Rx task.
 *          @p buf is a heap block (PSRAM-preferred) of @p len bytes holding the CSV frame
 *          rows (WITHOUT the header — das_sync prepends it); ownership TRANSFERS to das_sync,
 *          which frees it after writing. On a full queue this returns false and does NOT free
 *          (the caller frees). Names the file <barcode>_<stamp>.csv. */
bool das_sync_enqueue_raw(const char *barcode, char *buf, uint32_t len, bool truncated);

/** @brief  true while the LOCAL Data Server is reachable (control-MQTT session connected).
 *          Used for the IoT→LCD status heartbeat (0x7A0) uplink bit. */
bool das_sync_server_up(void);

#ifdef __cplusplus
}
#endif

#endif /* DAS_SYNC_H */
