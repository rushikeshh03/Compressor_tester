# Porting guide: lossless CAN frame buffer + acknowledged-batch MQTT publisher

**Audience:** a Claude Code instance working in a *different* ESP32-S3 BMS gateway firmware project (reference target discussed: `ESP32_S3_test9` at `C:\Users\hitar\My Drive\PC\Projects\Development\IOT\ESP32_S3_test9`), tasked with porting this subsystem in. This document is self-contained — you should not need the source conversation to act on it, only this file plus read access to both codebases.

**Source of truth:** this repo (`iot_autonxt_v2.0.0`), specifically [include/frame_buffer.h](include/frame_buffer.h), [src/frame_buffer.cpp](src/frame_buffer.cpp), [src/can.cpp](src/can.cpp), [src/analytics.cpp](src/analytics.cpp), [include/mqtt.h](include/mqtt.h), [include/main.h](include/main.h), [src/main.cpp](src/main.cpp). Read those files directly for the exact current code — this report explains *why* they're shaped the way they are and *what to change* in the target project; it is not a substitute for reading the actual diff.

---

## 1. Problem this solves

Both firmwares keep a **latest-value-per-CAN-id snapshot**: a fixed array `CanMsgs[200]`, indexed by `(id >> 8) & 0xFF`, overwritten in place every time a frame for that id arrives. This is fine for BLE/serial consumers that only ever want "the current value." It is lossy for MQTT/analytics: if two frames for the same device id arrive between publish cycles, the older one is silently discarded and the backend never sees it.

The target project's old (and still-current, at time of writing) MQTT path is even more susceptible: it does a scheduled sweep of the whole `CanMsgs[]` snapshot every ~1s and fires a **QoS0, fire-and-forget** `AT+QMTPUBEX` with no broker acknowledgment tracking at all — if the publish silently fails (`+QMTPUB:` URC is never even parsed there), the firmware has no way to know and never retries.

This subsystem replaces that with:
1. A **lossless ring buffer** that captures every accepted CAN frame (not just the latest per id), independent of the existing `CanMsgs[]` snapshot.
2. An **adaptive-batch, QoS1, acknowledged publisher** that drains the ring in bounded batches and only removes a batch from the ring once the broker has confirmed receipt (`+QMTPUB: <idx>,<msgId>,0`). A failed or timed-out publish leaves the frames in the ring for retry.

Net effect: CAN data reaching the backend becomes durable across transient MQTT/modem hiccups, at the cost of a bigger wire payload (per-frame, not per-device-snapshot) and more state to reason about.

---

## 2. Architecture

```
CAN_Rx task                         MQTT_Pub task
   |                                     |
   | twai_receive()                      | mqtt_service() every ~500ms
   |                                     |
   v                                     v
handle_rx_message()                 fb_count() -> decide whether to publish now
   |-- canmsgs_write() [unchanged]      |
   |-- fb_push(&frame) [NEW] ---------> ring buffer (frame_buffer.c)
                                         |
                                         fb_peek(frames, MQTT_BATCH_MAX, &firstSeq)
                                         |
                                         publish_batch(frames, n)  -- AT+QMTPUBEX, block for +QMTPUB URC
                                         |
                                    success? --yes--> fb_commit(firstSeq + n)
                                         |
                                         no --> leave buffered, force reconnect next cycle
```

Concurrency model is intentionally the simplest one that's correct: **single producer** (the CAN_Rx task calling `fb_push`), **single consumer** (the MQTT_Pub task calling `fb_peek`/`fb_commit`). All ring state is guarded by a FreeRTOS spinlock (`portMUX_TYPE`), not a mutex, because critical sections are O(1) copies, not blocking work. If the target project's CAN reception happens from more than one task/ISR, this single-producer assumption breaks and the ring needs a real lock or a lock-free SPSC scheme instead.

Sequence numbers are absolute (`uint64_t`, never wrap in practice) so that `fb_commit(upto_seq)` is safe to call even if an overflow drop has already advanced the tail past `upto_seq` — it's a no-op in that case, not a corruption.

---

## 3. New files to add verbatim

### `include/frame_buffer.h`

```c
#ifndef FRAME_BUFFER_H
#define FRAME_BUFFER_H

#include <stdint.h>
#include <stddef.h>

/* Ring capacity in frames. RAM-only (no PSRAM): sizeof(CanFrame_t)=24 B, so 1500 => ~36 KB.
   Outage tolerance = FB_CAPACITY / aggregate_frame_rate — tune after measuring the live bus. */
#ifndef FB_CAPACITY
#define FB_CAPACITY 1500u
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
```

### `src/frame_buffer.cpp`

```c
#include "frame_buffer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Statically allocated ring — avoids heap fragmentation on a device without PSRAM. */
static CanFrame_t s_ring[FB_CAPACITY];

/* Absolute (never-wrapping) sequence counters. count = head - tail, always <= FB_CAPACITY. */
static uint64_t s_head = 0;   /* index of the next slot to write (== total ever pushed) */
static uint64_t s_tail = 0;   /* index of the oldest valid frame */
static uint32_t s_dropped = 0;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

void fb_init(void)
{
  portENTER_CRITICAL(&s_mux);
  s_head = 0;
  s_tail = 0;
  s_dropped = 0;
  portEXIT_CRITICAL(&s_mux);
}

void fb_push(const CanFrame_t *frame)
{
  if (frame == NULL)
  {
    return;
  }

  portENTER_CRITICAL(&s_mux);

  if ((s_head - s_tail) >= FB_CAPACITY)
  {
    /* Full: drop the oldest to make room for the newest. */
    s_tail++;
    s_dropped++;
  }

  s_ring[s_head % FB_CAPACITY] = *frame;
  s_head++;

  portEXIT_CRITICAL(&s_mux);
}

size_t fb_count(void)
{
  size_t n;
  portENTER_CRITICAL(&s_mux);
  n = (size_t)(s_head - s_tail);
  portEXIT_CRITICAL(&s_mux);
  return n;
}

size_t fb_peek(CanFrame_t *out, size_t max, uint64_t *out_first_seq)
{
  if (out == NULL || max == 0)
  {
    return 0;
  }

  size_t copied = 0;

  portENTER_CRITICAL(&s_mux);

  uint64_t seq = s_tail;
  if (out_first_seq != NULL)
  {
    *out_first_seq = seq;
  }

  uint64_t available = s_head - s_tail;
  size_t n = (available < (uint64_t)max) ? (size_t)available : max;

  for (size_t i = 0; i < n; i++)
  {
    out[i] = s_ring[(seq + i) % FB_CAPACITY];
  }
  copied = n;

  portEXIT_CRITICAL(&s_mux);

  return copied;
}

void fb_commit(uint64_t upto_seq)
{
  portENTER_CRITICAL(&s_mux);
  /* Only advance forward, and never past what has actually been produced. Overflow drops may
     have already moved the tail beyond upto_seq — in that case this is a no-op. */
  if (upto_seq > s_tail)
  {
    s_tail = (upto_seq < s_head) ? upto_seq : s_head;
  }
  portEXIT_CRITICAL(&s_mux);
}

uint32_t fb_dropped(void)
{
  uint32_t d;
  portENTER_CRITICAL(&s_mux);
  d = s_dropped;
  portEXIT_CRITICAL(&s_mux);
  return d;
}
```

These two files are copy-paste portable — they have no dependency on anything project-specific (no `main.h`, no globals). PlatformIO auto-globs `src/*.cpp`, so dropping `frame_buffer.cpp` into `src/` is enough to build it; no `platformio.ini` change needed.

---

## 4. Integration points in the target project

Read the target files first — **do not blind-copy** the snippets below, because the target's `analytics.cpp`/`can.cpp` have diverged (voltage filtering, OTA subsystem, a different time source — see §5 gotchas). Treat this as "what needs to exist," and adapt to the target's actual variable/function names.

### 4.1 `include/main.h`
- `#include "frame_buffer.h"`
- Nothing else structural is required here, but you'll want a place for `now_epoch_ms()`'s prototype if it doesn't already exist (see §5.1).

### 4.2 `src/can.cpp` — CAN receive path
In the frame-accept path (in this repo, `handle_rx_message()` in [src/can.cpp:110-156](src/can.cpp:110)), **after** the existing `canmsgs_write(dev_idx, message)` call that maintains the latest-per-id snapshot, add a second, independent write into the ring:

```c
/* Lossless copy for the MQTT path: every accepted frame, stamped with its own time. */
CanFrame_t frame;
frame.epoch_ms = now_epoch_ms();
frame.id       = message->identifier;
frame.dlc      = dlc;
memset(frame.data, 0, sizeof(frame.data));
memcpy(frame.data, message->data, dlc);
fb_push(&frame);
```

Do this at the point where `dlc` is already clamped to ≤8 and the frame has already passed `is_target_id()`/extended-frame/RTR filtering — i.e. exactly the same frames that update `CanMsgs[]` should also go into the ring. The target's `handle_rx_message()` is structurally near-identical to this repo's (same `is_target_id()` pattern/mask, same `canmsgs_write()` helper), so the insertion point should be a direct match once you diff the two functions.

### 4.3 `include/mqtt.h` — new tuning constants
Add (values are this repo's starting points — see §6 for what to actually tune):

```c
/* Wire-format version for the batched payload (backend branches on this; legacy = no "v" key) */
#define MQTT_PAYLOAD_VERSION            2

/* Publisher behaviour (adaptive-batch, QoS1 with broker ack) */
#define MQTT_QOS                        1u        /* QoS1 => broker ACK required per publish   */
#define MQTT_PUBLISH_PERIOD_MS          5000u     /* base drain cadence                        */
#define MQTT_FLUSH_WATERMARK            300u      /* flush early once the ring reaches this     */
#define MQTT_BATCH_MAX                  40u       /* frames per MQTT message (bounds payload)   */
#define MQTT_HEARTBEAT_MS               30000u    /* liveness msg when no frames are queued     */
#define MQTT_PUB_ACK_TIMEOUT_MS         8000u     /* wait for +QMTPUB before treating as failed */
#define MQTT_CONN_TIMEOUT_MS            10000u    /* wait for +QMTCONN after connect            */
#define MQTT_MAX_CONN_RETRY             3u        /* reconnect attempts before power-cycling     */
```

### 4.4 `src/analytics.cpp` — the publisher itself
This is the bulk of the port. You need, in rough dependency order:

1. **New globals** for ack tracking:
   ```c
   volatile bool mqttOpened = false;
   volatile bool mqttConnected = false;        /* target may already have some form of this */
   volatile int  mqttLastPubResult = -1;
   volatile int  mqttLastPubMsgId = 0;
   SemaphoreHandle_t mqttPubDoneSem = NULL;    /* given when a +QMTPUB result arrives */
   ```
   Check what the target already has — it likely already tracks `mqttConnected` from `+QMTCONN`/`+QMTSTAT` in some form. Reuse rather than duplicate.

2. **`next_msg_id()`** — rolling non-zero packet id (QoS1 requires a real msgId, unlike the target's current `AT+QMTPUBEX=0,0,0,0,...` which hardcodes msgId 0 = fire-and-forget):
   ```c
   static uint16_t next_msg_id(void)
   {
     static uint16_t id = 0;
     id++;
     if (id == 0) id = 1;
     return id;
   }
   ```

3. **`frame_data_hex()`** — serialize a frame's payload bytes as lowercase hex for JSON:
   ```c
   static void frame_data_hex(const CanFrame_t *f, char *out, size_t out_size)
   {
     size_t pos = 0;
     for (uint8_t k = 0; k < f->dlc && (pos + 2) < out_size; k++)
       pos += snprintf(out + pos, out_size - pos, "%02x", f->data[k]);
     out[pos] = '\0';
   }
   ```

4. **URC handlers for `+QMTOPEN:`, `+QMTCONN:`, `+QMTSTAT:`, `+QMTPUB:`** — dispatched from wherever the target's UART GSM event loop already pattern-matches response prefixes (this repo's `uart_gsm_event()`). **The target currently does not parse `+QMTPUB:` at all** — this is the critical missing piece that makes ack-based commit possible:
   ```c
   static void get_mqtt_pub(char *response)
   {
     char *p = strstr(response, "+QMTPUB:");
     int idx, mid, res;
     if (p && sscanf(p, "+QMTPUB: %d,%d,%d", &idx, &mid, &res) >= 3)
     {
       mqttLastPubMsgId = mid;
       mqttLastPubResult = res;
       if (mqttPubDoneSem != NULL) xSemaphoreGive(mqttPubDoneSem);
     }
   }
   ```
   Add the corresponding `const char *mqttPubResp = "+QMTPUB:";` and a dispatch line `if (strstr(buffer, mqttPubResp)) get_mqtt_pub(buffer);` alongside the target's existing URC dispatch (it already does this pattern for `+QMTCONN:`, `+QMTSTAT:` — follow that convention).

5. **`ensure_mqtt_ready()`** — reconnect-with-retry helper, only called when not already connected (this repo's version at [src/analytics.cpp:172-225](src/analytics.cpp:172)). The target's current `mqtt_publish_messages()` calls `mqtt_init()` unconditionally on every disconnect detection with no retry bound or backoff — replace that with a bounded-retry version that also power-cycles the modem (`AT+CFUN=1,1`) after repeated failure, matching the target's existing `check_modem_status()`/`check_network_registration()` retry style.

6. **`publish_batch(const CanFrame_t *frames, size_t n)`** — builds the JSON, sends `AT+QMTPUBEX` with QoS1 and a real msgId, then **blocks on `mqttPubDoneSem`** waiting for the `+QMTPUB:` URC, and returns true only if `mqttLastPubResult == 0`. See [src/analytics.cpp:232-303](src/analytics.cpp:232) for the reference implementation, including the JSON shape (§5.2 below) and the "never publish truncated JSON" safety check against `measureJson()`.

7. **`mqtt_service()`** (replaces the target's `mqtt_publish_messages()`) — the adaptive-batch loop: decide whether to publish now (period elapsed / ring watermark reached / heartbeat due), ensure connectivity, then drain the ring via `fb_peek()`/`publish_batch()`/`fb_commit()` in a bounded loop (`MAX_BATCHES_PER_CALL`) so one task tick doesn't starve other tasks under a burst. See [src/analytics.cpp:310-376](src/analytics.cpp:310) for the reference implementation. Keep the target's existing `check_modem()`/`check_signal_quality()`/`check_can_status()`/location/timestamp calls — just call them at the top of this function instead of the old one.

8. **`mqtt_init()`** — create `mqttPubDoneSem` if not already created (`xSemaphoreCreateBinary()`), otherwise the target's existing open/conn/sub sequence is fine as-is.

### 4.5 `src/main.cpp`
- Call **`fb_init()`** early in `setup()`, before any task that could call `fb_push()` is created (in this repo it's the very first line of substance in `setup()` — see [src/main.cpp:78](src/main.cpp:78)).
- In `publish_mqtt_task()`, change the call from `mqtt_publish_messages()` to `mqtt_service()`. Consider shortening the task's tick period (this repo uses 500 ms instead of the target's 1000 ms) since `mqtt_service()` internally decides whether to actually do anything — the tighter tick just lets it react to a full ring sooner, it doesn't force more AT traffic.

### 4.6 `include/main.h` prototypes
Add/replace:
```c
void mqtt_service(void);            /* adaptive-batch, QoS1 publisher cycle (replaces mqtt_publish_messages) */
uint64_t now_epoch_ms(void);
```

---

## 5. Gotchas specific to the target project (read before porting)

These are real divergences found by diffing this repo against `ESP32_S3_test9` — do not skip this section.

### 5.1 Time source mismatch
This repo's `now_epoch_ms()` ([src/analytics.cpp:636-641](src/analytics.cpp:636)) is a thin wrapper over `gettimeofday()`, fed by `settimeofday()` inside `get_time()` after parsing the modem's `AT+QLTS=2` response. The target project has a **different, already-existing time subsystem**: it parses NTP/GPS time into a `timestampData` struct (`year/month/day/hour/minute/second`) and advances a **software RTC** between syncs via `rtc_update_timestamp()` (millis-elapsed since `rtc_base_millis`) — it does **not** call `settimeofday()`/`gettimeofday()` anywhere.

You have two options:
- **(Recommended)** Write a target-specific `now_epoch_ms()` that converts the target's `timestampData` + elapsed-millis-since-sync into a Unix epoch (`mktime()` on a `struct tm` built from `timestampData`, plus the millis delta for sub-second precision), rather than introducing a parallel `settimeofday()`-based clock.
- Or port this repo's `get_time()`/`settimeofday()` approach wholesale and retire the target's NTP/GPS software-RTC — bigger change, only do this if you're also asked to unify the time handling.

Either way, **do not skip stamping frames with a real timestamp** — a ring of frames with epoch_ms=0 defeats the purpose for backend analytics.

### 5.2 Wire format is a breaking change for the backend
This repo's `publish_batch()` emits:
```json
{"v":2,"sn":"...","sig":-75,"can":true,"lat":"12.9","lon":"77.5","drop":0,"rxof":0,
 "frames":[{"t":1730000000123,"id":"18a10203","d":"aabbccdd"}, ...]}
```
— an array of raw frames, each independently timestamped. The target's current `mqtt_publish_messages()` emits a **completely different shape**: one JSON key per CAN device id (`"27": ["12:34:56","18a1a002", 8, ...decoded bytes..., "\n"]`), with human-decoded (`refine_can_payload`) values, plus `canStatus`/`serialNumber`/`version`/`iot_version` top-level fields.

**Whatever backend/analytics service consumes the target's MQTT topic today expects the old per-id-keyed shape.** Porting this subsystem without also updating that backend will silently break ingestion (valid JSON, wrong schema). Before switching the target's live publish path over:
- Confirm with whoever owns the backend whether it can be updated to parse the new per-frame array shape (this repo's `MQTT_PAYLOAD_VERSION` field exists specifically so the backend can branch on wire format).
- Or, as an intermediate step, keep `mqtt_publish_messages()`'s JSON *shape* but source its contents from `fb_peek()`'s drained frames instead of a live `CanMsgs[]` sweep — i.e. adopt the lossless-buffering/ack-retry *mechanism* without the wire-format change. This is more work (you'd need to decide how to fold multiple per-id frames into the old single-snapshot-shaped doc) but avoids a backend migration.

Flag this to whoever's driving the port — it's a product decision, not a code decision.

### 5.3 OTA subsystem shares the same UART/AT-command channel
The target has a large OTA-over-MQTT subsystem (`ota_bootloader.cpp`, ~150 KB) that is completely absent from this repo. It listens on `MQTT_TOPIC_OTA`/`MQTT_TOPIC_OTA_RESPONSE`/`MQTT_TOPIC_CANCEL`, parses OTA commands out of `+QMTRECV:` payloads in the same GSM UART event handler this subsystem's new URC handlers plug into, and — critically — **also issues its own AT commands over the same UART1 / `atSemaphore`** (for triggering downloads, publishing OTA status, etc.).

Two things to verify carefully when porting:
1. **`atSemaphore` contention:** `publish_batch()`'s blocking wait on `mqttPubDoneSem` (up to `MQTT_PUB_ACK_TIMEOUT_MS` = 8 s by default) happens *while* other code may be mid-sequence on `atSemaphore` for an OTA operation. Make sure the new publish path takes/releases `atSemaphore` the same way the target's existing AT-command call sites do, and that an in-flight OTA download (which can hold the UART busy for a long time) can't cause the new ack-wait to time out spuriously and mark `mqttConnected = false`, forcing a reconnect mid-OTA.
2. **URC handler collision:** confirm the target's OTA JSON parser (in its `receive_mqtt_message`-equivalent) and the new `get_mqtt_pub()`/`get_mqtt_open()`/`get_mqtt_conn()` handlers don't both fire on overlapping substrings of the same UART line (Quectel URCs are prefix-matched via `strstr`, so this is a real risk if OTA also greps for `+QMT`-prefixed text).

### 5.4 `MQTT_MAX_CONN_RETRY` + modem power-cycle interacts with the target's other retry logic
`ensure_mqtt_ready()`'s fail-streak-triggered `AT+CFUN=1,1` (full modem restart) should not be layered on top of the target's own existing modem-recovery logic without checking for double-restart loops — if both `check_modem_status()`-style code and the new `ensure_mqtt_ready()` independently decide to restart the modem, you can end up in a restart livelock under sustained network trouble.

### 5.5 `CanFrame_t` size / RAM budget
`sizeof(CanFrame_t)` is 24 bytes (8-byte `epoch_ms` + 4-byte `id` + 1-byte `dlc` + 8-byte `data`, plus padding). At `FB_CAPACITY = 1500`, the ring is a static ~36 KB array. The target's ESP32-S3 has no PSRAM assumption baked into this design (deliberately — see the header comment), so confirm the target's actual free-RAM headroom before accepting 1500 as-is; the target already runs OTA (which needs its own scratch RAM for flash write buffers) on top of everything this repo's `setup()` allocates, so the two RAM budgets are not directly comparable. Start smaller (e.g. 500–800) and measure `esp_get_free_heap_size()` before/after if headroom looks tight.

---

## 6. Tuning knobs and how to think about them

| Constant | What it trades off |
|---|---|
| `FB_CAPACITY` | Outage tolerance (seconds of buffered CAN traffic before oldest frames start dropping) vs static RAM. `capacity / aggregate_frame_rate_hz` = seconds of MQTT/modem outage the device can absorb losslessly. |
| `MQTT_BATCH_MAX` | Frames per publish vs payload size. Must keep `serializeJson()` output well under the `jsonOutput` buffer and under the modem's max AT command / URC line length. |
| `MQTT_FLUSH_WATERMARK` | How full the ring gets before forcing an early publish outside the normal period — set well below `FB_CAPACITY` so a burst doesn't outrun the publisher before it reacts. |
| `MQTT_PUBLISH_PERIOD_MS` | Base cadence when the ring isn't under pressure — trades backend data latency vs modem/data usage. |
| `MQTT_HEARTBEAT_MS` | Liveness signal cadence when there's no CAN traffic at all (device idle) — lets the backend distinguish "device off" from "device on, nothing to report." |
| `MQTT_PUB_ACK_TIMEOUT_MS` | How long to block waiting for `+QMTPUB:` before declaring the publish failed and forcing a reconnect. Too short = false-positive reconnects on a slow network; too long = the publisher stalls and the ring fills. |
| `MQTT_MAX_CONN_RETRY` | Reconnect attempts before falling back to a full modem power-cycle (`AT+CFUN=1,1`) — the "give up and nuke it from orbit" threshold. |

Don't just copy this repo's numbers — they were chosen for this repo's CAN bus traffic rate and modem plan. Re-derive `FB_CAPACITY` and `MQTT_FLUSH_WATERMARK` from the target's actual measured frame rate.

---

## 7. Validation checklist after porting

- [ ] Build succeeds with `frame_buffer.cpp` compiled in (check the PlatformIO build log lists it).
- [ ] `fb_init()` runs before any task starts (log a line right after it and confirm ordering in the boot log).
- [ ] Every accepted CAN frame produces both a `CanMsgs[]` update *and* an `fb_push()` — add a temporary log of `fb_count()` growing under live CAN traffic.
- [ ] Unplug/disable the modem mid-run: confirm `fb_count()` grows (frames buffering) instead of the firmware crashing or silently discarding.
- [ ] Reconnect the modem: confirm the ring drains back toward 0 (batches publish and commit) rather than staying stuck (would indicate `fb_commit()` isn't being reached, or the ack semaphore is never given).
- [ ] Force a publish failure (e.g. wrong topic temporarily) and confirm `fb_dropped()`/`fb_count()` show frames are retained, not lost, and `mqttConnected` correctly flips to force a reconnect.
- [ ] Confirm OTA operations (§5.3) still complete successfully with the new publisher running concurrently — this is the highest-risk integration point.
- [ ] Confirm whatever consumes the MQTT topic on the backend can parse whichever wire format you shipped (§5.2) — check ingestion logs/dashboards, don't just check "AT command returned OK."
- [ ] Soak-test RAM: `esp_get_free_heap_size()` before/after adding the ring, with OTA and MQTT both active, to confirm §5.5's budget concern is unfounded for the chosen `FB_CAPACITY`.
