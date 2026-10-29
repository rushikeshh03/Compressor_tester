# IOT_Board — harness telemetry → MQTT uploader

> Part of the 3‑board station — see the root [`../README.md`](../README.md) and
> [`../CLAUDE.md`](../CLAUDE.md). **Keep docs updated** when you change pins,
> wiring, the CAN protocol, or a feature (root `CLAUDE.md` → *Documentation
> maintenance*).

Firmware for the **Coulomb IOT board** (ESP32-S3-WROOM-1 N8R2, SD card + CAN;
the EC200 LTE modem is **no longer used**), forked from the IOT charger/BMS
firmware `ESP32_S3_test10`. Its job: sit as a **passive listener** on the
harness test-station CAN bus, capture the **DAS harness protocol** frames the
harness master broadcasts (barcode, cell voltages, temperatures, fault, heartbeat),
**log them to the SD card, and sync those logs to a LOCAL Data Server** (see below).
It also types the scanned barcode to a tablet over BLE HID.

**Uplink = LOCAL Data Server sync** (`das_sync.*`, since 2026-09-26). The old cloud
MQTT frame uplink (`167.71.232.120`) was **retired**. The board now syncs its SD logs to
a local Node-RED + MQTT server, modeled on `../../Testing/Data Server` (`docs/PROTOCOL.md`):
files over **HTTP `POST /api/upload`** (SHA-256, resend on bad ACK), **MQTT control-only**
(presence + status + `sync/list`↔`sync/req` + `result`). Per scan it does BOTH: uploads a
**per-scan RAW-CAN CSV** over HTTP (`/barcodes/<barcode>_<stamp>.csv`, every frame from one
barcode to the next — like the LCD/EOL `sd_logger`) AND publishes a **decoded result summary
over MQTT** (`das/<station>/<dev>/result` — PASS/FAIL + cells/temps/fault, for the server's
yield/analytics). Plus a **daily 23:00** bundle of that day's raw NDJSON segments + the
previous day's event log; unreachable server ⇒ hourly retry. WiFi-STA +
one-shot SNTP live in `wifi_mqtt.*`; the storage half (RAM ring → SD NDJSON) is
`analytics.cpp` `sd_flush_service()`. The EC200 modem/AT code was purged from the app
path; in the factory bootloader the EC200 MQTT/HTTPS OTA path is compiled out
(`#define OTA_BL_MODEM_OTA 0`) — remote update is via CAN self-update (0x0C) + the factory
WiFi web updater.

⚠ **Server caveat (by design, no server change):** the reference server files *every*
upload as a barcode "module" and scores it into analytics, so the daily raw/event-log
files show up as bogus modules and skew the dashboard yield. Only the per-barcode CSVs
are meaningful there. A clean fix needs a small Node-RED edit (archive `.ndjson`/`.log`,
skip the manifest) — deferred.

⚠ **Shared SD lock:** three tasks touch the single global `SD` now (SD flush, event log,
das_sync). All SD access in `sd_log.cpp`/`event_log.cpp`/`das_sync.cpp` MUST go through
`sd_lock()`/`sd_unlock()` (created by `sd_lock_init()` in `setup()`). A long read/upload
holds the lock only for the SD read (into a PSRAM buffer), then releases it for the HTTP POST.

⚠ **New NVS keys** (namespace `iot_cfg`): config `ds_host`/`ds_http`/`ds_mqtt`/`ds_station`/
`ds_token`/`ds_muser`/`ds_mpass`; sync state `ds_seg_hw`/`ds_evdate`/`ds_lastday`. Server
defaults (`SAURABH.local`, station `harness`, token `das-line-token`) are in `das_sync.h`.

This is a sibling of the Harness Tester in the parent folder, but runs on
**different hardware** (the IOT board, not the Waveshare 7" LCD board). Do not
copy the Waveshare pin/LCD/USB-CDC config here.

## What was changed from `ESP32_S3_test10`

- **CAN protocol purged → DAS capture** (`src/can.cpp`, `include/main.h`). The
  charger/BMS parsing (voltage scaling, lifetime-aggregate exclusion,
  send-on-change dedup, BMS/desktop CAN transmits) was removed. `handle_rx_message()`
  now whitelists the 11-bit standard DAS IDs and pushes each raw frame into the
  lossless ring: `DAS_BARCODE0/1/2_ID` (0x600/0x601/0x602), `DAS_STATUS_ID` (0x710),
  `DAS_CELLS_ID` (0x711), `DAS_TEMPS_ID` (0x712), `DAS_FAULT_ID` (0x713),
  `DAS_HEARTBEAT_ID` (0x720). Protocol reference: `docs/harness_jig_v5.dbc` +
  `docs/harness_jig_v5_README.md` (frames 1536/1537/1538 barcode,
  1808/1809/1810/1811/1824). **We never transmit onto the harness bus**
  (diagnostic/timestamp/desktop TX all neutralised).
  - **Barcode IDs**: `0x600/0x601/0x602`, confirmed by `harness_jig_v5.dbc`
    ("ID changed from 0x700 to 0x600"). The LCD sends all three frames (DLC 8, ASCII,
    zero-padded).
  - **Fault + heartbeat detail → event log**: `das_log_semantic()` in `can.cpp`
    decodes `0x713` (v5 6-signal layout: byte0 `CellFaultMask`, byte1 `TempFaultMask`,
    byte3 `InitFailed`, plus the raw-sample detail byte4 `CellRawFaultMask`,
    byte5 `TempRepeatabilityFaultMask`, byte6 `PassingReadingCount` — logged when the
    jig sends the full DLC≥7 frame; they explain a FAIL when the averaged masks are
    clean) and `0x720` (counter, CAN TEC/REC/LEC, Tx-queue depth) into
    human-readable `/events` entries — faults edge-triggered, heartbeat throttled
    (periodic summary + on CAN error) so the 2 Hz heartbeat never floods SD. The raw
    frames also land on SD (NDJSON) for the daily raw sync. Separately, `can.cpp` captures a
    **per-scan raw-CAN session** (every received frame from one barcode to the next, incl.
    heartbeats, buffered in PSRAM) and hands it to `das_sync` → `/barcodes/<bc>_<stamp>.csv`.
- **BLE HID keyboard peripheral → tablet** (`src/ble_hid_device.cpp`,
  `include/ble_hid_device.h`), adapted PERIPHERAL-ONLY from `../LCD/src/ble_hid_device.*`
  (owns the radio itself; no scanner central). Advertises as `CONFIG_BLE_HID_NAME`; the
  tablet pairs with it. The barcode CAN frames are reassembled in `can.cpp` and typed to
  the tablet the instant a scan completes (`ble_hid_dev_type(code + "\n")`), on the CAN Rx
  path, BEFORE the ring/log/MQTT work — a scanner-wedge for the tablet. (The earlier
  "BLE removed entirely" from the fork is now reversed for this HID role only; the old
  charger-app GATT service `ble.cpp` stays deleted.)
- **Uplink = local Data Server sync** (`src/das_sync.cpp`, `include/das_sync.h`). Runs in
  its own task (core 0). Per-barcode CSV uploaded promptly on scan complete; daily 23:00
  raw+event bundle; hourly retry; control MQTT (presence/status/sync). Files over HTTP
  `POST /api/upload` with SHA-256. Server/station/token overridable from NVS (`iot_cfg`);
  defaults in `das_sync.h` (`SAURABH.local`, `harness`, `das-line-token`). WiFi-STA + SNTP
  come from `wifi_mqtt.*` (`wifi_ensure_connected()` — 3-slot NVS creds, anchors the
  software RTC via `rtc_record_sync()`). The old cloud esp-mqtt publisher was removed.

## Wire format / server protocol

The board no longer emits the cloud `v:2` frame-batch payload. It now speaks the **Data
Server protocol** (`../../Testing/Data Server/docs/PROTOCOL.md`): HTTP `POST /api/upload`
(headers `X-Token/X-Station/X-Device/X-Filename/X-Sha256`, body = file bytes) + the MQTT
control topics `das/<station>/<dev>/{presence,status/*,sync/*,upload/ack,result}`. Two per-scan
outputs, by design:
- **Raw frames → HTTP file**: `/barcodes/<barcode>_<stamp>.csv` (header `Time,Dir,ID,DLC,Data,Decoded`,
  one row per received frame — captured in `can.cpp`, written by `das_sync.cpp handle_raw_session()`).
- **Decoded summary → MQTT**: a JSON `{barcode,result,status,ts,cells_mV[8],temps_c[4],cellFaultMask,
  tempFaultMask,initFailed,incomplete,device,station}` published to `das/<station>/<dev>/result`
  (QoS1) by `publish_result()` — this is what feeds the server's PASS/FAIL yield/analytics, so the
  server must subscribe to `das/+/+/result`. `result` is `PASS`/`FAIL`/`INCOMPLETE`.

The DAS frame IDs (`main.h DAS_*`) still must match the harness master (`../LCD/include/ba100_can.h`).

## CAN flashing / partitions — kept identical to `ESP32_S3_test10`

The partition table (`partitions.csv`: `factory` + `ota_0` + `ota_1`) and the
CAN-flash bootloader flow are intentionally **unchanged** from the IOT source. The
`0x0C` CAN self-update enter trigger and the `0x2E0` runtime baud command are still
handled in `can_receive()` (before the DAS whitelist). **RULE: if you change the
partition/backup/enter-trigger/flash scheme here, make the same change in
`ESP32_S3_test10` — do not let them diverge.**

## Build

- `~/.platformio/penv/Scripts/pio.exe run` from this directory (env
  `esp32-s3-devkitc-1`, platform `espressif32@6.9.0` — precompiled, so a
  spaces-in-path project dir is fine here, unlike the pioarduino Harness Tester).
- `ARDUINO_USB_CDC_ON_BOOT=1` is correct for THIS board (IOT board has native USB
  for logs) — do not copy the Harness Tester's USB-CDC-off setting.
- **Log every change in `CHANGELOG.md`.**
