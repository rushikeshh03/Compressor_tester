# Changelog

## [IOT_Board] 2026-09-26 — Per-scan RAW CAN capture (replaces the decoded summary CSV)

The prompt per-scan upload now carries the **raw CAN log of the module test**, not a decoded
one-row summary — mirroring the LCD/EOL `sd_logger` (`/barcodes/<barcode>_<stamp>.csv`, header
`Time,Dir,ID,DLC,Data,Decoded`).

### Changed
- **`can.cpp`**: replaced the decoded-result accumulator (`das_accumulate`/`das_result_t`) with a
  **raw-CAN session capture**. A session runs **from one barcode (0x600) to the next** and records
  **every received DAS frame** (barcode + result burst + heartbeats) as CSV rows, buffered in PSRAM
  on the CAN Rx task (RX-only, single-task, no lock; `DS_RAW_SESSION_CAP` bytes, truncates past that).
  It also finalizes **`DS_RAW_LINGER_MS` (15 s) after the result** so the last scan of a batch uploads
  without waiting for the next barcode. The finished buffer is handed to das_sync
  (`das_sync_enqueue_raw`, ownership transfers).
- **`das_sync.cpp/.h`**: added `das_sync_enqueue_raw()` — writes `/barcodes/<barcode>_<stamp>.csv`
  (header + rows) and uploads it promptly (retry/`.sent`/`sync/list` paths unchanged). New knobs
  `DS_RAW_SESSION_CAP`, `DS_RAW_LINGER_MS`, `DS_RAW_QUEUE_DEPTH`.
- **Decoded result ALSO published over MQTT** (kept, not dropped): `das_sync_enqueue_result()` +
  `das_result_t` remain — `can.cpp` still decodes the burst (0x710 status, 0x711 cells, 0x712 temps,
  0x713 fault) and `das_sync` publishes a JSON summary to **`das/<station>/<dev>/result`** (QoS1)
  `{barcode,result,status,ts,cells_mV[8],temps_c[4],cellFaultMask,tempFaultMask,initFailed,incomplete,
  device,station}`. So the split is: **raw frames over HTTP file, decoded PASS/FAIL + values over
  MQTT** — the server keeps yield/analytics without parsing the raw CSV. Results wait in a queue
  while the broker is offline (dropped only on overflow; the raw file still carries the frames).
  ⚠ The server must subscribe to `das/+/+/result` to consume this (topic/format adjustable).

### Notes
- The continuous NDJSON `/log` + daily 23:00 raw sync are unchanged (still a full continuous archive);
  the per-scan raw file is the prompt, per-module view. Some frame data is therefore captured twice.

## [IOT_Board] 2026-09-26 — Replace cloud MQTT uplink with LOCAL Data Server sync

Big architectural change: the board no longer streams raw CAN-frame `v:2/v:3` JSON
batches to the cloud broker (`167.71.232.120`). Instead it **syncs its SD logs to a
LOCAL Data Server** (Node-RED + MQTT broker), modeled on `../../Testing/Data Server`
(`docs/PROTOCOL.md`): files go over **HTTP `POST /api/upload`** (SHA-256 verified,
resend on bad ACK); **MQTT is control-only** (presence + status + sync negotiation).

### Added
- **`das_sync.*`** — the local-server sync client (own task, core 0):
  - Builds a **per-barcode result CSV** from each completed scan (decoded from the DAS
    result frames) → `/barcodes/<barcode>.csv`, uploaded **promptly**. Columns:
    `timestamp,barcode,station,result,cell1..8_mV,temp1..4_C,cellFaultMask,tempFaultMask,initFailed`
    (`result` = uppercase `PASS`/`FAIL`, or `INCOMPLETE` for a barcode-only scan).
  - **Daily at 23:00** uploads that day's raw NDJSON segments (`raw_<YYYYMMDD>_<seg>.ndjson`)
    + the **previous day's** finalized event log (`events_<YYYYMMDD>.log`).
  - **Hourly retry** of anything pending when the server is unreachable (pending CSVs are
    the ones without a `<name>.sent` marker); also handles server-initiated `sync/req`.
  - MQTT control: retained `presence=online` + LWT `offline`, `status/*`, `sync/list`↔`sync/req`,
    `upload/ack`. Server/station/token/ports overridable from NVS (`iot_cfg`:
    `ds_host`,`ds_http`,`ds_mqtt`,`ds_station`,`ds_token`,`ds_muser`,`ds_mpass`). Device id
    `harness-<last3 MAC>`. mDNS/DNS/literal-IP host resolution (default `SAURABH.local`).
  - Sync state in NVS: `ds_seg_hw` (finalized-segment high-water), `ds_evdate`, `ds_lastday`.
- **Shared SD lock** (`sd_log.cpp`: `sd_lock_init/sd_lock/sd_unlock`) — three tasks now
  touch the single global `SD` (flush, event log, das_sync). Created in `setup()` before any
  SD task starts; all SD access in those modules is wrapped. `uploadFile()` reads a segment
  into a PSRAM buffer under the lock, then POSTs with the lock released (no network I/O while
  holding SD, so the flush task isn't starved).
- **`sd_log_write_segment()`** + **`sd_log_force_rotate()`** — the daily job rotates so
  today's tail becomes a finalized, immutable segment (stable SHA ⇒ idempotent retries).
- **`can.cpp`** decodes the multiplexed result burst (0x710 Status, 0x711 cells uint16 LE,
  0x712 temps int16 LE ×0.1°C, 0x713 fault masks) into a per-scan accumulator and hands it
  to `das_sync` on the fault frame (or at the result timeout) — off the CAN Rx work.

### Changed
- **`analytics.cpp`**: removed the cloud publisher (`publish_batch`, the v:2/v:3 JSON
  serializer, `is_das_payload_id`, RAM-only publish path). Kept the software RTC
  (`now_epoch_ms`/`rtc_record_sync`) and `flush_ring_to_sd()`. `mqtt_service()` → **`sd_flush_service()`**:
  drains the RAM ring into the SD NDJSON log every ~1 s, no network work.
- **`wifi_mqtt.*`**: stripped to a **WiFi-STA + one-shot SNTP** helper (`wifi_ensure_connected`,
  `wifi_connected`, `wifi_rssi`, `wifi_note_result`); the esp-mqtt cloud client + QoS1 publish
  were removed. The control MQTT session now lives in `das_sync` (its own esp-mqtt client).
- **`main.cpp`**: `sd_lock_init()` in `setup()`; `publish_mqtt_task`→**`sd_flush_task`**;
  `das_sync_init()` in `init_task` after SD mount (also sets `ds_seg_hw` on first boot so a
  pre-existing/stale segment backlog is never uploaded — replaces the old cloud-cursor purge).
- **IoT→LCD status heartbeat (0x7A0) now reports LOCAL SERVER status.** `IOT_HB_FLAG_MQTT`
  (bit1) → **`IOT_HB_FLAG_SERVER`** (same 0x02 bit) — set from `das_sync_server_up()` (the
  local control-MQTT session), not the old cloud broker. Same bit value, so the LCD needs no
  code change; just relabel its uplink chip "Server"/"Uplink". The `mqttConnected` global was
  removed (replaced by `das_sync_server_up()`).
- **Removed the dead cloud MQTT code.** `mqtt.h` no longer defines the cloud broker
  (`MQTT_SERVER`/`MQTT_PORT`/`MQTT_CLIENT_ID`/`MQTT_USERNAME`/`MQTT_PASSWORD`/`SETUP`), the
  data/OTA topics, `MQTT_PAYLOAD_VERSION`, or the batch tunables — it now holds only the live
  factory-WiFi service config. `analytics.cpp` dropped the unused `serialNumber`/`signalQuality`
  globals. (The only references to the removed broker macros were in the EC200 modem-OTA path in
  `ota_bootloader.cpp`, which is compiled out via `OTA_BL_MODEM_OTA 0`; if that path is ever
  revived, restore the macros it needs — noted at the top of `mqtt.h`.)

### Notes / caveats
- **No server change made (by request).** The reference server files every upload as a
  barcode "module" and scores it into analytics, so the raw/event-log files uploaded by the
  daily job appear as bogus modules and skew the dashboard yield/module counts. Only the
  per-barcode CSVs are meaningful there. Fixing this cleanly would need a small Node-RED edit
  (route `.ndjson`/`.log` to an archive folder, skip the manifest) — deferred.

## [IOT_Board] 2026-08-27 — Restart command from the LCD (0x2D0)

### Added
- **`0x2D0` restart handler** (`can.cpp` `can_receive()`, `main.h` `IOT_RESTART_CMD_ID`):
  a standard-11-bit frame carrying magic `'R''S'` (0x52 0x53), sent by the LCD's new
  "Restart system" button. On receipt the IoT `esp_restart()`s immediately (plain reboot
  into the running app — distinct from the `0x0C` bootloader-enter, which flashes). The
  LCD reboots itself right after, so both boards come back together. Intercepted before
  the DAS whitelist, alongside the baud (0x2E0) and bootloader-enter (0x0C) commands.
  ⚠ ID/magic must match `IOT_RESTART_*` in the LCD's `ba100_can.h`.

## [IOT_Board] 2026-08-27 — Align to harness_jig v5 DBC (0x713 raw-sample detail)

New DBC `docs/harness_jig_v5.dbc` (+ README) added. The MQTT wire format is
unchanged: raw DAS frames (full 8 bytes) are forwarded verbatim, so the v5
`0x713` bytes 4-6 already reach the backend without any payload change. Deltas:

### Added
- **0x713 raw-sample detail in the event log** (`can.cpp` `das_log_semantic()`): decode
  the new v5 signals byte4 `CellRawFaultMask`, byte5 `TempRepeatabilityFaultMask`,
  byte6 `PassingReadingCount` (only when the jig sends the full DLC≥7 frame). The
  `/events` HARNESS line now includes `raw cell=… temp=… pass=N/10`, and a FAIL whose
  averaged masks (bytes 0/1) are clean is logged as a distinct "raw-sample fault" entry.
  Edge-trigger state extended to cover the new bytes so changes still log once.

### Changed
- **Comments only** (`main.h`): the barcode-ID note that claimed we "deliberately do NOT
  follow" the 0x700→0x600 change was stale/contradicted by the code — corrected to state
  the IDs are 0x600-0x602 per the v5 DBC. Noted temps are signed int16 and 0x713 is now
  a 6-signal frame. No functional change (IOT is a passive raw-frame forwarder).

## [IOT_Board] 2026-08-19 — DBC-only frames in MQTT (drop foreign IDs)

### Added
- **Publish-time DBC-ID filter** (`analytics.cpp` `is_das_payload_id()` in `publish_batch()`):
  only `0x600/0x601/0x602` (barcode), `0x710` (status/PASS-FAIL), `0x711` (cells), `0x712`
  (temps), `0x713` (fault) are ever serialized into the MQTT `frames[]`. Any foreign frame
  (e.g. a stale charger-firmware `0x18A1` backlog) is dropped; a batch containing no DBC frames
  is drained (cursor advanced) without publishing, so garbage never reaches the uplink. This is
  belt-and-suspenders on top of the CAN-capture whitelist + extended-frame reject in `can.cpp`.
- **One-time SD stale-backlog purge** (`main.cpp` init_task, keyed on NVS `sdlog_proto`): on the
  first harness boot, any pre-existing SD-log backlog (from a previous firmware/protocol) is
  marked already-uploaded so it isn't sent. Skipped entirely when no SD card is present.

## [IOT_Board] 2026-08-19 — incomplete_read flag + full result frames

Raw-frame payload kept (no decoding). A test result's frames — barcode (0x600-602) plus the
jig's PASS/FAIL (0x710), cells (0x711), temps (0x712), fault (0x713) — are batched into one
message; the `0x710` PASS/FAIL frame is captured in `frames[]` when it arrives.

### Added
- **`incomplete_read`** in the MQTT payload (`analytics.cpp`): `true` when a scan carried ONLY
  the barcode and the jig's result frames never arrived within `DAS_RESULT_TIMEOUT_MS` (2.5 s).
- **Scan-complete vs incomplete trigger** (`can.cpp`): a scan begins on the barcode's first
  frame (`0x600`, `das_scan_begin()`); it publishes as COMPLETE on the temps frame (`0x712`,
  `g_scan_incomplete=false`) or, if no results arrive, as INCOMPLETE after the timeout
  (`das_poll_result_timeout()` from the CAN Rx task → `g_scan_incomplete=true`). Fixed the
  RAM-only path that used to stream partial ring contents; now it waits for a completed result.

### ⚠ Diagnosis (results not reaching the IoT)
- In the reported capture only `0x600-602` (LCD→jig barcode) appeared and `canStatus:false` — the
  jig's result frames (`0x710/0x711/0x712/0x713`) and its `0x720` heartbeat were NOT received by
  the IoT. Since the IoT clearly receives the LCD's barcode, its CAN RX works, so the jig frames
  aren't on the IoT's bus segment / the jig isn't transmitting them (CAN wiring, baud, or no DUT).
  With this build a barcode-only scan is now published with `incomplete_read:true` instead of a
  bare barcode message.

## [IOT_Board] 2026-08-19 — Accurate heartbeat status sources

### Changed
- **`mqttConnected` is now event-driven** — set on esp-mqtt `MQTT_EVENT_CONNECTED` and cleared
  on `MQTT_EVENT_DISCONNECTED` (plus on failover teardown / `wifi_mqtt_stop`) in `wifi_mqtt.cpp`.
  `analytics.cpp` no longer writes it. So the 1 s CAN heartbeat (`0x7A0`, bit1) reflects the
  LIVE broker session, not a per-cycle snapshot.
- **`canStatus` is now refreshed from the CAN Rx task** (`can_receive_task`, ~1 s) instead of
  only during MQTT uploads. A quiet bus now flips `canStatus` false within the 10 s window even
  when nothing is being uploaded, so the heartbeat's CAN-OK flag (`0x7A0` bit2) stays truthful.
  `mqtt_service()` no longer calls `check_can_status()` (single-writer, no cross-task race).

## [IOT_Board] 2026-08-19 — MQTT-driven WiFi failover

### Added
- **Auto WiFi failover on MQTT failure** (`wifi_mqtt.cpp`). After `WIFI_MQTT_MAX_MQTT_FAIL`
  (3) consecutive MQTT failures on the current network — broker won't connect, or a publish
  isn't ACKed — `mqtt_note_result()` advances to the next credential slot and tears down the
  broker+WiFi so the next cycle reconnects there, rotating until MQTT actually succeeds. A WiFi
  that merely *associates* but has no working broker path no longer traps the uplink.
  `ensure_wifi_connected()` now starts at the active slot (`s_active_idx`) and rotates. Any MQTT
  success (broker connect or publish ACK) resets the counter. Verified: `IoT→LCD 0x7A0` status
  frame (ID/magic/flags/counter) matches the LCD's RX handler in `LCD/src/ba100_can.cpp` — no
  change needed.

## [IOT_Board] 2026-08-19 — Multiple WiFi networks + web-set primary

### Added
- **Three WiFi credential slots, tried in order** (`wifi_mqtt.cpp` `build_sta_cred_list()` +
  `ensure_wifi_connected()`): slot 1 = primary (NVS `iot_cfg`/`sta_ssid`,`sta_pass`, else
  `CONFIG_STA_WIFI_SSID/PASS`); slots 2 & 3 = compile-time backups
  `CONFIG_STA_WIFI_SSID2/PASS2`, `CONFIG_STA_WIFI_SSID3/PASS3` (empty = disabled). On connect
  it attempts each network (each up to `WIFI_MQTT_CONNECT_TIMEOUT_MS`) until one associates.
- **Primary WiFi settable from the factory web page** (`wifi_service.cpp`): the Settings form
  gained SSID + password fields writing `iot_cfg`/`sta_ssid`,`sta_pass`; the password is only
  updated when a new one is entered (blank keeps the current). Applies next boot.

## [IOT_Board] 2026-08-19 — IoT→LCD status heartbeat on CAN

### Added
- **CAN status heartbeat** (`can.cpp` `send_iot_heartbeat_can()`, `IOT_HEARTBEAT_ID = 0x7A0`
  in `main.h`). The IoT board now TXes one low-priority frame every 1 s so the LCD/harness
  tester knows the IoT is powered/on-bus and whether a tablet is connected. DLC 4:
  `data[0]=0xA5` magic, `data[1]=flags` (bit0 tablet connected, bit1 MQTT/WiFi up, bit2 CAN
  OK), `data[2]=rolling counter`, `data[3]=reserved`. Emitted from the repurposed
  `diagnostic_can_task` (now 1 s; still logs local status ~every 45 s).
- This is the **only** frame the IoT transmits — a deliberate exception to the
  passive-listener rule, at a low-priority ID so it never contends with DAS result frames.
  MQTT stays scan-only (the earlier MQTT-heartbeat attempt was reverted).

### ⚠ LCD-side change required
- The LCD must add an RX handler for `0x7A0` to display IoT-on / tablet-connected status.
  Frame spec is documented at `IOT_HEARTBEAT_ID` in `include/main.h`.

## [IOT_Board] 2026-08-19 — Publish per-scan only + trimmed MQTT payload

### Changed
- **MQTT now publishes ONE message per test result**, not continuously. `mqtt_service()` only
  publishes when a new scan's voltage+temperature has arrived — `handle_rx_message()` sets
  `g_scan_ready` on the temps frame (0x712, the last of a result). When idle it does no network
  work.
- **Heartbeat (0x720) excluded from MQTT**: it's no longer pushed into the ring (so it can't
  flood a per-scan message) and never triggers a publish. It's still decoded to the SD event log
  (`das_log_semantic`). The periodic MQTT heartbeat/liveness message was removed.

### Removed from the MQTT payload
- `latitude` / `longitude` (GPS — none on this board), `drop`, `seq`, and `version`
  (`get_code_version_from_can` deleted; the `droppedBaseline` bookkeeping too).
- Payload is now: `v`, `serialNumber`, `signalQuality` (WiFi RSSI), `canStatus`, `deviceStatus`,
  `iot_version`, `frames[]` (`t`/`id`/`d`). The `frames` array carries the complete result
  (barcode 0x600–602, status 0x710, cells 0x711, temps 0x712, fault 0x713).

### Note
- Between scans the board is silent on MQTT (no liveness message) — by design.

The address lock blocked the tablet's manual connect (Android connects with a randomized
address). Reverted to plain HID advertising — any tablet pairs manually from its Bluetooth
settings, as before.

### Removed
- `ble_hid_device.cpp`/`.h`: the tablet-address lock (`CONFIG_BLE_TABLET_ADDR`, the
  onConnect/onAuthenticationComplete address checks, `parse_bd_addr`, `load_tablet_addr`,
  `s_tablet_addr`/`s_tablet_lock`) and the boot-time **bond wipe** — bonds now PERSIST, so the
  tablet stays paired across reboots after one manual connect. `wifi_service.cpp`: the web
  tablet-address field + its NVS helpers.
- Kept: BLE **security** (Just-Works bonding + Secure-Connections encryption) — required or the
  tablet gets `GATT_INSUF_AUTHENTICATION` and no keystrokes flow. `onConnect`/`onAuthenticationComplete`
  are back to simple status logging.

## [IOT_Board] 2026-08-19 — BLE HID locked to one tablet (web-settable)

### Added
- **Tablet lock** (`ble_hid_device.cpp`): the HID keyboard now serves only the configured
  tablet — `SrvCB::onConnect` (param overload) compares the peer address to the target and
  immediately disconnects any other device (it keeps advertising for the real tablet). Default
  `CONFIG_BLE_TABLET_ADDR = "D8:E3:74:57:DF:91"`, overridable at runtime from NVS
  (`iot_cfg`/`tablet_addr`); a blank/invalid value disables the lock (accept any). The peer
  address is logged on every connect/reject for diagnostics.
- **Web-settable tablet address** (`wifi_service.cpp`): the factory-mode Settings form gained a
  "BLE tablet address" text field; `/config` validates and writes `iot_cfg`/`tablet_addr`
  (blank clears the lock). Applies on next boot.

### Fixed (RPA-safe enforcement)
- The lock was initially checked in `onConnect`, but Android connects with a Resolvable Private
  Address first — so a legitimate tablet was dropped before pairing (no popup, no connect log).
  Moved enforcement to `SecCB::onAuthenticationComplete`, where the resolved IDENTITY address is
  known: `onConnect` is now permissive (logs the peer), and a mismatched identity is disconnected
  + its bond removed after pairing. Every connect/auth logs the address for diagnostics.

## [IOT_Board] 2026-08-19 — BLE HID pairing fix (bonding + encryption)

On-hardware test showed the tablet connecting but the HID service failing with
`GATT_INSUF_AUTHENTICATION` (+ `BTM_GetSecurityFlags false` flood) — no keystrokes.
Root cause: the peripheral-only HID port never configured BLE security, but HID
requires a bonded/encrypted link.

### Fixed
- `ble_hid_device.cpp`: added Just-Works bonding + Secure-Connections encryption via the
  Bluedroid API — `BLESecurityCallbacks` + `BLESecurity` (`ESP_LE_AUTH_REQ_SC_BOND`,
  `ESP_IO_CAP_NONE`, init/resp enc+id key masks), applied after `BLEDevice::init()` (and on
  stack restart). Added a boot-time bond wipe (`esp_ble_get/remove_bond_device`) to clear the
  corrupt bond left by the pre-security build, so pairing is fresh each boot (matches the LCD).
- NOTE for re-test: on the tablet, **Forget** the device once (its stale LTK won't match after
  reflash), then re-pair.

## [IOT_Board] 2026-08-19 — EC200 purged, WiFi-only MQTT

Removed the EC200 LTE modem entirely; MQTT (data uplink) is now WiFi-only. Verified the
barcode high-priority read matches the LCD's `ba100_send_barcode()` broadcast.

### Removed — app path (WiFi-only MQTT)
- **`analytics.cpp` rewritten** to a lean WiFi-only uploader: all EC200 AT-command code
  (send_AT_command, uart_gsm_event, gsm_* line parsing, check_modem/network/signal,
  QMTOPEN/QMTCONN/QMTPUBEX, NTP/GPS-over-modem, ensure_mqtt_ready, ec200_publish_json,
  receive_mqtt_message, etc.) deleted. Kept: the software RTC/`now_epoch_ms()`, the v:2 JSON
  serializer, and the ring→SD store-and-forward cycle — `publish_batch()` now sends only via
  `wifi_mqtt_publish()` (QoS1). Time comes from the WiFi SNTP sync. `mqttConnected` now
  reflects the WiFi broker session; `signalQuality` = WiFi RSSI.
- **`main.cpp`**: removed the GSM UART config/install, EC200 power-pulse, `uart_gsm_event_task`,
  and the `check_modem()`/`mqtt_init()` calls in init_task (WiFi MQTT is brought up lazily in
  `mqtt_service`). **`main.h`**: removed GSM/EC200 UART + pin defines, the modem externs
  (modemStatus/networkStatus/signalStatus) and LTE prototypes.

### Removed — factory bootloader (`ota_bootloader.cpp`)
- The EC200 modem + MQTT/HTTPS remote-download OTA path is **compiled out** via
  `#define OTA_BL_MODEM_OTA 0` (guards around ~1,600 lines: gsm_*/modem_*/mqtt_*_bl/http_*/
  sha256_verify/bl_ master-flasher/fw_*/do_can_ota_internal/mqtt_poll_task/can_ota_task).
  `ota_bootloader_run()`'s modem tail was replaced with a WiFi-web-updater idle loop; the
  app0-side result publishers and the trigger-reboot status publish now log to the event log
  instead of AT+QMTPUB. **Zero EC200 code is in the binary.** The guard keeps the source
  physically present so the critical CAN-flash bootloader is not destabilized.
- **CAN-flash bootloader, CAN self-update (0x0C), partition scheme, and the factory WiFi web
  updater are unchanged** — these are the remote-update paths now.
- Static RAM freed (the modem's 32 KB MQTT accumulator + AT/download buffers left `.bss`):
  build RAM 40.6% → **26.9%**, Flash 82.1% → **76.6%**.

### Verified
- Barcode high-priority read matches the LCD: `ba100_send_barcode()` TXes `0x600/0x601/0x602`
  (24 B, 3×DLC-8, ASCII zero-padded); `barcode_ingest()` reassembles and types to the tablet
  on the CAN Rx path before ring/log/MQTT.

## [IOT_Board] 2026-08-19 — BLE HID barcode-to-tablet + DBC v4 alignment

Adds a BLE HID keyboard peripheral so the scanned barcode is typed to the tablet, and
aligns CAN handling to `harness_jig__4_.dbc`. (Project reorganised: the LCD firmware now
lives in the sibling `../LCD/` folder; this is `IOT_Board/`.)

### Added
- **BLE HID keyboard peripheral** (`src/ble_hid_device.cpp`, `include/ble_hid_device.h`),
  adapted PERIPHERAL-ONLY from the LCD project's `ble_hid_device.*` (no scanner central —
  it owns the radio via its own `BLEDevice::init` on the 16 KB HID task). Advertises as
  `CONFIG_BLE_HID_NAME` ("Coulomb IOT HID"); the tablet pairs with it. Brought up in
  `init_task` after `mqtt_init` and before the SD mount (DRAM ordering). Classic Bluedroid
  BLE API — no `lib_deps` change.
- **Barcode → tablet HID forward (high priority)** (`can.cpp`). The barcode CAN frames
  (0x600/0x601/0x602, DLC 8, ASCII zero-padded) are reassembled on the CAN Rx path and,
  the instant a code completes (on 0x602, or a 150 ms gap fallback), typed to the tablet
  via `ble_hid_dev_type(code + "\n")` — before the ring/log/MQTT work, like a scanner
  wedge. The barcode frames STILL flow into the ring → SD → MQTT as normal.

### Changed
- **`0x713` Jig_HarnessFault decoder** updated to the revised `harness_jig__4_.dbc` (1811)
  layout: byte0 `CellFaultMask` (8 bits), byte1 `TempFaultMask` (bits 0-3), byte2 reserved,
  byte3 `InitFailed`. The old `badRange/vblkMismatch/lowest/highest` fields are gone; the
  event-log entry now reports `cellMask`/`tempMask` (and a distinct "DAS init FAILED" line
  when `InitFailed` is set). Still edge-triggered.

### Notes
- Barcode IDs stay `0x600/0x601/0x602` — now CONFIRMED correct by `harness_jig__4_.dbc`
  ("ID changed from 0x700 to 0x600"). The earlier mismatch note is resolved.
- Status/cells/temps/heartbeat IDs (0x710/0x711/0x712/0x720) unchanged in the new DBC.
- **BLE + WiFi coexistence**: BLE HID is always on; the WiFi MQTT fallback brings the radio
  up only when the EC200 can't connect. They time-share the 2.4 GHz radio — verify DRAM
  headroom on hardware when both are active.

## [IOT_Board] 2026-08-17 — harness fault + heartbeat handling

Aligned to the `harness_jig.dbc` protocol reference (frames 1808/1809/1810/1811/1824).

### Added
- **Harness fault frame `0x713`** (`Jig_HarnessFault`, DBC 1811) added to the DAS capture
  whitelist (`DAS_FAULT_ID` in `main.h`, `is_das_id()` in `can.cpp`), so it now flows into
  the lossless ring → SD log → MQTT (`v:2`, raw) like the other frames.
- **Decoded fault + heartbeat detail in the event log** (`das_log_semantic()` in `can.cpp`).
  `0x713` is logged edge-triggered (on fault set/clear, with BadRangeMask + lowest/highest
  flagged channel). `0x720` (`Jig_Heartbeat`, DBC 1824) is decoded (counter, CAN TEC/REC/LEC,
  Tx-queue depth) and logged as a ~60 s health summary plus an immediate throttled WARN on any
  CAN-error edge — so the 2 Hz heartbeat never floods the SD event log. MQTT stays raw `v:2`
  (both frames' details reach the backend via their raw bytes; the wire format is unchanged).

### Notes
- Barcode frames kept at `0x600/0x601/0x602` by request, even though `harness_jig.dbc` lists
  them as `0x700/0x701/0x702` — barcode handling intentionally left unchanged.

## [IOT_Board fork] 2026-08-17 — harness telemetry uploader (forked from `ESP32_S3_test10`)

Forked to make the Coulomb IOT board a **passive listener** on the harness test-station
CAN bus that captures the DAS harness protocol and uploads it to MQTT in the unchanged
`v:2` wire format. Runs on the IOT board hardware (EC200 + SD + CAN), NOT the Waveshare 7"
board of the parent Harness Tester project.

### Added
- **DAS harness capture** (`src/can.cpp`, `include/main.h`). `handle_rx_message()` now
  whitelists the 11-bit standard DAS IDs — `0x600/0x601/0x602` (barcode), `0x710` (status),
  `0x711` (cell voltages), `0x712` (temperatures), `0x720` (heartbeat) — and pushes each raw
  frame into the existing lossless ring. Wire format (`v:2`) is unchanged; the backend decodes.
- **WiFi MQTT fallback** (`src/wifi_mqtt.cpp`, `include/wifi_mqtt.h`). When the EC200 can't
  reach the broker, `mqtt_service()` brings up WiFi STA + esp-mqtt (QoS1) to the SAME
  broker/topic/credentials and publishes the identical payload. esp-mqtt chosen over
  PubSubClient for QoS1 PUBACK (ack-then-commit). STA creds: compile defaults in `wifi_mqtt.h`
  (`CONFIG_STA_WIFI_SSID/PASS`) + NVS override (`iot_cfg`/`sta_ssid`,`sta_pass`). Own one-shot
  SNTP anchors the shared software RTC so frame timestamps are valid without the modem.
  `rtc_record_sync()` was made non-static so the WiFi path can anchor the same clock.

### Changed
- `publish_batch()` split into a shared `v:2` JSON builder + a transport selected by
  `g_uplink` (EC200 `ec200_publish_json()` / WiFi `wifi_mqtt_publish()`). EC200 is preferred;
  WiFi is fallback-only.
- `mqtt_service()` uplink selection reworked: a non-restarting `ec200_try_connect_soft()` is
  tried first, then WiFi; the original hard modem recovery (`AT+CFUN=1,1` + `ESP.restart()`)
  fires only after BOTH uplinks stay down for `WIFI_FALLBACK_HARD_RECOVERY_CYCLES` (~2 min), so
  an active WiFi uplink is never rebooted out from under us. EC200-only per-cycle AT queries
  (`check_signal_quality`/`get_timestamp`/`get_location`) are skipped on the WiFi path.

### Removed
- **Sleep / low-power feature** entirely: `power_manager.cpp/.h` and `shutdown_manager.cpp/.h`
  deleted; `power_manager_task` (CAN-idle detection → graceful shutdown → modem power-off → CPU
  throttle → restart-on-wake), the `pm_notify_can_activity()` call, `suspend_application_tasks()`,
  and every `shutdown_requested`/`SHUTDOWN_READY_*` task guard removed. The board now runs
  continuously at full clock and never idles/sleeps or powers the modem down.
- **BLE** entirely (`src/ble.cpp`, `include/ble.h` deleted; task creation, handles, task bodies,
  `ble_stop()` idle call, BLE includes/prototypes, and `SHUTDOWN_READY_BLE_*` bits removed).
  `SHUTDOWN_READY_ALL` recomputed to `0x73` so `wait_for_tasks_ready()` doesn't hang.
- **Charger/BMS CAN protocol**: `is_target_id`/`is_lifetime_id`, `process_voltage_data`,
  voltage/lifetime/send-on-change logic, the BMS timestamp/desktop-config CAN transmits, and
  the IOT-identity diagnostic frames (`0x18A1FE02/03`). The board no longer transmits on the
  harness bus at all. `bms_timestamp_sync()` kept (name/signature) but now only advances the RTC.

### Unchanged (kept in lock-step with `ESP32_S3_test10`)
- Partition table (`factory`+`ota_0`+`ota_1`), CAN-flash bootloader, the `0x0C` enter trigger
  and `0x2E0` baud command, the lossless ring + SD store-and-forward, and the MQTT `v:2` payload.

---

# (Inherited history from ESP32_S3_test10)

Comparing `ESP32_S3_test10` against the prior version, `ESP32_S3_test9`.

## [Unreleased] — lossless, filtered, linearly-ordered MQTT telemetry

### Added
- **Lossless CAN ring buffer** (`include/frame_buffer.h`, `src/frame_buffer.cpp` — new files).
  Every accepted CAN frame is appended to a 500→**2000**-frame FIFO ring with an absolute,
  never-wrapping sequence number. Replaces the old model of dumping the live `CanMsgs[]`
  snapshot straight to MQTT.
- **QoS1, acknowledged batch publisher** (`analytics.cpp`: `publish_batch()`, `mqtt_service()`).
  Frames are drained from the ring in batches (`MQTT_BATCH_MAX`, 40→**80** frames), published via
  `AT+QMTPUBEX` with `MQTT_QOS=1`, and only removed from the ring (`fb_commit()`) after the
  broker acks. A failed/timed-out publish leaves the batch buffered for retry — no silent loss.
- **`seq` and `drop` in the wire payload.** `seq` is the ring's absolute sequence number of the
  batch's first frame — a stable, monotonic ordering key that survives network reordering/retries
  (unlike wall-clock time). `drop` is frames lost to ring overflow *since the last acked publish*
  (delta, not lifetime total) — first genuine loss-visibility in the payload.
- **Millisecond, strictly-monotonic per-frame timestamps** (`analytics.cpp`: `now_epoch_ms()`).
  Anchored at each NTP/GPS sync and advanced via `esp_timer_get_time()` (IDF-native monotonic
  µs clock — not `millis()`), with a strict-increment guard so no two frames ever share a
  timestamp. Previously the payload had no per-frame time at all; frames were sent as an
  unordered full snapshot once a second.
- **Send-on-change filtering** (`can.cpp`: `CAN_SEND_ON_CHANGE`, default on). A frame only enters
  the ring if its data differs from the last frame *sent* for that CAN ID. Steady-state repeats
  (e.g. unpopulated all-zero channels) are suppressed; the first value, every genuine change, and
  real→zero transitions still go out. `CanMsgs[]` (BLE/serial/diagnostics) is unaffected — it
  still updates on every frame.
- **Lifetime-aggregate exclusion** (`can.cpp`: `CAN_EXCLUDE_LIFETIME`, default on). The BMS's
  lifetime min/max history block (CAN matrix Sr 77–158, IDs `0x18A14D02`–`0x18A19E02`, dev_idx
  `0x4D`–`0x9E`) is historical, not live telemetry, and made up roughly half of every scan cycle.
  It's now kept out of the uplink payload entirely (still stored in `CanMsgs[]` locally).
- **Master-cycle-aware flush trigger** (`mqtt.h`: `MQTT_CYCLE_IDLE_GAP_MS`, `analytics.cpp`:
  `mqtt_service()`). Primary flush trigger is now "master frames have paused" (idle-gap on
  `canReceivedTime`, which only advances on accepted target-ID frames — unrelated bus traffic
  can't trigger it), so each MQTT message tends to carry one coherent capture window instead of
  an arbitrary slice of a fixed 1000 ms tick. The old fixed-period publish and ring watermark are
  kept as backstops for a bus that never goes idle.
- **Wire-format version marker** (`mqtt.h`: `MQTT_PAYLOAD_VERSION = 2`, payload key `"v"`). Lets
  the backend branch cleanly between this format and the old unversioned snapshot shape.
- **Build tooling**: `merge_firmware.py` + `platformio.ini` `extra_scripts` — auto-generates a
  single merged `full_flash.bin` after every build and adds a `pio run -t upload_full` target
  that erases the chip and flashes bootloader + partitions + boot_app0 + firmware (factory *and*
  ota_0) in one command, replacing a 5-step manual `esptool` sequence.

### Changed
- **MQTT publish model**: fire-and-forget (`AT+QMTPUBEX=0,0,0,0,...`, QoS0, no ack, no retry,
  1000 ms fixed tick, `mqtt_publish_messages()`) → acknowledged, retry-safe, cycle-aware batching
  (`mqtt_service()` / `publish_batch()`). This is the root fix for out-of-order / missing data in
  server-side logs — the old path had no ordering key and no loss detection at all.
- `FB_CAPACITY` raised 500 → **2000** frames (~48 KB SRAM) for longer outage tolerance under weak
  signal, now that `CAN_SEND_ON_CHANGE`/`CAN_EXCLUDE_LIFETIME` also reduce the effective frame rate
  feeding the ring.
- `MQTT_BATCH_MAX` raised 40 → **80** frames per publish, so each successful publish under a weak
  or intermittent link carries more data. (JSON buffer and stack usage scale with this via a
  `static_assert` in `analytics.cpp` — see Notes.)

### Fixed
- Per-frame CAN data had no timestamp at all in test9; frames arriving within the same publish
  cycle were indistinguishable in time and couldn't be ordered downstream.
- No way to detect or quantify lost data — a stalled/failed publish under test9's fire-and-forget
  model silently dropped that cycle's entire snapshot with no record.

### Notes / recommendations
- `IOT_VERSION` (`include/main.h`) is still `"1.0.0"` in both test9 and test10 despite this being
  a breaking wire-format change (`v:2`) — consider bumping it so devices/backend can distinguish
  builds at a glance.
- `MQTT_BATCH_MAX = 80` produces a ~5.5 KB JSON payload per publish — confirm the modem's
  `AT+QMTPUBEX` accepts that size on your EC200U firmware before relying on it in the field; some
  Quectel builds cap a single publish around 4 KB.
