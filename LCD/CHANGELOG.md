# Changelog — ChargerLCD_combinedV2.1.0

Every change is logged with **which LCD it was made for / tested on**, so that
when a change made for one LCD breaks the other, this file shows exactly what
was touched and for which hardware.

## 2026-09-18 — Per-scan raw-CAN capture (one file per barcode)

| LCD | Change |
|---|---|
| 7" | **Per-scan raw CAN logging** (`sd_logger.*`): new `sd_logger_session_open(barcode)` / `sd_logger_session_close()` + `sd_logger_frame(tx,id,data,dlc,decoded)`. Each scan opens `/barcodes/<barcode>_<stamp>.csv` and closes the previous one, so every CAN frame (RX + our TX) from one barcode to just before the next is captured (`Time,Dir,ID,DLC,Data,Decoded`). Modelled on the EOL-Tester fork's logger. |
| 7" | `ba100_can.cpp` logs every relayed RX frame (poll loop) and every TX (`bridge_send`) via `sd_logger_frame`; `harness_conn.cpp` `process_scan()` opens the session before the barcode TX. Bridge link-heartbeat/control frames are not CAN and are not logged. Queue bumped 48→64. |

## 2026-08-27 — Main-page: "Restart system" button + full-width barcode

| LCD | Change |
|---|---|
| 7" | **Idle slot below the logo is now a "Restart system" button** (`ui_fallback.cpp`), replacing the "Scan a barcode to test" text. Tap → TX a restart command to the IoT over CAN (new `0x2D0` frame, magic `'R''S'` — see `ba100_can.*` `ba100_send_iot_restart()`), then `esp_restart()` the LCD, so both boards reboot together (`harness_conn.cpp`). The button is shown **only while idle** — during a check the slot shows "Checking…" and the button is hidden (`check_apply_style()`), so it can't be tapped mid-scan. New UI API `ui_take_restart_system_request()`. |
| 7" | **Barcode card:** dropped the "BARCODE" caption; the scanned code now fills the whole card, centered, font auto-sized to length (montserrat 48→12 by length in `ui_set_barcode()`). |
| both | New CAN ID `0x2D0` (`IOT_RESTART_CMD_ID`) — the matching RX handler is in the IoT `can.cpp` / `main.h`. Clear of the flasher (0x0B/0x0C/0x0D) and baud (0x2CC/0x2CE/0x2E0) IDs. |

## 2026-08-27 — Align to harness_jig v5 DBC (0x713 raw-sample detail + signed temps)

New DBC `docs/harness_jig_v5.dbc` (+ README) added. Barcode IDs (0x600-0x602),
0x710 echo reassembly, and 0x720 heartbeat already matched v5 — the real deltas:

| LCD | Change |
|---|---|
| 7" | **0x713 `Jig_HarnessFault` now has 6 signals.** Parse the new raw-sample bytes 4-6 (`CellRawFaultMask`, `TempRepeatabilityFaultMask`, `PassingReadingCount`) in `ba100_can.cpp` (only when DLC≥7; `fault_ext_valid` gates them). Added the three fields to `BA100_State_t`. These come from the RAW per-sample checks that drive the 0x710 Status, so they explain a FAIL when the averaged masks (bytes 0/1) are clean. |
| 7" | **Data page:** when the averaged masks are clean but the raw detail flags a problem (`ui_fallback.cpp`), show a raw-fault line (cells/temps + `pass N/10`) instead of a misleading "Harness OK". |
| 7" | **SD data log:** three new CSV columns `cell_raw_fault_mask,temp_repeat_fault_mask,passing_reading_count` (`sd_logger.cpp` header + `harness_conn.cpp` row; `DATA_EMPTY_TAIL_COLS` +3, row buffer 240→288). A FAIL explained only by raw detail is also surfaced in `/events`. |
| 7" | **Temps are SIGNED** (`0x712 @1-`, 0.1 °C): `temp_raw[]` is now `int16_t` (was `uint16_t`) so sub-zero readings decode/log/display correctly (`ba100_can.*`, `harness_conn.cpp` `%u`→`%d`, `ui_fallback.cpp`). |
| 7" | De-provisionalised the `0x710` Status comment (v5 `VAL_TABLE_` confirms 0=FAIL, 1=PASS; added `DAS_STATUS_FAIL`). |

## 2026-08-20 — Relabel MQTT chip → "Uplink" (honest until IoT reports live state)

The IoT's `0x7A0` MQTT flag (bit1) currently means "last telemetry upload
succeeded" — `mqttConnected` is only written inside `mqtt_service()` during an
on-demand publish, so it's false from boot until the first upload and stale
between scans (not a live broker session). Until the IoT reports live MQTT state
(from esp-mqtt connect/disconnect events), the LCD top-bar chip is renamed
**"Uplink"** (`LV_SYMBOL_UPLOAD`) so it isn't misread as a live MQTT link.

| LCD | Change |
|---|---|
| 7" | `ui_fallback.cpp`: top-bar `UI_LINK_MQTT` chip label `MQTT`→`Uplink`, icon → upload. Enum name unchanged (one-line revert once the IoT sends live MQTT status). Build: RAM 19.7% / Flash 52.4%. |

## 2026-08-20 — Fix stuck-on-Checking (long link) + mismatch clarity

| LCD | Change |
|---|---|
| 7" | **Stuck-on-"Checking…" fixed.** The `0x710` echo caps at 20 bytes (Barcode_0..19), but completion checked against `DAS_BARCODE_MAXLEN=24` — a link ≥20 chars has no zero terminator, so the reply never finalized and `s_check_pending` never cleared. Added `DAS_ECHO_MAXLEN=20`; echo assembly/completion/`take_das_result` now use it, so a full 20-byte echo finalizes. |
| 7" | **Check timeout** (`CHECK_TIMEOUT_MS=5 s`): if no/partial DAS reply arrives, the UI stops "Checking…" and shows FAIL (event logged `check timeout …`), so it can never stick — covers a partial reply <20 chars with no zero, or a silent master. |
| 7" | **Mismatch made explicit:** a barcode whose BMS echo differs from the scan now always shows **MISMATCH** (wins over the status byte), logs a `barcode MISMATCH` event, and prints `[CHK] scanned=… echo=… status=… -> …` to UART for diagnosis. (A too-long link → truncated 20-char echo ≠ full scan → correctly MISMATCH.) |
| 7" | Build (PowerShell, `LCD/`): clean `.bin`, RAM 19.6% / Flash 52.3%. |

## 2026-08-20 — Tablet fully removed; IoT 0x7A0 heartbeat drives its state

The tablet HID now lives on the IoT board, not the LCD. Removed ALL tablet code
(incl. UI) and instead read the IoT's 1 Hz status heartbeat.

| LCD | Change |
|---|---|
| 7" | **Deleted `ble_hid_device.cpp/.h`** (BLE HID peripheral). `harness_conn.cpp` stripped of all tablet code (`poll_tablet`, `ble_hid_dev_*`, `s_last_tab_state`); Restart-Bluetooth now just forces a scanner reconnect (`ble_hid_disconnect` → the DOWN auto-connect loop). |
| 7" | `ba100_can.*`: parse **`0x7A0` IoT heartbeat** (`data[0]=0xA5`, `[1]=flags`, `[2]=rolling counter`; flags bit0=tablet, bit1=MQTT/WiFi, bit2=CAN OK). `iot_alive` = counter advancing within `IOT_STALE_MS` (3 s) — a frozen counter goes stale (stall detect). New `BA100_State_t.iot_*`. |
| 7" | UI: **removed the tablet panel + `ui_tablet_*`/`ui_take_tablet_*` API**. New top-bar **IoT** chip that **blinks green while the IoT is alive** (steady grey when down). Tablet + MQTT chips now driven from the heartbeat flags (`harness_conn_poll` → `ui_set_link`). |
| 7" | Logging: IoT online/offline + tablet-connected/disconnected (via IoT) → events. |
| 7" | Build (PowerShell, from `LCD/`): clean `.bin`, RAM 19.6% / Flash 52.3%. |

## 2026-08-13 — Scanner-only mode: fixed known scanner, tablet disabled

Temporarily focus on the scanner link only. Tablet HID pairing is commented out;
we hard-connect to one known scanner and keep retrying/reconnecting to it.

| LCD | Change |
|---|---|
| 7" | `app_config.h`: new `CONFIG_SCANNER_ADDR "06:9e:0f:32:75:14"`, `CONFIG_SCANNER_NAME "scannerBLE"`, `CONFIG_SCANNER_ADDR_TYPE 0` (public — top bits 00; try 1 if connect fails). |
| 7" | `harness_conn.cpp`: `ble_hid_dev_init()` + `poll_tablet()` wrapped in `#if 0` (tablet off). `poll_scanner` now **auto-connects directly to CONFIG_SCANNER_ADDR while DOWN** (retry every 4 s — covers boot-before-power-on and give-up-after-internal-retries), no scan/pick. Barcode forward gate dropped the tablet requirement (SD gate only); still runs the CAN check + data/event logging. |
| 7" | Tablet UI panel remains but is inert (its pollers aren't drained). Restart-Bluetooth is a no-op while the tablet task isn't started (auto-reconnect covers scanner drops). |
| 7" | Build (PowerShell, from `LCD/`): clean `.bin`, RAM 19.6% / Flash 52.4%. To restore the tablet: flip the two `#if 0` blocks back in `harness_conn.cpp`. |

## 2026-08-13 — Harness fault 0x713 REVISED layout (harness_jig__4_.dbc)

New DBC revised `0x713 Jig_HarnessFault`. (Barcode IDs reverted to 0x600/1/2 in
this DBC = what we already use, so no barcode change. Everything else — 0x710/
711/712/720, time-sync — unchanged.)

| LCD | Change |
|---|---|
| 7" | `ba100_can.*`: `0x713` reparsed to the new layout — byte0 **CellFaultMask** (bit n → cell n reads 0V/≥4V), byte1 **TempFaultMask** (bits 0-3 → temp n deviates), byte3 **InitFailed** (DAS never initialized). Dropped `BadRangeDetected`/`VblkMismatch`/`Lowest`/`HighestFlaggedChannel` + `DAS_FAULT_CH_NONE`. State now `fault_cell_mask`/`fault_temp_mask`/`fault_init_failed`. |
| 7" | UI HARNESS line: **InitFailed → "DAS INIT FAILED"** (checked first, since CellFaultMask reads 0xFF spuriously then), else OK, else lists the flagged cells + temps from the masks. |
| 7" | Logging: data-CSV fault columns are now `cell_fault_mask,temp_fault_mask,init_failed`; events fault line updated (separate "DAS init failed" message). |
| 7" | Repo reorg: the LCD firmware now lives under **`LCD/`** (alongside `IOT_Board/`). Build from `LCD/`, run `pio run` from **PowerShell** (Git-Bash/MSys breaks the esptool reinstall). Clean `.bin`, RAM 19.7% / Flash 52.7%. |

## 2026-08-13 — Harness fault (0x713) + heartbeat diagnostics (harness_jig.dbc)

Aligned to the new `harness_jig.dbc` — added the harness-fault report and richer
heartbeat. (Per request, the barcode-frame ID change 0x6xx→0x7xx and 20-byte/
frame-3 changes were NOT done; barcode TX stays 0x600/1/2 × 24B. Temps left
unsigned too.)

| LCD | Change |
|---|---|
| 7" | `ba100_can.*`: parse **`0x713 Jig_HarnessFault`** (`DAS_FAULT_ID`) — BadRangeDetected/VblkMismatchDetected, BadRangeMask, lowest/highest flagged channel (0xFF=none) → new `BA100_State_t.fault_*`. Cleared in `ba100_clear_das_data()` so a stale fault can't carry into the next test. |
| 7" | `ba100_can.*`: parse the richer **`0x720` heartbeat** (Counter, CAN TEC/REC/LEC, TxQueueDepth) → `hb_*` + `hb_valid`. |
| 7" | UI: data page (cells/temps) gained a **HARNESS** line — green "Harness OK", or red `⚠ BAD RANGE / VBLK MISMATCH (mask 0xNN) ch lo-hi`, driven from state in `ui_apply_state`. |
| 7" | Logging: data CSV gained 5 columns `bad_range,vblk_mismatch,fault_mask,fault_lo_ch,fault_hi_ch` (empty if no 0x713 that test); a detected fault also writes an **events** line. Heartbeat logged to events **on each change of the CAN error state** (TEC/REC/LEC) — healthy bus = one line, faults as they occur. |
| 7" | Build: `pio run` (from **PowerShell** — a pending tool-esptoolpy reinstall fails under Git-Bash/MSys) → clean `.bin`, RAM 19.7% / Flash 52.7%. |

## 2026-08-13 — SD logging: two daily files (CSV data + events) + CAN time sync

Split logging into two per-day files and set the clock from CAN (no NTP since
WiFi was removed).

| LCD | Change |
|---|---|
| 7" | `ba100_can.cpp`: parse the **BMS time-sync frame** (extended `0x18A10001`, `data[0]=0x2B`, raw `sec/min/hour/0/day/month/year+2000`) → `settimeofday()`. No TZ set, so local == the wall time the master sends. `ba100_timestamp()` then returns real dates; until the first sync it's `T+uptime`. |
| 7" | `sd_logger.*`: rewritten for **two files, rolled per day** (tag = `YYYYMMDD`, or `nodate` until time syncs; re-checked each write so it rolls on sync + midnight). New API `sd_logger_event/eventf` (→ `/events_<tag>.log`) and `sd_logger_data` (→ `/data_<tag>.csv`, CSV header auto-written). Dropped `sd_logger_log/logf/log_frame` and the raw CAN-frame dump. Queue line 224 B. |
| 7" | **Data CSV** = one row per scan: `scan_time,resp_time,latency_ms,scanned,received,result,status,c1_mV..c8_mV,t1..t4` — scan/response times to the **millisecond** (`ba100_timestamp_ms`, wall-clock or `T+uptime`) + round-trip latency. A scan opens a **pending record** whose response window stays open until the **next scan** (so all cell/temp frames arrive) or a 30 s idle safety-flush; then the row is written with the master response (PASS/FAIL/**MISMATCH** — echo-vs-scanned compare) or **`NORESP`** if none came. Scans not forwarded (tablet/SD missing) log `NOTSENT` immediately. Barcodes comma-sanitized; the logger no longer prepends a timestamp to data rows. |
| 7" | **Events file** = `device booted`, scanner connected/connecting/disconnected, tablet connected/advertising/disconnected, `CAN link up/down`, `bluetooth restarted`. |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.6% / Flash 52.7%. ⚠ cells/temps in a data row are read at the moment 0x710 completes — if 0x711/0x712 arrive slightly later they may log as 0 (best-effort). |

## 2026-08-13 — Scanner BLE: in-session auto-reconnect + connection-param tuning

Applied the patterns from the canonical NimBLE HID-central project
(esp32beans/BLE_HID_Client) to make the scanner link more robust. On an
UNEXPECTED drop of an established link, we now silently reconnect to the same
device instead of restarting the whole BLE stack + forcing the user to re-Scan.

| LCD | Change |
|---|---|
| 7" | `ble_hid_host.cpp`: on successful connect, remember the device (addr/type/name) and `updateConnParams(24,40,0,100)` → ~30-50 ms interval, 0 latency, **1 s supervision timeout** so a brief RF glitch no longer drops the link. |
| 7" | **Auto-reconnect**: the client `onDisconnect` re-queues a connect to the remembered device; `reconnect_or_give_up()` retries up to 6× with a 1.5 s backoff (5 s connect timeout each), then gives up (user re-Scans). A user **Disconnect / Scan / Restart-Bluetooth** clears `s_want_reconnect` so it does NOT auto-reconnect after an intentional action. First (user) connects don't auto-retry. |
| 7" | `harness_conn.cpp`: removed the old "restart the whole BLE stack on drop" behavior + the `s_expect_down` guard — the DOWN→CONNECTING→CONNECTED cycle is now driven internally and just mirrored to the UI (pill shows "Connecting…" during a reconnect). |
| 7" | Bonds still wiped at boot (fresh pair each power-up) — auto-reconnect is **within a session** only, reusing the live bond, so it never conflicts with that. |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.6% / Flash 52.6%. ⚠ Needs on-hardware validation with the HT20 Pro; the biggest remaining lever is a scanner-side config barcode to force a stable BLE-HID mode (get the HT20 Pro manual). |

## 2026-08-13 — Purge app-WiFi entirely (factory web updater kept)

Removed all app-mode WiFi code (not just disabled it). The factory-mode SoftAP
web updater (`wifi_service.*`, CAN-flash recovery only) is kept, so the WiFi
*library* still compiles — but the app has zero WiFi.

| LCD | Change |
|---|---|
| 7" | **Deleted `app_wifi.cpp/.h`** (STA connect + NTP). Removed `app_wifi_init()` from `main.cpp` and the `[MEM] before/after WiFi` logs; removed `#include "app_wifi.h"` + the `ui_set_link(UI_LINK_WIFI, …)` wiring from `harness_conn.cpp`. |
| 7" | `app_config.h`: removed `CONFIG_APP_WIFI` / `_SSID` / `_PASS` and all `CONFIG_NTP_*` (only app_wifi used them; SD-log timestamps already fall back to T+uptime). |
| 7" | UI: removed the **WiFi top-bar chip** — dropped `UI_LINK_WIFI` from the enum and its `make_status_chip`. Top bar is now CAN / Scanner / Tablet / MQTT / SD. |
| 7" | Scanner BLE connection re-checked — unchanged/known-good (scan → connect → secureConnection → subscribe notify/indicate). ⚠ Note: bonds are wiped every boot (fresh pairing), so a scanner that stubbornly keeps its own bond may drop during re-pair — clear/factory-reset the scanner if that happens. |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.6% / Flash 52.6%. |

## 2026-08-13 — Rebuild: dual-role BLE (WiFi dropped); wipe bonds on boot

Reverted the ZS-040/UART scanner detour and went back to **dual-role BLE**
(scanner = BLE central/HID host, tablet = BLE-HID peripheral). All BLE bonds are
wiped on every boot so both devices pair fresh. **App WiFi is OFF**: WiFi (~42 KB)
+ dual-role NimBLE (~90 KB) don't both fit in internal RAM, and the clean fix
(PSRAM offload via `custom_sdkconfig`) is **not achievable in this toolchain** —
it needs a space-free project path (ours is under `My Drive\…` with spaces) AND
hits an upstream pioarduino package-version mismatch: the libs `ld_flags` apply
`-Wl,--wrap=log_printf` but the core provides `__wrap_esp_log_write`, so the
from-source app link fails with undefined `__wrap_log_printf`. (Also: a
custom_sdkconfig build that overran the 10-min build limit and was killed gutted
the shared `framework-arduinoespressif32-libs` package — recovered by restoring
its `package.json` + reinstalling the `esp32s3` libs.) So: WiFi off, BLE fits.

| LCD | Change |
|---|---|
| 7" | **Deleted `scanner_uart.*`** + the ZS-040 AT/pairing UI. **Re-created `ble_hid_host.*`** (BLE central: scan → connect → subscribe → decode HID/text → barcode ring). It OWNS the radio: `ble_hid_init()` = `BLEDevice::init()` + **`ble_hid_wipe_bonds()`** (NimBLE `ble_store_clear()`) + security + scan. |
| 7" | `ble_hid_device.cpp`: peripheral-only (host owns init); `DEV_RESTART` re-inits radio, wipes bonds, reconfigures the central, rebuilds HID, re-advertises. Keeps the advertising host-sync keep-alive + the <30 KB build_hid guard. |
| 7" | `harness_conn.cpp`: scanner section restored to BLE scan/connect/disconnect/device-list/link-state; forward gate = scanner+tablet(+SD). Nothing remembered (no NVS) — fresh pair each boot. Restart-Bluetooth = full stack restart + bond wipe. |
| 7" | UI (`ui_fallback.cpp`+`ui.h`): restored the BLE scanner panel (device list + Scan/Disconnect + status) and its `ui_bt_*`/`ui_take_*` API; removed the ZS-040 AT panel + API. |
| 7" | `app_config.h`: **`CONFIG_APP_WIFI` → 0** (documented why); removed the ZS-040 `CONFIG_SCANNER_UART_*`. `platformio.ini`: `custom_sdkconfig` left commented with the full explanation (re-enable only from a space-free path once the upstream core/libs mismatch is fixed). |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.6% / Flash 52.6%. |

## 2026-08-13 — Boot loop ROOT CAUSE: BLE OOM (WiFi off) + graceful guard

The boot loop was **out of memory**, not the UART. Backtrace: `build_hid()` →
`BLEDevice::createServer()` → `BLEServer()` → `Semaphore()` → `xQueueGenericSend`
assert, with **free heap 1424 B**. Heap budget from the boot log: 134 KB free →
after WiFi 91 KB (WiFi ~42 KB) → after `BLEDevice::init()` ~1.4 KB (NimBLE ~90 KB).
`createServer()`'s semaphore alloc then fails and the wrapper asserts inside
FreeRTOS → reboot loop. WiFi + NimBLE don't both fit in this Arduino build's
fixed internal-RAM budget (previously flagged as a risk). Tablet HID is now the
essential path, so BLE wins.

| LCD | Change |
|---|---|
| 7" | `app_config.h`: **`CONFIG_APP_WIFI` default → 0** (frees ~42 KB internal RAM so NimBLE fits). Trade-off: SD-log timestamps fall back to T+uptime (no NTP). Documented how/when to re-enable. |
| 7" | `ble_hid_device.cpp` `build_hid()`: **refuses to build if free heap < 30 KB** (logs a clear reason and returns false) instead of letting `createServer()` OOM-assert → the device now stays up (no tablet HID) rather than boot-looping. |
| 7" | Build: `pio run` → clean `.bin`. |

## 2026-08-13 — ZS-040 pairing UI + boot-loop hardening

On-screen pairing/AT config for the soldered ZS-040 (can't hold the AT button
easily), plus a fix for the reported boot loop.

Boot-loop hardening: `scanner_uart_poll()` read loop is now **bounded** (96 bytes
per call) and no longer does a **per-byte `Serial.printf`**. Unbounded reads + a
per-byte log while holding the LVGL lock (e.g. a floating/noisy RX pin or the
module's power-on chatter at boot) could starve the RGB/LVGL loop task → watchdog
reset → boot loop. ⚠ Still want the serial backtrace to confirm; also power the
ZS-040 from **5V with common ground** (a 3.3V brownout at boot loops too).

| LCD | Change |
|---|---|
| 7" | `scanner_uart.*`: added an **AT/pairing layer** on the same UART — `scanner_uart_set_at_mode()` (route lines to a raw-line ring instead of barcodes), `scanner_uart_set_baud()` (switchable AT baud via `updateBaudRate`), `scanner_uart_send()` (writes `cmd`+CRLF), `scanner_uart_take_line()`. Read loop bounded; per-byte trace removed. |
| 7" | UI (`ui_fallback.cpp`+`ui.h`): scanner panel is now a **ZS-040 setup card** — a baud toggle (AT @ 38400⇄9600), AT buttons `[AT] [ROLE=1] [RESET]` / `[CMODE=0] [CMODE=1] [INQ]`, a `[Pair+Auto]` sequence (AT→ROLE=1→CMODE=1→RESET) + `[Scan mode]`, and a scrolling log of module replies. New API `ui_scanner_log`/`ui_take_scanner_cmd`/`ui_take_scanner_baud`. |
| 7" | `harness_conn.cpp`: drives the AT buttons → UART (one sequence command per ~500 ms), echoes `>>`/`<<` to the log, applies baud changes; while in AT mode it does NOT treat replies as barcodes. |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.6% / Flash 51.8%. |

## 2026-08-13 — Scanner moved to UART (ZS-040); BLE is now tablet-only

Pivoted the barcode scanner off the ESP32's BLE radio entirely. The scanner is a
Bluetooth-classic (SPP) unit paired to a **ZS-040 (HC-05/06)** module that bridges
to the ESP32 over a dedicated UART; the ESP32 just reads ASCII barcode text. The
BLE radio is now used ONLY as the HID keyboard peripheral to the tablet (no more
dual-role central+peripheral, and no more of the NimBLE-central pairing pain).

| LCD | Change |
|---|---|
| 7" | **New `scanner_uart.cpp/.h`**: opens `Serial2` on the board's spare UART header (GPIO15=RX / GPIO16=TX, configurable) at `CONFIG_SCANNER_UART_BAUD` (default 9600), assembles CR/LF- (or idle-gap-) terminated codes into a ring, exposes `scanner_uart_poll/take_barcode/link_up`. Per-byte RX trace to UART0 for bring-up. |
| 7" | **Deleted `ble_hid_host.cpp/.h`** (BLE central + HID-over-GATT scanner) — removed all scan/connect/bond/decode code. |
| 7" | `ble_hid_device.cpp` now **owns the radio**: `ble_radio_setup()` does `BLEDevice::init()` + just-works pairing (was previously done by the host); dev-task calls it before `build_hid()`; `DEV_RESTART` no longer touches the (gone) host. |
| 7" | `harness_conn.cpp`: scanner section rewritten to pump `scanner_uart_poll()`, reflect UART link liveness, and drain codes. Forward gate is now **tablet (+SD if required)** only — a received code already proves the scanner works. Restart-Bluetooth restarts only the tablet BLE. |
| 7" | UI (`ui_fallback.cpp` + `ui.h`): scanner panel **simplified to a link-status** card (pill + hint/last-code) — removed the BLE scan button, device list, connect/disconnect, and their `ui_bt_*`/`ui_take_*` API + callbacks. Subtitle now "Bluetooth scanner via ZS-040 (UART)". |
| 7" | `app_config.h`: added `CONFIG_SCANNER_UART_{NUM,RX_PIN,TX_PIN,BAUD}`; refreshed the header comment (and cleaned stray dictation text). |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.2% / Flash 51.7% (down from 19.6% / 52.7%). ⚠ Confirm the ZS-040 link baud on hardware — watch `[SCAN] rx ..` on UART0; if bytes are garbage, change `CONFIG_SCANNER_UART_BAUD` (try 38400/115200). |

## 2026-08-13 — Tablet advertising: fix NimBLE host-sync race (retry keep-alive)

**Root cause of "Advertise does nothing" found** in the bundled BLE library
(NimBLE backend, `BLEAdvertising.cpp` NimBLE `start()`): it returns `false`
("Host reset, wait for sync") until the NimBLE host has finished its *async*
sync, and can also transiently fail with `EBUSY`/`EPREEMPTED`. Our advertise was
a **one-shot** — at boot it fired before the host synced, got `false`, and never
retried. The scanner (central) worked because it's connected manually long after
sync. Confirmed the build is NimBLE-only (`CONFIG_BT_NIMBLE_ENABLED`, all four
roles, `MAX_CONNECTIONS=3`) — the stack supports dual-role; the failure was
purely the un-retried sync race.

| LCD | Change |
|---|---|
| 7" | `ble_hid_device.cpp`: `start_advertising()` now verifies success with `s_adv->isAdvertising()` and logs `start=/active=`; returns whether the radio is actually advertising. `dev_task` latches `s_want_adv` and turns the queue wait into a **1 s keep-alive**: if we want to advertise, aren't connected, and the radio isn't advertising, it re-attempts every second — so it rides out the host-sync race at boot and any transient failure, and re-advertises after a disconnect. `DEV_ADV_ON` and the post-stack-restart rebuild reuse the same helper. Advertise is on from boot until a tablet connects. |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.6% / Flash 52.7%. ⚠ On hardware: UART should show `[BLE-HID] advertising start=… active=1` within ~1 s of boot, and "Coulomb Harness" should appear in the tablet's Bluetooth list with no tap. Repeated `active=0` would mean a deeper stack/config issue. |

## 2026-08-13 — No scanner memory (pair fresh every boot); fixes Advertise

Removed the remembered-scanner auto-connect entirely. Root cause of Advertise
"doing nothing": at boot `harness_conn_init()` auto-connected the NVS-remembered
scanner via `do_connect()`, which **blocks** in `s_client->connect()` +
`secureConnection()`. When that scanner is absent, the single-radio NimBLE
controller stays in the **initiating** state, and `BLEAdvertising::start()` fails
while a central connect is in flight → the tablet Advertise command was accepted
but the radio never advertised. Verified NimBLE has all four roles +
`MAX_CONNECTIONS=3`, so the stack itself supports dual-role; the block was the
stuck initiate. Leaving the radio idle at boot lets Advertise start cleanly.

| LCD | Change |
|---|---|
| 7" | `harness_conn.cpp`: dropped `Preferences`/NVS (`btname`/`btaddr`/`bttype`) and the `s_saved_*` state; `harness_conn_init()` no longer auto-connects a remembered scanner, and the Scan→Connect path no longer saves one. Every boot starts idle → user pairs a fresh scanner via Scan. Established-link drop still restarts the whole BLE stack (unchanged). |
| 7" | `main.cpp`: updated the `harness_conn_init()` comment (no more remembered-scanner auto-connect). |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.6% / Flash 52.7%. ⚠ Advertise fix needs on-hardware confirmation (boot, open Connections, press Advertise → pill goes Busy + "Coulomb Harness" appears in the tablet's Bluetooth list). |

## 2026-08-11 — Tablet HID diagnostics (advertise "does nothing")

Advertise did nothing on-screen and no HID device appeared on the tablet — the
symptom of `build_hid()` failing/hanging on the dev task (then the Advertise
command is never processed and advertising never starts). Suspected cause: RAM
pressure from WiFi init (added just before BLE).

| LCD | Change |
|---|---|
| 7" | `ble_hid_device.cpp`: `build_hid()` now returns bool and **guards every allocation** (createServer / BLEHIDDevice / inputReport / getAdvertising) with `std::nothrow` + failure logs instead of crashing/hanging, and prints free-heap/largest-block at build. `dev_task` tracks `s_hid_built`; DEV_ADV_ON logs and refuses if the HID wasn't built; `s_adv->start()` result is logged. `ble_hid_dev_advertise()` logs the request + queue handle. Diagnostics to pinpoint whether it's memory or a build failure. |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.6% / Flash 52.7%. |

## 2026-08-11 — WiFi/BLE RAM: init WiFi before BLE (esp_wifi_init NO_MEM)

On-hardware: `esp_wifi_init 0x101 ESP_ERR_NO_MEM` — WiFi couldn't init because BLE
(already up) had taken the internal RAM. WiFi failed gracefully (chip red, device
otherwise fine), but no network time.

| LCD | Change |
|---|---|
| 7" | `main.cpp`: **bring WiFi up BEFORE BLE** — `esp_wifi_init()` needs a large contiguous internal block; doing BLE first fragmented the heap. Also log free heap + largest block `[MEM]` before/after WiFi and after BLE to quantify the budget. |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.6% / Flash 52.7%. ⚠ If WiFi now inits but BLE fails, both stacks don't fit in this Arduino build's fixed buffer sizes — then it's WiFi **or** BLE, or an alternate time source. Watch the `[MEM]` numbers + whether the scanner still connects. |

## 2026-08-11 — App-mode WiFi + NTP time; SD optional + timestamped scan log

| LCD | Change |
|---|---|
| 7" | **App-mode WiFi station** (`app_wifi.cpp/.h`): connects to `CONFIG_APP_WIFI_SSID`/`_PASS` (new in app_config.h) and arms NTP (`CONFIG_NTP_SERVER` + IST offset). Non-blocking; coexists with BLE (`WiFi.setSleep(true)`). `app_wifi_init()` called from `main.cpp` app path; the WiFi top-bar chip now reflects the live station link. Separate from the factory SoftAP updater. |
| 7" | **Real timestamps**: `ba100_get_datetime`/`ba100_timestamp` now read the ESP system clock (set by NTP) → SD logs use real wall time (`YYYY-MM-DD HH:MM:SS`), falling back to `T+uptime` until synced. |
| 7" | **SD optional**: `CONFIG_SD_REQUIRED` (app_config.h). 1 = SD is part of the forward gate (current behaviour); 0 = SD never blocks forwarding — ignored when absent, still used when present. |
| 7" | **Timestamped scan logging**: every scanned barcode is logged as `SCAN <code>` via `sd_logger_logf` (dropped silently if no card) — so when an SD is present you get which barcode was scanned at what (network) time. |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.6% / Flash 52.7%. ⚠ WiFi/BLE coexistence + NTP need on-hardware validation; set real WiFi creds in app_config.h first. |

## 2026-08-11 — No scanner reconnect: restart BLE stack on drop; SD wake-up fix

| LCD | Change |
|---|---|
| 7" | **Removed all scanner reconnection logic** (auto-retry loop gone). A real disconnect of an established scanner link (CONNECTED→DOWN, not user-initiated) now **restarts the whole BLE stack**: `ble_hid_restart_stack()` (on the 16 KB HID task) tears down both roles, `BLEDevice::deinit(true)`, re-inits, re-runs `ble_hid_host_setup()`, and rebuilds the HID server. After restart the scanner is NOT auto-reconnected — user presses Scan. Boot still does a one-shot auto-connect to the remembered device. Manual Disconnect / Restart-Bluetooth set `s_expect_down` so they don't double-trigger a restart (Restart-Bluetooth now IS a full stack restart). Split `ble_hid_host` into `ble_hid_host_setup()` + `ble_hid_host_teardown()`. |
| 7" | **SD card fixes** (cross-checked against the manufacturer's `03_SD_Test` demo — our pins/approach match it: SPI 11/12/13, CS = CH422G EXIO4, `SD.begin(-1)`). Key difference found: the demo uses `SD.begin(-1)` at the **default 4 MHz**; we were forcing **20 MHz**, which fails to init on this board — lowered `SD_SPI_HZ` to 4 MHz. Also added the SD SPI power-up handshake (CS high + ~80 clocks before `SD.begin()`, since CS is held low via the expander) and UART diagnostics (`SD.begin` fail / cardType NONE / card OK type+size). Direction is fine — the CH422G IO-output-enable is global, so our `pinMode(EXIO4, OUTPUT)` already does what the demo's `enableAllIO_Output()` does. |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.6% / Flash 52.6%. |

## 2026-08-11 — Gate barcode forwarding on SD+scanner+tablet; connect popup

| LCD | Change |
|---|---|
| 7" | A scanned barcode is **only forwarded** (to the tablet via HID and to the master via CAN/`start_check`) when **SD card + scanner + tablet are all connected**. Otherwise the code is still read and shown, but a **popup on the main page** lists the missing device(s) — "Please connect SD card, Tablet, then scan again." — and auto-hides after 5 s (`ui_show_connect_popup`, `lv_timer`). Gating uses `sd_logger_present()`, `ble_hid_get_state()`, `ble_hid_dev_state()`. |
| 7" | **GATT client restarted on every (re)connect attempt** (`ble_hid_host` do_connect): the old `BLEClient` is deleted and a fresh one created, so a stale client from a dropped link can't block reconnection. Safe under `CONFIG_BT_NIMBLE_MAX_CONNECTIONS` (disconnected clients are already out of the map). |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.6% / Flash 52.5%. |

## 2026-08-11 — Auto-check on scan + barcode-echo verify + master barcode on page 3

| LCD | Change |
|---|---|
| 7" | **Check is fully automatic** — scanning a barcode now runs the whole check (clears result + cell/temp + master echo, TXes the barcode, waits for the DAS reply). The **Check button is removed**; its slot on the left column is now a status strip ("Scan a barcode to test" / "Checking…"). `ui_take_check_request` / `cb_btn_check` deleted. |
| 7" | **Barcode-echo verification**: `ba100_can` reassembles the barcode the master echoes in 0x710 and exposes it via `ba100_take_das_result(status, echo, len)` (replaces `ba100_take_das_status`). Echo layout is **length-driven**: seq0 = Status(byte1) + barcode 0-5; every later seq N carries 7 bytes (bytes 1-7) at offset 6+(N-1)*7 — so the echo runs past 16 bytes (up to the 24-byte capacity). Reply is complete once the zero-padding terminator appears in the assembled prefix (or 24 B reached), latched once per reply. If echo == scanned → PASS/FAIL from Status; if echo != scanned → new **`UI_RESULT_MISMATCH`** ("MISMATCH", amber). |
| 7" | **Master barcode shown on the Cell Data (page-3) title bar** (`ui_set_rx_barcode` → "Master: …"), cleared at the start of each check. |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.6% / Flash 52.5%. |

## 2026-08-11 — Scanner auto-reconnect (persistent)

| LCD | Change |
|---|---|
| 7" | `harness_conn` keeps retrying the remembered scanner every 4 s whenever the link is down (`s_reconnect_enabled` + `RECONNECT_INTERVAL_MS`). Armed on any successful connect and on boot (power-up reconnects to the last device, already via NVS). Stopped by **Scan**, **Disconnect**, or **Restart Bluetooth** (Restart no longer re-arms — it's now a clean slate; reconnect via Scan). Retry only fires while `state==DOWN`, so it never stacks connect attempts. |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.5% / Flash 52.5%. |

## 2026-08-11 — Barcode TX IDs → 0x600/0x601/0x602; clear cell/temp on Check

| LCD | Change |
|---|---|
| 7" | Barcode TX IDs changed `0x800/0x801/0x802` → **`0x600/0x601/0x602`** (still 24 B, zero-padded). |
| 7" | On each Check, `ba100_clear_das_data()` zeroes the cached cells/temps so the Cell Data page shows "-" until fresh 0x711/0x712 arrive. |

## 2026-08-11 — Barcode TX 0x800/0x801/0x802 (24 B) + no result timeout

| LCD | Change |
|---|---|
| 7" | **Barcode TX moved to 0x800/0x801/0x802, 24 bytes** (0-7 / 8-15 / 16-23), zero-padded when the code is shorter than 24. `ba100_send_barcode()` now sends three frames; `DAS_BARCODE_MAXLEN` 16→24. (RX IDs 0x710/0x711/0x712/0x720 unchanged.) |
| 7" | **No PASS/FAIL timeout**: removed the 3 s "no reply → FAIL". After Check the result shows **"-"** and stays there until the DAS actually sends a Status — a silent master is no longer shown as FAIL. |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.5% / Flash 52.5%. |

## 2026-08-11 — Integrate DAS protocol (das_protocol.dbc) + cell/temp page

Wired the harness CAN protocol. We are the **ECU**: TX the scanned barcode, RX
status + cell/temperature data from the **DAS** (device-under-test).

| LCD | Change |
|---|---|
| 7" | **CAN layer (`ba100_can.*`)**: TX `ba100_send_barcode()` → 0x700/0x701 (16 ASCII bytes). RX parse: 0x710 Status (seq0 byte1 → PASS/FAIL via `ba100_take_das_status()`), 0x711 cells (8× LE16 mV by sequence), 0x712 temps (4× LE16 by sequence), 0x720 heartbeat (liveness). `BA100_State_t` gained `cell_mv[8]` / `temp_raw[4]` / `das_status`. Status→result mapping is PROVISIONAL (`DAS_STATUS_PASS 0x01` — capture only showed 0x01). |
| 7" | **Check now sends the barcode** and requires one: the button reads "Scan barcode first" and is disabled until a code is scanned; on Check it TXes 0x700/0x701 and the DAS 0x710 Status decides PASS/FAIL (3 s timeout → FAIL). The barcode is no longer blanked on Check (only the result is). `harness_conn` keeps the last scanned code and drives the real round-trip (simulated PASS/FAIL removed). |
| 7" | **New page 3 (Cell Data)**: swipe LEFT from main (swipe RIGHT back). 8 cell-voltage tiles (mV→V) + 4 temperature tiles (raw→°C), live from `BA100_State_t`. Navigation is now 3 pages: `[DATA] · MAIN · [CONN]`. Scaling is provisional (DBC values are "raw"). |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.5% / Flash 52.5%. |

## 2026-08-11 — Fix top-bar chips clipped (only 2 of 6 visible)

Only the two rightmost chips (MQTT, SD) showed on hardware. NOT a font issue —
all six glyphs (WiFi 0xF1EB, Bluetooth 0xF293, Keyboard 0xF11C, Shuffle 0xF074,
Envelope, SD_CARD) are confirmed compiled into `lv_font_montserrat_16`. Cause:
the chip row used a `LV_SIZE_CONTENT` flex container that ended up too narrow, so
with `FLEX_ALIGN_END` the four left chips overflowed to the left and were clipped.

| LCD | Change |
|---|---|
| 7" | Chip row now uses an **explicit width** (`SCR_W-240`, x≈240..794, right-aligned, clears the title) instead of `SIZE_CONTENT`; gap trimmed to 10 px. All six chips (WiFi, CAN, Scanner, Tablet, MQTT, SD) now fit on the 800 px bar. |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.5% / Flash 52.4%. |

## 2026-08-11 — Top bar: 6 links (add Tablet + SD), blink red when disconnected

| LCD | Change |
|---|---|
| 7" | Main top bar now shows **6** status chips in order: WiFi, CAN, Scanner, Tablet, MQTT, SD (added Tablet + SD via new `UI_LINK_SD`; icons `LV_SYMBOL_KEYBOARD` / `LV_SYMBOL_SD_CARD`). |
| 7" | **Status style is now green-solid / red-blink**: connected = solid green; not-connected = blinking red (~2.2 Hz via a 450 ms `lv_timer` toggling between `C_FAIL` and a dim red). `ui_set_link` records per-link status; the timer re-applies colour to every non-connected chip. (Busy states also blink red — only "connected" is green, per request.) |
| 7" | Wired real state: **SD** chip from `sd_logger_present()`, **CAN** from `can_alive` (both in `ui_apply_state`); Scanner/Tablet from their BLE links. WiFi/MQTT remain placeholders (blink red until wired). |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.5% / Flash 52.4%. |

## 2026-08-11 — UI polish: adaptive barcode font, bright PASS/FAIL, top-bar icons, version moved

| LCD | Change |
|---|---|
| 7" | **Barcode font auto-sizes to length** (`ui_set_barcode`): ≤10 chars → 32 px, ≤15 → 24 px, ≤24 → 16 px, else 12 px — long codes stay on one line. |
| 7" | **PASS/FAIL brighter**: new vivid colours `C_PASS 0x39FF7A` / `C_FAIL 0xFF3B3B`, brighter card tints, 3 px border. |
| 7" | **Top-bar status = icon + label chips** for WiFi, CAN, Scanner, MQTT (16 px, green up / amber busy / grey down). Icons: WiFi=`LV_SYMBOL_WIFI`, Scanner=`LV_SYMBOL_BLUETOOTH`; CAN=`LV_SYMBOL_SHUFFLE` and MQTT=`LV_SYMBOL_ENVELOPE` are placeholders (no native glyph). Only the scanner is a real link today; CAN is live from `can_alive`; **WiFi/MQTT stay grey** (their fake "coming up" simulation was removed — to be wired later). Dropped the QR/Tablet text chips from the top bar. |
| 7" | **Version/partition/baud moved to page 2 (Connections) title bar**, right-aligned, as regular full-opacity 16 px text (was a faded 12 px watermark bottom-right of page 1). |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.5% / Flash 52.4%. |

## 2026-08-11 — Fix HID-build crash (manufacturer) + forward scan → tablet

Boot crash in `build_hid` → `BLEHIDDevice::manufacturer(String)` → `setValue` →
`Semaphore::take` on garbage. Cause: in this library `manufacturer(String)` is a
SETTER that assumes the characteristic exists; the no-arg `manufacturer()` is what
CREATES it (the ctor makes pnp/hidInfo/reportMap but NOT manufacturer). Calling
the setter alone dereferenced an uninitialized pointer. (The earlier 8 KB stack
overflow had masked this; with the 16 KB task it reached the real bug.)

| LCD | Change |
|---|---|
| 7" | `ble_hid_device.cpp`: build the manufacturer characteristic before setting it — `manufacturer()->setValue("Coulomb")`. This was the reboot loop. |
| 7" | **Forward scanned barcode → tablet over BLE-HID**: `harness_conn` now types each decoded barcode (from the scanner) + Enter to the tablet via `ble_hid_dev_type()` (no-op when the tablet isn't connected), in addition to showing it on-screen. `DevMsg` buffer grown to 130 B for a full barcode + terminator. |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.5% / Flash 52.4%. |

## 2026-08-11 — Fix Advertise crash: build HID server before any connection

Scanner now connects fine (generic GATT client works). But tapping Advertise
crashed: `assert failed: ble_svc_gap_init (rc == 0)` in `BLEDevice::createServer()`.
Cause: the wrapper's `createServer()` calls `ble_gatts_reset()` + `ble_svc_gap_init()`,
and **`ble_gatts_reset()` fails while a BLE link is active** — and by the time the
user taps Advertise, the scanner is already connected. NimBLE requires the GATT
server to be registered before any connection exists.

| LCD | Change |
|---|---|
| 7" | `ble_hid_device.cpp`: the HID GATT server is now built **once at init, before the scanner auto-connects** (no active link → `ble_gatts_reset()` succeeds), still on the 16 KB `hid_dev` task (avoids the loop-stack overflow). `ble_hid_dev_init()` blocks on a semaphore until the build completes; `harness_conn_init()` calls it before `ble_hid_connect()`. Advertise now only calls `startAdvertising()` (safe while the scanner is connected — concurrent central + peripheral). |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.5% / Flash 52.4%. |

## 2026-08-11 — Fix boot crash: build tablet HID peripheral lazily off-thread

On-hardware boot crash (LoadProhibited) in `harness_conn_init → ble_hid_dev_init →
BLEHIDDevice::inputReport → BLECharacteristic::setValue` (bad semaphore; the
`esp_libc_init`/`pthread_mutex_init_if_static` frames = corrupted static-init
guards = stack overflow). `BLEHIDDevice`'s constructor allocates ~7 characteristics
+ descriptors (each with String + semaphore members); doing that in `setup()` on
the 8 KB loop task, right after the BLE **central** init, overflowed the stack.

| LCD | Change |
|---|---|
| 7" | `ble_hid_device.cpp`: **the HID peripheral is no longer built in `setup()`**. `ble_hid_dev_init()` now only creates a small command queue + a dedicated `hid_dev` task (8 KB stack). The server/`BLEHIDDevice`/characteristics/advertising are built LAZILY on that task the first time Advertise is pressed (`DEV_ADV_ON`), so boot — and the scanner — are unaffected even if HID setup misbehaves. Advertise/type/stop are queued to the task. |
| 7" | `platformio.ini`: bumped `CONFIG_ARDUINO_LOOP_STACK_SIZE` 8192 → 16384 (the BLE **central** init still runs in `setup()` on the loop task). |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.5% / Flash 52.4%. Boot should no longer crash; the tablet HID service is created only when you tap Advertise (on its own task). |

## 2026-08-11 — Scanner: generic GATT client (fixes 2 s disconnect)

The scanner connected then dropped in ~2 s on **every** host (Android, Windows,
ESP), which means it is not a plain HID keyboard in its current mode — it is in
"BLE data mode": it advertises a GATT service and drops the link unless a client
connects and **enables notifications** on its data characteristic. (Our old code
also bailed the instant it didn't find the HID service 0x1812.)

| LCD | Change |
|---|---|
| 7" | **`ble_hid_host.cpp` connect path is now a generic GATT client**: after connect (+ best-effort `secureConnection()`), it enumerates ALL services/characteristics and subscribes to EVERY notify/indicate characteristic — no longer requires the HID service. Enabling notifications is what keeps the scanner from disconnecting. |
| 7" | **Barcode decode routes by characteristic UUID**: HID report chars (0x2A4D/0x2A22) → keyboard-report decode; any other notify char → ASCII-text decode (emit on CR/LF or after a >300 ms gap, for scanners that send no terminator). Covers HID-mode and data-mode (e.g. Nordic UART) scanners with one path. |
| 7" | **UART debug**: logs each discovered `service`/`char` (UUID + notify/indicate/write flags) and a hex dump of every notification — so an unknown scanner's data format is visible on the wire. |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.5% / Flash 52.4%. Needs on-hardware retest: the scanner should now STAY connected; watch UART for the service/char dump + notification bytes. |

## 2026-08-11 — Scanner connect fix + tablet BLE-HID output (Test HID)

| LCD | Change |
|---|---|
| 7" | **Scanner connect fix**: `do_connect()` now calls `BLEClient::secureConnection()` right after connecting (before touching HID characteristics). BLE-HID scanners require an encrypted/bonded link and drop the connection within ~1-2 s otherwise — this is the "one beep then two beeps / never stays connected" symptom. Added serial logs (UART0) for each step: connecting / pairing / connected+subscribed / disconnected / no-HID-service. |
| 7" | **Tablet is now a REAL BLE-HID keyboard peripheral** (`ble_hid_device.cpp/.h`): advertises as "Coulomb Harness" (HID keyboard), and when the tablet pairs, `ble_hid_dev_type()` sends keystrokes on a typing task. Runs concurrently with the BLE central (scanner). |
| 7" | **Tablet panel redesigned** for the peripheral role (it no longer scans/lists — the tablet discovers *us*): an **Advertise** button (doubles as Stop/Disconnect), a status pill, instructions, and a **Test HID** button that types `1234`+Enter (enabled only while a tablet is connected). New UI API `ui_tablet_set_status/ui_take_tablet_advertise/ui_take_tablet_test`; the tablet no longer uses the shared discovery panel. |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.5% / Flash 52.4%. ⚠ Still on-hardware territory: if the scanner was previously bonded to a phone/PC it may need unpairing/factory-reset before it will bond to the ESP; concurrent central+peripheral and tablet pairing need bench validation. Serial logs are on UART0 (USB-CDC is off on this board). |

## 2026-08-11 — Real BLE-HID host for the QR scanner (scan + connect + barcode-in)

| LCD | Change |
|---|---|
| 7" | **New `ble_hid_host.cpp/.h`** — BLE central + HID-over-GATT host using the core's bundled `BLE` library. Active scan → thread-safe device snapshot; connect on a worker task; Just-Works bonding; subscribe to HID input reports (0x2A22 boot-kbd + 0x2A4D report); decode keyboard HID usage codes → ASCII → a completed barcode (Enter-terminated) pushed to a ring the LVGL thread drains. |
| 7" | **Scanner panel is now REAL** (`harness_conn.cpp`): Scan lists actually-discovered BLE devices, selecting one connects, Disconnect drops it, the device (name+addr+type) is remembered in NVS and auto-connects on boot, and decoded barcodes flow into the main-page Barcode field. Tablet panel + WiFi/MQTT + Check result remain simulated. |
| 7" | Stack note: this core's `BLE` library is a **NimBLE** wrapper — used the portable security API (`setAuthenticationMode(bonding,mitm,sc)` + `setCapability(3)`), dropped Bluedroid-only calls (`setEncryptionLevel`, `esp_ble_auth_cmpl_t`). `main.cpp` calls `harness_conn_init()` (which does `ble_hid_init()`) OUTSIDE the LVGL lock. |
| 7" | Build: `pio run` → clean `.bin`, RAM 19.4% / Flash 52.2% (BLE stack ~+270 KB). ⚠ COMPILE-VERIFIED ONLY — BLE pairing/bonding and the exact HID report layout vary by scanner, so this needs on-hardware bring-up. The HT20 Pro must be in **BLE / Bluetooth-HID mode** (not classic BT) to be discoverable. Tuning points if barcodes don't arrive: protocol-mode write, which report characteristic, and the report-ID offset heuristic in `notifyCb()`. |

## 2026-08-11 — Symmetric BT panels + Disconnect (scanner & tablet)

| LCD | Change |
|---|---|
| 7" | The tablet half of page 2 now uses the **same discovery menu as the scanner** (device list + Scan + select-to-connect + status pill) instead of the advertise switch. Both panels are built by one `build_bt_panel()` helper. |
| 7" | Added a **Disconnect** button to each panel (shown only while connected). |
| 7" | UI API is now **per-target** (`ui_bt_target_t` = SCANNER/TABLET): `ui_bt_set_scanning/clear_devices/add_device/add_note/set_status` and pollers `ui_take_scan_request/connect_request/disconnect_request(t)`. Replaced the old scanner/tablet-specific setters and the tablet toggle. `ui_bt_add_note()` renders a non-clickable list caption. |
| 7" | `harness_conn` stub now drives both panels + disconnect, and each Scan lists example devices prefixed with an on-screen "simulated — real BLE scan not enabled yet" note (so it's clear no real radio is running). |
| 7" | Build: `pio run` → clean `.bin`, RAM 18.3% / Flash 45.8%. NOTE: the device list is still SIMULATED — no real BLE scan yet, which is why a physical scanner (e.g. NETUM/HT20 Pro) does not appear. Real BLE is the next step (and the HT20 Pro must be in BLE-HID mode, not classic BT — ESP32-S3 is BLE-only). |

## 2026-08-11 — Connections page + top-bar status + link manager

| LCD | Change |
|---|---|
| 7" | **New page 2 (Connections)**, reached by swiping RIGHT from the main page (swipe LEFT to return). LEFT half = QR scanner over Bluetooth (scrollable device list, "Scan for devices" button, status pill, remembered/connected device line). RIGHT half = Android tablet BLE-HID (status pill, description, "Advertise to tablet" switch). BOTTOM = one "Restart Bluetooth" button. |
| 7" | **Main top bar** gained 5 status chips — CAN / WiFi / MQTT / QR / Tablet — green (up) / amber (busy) / grey (down). The lone CAN dot was folded into this row. |
| 7" | **Check button now refreshes**: on each tap the barcode and PASS/FAIL both reset to `-` before the (CAN) result comes back. Idle placeholders are `-`. |
| 7" | **UI API expanded** (`ui.h`): `ui_set_link`, `ui_bt_set_scanning/clear_devices/add_device/set_scanner_status`, `ui_tablet_set_status`, and pollers `ui_take_scan_request/connect_request/restart_bt_request/tablet_toggle`. `LV_USE_LIST` + `LV_USE_SWITCH` enabled in `lv_conf.h`. |
| 7" | **New `harness_conn.*` link manager** (polled from `loop()` under the LVGL lock; `init` after `ui_init`). ⚠ Currently a SIMULATION: fakes WiFi/MQTT coming up, BLE device discovery, connect + NVS-remembered auto-connect on boot, restart, tablet advertise, and a simulated Check PASS/FAIL. Marked with `>>> REAL BACKEND HERE <<<` for the eventual NimBLE dual-role / WiFi / MQTT / CAN-check code. |
| 7" | Build note: `pio run` → clean `.bin`, RAM 18.3% / Flash 45.8%. Compile-verified only; not bench-tested. Gotcha fixed: `*` + `/` inside doc comments (`ui_set_*/ui_bt_*`) prematurely closed the block comments. |

## 2026-08-11 — Charger + 4.3" removal (lean harness codebase)

Removed the charger/BMS application and the 4.3" board option so `app_config.h`
carries only what the harness tester uses. CAN-flash bootloader + partition
table are unchanged (the `0x0D`/`0x0B` protocol and factory/app split are still
in lock-step with the IOT code — no divergence).

| LCD | Change |
|---|---|
| 7" | **`ba100_can.*` gutted to a lean TWAI wrapper**: TWAI bring-up at NVS baud, RX poll with link-liveness (green dot), `0x2CC` baud change + ACK, `0x0D` enter-bootloader hand-off, uptime timestamp for the SD logger, btl-active gating, no-op `ba100_charger_off()`. Dropped the entire charger/BMS protocol (charging FSM, battery detection, faults, current sharing, KION, terminal check). `BA100_State_t` slimmed to a link-alive snapshot (`chg_state`/`sc_relay`/`current_A` kept only so the bootloader's reboot-safety gate compiles). |
| 7" | **`app_config.h` slimmed** to CAN pins (GPIO20/19), CAN baud, FW version, factory-WiFi config. Removed `CONFIG_LCD_TYPE`, `CONFIG_NUM_CHARGERS`, `CONFIG_CHARGE_CONTROL`, `CONFIG_KION_MODE`, current-share, and all charge ramp/foldback/terminal tunables. |
| 7" | **4.3" board removed.** `main.cpp`: dropped `CONFIG_LCD_TYPE` guards, USB_SEL CAN mux is now unconditional. `esp_panel_board_custom_conf.h`: board name fixed to the 7". `platformio.ini`: `-DARDUINO_USB_CDC_ON_BOOT=0` pinned directly; dropped `extra_scripts` and deleted `scripts/config_from_header.py`. |
| 7" | **`can_bootloader.cpp`**: `charger_safe_for_btl()` no longer reads BMS op-state (always safe — no charger). Comment de-chargered. |
| 7" | **`wifi_service.cpp`**: factory web UI trimmed to CAN-baud + firmware upload + backup status; removed the KION and num-chargers settings and the 4.3"/7" line. |
| 7" | Build note: `pio run -t clean` + `pio run` (PowerShell) → clean `.bin`, RAM 18.2% / Flash 45.6%. Compile-verified only; not bench-tested. |

## 2026-08-11 — Fork: Harness Tester UI (7")

Forked from ChargerLCD_combinedV2.1.0 into a new project for a barcode/harness
test station. The reused infrastructure is unchanged: partition scheme
(`partitions.csv`), CAN-flash bootloader (`can_bootloader.*`), version/partition
status watermark, and the top-right CAN-connection status dot.

| LCD | Change |
|---|---|
| 7" | Set `CONFIG_LCD_TYPE=1` (7" board, USB-CDC off) and `CONFIG_FW_VERSION="HT v0.1.0"`. |
| 7" | **New single-screen UI** (`src/ui/ui_fallback.cpp` + `ui.h`), rewritten from the charger's two-page layout. LEFT: Coulomb logo (same slot) + a full-width **Check** button below it that greys to "Checking…" after a tap until a CAN reply arrives. RIGHT: scanned **barcode** value on top + a big **PASS/FAIL** indicator below. Kept: CAN status dot (top-right) and `version \| partition \| baud` watermark (bottom-right). Dropped: SoC/voltage/current cards, charge control, and the fault page. |
| 7" | New UI API for the (future) barcode/BLE/CAN layers: `ui_set_barcode()`, `ui_set_result()`, `ui_set_checking()`, `ui_take_check_request()`. The CAN dot is currently driven by CAN telemetry freshness (placeholder link-liveness) — to be swapped for a harness heartbeat flag. |
| 7" | Build note: verified via `pio run -t clean` + `pio run` (PowerShell) — clean `.bin`, RAM 18.7% / Flash 46.0%. Compile-verified only; not yet bench-tested on hardware. TODO (later): BLE barcode-scanner ingest → BLE HID out to the Android tab; harness CAN read + Check confirmation. |

## 2026-08-06 — Factory WiFi Stage 2: runtime num_chargers + 5-min login timeout

| LCD | Change |
|---|---|
| both | **`num_chargers` is now runtime/NVS** (editable from the web page). `ba100_can.cpp` arrays sized to `MAX_CHARGERS` (60); a runtime `s_num_chargers` (NVS key `"nchg"`, default `CONFIG_NUM_CHARGERS`, clamp 1–60) replaces the compile-time `NUM_CHARGERS` in every loop / split / ID-range / current-share gate. Applies on next boot. New `ba100_get_num_chargers()`. Automatic current-sharing now gates at runtime on `s_num_chargers > 1` (the `#if CONFIG_ENABLE_CURRENT_SHARE` guards were removed; `sol_set_current_share()` is always compiled). Web config form gained a 1–60 field with a warning that it must equal the modules physically present (else the all-online start gate blocks charging). |
| both | **5-minute web-inactivity reboot timeout.** Once a technician logs in over WiFi, `btl_task` suppresses the 15 s CAN-idle factory recover and instead reboots to the app after `WEB_SESSION_TIMEOUT_MS` (5 min) of no HTTP activity — each web request resets it (`wifi_service_logged_in()` / `wifi_service_ms_since_activity()`, cross-core via volatile flags). Pre-login, web activity (e.g. loading the login page) also holds off the 15 s recover so we don't reboot mid-login. |
| both | Build note: verified via `pio run` (PowerShell) — clean `.bin`, RAM 18.8% / Flash 45.9%. Compile-verified only; WiFi/web/upload/timeout still need on-hardware bench testing. ⚠ Pre-login connect grace is still 15 s from the last web/CAN activity — see notes if that's too tight in the field. |

## 2026-08-06 — Factory-mode WiFi service UI + web updater (Stage 1)

| LCD | Change |
|---|---|
| both | New factory-mode **WiFi service UI + updater**. When the unit boots into the `factory` partition, `can_bootloader_run_if_factory()` now also raises a **hidden SoftAP** (SSID `ESP32LCD`, WPA2 `12345678`, `CONFIG_FACTORY_WIFI_*` in `app_config.h`) and a built-in `WebServer` on port 80 (new `src/wifi_service.cpp` / `include/wifi_service.h`, runs in its own task). The AP/server exist ONLY in factory mode and are torn down on the reboot that exits it — normal app mode never starts WiFi. |
| both | Web page (login `admin`/`admin`, cookie‑gated): dashboard shows firmware version, running partition, and the **app0→app1 backup result (PASS/FAILED/skipped)**; **firmware upload** (`.bin` → app0); **settings** (CAN baud, KION mode → NVS, next boot); **Reboot** (exits factory to the app). |
| both | Firmware upload is **wired into the existing flasher**: new `can_bootloader_web_flash_begin/write/end/abort()` reuse the exact OTA primitives the CAN flasher uses — `esp_ota_get_next_update_partition()` (→ app0), `esp_ota_begin/write/end`, `esp_ota_set_boot_partition(app0)` — so a browser upload flashes identically to a CAN flash, then reboots. New `can_bootloader_backup_status()` and `can_bootloader_reboot_to_app()`. |
| both | Additive: no change to the partition table, the `0x0B`/`0x0D` CAN‑flash protocol, or normal charging. WiFi is LCD‑only, so **no IOT‑parity mirroring needed**. Gated by `CONFIG_FACTORY_WIFI` (default 1). Stage 2 (runtime `num_chargers` editable from the web) still pending. |
| both | **Build note:** run PlatformIO from **PowerShell**, not Git‑Bash/MSys (the pioarduino platform rejects MSys — that was the cause of the earlier "esptool" build failures). Verified: `pio run` builds clean (full `.bin`) with this feature. Compile‑verified only — WiFi/web/upload need on‑hardware bench testing. |

## 2026-07-28 — Added docs/ChargerLCD.dbc (full CAN interface)

| LCD | Change |
|---|---|
| both | New `docs/ChargerLCD.dbc` — a DBC describing the LCD's complete CAN interface from the LCD's perspective, so integrators know how to talk to it. 22 messages across 4 nodes (LCD / BMS / CHARGER / HOST): RX battery protocols (CLB, generic-CAN, 11.10 HeartBeat + JEM + Event-Info + BMSToCharger setpoint, legacy Coulomb status/pack-params, CAN timestamp), the BA charger command/reply (multiplexed by the CMD byte) + temperature request/reply + current-share, the outgoing ChargerToBMS report, the host config commands (Baud 0x2CC, KION 0x2CD) + ACK (0x2CE), and the bootloader frames (enter 0x0D, flash 0x00B, heartbeat 0x456, beacon 0x702). Includes value tables for all enums and per-message comments (byte-layout quirks, the two HeartBeat layouts, the 0.05/0.01 setpoint factor, the -1600 A ChargerToBMS offset, multi-charger addressing). Docs-only; no firmware change. Validated by round-tripping the dpm.txt / code capture examples through cantools (decodes match exactly). |

## 2026-07-27 — CAN runtime control: KION toggle + baud/KION ACKs

| LCD | Change |
|---|---|
| both | KION mode is now **runtime-toggleable over CAN**, like the baud rate. New RX command `KION_CFG_ID = 0x2CD`, byte0 `0x00` OFF / `0x01` ON. The new state is persisted to NVS (namespace `"ba100"`, key `"kion"`) and applied on the **NEXT boot** — charger control is never switched underneath a live session. `CONFIG_KION_MODE` in `app_config.h` is now just the boot default / empty-NVS fallback. |
| both | KION gating changed from compile-time `#if CONFIG_KION_MODE` to a runtime `s_kion_mode` (loaded from NVS at init, default `CONFIG_KION_MODE`), so the toggle path is always compiled. `charger_tx_allowed()` now reads `s_kion_mode`. New API `ba100_get_kion_mode()`. Init logs the effective state. |
| both | New **ACK** frame `CFG_ACK_ID = 0x2CE` (DLC 3: `[setting, value, effect]`). A baud change (0x2CC) now ACKs with effect = `CFG_EFFECT_NOW` (0), sent at the *current* baud and flushed (`cfg_flush_tx`) before the driver is reinstalled so the host receives it pre-switch. A KION toggle (0x2CD) ACKs with effect = `CFG_EFFECT_NEXT_BOOT` (1). |
| both | No IOT-parity impact (config IDs + charge-side behaviour only; no partition/flash-flow change). New CAN IDs `0x2CD`/`0x2CE` are clear of the bootloader/enter range (`0x0B/0x0C/0x0D`) and adjacent to the existing baud ID `0x2CC`. Verified with `pio run` (7"). |

## 2026-07-27 — Config-selectable default CAN baud (all partitions)

| LCD | Change |
|---|---|
| both | New `CONFIG_CAN_BAUD_KBPS` in `app_config.h` (section 5b; 125/250/500, default 250) — a single knob for the CAN bit rate used by **all partitions**: the app (`ba100_can.cpp`) and the factory CAN-flash bootloader (`can_bootloader.cpp`). Compile-time `#error` rejects any value other than 125/250/500. Also added a helper macro `CONFIG_CAN_TWAI_TIMING()` that expands to the matching `TWAI_TIMING_CONFIG_*KBITS()` for use where a compile-time constant is needed. |
| both | App: `s_baud_kbps` default and the `ba100_can_init()` NVS fallback (missing/corrupt) now come from `CONFIG_CAN_BAUD_KBPS` instead of a hardcoded 250. Factory flasher: `TWAI_TIMING_CONFIG_250KBITS()` → `CONFIG_CAN_TWAI_TIMING()`. |
| both | **Runtime `0x2CC` baud change is unchanged** — it still retunes the app and persists to NVS, overriding the config default on that device. |
| both | The factory CAN-flash bootloader now **reads the same NVS-persisted baud** (Preferences namespace `"ba100"`, key `"baud"`) via new `btl_resolve_can_timing()`, falling back to `CONFIG_CAN_BAUD_KBPS` when NVS is unset/corrupt. So a device retuned at runtime is flashed at its actual bus baud — app, flasher, and bus always agree; no need to keep the flash host on the compile-time default. NVS is initialised by the Arduino core before `setup()` calls the factory path, and the app is not running in factory mode, so the read-only open is safe/uncontended. Logs `TWAI @ N kbps NORMAL (factory flash session)`. |
| both | Fixed stale/contradictory baud comments to reference the config default / NVS behaviour: `main.cpp` (was "500 Kbps" / "125 kbps, listen-only"), `ba100_can.cpp` header (was "500 kbps"), `can_bootloader.h` (was "250 Kbps"). No IOT-parity impact (baud value only; no partition/flow change). Verified with `pio run` (7"). |

## 2026-07-27 — KION mode (legacy-listen-only) + charger environment temperature

| LCD | Change |
|---|---|
| both | New `CONFIG_KION_MODE` in `app_config.h` (`0` = disabled/default, `1` = enabled). When enabled, the LCD stays a **passive display** for a legacy Coulomb BMS: it sends **nothing** to the BA charger module until it has positively identified the battery type, and for a **legacy** pack (`0x18A10002` / `0x18A11B02` → `BATT_LEGACY`) it **never** polls or commands the charger — so that BMS drives the charger directly, uninterrupted (mirrors the old read-only ChargerLCD2 build). Non-legacy batteries (BMS11/CLB/CAN) keep full charger control. When disabled, behaviour is byte-for-byte unchanged. |
| both | Implemented via one authoritative gate `charger_tx_allowed()` in `ba100_can.cpp` (checked in `sol_transmit()`, `sol_send_temp_req()`, `sol_set_current_share()`, the `charger_comms_task` periodics, and at the top of `charger_fsm_run()`), so no charger frame can leak — from the FSM, the poll task, **or the bootloader-entry safety-off `ba100_charger_off()`** — while KION is passive. **OTA / CAN-flashing / the 2 s bootloader heartbeat are untouched** in both modes (the TWAI driver still runs `NORMAL` so frames are ACKed and the flasher works). |
| both | New feature: **charger environment temperature**. Polled separately (it is *not* in the CMD=1 status reply): TX `0x19008080+addr` (empty), RX `0x18008080+addr`, temp = bytes 4-5 BE ×0.1 °C (per `docs/dpm.txt`). Polled at 1 Hz by `charger_comms_task`; per-module value + hottest-fresh aggregate (`mod_temp_C`/`mod_temp_valid`) added to `BA100_State_t`. Shown in the **bottom-right status watermark** as `… | <baud>k | 33.8C`. Because reading temperature requires a poll, it is **unavailable in KION listen-only** (shows nothing). Degree glyph intentionally omitted (`C`, not `°C`) to avoid a missing-font box; change if the UI font carries U+00B0. |
| both | Not IOT-related (charge FSM + temperature are LCD-only). No partition / CAN-flash / enter-trigger changes, so no IOT-parity action needed. Verified: `pio run` (7" `CONFIG_LCD_TYPE=1`) compiles clean with `CONFIG_KION_MODE` both `0` and `1`. 4.3" not separately built — changes are board-agnostic (CAN logic + shared watermark). |

## 2026-07-15 — Power-terminal connection check + "CHARGER TERMINAL OPEN!" popup

| LCD | Change |
|---|---|
| both | New pre-charge check: with the module OFF it reports the **back-fed battery voltage** on its output (CMD=1 status, e.g. `02 FA` = 76.2 V; `00 00` when the terminal is open). Before powering the charger, `charger_fsm_run()` (CHG_IDLE) holds the module OFF and compares `mod_voltage_V` against the BMS pack voltage `voltage_V`. Match within `TERMINAL_MATCH_TOLERANCE_V` (5 V) → `s_terminal_verified`, proceed to normal power-on/charging. No match within `TERMINAL_CHECK_TIMEOUT_MS` (10 s) → latch `s_terminal_open` + `s_session_locked`, charger forced off. New tunables in `app_config.h`; new API `ba100_get_terminal_open()`. |
| both | Charger power-on (`need_power`) and `wants_charge` now gated on `s_terminal_verified`, so the charger never drives its output until the terminal is confirmed connected. Check window opens when battery CAN is present (V0.2 auto) or after **START** (V0.1). |
| both | UI: latched **"CHARGER TERMINAL OPEN!"** modal on `lv_layer_top()` (overlays any screen, absorbs touches, dismissable **only by power cycle**). Battery telemetry keeps updating behind it (dimmed); the V0.2 STOP button reads "CHARGER STOPPED" (session locked). |
| both | Assumes the pack back-feeds voltage to the terminal while the charger is off (confirmed on the target pack). A pack that keeps its charge FET open until it sees a charger would read `00 00` and trip the fault. `s_terminal_verified` is sticky per power cycle (boot-time check), so a mid-session battery reconnect does not re-run it. V0.1 START label under this fault still reads "CHARGE COMPLETE" (popup is the primary indicator) — refine later if needed. Not IOT-related (charge FSM is LCD-only). |

## 2026-07-15 — Factory idle/inactivity recovery shortened to 15 s

| LCD | Change |
|---|---|
| both | Unified the factory "no data received → reboot to a valid app" window to **15 s**. Was: 60 s waiting-for-flasher (`FACTORY_IDLE_RECOVER_MS`) + 10 s mid-flash stall (`BTL_INACTIVITY_TIMEOUT_MS`); both now 15 s. Any 15 s of silence in factory (no `0x0B` bootloader frame) — whether pre-activation or mid-flash — calls `btl_recover_to_app()` (boots app0, else app1 backup; stays in factory only if neither is bootable). Live flash traffic keeps resetting the timer, so an in-progress flash is unaffected. |

## 2026-07-15 — Enter-bootloader (0x0D) refused while battery/BMS connected

| LCD | Change |
|---|---|
| both | `can_bootloader_enter_notify()` now **ignores** the `0x0D` enter trigger when `ba100_get_state().batt_data_valid` is true (a battery/BMS is actively communicating): entering hands the CAN bus to the flasher and drops charger control, so it must not happen with a pack attached. `batt_data_valid` self-clears when the battery is disconnected (telemetry goes stale), so the flasher works again once the pack is removed — no power cycle needed. Reverses the 2026-07-13 "enter always works" behaviour (that removed the *charging* gate; this adds a *battery-present* gate). ⚠ **IOT-parity: this touches the `0x0D` enter path — mirror in the IOT code if that product has equivalent battery telemetry** (it may not, in which case leave IOT unchanged). |

## 2026-07-15 — Fix: stuck in factory after power cycle / failed flash (enter-requested flag)

| LCD | Change |
|---|---|
| both | Factory boot pins `boot=factory` in otadata; the only way back to an app was runtime recovery, which had holes: the 60 s idle recovery is reset by any `0x0B` traffic (a live flasher never lets it fire), and it only fires at all after entry — so a power cycle into factory, or a flash that sent DATA but never committed `boot=app0` (VERIFY/RESET), left the unit sitting in factory even with a valid app0/app1. Fix: a new `RTC_NOINIT_ATTR` **enter-requested** flag (`s_flash_entry_requested`, magic `0xE47E4001`) set by `enter_factory_bootloader()`. On factory boot `can_bootloader_run_if_factory()` checks it: **not set → recover to app immediately** (via new `recover_boot_to_app()`), **set → consume and flash**. Only a deliberate `0x0D` enter keeps the unit in factory. |
| both | Refactored the app0/app1 boot-selection + reboot into `recover_boot_to_app()` (returns false without rebooting when neither app is bootable, so a blank device can still be flashed); `btl_recover_to_app()` now = `btl_abort_session()` + `recover_boot_to_app()`. Also fixes first-boot-after-`upload_full`: reaches app0 in ~1 s instead of waiting the 60 s idle window. ⚠ **IOT parity: mirror the enter-requested flag + immediate recovery in the IOT code.** |

## 2026-07-15 — Fix: app0-only guard always skipped backup (RTC_DATA_ATTR → RTC_NOINIT_ATTR)

| LCD | Change |
|---|---|
| both | The app0-only backup sentinel `s_backup_from_app0` was `RTC_DATA_ATTR`. Uninitialized, that lands in `.rtc.bss`, which the ESP-IDF startup **zeroes on every non-deep-sleep reset** — and our enter path is `esp_restart()` (a SW reset). So the flag always read 0 in `factory` and the bootloader logged "not entered from app0" / skipped the backup **even when booted from app0**. Changed to `RTC_NOINIT_ATTR` (`.rtc_noinit`, never cleared by startup) so it survives `esp_restart()`; only a true power-on leaves garbage → correctly fails the magic check and safely skips. ⚠ **IOT parity: the sentinel must be `RTC_NOINIT_ATTR` there too.** |

## 2026-07-15 — Merge V0.1 + V0.2 charge control (selectable via CONFIG_CHARGE_CONTROL)

| LCD | Change |
|---|---|
| both | Merged V0.1 (START/STOP) and V0.2 (auto-start + STOP-only) charge control into ONE code base, selectable from `include/app_config.h` — **nothing removed**. New `CONFIG_CHARGE_CONTROL`: `0` = V0.1 START/STOP, `1` = V0.2 auto-start + STOP-only (**default 1**, current behaviour). Range-guarded with `#error`. |
| both | `ba100_can.cpp`: both charge-control paths compiled behind `#if CONFIG_CHARGE_CONTROL` — state var (`s_manual_start` vs `s_stop_request`), setter/request fns, the V0.2-only global STOP + telemetry-loss stop, the IDLE detection power-on gate, the charge-start gate, comms-lost handling, and the CHARGING→STOPPING block. `ba100_can.h`: exposes `ba100_set_manual_charge`/`ba100_get_manual_charge` (mode 0) or `ba100_request_stop` (mode 1); `ba100_get_session_locked` in both. |
| both | `ui_fallback.cpp`: both UIs compiled behind `#if CONFIG_CHARGE_CONTROL` — V0.1 green HOLD-TO-START pill + red slide-to-STOP slider (`cb_btn_start`/`cb_slider_stop`) vs V0.2 single long-press STOP button (`cb_btn_stop`, 3 s hold), including widget decls, creation, and the `ui_apply_state()` button block. |
| both | CLAUDE.md updated: the old "no START button / STOP-only is permanent" fixed-decision is superseded — charge control is now a selectable merge (both branches must be kept working). No `pio run -t clean` needed to switch `CONFIG_CHARGE_CONTROL` (only `CONFIG_LCD_TYPE` needs it). |

## 2026-07-15 — app0-only backup guard (don't clobber app1 when booted from app1)

| LCD | Change |
|---|---|
| both | The factory bootloader copied app0→app1 on **every** factory boot. If the device had booted from **app1** (app0 invalid → `btl_recover_to_app` fell back to app1) and the operator then entered the bootloader, the unconditional copy overwrote the good app1 backup with a stale/bad app0 — a brick risk if the next flash failed. Now `enter_factory_bootloader()` records (in an `RTC_DATA_ATTR` sentinel, `s_backup_from_app0`) whether entry was from app0; `can_bootloader_run_if_factory()` runs `copy_app0_to_app1()` **only** when that sentinel matches, then consumes it. Entry from app1 (or a power-cycle into factory, where the RTC sentinel is garbage) skips the backup and preserves app1. ⚠ **IOT-parity: mirror this in the IOT code** (enter-handler records source partition; factory boot gates the backup). |

| LCD | Change |
|---|---|
| both | The `0x0D` enter-bootloader was gated by `charger_safe_for_btl()` (charger must be idle). In V0.2 charging auto-starts, so with a battery connected the charger was busy and `0x0D` was silently ignored ("charger not idle"). Removed the gate: enter now **always works** — `enter_factory_bootloader()` calls new `ba100_charger_off()` (synchronous 0 A + power-off to all modules) FIRST, then reboots into factory. Safe even mid-charge. Also `s_entering` now resets if entry fails (no factory partition), so a failed attempt no longer permanently blocks retries. |

## 2026-07-13 — Backup timing fix + copy progress bar

| LCD | Change |
|---|---|
| both | **Fixed stale/invalid app1 backup.** The app0→app1 backup was done in the app before rebooting to factory (blocking, no feedback, could silently fail → recovery booted a days-old app1). It now runs **in the factory partition, right after boot, BEFORE any flashing**, reading the **app0 partition explicitly** (running partition there is `factory`) — so app1 always captures the firmware that was running when the operator entered the bootloader. Full-partition copy so `esp_ota_end` validates it. |
| both | Added a **progress bar** ("Backing up firmware…") on the bootloader screen during the copy; yields periodically (feeds WDT / lets LVGL render). Beacons (0x456/0x702) now start only AFTER the backup, so "Ready to flash" appears once flashing is actually possible. `can_bootloader.cpp` byte-identical to V0.1. |

## 2026-07-13 — V0.2 bootloader auto-recovery + status line

| LCD | Change |
|---|---|
| both | Bootloader auto-recovery: after a failed/cancelled/incomplete flash (mid-flash inactivity 10 s) OR if no flasher connects within 60 s, the factory bootloader now boots back into a working app instead of staying stuck in factory. `btl_recover_to_app()` tries **app0** first; if app0 is invalid (half-written / erased-not-written) `esp_ota_set_boot_partition` rejects it and it falls back to **app1** (the pre-flash backup). If neither is bootable it stays in factory to retry. ⚠ IOT-parity: keep this recovery in sync with the IOT code. |
| both | `btl_abort_session()` no longer reboots — a host RECONNECT now correctly re-arms in place; only give-up/failure paths call `btl_recover_to_app()`. |
| both | Note: `0x456`/`0x702` are emitted ONLY from the factory partition (never app0/app1). Persistent `0x456` after a power-cycle = still booting factory (boot was pinned to factory by a failed flash) — now self-heals via the 60 s idle recovery. |
| both | UI: moved the version / battery-type / CAN-baud watermark from bottom-LEFT to bottom-RIGHT as one line, and added the running partition (**app0 / app1 / factory**): e.g. `v2.1.0 \| app0 \| BMS \| 250k`. |

## 2026-07-13 — V0.2 STOP tuning

| LCD | Change |
|---|---|
| both | Fixed STOP not stopping: the request is now handled GLOBALLY at the top of `charger_fsm_run()` (forces 0 A + power-off from any active state, not just CHG_CHARGING); `ba100_request_stop()` latches `session_locked` instantly so the button greys immediately. Button label → "CHARGER STOPPED". |
| both | STOP hold time increased to **3 seconds** (`STOP_HOLD_MS`, custom hold via PRESSED + LONG_PRESSED_REPEAT + `lv_tick_elaps`). Label "HOLD 3s TO STOP". |
| both | On battery telemetry loss (fields go "-", `batt_data_valid` false): STOP button greys out AND the charger is stopped immediately (new check in `charger_fsm_run`, faster than the 10 s demand-timeout). NON-latching — auto-resumes when data returns and the BMS commands again. |

## 2026-07-11 — V0.2 forked from V0.1 — charge control reworked

| LCD | Change |
|---|---|
| both | Forked ChargerLCD_combinedV0.2 from V0.1. **Only difference: charge control.** |
| both | Removed the START button. Charging now **auto-starts** (BMS-led) once a battery is detected and the BMS commands it — reverting to the pre-button behaviour. (`ba100_can.cpp`: dropped the `s_manual_start` gate on IDLE detection-power-on and on `wants_charge`.) |
| both | STOP is now a **long-press button** (was a slide-to-stop slider), always visible, **disabled unless charging**. Label: "HOLD TO STOP" (charging) / "CHARGE COMPLETE" / "STOPPED" / "WAITING FOR BATTERY" / "READY". |
| both | Stop now **latches the session** (no auto-restart until power cycle) for ALL clean stops — charge-complete, user STOP, and hard fault. A comms/demand timeout still does NOT latch (auto-resumes on reconnect). |
| both | CAN API: `ba100_set_manual_charge()`/`ba100_get_manual_charge()` → replaced by `ba100_request_stop()`. `ba100_get_session_locked()` kept. FW version → v2.1.0. |
| both | Everything else (partitions, IOT-parity CAN flashing, LCD/charger config) unchanged from V0.1. Build verified (compile + link). |

---


> **RULE (for anyone — human or AI — editing this project):**
> Before making a code change, know which LCD the change is for.
> **If you are not sure, ASK the user: "Which LCD are we working on — 4.3" or
> 7"?"** and record the answer here with the change. Never guess.

Legend for the **LCD** column:
- `4.3"` — change made for / tested on the 4.3" board (CONFIG_LCD_TYPE 0)
- `7"`   — change made for / tested on the 7" board (CONFIG_LCD_TYPE 1)
- `both` — hardware-independent change (shared logic), affects both builds

---

## 2026-07-11 — IOT-parity CAN-flash / partition scheme

| LCD | Change |
|---|---|
| both | Switched to IOT-style 3-slot partition table (`partitions.csv`: factory + ota_0 + ota_1, 4 MB each, 16 MB). App runs from app0; CAN-flash bootloader runs from factory; app1 is the app0 backup. |
| both | `0x0D` enter-bootloader trigger now backs up app0→app1 (4 KB blocks) then sets boot=factory and reboots — instead of flashing in place. Factory runs the CAN-slave flasher (heartbeat 0x456 + beacon 0x702, protocol on 0x0B, writes into app0) with a minimal "Updating" screen. `main.cpp` runs `can_bootloader_run_if_factory()` first. |
| both | Added `merge_firmware.py` + `upload_full` target (writes firmware to BOTH factory and app0). App no longer flashes in place; 0x0B/0x69 no longer handled in app mode. |
| both | **IOT parity:** this whole scheme is kept identical to the IOT code — deliberate divergence: no EC200 modem, so factory only does CAN-slave flashing. **Any change here must be mirrored in the IOT code** (see notices in partitions.csv, can_bootloader.cpp, merge_firmware.py, CLAUDE.md). |
| both | ⚠ Partition-table change → devices must be fully re-flashed (`pio run -t upload_full`), not app-only. |

## 2026-07-06 — config cleanup

| LCD | Change |
|---|---|
| both | Removed the `CONFIG_CHARGER_TO_BMS_BYTE0_DBC_FMT` manual option. The proven ACE.dbf byte0 format is now used unconditionally in `send_charger_to_bms11()` (it was hardcoded to 1 in the base 4.3" build and never toggled). NOTE: this is the OUTGOING ChargerToBMS report format — it is NOT the incoming battery auto-detection, which is unaffected. |
| both | Fixed a byte0/CAN-pin `#if` typo in app_config.h — the 4.3" branch tested `CONFIG_LCD_TYPE == 1` instead of `== 0`, which broke the 4.3" build (fell through to the `#error`). |

## 2026-07-05 — v2.0.0 — Project created (merge of three projects)

| LCD | Change |
|---|---|
| both | Project created by merging `ChargerLCD6_4.3inch_single` (BASE, latest CAN/BA100 handling v1.0.7), `ChargerLCD5_7inch_double` (multi-charger + current sharing), `ChargerLCD6_7inch_single` (7" pinout + START/STOP). See FEATURES.md for full provenance. |
| both | Central config `include/app_config.h`: `CONFIG_LCD_TYPE` (0=4.3", 1=7"), `CONFIG_NUM_CHARGERS` (1–60), tunables moved from ba100_can.cpp. |
| both | Multi-charger drive generalised to 1–60 modules; equal current split, round-up to multiple of N amps, reply dispatch by CAN-ID range, ALL-online start gate, degraded mid-charge loss handling. |
| both | Automatic current sharing (0x19C21880) — asserted automatically whenever `CONFIG_NUM_CHARGERS > 1` (derived, not a user option). |
| both | START/STOP front-panel control made a PERMANENT feature (config option removed on user request): HOLD-TO-START pill + slide-to-STOP slider, session lock until power cycle after completion/fault, 2 s post-stop safety re-send. Layout: left column = logo (278 px) + button area (120 px), logo zoom 165. |
| 7" | CAN on GPIO20/19 via FSUSB42 mux (CH422G USB_SEL=IO5 driven HIGH in main.cpp before CAN init). |
| 7" | USB-CDC **forced OFF at build time** (`scripts/config_from_header.py` reads `CONFIG_LCD_TYPE`); `#error` guard in main.cpp if a stale build has CDC on. Reason: GPIO19/20 are the native-USB pins — an enabled CDC with no host blocks Serial and freezes CAN+UI. Serial logs go to UART0 (GPIO43/44) / SD card. |
| 4.3" | USB-CDC stays ON (USB-C serial debugging available; CAN is on dedicated GPIO15/16). |
| both | Builds verified for: 4.3"/1-charger and 7"/3-charger configs. |
