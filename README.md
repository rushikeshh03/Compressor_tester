# Coulomb Harness Test Station

Firmware + hardware documentation for the wire‑harness **test station**. An operator
scans a harness barcode; the station runs it through a CAN‑based test jig and shows
**PASS / FAIL** with per‑cell voltages and temperatures, logs every result, and
gates the operator on failures.

> ⚠️ **AI agents / contributors:** this file and the per‑board docs are the source
> of truth. **Keep them updated** whenever you change hardware wiring, pins, the CAN
> protocol, the board architecture, or a user‑visible feature. See
> [`CLAUDE.md`](CLAUDE.md) → *Documentation maintenance* for the exact rule.

---

## 1. System overview

The station is **three ESP32 boards on one CAN bus**, plus the harness test jig
(the "DAS", an external SAME54 board — **not in this repo**) and an Android tablet.

```
                    ┌──────────────────────────────────────────── CAN bus (500 kbps) ───────────────┐
                    │                        │                         │                             │
             ┌──────┴──────┐          ┌──────┴───────┐          ┌──────┴───────┐              ┌───────┴────────┐
   USB‑HID   │  CAN_Bridge │  UART    │   Test Jig   │          │   IOT_Board  │ WiFi HTTP+MQTT│ Local Data     │
  scanner ─► │ ESP32‑WROOM │◄════════►│  "DAS" master│          │  ESP32‑S3    │ ───────────► │ Server (LAN)   │
             │ (transceiver)│  1 Mbps  │  (SAME54,    │          │ passive      │              └────────────────┘
             └──────┬──────┘   ▲       │   external)  │          │ listener +   │
                    │          │       └──────────────┘          │ SD-log sync  │──BLE HID──► Android tablet
             ┌──────┴──────┐   │ UART bridge                     └──────────────┘
             │     LCD     │───┘  (GPIO43/44)
             │ ESP32‑S3    │  the operator UI + test controller
             │ 7" touch    │  (USB scanner, SD log, FAIL lock‑out)
             └─────────────┘
```

| Board | Folder | MCU / hardware | Role |
|---|---|---|---|
| **LCD** | [`LCD/`](LCD/) | Waveshare **ESP32‑S3‑Touch‑LCD‑7** (800×480 RGB + GT911 touch) | Operator UI + test controller. Reads the USB‑HID scanner, sends the barcode to the jig and shows PASS/FAIL, logs to SD, and drives the FAIL lock‑out. |
| **CAN_Bridge** | [`CAN_Bridge/`](CAN_Bridge/) | **ESP32‑WROOM** (esp32dev) + CAN transceiver | Transparent **UART↔CAN bridge** — the LCD has no CAN transceiver, so all its CAN traffic is relayed here. |
| **IOT_Board** | [`IOT_Board/`](IOT_Board/) | **ESP32‑S3‑WROOM‑1 N8R2** + SD + CAN | Passive CAN listener → logs to SD and **syncs those logs to a LOCAL Data Server** (per‑barcode CSV over HTTP + MQTT control; daily raw/event bundle). Also types the scanned barcode to a tablet over **BLE‑HID**. |
| Test jig ("DAS") | *(external)* | SAME54 + MAX17841B | The device under test's master — runs the actual harness test and broadcasts results. Protocol only; no code here. |

### Why three boards?
The LCD's only CAN‑capable pins (GPIO19/20) are shared with its USB port via an
on‑board mux, and USB is needed for the **USB‑HID scanner**. Rather than sacrifice
one for the other, CAN was moved off‑board to the **CAN_Bridge**, so the scanner
(USB) and CAN (bridge, over UART) run at the same time. See the
[design history](#7-design-history--decisions) for the full story.

---

## 2. CAN protocol (harness_jig **v5** DBC)

Authoritative files: [`LCD/docs/harness_jig_v5.dbc`](LCD/docs/harness_jig_v5.dbc) +
`harness_jig_v5_README.md` (copied into both `LCD/docs` and `IOT_Board/docs`).
**Standard 11‑bit IDs, 500 kbps.**

| ID | Name | Dir | Meaning |
|---|---|---|---|
| `0x600/601/602` | Barcode frames 1–3 | Tester → Jig | Scanned barcode, ASCII, up to 20 chars (frame3 = chars 16–19) |
| `0x710` | ResultStatusBarcode | Jig → Tester | `Seq` 0–2: Status (0=FAIL, 1=PASS) + barcode echo |
| `0x711` | ResultCells | Jig → Tester | 8 averaged cell voltages, `uint16` LE, mV |
| `0x712` | ResultTemps | Jig → Tester | 4 averaged temps, **signed** `int16` LE, 0.1 °C |
| `0x713` | HarnessFault | Jig → Tester | 6 signals — avg masks (b0/b1/b3) + **raw‑sample** detail (b4 `CellRawFaultMask`, b5 `TempRepeatabilityFaultMask`, b6 `PassingReadingCount`) |
| `0x720` | Heartbeat | Jig → Tester | ~500 ms board‑alive + CAN TEC/REC/LEC/txq |
| `0x7A0` | IoT heartbeat | IOT → bus | 1 Hz: magic `0xA5`, flags (tablet/MQTT/CAN), counter |
| `0x18A10001` | BMS time‑sync | ext | Sets the LCD wall clock |
| `0x2D0` | Restart‑IoT | Tester → IoT | `'R''S'` — reboots the IoT board |
| `0x0C / 0x0D` | Enter bootloader | — | CAN self‑update (IOT / LCD). **Not relayed through the bridge.** |
| `0x2CC / 0x2E0` | Baud change | — | LCD / IOT runtime baud (LCD's is retired — bridge owns the CAN baud) |

**A test cycle:** the LCD sends `0x600(+601+602)`; a few seconds later the jig bursts
`0x710/711/712/713`. The IOT logs every frame to SD and syncs a per‑scan raw‑CAN CSV
(barcode→next barcode) to the local Data Server; the LCD decodes it for the UI + SD log.
See the DBC README for field‑level detail and the important
"FAIL with clean average masks" note (bytes 4–6 explain it).

---

## 3. Hardware wiring

### 3.1 LCD (Waveshare ESP32‑S3‑Touch‑LCD‑7)
The 7" RGB panel + touch consume most GPIOs. Free/used pins that matter:

| Signal | Pin(s) | Notes |
|---|---|---|
| **UART to CAN_Bridge** | GPIO43 (TX), GPIO44 (RX) | The "UART2" header P1 — **set the on‑board UART1/2 DIP switch to UART2**. Both boards 3.3 V → no level shifting. |
| **USB‑HID scanner** | GPIO19 (D‑), GPIO20 (D+) | Native USB. `USB_SEL` (CH422G IO5) held **LOW** routes these to the USB‑C port. USB host must supply 5 V VBUS to the scanner. |
| **Sensor‑AD FAIL output** | GPIO6 | "Sensor AD" header (P2: 3.3V/GND/AD). Digital output, HIGH on unresolved FAIL. 3.3 V ~20 mA — use a driver for bigger loads. |
| micro‑SD | GPIO11/12/13 (MOSI/SCK/MISO), CS = CH422G EXIO4 | 4 MHz SPI |
| Touch / IO‑expander I²C | GPIO8 (SDA), GPIO9 (SCL) | GT911 + CH422G |
| RGB LCD | many (HSYNC46/VSYNC3/DE5/PCLK7 + 16 data) | see `LCD/include/esp_panel_board_custom_conf.h` |

⚠ **No LCD serial console:** GPIO43/44 are UART0, now used for the bridge link, so the
LCD's USB‑serial console is gone — **LCD logs go to the SD card**. Flash the LCD over USB.

### 3.2 CAN_Bridge (ESP32‑WROOM)

| Signal | Pin | Notes |
|---|---|---|
| UART to LCD | GPIO16 (RX), GPIO17 (TX) | ← LCD GPIO43 / → LCD GPIO44, 1 Mbps |
| CAN | GPIO5 (TX), GPIO4 (RX) | to the on‑board CAN transceiver → CANH/CANL |
| LED — CAN activity | GPIO2 (onboard) | flickers on bus traffic |
| LED — UART/link | GPIO25 (external + ~330 Ω) | solid = LCD link up |
| LED — heartbeat | GPIO26 (external + ~330 Ω) | ~1 Hz = firmware alive |
| USB (UART0) | — | CAN debug console @115200 (`DBG_VERBOSE`) |

120 Ω termination at each end of the CAN bus.

### 3.3 IOT_Board (ESP32‑S3‑WROOM‑1 N8R2)

| Signal | Pin | Notes |
|---|---|---|
| CAN | GPIO4 (TX), GPIO5 (RX) | to its own CAN transceiver |
| USB (native) | GPIO19/20 | CDC log console (not a host; a blocking diode prevents VBUS out) |

---

## 4. Software layout

### LCD ([`LCD/`](LCD/), pioarduino / arduino‑esp32 3.3.8)
- `src/main.cpp` — boot: board+LVGL, `USB_SEL` LOW, CAN‑bridge link, SD, UI, USB scanner, link manager.
- `src/ba100_can.*` — CAN layer. **Transport is the UART bridge** (`bridge_send`/`bridge_recv`, 16‑byte frames) but keeps all DAS parsing. Also the bridge link‑heartbeat + reboot control frames, and `bridge_alive`.
- `src/harness_conn.*` — the controller: drains the USB scanner, runs the check, records to SD, drives the **FAIL lock‑out** + Sensor‑AD pin, and the Restart‑system flow.
- `src/usb_hid_scanner.*` — USB‑HID host (EspUsbHost) — the sole barcode input.
- `src/ui/ui_fallback.cpp` + `ui.h` — LVGL UI: main page (logo + Restart‑system, barcode + PASS/FAIL, top‑bar chips) and Cell‑Data page (cells/temps/fault + version watermark), plus the FAIL popup.
- `src/sd_logger.*` — per‑day CSVs (`data_*.csv`, `events_*.log`) + **per‑scan raw‑CAN
  session files** (`/barcodes/<bc>_<stamp>.csv`) via `sd_logger_session_open/close` +
  `sd_logger_frame` (raw frames logged from `ba100_can`).
- `src/can_bootloader.*` — CAN‑flash bootloader (dormant now; flash over USB).
- `src/sync_service.*` — **SD → Node‑RED sync client** (app‑mode WiFi + NTP): mirrors
  closed `/barcodes` files to a server, file bytes + SHA‑256 over **HTTP**, presence +
  hourly reconcile over **MQTT** (see §5 + [`node-red/`](node-red/)). Reads the card
  through the logger's shared lock; never touches the daily logs or the open file.
- `src/wifi_service.*` — factory‑mode SoftAP web updater (CAN‑flash recovery only).
- `include/app_config.h` — **central config** (pins, baud, feature flags). Edit here, not `platformio.ini`.

### CAN_Bridge ([`CAN_Bridge/`](CAN_Bridge/), espressif32@6.9.0)
- `src/main.cpp` — the whole bridge (~200 lines): TWAI ↔ UART framing, LEDs, LCD link heartbeat, reboot command. No libraries.

### IOT_Board ([`IOT_Board/`](IOT_Board/), espressif32@6.9.0)
- Passive CAN capture → lossless ring → SD NDJSON log, then **synced to a LOCAL Data
  Server** (`src/das_sync.*`): a **per‑scan raw‑CAN CSV** (`/barcodes/<barcode>_<stamp>.csv`,
  every frame from one barcode to the next — like the LCD/EOL `sd_logger`) over HTTP
  `POST /api/upload` on each scan, plus a **decoded result summary over MQTT**
  (`das/<station>/<dev>/result` — PASS/FAIL + cells/temps for yield/analytics), plus a daily
  23:00 raw‑segment + event‑log bundle; MQTT also carries control (presence/status/sync).
  Replaces the old cloud MQTT frame uplink. BLE‑HID
  keyboard types the barcode to a tablet. Server protocol:
  `../Testing/Data Server/docs/PROTOCOL.md`. See [`IOT_Board/CLAUDE.md`](IOT_Board/CLAUDE.md).

---

## 5. Key features / behaviours

- **USB‑HID barcode scanner** is the sole input (BLE was removed).
- **FAIL lock‑out:** on a harness FAIL the **Sensor‑AD pin (GPIO6)** goes HIGH and a
  main‑page popup appears. The failed harness must be re‑scanned (retest) or the
  operator must **Override** before a different harness is accepted; the pin drops
  when any fail popup closes. The popup is main‑page‑only and lets swipes through so
  navigation still works.
- **Restart system** (main page): reboots the IoT (over CAN), the CAN bridge, and
  the LCD together.
- **Top‑bar chips:** CAN (bus traffic) · UART (LCD↔bridge link) · IoT · Scanner
  (USB) · Tablet · Uplink · SD.
- **SD logging:** every scan → `data_*.csv` (result + 8 cells + 4 temps + 6 fault
  columns); events → `events_*.log`; **plus a per‑scan raw‑CAN file**
  `/barcodes/<barcode>_<stamp>.csv` capturing every CAN frame (RX + our TX) from
  that scan until the next barcode (`Time,Dir,ID,DLC,Data,Decoded`).
- **SD → Node‑RED sync** (`LCD/src/sync_service.*`, since 2026‑09‑23): the LCD joins
  the local **WiFi**, reads **NTP** (so SD files finally get real dated names), and
  mirrors every **closed** `/barcodes` file to a **Node‑RED** server — file bytes +
  **SHA‑256** over **HTTP** (`POST /sync/file`, acked `OK`/`FAIL`, resend on FAIL) and
  station presence + an **hourly today‑manifest reconcile** over **MQTT** (station
  `"harness tester"`). It never sends the `data_`/`events_` logs or the *currently‑open*
  scan file, and holds while WiFi is down. App‑mode WiFi became viable again once BLE
  was removed (USB‑HID scanner). Server flow + wire protocol: [`node-red/`](node-red/).
  Config lives in `LCD/include/app_config.h` (`CONFIG_SYNC_*` / `CONFIG_WIFI_*` /
  `CONFIG_MQTT_*` / `CONFIG_STATION_*`).

---

## 6. Build & flash

Run PlatformIO from **PowerShell** (MSys/Git‑Bash breaks a pending esptool install):

```powershell
# LCD
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -d LCD            # build
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -d LCD -t upload  # flash (USB)

# CAN_Bridge
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -d CAN_Bridge -t upload

# IOT_Board
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -d IOT_Board -t upload
```

**⚠ Toolchain gremlins (Windows / pioarduino):**
- The LCD's pioarduino platform sometimes loses its Arduino framework package
  (`TypeError: expected str … not NoneType` / `FRAMEWORK_DIR` is None). Fix:
  `pio pkg install -d LCD` (reinstalls `framework-arduinoespressif32@3.3.8`), then build.
- Building a board on a *different* espressif32 platform (the bridge/IOT use
  `@6.9.0`) can disturb the LCD's framework — after building those, the LCD may need
  the `pkg install` above before it builds again.
- Missing `esptool.py` in `tool-esptoolpy` → `pio pkg install` for that board.
- **The project path has spaces** (`…\My Drive\…\Harness testing`) — fine for the
  precompiled bridge/IOT platforms, but from‑source ESP‑IDF rebuilds (custom
  sdkconfig) reject it. Don't enable custom_sdkconfig here.

When you change CAN baud or the UART frame format, change it on **both** the LCD
(`LCD/include/app_config.h`) and the bridge (`CAN_Bridge/src/main.cpp`).

---

## 7. Design history / decisions

Condensed; full detail in the per‑board `CHANGELOG.md` + `CLAUDE.md`.

1. Forked from `ESP32LCD_Charger`; charger/BMS protocol removed.
2. Barcode input evolved: BLE‑HID scanner → **USB‑HID scanner** (BLE removed).
3. CAN transport evolved: on‑board mux (conflicts with USB) → external transceiver
   idea → **UART bridge to a dedicated ESP32‑WROOM** (`CAN_Bridge`), because it
   keeps USB + CAN both alive with no level shifting and gives back a CAN console.
4. DBC upgraded to **harness_jig v5** (0x713 gained raw‑sample bytes 4–6; temps are
   signed).
5. Added: bridge status LEDs + link heartbeat, a top‑bar UART chip, Restart‑system
   (IoT+bridge+LCD), and the FAIL lock‑out with the Sensor‑AD output.

---

## 8. Per‑board docs

- LCD: [`LCD/CLAUDE.md`](LCD/CLAUDE.md), [`LCD/README.md`](LCD/README.md), [`LCD/FEATURES.md`](LCD/FEATURES.md), [`LCD/CHANGELOG.md`](LCD/CHANGELOG.md)
- CAN_Bridge: [`CAN_Bridge/README.md`](CAN_Bridge/README.md)
- IOT_Board: [`IOT_Board/CLAUDE.md`](IOT_Board/CLAUDE.md), [`IOT_Board/CHANGELOG.md`](IOT_Board/CHANGELOG.md)
- SD → Node‑RED sync server: [`node-red/README.md`](node-red/README.md) + [`node-red/harness_sync_flow.json`](node-red/harness_sync_flow.json)
- CAN protocol: `*/docs/harness_jig_v5.dbc` + `harness_jig_v5_README.md`
