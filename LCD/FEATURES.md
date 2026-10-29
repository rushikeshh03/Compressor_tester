# ChargerLCD_combinedV2.1.0 — Feature provenance & configuration

One firmware for every hardware combination (4.3"/7" LCD, 1–60 parallel BA
charger modules. **Charge control is SELECTABLE via `CONFIG_CHARGE_CONTROL`
(both compiled in): `1` = V0.2 auto-start + long-press STOP only, no START
button (default); `0` = V0.1 HOLD-TO-START pill + slide-to-STOP.**). All
selection happens in **`include/app_config.h`** — the single central config
file.

## Source projects

| Project | Role in this merge |
|---|---|
| `ChargerLCD6_4.3inch_single` | **BASE.** Latest working build (v1.0.7) — all files were copied from here, then extended. Contains the newest CAN handling and BA100 logic. |
| `ChargerLCD5_7inch_double` | Multi-charger drive + automatic current sharing + degraded mid-charge loss handling. |
| `ChargerLCD6_7inch_single` | 7" LCD support (CAN pin mux, USB-CDC handling) + front-panel START/STOP charge control. |

## Features taken from ChargerLCD6_4.3inch_single (BASE — kept as-is)

Everything not listed under the other two projects, notably the latest CAN /
BA100 handling that was NOT present in the older builds:

- **Dual BMS11 telemetry sourcing** — old single-frame HeartBeat (byte 0-6 +
  ChgFET in byte 7) AND new split-frame battery (`JEM_BMS_to_VCU` 0x18904001 +
  `BMS_EVENT_INFO` 0x18FF07D0) parsed in parallel, with per-frame layout
  auto-detect and latch (`s_bms11_new_layout`).
- **Separate charge-FET (sc_relay) freshness tracking** — FET state goes stale
  on its own if 0x18FF07D0 stops while the heartbeat keeps streaming.
- **Setpoint voltage factor by battery generation** (0x18FF005B: ×0.05 new /
  ×0.01 old) and byte0 status masking.
- **ChargerToBMS in ACE.dbf (ZIVAN) DBF format** (byte0 state+pwrline packing,
  0.05 V/bit and 0.05 A/bit −1600 offset factors) — used unconditionally (the
  outgoing report format; not the incoming battery auto-detection).
- **Demand watchdog correctness** — telemetry frames never stamp the demand
  watchdog; only real charge requests do.
- **Charge-timer gating on fresh telemetry** (`batt_data_valid`) in the UI.
- Charging FSM with all FIXes 1–19: detection voltage offset, soft-current
  ramp (coarse+fine), adaptive foldback (FIX 12), charger-availability start
  gate (FIX 16), clean re-init on comms restore (FIX 17), power-on re-assert
  (FIX 19), jitter-free comms task, SD raw-frame logging, CAN OTA bootloader,
  runtime baud switching (0x00C), fault registry UI, charge-time persistence.

## Features taken from ChargerLCD5_7inch_double

- **Multi-charger drive**, generalised from 2 to **1–60 modules**
  (`CONFIG_NUM_CHARGERS`, protocol address range 1–60 per
  `docs/dpm.txt` / "Digital Power Module CANbus Protocol"):
  - Equal current split across modules in `sol_send_cccv()` (each clamped to
    `CHARGER_MAX_CURRENT_EACH_A`, total capped at each×N).
  - Request rounding UP to a multiple of N amps (generalised from the V5
    even-amp rounding) so the split is whole amps per module.
  - Every command (CCCV / power / status-req) sent to all modules; replies
    dispatched by CAN-ID range `0x1807C080+1 .. +N`.
  - Aggregate telemetry: total current = SUM, bus voltage = mean;
    `mod_data_valid` start gate requires **ALL** modules online.
- **Automatic current sharing** (`0x19C21880`, data[3]=0xAA) — broadcast at
  init and re-asserted once when a charger first comes online
  (`CONFIG_ENABLE_CURRENT_SHARE`, default ON when N > 1).
- **Degraded mid-charge charger-loss handling** — a module lost while charging
  does NOT stop the session; survivors hold their 1/N share and a returning
  module is re-powered and re-commanded so it rejoins.

## Features taken from ChargerLCD6_7inch_single

- **7" LCD (Waveshare ESP32-S3-Touch-LCD-7) support** (`CONFIG_LCD_TYPE 1`):
  - CAN on GPIO20 (TX) / GPIO19 (RX) — the native-USB pins, muxed to the CAN
    transceiver by the FSUSB42 switch; main.cpp drives CH422G USB_SEL (IO5)
    HIGH before CAN init.
  - USB-CDC OFF for this board (blocking-Serial freeze fix) — set
    automatically at build time by `scripts/config_from_header.py`, with an
    `#error` guard in main.cpp if the flag mismatches.
  - Board name in `esp_panel_board_custom_conf.h` (same 800×480 ST7262 panel
    timing as the 4.3B).
- **Charge control — selectable via `CONFIG_CHARGE_CONTROL` (both compiled in).**
  Default `1` = V0.2 (below); `0` = V0.1 (green HOLD-TO-START pill + red
  slide-to-STOP slider, a user STOP re-arms). The V0.2 default:
  - **No START button.** Charging starts **automatically** once a battery is
    detected and the BMS commands it (BMS-led, pre-button behaviour).
  - Red **HOLD 3s TO STOP** button (long-press, not slide), **always visible
    but disabled unless charging**.
  - **Session lock**: any clean stop — charge complete, user STOP, or hard
    fault — latches until power cycle (no auto-restart). Label shows
    "CHARGE COMPLETE" / "CHARGER STOPPED" accordingly.
  - **Post-stop safety re-send**: ~2 s after a stop, power-off + 0 A re-asserted.
  - Demand timeout (cable pulled) does NOT latch — charging auto-resumes on
    reconnect.
  - Layout: left column splits into logo card + 120 px button area (logo zoom
    165); right column unchanged.

## New in the combined project (glue)

- `include/app_config.h` — single central config: LCD type, charger count
  (compile-error outside 1–60), current-share and START/STOP toggles,
  per-module current cap, FW version, CAN pins
  (derived from LCD type), and ALL charging tunables previously defined in
  `ba100_can.cpp` (detect voltage/current, ramp, foldback).
- `scripts/config_from_header.py` — PlatformIO pre-script that derives
  `ARDUINO_USB_CDC_ON_BOOT` from `CONFIG_LCD_TYPE` so platformio.ini needs no
  hand-editing. **Run `pio run -t clean` after changing the LCD type.**
- BA reply RX dispatch generalised from fixed IDs to the 1–60 address range.
- `docs/` carries all charger/BMS protocol references from the base project
  plus `dpm.txt` (module protocol text, source of the 1–60 address range).

## Configuration quick reference (`include/app_config.h`)

| Option | Values | Default | Meaning |
|---|---|---|---|
| `CONFIG_LCD_TYPE` | 0 / 1 | 0 | 0 = 4.3" board, 1 = 7" board |
| `CONFIG_NUM_CHARGERS` | 1–60 | 2 (this build) | parallel BA modules (addresses 1..N) |
| `CONFIG_ENABLE_CURRENT_SHARE` | auto | N > 1 | AUTOMATIC (derived, do not edit): 0x19C21880 current-share broadcast whenever more than one charger is configured |
| `CONFIG_CHARGER_MAX_CURRENT_EACH_A` | float | 100.0 | per-module current cap |
| `CONFIG_CHARGE_CONTROL` | 0 / 1 | 1 | 0 = V0.1 START/STOP, 1 = V0.2 auto-start + STOP-only (both compiled in) |
| `CONFIG_KION_MODE` | 0 / 1 | boot default | legacy-listen-only mode; runtime-toggleable over CAN (0x2CD), applies next boot |
| `CONFIG_CAN_BAUD_KBPS` | 125/250/500 | 500 (this build) | default CAN baud; runtime-overridable (0x2CC), shared by app + flasher |
| `CONFIG_FW_VERSION` | string | v2.1.0 | version shown in the bottom-right status watermark |
| charging tunables | — | — | detect voltage/current, ramp, foldback |

Reproducing the three original hardware setups:

| Original project | `CONFIG_LCD_TYPE` | `CONFIG_NUM_CHARGERS` |
|---|---|---|
| ChargerLCD6_4.3inch_single | 0 | 1 |
| ChargerLCD5_7inch_double | 1 | 2 |
| ChargerLCD6_7inch_single | 1 | 1 |

(Notes: with the default `CONFIG_CHARGE_CONTROL 1` (V0.2) every combination
auto-starts charging and shows only the long-press STOP button; set it to `0`
for the V0.1 START/STOP behaviour. The two 7" reproductions additionally gain
all base-project CAN/BMS fixes those older builds did not have.)
