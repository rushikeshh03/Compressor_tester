# ChargerLCD — Coulomb Li Tech Charger Display (v2.1.0)

Firmware for the Coulomb Li Tech EV/battery **charger display**: an ESP32‑S3
touch LCD that reads battery/BMS demand off the CAN bus, drives one or more
**Beian BA100‑series** charging modules to satisfy it, and shows live pack and
charger status on screen. A CAN OTA bootloader is built in for field updates.

> **One code base, many builds.** A single firmware supports both Waveshare LCD
> boards, 1–60 parallel charger modules, and multiple battery protocols. All
> hardware/feature selection lives in [`include/app_config.h`](include/app_config.h).

---

## Hardware

| | |
|---|---|
| **MCU / board** | Waveshare **ESP32‑S3‑Touch‑LCD‑4.3B** *or* **ESP32‑S3‑Touch‑LCD‑7** (800×480 ST7262 RGB + GT911 touch, CH422G IO expander) |
| **Charger** | Beian **BA100‑10000** digital power module(s), 100 A each, 1–60 in parallel on one DC bus |
| **CAN** | TWAI @ 125/250/500 kbps. 4.3B: GPIO15/16. 7": GPIO20/19 (native‑USB pins, FSUSB42‑muxed — USB‑CDC must be off) |
| **Storage** | microSD (raw CAN frame logging), NVS (persisted baud / KION / last charge time) |

The board is selected with `CONFIG_LCD_TYPE` (`0` = 4.3B, `1` = 7"). The build
system sets `ARDUINO_USB_CDC_ON_BOOT` automatically from that value.

## Features

- **Battery auto‑detection** across four protocols: CLB (`0x18FF0001`), generic
  CAN (`0x18FF0002`), 11.10/ZIVAN BMS (`0x18963257` + `0x18FF005B`), and legacy
  Coulomb (`0x18A10002` / `0x18A11B02`).
- **Multi‑module charging** — drives 1–60 BA modules; the BMS current request is
  split equally, each module capped, all must be online before current flows.
  Automatic hardware current‑sharing when >1 module.
- **Selectable charge control** (`CONFIG_CHARGE_CONTROL`): V0.2 auto‑start +
  long‑press STOP (default), or V0.1 hold‑to‑START / slide‑to‑STOP.
- **Power‑terminal connection check** — refuses to drive current until the DC
  terminal is confirmed connected ("CHARGER TERMINAL OPEN!" fault otherwise).
- **KION mode** (`CONFIG_KION_MODE`, and runtime‑toggleable) — listen‑only for a
  legacy Coulomb pack: the display never touches the charger, letting the
  Coulomb BMS drive it directly. See [CAN control](#runtime-can-control).
- **Charger temperature** shown in the on‑screen status line (polled per module).
- **Runtime CAN control** of baud and KION with ACKs (see below).
- **CAN OTA bootloader** (ABMS‑compatible) with a `factory + ota_0 + ota_1`
  partition scheme and automatic recovery.
- **Full CAN interface documented** in [`docs/ChargerLCD.dbc`](docs/ChargerLCD.dbc).

## Repository layout

```
include/app_config.h     ← single central configuration (edit this)
include/                 firmware headers (CAN, bootloader, UI)
src/                     firmware sources
  main.cpp               boot + main loop
  ba100_can.cpp          CAN driver, battery parsers, charging FSM
  can_bootloader.cpp     CAN OTA bootloader (factory partition)
  ui/                    LVGL UI
  sd_logger.cpp          SD-card raw CAN logging
scripts/config_from_header.py   derives USB-CDC flag from CONFIG_LCD_TYPE
docs/                    protocol references (DBC/DBF/PDF) + ChargerLCD.dbc
bin/                     prebuilt firmware images (full / update)
partitions.csv           factory + ota_0 + ota_1
platformio.ini           PlatformIO project (pioarduino platform)
CHANGELOG.md             release history
FEATURES.md              feature provenance + config reference
```

## Configuration

Everything is selected in [`include/app_config.h`](include/app_config.h):

| Option | Values | Meaning |
|---|---|---|
| `CONFIG_LCD_TYPE` | `0` / `1` | 4.3B / 7" board (sets CAN pins + USB‑CDC) |
| `CONFIG_NUM_CHARGERS` | `1`–`60` | parallel BA modules (current‑share auto if >1) |
| `CONFIG_CHARGE_CONTROL` | `0` / `1` | V0.1 START/STOP · V0.2 auto‑start + STOP‑only |
| `CONFIG_KION_MODE` | `0` / `1` | boot default for legacy‑listen‑only mode |
| `CONFIG_CAN_BAUD_KBPS` | `125`/`250`/`500` | default CAN baud (app **and** flasher) |
| `CONFIG_CHARGER_MAX_CURRENT_EACH_A` | float | per‑module current cap |
| `CONFIG_FW_VERSION` | string | version shown in the status watermark |

Charging tunables (detection voltage/current, ramp, foldback, terminal‑check
tolerances) are also in `app_config.h` with rationale inline.

## Build & flash

Built with [PlatformIO](https://platformio.org/) on the **pioarduino** Espressif
platform (Arduino‑ESP32 3.x / IDF 5.x), LVGL 8.3.x, ESP32_Display_Panel.

```bash
pio run                       # build
pio run -t upload             # build + flash over USB (first flash)
pio run -t clean && pio run   # REQUIRED after changing CONFIG_LCD_TYPE
```

> After changing `CONFIG_LCD_TYPE` you must `clean` first, so the Arduino core
> rebuilds with the correct `ARDUINO_USB_CDC_ON_BOOT` flag.

A partition‑table change requires a full re‑flash (per‑partition images at the
esptool offsets in `partitions.csv`), not an app‑only OTA.

## Runtime CAN control

Standard 11‑bit frames let a host reconfigure the unit; each is acknowledged on
`0x2CE` (`[setting, value, effect]`, effect `0`=applied now, `1`=next boot):

| Command | ID | Data | Effect |
|---|---|---|---|
| Change baud | `0x2CC` | `data[0]` = `01`(125) / `02`(250) / `05`(500) | applies now, persists |
| Toggle KION | `0x2CD` | `data[0]` = `00` off / `01` on | persists, **next boot** |

## CAN interface

The complete interface — every RX/TX message, byte layout, scaling and enum — is
in [`docs/ChargerLCD.dbc`](docs/ChargerLCD.dbc) (load it in any CAN tool or
`cantools`). Vendor protocol references are in [`docs/`](docs/).

## Version

**v2.1.0.** See [CHANGELOG.md](CHANGELOG.md) for full history. Lineage:
v1.0.0 (combined base) → v2.0.0 (+ terminal‑open check) → **v2.1.0** (+ KION mode,
charger temperature, config/runtime CAN baud, KION CAN toggle + ACKs, DBC).

---

© Coulomb Li Tech. Internal firmware — see repository access for licensing.
