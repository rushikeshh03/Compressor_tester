# Coulomb Harness Test Station — project rules (root)

This repo is a **three‑board ESP32 harness test station**. Read
[`README.md`](README.md) first for the architecture, wiring, CAN protocol, and
build steps. This file carries the rules that apply across all boards; each board
also has its own `CLAUDE.md` with board‑specific rules — **read the board's
`CLAUDE.md` before editing that board.**

| Board | Folder | What it is |
|---|---|---|
| LCD | `LCD/` | Waveshare ESP32‑S3 7" touch — operator UI + test controller (USB scanner, SD log, FAIL lock‑out). No CAN transceiver of its own. |
| CAN_Bridge | `CAN_Bridge/` | ESP32‑WROOM — transparent UART↔CAN bridge for the LCD. |
| IOT_Board | `IOT_Board/` | ESP32‑S3 — passive CAN→MQTT uploader + BLE‑HID to a tablet. |

## ⚠️ Documentation maintenance — READ THIS (applies to every AI/contributor)

The docs are the source of truth and **must not drift from the code**. Whenever a
change touches any of the following, **update the docs in the same change**, before
you consider the task done:

- **Hardware:** a pin, header, wire, transceiver, LED, voltage, or board →
  update the relevant table in [`README.md`](README.md) §3 **and** the board's
  `CLAUDE.md`/`README.md`.
- **CAN protocol / DBC:** an ID, signal, layout, or baud → update `README.md` §2,
  the `*/docs/harness_jig_v*` DBC + its README, **and both** the LCD and IOT sides
  that must stay in lock‑step.
- **Architecture / data flow:** how boards talk (UART bridge frame format, control
  frames, who owns the CAN baud) → `README.md` §1/§4 and both endpoints.
- **A user‑visible feature or behaviour:** → `README.md` §5 and the board's
  `CHANGELOG.md`.
- **Build/flash steps or toolchain quirks:** → `README.md` §6.

Rules of thumb:
1. **Every code change that a future reader would need to know about gets a
   `CHANGELOG.md` entry** in that board's folder (LCD/IOT already have one; create
   `CAN_Bridge/CHANGELOG.md` when you first change it).
2. **Settings that must match across boards** (CAN baud, the UART bridge frame
   format, partition scheme) must be changed on **both** sides and noted in both.
3. If you add a new board or a new top‑level concept, add it to `README.md` and this
   file's table.
4. Prefer updating existing docs over adding new files; keep this rule intact so the
   next agent sees it.

## Cross‑board lock‑step rules (do not let these diverge silently)

- **CAN baud** — `LCD/include/app_config.h` `CONFIG_CAN_BAUD_KBPS` **==**
  `CAN_Bridge/src/main.cpp` `CAN_BAUD_KBPS`. UART link baud likewise
  (`CONFIG_BRIDGE_UART_BAUD` == bridge `UART_BAUD`).
- **UART bridge frame format** (16‑byte `[0xAA][flags][id LE32][dlc][data×8][xor]`,
  flags bit0=ext / bit1=ctrl, ctrl `data[0]`=reboot) — identical on both ends
  (`LCD/src/ba100_can.cpp` and `CAN_Bridge/src/main.cpp`). **Flash both together**
  when it changes.
- **Partition table + CAN‑flash scheme** — intentionally identical between LCD and
  IOT (`partitions.csv`, factory+ota_0+ota_1). If you touch it on one, mirror it on
  the other (see each board's `CLAUDE.md`).

## Build (from PowerShell)

`& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -d <LCD|CAN_Bridge|IOT_Board>`
— see `README.md` §6 for the toolchain gremlins (LCD framework reinstall, etc.).
