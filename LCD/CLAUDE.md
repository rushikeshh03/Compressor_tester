# Coulomb Harness Tester — project rules

> Part of the 3‑board station — see the root [`../README.md`](../README.md) and
> [`../CLAUDE.md`](../CLAUDE.md). **Keep docs updated** when you change pins,
> wiring, the CAN protocol, or a feature (root `CLAUDE.md` → *Documentation
> maintenance*).

Firmware for a barcode/harness **test station** on the Waveshare
ESP32-S3-Touch-LCD-7 (800x480 ST7262 RGB + GT911 touch). Forked from
`ESP32LCD_Charger`; the entire charger/BMS protocol and the 4.3" board option
were removed. Goal: read the **one known BLE barcode scanner** as a BLE central /
HID host (`src/ble_hid_host.cpp` — owns the radio: `BLEDevice::init` + wipe bonds
+ security; hard-connects to `CONFIG_SCANNER_ADDR`), and read **CAN** from a
device-under-test to drive a PASS/FAIL check + logging. The **tablet is NOT on the
LCD** (it pairs to the IoT board); the LCD reads the IoT's **0x7A0 CAN heartbeat**
to show tablet/MQTT/CAN status + a blinking "IoT alive" chip. Scanner is the only
BLE role. App WiFi was removed entirely.

The firmware lives in the **`LCD/`** subfolder; build from there, and run
`pio run` from **PowerShell** (Git-Bash/MSys breaks a pending esptool reinstall).

Hardware/feature selection lives in **`include/app_config.h`** — never hand-edit
platformio.ini for hardware switching.

## Hardware is fixed (7" only)

There is no LCD-type switch anymore. CAN uses GPIO20/19 (the native-USB pins)
muxed to the CAN transceiver by the FSUSB42 (CH422G USB_SEL=IO5 driven HIGH in
`main.cpp`), so **USB-CDC must stay off** — pinned via
`-DARDUINO_USB_CDC_ON_BOOT=0` in `platformio.ini`. Logs come from UART0
(GPIO43/44) / the SD card. Do not re-enable USB-CDC.

**Log every change in `CHANGELOG.md`** (the LCD is always `7"` now).

## UI (`src/ui/ui_fallback.cpp` + `ui.h`)

Main page: LEFT = Coulomb logo (top) + a slot below it that, **while idle**, is a
**"Restart system"** button (tap → restart the IoT over CAN via `0x2D0`, then
`esp_restart()` the LCD — both reboot together) and, **while a check is running**,
shows "Checking…" instead (the button is hidden mid-scan). Checks auto-trigger on
each barcode scan — there is no Check button anymore. RIGHT = scanned **barcode**
(fills the whole card, no caption, font auto-sized to length) on top + a big
**PASS/FAIL** indicator. Top-bar status chips; page-2 title bar has the
`version | partition | baud` watermark. UI hooks: `ui_set_barcode()`,
`ui_set_result()`, `ui_set_checking()`, `ui_take_restart_system_request()`.

## CAN layer (`src/ba100_can.cpp` + `ba100_can.h`)

A **lean** TWAI wrapper (no charger protocol). It provides TWAI bring-up at the
NVS baud, RX polling with a link-liveness flag (the green dot), the `0x2CC`
runtime baud-change, the `0x0D` enter-bootloader hand-off, an uptime timestamp
for the SD logger, and bootloader-active gating. The `ba100_*` names and
`BA100_State_t` are kept only to preserve the include contract with the
bootloader / SD logger / UI. Harness message parsing goes here.

## ⚠ CAN flashing / partitions are kept in lock-step with the IOT code

The partition table (`partitions.csv`: `factory` + `ota_0` + `ota_1`) and the
CAN-flash flow are intentionally **identical to the IOT code**: the app runs from
`app0`; the `0x0D` enter trigger reboots into the `factory` partition, which
backs up `app0→app1` **only when entry was from app0** (an `RTC_NOINIT_ATTR`
sentinel recorded before the reboot — booting from app1 must not clobber the good
backup), then runs the CAN flash into `app0`. The sentinel MUST be
`RTC_NOINIT_ATTR` (not `RTC_DATA_ATTR`, whose `.rtc.bss` is zeroed by the SW
reset our enter path uses). Factory only *stays* to flash on a **deliberate
enter** (a second `RTC_NOINIT_ATTR` flag, `s_flash_entry_requested`); any other
factory boot recovers straight to app0/app1. No EC200 modem here, so `factory`
only ever does CAN-slave flashing (a factory-mode WiFi web updater is the one
alternative — see `wifi_service.cpp`).

**RULE: if you change ANYTHING in this CAN-flashing / partition scheme
(partitions, backup, enter trigger `0x0D`, the `0x0B` flash protocol, or the
factory/app split), you MUST remind the user to make the same change in the
IOT code.** Do not let the two diverge silently.

A partition-table change means devices must be **fully re-flashed**, not
app-only OTA'd. There is no `upload_full` target; write the per-partition images
by hand at the esptool offsets in `partitions.csv`.

## Build

- `~/.platformio/penv/Scripts/pio.exe run` (pio is not on PATH).
- CAN/OTA protocol references are in `docs/`.
