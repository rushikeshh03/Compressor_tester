# Harness / HVAC Test Station — Wiring Guide

Three ESP32 boards on **one CAN bus (500 kbps)**, plus the compressor, charger,
flow meter, an MCP3208 ADC (NTC temps), a USB‑HID barcode scanner, and (on the
IoT board) WiFi/MQTT + a BLE tablet.

```
                          CAN bus (500 kbps, 120Ω at each end)
   ┌────────────┬───────────────┬───────────────┬───────────────┬────────────┐
 CANH/CANL   Compressor       Charger        CAN_Bridge        IOT_Board
                (ECU)        (XNYB/ACDC)     (ESP32-WROOM)     (ESP32-S3)
                                              │  ▲ UART 1Mbps      │ WiFi/MQTT + BLE
                                              ▼  │                 │
                                            LCD (ESP32-S3 7") ─ USB ─ Barcode scanner
```

The LCD has **no CAN transceiver** — it reaches the bus over a UART link to the
CAN_Bridge. The bridge does all physical CAN. The IoT board is an independent
passive listener with its own transceiver.

---

## 1. CAN bus (shared, 500 kbps)

Every node's transceiver ties to the same differential pair:

| Signal | Runs to |
|---|---|
| **CANH** | Compressor CANH · Charger CANH · Bridge transceiver CANH · IoT transceiver CANH |
| **CANL** | Compressor CANL · Charger CANL · Bridge transceiver CANL · IoT transceiver CANL |
| **GND**  | common ground between all nodes |

- **120 Ω termination at the two physical ends** of the bus only.
- All nodes must run **500 kbps** (compressor is 500 k; strap the charger to 500 k too).

---

## 2. CAN_Bridge — ESP32‑WROOM "Compressor" board

Owns the CAN transceiver (TJA1050), the flow meter, and the MCP3208. Talks to
the LCD over UART.

### 2.1 CAN transceiver (TJA1050)
| Bridge GPIO | Transceiver |
|---|---|
| GPIO17 (CAN_TX) | TXD |
| GPIO16 (CAN_RX) | RXD |
| 3.3 V / 5 V per transceiver | VCC |
| GND | GND |
| — | CANH / CANL → bus |

### 2.2 UART link to the LCD (1 Mbps, 3.3 V — no level shift)
| Bridge GPIO | LCD |
|---|---|
| GPIO22 (RX) | ← LCD GPIO43 (TX) |
| GPIO23 (TX) | → LCD GPIO44 (RX) |
| GND | GND |

### 2.3 Flow meter (YF‑B2)
| Bridge | Flow meter |
|---|---|
| **GPIO35** (input‑only) | signal (pulse) |
| — | **1 kΩ pull‑up from signal to 3.3 V** (required — GPIO35 has no internal pull‑up) |
| 5 V (VIN) | meter V+ |
| GND | meter GND |

> The YF‑B2 output is open‑collector; pulled to **3.3 V** it's safe for the ESP32.
> Broadcast as CAN **`0x7B0`** at 1 Hz (Hz, L/min×10, total L×10).

### 2.4 MCP3208 ADC (8× NTC), SPI @ 1 MHz — **run at 3.3 V**
| Bridge GPIO | MCP3208 pin |
|---|---|
| GPIO18 | CLK (pin 13) |
| GPIO19 | DOUT (pin 12) |
| GPIO13 | DIN (pin 11) |
| GPIO27 | CS/SHDN (pin 10) |
| **3.3 V** | VDD (pin 16) **and** VREF (pin 15) |
| GND | AGND (pin 14) **and** DGND (pin 9) |

> ⚠ Use **3.3 V**, not 5 V — 5 V on DOUT would exceed the ESP32's 3.3 V input.
> VDD = VREF = 3.3 V makes the reading ratiometric (supply drift cancels).
> Temps broadcast as CAN **`0x7B1`** (NTC 1–4) + **`0x7B2`** (NTC 5–8), int16 ×0.1 °C.

### 2.5 NTC channels (10k NTC + 10k divider) — one per MCP3208 channel CH0…CH7
```
3.3V ──[ 10k NTC ]──┬──[ 10k fixed ]── GND
                    └── MCP3208 CHx
```
- 8 single‑ended channels → **up to 8 NTCs** (CH0…CH7).
- Default firmware assumes this orientation (`NTC_ON_TOP = 1`). If you instead wire
  NTC→GND with the 10k as a pull‑up to 3.3 V, set `NTC_ON_TOP = 0`.
- Set `NTC_BETA` in `CAN_Bridge/src/main.cpp` to your NTC's Beta (3435 / 3950 / 3977…).

### 2.6 Status LEDs (optional, active‑high)
| Bridge GPIO | LED |
|---|---|
| GPIO2 (onboard) | flickers on CAN activity |
| GPIO25 (+ ~330 Ω to GND) | solid = LCD link up |
| GPIO26 (+ ~330 Ω to GND) | ~1 Hz = firmware alive |

USB (UART0) = 115200 CAN debug console.

---

## 3. LCD — Waveshare ESP32‑S3‑Touch‑LCD‑7

Operator UI. No CAN — reaches the bus via the bridge over UART.

| Signal | Pin(s) | Notes |
|---|---|---|
| **UART to bridge** | GPIO43 (TX) → bridge GPIO22 · GPIO44 (RX) ← bridge GPIO23 | **Set the on‑board UART1/2 DIP switch to UART2** so header P1 carries these. 3.3 V, no level shift. |
| **USB‑HID scanner** | GPIO19 (D‑), GPIO20 (D+) → USB‑C port | `USB_SEL` (CH422G IO5) held **LOW** routes native USB to the USB‑C port. The port must source **5 V VBUS** to the scanner. |
| micro‑SD | GPIO11/12/13, CS = CH422G EXIO4 | on‑board slot |
| Touch / expander I²C | GPIO8 (SDA), GPIO9 (SCL) | GT911 + CH422G, on‑board |
| Sensor‑AD header | GPIO6 | idle output (unused now) |
| RGB panel | HSYNC46 / VSYNC3 / DE5 / PCLK7 + 16 data | on‑board |

- **No USB‑serial console** (GPIO43/44 are UART0, now the bridge link) — LCD logs go to SD.
- Flash the LCD over its **USB‑C**.

---

## 4. IOT_Board — ESP32‑S3‑WROOM‑1 N8R2

Independent passive CAN listener → SD log + HTTP/MQTT upload; BLE‑HID to a tablet.

| Signal | Pin | Notes |
|---|---|---|
| CAN | GPIO4 (TX), GPIO5 (RX) | to **its own** CAN transceiver → CANH/CANL bus |
| USB (native) | GPIO19/20 | CDC log console |
| micro‑SD | on‑board | per‑barcode raw‑CAN CSV |

WiFi (HTTP + MQTT) and BLE are on‑chip — no extra wiring. Powered per the board.

---

## 5. Compressor & Charger (devices under test)

| Device | Bus | Notes |
|---|---|---|
| Compressor | CANH/CANL, 500 kbps | Auto‑detected: Variant A (`0x238`/`0x248`) or Variant B (`0x34C`/`0x349`). Power per its own spec (HV/48 V). |
| Charger | CANH/CANL, 500 kbps | Auto‑detected: XNYB (LEVDC) or NIUERA ACDC. Strap to **500 kbps**. Power per its own spec. |

The LCD sends the scanned barcode (`0x600/601/602`) and the compressor/charger
commands **through the bridge**; the bridge relays them to CAN and forwards all
bus traffic back to the LCD.

---

## 6. CAN ID map (station‑generated frames)

| ID | From | Meaning |
|---|---|---|
| `0x600/601/602` | LCD → bus | scanned barcode (24 ASCII) — session marker |
| `0x238` / `0x34C` | LCD → compressor | command (rpm / power / on‑off) |
| `0x248` / `0x349` | compressor → LCD | status |
| `0x200…` / ext | LCD ↔ charger | XNYB / ACDC command + status |
| `0x7A0` | IoT → bus | IoT heartbeat (tablet/MQTT/CAN flags) |
| `0x7B0` | bridge → bus | flow: Hz, L/min×10, total L×10 |
| `0x7B1` | bridge → bus | NTC temps ch0–3 (int16 ×0.1 °C, `0x8000` = open) |
| `0x7B2` | bridge → bus | NTC temps ch4–7 |
| `0x2D0` | LCD → IoT | restart IoT (`'R''S'`) |

---

## 7. Power summary

| Rail | Feeds |
|---|---|
| 5 V | LCD USB‑C VBUS → scanner; flow meter V+; bridge/IoT boards (VIN) |
| 3.3 V (bridge) | MCP3208 VDD+VREF, NTC dividers, flow‑meter signal pull‑up |
| GND | common across **all** boards, transceivers, sensors, and the CAN bus |

**One common ground across the entire station is mandatory** — the CAN bus,
the UART link, and every sensor reference it.
