# CAN_Bridge — UART ↔ CAN bridge for the Harness Tester LCD

> Part of the 3‑board station — see the root [`../README.md`](../README.md) and
> [`../CLAUDE.md`](../CLAUDE.md). **Keep docs updated** when you change pins,
> wiring, the frame format, or the CAN baud (root `CLAUDE.md` → *Documentation
> maintenance* + *Cross‑board lock‑step*). Add a `CHANGELOG.md` here on first change.

The Waveshare LCD board has no usable CAN transceiver (its native-USB pins are
taken by the USB-HID scanner, and its other free pins can't host a CAN
transceiver cleanly). So CAN is offloaded to this little **ESP32-WROOM** board,
which has **native TWAI + a CAN transceiver on GPIO4/5**. It's a transparent
pipe: CAN frames in → UART to the LCD; UART frames from the LCD → CAN. All the
DAS protocol decoding stays on the LCD (`LCD/src/ba100_can.cpp`).

Both boards are 3.3 V, so the UART link needs **no level shifting**.

## Wiring

```
   LCD (ESP32-S3)                 Bridge (ESP32-WROOM)              CAN bus
   ─────────────                  ────────────────────             ───────
   GPIO43 (TX) ───────────────────→ GPIO16 (RX, Serial2)
   GPIO44 (RX) ←─────────────────── GPIO17 (TX, Serial2)
   GND ──────────────────────────── GND
                                     GPIO5 (CAN_TX) → transceiver TXD
                                     GPIO4 (CAN_RX) ← transceiver RXD
                                     transceiver CANH/CANL ───────→ CANH/CANL
```

### Status LEDs

| Pin | LED | Behaviour |
|---|---|---|
| GPIO2  | onboard  | flickers on **CAN** bus activity (TX or RX) |
| GPIO25 | external | **solid while the LCD link is up** ("ESP32 connected"); off when stale |
| GPIO26 | external | **~1 Hz heartbeat** — the bridge firmware is alive |

Add an external LED + ~330 Ω to GND on GPIO25/GPIO26 (GPIO2 is the onboard LED). The pins are `#define`s at the top of `src/main.cpp`. The link LED is driven by a **~2 Hz link heartbeat** the LCD sends over UART (a bridge-local control frame that is never placed on the CAN bus), so it reads "connected" even when the bus is idle.

- On the LCD, set the **UART1/2 DIP switch to UART2** so P1 breaks out GPIO43/44.
- Put **120 Ω termination at each end** of the CAN bus.
- Bridge USB‑serial (`Serial` @115200) is a **CAN debug console** — it prints every
  relayed frame. Set `DBG_VERBOSE 0` in `src/main.cpp` if the bus is busy enough
  that per-frame printing adds latency.

## Must match on both sides

| Setting | Bridge (`src/main.cpp`) | LCD (`LCD/include/app_config.h`) |
|---|---|---|
| UART baud | `UART_BAUD` = 1000000 | `CONFIG_BRIDGE_UART_BAUD` = 1000000 |
| CAN baud | `CAN_BAUD_KBPS` = 500 | `CONFIG_CAN_BAUD_KBPS` = 500 |
| Wire frame | 16-byte `[0xAA][flags][id LE32][dlc][data×8][xor]` | same |

Change the CAN baud in **both** places if you retune the bus.

## Wire frame (16 bytes, fixed)

| Byte | Meaning |
|---|---|
| 0 | `0xAA` sync |
| 1 | flags: bit0 = extended ID; bit1 = control/link-heartbeat (LCD→bridge only, not relayed to CAN) |
| 2–5 | identifier, little-endian uint32 |
| 6 | DLC (0–8) |
| 7–14 | data (8 bytes, unused = 0) |
| 15 | XOR checksum of bytes 1–14 |

The receiver resyncs on `0xAA` and drops any frame whose checksum fails.

## Build / flash (PlatformIO, from PowerShell)

```
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -d CAN_Bridge
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -d CAN_Bridge -t upload
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" device monitor -b 115200
```

Board `esp32dev`, no external libraries (TWAI ships with the Arduino core).

## Notes / limits

- **Latency:** a frame is 16 UART bytes ≈ 0.16 ms at 1 Mbps + one CAN frame time
  — negligible for the harness test (cycle is seconds).
- **CAN-flash** of the LCD is *not* relayed through the bridge — flash the LCD
  over USB. The bridge is data-plane only.
- The bridge does no filtering — it forwards *all* bus traffic to the LCD, which
  ignores IDs it doesn't recognise.
