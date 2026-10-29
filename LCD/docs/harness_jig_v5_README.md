# Harness Test Jig — CAN DBC Reference

This describes `harness_jig__5_.dbc`, the CAN interface for the wire-harness
test jig. Two nodes: `Tester` (the PC/CAN tool) and `Jig` (the board).
Standard 11-bit IDs throughout.

## Message list

| ID (hex) | Name | Sender | DLC | Purpose |
|---|---|---|---|---|
| 0x600 | `Jig_BarcodeFrame1` | Tester → Jig | 8 | Barcode bytes 0–7 |
| 0x601 | `Jig_BarcodeFrame2` | Tester → Jig | 8 | Barcode bytes 8–15 |
| 0x602 | `Jig_BarcodeFrame3` | Tester → Jig | 4 | Barcode bytes 16–19 (optional) |
| 0x710 | `Jig_ResultStatusBarcode` | Jig → Tester | 8 | PASS/FAIL + barcode echo (3 frames) |
| 0x711 | `Jig_ResultCells` | Jig → Tester | 8 | 8 averaged cell voltages (3 frames) |
| 0x712 | `Jig_ResultTemps` | Jig → Tester | 8 | 4 averaged temp readings (2 frames) |
| 0x713 | `Jig_HarnessFault` | Jig → Tester | 8 | Fault localization detail (1 frame) |
| 0x720 | `Jig_Heartbeat` | Jig → Tester | 8 | Board-alive heartbeat, ~500 ms |

**A "test cycle" on the bus looks like:** Tester sends 0x600 (+0x601, +0x602
if the barcode is 20 bytes) → after a few seconds the Jig sends one burst of
0x710/0x711/0x712/0x713, usually all at the same timestamp.

---

## 0x600 / 0x601 / 0x602 — Barcode input

Tester sends the scanned barcode split across up to 3 frames:
- `Jig_BarcodeFrame1` (0x600): barcode chars 0–7, one byte each (`Char0`…`Char7`)
- `Jig_BarcodeFrame2` (0x601): barcode chars 8–15 (`Char8`…`Char15`)
- `Jig_BarcodeFrame3` (0x602): barcode chars 16–19 (`Char16`…`Char19`), **only sent for 20-byte barcodes**

Each byte is a raw ASCII character code — decode as `chr(byte)`, stopping at
the first `0x00` (barcodes shorter than the frame are zero-padded).

**Important:** if frame 3 doesn't arrive within ~20 ms of frame 2, the jig
assumes a 16-byte barcode and proceeds without it. So don't wait indefinitely
for 0x602 — a barcode is "complete" once either 0x602 arrives, or ~20 ms has
passed since 0x601 with nothing else pending.

---

## 0x710 — `Jig_ResultStatusBarcode` (multiplexed, Seq 0–2)

Byte 0 is a multiplexor `Seq` (0, 1, or 2). All three sub-frames normally
arrive back-to-back with the same timestamp.

| Seq | Bytes 1–7 contain |
|---|---|
| 0 | `Status` (byte 1) + barcode chars 0–5 |
| 1 | barcode chars 6–12 |
| 2 | barcode chars 13–19 |

`Status`: **0 = FAIL, 1 = PASS** (see `VAL_TABLE_`/`VAL_` in the DBC — most
DBC tools will auto-show this as an enum).

This is an *echo* of the barcode the jig received, for the tester to confirm
against what it sent — reassemble it the same way as the RX barcode frames.

---

## 0x711 — `Jig_ResultCells` (multiplexed, Seq 0–2)

8 averaged cell voltages, `uint16` little-endian, units **mV**, 3 cells per
frame except the last (2 cells + 2 zero-padded bytes).

| Seq | Cells carried |
|---|---|
| 0 | Cell0_mV, Cell1_mV, Cell2_mV |
| 1 | Cell3_mV, Cell4_mV, Cell5_mV |
| 2 | Cell6_mV, Cell7_mV |

## 0x712 — `Jig_ResultTemps` (multiplexed, Seq 0–1)

4 averaged temperature readings, `int16` little-endian (**signed** — cells
above can go negative in theory, temps definitely can), units **tenths of a
degree C** (divide by 10 for °C).

| Seq | Temps carried |
|---|---|
| 0 | Temp0_C10, Temp1_C10, Temp2_C10 |
| 1 | Temp3_C10 |

---

## 0x713 — `Jig_HarnessFault` — fault localization detail

**Not multiplexed** — a single flat 8-byte frame, sent once per test cycle
alongside 0x710/0x711/0x712.

| Byte | Signal | Range | Meaning |
|---|---|---|---|
| 0 | `CellFaultMask` | 0–255 | bit *n* set → cell *n*'s **averaged** reading is ≤0V or ≥4V. All 8 bits meaningful. |
| 1 | `TempFaultMask` | 0–15 | bit *n* (0–3) set → temp sensor *n*'s **averaged** reading is an outlier vs. the other 3 (leave-one-out comparison). Bits 4–7 always 0. |
| 2 | *(reserved)* | — | Always 0x00. (An earlier revision put a VBLK-mismatch flag here; that check was removed and no longer reports over CAN — ignore this byte.) |
| 3 | `InitFailed` | 0–1 | 1 = the DAS never finished initializing this cycle. **See note below.** |
| 4 | `CellRawFaultMask` | 0–255 | bit *n* set → cell *n* hit the OV limit on **at least one of the 10 raw samples**, even if the average (byte 0) came back clean. |
| 5 | `TempRepeatabilityFaultMask` | 0–15 | bit *n* (0–3) set → temp sensor *n*'s own reading varied more than tolerance **across the 10 raw samples** (instability, not an offset vs. the other sensors — different from byte 1). |
| 6 | `PassingReadingCount` | 0–10 | How many of the 10 raw samples individually passed. If `Status=FAIL` (0x710) and this is below 8, that alone explains the FAIL — even if every mask byte above is 0x00. |
| 7 | *(reserved)* | — | Always 0x00. |

### ⚠️ Why `Status` can be FAIL with everything in this frame reading clean

`Status` (in 0x710) reflects a decision made from the **raw**, per-sample
data across all 10 readings the jig takes internally. Bytes 0/1 of this
frame only ever describe the **averaged** result. Averaging can hide a
transient spike or a jittery sensor that still failed on the raw data — so
don't treat "`Status=FAIL` but `CellFaultMask`/`TempFaultMask` are both 0" as
a bug. **Always check bytes 4–6 in that case**:
- `CellRawFaultMask != 0` → some cell spiked in at least one sample
- `TempRepeatabilityFaultMask != 0` → some sensor was unstable run-to-run
- `PassingReadingCount < 8` → too many individual samples failed outright

If bytes 0/1/4/5 are all 0 and `PassingReadingCount == 10` but `Status` is
still FAIL, that combination shouldn't happen under current firmware — flag
it as a bug if you ever see it.

### `InitFailed` sentinel — don't double-report as a cell fault

When `InitFailed=1`, `CellFaultMask` will read `0xFF` (every cell "faulted")
purely because no real reading was ever taken — all cell data is zero by
construction, not because of 8 simultaneously bad cells. **UI should check
`InitFailed` first**: if set, show "DAS did not initialize" and don't also
render a per-cell fault breakdown from byte 0/4 — they're not meaningful in
this state. `TempFaultMask`/`TempRepeatabilityFaultMask` will read 0 in this
case (trivially, since all-zero temps "agree" with each other).

---

## 0x720 — `Jig_Heartbeat`

Sent by the jig every ~500 ms regardless of test activity — useful for a UI
"board connected" indicator independent of test cycles.

| Byte(s) | Signal | Notes |
|---|---|---|
| 0–1 | `Marker_H`, `Marker_B` | Always ASCII `'H'` (0x48) and `'B'` (0x42) — sanity check that you're decoding the right frame. |
| 2–3 | `Counter` | `uint16` LE, increments every heartbeat, wraps at 65535. |
| 4 | `CAN_TEC` | CAN1 Transmit Error Counter (0–255). Should sit at 0 in normal operation. |
| 5 | `CAN_REC` | CAN1 Receive Error Counter (0–127 meaningful). Should sit at 0. |
| 6 | `CAN_LEC` | CAN1 Last Error Code (0–7). `0` = no error. **`7` specifically means "no change since last read" and is also the power-on default** — seeing `CAN_LEC=7` right after `Counter` has reset to 0 is the signature of a board reboot, not an active bus fault. Values 1–6 are real protocol errors (stuff/form/ack/bit/CRC — see the `CM_ SG_` comment on this signal in the DBC for the full table). |
| 7 | `TxQueueDepth` | Jig's outbound CAN queue depth (0–50-ish). Should hover near 0; a value climbing over several heartbeats indicates the jig is producing frames faster than the bus/tester can drain them. |

If `Counter` resets to 0 mid-log, treat it as the board having reset —
correlate with a `CAN_LEC=7` on the same frame to confirm.

---

## General notes for the UI implementation

- **Multiplexed messages (0x710, 0x711, 0x712) can arrive out of order or
  interleaved** with other IDs in a burst — buffer by `Seq` and only treat
  a group as "complete" once all expected `Seq` values for that burst have
  been seen. In practice the jig sends them back-to-back with identical
  timestamps, but don't rely on strict ordering at the transport layer.
- **A "result" for one test cycle isn't a single CAN ID** — it's the
  combination of one 0x710 (3 frames), one 0x711 (3 frames), one 0x712
  (2 frames), and one 0x713 (1 frame), all sent within the same short
  window. Group by timestamp proximity (they're normally simultaneous) to
  reconstruct one complete test result.
- **Cell/temp values in 0x711/0x712 are averages**, not the last raw
  reading — see the `Jig_HarnessFault` notes above for why that matters
  when interpreting a FAIL.
- Any CAN ID **not** in this DBC (e.g. `0x7a0`, seen in some captures) is
  from a different device on the bus and isn't part of the jig protocol —
  don't attempt to decode it against this file.
- Standard `cantools`-compatible DBC — any standard CAN tooling (Vector
  CANoe, PCAN-View + DBC import, `cantools`/`python-can` in Python, etc.)
  should load and decode this file directly. `VAL_TABLE_`/`VAL_` gives
  `Status` in 0x710 as an enum (`PASS`/`FAIL`) automatically in tools that
  support it.
