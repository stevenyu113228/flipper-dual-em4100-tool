# Dual EM4100 Tool

A [Flipper Zero](https://flipperzero.one/) application to **read, save and emulate dual-frame (128-bit) EM4100 access cards** — the kind that standard tools only ever copy *half* of.

[![download](https://img.shields.io/badge/download-latest%20.fap-brightgreen)](../../releases/latest)
![category](https://img.shields.io/badge/category-RFID-orange)
![platform](https://img.shields.io/badge/platform-Flipper%20Zero-black)
![license](https://img.shields.io/badge/license-MIT-blue)

> **No build tools?** Just grab the prebuilt app from [**Releases**](../../releases/latest) and drop it in `apps/RFID/` on your SD card.

---

## Why this exists

Some 125 kHz access cards look like an ordinary EM4100 but don't behave like one.

A standard EM4100 tag broadcasts a single **64-bit frame** in a loop. Some cards — physically a T5577 chip configured with `MAXBLOCK=4` — instead broadcast **two 64-bit frames back-to-back (128 bits)**, looping forever:

```
[ frame1 (64-bit, valid EM4100) ][ frame2 (64-bit, second segment) ] [ frame1 ] [ frame2 ] ...
```

The second frame often carries a deliberately broken row-parity bit, so a **standard EM4100 decoder reads the first frame, validates it, and stops** — silently discarding the second half. The Flipper's built-in `Read` shows a perfectly normal ID, you clone it, and… the door stays shut, because the reader actually wants *both* frames.

This tool fixes that. It captures the raw LF waveform itself, decodes the **full 128 bits**, and emulates the complete signal.

> If your Flipper reads a card as plain EM4100 but emulation/cloning won't open the door — while a full Proxmark3 `lf t55 dump` clone does — this is very likely your situation.

## Features

- **Read** any EM4100 card and auto-detect whether it is single-frame (64-bit) or dual-frame (128-bit).
- **Full second-segment display** — see the complete `frame2` bytes, not just the ID.
- **Haptic + audio feedback** on a successful read.
- **Save / rename / delete** cards to SD card in a simple text format.
- **Add manually** — type in `frame1` + `frame2` by hand (hex keypad) without a physical card.
- **Emulate** the full 128-bit signal via the LF antenna. Single-frame cards are emulated as standard EM4100.
- **Write to a blank T5577** — clone the card onto a physical tag, then read it back to verify.

## Installation

### Option A — download a prebuilt `.fap` (easiest)

**👉 Grab the latest `dual_em4100_tool.fap` from [Releases](../../releases/latest)** — no build tools needed.

Copy it to your Flipper:

```
SD Card / apps / RFID / dual_em4100_tool.fap
```

(Use [qFlipper](https://flipperzero.one/update)'s File Manager, or copy directly to the microSD card.) Then open **Apps → RFID → Dual EM4100 Tool**.

### Option B — build it yourself

See [Building](#building) below.

> **Firmware compatibility:** built and tested against **Unleashed** firmware (API 87 / `unlshd-089`). It uses only public SDK symbols and should build on recent official, Unleashed, RogueMaster or Momentum firmware. If you see *"App Too New / Old"*, rebuild against the SDK matching your installed firmware.

## Usage

Open **Apps → RFID → Dual EM4100 Tool**.

### Read a card

1. Choose **Read card**.
2. Hold the card flat against the back of the Flipper.
3. On success the Flipper buzzes/vibrates and shows the ID and, for dual-frame cards, the full second segment.
4. Press **OK** to save (you'll be prompted for a name), or **Back** to discard.

### Manage saved cards

**Saved cards →** pick a card to open its detail menu:

- **Emulate** — broadcast the card over the LF antenna (press **Back** to stop).
- **Write to T5577** — clone the card onto a blank T5577 tag held against the
  back of the Flipper. The write is verified by reading it back.
- **Rename**
- **Delete**

> **Writing tip:** cloning a dual-frame (5-block) card from a Flipper depends on
> good antenna coupling. Hold the blank card flat and still against the back; if
> the write reports a mismatch, reposition and try again (a couple of retries is
> normal). For a guaranteed write, use a Proxmark3.

### Add a card manually

**Add manually →** enter 16 bytes with the hex keypad: the first 8 bytes are `frame1`, the next 8 are `frame2`. The app auto-detects single vs dual frame, then lets you save it.

## Saved file format

Cards are stored as plain text under `/ext/apps_data/dual_em4100/<name>.dem4100`:

```
Filetype: Dual EM4100
Version: 1
Frame1: FF8XXXXXXXXXXXXX
Frame2: FF8XXXXXXXXXXXXX
Segments: 2
```

`Frame1` / `Frame2` are the 64-bit **encoded** EM4100 frames (header + nibbles + parity), big-endian hex. `Segments` is `1` or `2`. For a single-frame card, `Frame2` mirrors `Frame1` and `Segments` is `1`.

## How it works

### Reading — custom 128-bit decoder

The built-in EM4100 decoder keeps only a 64-bit sliding window, so it cannot represent two frames at once. This app drives the capture hardware directly:

1. `furi_hal_rfid_tim_read_start()` energises the 125 kHz field; `furi_hal_rfid_tim_read_capture_start()` begins input capture.
2. The capture ISR fires **twice per carrier period** — once with the high-level duration (*pulse*), once with the full period. These are **paired**, then each period is split into a high half (`pulse`) and a low half (`period − pulse`) and fed into a Manchester decoder. *(Feeding the raw capture values straight in is wrong — the second value is the whole period, not the low half. This is the single most common mistake when writing an LF decoder from scratch.)*
3. A **128-bit sliding window** is maintained (`hi` = frame1 candidate, `lo` = frame2 candidate). A card is confirmed only when `hi` passes **full row + column parity** and the same result is seen several times in a row. If `lo` also carries a valid 9-bit header, the card is dual-frame; otherwise it's single.

### Emulating — exact waveform reconstruction

Emulation is a *digital* replay (rebuilt from the two 64-bit numbers), not a captured-waveform replay:

1. Expand the 128 bits into 256 Manchester half-cells.
2. Rotate to the first rising edge, then run-length-encode runs of equal level into PWM periods (`duration` = full period, `pulse` = high time). This avoids a subtle off-by-half-a-bit error at the loop seam.
3. Repeat the loop to fill a deep DMA buffer and stream it out with `furi_hal_rfid_tim_emulate_dma_start()` in circular mode.

The reconstruction is energy-exact: total high time and total period match the ideal signal to the clock.

## Building

This is a standard external FAP. Build it against a Flipper firmware source tree.

**With the firmware's `fbt`:**

```bash
# place this folder under applications_user/ in a firmware checkout
cp -r dual_em4100_tool <firmware>/applications_user/
cd <firmware>
./fbt fap_dual_em4100_tool
# output: build/<target>/.extapps/dual_em4100_tool.fap
```

**With [`ufbt`](https://github.com/flipperdevices/flipperzero-ufbt)** (standalone, recommended):

```bash
cd dual_em4100_tool
ufbt            # build
ufbt launch     # build, install and run on a connected Flipper
```

## Disclaimer

This project is for **interoperability, research, and working with access credentials you own or are authorised to test**. RFID cloning can have legal and security implications — only use it on cards and systems you have permission to. The authors accept no liability for misuse.

## License

[MIT](LICENSE) © 2026 steven
