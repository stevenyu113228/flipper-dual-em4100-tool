# Changelog

All notable changes to this project are documented here.

## [2.1] - 2026-06-17

Added writing to physical blank T5577 cards, plus robustness fixes.

### Added
- **Write to T5577**: clone the card to a blank T5577 tag (MAXBLOCK=4 for
  dual-frame, MAXBLOCK=2 for single-frame), then read it back to verify.
- Card detail screen is now a vertical menu: Emulate / Write to T5577 /
  Rename / Delete.

### Fixed
- Settle the RF field (`furi_delay_ms(5)`) before `t5577_write()`, matching the
  firmware's built-in writer.
- Timers moved from file-static globals into the app struct (no dangling context
  across app restarts); ordered teardown of timers and worker threads.
- A write in progress can no longer be interrupted by Back (avoids a UI stall
  while the blocking write runs); rename keeps the loaded path in sync; Save can
  be cancelled without losing the just-read card.
- Memory barrier before publishing read results to the UI timer.

### Note
- Writing multi-block (dual-frame) cards from a Flipper is sensitive to antenna
  coupling — hold the blank card flat and still; a retry or two may be needed.
  This is a Flipper LF hardware limitation (the built-in Electra writer has the
  same trait). For a guaranteed write, use a Proxmark3.

## [2.0] - 2026-06-16

Rebuilt on ViewDispatcher with the stock GUI modules and added card management.

### Added
- Read any EM4100 card with a custom 128-bit decoder; auto-detects single vs dual frame.
- Full second-segment (`frame2`) display on the read-result screen.
- Haptic + audio feedback on a successful read.
- Save cards with a custom name (text keypad).
- Card detail screen with Emulate / Rename / Delete.
- Add cards manually via a hex keypad (`frame1` + `frame2`).

### Fixed
- Capture pairing bug: the capture ISR reports the high-level duration and the
  full period separately. Each period is now split into a high half and a low
  half (`period − pulse`) before being fed to the Manchester decoder. The first
  read implementation fed the raw values directly and never detected a card.

## [1.0] - 2026-06-14

Initial emulation-only release (hardcoded card), proving the concept.

### Added
- Emulate a full 128-bit dual-frame EM4100 signal via circular DMA.
- Waveform built by rotate + run-length encoding (energy-exact), avoiding an
  off-by-half-a-bit error at the loop seam that a `pulse_glue` approach hit.
