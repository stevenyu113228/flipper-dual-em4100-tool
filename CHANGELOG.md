# Changelog

All notable changes to this project are documented here.

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
