# Diary

## 2026-03-19

### Jitter Root-Cause Investigation

#### Hypothesis Tested: DMX Signal Noise
- Deployed a minimal diagnostic script (`firmware/dmx_diag.py`) that reads raw DMX frames with no motor code running.
- Captured 594 frames over 20 seconds while the lighting desk held static positions.
- **Result: DMX signal is perfectly clean.** Only 4 u16 value changes across the entire capture, all intentional desk cue transitions. Zero noise during hold positions.
- The existing `DMX_FRAME_IMMEDIATE_DELTA_LIMIT` and `DMX_FRAME_CONFIRM_COUNT` config values (previously unused) were wired into `dmx_worker()` as a precaution, but **DMX noise is not the cause of jitter**.

#### Firmware Logic Trace
Traced the full path from DMX input to motor steps:
1. `dmx_worker()` → `SharedDMXState.update_from_channels()` → `snapshot()` → `apply_snapshot()` → `update()` → `move_fixed_steps_blocking()`
2. The position controller hold logic is correct: when `distance == 0` and `speed < 1.0`, `update()` zeroes speed and accumulator and returns 0. No false steps are generated from the controller when the motor is truly at its tracked target.
3. The `map_u16_to_steps_with_margin()` mapping is deterministic — same u16 always produces the same step target.

#### Root Cause Found: PIO Step Overshoot in `move_fixed_steps_blocking()`
The PIO step generator (`step_pulse_pio`) is a free-running loop that produces pulses continuously once activated. `move_fixed_steps_blocking()` polls the hardware counter every `poll_ms=1` and calls `stop()` when `moved >= steps`. The bug:
- Between the last counter read and `stop()`, the PIO continues emitting step pulses. These are **real physical motor steps** that move the shaft.
- The function returns the *requested* step count, not the actual pulses emitted.
- `current_position_steps` therefore tracks the *intended* position, while the motor's physical position drifts ahead by the uncounted overshoot.
- Over many small chunk moves (the runtime uses chunks of up to 64 steps), the overshoot accumulates.
- The controller thinks it's at position X, but the motor is physically at X+N.
- This explains **position drift between cycles** (same DMX values → different physical positions).
- This also explains **jitter during holds**: after a move sequence with accumulated overshoot, the controller's tracked position and the physical position are misaligned, so subsequent target updates cause unnecessary correction moves.

At high speeds (e.g. 50000 Hz = 0.02 ms/step), the PIO can emit dozens of extra steps between the last poll and `stop()`. Even at the minimum chunk speed of 500 Hz, the overshoot window exists.

#### Changes Made So Far
- Added `RUNTIME_POSITION_DEADBAND_STEPS = 2` to `firmware/config.py` — suppresses target updates within 2 steps when motor is idle.
- Added DMX frame confirmation filtering in `dmx_worker()` using existing config constants — harmless but not the fix.
- Added position deadband in `apply_snapshot()` — only suppresses target chasing when idle, does not fix root cause.
- Created `firmware/dmx_diag.py` — standalone diagnostic for observing raw DMX values.

#### Fix Iterations

**Attempt 1 — Return actual counter after stop:**
Changed `move_fixed_steps_blocking()` to read the counter after `stop()` and return actual count. Result: drift improved (mean drift ±2px vs ±25px before), but oscillation continued (24-39px). Cause: tracking overshoot made the controller reverse direction, creating back-and-forth oscillation.

**Attempt 2 — Ghost edge discovery:**
Debug logging revealed `total_steps` increasing by 100-200 while controller reported zero movement. The counter SM was continuously running and counting false rising edges on the step pin during idle. Added counter SM gating (only active during moves) and returned requested count. Reduced ghost events but the fundamental jitter problem remained.

**Attempt 3 — Landing zone (slow down near target):**
When within a few steps of target, reduce PIO speed to 300 Hz to minimize overshoot window. Failed: at 30kHz with 64-step chunks, the entire chunk completes in ~2ms (only 2 polls), so the landing zone threshold is never caught in time.

**Attempt 4 — Count-limited PIO (final fix):**
Rewrote the PIO step generator with a new `step_count_pio` program that accepts a step count and delay value, generates exactly that many pulses, then idles. Zero overshoot by hardware design. **BUT:** still jittery, so the problem is very likely **somewhere else!!**

Key changes to `firmware/pio_stepper.py`:
- New `step_count_pio` PIO program: `pull(block)` for count, `pull(block)` for delay, generate exactly N pulses, wrap back to `pull(block)` to idle.
- Kept original `step_freerun_pio` for homing/`run_until` (free-running mode with counter polling).
- `move_fixed_steps_blocking()` now uses count-limited PIO: pushes count-1 and delay to FIFO, sleeps for calculated completion time, returns exact step count.
- SM programs are swapped between counted and freerun modes as needed.
- Added `Pin.PULL_DOWN` on step pin to prevent ghost edges during idle.
- `run_until()` and homing still use free-running PIO with counter SM, returning actual count via `_stop_and_read()`.

#### Results

70-second X capture comparison:
- **Before fix:** 1008 X changes, 25-46px jitter at hold positions, 21-30px drift between cycles.
- **After count-limited PIO:** 877 X changes, hold segments show 4-18px range (std 1.3-4.9) at right position and 1-16px range (std 0.5-6.2) at left position. 74% of all X changes are stable (|delta| ≤ 8px).
- Best hold segment: range = 1px (idx 336-353).

Remaining observations:
- Position zero holds (position at DMX MSB 0) are very stable (near camera measurement floor).
- The hold segments are interrupted by brief transitions — still investigating whether these are real motor movements or measurement artifacts.
