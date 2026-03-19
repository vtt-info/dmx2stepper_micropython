# DMX Stepper Implementation Plan

## Current Status
- The repo layout is stable:
  - `firmware/` for active Pico code
  - `hil/` for host-side vision validation
  - `puzzle_pieces/` for historical references
- RP2040 DMX reception on `GP29` is proven with PIO.
- PIO step generation is implemented in the active firmware.
- TMC2209 UART on `UART0 TX=GP0 / RX=GP1` is proven and remains the required control path for:
  - microstep selection
  - run current
  - hold current
  - driver enable/disable, because `EN` is hardwired to `GND`
- The current active runtime is at `1/128` microstepping.
- Single-axis startup homing with `UART StallGuard` is working.
- Single-axis homing plus centering has already passed optical validation.
- One-axis DMX runtime at `44 fps` has been validated functionally.
- The runtime firmware is now quiet by default:
  - RP2040 `print()` output is disabled unless explicitly enabled for debugging
- DMX channels `3..7` are now live again with:
  - channels `1..2` = 16-bit position target
  - channels `3..6` = run current, hold current, max speed, acceleration
  - channel `7` = enable
  - channel 8 = 255 triggeres a reset of the rp2040 to recalibrate
  - values `0..9` preserving firmware defaults for channels `3..7`
  - values `10..255` activating the configured runtime ranges
- The only CV system is the live opencv_streamer serving MJPEG on port 8080 and TCP X coordinates on port 9999.
- All DMX input comes exclusively from the lighting desk — no OLA, no scripted scenarios.
- The active runtime no longer measures its usable span on startup.
- The active one-axis startup flow is now:
  - seek one end with `UART StallGuard`
  - back off that end
- The current fixed logical travel window is `20000` microsteps with a `1000` step soft-end margin, so full-scale DMX currently maps into `1000..19000`.
- Recent live-DMX tuning confirmed a new limitation:
  - startup homing remains smooth
  - live DMX motion is still visibly jittery

## MVP Goal
Ship a reliable first firmware that can:
- boot cleanly DONE
- home reliably at startup DONE
- receive DMX at `44 fps`
- drive both motors with PIO-generated steps
- behave predictably on DMX loss
- be validated optically on real motion

For MVP, homing and runtime fault handling should be based on `UART StallGuard`, not external `DIAG`.

## Architecture Direction
- Keep `DMX receive` on PIO.
- Keep `step generation` on PIO for both axes.
- Keep `TMC2209 configuration` on UART.
- Keep the active firmware quiet by default to avoid adding serial overhead during runtime tests.
- Treat external `DIAG` as optional future work, not an MVP dependency.

## Why UART-Only Still Makes Sense
- DMX receive is already offloaded to PIO.
- Step generation is already offloaded to PIO.
- `44 fps` DMX is a light control workload for the RP2040 CPU.
- UART StallGuard is already working on real hardware.
- External `DIAG` remains electrically unresolved and has produced misleading results before.

## Completed Milestones

### Milestone 1: Single-Axis UART Homing
- Implemented startup homing with:
  - PIO step generation
  - UART StallGuard detection
  - UART-only enable/disable

### Milestone 2: One-Axis DMX Runtime
- Reintroduced one-axis DMX runtime on top of the verified homing result.
- Validated stable DMX reception at `44 fps`.
- Simplified the current runtime test mode so manual testing only needs:
  - channel 1 = position MSB
  - channel 2 = position LSB

## Current Validation State
- Homing has optical proof.
- One-axis runtime has functional proof.
- One-axis smooth runtime motion has one historical optical proof point, but the current fixed-span runtime still needs a fresh optical proof after the latest motion changes.
- The highest-priority open issue is visible jitter under live DMX updates even when homing remains mechanically smooth.
- **Root cause identified and fixed (2026-03-19):** DMX signal is confirmed clean. The jitter came from two PIO-level bugs:
  1. **PIO step overshoot:** The free-running PIO emitted extra pulses between the last counter poll and `stop()`. At 30kHz with 64-step chunks (~2ms per chunk, 2 polls), overshoot was up to 50%.
  2. **Ghost edge counting:** The counter SM ran continuously and counted false rising edges on the step pin during idle.
- **Fix deployed:** Count-limited PIO (`step_count_pio`) that generates exactly N pulses then idles, plus `Pin.PULL_DOWN` on the step pin.
- **Result:** Hold jitter reduced from 25-46px to 4-18px (right position) and 1-16px (left position). Position zero holds are at camera measurement floor.
- Remaining right-side instability is NOT mechanical (DMX MSB at 25 of 255, so far away from the end stop).

## Current Immediate Milestone: Verify Stability And Bring Up Second Axis
- Run extended hold test to verify long-term positional accuracy.
- Bring up second axis once one-axis stability is confirmed.

## Research Notes
- Martin fixture manuals describe two motion strategies:
  - tracking mode, where the controller sends small updates and the fixture tracks them
  - vector mode, where the fixture uses an internal speed channel and can produce smoother motion, especially when incoming updates are slow or irregular
- Martin manuals also explicitly mention digital filtering of tracking updates for smooth movement and `16-bit` pan/tilt positioning.
- ETC / High End documentation shows that commercial fixtures also invest in:
  - encoder calibration and multiple encoder technologies
  - pan/tilt curve selection such as `S-curve` vs `Linear`
  - software optimization to prevent misstepping
  - lower parked motor current and other motion-quality tuning
- Trinamic documents reinforce that driver-level features such as interpolation and chopper-mode tuning matter, but those alone are not enough if the higher-level motion planner chatters.

## Phase 3: Second Axis Bring-Up
- Add the second axis on top of the same architecture only after the new one-axis motion-quality milestone is met:
  - shared DMX receiver
  - one PIO step generator per axis
  - shared UART configuration logic
- Keep the refreshed one-axis smooth-motion regression passing while the second axis is added.

## Phase 4: Dual-Axis Optical Validation
- Extend the smooth-ramp workflow to score both visible traces.
- Verify that both axes remain stable under continuous `44 fps` DMX updates.

## Phase 5: Soak And Failure Handling
- Add longer-duration runtime tests.
- Define and verify behavior for:
  - DMX loss
  - startup homing failure
  - runtime jam / stall
- Keep UART StallGuard as the default MVP fault path.

## Phase 6: Optional External DIAG
- Return to external `DIAG` only after the MVP path is stable.
- Goals:
  - identify the real RP2040 input pin
  - verify polarity and pull requirements
  - prove that the external signal corresponds to real mechanical events

## Validation Strategy
- Prefer optical validation over console output.
- Use firmware JSON outputs for structured status.
- Keep the Pico runtime silent by default.
- Treat a runtime test as incomplete unless the OpenCV trace is available when optical proof is expected.

## Recommended Order From Here
1. Investigate Jitter cause on static DMX values and fix that first!
2. Run extended hold test to verify long-term positional accuracy with count-limited PIO.
3. Add the second axis and repeat functional runtime validation under load.
4. Extend optical validation to score both axes together.
5. Add soak and fault-handling checks before revisiting external `DIAG`.
