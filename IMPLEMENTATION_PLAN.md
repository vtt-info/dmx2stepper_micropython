# DMX Stepper Implementation Plan

## Current Status
- The repo layout is stable:
  - `firmware/` for active Pico code
  - `hil/` for host-side DMX and vision validation
  - `puzzle_pieces/` for historical references
- RP2040 DMX reception on `GP29` is proven with PIO.
- PIO step generation is implemented in the active firmware.
- TMC2209 UART on `UART0 TX=GP0 / RX=GP1` is proven and remains the required control path for:
  - microstep selection
  - run current
  - hold current
  - driver enable/disable, because `EN` is hardwired to `GND`
- Single-axis startup homing with `UART StallGuard` is working.
- Single-axis homing plus centering has already passed optical validation.
- One-axis DMX runtime at `44 fps` has been validated functionally.
- The runtime firmware is now quiet by default:
  - RP2040 `print()` output is disabled unless explicitly enabled for debugging
- A smooth-ramp verification workflow now exists:
  - DMX scenario generation
  - OpenCV capture
  - trace smoothness scoring
- One-axis smooth-ramp runtime motion has now passed optical verification.

## MVP Goal
Ship a reliable first firmware that can:
- boot cleanly
- home reliably at startup
- move to center after homing
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
- Implemented automatic move-to-center after homing.
- Verified optically with full travel.

### Milestone 2: One-Axis DMX Runtime
- Reintroduced one-axis DMX runtime on top of the verified homing result.
- Validated stable DMX reception at `44 fps`.
- Simplified the current runtime test mode so manual testing only needs:
  - channel 1 = position MSB
  - channel 2 = position LSB

## Current Validation State
- Homing-and-centering has optical proof.
- One-axis runtime has functional proof.
- One-axis smooth runtime motion now has optical proof.
- The smooth-ramp verifier is the regression check for further runtime changes.

## Phase 3: Second Axis Bring-Up
- Add the second axis on top of the same architecture:
  - shared DMX receiver
  - one PIO step generator per axis
  - shared UART configuration logic
- Keep the one-axis smooth-ramp workflow passing while the second axis is added.

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
1. Treat the one-axis smooth-ramp workflow as the baseline regression check.
2. Add the second axis and repeat functional runtime validation under load.
3. Extend the optical ramp workflow to score both axes together.
4. Add soak and fault-handling checks before revisiting external `DIAG`.
