# Diary

## 2026-03-16

### Goal
- Finish milestone 1:
  - `UART-only` startup homing
  - `PIO` step generation
  - move to center after homing
  - optical verification
- Finish milestone 2:
  - one-axis DMX runtime on top of the homing result
  - prove stable DMX reception at `44 fps`

### What I Changed
- Reworked the Pico firmware in `firmware/main.py` so it now:
  - homes with `PIO` step generation
  - uses `UART StallGuard` only for end-stop detection
  - centers after measuring the end-to-end span
  - stays alive for one-axis DMX runtime
- Kept `TMC2209` configuration on UART:
  - microsteps
  - run current
  - hold current
  - enable/disable because `EN` is tied to `GND`
- Moved the DMX receiver state machine to `SM4` so it no longer conflicts with the stepper PIO state machines.
- Added explicit verification-mode overrides:
  - `RUN_RUNTIME_AFTER_HOMING`
  - `RUNTIME_EXIT_AFTER_MS`
  These let the host run homing-only or time-boxed runtime checks cleanly.
- Added a one-axis runtime scenario:
  - `hil/scenarios/one_axis_runtime.csv`
- Added a dedicated runtime verifier:
  - `hil/verify_one_axis_dmx_runtime.py`
- Updated `hil/verify_pio_homing.py` so it verifies the new `UART-only` homing contract instead of insisting on external `DIAG`.
- Increased the DMX frame gap timeout in `firmware/dmx_receiver.py` so the Pico keeps reading the full 8-channel block under load.

### Key Tuning Decisions
- The first working homing setup was:
  - `STEP=GP2`
  - `DIR=GP3`
  - `home_direction=-1`
  - `speed=800 Hz`
- The important homing change was reducing the UART stall confirmation count from `4` to `2`.
  - Before that, the firmware kept banging into the end-stop too long.
  - After that, it stopped early enough to be mechanically reasonable.
- I also removed the speed sweep for homing bring-up.
  - The wider sweep created too much unnecessary stalling when the first good trial was already known.

### Milestone 1 Result
- Milestone 1 passed with optical verification.
- Passing artifacts:
  - `hil/captures/vision_homing_20260316_170603.csv`
  - `hil/captures/homing_result_20260316_170603.json`
- Passing summary:
  - firmware `success=true`
  - firmware `centered=true`
  - selected trial `0`
  - optical travel span `338.7 deg`
  - final end span `0.0 deg`
  - center error `0.95 deg`

### Milestone 2 Result
- Milestone 2 passed functionally in a headless check.
- Passing artifacts:
  - `hil/captures/runtime_homing_result_20260316_171443.json`
  - `hil/captures/runtime_status_20260316_171443.json`
  - `hil/captures/dmx_runtime_20260316_171443.csv`
- Passing summary:
  - startup homing succeeded first
  - Pico reported `644` received DMX frames during the timed runtime
  - runtime stayed active
  - the position moved from `1530` to `1922` steps under DMX control

### Important Findings
- External `DIAG` is still not needed for the MVP path.
  - `UART StallGuard` is now the more trustworthy path on this hardware.
- The DMX receiver was truncating frames under runtime load.
  - Symptom: only channels `1` and `2` arrived reliably, while channels `3..7` looked like zeros.
  - Fix: widen the inter-byte timeout in `firmware/dmx_receiver.py`.
- The host-side camera stack became unstable during milestone 2 verification.
  - `vision_observer.py` started failing with `Device or resource busy`.
  - `pipewire` and `wireplumber` were part of the problem, but even after killing them the camera path stayed flaky.
  - Because of that, milestone 2 was verified headlessly instead of optically.

### What Was Learned
- The RP2040 architecture is still the right one:
  - PIO for DMX input
  - PIO for step generation
  - UART for TMC2209 configuration and StallGuard polling
- The current single-axis path is strong enough to keep building on:
  - startup homing works
  - centering works
  - one-axis DMX runtime works
  - the Pico can keep up with `44 fps` DMX traffic

### What I Would Check Next
- Restore reliable host camera access so runtime motion can be optically verified again.
- Re-run milestone 2 with the camera once the host-side video stack is stable.
- Then add the second axis using the same architecture instead of returning to the external `DIAG` problem first.

### Milestone 2 Optical Follow-Up
- The camera path recovered later the same day and `vision_observer.py` completed successfully again.
- I added a dedicated smooth-ramp workflow:
  - `hil/scenarios/smooth_position_ramp.csv`
  - `hil/verify_smooth_dmx_ramp.py`
  - `run_smooth_ramp_check.sh`
- The Pico runtime was kept quiet by default and made easier to debug explicitly with:
  - `firmware/config.py: DEBUG_LOGGING`
  - `./run_firmware.sh --debug`
- The motion loop needed one practical runtime tuning change:
  - reduce the chunk size to `10` steps
  - reduce the runtime control sleep to `2 ms`
- The host-side vision path also needed calibration for this Pi camera rig:
  - use `640x480` capture
  - lower contour `MIN_AREA` to `150`
  - keep the smooth-ramp verifier thresholds realistic for the observed camera cadence

### Smooth Ramp Result
- The one-axis smooth-ramp run now passes optically.
- Passing artifacts:
  - `hil/captures/smooth_ramp_homing_result_20260316_214659.json`
  - `hil/captures/smooth_ramp_runtime_status_20260316_214659.json`
  - `hil/captures/vision_smooth_ramp_20260316_214659.csv`
  - `hil/captures/dmx_smooth_ramp_20260316_214659.csv`
  - `hil/captures/smooth_ramp_summary_20260316_214659.json`
- Passing summary:
  - runtime active `true`
  - received DMX frames `917`
  - visible travel span `242.0 deg`
  - segment 1 monotonic ratio `1.0`, max step `6.4 deg`
  - segment 2 monotonic ratio `1.0`, max step `28.7 deg`
  - segment 3 monotonic ratio `0.7553`, max step `16.5 deg`

### Updated Conclusion
- Milestone 2 is now complete both functionally and optically.
- The smooth-ramp workflow is the correct regression check before further runtime changes.
- The next engineering step is second-axis bring-up, not more one-axis firmware refactoring.
