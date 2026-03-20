# Sleeker Firmware Development Plan

## Goal
Reduce the firmware codebase by removing DMX mapping code (channels 3-7), hardcoding motor parameters as constants, and stripping all DMX metadata tracking. Keep position control (channels 1-2) and channel 8 reset trigger.

## Development Method
1. Each step: modify code → upload to Pico → verify X data on port 9999 matches expected sawtooth pattern
2. If X data breaks → revert and retry differently
3. Only proceed when step is verified

## Reference X Data (port 9999 — OpenCV vision stream)
Expected sawtooth pattern from lighting desk loop (~60s cycle):

| Phase | DMX position | X value | Notes |
|---|---|---|---|
| Homing | CH8=255 → reset | 199 → 17-35 | Stepper hits hard endstop |
| Soft-end left | 0 | 222 | Left margin after homing |
| LEFT | 558 | 250-270 | ~25 right of soft-end |
| RIGHT | 6521 | 580-592 | ~360 right of soft-end |
| LEFT | 558 | 250-270 | repeat |
| RIGHT | 6521 | 580-592 | repeat |
| Reset | CH8=255 | drops to 17-35 | loop restarts |

## Motor Constants (hardcoded, no DMX control)

| Constant | Value | Source |
|---|---|---|
| Run current | 24 (TMC) | `DEFAULT_RUN_CURRENT` |
| Hold current | 12 (TMC) | `DEFAULT_HOLD_CURRENT` |
| Max speed | 18684 Hz | DMX 100 maps to ~18684 Hz via current scaling |
| Acceleration | 300000 steps/s² | DMX 255 maps to 300000 (max range) |
| Enabled | True | Motor always on (via `configure_driver()` at init) |

## Steps

### Step 0: Archive original firmware ✅
- [x] Rename `firmware/` → `firmware_v1/`
- [x] Create new `firmware/` directory
- [x] Copy unchanged files: `dmx_receiver.py`, `tmc2209.py`, `tmc2209_uart.py`, `pio_stepper.py`, `motion_axis.py`

### Step 1: Minimal `config.py` ✅
- [x] `DMX_CHANNEL_COUNT = 2` (was 8)
- [x] Remove: `DMX_LOSS_TIMEOUT_MS`, `DMX_FRAME_IMMEDIATE_DELTA_LIMIT`, `DMX_FRAME_CONFIRM_COUNT`
- [x] Remove: named channel constants (`POSITION_MSB_CHANNEL`, `RUN_CURRENT_CHANNEL`, etc.)
- [x] Remove: `ENABLE_THRESHOLD`, `DMX_PARAMETER_ACTIVE_MIN`
- [x] Remove: `RUNTIME_MIN/MAX_*` for currents/speed/accel
- [x] Remove: `RUNTIME_DEFAULT_*` for current/speed/accel/enable
- [x] Remove: `RUNTIME_POSITION_ONLY_MODE`
- [x] Remove: `RUNTIME_DMX_*` startup/delay constants
- [x] Add: `MOTOR_MAX_SPEED_HZ = 18684`, `MOTOR_ACCELERATION_S2 = 300000`
- Note: structural only, no behavior change to verify

### Step 2: Strip DMX mapping functions from `main.py` ✅
- [x] Remove: `map_dmx_parameter()`, `map_dmx_enable()`, `map_u16_to_steps_with_margin()`
- [x] Keep: `map_u16_to_steps()` (still used for position mapping)
- Verified via Steps 3-8 combined upload

### Step 3: Strip `SharedDMXState.update_from_channels()` — motor params only ✅
- [x] Replace channels 2-7 mapping with hardcoded values
- [x] Keep: `target_u16` from channels 0-1, `frame_count`
- Verified via Steps 3-8 combined upload

### Step 4: Strip `SharedDMXState.snapshot()` — remove metadata ✅
- [x] Return only: `target_u16`, `frame_count`
- [x] Remove: `signal_present`, `control_signal_present`, `age_ms`, `complete_age_ms`, `last_channels`, `last_bytes_received`, `frame_complete`, `short_frame_count`, `complete_frame_count`, `start_code_errors`, `last_alignment_offset`, `last_candidate_count`, `last_candidate_score`, `rejected_frame_count`, `pending_frame_count`
- Verified via Steps 3-8 combined upload

### Step 5: Simplify `dmx_worker()` — remove jump protection ✅
- [x] Remove: `last_confirmed_u16`, `pending_u16`, `confirm_count`, `delta` checks
- [x] Direct: each frame sets `target_u16` directly
- [x] Keep: start code validation (but removed frame length check — original always received 512 bytes)
- [x] Keep: channel 8 reset trigger (`if channels[7] == 255: machine.reset()`)
- [x] Simplify `update_from_channels()` call to pass only needed args
- Verified via Steps 3-8 combined upload

### Step 6: Simplify `ChunkedPositionController.apply_snapshot()` ✅
- [x] Use `config.MOTOR_MAX_SPEED_HZ` and `config.MOTOR_ACCELERATION_S2` directly
- [x] Keep: `target_u16` → `target_position_steps` mapping via `map_u16_to_steps_with_margin()`
- [x] `map_u16_to_steps_with_margin()` was NOT removed — it was kept in Step 2
- Verified via Steps 3-8 combined upload

### Step 7: Strip runtime loop — remove enable/current management ✅
- [x] Remove: `desired_enabled`, `driver_enabled`, `applied_run_current`, `applied_hold_current`, `last_currents`, `target_signature`, `last_state_signature`
- [x] Remove: all `driver.set_driver_enabled_via_uart()` calls in loop
- [x] Remove: all `driver.set_run_hold_current()` calls in loop
- [x] Keep: `controller.update()`, `time.sleep_ms(0)`, idle/stable tracking
- [x] Added `_last_target_u16` to `ChunkedPositionController` for stable_since tracking
- Verified via Steps 3-8 combined upload

### Step 8: Strip `build_runtime_status()` — remove DMX metadata ✅
- [x] Remove: all `snapshot["..."]` fields except `target_u16`
- [x] Remove: `requested_enabled`, `applied_enabled` (always True)
- [x] Remove: `run_current`, `hold_current`, `max_speed_hz`, `acceleration_steps_s2`
- [x] Removed event logging entirely (`append_recent_event` calls)
- [x] Removed `debug_log` periodic print (was full of DMX metadata)
- Verified via Steps 3-8 combined upload

### Verification: Full ~60s sawtooth loop ✅
After hard reset of Pico (required after deploy):

```
Phase 1 (DMX=6521 RIGHT): 262 → 589-592
Phase 2 (CH8=255 reset): drops to null
Phase 3 (homing): 636 → 620 → ... → 16-22
Phase 4 (DMX=0 center): 209 → 221-222
Phase 5 (DMX=558 LEFT): rises to 593-594
Phase 6 (DMX=6521 RIGHT): 580 → 268 → 293 → 593
Phase 7 (CH8=255 reset): null
Phase 8 (homing): 636 → 16-22
Phase 9 (DMX=0 center): 221-222
```
Matches reference pattern perfectly. CH8 reset confirmed working.

## Final Deliverable
- `firmware/` contains: `main.py` (767 lines), `config.py` (91 lines), `dmx_receiver.py`, `tmc2209.py`, `tmc2209_uart.py`, `pio_stepper.py`, `motion_axis.py`, `deploy.sh` (8 files)
- `firmware_v1/` contains: original 10 files (untouched, archived)
- `main.py`: 1109 → 767 lines (~31% reduction)
- `config.py`: 123 → 91 lines (~26% reduction)
- Behavior: Identical sawtooth X pattern, no DMX mapping, no metadata tracking, motor always enabled
