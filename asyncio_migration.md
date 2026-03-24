# Asyncio Migration Plan

## Why

The current firmware uses `_thread` to run DMX reception on a second core. MicroPython on RP2040 shares a GIL between cores — only one core executes Python at a time. The DMX receiver's busy-wait loops (`_wait_for_break`: up to 100ms, `read_frame`: ~23ms) hold the GIL, starving the motion control loop. When the control loop resumes after a stall, it emits catch-up step bursts that appear as visible position spikes.

Additionally, `move_fixed_steps_blocking()` sleeps for the full PIO step duration + 2ms margin. During that sleep, no computation happens — the next move can't be prepared, and DMX can't be read.

Moving to cooperative `asyncio` on a single core eliminates the GIL problem entirely and turns every wait into productive time.

## Architecture: Before and After

### Before (current)

```
Core 0 (main thread)              Core 1 (DMX thread)
┌─────────────────────┐           ┌──────────────────┐
│ while True:         │           │ while True:      │
│   snapshot()     ←──┼── GIL ──→│   read_frame()   │
│   controller.update()│          │   update_shared() │
│   move_blocking()   │           │                  │
│   sleep(wait_ms) ◄──┼── IDLE   │   wait_for_break()│
│                     │           │   ◄── BUSY POLL  │
└─────────────────────┘           └──────────────────┘
       ▲ GIL contention causes 20-100ms stalls
```

### After (asyncio)

```
Single core — cooperative event loop
┌──────────────────────────────────────────┐
│ asyncio.run(main_loop())                 │
│                                          │
│ ┌─ motion_task ──────────────────────┐   │
│ │ compute next chunk (math only)     │   │
│ │ fire PIO (non-blocking)            │   │
│ │ await asyncio.sleep_ms(wait)  ─────┼─► │
│ │ finalize PIO, record steps         │   │
│ └────────────────────────────────────┘   │
│                                          │
│ ┌─ dmx_task ─────────────────────────┐   │
│ │ poll for break (yield between)     │   │
│ │ read bytes from FIFO               │   │
│ │ update target                      │   │
│ │ await asyncio.sleep_ms(0)     ─────┼─► │
│ └────────────────────────────────────┘   │
│                                          │
│ ┌─ status_task (optional) ───────────┐   │
│ │ periodic status file writes        │   │
│ │ await asyncio.sleep_ms(500)   ─────┼─► │
│ └────────────────────────────────────┘   │
└──────────────────────────────────────────┘
  Every await yields to the event loop →
  other tasks run during every wait
```

## Multi-axis scaling

With asyncio, a second stepper is nearly free:

```python
async def motion_task(controllers, axes):
    while True:
        chunks = [c.compute_next() for c in controllers]
        for ax, chunk in zip(axes, chunks):
            ax.start_pio(chunk)          # all PIOs fire in parallel
        await asyncio.sleep_ms(wait)     # one wait covers all axes
        for ax, c, chunk in zip(axes, controllers, chunks):
            ax.finalize_pio()
            c.record_steps(chunk)
```

Both PIO hardware blocks generate pulses simultaneously while Python prepares the next iteration. The blocking architecture would serialize them (5ms + 5ms = 10ms per iteration).

## Files to change

| File | Change | Notes |
|------|--------|-------|
| `main.py` | Major rewrite | asyncio event loop, tasks instead of threads |
| `pio_stepper.py` | Add async step method | `start_counted()` + `async wait_done()` |
| `dmx_receiver.py` | Add async read method | `async read_frame()` with cooperative yields |
| `config.py` | Remove `RUNTIME_CONTROL_SLEEP_MS` | No longer needed; add any new async timing params |
| `tmc2209.py` / `tmc2209_uart.py` | No change | Only used during homing (synchronous is fine) |

## Detailed changes

### 1. `pio_stepper.py` — non-blocking step API

Keep `move_fixed_steps_blocking()` for homing (synchronous context). Add a new non-blocking pair:

```python
def start_counted(self, steps, direction, speed_hz):
    """Fire PIO to generate exactly N pulses. Returns immediately."""
    steps = max(0, int(steps))
    if steps == 0:
        return 0
    if self._freerun_mode:
        self.stop()
        self._init_step_sm_counted()
    self.set_direction(direction)
    delay = self.speed_to_delay(speed_hz)
    cycles_per_step = self.PIO_FIX + delay * self.PIO_VAR
    self._pending_wait_us = int(steps * cycles_per_step * 1_000_000 / self.step_frequency)
    self._pending_steps = steps
    self.step_sm.put(steps - 1)
    self.step_sm.put(delay)
    self.step_sm.active(1)
    self._running = True
    return steps

def finalize_counted(self):
    """Call after PIO has finished. Cleans up state."""
    self.step_sm.active(0)
    self.step_pin.value(0)
    self._running = False
    return self._pending_steps

@property
def pending_wait_ms(self):
    """Minimum ms to wait before calling finalize_counted()."""
    return max(1, self._pending_wait_us // 1000) + 1
```

The +1ms margin (instead of +2) is safe here because the async loop can absorb small overruns — `finalize_counted()` just deactivates the SM, and if the PIO finished early, that's fine.

### 2. `dmx_receiver.py` — async frame reading

Add an async method alongside the existing sync one (homing still uses sync):

```python
async def async_read_frame(self):
    """Read one DMX frame, yielding between polls."""
    if not self.receiving:
        return False

    # Wait for break — yield every 1ms instead of busy-polling
    deadline = time.ticks_add(time.ticks_ms(), 100)
    found_break = False
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        if self._pin_value() == 0:
            start = time.ticks_us()
            # Short busy-wait for break duration (OK — <200us max)
            while self._pin_value() == 0:
                if time.ticks_diff(time.ticks_us(), start) > 200:
                    while self._pin_value() == 0:
                        pass
                    found_break = True
                    break
            if found_break:
                break
            duration = time.ticks_diff(time.ticks_us(), start)
            if duration > 44:
                found_break = True
                break
        await asyncio.sleep_ms(1)  # yield to motion task

    if not found_break:
        return False

    # Drain stale FIFO
    while self.sm.rx_fifo() > 0:
        self.sm.get()

    time.sleep_us(50)  # mark-after-break (must be tight)

    # Read frame bytes — yield periodically
    bytes_received = 0
    timeout_start = time.ticks_us()
    while bytes_received <= DMX_MAX_CHANNELS:
        if self.sm.rx_fifo() > 0:
            self.frame_buffer[bytes_received] = (self.sm.get() >> 24) & 0xFF
            bytes_received += 1
            timeout_start = time.ticks_us()
        else:
            if time.ticks_diff(time.ticks_us(), timeout_start) > FRAME_GAP_TIMEOUT_US:
                break
            await asyncio.sleep_ms(0)  # yield on empty FIFO

    if bytes_received == 0:
        return False

    self.last_start_code = self.frame_buffer[0]
    if self.last_start_code != DMX_START_CODE:
        self.start_code_errors += 1
    self.last_bytes_received = bytes_received
    self.frame_count += 1
    self.last_frame_time = time.ticks_ms()
    return True
```

Note: the short busy-waits for break detection (<200us) are kept as-is. Only the outer polling loop yields. The byte-by-byte FIFO read yields on empty FIFO with `sleep_ms(0)`.

### 3. `main.py` — asyncio runtime

**Homing stays synchronous.** It runs once at boot before the event loop starts. No threads, no async — just the existing sequential homing code. This avoids touching the well-tested homing logic.

**Runtime becomes async:**

```python
import asyncio

# Shared state — no lock needed, single-threaded
class DMXTarget:
    def __init__(self):
        self.target_u16 = config.DEFAULT_TARGET_U16
        self.frame_count = 0

async def dmx_task(dmx, target):
    """Read DMX frames and update target. Yields between frames."""
    while True:
        if await dmx.async_read_frame():
            if dmx.last_start_code == 0x00:
                channels = dmx.get_channels(config.DMX_START_CHANNEL, 8)
                if int(channels[7]) == 255:
                    machine.reset()
                target.target_u16 = (int(channels[0]) << 8) | int(channels[1])
                target.frame_count = dmx.get_frame_count()

async def motion_task(controller, target, axis):
    """Run motion control loop. Yields during PIO step execution."""
    while True:
        controller.apply_target(target.target_u16)
        chunk = controller.compute_next_chunk()

        if chunk.steps > 0:
            axis.start_counted(chunk.steps, chunk.direction, chunk.speed_hz)
            await asyncio.sleep_ms(axis.pending_wait_ms)
            axis.finalize_counted()
            controller.record_moved(chunk)
        else:
            await asyncio.sleep_ms(1)  # idle — wait for target change

async def main_async(result, homing_trial):
    runtime_axis = build_axis(...)
    controller = PositionController(runtime_axis, ...)

    dmx = DMXReceiver(pin_num=config.DMX_PIN, sm_id=config.DMX_SM_ID)
    dmx.start()
    target = DMXTarget()

    await asyncio.gather(
        dmx_task(dmx, target),
        motion_task(controller, target, runtime_axis),
    )

# Boot sequence
def main():
    driver = build_driver()
    configure_driver(driver)
    result = run_homing(driver, {})  # synchronous, no change
    if result["success"]:
        asyncio.run(main_async(result, ...))
```

**Key differences from current code:**
- `SharedDMXState` becomes `DMXTarget` — no lock needed (single-threaded)
- `_thread.start_new_thread()` is gone
- `controller.update()` splits into `compute_next_chunk()` (pure math) and `record_moved()` (state update)
- The `await` in motion_task is where DMX reading happens

### 4. `PositionController` refactor

Split the current `ChunkedPositionController.update()` into two pure functions:

```python
class PositionController:
    def compute_next_chunk(self):
        """Pure math — returns (steps, direction, speed_hz) or None."""
        now_ms = time.ticks_ms()
        elapsed_ms = time.ticks_diff(now_ms, self._last_update_ms)
        if elapsed_ms <= 0:
            return Chunk(0, 0, 0)
        self._last_update_ms = now_ms

        # ... existing acceleration/speed/accumulator logic ...

        return Chunk(steps_to_take, direction, effective_speed)

    def record_moved(self, chunk, actual_steps):
        """Update position after PIO execution."""
        self.current_position_steps += actual_steps if chunk.direction > 0 else -actual_steps
        self._step_accumulator -= actual_steps
```

The `Chunk` can be a simple namedtuple or just a tuple. This separation means the math runs BEFORE the PIO fires, overlapping computation with the previous chunk's execution.

### 5. `config.py` cleanup

Remove:
- `RUNTIME_CONTROL_SLEEP_MS` — replaced by natural async yields

Keep unchanged:
- All homing parameters
- `RUNTIME_MAX_CHUNK_STEPS`, `MOTOR_MAX_SPEED_HZ`, `MOTOR_ACCELERATION_S2`
- All PIO/pin/driver configuration

## Migration steps — incremental with validation

Each step produces a deployable firmware that is validated against the camera before proceeding. No step changes more than one subsystem.

### Step 1: Add non-blocking PIO API

**Change:** `pio_stepper.py` only — add `start_counted()`, `finalize_counted()`, `pending_wait_ms`

**Validation:** Deploy, run homing + runtime with existing blocking code. Camera capture must show no regression (the new methods aren't called yet, just added).

### Step 2: Add async DMX reader

**Change:** `dmx_receiver.py` only — add `async_read_frame()` alongside existing `read_frame()`

**Validation:** Same as step 1 — new code exists but isn't called yet.

### Step 3: Split controller into compute + record

**Change:** `main.py` — refactor `ChunkedPositionController` to separate `compute_next_chunk()` and `record_moved()`. Keep `update()` as a thin wrapper calling both for backwards compatibility.

**Validation:** Deploy, capture. The wrapper ensures identical behavior. Camera must show no regression.

### Step 4: Convert runtime loop to asyncio

**Change:** `main.py` — replace `_thread` + `while True` with `asyncio.run()` + `asyncio.gather()`. Remove `SharedDMXState` lock. Wire up `dmx_task` and `motion_task`.

**Validation:** This is the critical step. Deploy, capture 90s, analyze. Compare stdev and max_dev against baseline. Expect improvement (fewer spikes) or at worst parity.

### Step 5: Clean up dead code

**Change:** Remove `SharedDMXState`, `RUNTIME_CONTROL_SLEEP_MS`, the `update()` wrapper, `_thread` import, any other code that only existed for the threaded architecture.

**Validation:** Final capture. Confirm clean architecture with no regression.

## Agentic validation loop

Each migration step runs through an automated validation cycle:

```
┌─────────────────────────────────────────────────────┐
│                  MIGRATION STEP N                   │
│                                                     │
│  1. Make code change (one subsystem only)            │
│  2. Deploy to Pico via mpremote                      │
│  3. Wait for homing (~35s)                           │
│  4. Run 3x 90s captures via capture_and_visualize.sh │
│  5. Compute median stdev and max_dev across 3 runs   │
│  6. Compare against baseline median (3-run baseline) │
│                                                     │
│  ┌─ IF regression (median stdev > baseline + 2px) ──┐│
│  │  • Diagnose: read capture PNGs, check patterns   ││
│  │  • Identify root cause in the step's changes     ││
│  │  • Fix and re-run from step 2                    ││
│  │  • Max 3 fix attempts per step                   ││
│  │  • If still regressed: revert step, report       ││
│  └──────────────────────────────────────────────────┘│
│                                                     │
│  ┌─ IF pass ────────────────────────────────────────┐│
│  │  • Record new baseline = this step's median      ││
│  │  • Proceed to step N+1                           ││
│  └──────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────┘
```

### Baseline establishment (before any changes)

Run 5x 90s captures on the current committed firmware. Record:
- Median stdev, median max_dev
- Min/max range (to understand natural variance)

This establishes the quality floor that no migration step may breach.

### Per-step metrics

For each step, run 3x 90s captures and record:
- Per-run: stdev, max_dev, spike count (>8px deviation from local trend)
- Median of 3 runs for comparison
- Pass criterion: median stdev ≤ baseline median + 2px (within natural variance)

### Spike counting

The current `analyze_x_data.py` reports fade stdev/max_dev but doesn't count individual spikes. Add a spike counter to the analysis output:

```python
# Count points deviating >8px from local trend (3-point window)
spike_count = 0
for i in range(2, len(values) - 2):
    local_avg = (values[i-2] + values[i-1] + values[i+1] + values[i+2]) / 4
    if abs(values[i] - local_avg) > 8:
        spike_count += 1
```

This gives a discrete count of visible spikes per capture — more stable than stdev across runs.

### End-state validation

After step 5 (cleanup), run a final 5x 90s capture set. Compare against the original baseline:
- Spike count should be equal or lower
- Stdev should be equal or lower
- No amplitude drift (x range stays within ±10px of baseline)
- Homing still succeeds reliably (test 3 consecutive power cycles)

## Risk mitigations

| Risk | Mitigation |
|------|------------|
| `asyncio.sleep_ms(0)` doesn't actually yield on MicroPython RP2040 | Test with `asyncio.sleep_ms(1)` as fallback; verify with timing instrumentation |
| DMX frame timing too tight for cooperative yields | Keep short busy-waits (<200us) for break detection; only yield on longer waits |
| Homing breaks during migration | Homing stays fully synchronous — no asyncio changes to homing code |
| MicroPython `asyncio` overhead too high | Profile: if event loop overhead >1ms, consider `uasyncio` optimizations or reduce task count |
| PIO finalize called too early (step loss) | `pending_wait_ms` includes +1ms margin; also verify with counter SM read after finalize |
| Multi-axis PIO resource conflict | PIO0: SM0+SM1 (axis A step+counter), SM2+SM3 (axis B step+counter). PIO1: SM4 (DMX). Fits exactly. |

## What stays the same

- All PIO programs (`step_count_pio`, `step_freerun_pio`, `step_counter_pio`, `dmx_rx`)
- All TMC2209 UART communication
- Homing algorithm (synchronous, runs before asyncio loop)
- `config.py` parameter meanings and values
- HIL test infrastructure (capture.py, analyze_x_data.py, etc.)
- OpenCV streamer
- Hardware wiring
