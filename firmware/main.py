"""UART-only homing with PIO steps followed by one-axis DMX runtime."""

import _thread
import json
import os
import time

import config
from dmx_receiver import DMXReceiver
from pio_stepper import PIOStepper
from tmc2209 import TMC2209


def clamp(value, low, high):
    return max(low, min(high, int(value)))


def board_name():
    return os.uname().machine


def write_json(path, payload):
    with open(path, "w") as handle:
        handle.write(json.dumps(payload))


def map_byte(value, low, high):
    value = clamp(value, 0, 255)
    low = int(low)
    high = int(high)
    return low + ((high - low) * value + 127) // 255


def map_u16_to_steps(value, span_steps):
    value = clamp(value, 0, 65535)
    span_steps = max(1, int(span_steps))
    return (value * span_steps + 32767) // 65535


def stallguard_adjustment(microsteps):
    return max(1.0, float(microsteps) / 8.0)


def home_speed_trials():
    adjustment = stallguard_adjustment(config.MICROSTEP_MODE)
    speeds = []
    for requested in config.HOME_SPEED_TRIALS_1_8_HZ:
        scaled = int(float(requested) * adjustment)
        clamped = clamp(scaled, config.HOME_MIN_FREQ_1_8_HZ, config.HOME_MAX_FREQ_1_8_HZ)
        if clamped not in speeds:
            speeds.append(clamped)
    return tuple(speeds)


def median_int(values):
    ordered = sorted(int(value) for value in values)
    count = len(ordered)
    middle = count // 2
    if count % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) // 2


def derive_uart_threshold(startup_sg_values):
    baseline = median_int(startup_sg_values)
    threshold = int(float(baseline) * float(config.HOME_UART_THRESHOLD_RATIO))
    return clamp(threshold, config.HOME_UART_THRESHOLD_MIN, config.HOME_UART_THRESHOLD_MAX)


def debug_log(message):
    if bool(config.DEBUG_LOGGING):
        print(message)


class SharedDMXState:
    """State shared between the DMX reader thread and the motion loop."""

    def __init__(self):
        self._lock = _thread.allocate_lock()
        self.target_u16 = int(config.RUNTIME_DEFAULT_TARGET_U16)
        self.run_current = int(config.DEFAULT_RUN_CURRENT)
        self.hold_current = int(config.DEFAULT_HOLD_CURRENT)
        self.max_speed_hz = int(config.RUNTIME_DEFAULT_MAX_SPEED_HZ)
        self.acceleration_steps_s2 = int(config.RUNTIME_DEFAULT_ACCEL_STEPS_S2)
        self.enabled = bool(config.RUNTIME_DEFAULT_ENABLED)
        self.frame_count = 0
        self.last_frame_ms = 0
        self.start_code_errors = 0
        self.last_channels = [0] * int(config.DMX_CHANNEL_COUNT)

    def update_from_channels(self, channels, frame_count, last_frame_ms, start_code_errors):
        self._lock.acquire()
        try:
            self.last_channels = list(channels)
            self.target_u16 = (int(channels[0]) << 8) | int(channels[1])
            if bool(config.RUNTIME_POSITION_ONLY_MODE):
                self.run_current = int(config.DEFAULT_RUN_CURRENT)
                self.hold_current = int(config.DEFAULT_HOLD_CURRENT)
                self.max_speed_hz = int(config.RUNTIME_DEFAULT_MAX_SPEED_HZ)
                self.acceleration_steps_s2 = int(config.RUNTIME_DEFAULT_ACCEL_STEPS_S2)
                self.enabled = bool(config.RUNTIME_DEFAULT_ENABLED)
            else:
                self.run_current = map_byte(channels[2], 0, 31)
                self.hold_current = map_byte(channels[3], 0, 31)
                self.max_speed_hz = map_byte(
                    channels[4],
                    config.RUNTIME_MIN_SPEED_HZ,
                    config.RUNTIME_MAX_SPEED_HZ,
                )
                self.acceleration_steps_s2 = map_byte(
                    channels[5],
                    config.RUNTIME_MIN_ACCEL_STEPS_S2,
                    config.RUNTIME_MAX_ACCEL_STEPS_S2,
                )
                self.enabled = int(channels[6]) >= int(config.ENABLE_THRESHOLD)
            self.frame_count = int(frame_count)
            self.last_frame_ms = int(last_frame_ms)
            self.start_code_errors = int(start_code_errors)
        finally:
            self._lock.release()

    def snapshot(self, now_ms):
        self._lock.acquire()
        try:
            age_ms = None
            signal_present = False
            if self.last_frame_ms:
                age_ms = time.ticks_diff(int(now_ms), int(self.last_frame_ms))
                signal_present = age_ms <= int(config.DMX_LOSS_TIMEOUT_MS)
            return {
                "target_u16": int(self.target_u16),
                "run_current": int(self.run_current),
                "hold_current": int(self.hold_current),
                "max_speed_hz": int(self.max_speed_hz),
                "acceleration_steps_s2": int(self.acceleration_steps_s2),
                "enabled": bool(self.enabled),
                "frame_count": int(self.frame_count),
                "last_frame_ms": int(self.last_frame_ms),
                "age_ms": age_ms,
                "signal_present": bool(signal_present),
                "start_code_errors": int(self.start_code_errors),
                "last_channels": list(self.last_channels),
            }
        finally:
            self._lock.release()


class ChunkedPositionController:
    """Acceleration-limited position tracking using short PIO step chunks."""

    def __init__(self, axis, span_steps):
        self.axis = axis
        self.span_steps = max(1, int(span_steps))
        self.current_position_steps = self.span_steps // 2
        self.target_position_steps = self.current_position_steps
        self.current_speed_hz = 0.0
        self.max_speed_hz = float(config.RUNTIME_DEFAULT_MAX_SPEED_HZ)
        self.acceleration_steps_s2 = float(config.RUNTIME_DEFAULT_ACCEL_STEPS_S2)
        self.enabled = True
        self._last_update_ms = time.ticks_ms()
        self._step_accumulator = 0.0

    def hold_position(self):
        self.target_position_steps = int(self.current_position_steps)
        self.current_speed_hz = 0.0
        self._step_accumulator = 0.0
        self._last_update_ms = time.ticks_ms()

    def apply_snapshot(self, snapshot):
        self.max_speed_hz = float(max(config.RUNTIME_MIN_SPEED_HZ, int(snapshot["max_speed_hz"])))
        self.acceleration_steps_s2 = float(max(config.RUNTIME_MIN_ACCEL_STEPS_S2, int(snapshot["acceleration_steps_s2"])))
        self.enabled = bool(snapshot["enabled"])
        if self.enabled:
            self.target_position_steps = int(map_u16_to_steps(snapshot["target_u16"], self.span_steps))
        else:
            self.hold_position()

    def _approach(self, current, target, delta):
        if current < target:
            return min(target, current + delta)
        if current > target:
            return max(target, current - delta)
        return target

    def update(self):
        now_ms = time.ticks_ms()
        elapsed_ms = time.ticks_diff(now_ms, self._last_update_ms)
        if elapsed_ms <= 0:
            return 0
        self._last_update_ms = now_ms

        if not self.enabled:
            self.current_speed_hz = 0.0
            self._step_accumulator = 0.0
            return 0

        distance = int(self.target_position_steps) - int(self.current_position_steps)
        if distance == 0 and abs(self.current_speed_hz) < 1.0:
            self.current_speed_hz = 0.0
            self._step_accumulator = 0.0
            return 0

        elapsed_s = elapsed_ms / 1000.0
        direction = 0
        if distance > 0:
            direction = 1
        elif distance < 0:
            direction = -1

        desired_speed = 0.0
        if direction != 0:
            moving_direction = 0
            if self.current_speed_hz > 0:
                moving_direction = 1
            elif self.current_speed_hz < 0:
                moving_direction = -1

            if moving_direction != 0 and moving_direction != direction:
                desired_speed = 0.0
            else:
                stop_distance = 0.0
                if self.acceleration_steps_s2 > 0:
                    stop_distance = (self.current_speed_hz * self.current_speed_hz) / (2.0 * self.acceleration_steps_s2)

                if abs(distance) <= stop_distance:
                    desired_speed = direction * min(
                        self.max_speed_hz,
                        max(1.0, (2.0 * self.acceleration_steps_s2 * abs(distance)) ** 0.5),
                    )
                else:
                    desired_speed = direction * self.max_speed_hz

        max_delta = self.acceleration_steps_s2 * elapsed_s
        self.current_speed_hz = self._approach(self.current_speed_hz, desired_speed, max_delta)

        self._step_accumulator += abs(self.current_speed_hz) * elapsed_s
        steps_due = int(self._step_accumulator)
        if steps_due <= 0:
            return 0

        remaining = abs(int(self.target_position_steps) - int(self.current_position_steps))
        if remaining <= 0:
            self._step_accumulator = 0.0
            return 0

        steps_to_take = min(steps_due, remaining, int(config.RUNTIME_MAX_CHUNK_STEPS))
        if steps_to_take <= 0:
            return 0

        effective_speed = int(max(config.RUNTIME_MIN_CHUNK_SPEED_HZ, min(abs(self.current_speed_hz), self.max_speed_hz)))
        moved = int(
            self.axis.move_fixed_steps_blocking(
                steps_to_take,
                direction,
                effective_speed,
                poll_ms=1,
            )
        )
        self.current_position_steps += moved if direction > 0 else -moved
        self._step_accumulator -= moved
        if moved < steps_to_take:
            self.current_speed_hz = 0.0
            self._step_accumulator = 0.0
        return moved


def build_driver():
    return TMC2209(
        uart_id=config.UART_ID,
        baudrate=config.UART_BAUDRATE,
        rx_pin=config.UART_RX_PIN,
        tx_pin=config.UART_TX_PIN,
        driver_address=config.TMC_ADDRESS,
        en_pin=config.EN_PIN,
        diag_pin=config.DIAG_PIN,
    )


def configure_driver(driver):
    if not driver.initialize(
        run_current=config.DEFAULT_RUN_CURRENT,
        hold_current=config.DEFAULT_HOLD_CURRENT,
        microsteps=config.MICROSTEP_MODE,
        hold_delay=config.CURRENT_HOLD_DELAY,
    ):
        return False
    return driver.set_driver_enabled_via_uart(True, fallback_toff=config.DRIVER_ENABLE_TOFF)


def build_axis(step_pin, dir_pin, axis_slot):
    axis_slot = max(0, int(axis_slot))
    return PIOStepper(
        step_pin=step_pin,
        dir_pin=dir_pin,
        step_sm_id=config.PIO_STEP_SM_ID + (axis_slot * 2),
        counter_sm_id=config.PIO_COUNTER_SM_ID + (axis_slot * 2),
        step_frequency=config.PIO_STEP_FREQUENCY,
        counter_frequency=config.PIO_COUNTER_FREQUENCY,
    )


def seek_endstop_uart(driver, axis, direction, speed_hz, label):
    timeout_ms = int((config.HOME_MAX_STEPS * 1000) / max(1.0, float(speed_hz))) + config.HOME_TIMEOUT_MARGIN_MS
    status = {
        "label": label,
        "direction": int(direction),
        "speed_hz": int(speed_hz),
        "search_steps": 0,
        "search_elapsed_ms": 0,
        "last_sg": None,
        "sg_history": [],
        "startup_sg_values": [],
        "startup_sg_samples": 0,
        "uart_threshold": None,
        "low_sg_peak": 0,
        "stop_reason": "not_started",
        "success": False,
    }
    last_status_ms = -config.PRINT_INTERVAL_MS
    low_sg_count = 0

    if not driver.set_coolstep_threshold(config.HOME_COOLSTEP_THRESHOLD):
        status["stop_reason"] = "coolstep_config_failed"
        return status
    driver.set_stallguard_threshold(0)

    def stop_fn(steps, elapsed_ms):
        nonlocal last_status_ms, low_sg_count

        sg = driver.read_stallguard_result()
        if sg is not None:
            sg = int(sg)
            status["last_sg"] = sg
            history = status["sg_history"]
            history.append(sg)
            if len(history) > 12:
                del history[0]

            if status["startup_sg_samples"] < config.HOME_STARTUP_SG_SAMPLES:
                status["startup_sg_samples"] += 1
                status["startup_sg_values"].append(sg)
                if status["startup_sg_samples"] == config.HOME_STARTUP_SG_SAMPLES:
                    status["uart_threshold"] = int(derive_uart_threshold(status["startup_sg_values"]))
                    debug_log(
                        "[seek:{}] armed uart threshold {} from startup median {}".format(
                            label,
                            status["uart_threshold"],
                            median_int(status["startup_sg_values"]),
                        )
                    )
            elif steps >= config.HOME_MIN_STALL_STEPS:
                threshold = int(status["uart_threshold"])
                if sg <= threshold:
                    low_sg_count += 1
                    if low_sg_count > status["low_sg_peak"]:
                        status["low_sg_peak"] = int(low_sg_count)
                else:
                    low_sg_count = 0

                if low_sg_count >= config.HOME_UART_CONFIRM_POLLS:
                    return "uart_stall"

        if elapsed_ms - last_status_ms >= config.PRINT_INTERVAL_MS:
            debug_log(
                "[seek:{}] elapsed={}ms steps={} sg={} low_hits={} thr={}".format(
                    label,
                    elapsed_ms,
                    steps,
                    status["last_sg"],
                    low_sg_count,
                    status["uart_threshold"],
                )
            )
            last_status_ms = elapsed_ms

        return None

    search = axis.run_until(
        direction=direction,
        speed_hz=speed_hz,
        max_steps=config.HOME_MAX_STEPS,
        stop_fn=stop_fn,
        poll_ms=config.HOME_POLL_MS,
        timeout_ms=timeout_ms,
    )

    status["search_steps"] = int(search["steps"])
    status["search_elapsed_ms"] = int(search["elapsed_ms"])
    status["stop_reason"] = search["stop_reason"]
    status["success"] = search["stop_reason"] == "uart_stall"

    debug_log(
        "[seek:{}] result success={} stop_reason={} steps={} elapsed={}ms".format(
            label,
            int(status["success"]),
            status["stop_reason"],
            status["search_steps"],
            status["search_elapsed_ms"],
        )
    )
    time.sleep_ms(config.HOME_SETTLE_MS)
    return status


def run_centering_trial(driver, step_pin, dir_pin, axis_slot, home_direction, speed_hz, trial_index):
    axis = build_axis(step_pin, dir_pin, axis_slot)
    status = {
        "trial_index": int(trial_index),
        "axis_slot": int(axis_slot),
        "step_pin": int(step_pin),
        "dir_pin": int(dir_pin),
        "home_direction": int(home_direction),
        "speed_hz": int(speed_hz),
        "retract_steps": 0,
        "release_steps": 0,
        "first_end": None,
        "second_end": None,
        "travel_steps": 0,
        "center_steps_requested": 0,
        "center_steps_moved": 0,
        "center_direction": int(home_direction),
        "centered": False,
        "stop_reason": "not_started",
        "success": False,
    }

    debug_log(
        "[trial] index={} slot={} step=GP{} dir=GP{} home_dir={} speed={}Hz".format(
            trial_index,
            axis_slot,
            step_pin,
            dir_pin,
            home_direction,
            speed_hz,
        )
    )

    try:
        if config.HOME_RETRACT_STEPS > 0:
            status["retract_steps"] = int(
                axis.move_fixed_steps_blocking(
                    config.HOME_RETRACT_STEPS,
                    -home_direction,
                    config.HOME_RETRACT_SPEED_HZ,
                    poll_ms=config.HOME_POLL_MS,
                )
            )
            debug_log("[trial] retract completed steps={}".format(status["retract_steps"]))
            time.sleep_ms(config.HOME_SETTLE_MS)

        first_end = seek_endstop_uart(driver, axis, home_direction, speed_hz, "first_end")
        status["first_end"] = first_end
        if not first_end["success"]:
            status["stop_reason"] = "first_end_" + first_end["stop_reason"]
            return status

        if config.HOME_RELEASE_STEPS > 0:
            status["release_steps"] = int(
                axis.move_fixed_steps_blocking(
                    config.HOME_RELEASE_STEPS,
                    -home_direction,
                    config.HOME_RELEASE_SPEED_HZ,
                    poll_ms=config.HOME_POLL_MS,
                )
            )
            debug_log("[trial] release completed steps={}".format(status["release_steps"]))
            time.sleep_ms(config.HOME_SETTLE_MS)

        second_end = seek_endstop_uart(driver, axis, -home_direction, speed_hz, "second_end")
        status["second_end"] = second_end
        if not second_end["success"]:
            status["stop_reason"] = "second_end_" + second_end["stop_reason"]
            return status

        status["travel_steps"] = int(status["release_steps"] + second_end["search_steps"])
        if status["travel_steps"] < int(config.HOME_MIN_TRAVEL_STEPS):
            status["stop_reason"] = "measured_span_too_small"
            debug_log(
                "[trial] rejecting span={} steps because it is below minimum {}".format(
                    status["travel_steps"],
                    config.HOME_MIN_TRAVEL_STEPS,
                )
            )
            return status

        status["center_steps_requested"] = int(status["travel_steps"] // 2)
        status["center_steps_moved"] = int(
            axis.move_fixed_steps_blocking(
                status["center_steps_requested"],
                home_direction,
                config.HOME_RELEASE_SPEED_HZ,
                poll_ms=config.HOME_POLL_MS,
            )
        )
        status["centered"] = status["center_steps_moved"] >= status["center_steps_requested"]
        status["success"] = bool(status["centered"])
        status["stop_reason"] = "uart_centered" if status["success"] else "center_move_incomplete"
        time.sleep_ms(config.HOME_SETTLE_MS)

        debug_log(
            "[trial] travel={} center_requested={} center_moved={} success={}".format(
                status["travel_steps"],
                status["center_steps_requested"],
                status["center_steps_moved"],
                int(status["success"]),
            )
        )
        return status
    finally:
        axis.deinit()


def run_homing(driver, result):
    trial_index = 0
    speed_trials = home_speed_trials()

    for axis_slot, (step_pin, dir_pin) in enumerate(config.STEP_DIR_TRIALS):
        for home_direction in config.HOME_DIRECTION_TRIALS:
            for speed_hz in speed_trials:
                trial = run_centering_trial(
                    driver,
                    step_pin,
                    dir_pin,
                    axis_slot,
                    home_direction,
                    speed_hz,
                    trial_index,
                )
                result["trials"].append(trial)
                write_json(config.RESULT_FILE, result)
                if trial["success"]:
                    result["success"] = True
                    result["centered"] = True
                    result["selected_trial"] = int(trial_index)
                    result["selected_axis_slot"] = int(axis_slot)
                    result["stop_reason"] = trial["stop_reason"]
                    return trial
                trial_index += 1

    return None


def dmx_worker(shared):
    dmx = DMXReceiver(pin_num=config.DMX_PIN, sm_id=config.DMX_SM_ID)
    dmx.start()
    while True:
        frame_received = dmx.read_frame()
        if not frame_received:
            continue
        if dmx.last_start_code != 0x00:
            continue
        channels = dmx.get_channels(config.DMX_START_CHANNEL, config.DMX_CHANNEL_COUNT)
        shared.update_from_channels(
            channels,
            dmx.get_frame_count(),
            time.ticks_ms(),
            dmx.start_code_errors,
        )


def main():
    result = {
        "status": "running",
        "success": False,
        "centered": False,
        "runtime_ready": False,
        "board": board_name(),
        "microsteps": int(config.MICROSTEP_MODE),
        "run_current": int(config.DEFAULT_RUN_CURRENT),
        "hold_current": int(config.DEFAULT_HOLD_CURRENT),
        "step_dir_trials": [list(pair) for pair in config.STEP_DIR_TRIALS],
        "home_direction_trials": [int(value) for value in config.HOME_DIRECTION_TRIALS],
        "home_speed_trials_hz": list(home_speed_trials()),
        "trials": [],
        "selected_trial": None,
        "selected_axis_slot": None,
        "result_file": config.RESULT_FILE,
        "status_file": config.STATUS_FILE,
    }
    write_json(config.RESULT_FILE, result)

    driver = build_driver()
    runtime_axis = None
    controller = None

    debug_log("=" * 72)
    debug_log("UART HOMING + ONE-AXIS DMX RUNTIME")
    debug_log("board={}".format(result["board"]))
    debug_log(
        "uart0 tx=GP{} rx=GP{} dmx=GP{} sm={} microsteps={} run={} hold={} en=UART-only".format(
            config.UART_TX_PIN,
            config.UART_RX_PIN,
            config.DMX_PIN,
            config.DMX_SM_ID,
            config.MICROSTEP_MODE,
            config.DEFAULT_RUN_CURRENT,
            config.DEFAULT_HOLD_CURRENT,
        )
    )
    debug_log("=" * 72)

    try:
        if not configure_driver(driver):
            result["status"] = "done"
            result["stop_reason"] = "driver_init_failed"
            write_json(config.RESULT_FILE, result)
            debug_log("[error] TMC2209 initialization or UART enable failed")
            driver.set_driver_enabled_via_uart(False, fallback_toff=config.DRIVER_ENABLE_TOFF)
            return

        homing_trial = run_homing(driver, result)
        result["status"] = "done"
        if homing_trial is None:
            result["stop_reason"] = "all_trials_failed"
            write_json(config.RESULT_FILE, result)
            driver.set_driver_enabled_via_uart(False, fallback_toff=config.DRIVER_ENABLE_TOFF)
            debug_log("[result] homing failed on all trials")
            return

        result["runtime_ready"] = True
        write_json(config.RESULT_FILE, result)

        if not config.RUN_RUNTIME_AFTER_HOMING:
            debug_log("[result] homing complete; runtime disabled by configuration")
            return

        runtime_axis = build_axis(
            homing_trial["step_pin"],
            homing_trial["dir_pin"],
            homing_trial["axis_slot"],
        )
        controller = ChunkedPositionController(runtime_axis, homing_trial["travel_steps"])

        shared = SharedDMXState()
        _thread.start_new_thread(dmx_worker, (shared,))

        last_status_ms = -config.STATUS_INTERVAL_MS
        last_print_ms = -config.PRINT_INTERVAL_MS
        last_enabled = True
        last_currents = None
        runtime_start_ms = time.ticks_ms()

        debug_log(
            "[runtime] selected_trial={} step=GP{} dir=GP{} home_dir={} travel_steps={}".format(
                result["selected_trial"],
                homing_trial["step_pin"],
                homing_trial["dir_pin"],
                homing_trial["home_direction"],
                homing_trial["travel_steps"],
            )
        )

        while True:
            now_ms = time.ticks_ms()
            snapshot = shared.snapshot(now_ms)
            controller.apply_snapshot(snapshot)

            desired_enabled = bool(snapshot["enabled"])
            if desired_enabled != last_enabled:
                driver.set_driver_enabled_via_uart(desired_enabled, fallback_toff=config.DRIVER_ENABLE_TOFF)
                if not desired_enabled:
                    controller.hold_position()
                last_enabled = desired_enabled

            applied_run_current = int(snapshot["run_current"])
            applied_hold_current = int(snapshot["hold_current"])
            if not snapshot["signal_present"]:
                applied_run_current = int(snapshot["hold_current"])
                applied_hold_current = int(snapshot["hold_current"])

            current_signature = (applied_hold_current, applied_run_current)
            if current_signature != last_currents:
                driver.set_run_hold_current(
                    run_current=applied_run_current,
                    hold_current=applied_hold_current,
                    hold_delay=config.CURRENT_HOLD_DELAY,
                )
                last_currents = current_signature

            moved = controller.update()

            if time.ticks_diff(now_ms, last_status_ms) >= config.STATUS_INTERVAL_MS:
                status_payload = {
                    "runtime_active": True,
                    "board": result["board"],
                    "selected_trial": result["selected_trial"],
                    "selected_axis_slot": result["selected_axis_slot"],
                    "step_pin": homing_trial["step_pin"],
                    "dir_pin": homing_trial["dir_pin"],
                    "home_direction": homing_trial["home_direction"],
                    "travel_steps": homing_trial["travel_steps"],
                    "current_position_steps": int(controller.current_position_steps),
                    "target_position_steps": int(controller.target_position_steps),
                    "current_speed_hz": round(float(controller.current_speed_hz), 3),
                    "enabled": bool(snapshot["enabled"]),
                    "signal_present": bool(snapshot["signal_present"]),
                    "dmx_frame_count": int(snapshot["frame_count"]),
                    "dmx_last_frame_age_ms": snapshot["age_ms"],
                    "dmx_start_code_errors": int(snapshot["start_code_errors"]),
                    "last_channels": list(snapshot["last_channels"]),
                    "run_current": int(snapshot["run_current"]),
                    "hold_current": int(snapshot["hold_current"]),
                    "max_speed_hz": int(snapshot["max_speed_hz"]),
                    "acceleration_steps_s2": int(snapshot["acceleration_steps_s2"]),
                    "moved_last_update": int(moved),
                }
                write_json(config.STATUS_FILE, status_payload)
                last_status_ms = now_ms

            if time.ticks_diff(now_ms, last_print_ms) >= config.PRINT_INTERVAL_MS:
                debug_log(
                    "[runtime] frames={} signal={} en={} pos={}/{} speed={:.1f} moved={}".format(
                        snapshot["frame_count"],
                        int(snapshot["signal_present"]),
                        int(snapshot["enabled"]),
                        controller.current_position_steps,
                        controller.target_position_steps,
                        controller.current_speed_hz,
                        moved,
                    )
                )
                last_print_ms = now_ms

            if (
                int(config.RUNTIME_EXIT_AFTER_MS) > 0
                and time.ticks_diff(now_ms, runtime_start_ms) >= int(config.RUNTIME_EXIT_AFTER_MS)
            ):
                debug_log("[runtime] exiting after configured runtime window")
                return

            time.sleep_ms(config.RUNTIME_CONTROL_SLEEP_MS)
    finally:
        if runtime_axis is not None:
            runtime_axis.deinit()


if __name__ == "__main__":
    main()
