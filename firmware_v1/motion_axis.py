"""Single-axis stepper motion helper for MicroPython."""

import time

from machine import Pin


class MotionAxis:
    """Acceleration-limited single-axis stepper movement with blocking homing."""

    def __init__(self, step_pin, dir_pin, step_pulse_us=4):
        self.step_pin = Pin(step_pin, Pin.OUT, value=0)
        self.dir_pin = Pin(dir_pin, Pin.OUT, value=0)
        self.step_pulse_us = max(2, int(step_pulse_us))

        self.current_position_steps = 0
        self.target_position_steps = 0
        self.max_speed_hz = 100.0
        self.acceleration_steps_s2 = 500.0
        self.current_speed_hz = 0.0
        self.enabled = False
        self.homed = False

        self._last_update_us = time.ticks_us()
        self._step_accumulator = 0.0

    def set_enabled(self, enabled):
        self.enabled = bool(enabled)
        if not self.enabled:
            self.current_speed_hz = 0.0
            self._step_accumulator = 0.0
        self._last_update_us = time.ticks_us()

    def set_motion_limits(self, max_speed_hz, acceleration_steps_s2):
        self.max_speed_hz = max(1.0, float(max_speed_hz))
        self.acceleration_steps_s2 = max(1.0, float(acceleration_steps_s2))

    def set_target_steps(self, target_steps):
        self.target_position_steps = int(target_steps)

    def hold_position(self):
        self.target_position_steps = self.current_position_steps

    def snap_to(self, position_steps):
        position_steps = int(position_steps)
        self.current_position_steps = position_steps
        self.target_position_steps = position_steps
        self.current_speed_hz = 0.0
        self._step_accumulator = 0.0
        self._last_update_us = time.ticks_us()
        self.homed = True

    def _set_direction(self, direction):
        self.dir_pin.value(1 if direction > 0 else 0)

    def _pulse_step(self, direction):
        self._set_direction(direction)
        self.step_pin.value(1)
        time.sleep_us(self.step_pulse_us)
        self.step_pin.value(0)
        self.current_position_steps += 1 if direction > 0 else -1

    def _approach(self, current, target, delta):
        if current < target:
            return min(target, current + delta)
        if current > target:
            return max(target, current - delta)
        return target

    def move_fixed_steps_blocking(self, steps, direction, speed_hz):
        steps = max(0, int(steps))
        if steps == 0:
            return 0

        interval_us = max(self.step_pulse_us + 2, int(1_000_000 / max(1.0, float(speed_hz))))
        next_step_us = time.ticks_us()
        moved = 0

        while moved < steps:
            now_us = time.ticks_us()
            remaining = time.ticks_diff(next_step_us, now_us)
            if remaining > 0:
                time.sleep_us(min(remaining, 200))
                continue

            self._pulse_step(direction)
            moved += 1
            next_step_us = time.ticks_add(next_step_us, interval_us)

        self.current_speed_hz = 0.0
        self._step_accumulator = 0.0
        self._last_update_us = time.ticks_us()
        return moved

    def home(
        self,
        driver,
        home_direction,
        speed_hz,
        max_steps,
        stall_threshold,
        coolstep_threshold,
        release_steps=0,
        release_speed_hz=None,
        settle_ms=100,
        home_position_steps=0,
    ):
        home_direction = -1 if home_direction < 0 else 1
        driver.set_enabled(True)
        self.set_enabled(True)
        self.homed = False
        self.current_speed_hz = 0.0
        self._step_accumulator = 0.0
        self._last_update_us = time.ticks_us()

        if not driver.set_coolstep_threshold(coolstep_threshold):
            return False
        if not driver.set_stallguard_threshold(stall_threshold):
            return False

        interval_us = max(self.step_pulse_us + 2, int(1_000_000 / max(1.0, float(speed_hz))))
        next_step_us = time.ticks_us()
        moved = 0

        while moved < int(max_steps):
            if driver.diag_triggered():
                break

            now_us = time.ticks_us()
            remaining = time.ticks_diff(next_step_us, now_us)
            if remaining > 0:
                time.sleep_us(min(remaining, 200))
                continue

            self._pulse_step(home_direction)
            moved += 1
            next_step_us = time.ticks_add(next_step_us, interval_us)

        success = driver.diag_triggered()
        driver.set_stallguard_threshold(0)
        time.sleep_ms(settle_ms)

        if success and release_steps > 0:
            release_direction = -home_direction
            self.move_fixed_steps_blocking(release_steps, release_direction, release_speed_hz or speed_hz / 2)

        if success:
            self.snap_to(home_position_steps)

        return success

    def update(self, now_us=None):
        if now_us is None:
            now_us = time.ticks_us()

        elapsed_us = time.ticks_diff(now_us, self._last_update_us)
        if elapsed_us <= 0:
            return
        self._last_update_us = now_us

        if not self.enabled or not self.homed:
            self.current_speed_hz = 0.0
            self._step_accumulator = 0.0
            return

        distance = self.target_position_steps - self.current_position_steps
        if distance == 0 and abs(self.current_speed_hz) < 0.01:
            self.current_speed_hz = 0.0
            self._step_accumulator = 0.0
            return

        elapsed_s = elapsed_us / 1_000_000.0
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
            return

        steps_to_take = min(steps_due, abs(self.target_position_steps - self.current_position_steps))
        for _ in range(steps_to_take):
            if self.target_position_steps == self.current_position_steps:
                break
            direction = 1 if self.target_position_steps > self.current_position_steps else -1
            self._pulse_step(direction)

        self._step_accumulator -= steps_to_take
        if self.target_position_steps == self.current_position_steps and abs(self.current_speed_hz) < 1.0:
            self.current_speed_hz = 0.0
            self._step_accumulator = 0.0
