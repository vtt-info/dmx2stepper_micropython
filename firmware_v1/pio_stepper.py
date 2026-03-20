"""PIO-based step generation and pulse counting for RP2040 MicroPython."""

import time

import rp2
from machine import Pin


@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def step_count_pio():
    """Generate exactly N step pulses then idle.

    TX FIFO protocol: push step_count-1 first, then push delay value.
    The SM generates step_count pulses and blocks on pull(block) until the
    next command.
    """
    wrap_target()
    pull(block)          # wait for step count -> X
    mov(x, osr)
    pull(block)          # wait for delay value -> stays in OSR
    label("step")
    mov(y, osr)          # y = delay
    set(pins, 1) [15]    # pulse high for 32 PIO cycles
    set(pins, 1) [15]
    label("delay")
    set(pins, 0)         # pin low
    jmp(y_dec, "delay")  # delay loop
    jmp(x_dec, "step")   # next pulse, or fall through when done
    wrap()               # back to pull(block) — idles until next command


@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def step_freerun_pio():
    """Free-running step generation for homing / run_until.

    Push a delay value to control speed.  Runs until SM is deactivated.
    Uses pull(noblock) so speed can be updated on the fly.
    """
    wrap_target()
    pull(noblock)
    mov(x, osr)
    mov(y, x)
    set(pins, 1) [15]
    set(pins, 1) [15]
    label("delay")
    set(pins, 0)
    jmp(y_dec, "delay")
    wrap()


@rp2.asm_pio()
def step_counter_pio():
    label("loop")
    wait(0, pin, 0)
    wait(1, pin, 0)
    jmp(x_dec, "loop")


class PIOStepper:
    """Generate step pulses with PIO and track emitted steps with a second state machine."""

    PIO_VAR = 2
    PIO_FIX = 37

    def __init__(
        self,
        step_pin,
        dir_pin,
        step_sm_id=0,
        counter_sm_id=1,
        step_frequency=5_000_000,
        counter_frequency=125_000_000,
    ):
        self.step_pin_num = int(step_pin)
        self.dir_pin_num = int(dir_pin)
        self.step_pin = Pin(self.step_pin_num, Pin.OUT, value=0, pull=Pin.PULL_DOWN)
        self.dir_pin = Pin(self.dir_pin_num, Pin.OUT, value=0)
        self.step_frequency = int(step_frequency)
        self.counter_frequency = int(counter_frequency)
        self._running = False
        self._freerun_mode = False

        self._pull_encoded = rp2.asm_pio_encode("pull()", 0)
        self._mov_x_osr_encoded = rp2.asm_pio_encode("mov(x, osr)", 0)
        self._mov_isr_x_encoded = rp2.asm_pio_encode("mov(isr, x)", 0)
        self._push_encoded = rp2.asm_pio_encode("push()", 0)

        self.step_sm_id = int(step_sm_id)
        self._init_step_sm_counted()

        self.counter_sm = rp2.StateMachine(
            int(counter_sm_id),
            step_counter_pio,
            freq=self.counter_frequency,
            in_base=self.step_pin,
        )

        self.step_sm.active(0)
        self.counter_sm.active(0)

    def _init_step_sm_counted(self):
        """Initialize step SM with count-limited program."""
        self.step_sm = rp2.StateMachine(
            self.step_sm_id,
            step_count_pio,
            freq=self.step_frequency,
            set_base=self.step_pin,
        )
        self._freerun_mode = False

    def _init_step_sm_freerun(self):
        """Initialize step SM with free-running program."""
        self.step_sm = rp2.StateMachine(
            self.step_sm_id,
            step_freerun_pio,
            freq=self.step_frequency,
            set_base=self.step_pin,
        )
        self.step_sm.put(65535)  # initial high delay = stopped
        self._freerun_mode = True

    def _drain_rx_fifo(self):
        while self.counter_sm.rx_fifo() > 0:
            self.counter_sm.get()

    def set_direction(self, direction):
        self.dir_pin.value(1 if int(direction) > 0 else 0)

    def speed_to_delay(self, speed_hz):
        speed_hz = max(1.0, float(speed_hz))
        delay = int((self.step_frequency - (speed_hz * self.PIO_FIX)) / (speed_hz * self.PIO_VAR))
        return max(1, delay)

    def reset_counter(self):
        self._drain_rx_fifo()
        self.counter_sm.put(0)
        self.counter_sm.exec(self._pull_encoded)
        self.counter_sm.exec(self._mov_x_osr_encoded)
        self._drain_rx_fifo()

    def read_counter(self):
        self.counter_sm.exec(self._mov_isr_x_encoded)
        self.counter_sm.exec(self._push_encoded)
        if self.counter_sm.rx_fifo() <= 0:
            return 0
        raw = self.counter_sm.get()
        return (-raw) & 0xFFFFFFFF

    def start(self, direction, speed_hz):
        """Start free-running step generation (for homing / run_until)."""
        self.stop()
        if not self._freerun_mode:
            self._init_step_sm_freerun()
        self.set_direction(direction)
        self.reset_counter()
        self.counter_sm.active(1)
        self.step_sm.put(self.speed_to_delay(speed_hz))
        self.step_sm.active(1)
        self._running = True

    def stop(self):
        self.step_sm.active(0)
        self.step_pin.value(0)
        self.counter_sm.active(0)
        self._running = False

    def _stop_and_read(self):
        """Stop step generation, read actual steps, then stop counter."""
        self.step_sm.active(0)
        self.step_pin.value(0)
        actual = self.read_counter()
        self.counter_sm.active(0)
        self._running = False
        return actual

    def move_fixed_steps_blocking(self, steps, direction, speed_hz, poll_ms=1):
        """Move exactly `steps` pulses using count-limited PIO. Returns steps."""
        steps = max(0, int(steps))
        if steps == 0:
            return 0

        if self._freerun_mode:
            self.stop()
            self._init_step_sm_counted()

        self.set_direction(direction)
        delay = self.speed_to_delay(speed_hz)

        # Calculate expected completion time
        cycles_per_step = self.PIO_FIX + delay * self.PIO_VAR
        total_us = int(steps * cycles_per_step * 1_000_000 / self.step_frequency)
        wait_ms = max(1, total_us // 1000) + 2  # +2ms margin

        # Push count and delay — PIO generates exactly `steps` pulses
        self.step_sm.put(steps - 1)
        self.step_sm.put(delay)
        self.step_sm.active(1)
        self._running = True

        time.sleep_ms(wait_ms)

        self.step_sm.active(0)
        self.step_pin.value(0)
        self._running = False
        return steps

    def run_until(self, direction, speed_hz, max_steps, stop_fn, poll_ms=2, timeout_ms=None):
        max_steps = max(1, int(max_steps))
        timeout_ms = None if timeout_ms is None else max(1, int(timeout_ms))
        self.start(direction, speed_hz)
        start_ms = time.ticks_ms()

        try:
            while True:
                moved = self.read_counter()
                if moved >= max_steps:
                    return {
                        "steps": self._stop_and_read(),
                        "elapsed_ms": time.ticks_diff(time.ticks_ms(), start_ms),
                        "stop_reason": "max_steps",
                    }

                reason = stop_fn(moved, time.ticks_diff(time.ticks_ms(), start_ms))
                if reason:
                    return {
                        "steps": self._stop_and_read(),
                        "elapsed_ms": time.ticks_diff(time.ticks_ms(), start_ms),
                        "stop_reason": reason,
                    }

                if timeout_ms is not None and time.ticks_diff(time.ticks_ms(), start_ms) >= timeout_ms:
                    return {
                        "steps": self._stop_and_read(),
                        "elapsed_ms": time.ticks_diff(time.ticks_ms(), start_ms),
                        "stop_reason": "timeout",
                    }

                time.sleep_ms(max(1, int(poll_ms)))
        finally:
            self.stop()

    def deinit(self):
        self.stop()
        self.counter_sm.active(0)
