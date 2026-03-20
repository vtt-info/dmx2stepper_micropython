"""PIO-based DMX512 receiver for RP2040 MicroPython."""

import array
import time

import rp2
from machine import Pin, mem32


DMX_MAX_CHANNELS = 512
DMX_START_CODE = 0x00
GPIO_IN = 0xD0000004
FRAME_GAP_TIMEOUT_US = 5000


@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_RIGHT, autopush=True, push_thresh=8, fifo_join=rp2.PIO.JOIN_RX)
def dmx_rx():
    wait(0, pin, 0)
    set(x, 7) [10]
    label("bitloop")
    in_(pins, 1)
    jmp(x_dec, "bitloop") [6]


class DMXReceiver:
    """Read complete DMX frames using one PIO state machine."""

    def __init__(self, pin_num, sm_id=0):
        self.pin_num = pin_num
        self.sm = rp2.StateMachine(sm_id, dmx_rx, freq=2_000_000, in_base=Pin(pin_num, Pin.IN, Pin.PULL_UP))
        self.frame_buffer = array.array("B", [0] * (DMX_MAX_CHANNELS + 1))
        self.frame_count = 0
        self.last_frame_time = 0
        self.start_code_errors = 0
        self.last_start_code = 0
        self.last_bytes_received = 0
        self.receiving = False

    def _pin_value(self):
        return (mem32[GPIO_IN] >> self.pin_num) & 1

    def start(self):
        self.receiving = True
        self.sm.active(1)
        while self.sm.rx_fifo() > 0:
            self.sm.get()

    def stop(self):
        self.receiving = False
        self.sm.active(0)

    def _wait_for_break(self, timeout_ms=100):
        deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            if self._pin_value() == 0:
                start = time.ticks_us()
                while self._pin_value() == 0:
                    if time.ticks_diff(time.ticks_us(), start) > 200:
                        while self._pin_value() == 0:
                            pass
                        return True
                duration = time.ticks_diff(time.ticks_us(), start)
                if duration > 44:
                    return True
        return False

    def read_frame(self):
        if not self.receiving:
            return False

        if not self._wait_for_break():
            return False

        while self.sm.rx_fifo() > 0:
            self.sm.get()

        time.sleep_us(50)

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
                time.sleep_us(10)

        if bytes_received == 0:
            return False

        self.last_start_code = self.frame_buffer[0]
        if self.last_start_code != DMX_START_CODE:
            self.start_code_errors += 1

        self.last_bytes_received = bytes_received
        self.frame_count += 1
        self.last_frame_time = time.ticks_ms()
        return True

    def get_channel(self, channel):
        if 1 <= channel <= DMX_MAX_CHANNELS:
            return self.frame_buffer[channel]
        return 0

    def get_channels(self, start_channel, num_channels):
        values = []
        for offset in range(num_channels):
            channel = start_channel + offset
            values.append(self.get_channel(channel))
        return values

    def get_frame_count(self):
        return self.frame_count

    def get_errors(self):
        return {"start_code": self.start_code_errors}

    def reset_errors(self):
        self.start_code_errors = 0
