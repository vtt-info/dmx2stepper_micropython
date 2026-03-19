"""
DMX512 Receiver using PIO + GPIO Break Detection
Uses RP2040 PIO state machine for byte reception at 250kbaud
and direct GPIO register reads for break detection.
"""

import rp2
from machine import Pin, mem32
import array
import time

DMX_MAX_CHANNELS = 512
DMX_START_CODE = 0x00
GPIO_IN = 0xd0000004  # RP2040 GPIO input register


@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_RIGHT, autopush=True,
             push_thresh=8, fifo_join=rp2.PIO.JOIN_RX)
def dmx_rx():
    wait(0, pin, 0)           # Wait for start bit
    set(x, 7)        [10]     # 11 cycles to center of bit 0
    label("bitloop")
    in_(pins, 1)              # Sample bit (1 cycle)
    jmp(x_dec, "bitloop") [6] # 8 cycles per bit total


class DMXReceiver:
    """DMX512 Receiver using PIO with break detection."""

    def __init__(self, pin_num, sm_id=0):
        self.pin_num = pin_num
        self.pin_mask = 1 << pin_num

        # PIO state machine for byte reception
        self.sm = rp2.StateMachine(sm_id, dmx_rx, freq=2_000_000,
                                   in_base=Pin(pin_num, Pin.IN, Pin.PULL_UP))

        # Frame buffer: start code + 512 channels
        self.frame_buffer = array.array('B', [0] * (DMX_MAX_CHANNELS + 1))

        # Statistics
        self.frame_count = 0
        self.last_frame_time = 0
        self.start_code_errors = 0
        self.last_start_code = 0
        self.last_bytes_received = 0

        self.receiving = False

    def _pin_value(self):
        """Read pin state directly from GPIO register."""
        return (mem32[GPIO_IN] >> self.pin_num) & 1

    def start(self):
        """Start the receiver."""
        self.receiving = True
        self.sm.active(1)
        # Drain any stale data
        while self.sm.rx_fifo() > 0:
            self.sm.get()

    def stop(self):
        """Stop the receiver."""
        self.receiving = False
        self.sm.active(0)

    def _wait_for_break(self, timeout_ms=100):
        """Detect DMX break: line LOW for >44us (longer than a byte)."""
        deadline = time.ticks_add(time.ticks_ms(), timeout_ms)

        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            if self._pin_value() == 0:
                start = time.ticks_us()
                while self._pin_value() == 0:
                    if time.ticks_diff(time.ticks_us(), start) > 200:
                        # Definitely a break. Wait for MAB (line goes HIGH).
                        while self._pin_value() == 0:
                            pass
                        return True
                duration = time.ticks_diff(time.ticks_us(), start)
                if duration > 44:
                    return True
        return False

    def read_frame(self):
        """
        Read a complete DMX frame (blocking).
        Waits for break, then reads all bytes until inter-frame gap.

        Returns:
            True if a valid frame was received.
        """
        if not self.receiving:
            return False

        if not self._wait_for_break():
            return False

        # Drain stale PIO data from break-period garbage bytes
        while self.sm.rx_fifo() > 0:
            self.sm.get()

        # Wait for MAB to complete and first byte to arrive
        time.sleep_us(50)

        # Read frame bytes
        bytes_received = 0
        timeout_start = time.ticks_us()

        while bytes_received <= DMX_MAX_CHANNELS:
            if self.sm.rx_fifo() > 0:
                self.frame_buffer[bytes_received] = (self.sm.get() >> 24) & 0xFF
                bytes_received += 1
                timeout_start = time.ticks_us()
            else:
                # No data for >1ms = frame complete (inter-frame gap)
                if time.ticks_diff(time.ticks_us(), timeout_start) > 1000:
                    break
                time.sleep_us(10)

        if bytes_received == 0:
            return False

        # Track start code
        self.last_start_code = self.frame_buffer[0]
        if self.last_start_code != DMX_START_CODE:
            self.start_code_errors += 1

        self.last_bytes_received = bytes_received
        self.frame_count += 1
        self.last_frame_time = time.ticks_ms()
        return True

    def get_channel(self, channel):
        """Get DMX channel value (1-512)."""
        if channel < 1 or channel > DMX_MAX_CHANNELS:
            return 0
        return self.frame_buffer[channel]

    def get_channels(self, start_channel, num_channels):
        """Get multiple consecutive channel values."""
        values = []
        for i in range(num_channels):
            ch = start_channel + i
            if 1 <= ch <= DMX_MAX_CHANNELS:
                values.append(self.frame_buffer[ch])
            else:
                values.append(0)
        return values

    def get_frame_count(self):
        return self.frame_count

    def get_last_frame_time(self):
        return self.last_frame_time

    def get_errors(self):
        return {'start_code': self.start_code_errors}

    def reset_errors(self):
        self.start_code_errors = 0
