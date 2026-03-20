"""Low-level UART transport for a TMC2209 driver."""

import struct
import time

from machine import UART


class TMC2209UART:
    """Read and write TMC2209 registers over UART."""

    def __init__(self, uart_id, baudrate, rx_pin, tx_pin, driver_address):
        self._uart = UART(uart_id, baudrate=baudrate, bits=8, parity=None, stop=1, tx=tx_pin, rx=rx_pin)
        self._driver_address = driver_address
        self._ifcnt_reg = 0x02
        self._read_frame = [0x55, 0, 0, 0]
        self._write_frame = [0x55, 0, 0, 0, 0, 0, 0, 0]
        self._pause_s = 500 / baudrate

    def close(self):
        if hasattr(self._uart, "deinit"):
            self._uart.deinit()

    def compute_crc8_atm(self, datagram, initial_value=0):
        crc = initial_value
        for byte in datagram:
            for _ in range(8):
                if (crc >> 7) ^ (byte & 0x01):
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
                byte >>= 1
        return crc

    def read_reg(self, reg):
        self._read_frame[1] = self._driver_address
        self._read_frame[2] = reg
        self._read_frame[3] = self.compute_crc8_atm(self._read_frame[:-1])

        if self._uart.write(bytes(self._read_frame)) != len(self._read_frame):
            return b""

        time.sleep(self._pause_s)
        raw = self._uart.read() if self._uart.any() else None
        time.sleep(self._pause_s)

        if raw is None or len(raw) < 11:
            return b""
        return raw[7:11]

    def read_int(self, reg):
        for _ in range(10):
            raw = self.read_reg(reg)
            if len(raw) >= 4:
                return struct.unpack(">I", raw)[0]
        return None

    def write_reg(self, reg, value):
        self._write_frame[1] = self._driver_address
        self._write_frame[2] = reg | 0x80
        self._write_frame[3] = (value >> 24) & 0xFF
        self._write_frame[4] = (value >> 16) & 0xFF
        self._write_frame[5] = (value >> 8) & 0xFF
        self._write_frame[6] = value & 0xFF
        self._write_frame[7] = self.compute_crc8_atm(self._write_frame[:-1])

        if self._uart.write(bytes(self._write_frame)) != len(self._write_frame):
            return False

        time.sleep(self._pause_s)
        return True

    def write_reg_check(self, reg, value):
        before = self.read_int(self._ifcnt_reg)
        if before is None:
            return False
        if not self.write_reg(reg, value):
            return False
        after = self.read_int(self._ifcnt_reg)
        if after is None:
            return False
        return after > before

    def test(self):
        return self.read_int(0x06) is not None
