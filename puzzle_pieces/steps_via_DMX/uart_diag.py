"""
TMC2209 UART diagnostics for MicroPython.

- Sends direct read requests for key registers
- Prints raw reply bytes (hex) so parser/address issues are visible
- Runs a small write test and checks IFCNT if readable
"""

import time
from machine import UART, Pin
import config


class TMC2209Diag:
    def __init__(self, uart, motor_id=0):
        self.uart = uart
        self.motor_id = motor_id

    def crc8(self, data):
        crc = 0
        for byte in data:
            for _ in range(8):
                if (crc >> 7) ^ (byte & 0x01):
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
                byte >>= 1
        return crc

    def drain(self):
        while self.uart.any():
            self.uart.read()

    def send_read(self, reg):
        cmd = bytearray(4)
        cmd[0] = 0x55
        cmd[1] = self.motor_id
        cmd[2] = reg & 0x7F
        cmd[3] = self.crc8(cmd[:3])
        self.uart.write(cmd)
        return cmd

    def send_write(self, reg, value):
        cmd = bytearray(8)
        cmd[0] = 0x55
        cmd[1] = self.motor_id
        cmd[2] = (reg & 0x7F) | 0x80
        cmd[3] = (value >> 24) & 0xFF
        cmd[4] = (value >> 16) & 0xFF
        cmd[5] = (value >> 8) & 0xFF
        cmd[6] = value & 0xFF
        cmd[7] = self.crc8(cmd[:7])
        self.uart.write(cmd)
        return cmd

    def read_raw(self, reg, wait_ms=15):
        self.drain()
        tx = self.send_read(reg)
        time.sleep_ms(wait_ms)
        rx = self.uart.read() if self.uart.any() else None
        return tx, rx


REGS = [
    (0x00, "GCONF"),
    (0x01, "GSTAT"),
    (0x02, "IFCNT"),
    (0x06, "IOIN"),
    (0x10, "IHOLD_IRUN"),
    (0x22, "VACTUAL"),
    (0x6C, "CHOPCONF"),
    (0x6F, "DRV_STATUS"),
]


def hx(buf):
    if not buf:
        return "<no response>"
    return " ".join("{:02X}".format(b) for b in buf)


def main():
    print("=" * 60)
    print("TMC2209 UART DIAG")
    print("UART{} TX=GP{} RX=GP{} BAUD={} MOTOR_ID={}".format(
        config.TMC_UART_ID,
        config.TMC_UART_TX_PIN,
        config.TMC_UART_RX_PIN,
        config.TMC_UART_BAUD,
        config.TMC_MOTOR_ID,
    ))
    print("=" * 60)

    uart = UART(
        config.TMC_UART_ID,
        baudrate=config.TMC_UART_BAUD,
        tx=Pin(config.TMC_UART_TX_PIN),
        rx=Pin(config.TMC_UART_RX_PIN),
    )
    time.sleep_ms(100)

    d = TMC2209Diag(uart, motor_id=config.TMC_MOTOR_ID)

    # Baseline register scan
    print("\n[1] Baseline register reads (raw):")
    for reg, name in REGS:
        tx, rx = d.read_raw(reg)
        print("{:10s} reg 0x{:02X} | TX: {} | RX: {}".format(name, reg, hx(tx), hx(rx)))

    # IFCNT write-ack test
    print("\n[2] IFCNT write-ack test:")
    _, ifcnt_before = d.read_raw(0x02)
    print("IFCNT before: {}".format(hx(ifcnt_before)))

    write_cmd = d.send_write(0x10, 0x00141414)  # IHOLD_IRUN example payload
    time.sleep_ms(15)
    write_echo = uart.read() if uart.any() else None
    print("W IHOLD_IRUN TX: {} | RX-after-write: {}".format(hx(write_cmd), hx(write_echo)))

    _, ifcnt_after = d.read_raw(0x02)
    print("IFCNT after : {}".format(hx(ifcnt_after)))

    # Multi-id probe if no reads from configured id
    print("\n[3] Motor-ID probe (read IFCNT with IDs 0..3):")
    for probe_id in range(4):
        p = TMC2209Diag(uart, motor_id=probe_id)
        tx, rx = p.read_raw(0x02)
        print("ID {} | TX: {} | RX: {}".format(probe_id, hx(tx), hx(rx)))

    print("\nDone.")


if __name__ == "__main__":
    main()
