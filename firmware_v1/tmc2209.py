"""Focused TMC2209 driver wrapper for enable, current, microstep, and stallguard settings."""

from machine import Pin

from tmc2209_uart import TMC2209UART


class TMC2209:
    REG_GCONF = 0x00
    REG_IHOLD_IRUN = 0x10
    REG_TPOWERDOWN = 0x11
    REG_TCOOLTHRS = 0x14
    REG_SGTHRS = 0x40
    REG_SG_RESULT = 0x41
    REG_CHOPCONF = 0x6C
    REG_IOIN = 0x06

    GCONF_PDN_DISABLE = 1 << 6
    GCONF_MSTEP_REG_SELECT = 1 << 7
    GCONF_MULTISTEP_FILT = 1 << 8

    MICROSTEP_TO_MRES = {
        256: 0,
        128: 1,
        64: 2,
        32: 3,
        16: 4,
        8: 5,
        4: 6,
        2: 7,
        1: 8,
    }

    def __init__(self, uart_id, baudrate, rx_pin, tx_pin, driver_address, en_pin=None, diag_pin=None):
        self._uart = TMC2209UART(uart_id, baudrate, rx_pin, tx_pin, driver_address)
        self._enable_pin = Pin(en_pin, Pin.OUT) if en_pin is not None else None
        self._diag_pin = Pin(diag_pin, Pin.IN, Pin.PULL_DOWN) if diag_pin is not None else None
        self._last_run_current = None
        self._last_hold_current = None
        self._enabled = False
        self._chopconf_shadow = None
        self._saved_toff = 4

        if self._enable_pin is not None:
            self.set_enabled(False)

    def close(self):
        self._uart.close()

    def test(self):
        return self._uart.test()

    def read_register(self, reg):
        return self._uart.read_int(reg)

    def write_register(self, reg, value):
        return self._uart.write_reg_check(reg, value)

    def read_chopconf(self):
        chopconf = self.read_register(self.REG_CHOPCONF)
        if chopconf is not None:
            self._chopconf_shadow = chopconf
            toff = self.get_toff(chopconf)
            if toff > 0:
                self._saved_toff = toff
        return chopconf

    def write_chopconf(self, value):
        if self.write_register(self.REG_CHOPCONF, value):
            self._chopconf_shadow = value
            toff = self.get_toff(value)
            if toff > 0:
                self._saved_toff = toff
            return True
        return False

    def get_toff(self, chopconf):
        return int(chopconf) & 0x0F

    def set_toff(self, chopconf, toff):
        return (int(chopconf) & ~0x0F) | (int(toff) & 0x0F)

    def configure_interface(self):
        gconf = self.read_register(self.REG_GCONF)
        if gconf is None:
            return False
        gconf |= self.GCONF_PDN_DISABLE | self.GCONF_MSTEP_REG_SELECT | self.GCONF_MULTISTEP_FILT
        return self.write_register(self.REG_GCONF, gconf)

    def set_enabled(self, enabled):
        if self._enable_pin is not None:
            self._enabled = bool(enabled)
            self._enable_pin.value(0 if enabled else 1)
            return True
        return self.set_driver_enabled_via_uart(enabled)

    def is_enabled(self):
        return self._enabled

    def set_driver_enabled_via_uart(self, enabled, fallback_toff=4):
        chopconf = self.read_chopconf()
        if chopconf is None:
            chopconf = self._chopconf_shadow
        if chopconf is None:
            return False

        if enabled:
            toff = self._saved_toff if self._saved_toff > 0 else max(1, int(fallback_toff))
        else:
            toff = 0

        updated = self.set_toff(chopconf, toff)
        if not self.write_chopconf(updated):
            return False

        self._enabled = bool(enabled)
        return True

    def apply_microstep_config(self, microsteps):
        if microsteps not in self.MICROSTEP_TO_MRES:
            raise ValueError("Unsupported microstep mode")
        chopconf = self.read_chopconf()
        if chopconf is None:
            return False
        chopconf &= ~(0x0F << 24)
        chopconf |= self.MICROSTEP_TO_MRES[microsteps] << 24
        return self.write_chopconf(chopconf)

    def set_run_hold_current(self, run_current, hold_current, hold_delay=8):
        run_current = max(0, min(31, int(run_current)))
        hold_current = max(0, min(31, int(hold_current)))
        hold_delay = max(0, min(15, int(hold_delay)))
        value = hold_current | (run_current << 8) | (hold_delay << 16)
        if self.write_register(self.REG_IHOLD_IRUN, value):
            self._last_run_current = run_current
            self._last_hold_current = hold_current
            return True
        return False

    def currents(self):
        return self._last_run_current, self._last_hold_current

    def set_powerdown_delay(self, delay):
        delay = max(0, min(255, int(delay)))
        return self.write_register(self.REG_TPOWERDOWN, delay)

    def set_stallguard_threshold(self, threshold):
        threshold = max(0, min(255, int(threshold)))
        return self.write_register(self.REG_SGTHRS, threshold)

    def set_coolstep_threshold(self, threshold):
        threshold = max(0, min(0xFFFFF, int(threshold)))
        return self.write_register(self.REG_TCOOLTHRS, threshold)

    def read_stallguard_result(self):
        return self.read_register(self.REG_SG_RESULT)

    def read_ioin(self):
        return self.read_register(self.REG_IOIN)

    def diag_output_state(self):
        value = self.read_ioin()
        if value is None:
            return None
        return (int(value) >> 4) & 0x01

    def diag_triggered(self):
        if self._diag_pin is None:
            return False
        return bool(self._diag_pin.value())

    def diag_raw(self):
        if self._diag_pin is None:
            return None
        return int(self._diag_pin.value())

    def set_diag_callback(self, handler=None, trigger=Pin.IRQ_RISING):
        if self._diag_pin is None:
            return False
        if handler is None:
            self._diag_pin.irq(handler=None)
            return True
        self._diag_pin.irq(trigger=trigger, handler=handler)
        return True

    def initialize(self, run_current, hold_current, microsteps, hold_delay=8):
        if not self.test():
            return False
        if not self.configure_interface():
            return False
        if not self.apply_microstep_config(microsteps):
            return False
        if not self.set_powerdown_delay(20):
            return False
        if not self.set_run_hold_current(run_current, hold_current, hold_delay):
            return False
        return self.read_chopconf() is not None
