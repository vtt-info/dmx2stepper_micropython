"""
Control a TMC2209 stepper driver from DMX input.

DMX mapping:
- Channel 1 -> motor speed (triangular, with speed=0 at DMX values 0, 127, 255)
  - 1..126: forward profile
  - 128..254: backward profile
- Channel 2 -> motor current (mapped to both IHOLD and IRUN, 0..31)
- Channel 3 -> software power-stage control
  - 0: enabled (TOFF restored)
  - >0: disabled (TOFF=0, equivalent to EN high behavior)
- Channel 4 -> sensorless homing trigger (hold above threshold to trigger once)
"""

import time
from machine import UART, Pin

import config
from dmx_receiver import DMXReceiver


class TMC2209:
    VACTUAL = 0x22
    IHOLD_IRUN = 0x10
    CHOPCONF = 0x6C
    GSTAT = 0x01
    IFCNT = 0x02
    DRV_STATUS = 0x6F
    TCOOLTHRS = 0x14
    SGTHRS = 0x40
    SG_RESULT = 0x41

    def __init__(self, uart, motor_id=0):
        self.uart = uart
        self.motor_id = motor_id

    def compute_crc8(self, data):
        crc = 0
        for byte in data:
            for _ in range(8):
                if (crc >> 7) ^ (byte & 0x01):
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
                byte >>= 1
        return crc

    def _drain_uart(self):
        while self.uart.any():
            self.uart.read()

    def send_command(self, reg, data_int=None):
        if data_int is None:
            cmd = bytearray(4)
            cmd[0] = 0x55
            cmd[1] = self.motor_id
            cmd[2] = reg & 0x7F
            cmd[3] = self.compute_crc8(cmd[:3])
            self.uart.write(cmd)
        else:
            cmd = bytearray(8)
            cmd[0] = 0x55
            cmd[1] = self.motor_id
            cmd[2] = (reg & 0x7F) | 0x80
            cmd[3] = (data_int >> 24) & 0xFF
            cmd[4] = (data_int >> 16) & 0xFF
            cmd[5] = (data_int >> 8) & 0xFF
            cmd[6] = data_int & 0xFF
            cmd[7] = self.compute_crc8(cmd[:7])
            self.uart.write(cmd)

        time.sleep_ms(10)
        if self.uart.any():
            return self.uart.read()
        return None

    def _parse_read_response(self, response, reg):
        if not response:
            return None

        target_reg = reg & 0x7F
        n = len(response)
        if n < 8:
            return None

        for i in range(n - 7):
            frame = response[i : i + 8]
            # TMC2209 commonly replies with slave address 0xFF and can include
            # an echoed request in the same RX buffer.
            if frame[1] not in (self.motor_id, 0xFF):
                continue
            if (frame[2] & 0x7F) != target_reg:
                continue
            if self.compute_crc8(frame[:7]) != frame[7]:
                continue
            return (frame[3] << 24) | (frame[4] << 16) | (frame[5] << 8) | frame[6]

        return None

    def read_register(self, reg, retries=3):
        for _ in range(retries):
            self._drain_uart()
            response = self.send_command(reg)
            value = self._parse_read_response(response, reg)
            if value is not None:
                return value
            time.sleep_ms(2)
        return None

    def write_register(self, reg, value):
        return self.send_command(reg, value)

    def set_velocity(self, velocity):
        return self.send_command(self.VACTUAL, velocity)

    def set_currents(self, hold_current, run_current, hold_delay):
        data = (hold_delay << 16) | ((run_current & 0x1F) << 8) | (hold_current & 0x1F)
        return self.send_command(self.IHOLD_IRUN, data)

    def set_toff(self, chopconf_value, toff):
        return (chopconf_value & ~0x0F) | (toff & 0x0F)

    def get_toff(self, chopconf_value):
        return chopconf_value & 0x0F

    def set_mres(self, chopconf_value, mres_code):
        return (chopconf_value & ~(0x0F << 24)) | ((mres_code & 0x0F) << 24)

    def get_mres(self, chopconf_value):
        return (chopconf_value >> 24) & 0x0F


def microstep_to_mres_code(microsteps):
    mapping = {
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
    return mapping.get(microsteps, 4)


def map_dmx_speed(value, max_velocity):
    """Map DMX 0..255 to speed with 0 at 0, 127 and 255."""
    if value <= 0 or value >= 255 or value == 127:
        return 0

    if value < 127:
        if value <= 64:
            ratio = value / 64.0
        else:
            ratio = (127 - value) / 63.0
        return int(ratio * max_velocity)

    if value <= 191:
        ratio = (value - 127) / 64.0
    else:
        ratio = (255 - value) / 64.0
    return -int(ratio * max_velocity)


def map_dmx_current(value, max_current=31):
    """Map DMX 0..255 to current 0..max_current."""
    return int((value / 255.0) * max_current)


def s32(value):
    if value is None:
        return None
    if value & 0x80000000:
        return value - 0x100000000
    return value


def decode_drv_status(value):
    if value is None:
        return "DRV_STATUS=<no data>"

    cs_actual = (value >> 16) & 0x1F
    flags = []

    if (value >> 31) & 0x01:
        flags.append("stst")
    flags.append("mode=stealth" if ((value >> 30) & 0x01) else "mode=spread")

    if (value >> 11) & 0x01:
        flags.append("t157")
    if (value >> 10) & 0x01:
        flags.append("t150")
    if (value >> 9) & 0x01:
        flags.append("t143")
    if (value >> 8) & 0x01:
        flags.append("t120")
    if (value >> 7) & 0x01:
        flags.append("olb")
    if (value >> 6) & 0x01:
        flags.append("ola")
    if (value >> 5) & 0x01:
        flags.append("s2vsb")
    if (value >> 4) & 0x01:
        flags.append("s2vsa")
    if (value >> 3) & 0x01:
        flags.append("s2gb")
    if (value >> 2) & 0x01:
        flags.append("s2ga")
    if (value >> 1) & 0x01:
        flags.append("ot")
    if value & 0x01:
        flags.append("otpw")

    if len(flags) == 1 and (flags[0].startswith("mode=")):
        flags.append("no_fault_flags")

    return "CS_ACTUAL={} {}".format(cs_actual, ",".join(flags))


def run_sensorless_homing(driver):
    """Run two-end sensorless homing and center by half-traverse time."""
    def wait_for_stall(direction, speed, timeout_ms):
        driver.set_velocity(direction * speed)
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
            sg = driver.read_register(TMC2209.SG_RESULT)
            if sg is not None and sg <= config.HOMING_SG_RESULT_THRESHOLD:
                driver.set_velocity(0)
                return True, time.ticks_diff(time.ticks_ms(), t0)
            time.sleep_ms(config.HOMING_POLL_MS)
        driver.set_velocity(0)
        return False, time.ticks_diff(time.ticks_ms(), t0)

    print("HOMING: configuring StallGuard")
    driver.write_register(TMC2209.TCOOLTHRS, config.HOMING_TCOOLTHRS)
    driver.write_register(TMC2209.SGTHRS, config.HOMING_SGTHRS)
    time.sleep_ms(50)

    print("HOMING: moving to first end-stop")
    ok1, _ = wait_for_stall(-1, config.HOMING_SPEED, config.HOMING_TIMEOUT_MS)
    if not ok1:
        driver.write_register(TMC2209.SGTHRS, 0)
        print("HOMING: failed on first end-stop")
        return False

    print("HOMING: backing off")
    driver.set_velocity(config.HOMING_BACKOFF_SPEED)
    time.sleep_ms(config.HOMING_BACKOFF_MS)
    driver.set_velocity(0)
    time.sleep_ms(80)

    print("HOMING: moving to second end-stop")
    ok2, traverse_ms = wait_for_stall(1, config.HOMING_SPEED, config.HOMING_TIMEOUT_MS)
    if not ok2:
        driver.write_register(TMC2209.SGTHRS, 0)
        print("HOMING: failed on second end-stop")
        return False

    center_ms = traverse_ms // 2
    print("HOMING: centering for {} ms".format(center_ms))
    if center_ms > 0:
        driver.set_velocity(-config.HOMING_SPEED)
        time.sleep_ms(center_ms)
    driver.set_velocity(0)

    driver.write_register(TMC2209.SGTHRS, 0)
    print("HOMING: completed")
    return True


def main():
    uart = UART(
        config.TMC_UART_ID,
        baudrate=config.TMC_UART_BAUD,
        tx=Pin(config.TMC_UART_TX_PIN),
        rx=Pin(config.TMC_UART_RX_PIN),
    )
    time.sleep_ms(100)

    driver = TMC2209(uart, motor_id=config.TMC_MOTOR_ID)

    chopconf_shadow = driver.read_register(TMC2209.CHOPCONF)
    if chopconf_shadow is None:
        chopconf_shadow = config.CHOPCONF_FALLBACK
        print("WARN: CHOPCONF read failed; using fallback 0x{:08X}".format(chopconf_shadow))

    saved_toff = driver.get_toff(chopconf_shadow)
    if saved_toff == 0:
        saved_toff = config.TOFF_FALLBACK

    selected_microsteps = config.MICROSTEP_SETTING
    if selected_microsteps not in config.MICROSTEP_OPTIONS:
        selected_microsteps = 16
        print("WARN: invalid MICROSTEP_SETTING; using 16")
    mres_code = microstep_to_mres_code(selected_microsteps)
    chopconf_shadow = driver.set_mres(chopconf_shadow, mres_code)
    driver.write_register(TMC2209.CHOPCONF, chopconf_shadow)

    current_value = map_dmx_current(0, config.MAX_CURRENT)
    driver.set_currents(current_value, current_value, config.HOLD_DELAY)

    dmx = DMXReceiver(pin_num=config.DMX_PIN, sm_id=0)
    dmx.start()

    print("DMX stepper control started")
    print("Microstepping: {} (MRES={})".format(selected_microsteps, mres_code))
    print(
        "CH{}=speed, CH{}=hold+run current, CH{}=power-stage disable".format(
            config.SPEED_CHANNEL,
            config.CURRENT_CHANNEL,
            config.POWER_STAGE_CHANNEL,
        )
    )

    last_signal_time = time.ticks_ms()
    last_print_time = time.ticks_ms()
    last_velocity = 0
    desired_velocity = 0
    power_stage_disabled = False
    homing_active = False
    homing_latched = False
    homing_trigger_since = 0

    try:
        while True:
            frame_received = dmx.read_frame()

            if frame_received and dmx.last_start_code == 0x00:
                last_signal_time = time.ticks_ms()

                speed_raw = dmx.get_channel(config.SPEED_CHANNEL)
                current_raw = dmx.get_channel(config.CURRENT_CHANNEL)
                power_raw = dmx.get_channel(config.POWER_STAGE_CHANNEL)
                homing_raw = dmx.get_channel(config.HOMING_TRIGGER_CHANNEL)

                desired_velocity = map_dmx_speed(speed_raw, config.MAX_VELOCITY)
                mapped_current = map_dmx_current(current_raw, config.MAX_CURRENT)

                if mapped_current != current_value:
                    driver.set_currents(mapped_current, mapped_current, config.HOLD_DELAY)
                    current_value = mapped_current

                requested_disable = power_raw > 0

                if requested_disable and not power_stage_disabled:
                    if last_velocity != 0:
                        driver.set_velocity(0)
                        last_velocity = 0

                    chopconf_shadow = driver.set_toff(chopconf_shadow, 0)
                    driver.write_register(TMC2209.CHOPCONF, chopconf_shadow)
                    power_stage_disabled = True

                elif not requested_disable and power_stage_disabled:
                    toff_restore = saved_toff if saved_toff > 0 else config.TOFF_FALLBACK
                    chopconf_shadow = driver.set_toff(chopconf_shadow, toff_restore)
                    driver.write_register(TMC2209.CHOPCONF, chopconf_shadow)
                    power_stage_disabled = False

                if homing_raw > config.HOMING_TRIGGER_THRESHOLD:
                    if homing_trigger_since == 0:
                        homing_trigger_since = time.ticks_ms()
                    elif (
                        not homing_latched
                        and not homing_active
                        and time.ticks_diff(time.ticks_ms(), homing_trigger_since) >= config.HOMING_TRIGGER_HOLD_MS
                    ):
                        homing_latched = True
                        if power_stage_disabled:
                            print("HOMING: skipped (power stage disabled by CH3)")
                        else:
                            homing_active = True
                            last_velocity = 0
                            driver.set_velocity(0)
                            run_sensorless_homing(driver)
                            homing_active = False
                else:
                    homing_trigger_since = 0
                    homing_latched = False

                if not power_stage_disabled and desired_velocity != last_velocity:
                    driver.set_velocity(desired_velocity)
                    last_velocity = desired_velocity

            if time.ticks_diff(time.ticks_ms(), last_signal_time) > config.SIGNAL_TIMEOUT_MS:
                if last_velocity != 0:
                    driver.set_velocity(0)
                    last_velocity = 0

            if time.ticks_diff(time.ticks_ms(), last_print_time) >= config.PRINT_INTERVAL_MS:
                ihold_irun = driver.read_register(TMC2209.IHOLD_IRUN)
                chopconf = driver.read_register(TMC2209.CHOPCONF)
                gstat = driver.read_register(TMC2209.GSTAT)
                ifcnt = driver.read_register(TMC2209.IFCNT)
                drv_status = driver.read_register(TMC2209.DRV_STATUS)
                vactual_reg = driver.read_register(TMC2209.VACTUAL)
                sg_result = driver.read_register(TMC2209.SG_RESULT)

                ihold = (ihold_irun & 0x1F) if ihold_irun is not None else -1
                irun = ((ihold_irun >> 8) & 0x1F) if ihold_irun is not None else -1
                ihold_delay = ((ihold_irun >> 16) & 0xFF) if ihold_irun is not None else -1
                toff = (chopconf & 0x0F) if chopconf is not None else -1
                mres = ((chopconf >> 24) & 0x0F) if chopconf is not None else -1
                vactual_signed = s32(vactual_reg)
                drv_text = decode_drv_status(drv_status)

                print(
                    "UART IHOLD={} IRUN={} IHOLDDELAY={} | VACTUAL={} | TOFF={} MRES={} | SG_RESULT={} | IFCNT={} GSTAT=0x{:08X} | {} | stage={} homing={}".format(
                        ihold,
                        irun,
                        ihold_delay,
                        vactual_signed if vactual_signed is not None else -1,
                        toff,
                        mres,
                        sg_result if sg_result is not None else -1,
                        ifcnt if ifcnt is not None else -1,
                        gstat if gstat is not None else 0,
                        drv_text,
                        "DISABLED" if power_stage_disabled else "ENABLED",
                        "ACTIVE" if homing_active else "IDLE",
                    )
                )
                last_print_time = time.ticks_ms()

    except KeyboardInterrupt:
        driver.set_velocity(0)
        dmx.stop()


if __name__ == "__main__":
    main()
