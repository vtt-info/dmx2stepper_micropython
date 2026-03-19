"""
Andrea Favero 16/05/2025   (rev 25/06/2025)

Micropython code for Raspberry Pi Pico (RP2040 and RP2350)
It demonstrates how to use StallGuard function from TMC2209 stepper driver.
The RP2040 (or RP2350) use PIO to generate the stepper steps




MIT License

Copyright (c) 2025 Andrea Favero

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

from machine import Pin
import time, os

# DMX512 Receiver imports and constants
import rp2
from machine import mem32
import array

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

    def __init__(self, pin_num, sm_id=3):
        self.pin_num = pin_num
        self.pin_mask = 1 << pin_num

        # PIO state machine for byte reception (sm_id=3 to avoid conflict with stepper)
        self.sm = rp2.StateMachine(sm_id, dmx_rx, freq=2_000_000,
                                   in_base=Pin(pin_num, Pin.IN, Pin.PULL_UP))

        # Frame buffer: start code + 512 channels
        self.frame_buffer = array.array('B', [0] * (DMX_MAX_CHANNELS + 1))

        # Statistics
        self.frame_count = 0
        self.last_frame_time = 0
        self.start_code_errors = 0
        self.last_start_code = 0

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
        """Detect DMX break: line LOW for >44us."""
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
        """Read a complete DMX frame (blocking)."""
        if not self.receiving:
            return False

        if not self._wait_for_break():
            return False

        # Drain stale PIO data
        while self.sm.rx_fifo() > 0:
            self.sm.get()

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
                if time.ticks_diff(time.ticks_us(), timeout_start) > 1000:
                    break
                time.sleep_us(10)

        if bytes_received == 0:
            return False

        self.last_start_code = self.frame_buffer[0]
        if self.last_start_code != DMX_START_CODE:
            self.start_code_errors += 1

        self.frame_count += 1
        self.last_frame_time = time.ticks_ms()
        return True

    def get_channel(self, channel):
        """Get DMX channel value (1-512)."""
        if channel < 1 or channel > DMX_MAX_CHANNELS:
            return 0
        return self.frame_buffer[channel]

    def get_last_frame_time(self):
        return self.last_frame_time


def import_led():
    """
    The rgb_led is used to visually feedback once the code is started.
    The led flashes for quite some time, allowing to connect to the RP2040 via an IDE (i.e. Thonny).
    During this time is possible to interrupt (CTRL + C) the code from further imports.
    """
    print("waiting time to eventually stop the code before further imports ...")
    from rgb_led import rgb_led
    ret = False
    ret = rgb_led.heart_beat(n=10, delay=1)
    while not ret:
        time. sleep(0.1)


def import_stepper():
    """
    Stepper module is imported via a function for a delayed import.
    This gives time to eventually stop the code before further imports.
    """
    from stepper import Stepper
    return Stepper


def _centering(pin, stepper_frequencies):
    """
    Internal helper to call the centering method at the stepper.py Class.
    This function updates Global variables.
    """
    global last_idx, centering
    
    centering = True
    print("\n\n")
    print("#"*78)
    print("#"*12, "  Stepper centering via SENSORLESS homing function  ", "#"*12)
    print("#"*78)
    
    idx = 0 if last_idx == 1 else 1          # flag 0 and 1 (alternates every time this function is called)
    last_idx = idx                           # flag tracking the last idx value
    
    # call to the stepper centering method. Note: The stepper frequency alternates each time between
    # the two vaues set in stepper_frequencies, therefore testing the extreme speed cases
    ret = stepper.centering(stepper_frequencies[idx])
    
    if ret:
        print("\nStepper is centered\n\n")
    else:
        print("\nFailed to center the stepper\n\n")
    
    centering = False



def stop_code():
    if 'stepper' in locals():                # case stepper has been imported
        stepper.stop_stepper()               # stepper gets stopped (PIO steps generation)
        stepper.deactivate_pio()             # PIO's get deactivated
    if 'enable_pin' in locals():             # case enable_pin has been defined
        enable_pin.value(1)                  # pin is set high (disable TMC2209 current to stepper)
    if 'dmx' in locals() and dmx is not None:  # case dmx receiver has been defined
        dmx.stop()                           # stop DMX receiver
    print("\nClosing the program ...")       # feedback is printed to the terminal
    


################################################################################################
################################################################################################
################################################################################################

# variables setting

last_idx = 1                                 # flag used to alternate between the 2 stepper frequencies
stepper_frequencies = (400, 2000)            # stepper speeds (Hz) alternatively used for the centering demo
# The stepper_frequencies values for homing could be changed based on your need
# Note1: values within the range 400 ~ 1200Hz respond well to the SENSORLESS HOMING.
# Note2: values outside the range 400 ~ 2000Hz will be clamped to these values by stepper.py

# DMX trigger variables
dmx_trigger_channel = 1        # DMX channel to monitor
dmx_trigger_threshold = 200   # Value above which triggers homing
dmx_trigger_time_ms = 2000    # Time value must be above threshold to trigger
dmx_trigger_timer = 0         # Timer for tracking hold time
dmx_last_value = 0           # Last DMX channel 1 value
dmx_signal_ok = False        # Flag for DMX signal status

homing_requested = False       # flag used by the DMX trigger to start centering
centering = False              # flag tracking the centering process (not in action / in action)

debug = True                                 # if True some informative prints will be made on the Shell




try:
    
    rgb_led = import_led()                   # led module for visual feedback
    Stepper = import_stepper()               # stepper module
    board_info = os.uname()                  # determining wich board_type is used

    # assigning max PIO frequency, depending on the board type
    if '2040' in board_info.machine:
        if 'W' in board_info.machine:
            board_type = 'RP2040 W'
        else:
            board_type = 'RP2040'
        max_pio_frequency = 125_000_000
        
    elif '2350' in board_info.machine.lower():
        if 'W' in board_info.machine:
            board_type = 'RP2350 W'
        else:
            board_type = 'RP2350'
        max_pio_frequency = 150_000_000
    else:
        board_type = '???'
        max_pio_frequency = 125_000_000



    # GPIO pin to enable the motor
    # Note: Alternatively, the TMC2209 EN pin must be wired to GND
    enable_pin = Pin(2, Pin.IN, Pin.PULL_UP)
    enable_pin.value(0)  # pin is set low (TMC2209 enabled, stepper always energized)

    # Initialize DMX receiver on GPIO 29
    print("\nInitializing DMX receiver on GPIO 29...")
    dmx = DMXReceiver(pin_num=29, sm_id=3)
    try:
        dmx.start()
        print("DMX receiver started successfully")
    except Exception as e:
        print(f"Failed to start DMX receiver: {e}")
        dmx = None

    # stepper Class instantiatiation
    stepper = Stepper(max_frequency=max_pio_frequency, frequency=5_000_000, debug=debug)

    # case the TMC driver UART reacts properly (it is powered and properly wired)
    if stepper.tmc_test():
        
        # board tupe and other info are printed to the terminal
        print("\nCode running in {} board".format(board_type))
        print("Sensorless homing example")
        print("\nWaiting for DMX signal on channel 1 (value > 200 for 2 seconds)...")


        # iterative part of the mainn function
        while True:                              # infinite loop
            # Check DMX trigger
            if dmx is not None and dmx.receiving:
                # Try to read a DMX frame
                if dmx.read_frame():
                    dmx_signal_ok = True
                    dmx_last_value = dmx.get_channel(dmx_trigger_channel)

                    if dmx_last_value > dmx_trigger_threshold:
                        # Start timer if not started
                        if dmx_trigger_timer == 0:
                            dmx_trigger_timer = time.ticks_ms()
                        # Check if threshold exceeded
                        elif time.ticks_diff(time.ticks_ms(), dmx_trigger_timer) > dmx_trigger_time_ms:
                            if not centering:
                                homing_requested = True
                                dmx_trigger_timer = 0  # Reset after triggering
                else:
                    # Check for signal loss (>1 second since last frame)
                    if time.ticks_diff(time.ticks_ms(), dmx.get_last_frame_time()) > 1000:
                        if dmx_signal_ok:
                            print("NO SIGNAL")
                            dmx_signal_ok = False

            # Handle homing request
            if homing_requested:
                homing_requested = False
                _centering(None, stepper_frequencies)

            time.sleep(0.1)                      # small sleeping while waiting for the 'homing request'
            
            
            # ################################################################################################### #
            # Here you'd put the application code, to be executed after the SENSORLESS HOMING                     #
            #                                                                                                     #
            # Recall to set the StallGuard to a proper value for the application.                                 #
            # If the application do not require Stall control, set the StallGuard threshol to 0 (max torque).     #
            #                                                                                                     #
            # Example of no Stall control:                                                                        #
            # stepper.set_stallguard(threshold = 0)    # set SG threshold acting on the DIAG pin, to max torque   #
            #                                                                                                     #
            # For a proper Stall control, first set the driver current and the stepper speed for your usage case. #
            # Afterward, check the StallGuard value with and without load.                                        #
            # Set the StallGuard threshold to <= 0.5 * measured SG_value in normal application.                   #
            #                                                                                                     #
            # Example of no Stall control:                                                                        #
            # sg_threshold = 0.45 * minimum_measure_SG_value_in_application                                       #
            # stepper.set_stallguard(threshold = sg_threshold)    # set SG threshold acting on the DIAG pin       #
            # ################################################################################################### #


    # case the TMC driver UART does not react (not powered or not wired properly)
    else:
        
        # info is printed to the terminal
        print("\n"*2)
        print("#"*67)
        print("#"*67)
        print("#", " "*63, "#",)
        print("#   The TMC driver UART does not react: IS THE DRIVER POWERED ?   #")
        print("#", " "*63, "#",)
        print("#"*67)
        print("#"*67)
        print("\n"*2)


except KeyboardInterrupt:                    # keyboard interrupts
    print("\nCtrl+C detected!")              # feedback is printed to the terminal
    
except Exception as e:                       # error 
    print(f"\nAn error occured: {e}")        # feedback is printed to the terminal

finally:                                     # closing the try loop
    stop_code()                              # stop_code function to stop PIOs
