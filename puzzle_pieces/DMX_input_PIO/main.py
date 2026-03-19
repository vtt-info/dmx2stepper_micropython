"""
DMX Receiver Main Application
Receives DMX frames and displays channel values
"""

import time
from dmx_receiver import DMXReceiver
import config


def format_time():
    """Format current time as HH:MM:SS"""
    t = time.localtime()
    return "{:02d}:{:02d}:{:02d}".format(t[3], t[4], t[5])


def main():
    """Main application loop"""
    print("\n" + "="*60)
    print("DMX512 Receiver Starting...")
    print("="*60)
    print(f"Configuration:")
    print(f"  DMX Input Pin: GPIO{config.DMX_PIN}")
    print(f"  Start Channel: {config.START_CHANNEL}")
    print(f"  Num Channels:  {config.NUM_CHANNELS}")
    print(f"  Print Interval: {config.PRINT_INTERVAL}s")
    print("="*60 + "\n")

    # Initialize DMX receiver
    try:
        dmx = DMXReceiver(pin_num=config.DMX_PIN, sm_id=0)
        dmx.start()
        print(f"[{format_time()}] PIO DMX receiver initialized on GPIO{config.DMX_PIN}")
    except Exception as e:
        print(f"[ERROR] Failed to initialize DMX receiver: {e}")
        import sys
        sys.print_exception(e)
        return

    # State tracking
    last_print_time = time.ticks_ms()
    last_frame_count = 0
    last_signal_time = time.ticks_ms()
    signal_lost = False
    channel_values = [0] * config.NUM_CHANNELS

    print(f"[{format_time()}] Waiting for DMX signal...")
    print()

    try:
        while True:
            # Read next DMX frame (blocking, with 100ms timeout)
            frame_received = dmx.read_frame()

            if frame_received:
                # Update last signal time
                last_signal_time = time.ticks_ms()

                # Clear signal lost flag if it was set
                if signal_lost:
                    print(f"[{format_time()}] Signal restored - receiving DMX frames")
                    signal_lost = False

                # Get channel values
                channel_values = dmx.get_channels(
                    config.START_CHANNEL,
                    config.NUM_CHANNELS
                )

                # Check for non-standard start code
                if dmx.last_start_code != 0x00:
                    print(f"[WARN {format_time()}] Non-standard start code: 0x{dmx.last_start_code:02X}")

            # Check for signal loss (no frames for >1 second)
            time_since_signal = time.ticks_diff(time.ticks_ms(), last_signal_time)
            if time_since_signal > 1000 and not signal_lost:
                signal_lost = True
                print(f"[ERROR {format_time()}] DMX signal loss - no frames for {time_since_signal/1000:.1f}s")

            # Print status at regular intervals
            time_since_print = time.ticks_diff(time.ticks_ms(), last_print_time)
            if time_since_print >= (config.PRINT_INTERVAL * 1000):
                current_frame_count = dmx.get_frame_count()
                frames_received = current_frame_count - last_frame_count

                # Calculate frame rate
                fps = frames_received / (time_since_print / 1000)

                # Format channel values
                ch_str = " | ".join([
                    f"CH{config.START_CHANNEL + i}:{channel_values[i]:3d}"
                    for i in range(config.NUM_CHANNELS)
                ])

                # Print status line
                if not signal_lost:
                    print(f"[{format_time()}] Frame: {current_frame_count:6d} ({fps:4.1f} fps) | {ch_str}")
                else:
                    print(f"[{format_time()}] NO SIGNAL - Last frame: {current_frame_count:6d}")

                # Check for errors
                errors = dmx.get_errors()
                if errors['start_code'] > 0:
                    print(f"         Errors - Start Code: {errors['start_code']}")
                    dmx.reset_errors()

                # Update tracking
                last_print_time = time.ticks_ms()
                last_frame_count = current_frame_count

            # No sleep needed - read_frame() blocks until frame or timeout

    except KeyboardInterrupt:
        print(f"\n[{format_time()}] Shutting down...")
        dmx.stop()
        print(f"[{format_time()}] Total frames received: {dmx.get_frame_count()}")
        print("="*60)

    except Exception as e:
        print(f"\n[ERROR {format_time()}] Exception: {e}")
        import sys
        sys.print_exception(e)
        dmx.stop()


if __name__ == "__main__":
    main()
