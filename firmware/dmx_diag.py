"""Minimal DMX diagnostic: prints raw channel values and computed u16 position.

Deploy as main.py to observe DMX noise without any motor code running.
Usage:
  mpremote connect /dev/ttyACM0 fs cp config.py dmx_receiver.py dmx_diag.py :main.py
  mpremote connect /dev/ttyACM0 repl
"""

import time
import config
from dmx_receiver import DMXReceiver


def main():
    dmx = DMXReceiver(pin_num=config.DMX_PIN, sm_id=config.DMX_SM_ID)
    required_bytes = int(config.DMX_START_CHANNEL) + int(config.DMX_CHANNEL_COUNT)
    dmx.start()
    frame_num = 0
    print("DMX diagnostic started — watching channels {}-{}".format(
        config.DMX_START_CHANNEL,
        config.DMX_START_CHANNEL + config.DMX_CHANNEL_COUNT - 1,
    ))
    while True:
        if not dmx.read_frame():
            continue
        if dmx.last_start_code != 0x00:
            continue
        if int(dmx.last_bytes_received) < int(required_bytes):
            continue
        channels = dmx.get_channels(config.DMX_START_CHANNEL, config.DMX_CHANNEL_COUNT)
        u16 = (int(channels[0]) << 8) | int(channels[1])
        frame_num += 1
        print("frame={} ch={} u16={} bytes={}".format(
            frame_num,
            list(channels),
            u16,
            dmx.last_bytes_received,
        ))


if __name__ == "__main__":
    main()
