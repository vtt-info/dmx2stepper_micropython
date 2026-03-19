# DMX Receiver Configuration

# Starting DMX channel to monitor (1-505)
# DMX channels are numbered 1-512, but we need room for 8 channels
START_CHANNEL = 1

# Number of channels to capture (fixed at 8)
NUM_CHANNELS = 8

# Interval between console prints (seconds)
PRINT_INTERVAL = 1.0

# GPIO pin for DMX input (connected to RS485 TTL RX)
DMX_PIN = 1
