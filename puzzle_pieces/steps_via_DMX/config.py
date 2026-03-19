"""Configuration for TMC2209 control via DMX input."""

# Microstepping configuration (applied via UART CHOPCONF.MRES).
# Valid microstep values (steps per full step): 256, 128, 64, 32, 16, 8, 4, 2, 1
# TMC2209 MRES encoding is inverse (256->0 ... 1->8), handled in main.py.
MICROSTEP_OPTIONS = [256, 128, 64, 32, 16, 8, 4, 2, 1]
MICROSTEP_SETTING = 64

# DMX receiver pin (from RS485 TTL receiver).
# Use a pin that does not conflict with the TMC UART pins.
DMX_PIN = 29

# TMC2209 UART configuration.
TMC_UART_ID = 0
TMC_UART_BAUD = 115200
TMC_UART_TX_PIN = 0
TMC_UART_RX_PIN = 1
TMC_MOTOR_ID = 0

# DMX channel mapping.
SPEED_CHANNEL = 1
CURRENT_CHANNEL = 2
POWER_STAGE_CHANNEL = 3
HOMING_TRIGGER_CHANNEL = 4

# TMC2209 current settings.
HOLD_DELAY = 25
MAX_CURRENT = 31

# CHOPCONF handling for software power-stage enable/disable via TOFF.
# TOFF=0 disables outputs. TOFF>0 enables outputs.
TOFF_FALLBACK = 3
# Used only if CHOPCONF cannot be read from the driver.
CHOPCONF_FALLBACK = 0x10000053

# Maximum velocity magnitude in microsteps/second.
MAX_VELOCITY = 800

# If no DMX frames arrive for this many ms, motor is stopped.
SIGNAL_TIMEOUT_MS = 1000

# Sensorless homing settings (triggered by DMX channel HOMING_TRIGGER_CHANNEL).
HOMING_TRIGGER_THRESHOLD = 200
HOMING_TRIGGER_HOLD_MS = 800
HOMING_SPEED = 550
HOMING_BACKOFF_SPEED = 400
HOMING_BACKOFF_MS = 250
HOMING_TIMEOUT_MS = 8000
HOMING_POLL_MS = 20
HOMING_SGTHRS = 30
HOMING_TCOOLTHRS = 200000
HOMING_SG_RESULT_THRESHOLD = 40

# Periodic debug print interval in ms.
PRINT_INTERVAL_MS = 10000

# Print incoming DMX values on every valid frame.
DEBUG_DMX_VALUES = False
