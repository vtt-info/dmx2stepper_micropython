"""Configuration for the Pico-side UART-homing and one-axis DMX runtime firmware."""

# DMX input
DMX_PIN = 29
DMX_SM_ID = 4
DMX_START_CHANNEL = 1

# Step/dir trials for the active hardware
STEP_DIR_TRIALS = (
    (2, 3),
    (3, 2),
)
HOME_DIRECTION_TRIALS = (
    -1,
    1,
)

# Optional external DIAG debug input. It is not required for MVP homing.
DIAG_PIN = 8
DIAG_ACTIVE_LEVEL = 1

# TMC2209 UART
UART_ID = 0
UART_TX_PIN = 0
UART_RX_PIN = 1
UART_BAUDRATE = 230400
TMC_ADDRESS = 0

# EN is hardwired low on this hardware, so driver on/off must go through UART.
EN_PIN = None
DRIVER_ENABLE_TOFF = 4

# PIO stepping
PIO_STEP_SM_ID = 0
PIO_COUNTER_SM_ID = 1
PIO_STEP_FREQUENCY = 5_000_000
PIO_COUNTER_FREQUENCY = 125_000_000

# Driver defaults
MICROSTEP_MODE = 128
DEFAULT_RUN_CURRENT = 24
DEFAULT_HOLD_CURRENT = 12
CURRENT_HOLD_DELAY = 8

# UART-only homing
HOME_SPEED_TRIALS_1_8_HZ = (
    500,
)
HOME_MIN_FREQ_1_8_HZ = 300
HOME_MAX_FREQ_1_8_HZ = 1200
HOME_RETRACT_STEPS = 128
HOME_RETRACT_SPEED_HZ = 350
HOME_RELEASE_STEPS = 96
HOME_RELEASE_SPEED_HZ = 350
HOME_MEASURE_TRAVEL_STEPS = False
HOME_FIXED_TRAVEL_STEPS = 20000
RUNTIME_TRAVEL_STEPS = 20000
RUNTIME_SOFT_END_MARGIN_STEPS = 1000
HOME_MAX_STEPS = 4800
HOME_MIN_TRAVEL_STEPS = 1000
HOME_POLL_MS = 1
HOME_STARTUP_SG_SAMPLES = 12
HOME_MIN_STALL_STEPS = 96
HOME_UART_THRESHOLD_RATIO = 0.30
HOME_UART_THRESHOLD_MIN = 4
HOME_UART_THRESHOLD_MAX = 16
HOME_UART_CONFIRM_POLLS = 2
HOME_SETTLE_MS = 150
HOME_TIMEOUT_MARGIN_MS = 1200
HOME_COOLSTEP_THRESHOLD = 3200
HOME_SGTHRS = 0

# Stall detection mode: "uart", "diag_confirm", "hybrid"
#   uart         = original UART-only SG polling (poll_ms=5)
#   diag_confirm = DIAG pin with consecutive confirmation (poll_ms=1, fast)
#   hybrid       = DIAG triggers immediate UART confirmation
HOME_STALL_MODE = "uart"
HOME_DIAG_CONFIRM_POLLS = 3   # consecutive DIAG=1 readings to confirm stall
HOME_DIAG_POLL_MS = 1         # polling interval in DIAG modes
HOME_DIAG_WINDOW_SIZE = 10    # sliding window size for DIAG density detection
HOME_DIAG_WINDOW_THRESHOLD = 5  # triggers needed within window to confirm

# Runtime motion limits
DEFAULT_TARGET_U16 = 32768
MOTOR_MAX_SPEED_HZ = 18684
MOTOR_ACCELERATION_S2 = 300000
RUNTIME_POSITION_DEADBAND_STEPS = 2
RUNTIME_CONTROL_SLEEP_MS = 0
RUNTIME_MAX_CHUNK_STEPS = 64
RUNTIME_MIN_CHUNK_SPEED_HZ = 500

# Result / status output
RESULT_FILE = "homing_result.json"
STATUS_FILE = "controller_status.json"
STATUS_INTERVAL_MS = 500
RUNTIME_STATUS_STREAM_ENABLED = False
PRINT_INTERVAL_MS = 500
DEBUG_LOGGING = False

# Verification / bring-up controls
RUN_RUNTIME_AFTER_HOMING = True
RUNTIME_EXIT_AFTER_MS = 0
