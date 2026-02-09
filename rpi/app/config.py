"""Configuration values for the SWARM Raspberry Pi UI."""

PORT = "/dev/ttyUSB0"
BAUD = 115200

ENABLE_FAULT_SCREEN = True          # set False to disable overlay (debug)
RX_TIMEOUT_S = 3.0                  # show fault if no RX for this many seconds

LOG_DIR = "logs"

# UI defaults
DEFAULT_CYCLE_MIN = 10.0
DEFAULT_TEMP_F = 100.0

# Faults
FAULTS_ENABLED = {
    "temp": True,
    "level": True,
    "flow": False,
    "lid": True,
    "turbidity": False,
    }
OVERTEMP_F = 82.0                     # over-temperature threshold for fault screen
TURBIDITY_FAULT_PCT = 80.0            # turbidity threshold for fault screen
FLOW_FAULT_LPM = 0.5                  # flow threshold for fault screen
