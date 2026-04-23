"""Configuration values for the SWARM Raspberry Pi UI."""
import os
from glob import glob

PORT = "/dev/ttyUSB0"
BAUD = 115200

ENABLE_FAULT_SCREEN = True          # set False to disable overlay (debug)
RX_TIMEOUT_S = 3.0                  # show fault if no RX for this many seconds

LOG_FALLBACK_DIR = "logs"
LOG_SUBDIR = "swarm_logs"
LOG_MAX_BYTES = 512 * 1024 * 1024
LOG_PRUNE_TARGET_BYTES = 384 * 1024 * 1024


def resolve_log_dir() -> str:
    """Prefer a mounted USB log directory, with a local fallback for development."""
    env_dir = os.environ.get("SWARM_LOG_DIR")
    if env_dir:
        return env_dir

    usb_candidates = []
    usb_candidates.extend(sorted(glob("/media/pi/*")))
    usb_candidates.extend(sorted(glob("/media/*")))
    usb_candidates.extend(sorted(glob("/mnt/*")))

    for mount_dir in usb_candidates:
        if os.path.isdir(mount_dir) and os.access(mount_dir, os.W_OK):
            return os.path.join(mount_dir, LOG_SUBDIR)

    return LOG_FALLBACK_DIR

# UI defaults
DEFAULT_CYCLE_MIN = 10.0
DEFAULT_TEMP_F = 95.0

# Fault thresholds are now sourced from ESP32 telemetry.
