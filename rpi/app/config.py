"""Configuration values for the SWARM Raspberry Pi UI."""
import os
from glob import glob
from typing import Optional

PORT = "/dev/ttyUSB0"
BAUD = 115200

ENABLE_FAULT_SCREEN = True          # set False to disable overlay (debug)
RX_TIMEOUT_S = 3.0                  # show fault if no RX for this many seconds

LOG_SUBDIR = "swarm_logs"
LOG_MAX_BYTES = 512 * 1024 * 1024
LOG_PRUNE_TARGET_BYTES = 384 * 1024 * 1024


def usb_mount_candidates() -> list[str]:
    """Return actual mounted USB/media directories in preferred order."""
    patterns = (
        "/media/swarm/*",
        "/media/*/*",
        "/run/media/*/*",
        "/mnt/*",
    )

    candidates = []
    seen = set()
    for pattern in patterns:
        for path in sorted(glob(pattern)):
            if path in seen or not os.path.isdir(path):
                continue
            seen.add(path)
            candidates.append(path)

    return candidates


def resolve_log_dir() -> Optional[str]:
    """Return the USB log directory, or None when no mounted USB media is available."""
    env_dir = os.environ.get("SWARM_LOG_DIR")
    if env_dir:
        mount_dir = env_dir if os.path.ismount(env_dir) else os.path.dirname(env_dir)
        if os.path.ismount(mount_dir) and os.access(mount_dir, os.W_OK):
            return env_dir
        return None

    for mount_dir in usb_mount_candidates():
        if os.path.ismount(mount_dir) and os.access(mount_dir, os.W_OK):
            return os.path.join(mount_dir, LOG_SUBDIR)

    return None

# UI defaults
DEFAULT_CYCLE_MIN = 10.0
DEFAULT_TEMP_F = 95.0
