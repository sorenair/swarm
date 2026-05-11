"""High-level command interface for the SWARM controller."""
from dataclasses import dataclass
from typing import Callable

@dataclass
class Controller:
    """Wraps command formatting and transmission for the ESP32."""
    send_line: Callable[[str], None]      # serial client send
    log_tx: Callable[[str], None]         # tx logger wrapper

    def _send(self, cmd: str):
        """Log and transmit a command line."""
        self.log_tx(cmd)
        self.send_line(cmd)

    # --- cycle commands ---
    def start_cycle(self, duration_s: int, temp_set_f: float):
        """Send the start-cycle command with duration and temperature."""
        self._send(f"CYCLE START {int(duration_s)} {float(temp_set_f):.1f}")

    def pause_cycle(self):
        """Send the pause-cycle command."""
        self._send("CYCLE PAUSE")

    def resume_cycle(self):
        """Send the resume-cycle command."""
        self._send("CYCLE RESUME")

    def stop_cycle(self):
        """Send the stop-cycle command."""
        self._send("CYCLE STOP")
