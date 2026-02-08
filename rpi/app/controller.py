# app/controller.py
from dataclasses import dataclass
from typing import Callable

@dataclass
class Controller:
    send_line: Callable[[str], None]      # serial client send
    log_tx: Callable[[str], None]         # tx logger wrapper

    def _send(self, cmd: str):
        self.log_tx(cmd)
        self.send_line(cmd)

    # --- cycle commands ---
    def start_cycle(self, duration_s: int, temp_set_f: float):
        self._send(f"CYCLE START {int(duration_s)} {float(temp_set_f):.1f}")

    def pause_cycle(self):
        self._send("CYCLE PAUSE")

    def resume_cycle(self):
        self._send("CYCLE RESUME")

    def stop_cycle(self):
        self._send("CYCLE STOP")

    # --- heater commands (if you have them) ---
    def set_heater_enable(self, enabled: bool):
        self._send(f"HEATER ENABLE {1 if enabled else 0}")
