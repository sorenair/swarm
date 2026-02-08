# app/log_async.py
import os, csv, time, queue, threading
from typing import Any, Dict, List, Optional
from datetime import datetime

from openpyxl import Workbook, load_workbook
from openpyxl.utils import get_column_letter

from app.state import AppModel


class AsyncLogger:
    def __init__(self, xlsx_path: str, csv_path: str, event_headers: List[str]):
        self.xlsx_path = xlsx_path
        self.csv_path = csv_path
        self.event_headers = event_headers

        self.lq: "queue.Queue[dict]" = queue.Queue()
        self.stop_flag = threading.Event()

        self._init_csv()
        self._init_xlsx()

        self.t = threading.Thread(target=self._worker, daemon=True)
        self.t.start()

    def _init_csv(self):
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, "w", newline="", encoding="utf-8") as f:
                csv.writer(f).writerow(self.event_headers)

    def _init_xlsx(self):
        if os.path.exists(self.xlsx_path):
            return

        wb = Workbook()
        ws_events = wb.active
        ws_events.title = "Events"
        ws_events.append(self.event_headers)

        ws_rx = wb.create_sheet("RxRaw")
        ws_rx.append(["timestamp", "raw_line"])

        ws_tx = wb.create_sheet("TxCmd")
        ws_tx.append(["timestamp", "command"])

        for ws in (ws_events, ws_rx, ws_tx):
            for i, h in enumerate(ws[1], start=1):
                ws.column_dimensions[get_column_letter(i)].width = max(12, min(42, len(str(h)) + 2))

        wb.save(self.xlsx_path)

    def log_event(self, row: dict):
        self.lq.put(row)

    # Convenience wrappers so main.py stays clean
    def log_status(self, timestamp: str, msg: str):
        self.log_event({"timestamp": timestamp, "event_type": "status", "raw": msg})

    def log_rx(self, timestamp: str, raw_line: str):
        self.log_event({"timestamp": timestamp, "event_type": "rx_raw", "raw": raw_line})

    def log_tx(self, timestamp: str, cmd: str):
        self.log_event({"timestamp": timestamp, "event_type": "tx_cmd", "raw": cmd})

    def _append_xlsx(self, batch_rows: List[dict]):
        wb = load_workbook(self.xlsx_path)
        ws_events = wb["Events"]
        ws_rx = wb["RxRaw"]
        ws_tx = wb["TxCmd"]

        for r in batch_rows:
            et = r.get("event_type", "")
            ts = r.get("timestamp", "")
            raw = r.get("raw", "")

            ws_events.append([r.get(k, "") for k in self.event_headers])

            if et == "rx_raw":
                ws_rx.append([ts, raw])
            elif et == "tx_cmd":
                ws_tx.append([ts, raw])

        wb.save(self.xlsx_path)

    def _append_csv(self, batch_rows: List[dict]):
        with open(self.csv_path, "a", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            for r in batch_rows:
                w.writerow([r.get(k, "") for k in self.event_headers])

    def _worker(self):
        batch: List[dict] = []
        last_flush = time.time()
        FLUSH_EVERY_SEC = 2.0
        FLUSH_EVERY_ROWS = 25

        while not self.stop_flag.is_set():
            try:
                batch.append(self.lq.get(timeout=0.25))
            except queue.Empty:
                pass

            do_flush = False
            if batch and len(batch) >= FLUSH_EVERY_ROWS:
                do_flush = True
            if batch and (time.time() - last_flush) >= FLUSH_EVERY_SEC:
                do_flush = True

            if do_flush:
                try:
                    self._append_csv(batch)
                    self._append_xlsx(batch)
                except Exception:
                    # fallback to CSV only
                    try:
                        self._append_csv(batch)
                    except Exception:
                        pass
                batch.clear()
                last_flush = time.time()

    def stop(self):
        self.stop_flag.set()

def now_iso():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

def log_telemetry(logger: AsyncLogger, model: AppModel, ui: Dict[str, Any]) -> None:
    t = model.telemetry
    if not t.ok:
        return

    logger.log_event({
        "timestamp": ui.get("timestamp", ""),
        "event_type": "telemetry",
        "raw": "",

        "C": t.C,
        "F": t.F,
        "flowLpm": t.flowLpm,
        "turbidity_pct": t.turbidity_pct,
        "NTU": t.NTU,

        "heaterEnable": t.heaterEnable,
        "setTempF": t.setTempF,
        "heaterDemand": t.heaterDemand,
        "heaterOn": t.heaterOn,
        "overTemp": t.overTemp,
        "secOver": t.secOver,
        "heaterStop": t.heaterStop,
        "levelOk": t.levelOk,
        "lidClosed": t.lidClosed,

        "ui_heater_enable": ui.get("ui_heater_enable"),
        "ui_cycle_state": ui.get("ui_cycle_state"),
        "ui_cycle_remaining_s": ui.get("ui_cycle_remaining_s"),
        "ui_cycle_duration_s": ui.get("ui_cycle_duration_s"),
        "ui_cycle_temp_set_f": ui.get("ui_cycle_temp_set_f"),
    })

def log_cycle_snapshot(logger: AsyncLogger, ui: Dict[str, Any]) -> None:
    logger.log_event({
        "timestamp": now_iso(),
        "event_type": "status",
        "raw": "UI cycle state update",
        "ui_cycle_state": ui.cycle_state.get(),
        "ui_cycle_remaining_s": ui.cycle_remaining_s.get(),
        "ui_cycle_duration_s": int(ui.cycle_duration_min_in.get() * 60),
        "ui_cycle_temp_set_f": float(ui.cycle_temp_set_f_in.get()),
    })