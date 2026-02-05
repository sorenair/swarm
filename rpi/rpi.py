# ui_modern_fullscreen.py
import json, threading, queue, serial, time, os, csv
from datetime import datetime
import customtkinter as ctk

# Excel logging
from openpyxl import Workbook, load_workbook
from openpyxl.utils import get_column_letter

PORT = "/dev/ttyUSB0"          # set to /dev/serial/by-id/... for stability
BAUD = 115200
q = queue.Queue()

ser = None
ser_lock = threading.Lock()

# --------------------------
# Logging
# --------------------------
LOG_DIR = "logs"
os.makedirs(LOG_DIR, exist_ok=True)

def now_iso():
    # local time, ISO-ish
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

def log_base_name():
    return f"swarm_log_{datetime.now().strftime('%Y-%m-%d')}"

XLSX_PATH = os.path.join(LOG_DIR, log_base_name() + ".xlsx")
CSV_PATH  = os.path.join(LOG_DIR, log_base_name() + ".csv")

EVENT_HEADERS = [
    "timestamp",
    "event_type",      # status | rx_raw | tx_cmd | telemetry
    "raw",             # raw line or command (if applicable)
    # telemetry fields (blank for non-telemetry events)
    "C", "F", "flowLpm", "turbidity_pct", "NTU",
    "heaterEnable", "setTempF", "heaterDemand", "heaterOn",
    "overTemp", "secOver", "heaterStop",
    "levelOk", "lidClosed",
    # UI-side actuator controls (what the Pi UI currently asked for)
    "ui_motor_speed_pct", "ui_motor_dir", "ui_heater_enable",
]

class AsyncLogger:
    """
    Threaded logger: UI/serial threads enqueue events, logger flushes to disk.
    Writes a single row schema to CSV and to Excel (Events sheet).
    Also writes Tx/Rx sheets for convenience.
    """
    def __init__(self, xlsx_path: str, csv_path: str):
        self.xlsx_path = xlsx_path
        self.csv_path = csv_path
        self.lq = queue.Queue()
        self.stop_flag = threading.Event()

        self._init_csv()
        self._init_xlsx()

        self.t = threading.Thread(target=self._worker, daemon=True)
        self.t.start()

    def _init_csv(self):
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(EVENT_HEADERS)

    def _init_xlsx(self):
        if os.path.exists(self.xlsx_path):
            return
        wb = Workbook()
        ws_events = wb.active
        ws_events.title = "Events"
        ws_events.append(EVENT_HEADERS)

        ws_rx = wb.create_sheet("RxRaw")
        ws_rx.append(["timestamp", "raw_line"])

        ws_tx = wb.create_sheet("TxCmd")
        ws_tx.append(["timestamp", "command"])

        # widen columns a bit
        for ws in (ws_events, ws_rx, ws_tx):
            for i, h in enumerate(ws[1], start=1):
                ws.column_dimensions[get_column_letter(i)].width = max(12, min(38, len(str(h)) + 2))

        wb.save(self.xlsx_path)

    def log_event(self, row: dict):
        # row is dict with keys in EVENT_HEADERS (missing keys allowed)
        self.lq.put(row)

    def _append_xlsx(self, batch_rows: list[dict]):
        # load/save each batch (simple + reliable for low rates)
        wb = load_workbook(self.xlsx_path)
        ws_events = wb["Events"]
        ws_rx = wb["RxRaw"]
        ws_tx = wb["TxCmd"]

        for r in batch_rows:
            et = r.get("event_type", "")
            ts = r.get("timestamp", "")
            raw = r.get("raw", "")

            # main Events row
            ws_events.append([r.get(k, "") for k in EVENT_HEADERS])

            # convenience sheets
            if et == "rx_raw":
                ws_rx.append([ts, raw])
            elif et == "tx_cmd":
                ws_tx.append([ts, raw])

        wb.save(self.xlsx_path)

    def _append_csv(self, batch_rows: list[dict]):
        with open(self.csv_path, "a", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            for r in batch_rows:
                w.writerow([r.get(k, "") for k in EVENT_HEADERS])

    def _worker(self):
        batch = []
        last_flush = time.time()
        FLUSH_EVERY_SEC = 2.0
        FLUSH_EVERY_ROWS = 25

        while not self.stop_flag.is_set():
            try:
                r = self.lq.get(timeout=0.25)
                batch.append(r)
            except queue.Empty:
                pass

            do_flush = False
            if batch and (len(batch) >= FLUSH_EVERY_ROWS):
                do_flush = True
            if batch and (time.time() - last_flush >= FLUSH_EVERY_SEC):
                do_flush = True

            if do_flush:
                try:
                    self._append_csv(batch)
                    self._append_xlsx(batch)
                except Exception:
                    # Worst case: CSV likely still works; avoid crashing the app
                    try:
                        self._append_csv(batch)
                    except Exception:
                        pass
                batch.clear()
                last_flush = time.time()

    def stop(self):
        self.stop_flag.set()

logger = AsyncLogger(XLSX_PATH, CSV_PATH)

def log_status(msg: str):
    logger.log_event({
        "timestamp": now_iso(),
        "event_type": "status",
        "raw": msg,
    })

def log_rx(raw_line: str):
    logger.log_event({
        "timestamp": now_iso(),
        "event_type": "rx_raw",
        "raw": raw_line,
    })

def log_tx(cmd: str):
    logger.log_event({
        "timestamp": now_iso(),
        "event_type": "tx_cmd",
        "raw": cmd,
    })


# --------------------------
# Serial thread
# --------------------------
def reader():
    global ser
    while True:
        try:
            ser = serial.Serial(PORT, BAUD, timeout=1)
            q.put(("status", f"Connected: {PORT}"))
            log_status(f"Connected: {PORT}")

            while True:
                line = ser.readline().decode("utf-8","ignore").strip()
                if line:
                    q.put(("data", line))
                    log_rx(line)
        except Exception as e:
            q.put(("status", f"Disconnected: {e}"))
            log_status(f"Disconnected: {e}")
            try:
                if ser:
                    ser.close()
            except:
                pass
            ser = None
            time.sleep(1)

def send(cmd):
    global ser
    # always log intent to send
    log_tx(cmd)
    try:
        with ser_lock:
            if ser and ser.is_open:
                ser.write((cmd + "\n").encode("utf-8"))
            else:
                q.put(("status", "Send failed: serial not connected"))
                log_status("Send failed: serial not connected")
    except Exception as e:
        q.put(("status", f"Send failed: {e}"))
        log_status(f"Send failed: {e}")


def main():
    ctk.set_appearance_mode("light")
    ctk.set_default_color_theme("blue")

    app = ctk.CTk()
    app.update_idletasks()
    app.attributes("-fullscreen", True)        # hard fullscreen
    app.title("SWARM")

    def on_close(_=None):
        try:
            logger.stop()
        except:
            pass
        app.destroy()

    app.bind("<Escape>", on_close)

    # Root layout
    root = ctk.CTkFrame(app, corner_radius=0, fg_color="white")
    root.pack(fill="both", expand=True)

    # Header
    header = ctk.CTkFrame(root, corner_radius=0, fg_color="white")
    header.pack(fill="x", padx=20, pady=(16,8))
    ctk.CTkLabel(header, text="SWARM", text_color="black",
                 font=("SF Pro Display", 28, "bold")).pack(side="left")
    title_status = ctk.StringVar(value="")
    ctk.CTkLabel(header, textvariable=title_status, text_color="black",
                 font=("SF Pro Text", 14)).pack(side="right")

    # Content area
    content = ctk.CTkFrame(root, corner_radius=0, fg_color="white")
    content.pack(fill="both", expand=True, padx=20, pady=8)
    content.grid_columnconfigure((0,1), weight=1)
    content.grid_rowconfigure((0,1), weight=1)

    # temp card
    tempCard = ctk.CTkFrame(content, corner_radius=16)
    tempCard.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
    ctk.CTkLabel(tempCard, text="Temperature", font=("SF Pro Text",16,"bold")).grid(row=0, column=0, sticky="w", padx=12, pady=(12,4))
    tempC = ctk.StringVar(value="—")
    tempF = ctk.StringVar(value="—")
    ctk.CTkLabel(tempCard, textvariable=tempC, font=("SF Pro Text", 20)).grid(row=1, column=0, sticky="w", padx=12, pady=4)
    ctk.CTkLabel(tempCard, textvariable=tempF, font=("SF Pro Text", 20)).grid(row=2, column=0, sticky="w", padx=12, pady=(0, 12))

    # flow card
    flowCard = ctk.CTkFrame(content, corner_radius=16)
    flowCard.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)
    ctk.CTkLabel(flowCard, text="Flow Rate (L/min)", font=("SF Pro Text",16,"bold")).grid(row=0, column=1, sticky="w", padx=12, pady=(12,4))
    flow = ctk.StringVar(value="—")
    ctk.CTkLabel(flowCard, textvariable=flow, font=("SF Pro Text", 20)).grid(row=1, column=0, sticky="w", padx=12, pady=(4, 12))

    # turbidity card
    turbCard = ctk.CTkFrame(content, corner_radius=16)
    turbCard.grid(row=1, column=0, sticky="nsew", padx=10, pady=10)
    ctk.CTkLabel(turbCard, text="Turbidity", font=("SF Pro Text",16,"bold")).grid(row=0, column=0, sticky="w", padx=12, pady=(12,4))
    turbPct = ctk.StringVar(value="—")
    ntu = ctk.StringVar(value="—")
    ctk.CTkLabel(turbCard, textvariable=turbPct, font=("SF Pro Text", 24)).grid(row=1, column=0, sticky="w", padx=12, pady=4)
    ctk.CTkLabel(turbCard, textvariable=ntu, font=("SF Pro Text", 14), text_color="gray40").grid(row=2, column=0, sticky="w", padx=12, pady=(0, 12))

    # float / liquid level card
    levelCard = ctk.CTkFrame(content, corner_radius=16)
    levelCard.grid(row=1, column=1, sticky="nsew", padx=10, pady=10)
    ctk.CTkLabel(levelCard, text="Liquid Level", font=("SF Pro Text",16,"bold")).grid(row=0, column=0, sticky="w", padx=12, pady=(12,4))
    levelText = ctk.StringVar(value="—")
    ctk.CTkLabel(levelCard, textvariable=levelText, font=("SF Pro Text", 20)).grid(row=1, column=0, sticky="w", padx=12, pady=(4,12))

    # motor card
    motorCard = ctk.CTkFrame(content, corner_radius=16)
    motorCard.grid(row=2, column=0, columnspan=2, sticky="nsew", padx=10, pady=10)
    ctk.CTkLabel(motorCard, text="Motor", font=("SF Pro Text",16,"bold")).grid(row=0, column=0, sticky="w", padx=12, pady=(12,4))
    dirv = ctk.StringVar(value="FWD")
    dir_row = ctk.CTkSegmentedButton(motorCard, values=["FWD", "REV"], variable=dirv)
    dir_row.grid(row=0, column=1, sticky="e", padx=12, pady=(12,6))

    spd = ctk.IntVar(value=0)

    def apply_speed(_=None):
        duty = int(spd.get() * 255 / 100)
        if duty <= 0:
            send("MOTOR 0")
        else:
            send(f"MOTOR {duty} {dirv.get()}")

    ctk.CTkSlider(
        motorCard, from_=0, to=100, number_of_steps=100,
        variable=spd, command=lambda _ : None
    ).grid(row=1, column=0, columnspan=3, sticky="we", padx=12, pady=8)
    ctk.CTkButton(motorCard, text="Apply Speed", command=apply_speed).grid(row=2, column=0, columnspan=3, padx=12, pady=(4,12))

    # heater card
    heaterCard = ctk.CTkFrame(content, corner_radius=16)
    heaterCard.grid(row=3, column=0, columnspan=2, sticky="nsew", padx=10, pady=10)
    ctk.CTkLabel(heaterCard, text="Heater", font=("SF Pro Text",16,"bold")).grid(row=0, column=0, sticky="w", padx=12, pady=(12,6))

    heaterEnableVar = ctk.BooleanVar(value=False)
    heaterStatusText = ctk.StringVar(value="Disabled")
    heaterSetText = ctk.StringVar(value="Set: —")
    heaterOutText = ctk.StringVar(value="Output: —")
    heaterWarnText = ctk.StringVar(value="")
    lidText = ctk.StringVar(value="Lid: —")

    def on_heater_toggle():
        send(f"HEATER {1 if heaterEnableVar.get() else 0}")

    ctk.CTkSwitch(heaterCard, text="Enable", variable=heaterEnableVar, command=on_heater_toggle).grid(row=0, column=1, sticky="e", padx=12, pady=(12,6))
    ctk.CTkLabel(heaterCard, textvariable=lidText, font=("SF Pro Text", 14), text_color="gray40").grid(row=1, column=1, sticky="e", padx=12, pady=(0,4))
    ctk.CTkLabel(heaterCard, textvariable=heaterStatusText, font=("SF Pro Text", 14)).grid(row=1, column=0, sticky="w", padx=12, pady=(0,4))
    ctk.CTkLabel(heaterCard, textvariable=heaterSetText, font=("SF Pro Text", 14), text_color="gray40").grid(row=2, column=0, sticky="w", padx=12, pady=(0,4))
    ctk.CTkLabel(heaterCard, textvariable=heaterOutText, font=("SF Pro Text", 14), text_color="gray40").grid(row=3, column=0, sticky="w", padx=12, pady=(0,8))
    ctk.CTkLabel(heaterCard, textvariable=heaterWarnText, font=("SF Pro Text", 14, "bold"), text_color="red").grid(row=4, column=0, columnspan=2, sticky="w", padx=12, pady=(0,12))
    heaterCard.grid_columnconfigure(0, weight=1)

    # Queue pump
    def poll_queue():
        try:
            while True:
                kind, payload = q.get_nowait()
                if kind == "status":
                    title_status.set(payload)
                    # status already logged via log_status where produced
                elif kind == "data":
                    try:
                        m = json.loads(payload)
                        if m.get("ok"):
                            # UI updates
                            tempC.set(f"{m.get('C', 0):.2f} C")
                            tempF.set(f"{m.get('F', 0):.2f} F")
                            flow.set(f"{m.get('flowLpm', 0):.2f} L/min")
                            turbPct.set(f"{m.get('% Turbidity', 0):.1f} %")
                            ntu.set(f"{m.get('NTU', 0):.1f} NTU")

                            h_en   = m.get("heaterEnable", None)
                            h_set  = m.get("setTempF", None)
                            h_dem  = m.get("heaterDemand", None)
                            h_on   = m.get("heaterOn", None)
                            h_over = m.get("overTemp", None)
                            h_sec  = m.get("secOver", None)
                            h_stop = m.get("heaterStop", None)
                            lvl = m.get("levelOk", None)
                            lid_closed = m.get("lidClosed", None)

                            # lid interlock
                            if isinstance(lid_closed, bool):
                                lidText.set("Lid is closed" if lid_closed else "Lid is open")
                                if not lid_closed and heaterEnableVar.get():
                                    heaterEnableVar.set(False)
                            else:
                                lidText.set("Lid status error!")

                            # float level
                            if lvl is None:
                                levelText.set("—")
                            elif lvl:
                                levelText.set("OK")
                            else:
                                levelText.set("LOW / FAULT")

                            # keep GUI switch synced with ESP32
                            if isinstance(h_en, bool):
                                heaterEnableVar.set(h_en)

                            if h_set is not None:
                                heaterSetText.set(f"Temp set to: {float(h_set):.1f} °F")
                            else:
                                heaterSetText.set("Temp set to: —")

                            status_parts = []
                            if isinstance(h_en, bool):
                                status_parts.append("Enabled" if h_en else "Disabled")
                            if isinstance(h_dem, bool):
                                status_parts.append(f"Demand: {'ON' if h_dem else 'OFF'}")
                            if isinstance(h_on, bool):
                                status_parts.append(f"Output: {'ON' if h_on else 'OFF'}")

                            heaterStatusText.set(" | ".join(status_parts) if status_parts else "—")
                            heaterOutText.set(f"Output: {'ON' if h_on else 'OFF'}" if isinstance(h_on, bool) else "Output: —")
                            timeLeft = 21 - h_sec if isinstance(h_sec, (int, float)) else None

                            # warnings
                            if h_stop:
                                heaterWarnText.set("SHUTDOWN: Operating temp too high. Resume cycle once temp is below 120°F.")
                            elif h_over:
                                heaterWarnText.set(
                                    f"WARNING: Temp exceeded 120°F. System will shut down in {timeLeft} sec if system does not cool"
                                    if timeLeft is not None else
                                    "WARNING: ≥120°F"
                                )
                            else:
                                heaterWarnText.set("")

                            # ---- Telemetry log row (structured) ----
                            logger.log_event({
                                "timestamp": now_iso(),
                                "event_type": "telemetry",
                                "raw": "",

                                "C": m.get("C", ""),
                                "F": m.get("F", ""),
                                "flowLpm": m.get("flowLpm", ""),
                                "turbidity_pct": m.get("% Turbidity", ""),
                                "NTU": m.get("NTU", ""),

                                "heaterEnable": h_en,
                                "setTempF": h_set,
                                "heaterDemand": h_dem,
                                "heaterOn": h_on,
                                "overTemp": h_over,
                                "secOver": h_sec,
                                "heaterStop": h_stop,
                                "levelOk": lvl,
                                "lidClosed": lid_closed,

                                # UI-side (what the operator set)
                                "ui_motor_speed_pct": spd.get(),
                                "ui_motor_dir": dirv.get(),
                                "ui_heater_enable": heaterEnableVar.get(),
                            })

                    except json.JSONDecodeError:
                        title_status.set("Bad JSON")
                        log_status(f"Bad JSON: {payload[:200]}")
        except queue.Empty:
            pass
        app.after(200, poll_queue)

    threading.Thread(target=reader, daemon=True).start()
    app.after(200, poll_queue)
    app.mainloop()

if __name__ == "__main__":
    main()
