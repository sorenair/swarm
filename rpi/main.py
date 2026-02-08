# main.py
import queue, time, os
from datetime import datetime
import customtkinter as ctk

from app.config import PORT, BAUD, ENABLE_FAULT_SCREEN, RX_TIMEOUT_S, LOG_DIR
from app.comms import SerialClient
from app.state import AppModel
from app.protocol import parse_line, get, apply_message
from app.log_async import AsyncLogger, log_telemetry, log_cycle_snapshot, now_iso
from app.controller import Controller
from app.ui_layout import build_ui

# -------
# Logging
# -------
os.makedirs(LOG_DIR, exist_ok=True)

def log_base_name():
    return f"swarm_log_{datetime.now().strftime('%Y-%m-%d')}"

XLSX_PATH = os.path.join(LOG_DIR, log_base_name() + ".xlsx")
CSV_PATH  = os.path.join(LOG_DIR, log_base_name() + ".csv")
EVENT_HEADERS = [
    "timestamp",
    "event_type",
    "raw",
    "C", "F", "flowLpm", "turbidity_pct", "NTU",
    "heaterEnable", "setTempF", "heaterDemand", "heaterOn",
    "overTemp", "secOver", "heaterStop",
    "levelOk", "lidClosed",
    "ui_heater_enable",
    "ui_cycle_state", "ui_cycle_remaining_s", "ui_cycle_duration_s", "ui_cycle_temp_set_f",
]

logger = AsyncLogger(XLSX_PATH, CSV_PATH, EVENT_HEADERS)

# --------
# Main App
# --------
def main():
    # --------------
    # Instantiate UI
    # --------------
    ctk.set_appearance_mode("light")
    ctk.set_default_color_theme("blue")

    app = ctk.CTk()
    app.update_idletasks()
    app.attributes("-fullscreen", True)
    app.title("SWARM")

    ui = build_ui(app)

    # ------------------------------------------
    # Open Serial Client + Machine Controller
    # ------------------------------------------
    q = queue.Queue()

    global ser_client
    ser_client = SerialClient(PORT, BAUD, q)
    ser_client.start()

    controller = Controller(
        send_line=ser_client.send_line,
        log_tx=lambda cmd: logger.log_tx(now_iso(), cmd),
    )

    # Communication Monitoring
    def comms_watchdog():
        if not ENABLE_FAULT_SCREEN:
            ui.hide_fault_overlay()
            app.after(250, comms_watchdog)
            return

        now = time.time()
        timed_out = (ser_client.last_rx_time > 0.0) and ((now - ser_client.last_rx_time) > RX_TIMEOUT_S)

        if (not ser_client.connected) or timed_out:
            if (not ser_client.connected) and ser_client.last_disconnect_reason:
                reason = f"ESP32 disconnected, {ser_client.last_disconnect_reason}"
            elif timed_out:
                reason = f"No telemetry received for {RX_TIMEOUT_S:.1f} seconds"
            else:
                reason = "Communication fault"
            ui.show_fault_overlay(reason)
        else:
            ui.hide_fault_overlay()

        app.after(250, comms_watchdog)

    # ------------------------
    # Connect UI to Controller
    # ------------------------
    def on_close(_=None):
        try:
            logger.stop()
        except:
            pass
        try:
            ser_client.stop()
        except:
            pass

        app.destroy()

    def start_cycle():
        if ui.cycle_state.get() in ("WASHING", "PAUSED"):
            return
        log_cycle_snapshot(logger, ui)
        duration_s = int(float(ui.cycle_duration_min_in.get()) * 60)
        temp_f = float(ui.cycle_temp_set_f_in.get())
        controller.start_cycle(duration_s, temp_f)

    def pause_resume_cycle():
        log_cycle_snapshot(logger, ui)
        if ui.cycle_state.get() == "PAUSED":
            controller.resume_cycle()
        else:
            controller.pause_cycle()

    def stop_cycle():
        if ui.cycle_state.get() in ("IDLE", "COMPLETE"):
            return
        log_cycle_snapshot(logger, ui)
        ui.cycle_remaining_s.set(0)
        controller.stop_cycle()

    def on_heater_toggle():
        log_cycle_snapshot(logger, ui)
        controller.set_heater_enable(bool(ui.heaterEnableVar.get()))

    app.bind("<Escape>", on_close)
    ui.btn_start.configure(command=start_cycle)
    ui.btn_pause.configure(command=pause_resume_cycle)
    ui.btn_stop.configure(command=stop_cycle)
    ui.heater_switch.configure(command=on_heater_toggle)

    # ------------------------------------------
    # Store Machine State in Model and Update UI
    # ------------------------------------------
    model = AppModel()

    def render_from_model(model: AppModel):
        t = model.telemetry
        if not t.ok:
            return

        ui.tempNowF.set(f"{t.F:.1f} °F")
        ui.flow.set(f"{t.flowLpm:.2f} L/min")
        ui.turbPct.set(f"{t.turbidity_pct:.1f} %")
        ui.ntu.set(f"{t.NTU:.1f} NTU")

        if t.setTempF is not None:
            ui.tempSetF.set(f"Set: {float(t.setTempF):.1f} °F")
        else:
            ui.tempSetF.set("Set: —")

        # cycle
        if isinstance(t.cycleState, str) and t.cycleState:
            ui.cycle_state.set(t.cycleState)
        if t.cycleRemainingS is not None:
            try:
                ui.cycle_remaining_s.set(int(float(t.cycleRemainingS)))
            except Exception:
                pass
        #if t.cycleTempSetF is not None:
        #    try:
        #        ui.cycle_temp_set_f_in.set(float(t.cycleTempSetF))
        #    except Exception:
        #        pass

        # lid logic (keep your existing behavior)
        if isinstance(t.lidClosed, bool):
            ui.lidText.set("Lid is closed" if t.lidClosed else "Lid is open")
            if (not t.lidClosed) and ui.heaterEnableVar.get():
                ui.heaterEnableVar.set(False)
        else:
            ui.lidText.set("Lid status error!")

        # level
        if t.levelOk is None:
            ui.levelText.set("—")
        elif t.levelOk:
            ui.levelText.set("OK")
        else:
            ui.levelText.set("LOW / FAULT")

        # heater enable (ESP32 authoritative)
        if isinstance(t.heaterEnable, bool):
            ui.heaterEnableVar.set(t.heaterEnable)

        status_parts = []
        if isinstance(t.heaterEnable, bool):
            status_parts.append("Enabled" if t.heaterEnable else "Disabled")
        if isinstance(t.heaterDemand, bool):
            status_parts.append(f"Demand: {'ON' if t.heaterDemand else 'OFF'}")
        if isinstance(t.heaterOn, bool):
            status_parts.append(f"Output: {'ON' if t.heaterOn else 'OFF'}")

        ui.heaterStatusText.set(" | ".join(status_parts) if status_parts else "—")
        ui.heaterOutText.set(
            f"Output: {'ON' if t.heaterOn else 'OFF'}" if isinstance(t.heaterOn, bool) else "Output: —"
        )

        timeLeft = 21 - t.secOver if isinstance(t.secOver, (int, float)) else None

        if t.heaterStop:
            ui.heaterWarnText.set("SHUTDOWN: Operating temp too high. Resume cycle once temp is below 120°F.")
        elif t.overTemp:
            ui.heaterWarnText.set(
                f"WARNING: Temp exceeded 120°F. System will shut down in {timeLeft} sec if system does not cool"
                if timeLeft is not None else
                "WARNING: ≥120°F"
            )
        else:
            ui.heaterWarnText.set("")

        ui.refresh_operation_panel()

    # ------------------------------
    # Poll Serial Queue and Log Data
    # ------------------------------
    def poll_queue():
        try:
            while True:
                kind, payload = q.get_nowait()

                if kind == "status":
                    ui.title_status.set(payload)
                    continue

                if kind != "data":
                    continue

                m = parse_line(payload)
                if m is None:
                    ui.title_status.set("Bad JSON")
                    logger.log_status(now_iso(), f"Bad JSON: {payload[:200]}")
                    continue

                apply_message(model, m)

                if model.status_line:
                    ui.title_status.set(model.status_line)

                render_from_model(model)

                # logging uses model (same structure as before)
                t = model.telemetry
                if t.ok:
                    logger.log_event({
                        "timestamp": now_iso(),
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
                        "ui_heater_enable": ui.heaterEnableVar.get(),
                        "ui_cycle_state": ui.cycle_state.get(),
                        "ui_cycle_remaining_s": ui.cycle_remaining_s.get(),
                        "ui_cycle_duration_s": int(ui.cycle_duration_min_in.get() * 60),
                        "ui_cycle_temp_set_f": float(ui.cycle_temp_set_f_in.get()),
                    })

        except queue.Empty:
            pass

        app.after(200, poll_queue)

    # --------------------
    # App Loop Initiations
    # --------------------
    app.after(200, poll_queue)

    app.after(250, comms_watchdog)

    app.mainloop()

if __name__ == "__main__":
    main()
