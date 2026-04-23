"""Main entry point for the SWARM Raspberry Pi UI application."""
import queue, time, os
from datetime import datetime
import customtkinter as ctk

from app.config import (
    PORT,
    BAUD,
    ENABLE_FAULT_SCREEN,
    RX_TIMEOUT_S,
    LOG_MAX_BYTES,
    LOG_PRUNE_TARGET_BYTES,
    resolve_log_dir,
)
from app.comms import SerialClient
from app.state import AppModel
from app.protocol import parse_line, apply_message
from app.log_async import AsyncLogger, log_telemetry, log_cycle_snapshot, now_iso
from app.controller import Controller
from app.ui_layout import build_ui

# -------
# Logging
# -------
LOG_DIR = resolve_log_dir()
os.makedirs(LOG_DIR, exist_ok=True)

def log_base_name():
    """Return the base filename for daily log artifacts."""
    return f"swarm_log_{datetime.now().strftime('%Y-%m-%d')}"

XLSX_PATH = os.path.join(LOG_DIR, log_base_name() + ".xlsx")
CSV_PATH  = os.path.join(LOG_DIR, log_base_name() + ".csv")
EVENT_HEADERS = [
    "timestamp",
    "event_type",
    "raw",
    "cycle_state", "cycle_duration_s", "cycle_remaining_s",
    "setTempF",
    "F", "flowLpm", "turbidity_pct", "NTU",
    "heaterEnable", "heaterDemand", "heaterOn",
    "overTemp", "secOver", "heaterStop",
    "levelOk", "lidClosed",
    "active_faults",
]

logger = AsyncLogger(
    XLSX_PATH,
    CSV_PATH,
    EVENT_HEADERS,
    max_dir_bytes=LOG_MAX_BYTES,
    prune_target_bytes=LOG_PRUNE_TARGET_BYTES,
)

FAULT_DISPLAY = {
    "HEATER_SHUTDOWN": {
        "title": "Heater shutdown",
        "detail": "Temperature remained above the controller limit long enough to force shutdown.",
    },
    "FLOW_LOW": {
        "title": "Low flow",
        "detail": "Flow stayed below the controller threshold.",
    },
    "OVERTEMP": {
        "title": "Overtemperature",
        "detail": "Temperature exceeded the controller threshold.",
    },
    "LOW_LEVEL": {
        "title": "Low liquid level",
        "detail": "Refill required before continuing.",
    },
    "LID_OPEN": {
        "title": "Lid is open",
        "detail": "Close lid to continue operation.",
    },
    "HIGH_TURBIDITY": {
        "title": "High turbidity",
        "detail": "Turbidity exceeded the controller threshold.",
    },
    "TEMP_SENSOR": {
        "title": "Temperature sensor fault",
        "detail": "Controller is not receiving a valid temperature reading.",
    },
}

# --------
# Main App
# --------
def main():
    """Initialize the UI, comms, controller, and event loops."""
    # --------------
    # Instantiate UI
    # --------------
    ctk.set_appearance_mode("light")
    ctk.set_default_color_theme("blue")

    app = ctk.CTk()
    app.update_idletasks()
    app.attributes("-fullscreen", True)
    #app.geometry("800x480")
    app.title("SWARM")

    ui = build_ui(app)

    # ------------------------------------------
    # Serial Client + Machine Controller
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
        now = time.time()
        timed_out = (ser_client.last_rx_time > 0.0) and ((now - ser_client.last_rx_time) > RX_TIMEOUT_S)

        if (not ser_client.connected) or timed_out:
            if (not ser_client.connected) and ser_client.last_disconnect_reason:
                reason = f"ESP32 disconnected, {ser_client.last_disconnect_reason}"
            elif timed_out:
                reason = f"No telemetry received for {RX_TIMEOUT_S:.1f} seconds"
            else:
                reason = "Communication fault"
            #ui.show_fault_overlay(reason)
        else:
            #ui.hide_fault_overlay()
            pass

        app.after(250, comms_watchdog)

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

    # ------------------------
    # Connect UI to Controller
    # ------------------------
    def start_cycle():
        if ui.cycle_state.get() in ("PREHEATING", "WASHING", "PAUSED"):
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

    def controller_faults_for_display(t):
        """Return ESP32-declared faults formatted for the UI."""
        faults = []
        for code in t.activeFaults or []:
            meta = FAULT_DISPLAY.get(code, {})
            faults.append({
                "code": code,
                "title": meta.get("title", code.replace("_", " ").title()),
                "detail": meta.get("detail", "Controller reported an active fault."),
            })
        return faults

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

        if isinstance(t.lidClosed, bool):
            ui.lidText.set("Lid is closed" if t.lidClosed else "Lid is open")

        if isinstance(t.levelOk, bool):
            ui.levelText.set("OK" if t.levelOk else "LOW / FAULT")

        # cycle
        if isinstance(t.cycleState, str) and t.cycleState:
            ui.cycle_state.set(t.cycleState)
        if t.cycleRemainingS is not None:
            try:
                ui.cycle_remaining_s.set(int(float(t.cycleRemainingS)))
            except Exception:
                pass

        handle_faults(t, ui)

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

        # Overtemp shutoff
        over_temp_limit_s = int(t.overTempLimitS) if isinstance(t.overTempLimitS, (int, float)) else None
        over_temp_threshold_f = float(t.overTempThresholdF) if isinstance(t.overTempThresholdF, (int, float)) else None
        timeLeft = None
        if over_temp_limit_s is not None and isinstance(t.secOver, (int, float)):
            timeLeft = max(0, over_temp_limit_s - int(t.secOver))

        if t.heaterStop:
            if over_temp_threshold_f is not None:
                ui.heaterWarnText.set(
                    f"SHUTDOWN: Operating temp too high. Resume cycle once temp is below {over_temp_threshold_f:.1f}°F."
                )
            else:
                ui.heaterWarnText.set("SHUTDOWN: Operating temp too high.")
        elif t.overTemp:
            threshold_text = f"{over_temp_threshold_f:.1f}°F" if over_temp_threshold_f is not None else "threshold"
            ui.heaterWarnText.set(
                f"WARNING: Temp exceeded {threshold_text}. System will shut down in {timeLeft} sec if system does not cool."
                if timeLeft is not None else
                f"WARNING: Temperature exceeded {threshold_text}."
            )
        else:
            ui.heaterWarnText.set("")
            
        #Flow fault shutoff
        flow_warn = ""
        if isinstance(t.flowFault, bool) and t.flowFault:
            flow_warn = "FLOW FAULT: Flow rate critically low. Please service immediately"
        
        elif isinstance(t.flowLow, bool) and t.flowLow:
            flow_limit_s = int(t.flowLowLimitS) if isinstance(t.flowLowLimitS, (int, float)) else None
            flow_time_left = None
            if flow_limit_s is not None:
                flow_time_left = max(0, flow_limit_s - int(t.secLowFlow or 0))
            if flow_time_left is not None:
                flow_warn = (
                    f"WARNING: Low flow detected. System will shut off in {flow_time_left} second(s) if flow rate does not improve."
                )
            else:
                flow_warn = "WARNING: Low flow detected."
            
        #Combine heater and flow warnings if simultaneous
        heater_warn = ui.heaterWarnText.get()
        if heater_warn and flow_warn:
            ui.heaterWarnText.set(heater_warn + " | " + flow_warn)
            
        elif flow_warn:
            ui.heaterWarnText.set(flow_warn)

        ui.refresh_operation_panel()

    active_faults = []

    last_faults_snapshot = None

    def handle_faults(t, ui):
        """Apply fault handling based on the latest telemetry."""
        nonlocal last_faults_snapshot, active_faults
        faults = controller_faults_for_display(t)
        active_faults = [fault.get("code", "") for fault in faults]

        if "LID_OPEN" in active_faults and ui.heaterEnableVar.get():
            ui.heaterEnableVar.set(False)

        snapshot = tuple(sorted((f.get("code", ""), f.get("title", ""), f.get("detail", "")) for f in faults))

        if snapshot == last_faults_snapshot:
            return

        last_faults_snapshot = snapshot

        if not faults:
            ui.hide_fault_overlay()
            return

        ui.show_fault_overlay(faults)

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
                        # Metadata
                        "timestamp": now_iso(),
                        "event_type": "telemetry",
                        "raw": "",
                        # Cycle state fields
                        "cycle_state": ui.cycle_state.get(),
                        "cycle_duration_s": int(ui.cycle_duration_min_in.get() * 60),
                        "cycle_remaining_s": ui.cycle_remaining_s.get(),
                        #"cycle_temp_set_f": float(ui.cycle_temp_set_f_in.get())
                        "setTempF": t.setTempF,
                        # Telemetry fields
                        "F": t.F,
                        "flowLpm": t.flowLpm,
                        "turbidity_pct": t.turbidity_pct,
                        "NTU": t.NTU,
                        # Heater state fields
                        "heaterEnable": t.heaterEnable,
                        #"ui_heater_enable": ui.heaterEnableVar.get(),
                        "heaterDemand": t.heaterDemand,
                        "heaterOn": t.heaterOn,
                        "heaterStop": t.heaterStop,
                        # Fault fields
                        "overTemp": t.overTemp,
                        "secOver": t.secOver,
                        "levelOk": t.levelOk,
                        "lidClosed": t.lidClosed,
                        "active_faults": ",".join(active_faults) if active_faults else "",
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
