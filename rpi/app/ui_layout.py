"""UI layout construction for the SWARM control panel."""
from dataclasses import dataclass
from typing import Optional, Callable
import customtkinter as ctk
from app.config import DEFAULT_TEMP_F, DEFAULT_CYCLE_MIN

@dataclass
class UI:
    """Container for UI variables, widgets, and callbacks."""
    app: ctk.CTk

    # top status (header right)
    title_status: ctk.StringVar

    # page / panel controls used by main.py
    show_page: callable
    refresh_operation_panel: callable

    # fault overlay controls
    show_fault_overlay: callable
    hide_fault_overlay: callable

    # operation state vars
    cycle_state: ctk.StringVar
    cycle_remaining_s: ctk.IntVar
    cycle_duration_min_in: ctk.IntVar
    cycle_temp_set_f_in: ctk.DoubleVar

    # telemetry vars
    tempNowF: ctk.StringVar
    tempSetF: ctk.StringVar
    flow: ctk.StringVar
    turbPct: ctk.StringVar
    ntu: ctk.StringVar
    levelText: ctk.StringVar
    lidText: ctk.StringVar

    heaterEnableVar: ctk.BooleanVar
    heaterStatusText: ctk.StringVar
    heaterOutText: ctk.StringVar
    heaterWarnText: ctk.StringVar

    # widgets main will bind commands to
    btn_start: ctk.CTkButton
    btn_pause: ctk.CTkButton
    btn_stop: ctk.CTkButton
    heater_switch: ctk.CTkSwitch

    # widgets main may need to manipulate (optional)
    set_card: ctk.CTkFrame
    time_card: ctk.CTkFrame
    action: ctk.CTkFrame


def build_ui(app: ctk.CTk) -> UI:
    """Build the full UI layout and return bound UI handles."""
    # --------------------------
    # Top-level shell
    # --------------------------
    container = ctk.CTkFrame(app, corner_radius=0, fg_color="white")
    container.pack(fill="both", expand=True)
    container.grid_rowconfigure(0, weight=1)
    container.grid_columnconfigure(1, weight=1)

    nav = ctk.CTkFrame(container, corner_radius=0, fg_color="#F6F7F9", width=220)
    nav.grid(row=0, column=0, sticky="nsw")
    nav.grid_propagate(False)

    main_area = ctk.CTkFrame(container, corner_radius=0, fg_color="white")
    main_area.grid(row=0, column=1, sticky="nsew")
    main_area.grid_rowconfigure(1, weight=1)
    main_area.grid_columnconfigure(0, weight=1)

    header = ctk.CTkFrame(main_area, corner_radius=0, fg_color="white")
    header.grid(row=0, column=0, sticky="ew", padx=20, pady=(16, 8))
    header.grid_columnconfigure(1, weight=1)

    header_title = ctk.StringVar(value="Operation Panel")
    ctk.CTkLabel(
        header, textvariable=header_title, font=("SF Pro Display", 28, "bold"), text_color="black"
    ).grid(row=0, column=0, sticky="w")

    title_status = ctk.StringVar(value="")
    ctk.CTkLabel(header, textvariable=title_status, font=("SF Pro Text", 14), text_color="black").grid(
        row=0, column=2, sticky="e"
    )

    page_container = ctk.CTkFrame(main_area, corner_radius=0, fg_color="white")
    page_container.grid(row=1, column=0, sticky="nsew", padx=20, pady=(0, 16))
    page_container.grid_rowconfigure(0, weight=1)
    page_container.grid_columnconfigure(0, weight=1)

    # --------------------------
    # Pages
    # --------------------------
    page_operation = ctk.CTkFrame(page_container, corner_radius=0, fg_color="white")
    page_status    = ctk.CTkFrame(page_container, corner_radius=0, fg_color="white")
    page_faults    = ctk.CTkFrame(page_container, corner_radius=0, fg_color="white")

    for p in (page_operation, page_status, page_faults):
        p.grid(row=0, column=0, sticky="nsew")

    def show_page(name: str):
        if name == "operation":
            page_operation.tkraise()
            header_title.set("Operation Panel")
        elif name == "status":
            page_status.tkraise()
            header_title.set("System Status")
        else:
            page_faults.tkraise()
            header_title.set("System Faults")

    # --------------------------
    # Faults page
    # --------------------------
    fault_title = ctk.StringVar(value="No active faults")
    fault_subtitle = ctk.StringVar(value="System is operating normally.")

    ctk.CTkLabel(
        page_faults,
        textvariable=fault_title,
        font=("SF Pro Display", 36, "bold"),
        text_color="black",
    ).pack(pady=(48, 8))
    ctk.CTkLabel(
        page_faults,
        textvariable=fault_subtitle,
        font=("SF Pro Text", 16),
        text_color="gray30",
    ).pack(pady=(0, 24))

    fault_list = ctk.CTkFrame(page_faults, fg_color="transparent")
    fault_list.pack(fill="both", expand=True, padx=40, pady=(0, 48))

    def _render_faults(faults):
        for child in fault_list.winfo_children():
            child.destroy()

        for f in faults:
            title = f.get("title", "Fault")
            detail = f.get("detail", "")
            card = ctk.CTkFrame(fault_list, corner_radius=16, fg_color="#F6F7F9")
            card.pack(fill="x", pady=10)
            ctk.CTkLabel(
                card, text=title, font=("SF Pro Display", 22, "bold"), text_color="black"
            ).pack(anchor="w", padx=18, pady=(14, 4))
            if detail:
                ctk.CTkLabel(
                    card, text=detail, font=("SF Pro Text", 14), text_color="gray30", wraplength=900
                ).pack(anchor="w", padx=18, pady=(0, 14))

    def show_fault_overlay(faults):
        count = len(faults)
        if count == 0:
            fault_title.set("No active faults")
            fault_subtitle.set("System is operating normally.")
        else:
            fault_title.set(f"{count} active fault{'s' if count != 1 else ''}")
            fault_subtitle.set("System is disabled until faults are resolved.")
        _render_faults(faults)
        show_page("faults")

    def hide_fault_overlay():
        fault_title.set("No active faults")
        fault_subtitle.set("System is operating normally.")
        _render_faults([])

    # --------------------------
    # Left navigation
    # --------------------------
    ctk.CTkLabel(nav, text="SWARM", font=("SF Pro Text", 48, "bold"), text_color="black").pack(
        anchor="w", padx=16, pady=(18, 8)
    )

    nav_btns = ctk.CTkFrame(nav, corner_radius=0, fg_color="transparent")
    nav_btns.pack(fill="both", expand=True, padx=0, pady=0)
    nav_btns.grid_rowconfigure(0, weight=1)
    nav_btns.grid_rowconfigure(4, weight=1)
    nav_btns.grid_columnconfigure(0, weight=1)

    def nav_button(parent, text, command):
        return ctk.CTkButton(
            parent, text=text, command=command,
            height=64, corner_radius=10,
            fg_color="#FFFFFF", hover_color="#EDEFF3",
            text_color="black", anchor="w"
        )

    ctk.CTkFrame(nav_btns, fg_color="transparent").grid(row=0, column=0, sticky="nsew")
    nav_button(nav_btns, "Operation", lambda: show_page("operation")).grid(row=1, column=0, sticky="ew", padx=12, pady=(0, 10))
    nav_button(nav_btns, "Status",    lambda: show_page("status")).grid(row=2, column=0, sticky="ew", padx=12, pady=(0, 10))
    nav_button(nav_btns, "Faults",    lambda: show_page("faults")).grid(row=3, column=0, sticky="ew", padx=12, pady=(0, 10))
    ctk.CTkFrame(nav_btns, fg_color="transparent").grid(row=4, column=0, sticky="nsew")

    # --------------------------
    # Vars (moved from main)
    # --------------------------
    cycle_state = ctk.StringVar(value="IDLE")
    cycle_remaining_s = ctk.IntVar(value=0)
    cycle_duration_min_in = ctk.IntVar(value=DEFAULT_CYCLE_MIN)
    cycle_temp_set_f_in   = ctk.DoubleVar(value=DEFAULT_TEMP_F)
    last_mode = None    # track for operation panel refresh logic

    # --------------------------
    # Operation Page Layout
    # --------------------------
    page_operation.grid_columnconfigure(0, weight=1)
    page_operation.grid_rowconfigure(0, weight=1)
    page_operation.grid_rowconfigure(1, weight=1)
    page_operation.grid_rowconfigure(2, weight=1)

    status_card = ctk.CTkFrame(page_operation, corner_radius=16, fg_color="#F6F7F9")
    status_card.grid(row=0, column=0, sticky="nsew", pady=(0, 12))
    status_card.grid_columnconfigure(0, weight=1)

    ctk.CTkLabel(status_card, text="Machine Status", font=("SF Pro Text", 16, "bold"), text_color="black").grid(
        row=0, column=0, sticky="w", padx=16, pady=(14, 6)
    )
    ctk.CTkLabel(status_card, textvariable=cycle_state, font=("SF Pro Display", 30, "bold"), text_color="black").grid(
        row=1, column=0, sticky="w", padx=16, pady=(0, 14)
    )

    set_card = ctk.CTkFrame(page_operation, corner_radius=16)
    time_card = ctk.CTkFrame(page_operation, corner_radius=16)

    # Setpoints (touch-friendly)
    set_card.grid_columnconfigure(0, weight=1)
    ctk.CTkLabel(set_card, text="Cycle Configuration", font=("SF Pro Text", 16, "bold")).grid(
        row=0, column=0, sticky="w", padx=14, pady=(14, 10)
    )

    def _open_numeric_popup(
        title: str,
        value_var: ctk.Variable,
        unit: str,
        step: float,
        min_value: Optional[float] = None,
        max_value: Optional[float] = None,
        formatter: Optional[Callable[[float], str]] = None,
    ):
        popup = ctk.CTkToplevel(app)
        popup.attributes("-fullscreen", True)
        popup.configure(fg_color="#FFFFFF")
        popup.grab_set()

        def clamp(v: float) -> float:
            if min_value is not None:
                v = max(min_value, v)
            if max_value is not None:
                v = min(max_value, v)
            return v

        try:
            current_value = float(value_var.get())
        except Exception:
            current_value = 0.0
        current_value = clamp(current_value)

        display_value = ctk.StringVar()

        def format_value(v: float) -> str:
            if formatter:
                return formatter(v)
            if unit:
                return f"{v:g} {unit}"
            return f"{v:g}"

        def update_display():
            display_value.set(format_value(current_value))

        def on_inc():
            nonlocal current_value
            current_value = clamp(current_value + step)
            update_display()

        def on_dec():
            nonlocal current_value
            current_value = clamp(current_value - step)
            update_display()

        def on_set():
            if isinstance(value_var, ctk.IntVar):
                value_var.set(int(round(current_value)))
            else:
                value_var.set(float(current_value))
            popup.grab_release()
            popup.destroy()

        def on_cancel():
            popup.grab_release()
            popup.destroy()

        popup.grid_columnconfigure(0, weight=1)
        popup.grid_rowconfigure(1, weight=1)

        ctk.CTkLabel(
            popup,
            text=title,
            font=("SF Pro Display", 30, "bold"),
            text_color="black",
        ).grid(row=0, column=0, sticky="ew", padx=28, pady=(28, 10))

        value_frame = ctk.CTkFrame(popup, corner_radius=20, fg_color="#F6F7F9")
        value_frame.grid(row=1, column=0, sticky="nsew", padx=28, pady=10)
        value_frame.grid_columnconfigure((0, 2), weight=1)
        value_frame.grid_rowconfigure(0, weight=1)

        btn_style = dict(height=120, width=160, corner_radius=18, fg_color="#FFFFFF", text_color="black")

        ctk.CTkButton(
            value_frame, text="−", font=("SF Pro Display", 64, "bold"), command=on_dec, **btn_style
        ).grid(row=0, column=0, padx=24, pady=24, sticky="nsew")

        ctk.CTkLabel(
            value_frame, textvariable=display_value, font=("SF Pro Display", 54, "bold"), text_color="black"
        ).grid(row=0, column=1, padx=20, pady=24)

        ctk.CTkButton(
            value_frame, text="+", font=("SF Pro Display", 64, "bold"), command=on_inc, **btn_style
        ).grid(row=0, column=2, padx=24, pady=24, sticky="nsew")

        action_frame = ctk.CTkFrame(popup, corner_radius=0, fg_color="white")
        action_frame.grid(row=2, column=0, sticky="ew", padx=28, pady=(10, 28))
        action_frame.grid_columnconfigure((0, 1), weight=1)

        ctk.CTkButton(
            action_frame,
            text="Cancel",
            font=("SF Pro Text", 24, "bold"),
            height=70,
            corner_radius=18,
            fg_color="#EDEFF3",
            text_color="black",
            command=on_cancel,
        ).grid(row=0, column=0, padx=(0, 12), sticky="ew")

        ctk.CTkButton(
            action_frame,
            text="Set",
            font=("SF Pro Text", 24, "bold"),
            height=70,
            corner_radius=18,
            command=on_set,
        ).grid(row=0, column=1, padx=(12, 0), sticky="ew")

        update_display()

    subcards = ctk.CTkFrame(set_card, corner_radius=0, fg_color="transparent")
    subcards.grid(row=1, column=0, sticky="nsew", padx=12, pady=(0, 14))
    subcards.grid_columnconfigure((0, 1), weight=1)

    duration_card = ctk.CTkFrame(subcards, corner_radius=16, fg_color="#F6F7F9")
    duration_card.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
    duration_card.grid_columnconfigure(0, weight=1)

    ctk.CTkLabel(duration_card, text="Duration", font=("SF Pro Text", 14, "bold"), text_color="black").grid(
        row=0, column=0, sticky="w", padx=14, pady=(12, 4)
    )

    duration_text = ctk.StringVar(value=f"{cycle_duration_min_in.get():g} min")
    ctk.CTkLabel(duration_card, textvariable=duration_text, font=("SF Pro Display", 30, "bold"), text_color="black").grid(
        row=1, column=0, sticky="w", padx=14, pady=(0, 10)
    )

    ctk.CTkButton(
        duration_card,
        text="Change",
        font=("SF Pro Text", 18, "bold"),
        height=48,
        corner_radius=12,
        command=lambda: _open_numeric_popup(
            title="Set Cycle Duration",
            value_var=cycle_duration_min_in,
            unit="min",
            step=1.0,
            min_value=0.0,
            formatter=lambda v: f"{int(round(v))} min",
        ),
    ).grid(row=2, column=0, sticky="ew", padx=14, pady=(0, 12))

    temp_card = ctk.CTkFrame(subcards, corner_radius=16, fg_color="#F6F7F9")
    temp_card.grid(row=0, column=1, sticky="nsew", padx=(10, 0))
    temp_card.grid_columnconfigure(0, weight=1)

    ctk.CTkLabel(temp_card, text="Set Temp", font=("SF Pro Text", 14, "bold"), text_color="black").grid(
        row=0, column=0, sticky="w", padx=14, pady=(12, 4)
    )

    temp_text = ctk.StringVar(value=f"{cycle_temp_set_f_in.get():.1f} °F")
    ctk.CTkLabel(temp_card, textvariable=temp_text, font=("SF Pro Display", 30, "bold"), text_color="black").grid(
        row=1, column=0, sticky="w", padx=14, pady=(0, 10)
    )

    ctk.CTkButton(
        temp_card,
        text="Change",
        font=("SF Pro Text", 18, "bold"),
        height=48,
        corner_radius=12,
        command=lambda: _open_numeric_popup(
            title="Set Temperature",
            value_var=cycle_temp_set_f_in,
            unit="°F",
            step=1.0,
            min_value=0.0,
            formatter=lambda v: f"{v:.1f} °F",
        ),
    ).grid(row=2, column=0, sticky="ew", padx=14, pady=(0, 12))

    def _on_duration_changed(*_):
        try:
            duration_text.set(f"{float(cycle_duration_min_in.get()):g} min")
        except Exception:
            duration_text.set("—")

    def _on_temp_changed(*_):
        try:
            temp_text.set(f"{float(cycle_temp_set_f_in.get()):.1f} °F")
        except Exception:
            temp_text.set("—")

    cycle_duration_min_in.trace_add("write", _on_duration_changed)
    cycle_temp_set_f_in.trace_add("write", _on_temp_changed)
    _on_duration_changed()
    _on_temp_changed()

    # Time Remaining
    time_card.grid_columnconfigure(0, weight=1)
    ctk.CTkLabel(time_card, text="Time Remaining", font=("SF Pro Text", 16, "bold")).grid(
        row=0, column=0, sticky="w", padx=14, pady=(14, 10)
    )

    def fmt_mmss(s: int):
        m = max(0, int(s)) // 60
        sec = max(0, int(s)) % 60
        return f"{m:02d}:{sec:02d}"

    remaining_text = ctk.StringVar(value="00:00")
    ctk.CTkLabel(time_card, textvariable=remaining_text, font=("SF Pro Display", 34, "bold")).grid(
        row=1, column=0, sticky="w", padx=14, pady=(0, 14)
    )

    def _on_remaining_changed(*_):
        remaining_text.set(fmt_mmss(cycle_remaining_s.get()))
    cycle_remaining_s.trace_add("write", _on_remaining_changed)
    _on_remaining_changed()

    # Actions row
    action = ctk.CTkFrame(page_operation, corner_radius=16, fg_color="#F6F7F9")
    action.grid(row=2, column=0, sticky="nsew")
    action.grid_rowconfigure(0, weight=1)
    action.grid_columnconfigure((0,1,2), weight=1)

    # Commands will be bound in main.py after build_ui returns
    btn_start = ctk.CTkButton(action, text="START", font=("SF Pro Text", 48), corner_radius=12, command=lambda: None)
    btn_pause = ctk.CTkButton(action, text="PAUSE / RESUME", font=("SF Pro Text", 48), corner_radius=12, command=lambda: None)
    btn_stop  = ctk.CTkButton(action, text="STOP", font=("SF Pro Text", 48), corner_radius=12, command=lambda: None)

    def is_cycle_active() -> bool:
        return cycle_state.get() in ("WASHING", "PAUSED")

    def refresh_operation_panel():
        nonlocal last_mode

        mode = "idle" if cycle_state.get() in ("IDLE", "COMPLETE") else "active"

        if mode != last_mode:
            if is_cycle_active():
                set_card.grid_forget()
                time_card.grid(row=1, column=0, sticky="nsew", pady=(0, 12))
            else:
                time_card.grid_forget()
                set_card.grid(row=1, column=0, sticky="nsew", pady=(0, 12))

            btn_start.grid_forget()
            btn_pause.grid_forget()
            btn_stop.grid_forget()

            if mode == "idle":
                action.grid_columnconfigure(0, weight=1)
                action.grid_columnconfigure(1, weight=0)
                action.grid_columnconfigure(2, weight=0)
                btn_start.grid(row=0, column=0, columnspan=3, padx=12, pady=12, sticky="nsew")
            else:
                action.grid_columnconfigure(0, weight=0)
                action.grid_columnconfigure(1, weight=1)
                action.grid_columnconfigure(2, weight=1)
                btn_pause.grid(row=0, column=1, padx=12, pady=12, sticky="nsew")
                btn_stop.grid(row=0, column=2, padx=12, pady=12, sticky="nsew")

        last_mode = mode

    cycle_state.trace_add("write", lambda *_: refresh_operation_panel())
    refresh_operation_panel()

    # --------------------------
    # Status Page Layout
    # --------------------------
    content = ctk.CTkFrame(page_status, corner_radius=0, fg_color="white")
    content.pack(fill="both", expand=True, padx=0, pady=0)
    content.grid_columnconfigure((0,1), weight=1)
    content.grid_rowconfigure((0,1), weight=1)

    tempCard = ctk.CTkFrame(content, corner_radius=16)
    tempCard.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
    ctk.CTkLabel(tempCard, text="Temperature", font=("SF Pro Text",16,"bold")).grid(row=0, column=0, sticky="w", padx=12, pady=(12,4))
    tempNowF = ctk.StringVar(value="—")
    tempSetF = ctk.StringVar(value="Set: —")
    ctk.CTkLabel(tempCard, textvariable=tempNowF, font=("SF Pro Display", 28, "bold")).grid(row=1, column=0, sticky="w", padx=12, pady=(4, 0))
    ctk.CTkLabel(tempCard, textvariable=tempSetF, font=("SF Pro Text", 14), text_color="gray40").grid(row=2, column=0, sticky="w", padx=12, pady=(2, 12))

    flowCard = ctk.CTkFrame(content, corner_radius=16)
    flowCard.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)
    ctk.CTkLabel(flowCard, text="Flow Rate (L/min)", font=("SF Pro Text",16,"bold")).grid(row=0, column=1, sticky="w", padx=12, pady=(12,4))
    flow = ctk.StringVar(value="—")
    ctk.CTkLabel(flowCard, textvariable=flow, font=("SF Pro Text", 20)).grid(row=1, column=0, sticky="w", padx=12, pady=(4, 12))

    turbCard = ctk.CTkFrame(content, corner_radius=16)
    turbCard.grid(row=1, column=0, sticky="nsew", padx=10, pady=10)
    ctk.CTkLabel(turbCard, text="Turbidity", font=("SF Pro Text",16,"bold")).grid(row=0, column=0, sticky="w", padx=12, pady=(12,4))
    turbPct = ctk.StringVar(value="—")
    ntu = ctk.StringVar(value="—")
    ctk.CTkLabel(turbCard, textvariable=turbPct, font=("SF Pro Text", 24)).grid(row=1, column=0, sticky="w", padx=12, pady=4)
    ctk.CTkLabel(turbCard, textvariable=ntu, font=("SF Pro Text", 14), text_color="gray40").grid(row=2, column=0, sticky="w", padx=12, pady=(0, 12))

    levelCard = ctk.CTkFrame(content, corner_radius=16)
    levelCard.grid(row=1, column=1, sticky="nsew", padx=10, pady=10)
    ctk.CTkLabel(levelCard, text="Liquid Level", font=("SF Pro Text",16,"bold")).grid(row=0, column=0, sticky="w", padx=12, pady=(12,4))
    levelText = ctk.StringVar(value="—")
    ctk.CTkLabel(levelCard, textvariable=levelText, font=("SF Pro Text", 20)).grid(row=1, column=0, sticky="w", padx=12, pady=(4,12))
    ctk.CTkLabel(levelCard, text="Lid Interlock", font=("SF Pro Text",16,"bold")).grid(row=3, column=0, sticky="w", padx=12, pady=(12,4))
    lidText = ctk.StringVar(value="—")
    ctk.CTkLabel(levelCard, textvariable=lidText, font=("SF Pro Text", 20)).grid(row=4, column=0, sticky="w", padx=12, pady=(4,12))

    heaterCard = ctk.CTkFrame(content, corner_radius=16)
    heaterCard.grid(row=2, column=0, columnspan=2, sticky="nsew", padx=10, pady=10)
    ctk.CTkLabel(heaterCard, text="Heater", font=("SF Pro Text",16,"bold")).grid(row=0, column=0, sticky="w", padx=12, pady=(12,6))

    heaterEnableVar = ctk.BooleanVar(value=False)
    heaterStatusText = ctk.StringVar(value="Disabled")
    heaterOutText = ctk.StringVar(value="Output: —")
    heaterWarnText = ctk.StringVar(value="")

    heater_switch = ctk.CTkSwitch(heaterCard, text="Enable", variable=heaterEnableVar, command=lambda: None)
    heater_switch.grid(row=0, column=1, sticky="e", padx=12, pady=(12,6))
    ctk.CTkLabel(heaterCard, textvariable=heaterOutText, font=("SF Pro Text", 14), text_color="gray40").grid(
        row=3, column=0, sticky="w", padx=12, pady=(0,8)
    )
    ctk.CTkLabel(heaterCard, textvariable=heaterWarnText, font=("SF Pro Text", 14, "bold"), text_color="red").grid(
        row=4, column=0, columnspan=2, sticky="w", padx=12, pady=(0,12)
    )
    heaterCard.grid_columnconfigure(0, weight=1)

    # default page
    show_page("operation")

    return UI(
        app=app,
        title_status=title_status,
        show_page=show_page,
        refresh_operation_panel=refresh_operation_panel,
        show_fault_overlay=show_fault_overlay,
        hide_fault_overlay=hide_fault_overlay,
        cycle_state=cycle_state,
        cycle_remaining_s=cycle_remaining_s,
        cycle_duration_min_in=cycle_duration_min_in,
        cycle_temp_set_f_in=cycle_temp_set_f_in,
        tempNowF=tempNowF,
        tempSetF=tempSetF,
        flow=flow,
        turbPct=turbPct,
        ntu=ntu,
        levelText=levelText,
        lidText=lidText,
        heaterEnableVar=heaterEnableVar,
        heaterStatusText=heaterStatusText,
        heaterOutText=heaterOutText,
        heaterWarnText=heaterWarnText,
        btn_start=btn_start,
        btn_pause=btn_pause,
        btn_stop=btn_stop,
        heater_switch=heater_switch,
        set_card=set_card,
        time_card=time_card,
        action=action,
    )
