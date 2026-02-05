# ui_modern_fullscreen.py
import json, threading, queue, serial, time
import customtkinter as ctk

PORT = "/dev/ttyUSB0"          # set to /dev/serial/by-id/... for stability
BAUD = 115200
q = queue.Queue()

def reader():
    while True:
        try:
            with serial.Serial(PORT, BAUD, timeout=1) as ser:
                q.put(("status", f"Connected: {PORT}"))
                while True:
                    line = ser.readline().decode("utf-8","ignore").strip()
                    if line:
                        q.put(("data", line))
        except Exception as e:
            q.put(("status", f"Disconnected: {e}"))
            time.sleep(1)

def send(cmd):
    try:
        with serial.Serial(PORT, BAUD, timeout=1) as ser:
            ser.write((cmd+"\n").encode("utf-8"))
    except Exception as e:
        q.put(("status", f"Send failed: {e}"))

def main():
    ctk.set_appearance_mode("light")
    ctk.set_default_color_theme("blue")

    app = ctk.CTk()
    app.update_idletasks()
    app.attributes("-fullscreen", True)        # hard fullscreen
    app.title("SWARM")
    app.attributes("-fullscreen", True)           # fullscreen
    app.bind("<Escape>", lambda e: app.destroy()) # exit with ESC

    # Root layout
    root = ctk.CTkFrame(app, corner_radius=0, fg_color="white")
    root.pack(fill="both", expand=True)

    # Header
    header = ctk.CTkFrame(root, corner_radius=0, fg_color="white")
    header.pack(fill="x", padx=20, pady=(16,8))
    ctk.CTkLabel(header, text="SWARM", text_color="black",
                 font=("SF Pro Display", 28, "bold")).pack(side="left")
    # optional status at right
    title_status = ctk.StringVar(value="")
    ctk.CTkLabel(header, textvariable=title_status, text_color="black",
                 font=("SF Pro Text", 14)).pack(side="right")

    # Content area
    content = ctk.CTkFrame(root, corner_radius=0, fg_color="white")
    content.pack(fill="both", expand=True, padx=20, pady=8)
    content.grid_columnconfigure((0,1), weight=1)
    content.grid_rowconfigure((0,1), weight=1)

    # Cards
    card1 = ctk.CTkFrame(content, corner_radius=16)
    card2 = ctk.CTkFrame(content, corner_radius=16)
    card3 = ctk.CTkFrame(content, corner_radius=16)
    card1.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
    card2.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)
    card3.grid(row=1, column=0, columnspan=2, sticky="nsew", padx=10, pady=10)

    # --- Status + readings (card1) ---
    status = ctk.StringVar(value="Starting")
    okv    = ctk.StringVar(value="—")
    cmv    = ctk.StringVar(value="—")
    inv    = ctk.StringVar(value="—")

    ctk.CTkLabel(card1, text="Status", font=("SF Pro Text",14,"bold")).grid(row=0, column=0, sticky="w", padx=12, pady=(12,4))
    ctk.CTkLabel(card1, textvariable=status).grid(row=0, column=1, sticky="w", padx=12, pady=(12,4))
    ctk.CTkLabel(card1, text="ok").grid(row=1, column=0, sticky="w", padx=12, pady=4)
    ctk.CTkLabel(card1, textvariable=okv).grid(row=1, column=1, sticky="w", padx=12, pady=4)
    ctk.CTkLabel(card1, text="cm").grid(row=2, column=0, sticky="w", padx=12, pady=4)
    ctk.CTkLabel(card1, textvariable=cmv).grid(row=2, column=1, sticky="w", padx=12, pady=4)
    ctk.CTkLabel(card1, text="in").grid(row=3, column=0, sticky="w", padx=12, pady=(4,12))
    ctk.CTkLabel(card1, textvariable=inv).grid(row=3, column=1, sticky="w", padx=12, pady=(4,12))
    card1.grid_columnconfigure(1, weight=1)

    # --- LED (card2) ---
    led_state = ctk.BooleanVar(value=False)
    def on_led():
        new = led_state.get()
        send(f"LED {1 if new else 0}")
    ctk.CTkLabel(card2, text="LED", font=("SF Pro Text",14,"bold")).grid(row=0, column=0, sticky="w", padx=12, pady=(12,6))
    ctk.CTkSwitch(card2, text="On", variable=led_state, command=on_led).grid(row=0, column=1, sticky="e", padx=12, pady=(12,6))
    card2.grid_columnconfigure(1, weight=1)

    # --- Motor (card3) ---
    ctk.CTkLabel(card3, text="Motor", font=("SF Pro Text",14,"bold")).grid(row=0, column=0, sticky="w", padx=12, pady=(12,6))

    # direction
    try:
        dirv = ctk.StringVar(value="FWD")
        dir_row = ctk.CTkSegmentedButton(card3, values=["FWD","REV"], variable=dirv)
        dir_row.grid(row=0, column=1, sticky="e", padx=12, pady=(12,6))
    except Exception:
        dirv = ctk.StringVar(value="FWD")
        ctk.CTkRadioButton(card3, text="FWD", variable=dirv, value="FWD").grid(row=0, column=1, padx=6, pady=(12,6), sticky="e")
        ctk.CTkRadioButton(card3, text="REV", variable=dirv, value="REV").grid(row=0, column=2, padx=6, pady=(12,6), sticky="e")

    spd = ctk.IntVar(value=0)
    def apply_speed(_=None):
        duty = int(spd.get() * 255 / 100)
        if duty <= 0: send("MOTOR 0")
        else:         send(f"MOTOR {duty} {dirv.get()}")

    ctk.CTkSlider(card3, from_=0, to=100, number_of_steps=100,
                  variable=spd, command=lambda _ : None).grid(row=1, column=0, columnspan=3, sticky="we", padx=12, pady=8)

    btnrow = ctk.CTkFrame(card3, fg_color="transparent")
    btnrow.grid(row=2, column=0, columnspan=3, sticky="we", padx=12, pady=(6,12))
    ctk.CTkButton(btnrow, text="Apply Speed", command=apply_speed).pack(side="left", expand=True, fill="x", padx=(0,6))
    ctk.CTkButton(btnrow, text="Stop", fg_color="#222", hover_color="#333",
                  command=lambda: (spd.set(0), send("MOTOR 0"))).pack(side="left", expand=True, fill="x", padx=(6,0))

    # Queue pump
    def poll_queue():
        try:
            while True:
                kind, payload = q.get_nowait()
                if kind == "status":
                    status.set(payload)
                    title_status.set(payload.replace("Connected:","").strip())
                elif kind == "data":
                    try:
                        m = json.loads(payload)
                        okv.set(str(m.get("ok")))
                        if m.get("ok"):
                            cmv.set(f'{m.get("cm",0):.2f}')
                            inv.set(f'{m.get("in",0):.2f}')
                        else:
                            cmv.set("—"); inv.set("—")
                    except json.JSONDecodeError:
                        status.set(payload)
        except queue.Empty:
            pass
        app.after(100, poll_queue)

    threading.Thread(target=reader, daemon=True).start()
    app.after(100, poll_queue)
    app.mainloop()

if __name__ == "__main__":
    main()
