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
            ser.write((cmd + "\n").encode("utf-8"))
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
    turbCard.grid(row=1, column=0, columnspan=2, sticky="nsew", padx=10, pady=10)
    ctk.CTkLabel(turbCard, text="Turbidity", font=("SF Pro Text",16,"bold")).grid(row=0, column=0, sticky="w", padx=12, pady=(12,4))
    turbPct = ctk.StringVar(value="—")
    ntu = ctk.StringVar(value="—")
    ctk.CTkLabel(turbCard, textvariable=turbPct, font=("SF Pro Text", 24)).grid(row=1, column=0, sticky="w", padx=12, pady=4)
    ctk.CTkLabel(turbCard, textvariable=ntu, font=("SF Pro Text", 14), text_color="gray40").grid(row=2, column=0, sticky="w", padx=12, pady=(0, 12))\
    
    #motor card
    motorCard = ctk.CTkFrame(content, corner_radius=16)
    motorCard.grid(row=2, column=0, columnspan=2, sticky="nsew", padx=10, pady=10)
    ctk.CTkLabel(motorCard, text="Motor", font=("SF Pro Text",16,"bold")).grid(row=0, column=0, sticky="w", padx=12, pady=(12,4))
    dirv = ctk.StringVar(value="FWD")
    dir_row = ctk.CTkSegmentedButton(motorCard, values=["FWD", "REV"], variable=dirv)
    dir_row.grid(row=0, column=1, sticky="e", padx=12, pady=(12,6))
    ###############################################################
    spd = ctk.IntVar(value=0)
    def apply_speed(_=None):
        duty = int(spd.get() * 255 / 100)
        if duty <= 0:
            send("MOTOR 0")
        else:
            send(f"MOTOR {duty} {dirv.get()}")
    ctk.CTkSlider(motorCard, from_=0, to=100, number_of_steps=100, variable=spd, command=lambda _ : None).grid(row=1, column=0, columnspan=3, sticky="we", padx=12, pady=8)
    ctk.CTkButton(motorCard, text="Apply Speed", command=apply_speed).grid(row=2, column=0, columnspan=3, padx=12, pady=(4,12))

    # Queue pump
    def poll_queue():
        try:
            while True:
                kind, payload = q.get_nowait()
                if kind == "status":
                    title_status.set(payload)
                elif kind == "data":
                    try:
                        #print("RAW:", payload)
                        m = json.loads(payload)
                        if m.get("ok"):
                            tempC.set(f"{m.get('C', 0):.2f} C")
                            tempF.set(f"{m.get('F', 0):.2f} F")
                            flow.set(f"{m.get('flowLpm', 0):.2f} L/min")
                            turbPct.set(f"{m.get('% Turbidity', 0):.1f} %")
                            ntu.set(f"{m.get('NTU', 0):.1f} NTU")
                    except json.JSONDecodeError:
                        title_status.set("Bad JSON")
        except queue.Empty:
            pass
        app.after(200, poll_queue)

    threading.Thread(target=reader, daemon=True).start()
    app.after(200, poll_queue)
    app.mainloop()

if __name__ == "__main__":
    main()
