import threading, time, queue, serial

class SerialClient:
    def __init__(self, port: str, baud: int, rx_queue: "queue.Queue[tuple[str,str]]", tx_lock=None):
        self.port = port
        self.baud = baud
        self.q = rx_queue
        self._ser = None
        self._lock = tx_lock or threading.Lock()
        self._stop = threading.Event()

        self.connected = False
        self.last_rx_time = 0.0
        self.last_disconnect_reason = ""

    def start(self):
        t = threading.Thread(target=self._reader_loop, daemon=True)
        t.start()

    def stop(self):
        self._stop.set()
        with self._lock:
            try:
                if self._ser:
                    self._ser.close()
            except Exception:
                pass

    def send_line(self, line: str):
        with self._lock:
            if self._ser and self._ser.is_open:
                self._ser.write((line + "\n").encode("utf-8"))
            else:
                self.q.put(("status", "Send failed: serial not connected"))

    def _reader_loop(self):
        while not self._stop.is_set():
            try:
                self._ser = serial.Serial(self.port, self.baud, timeout=1)
                self.connected = True
                self.last_disconnect_reason = ""
                self.last_rx_time = time.time()
                self.q.put(("status", f"Connected: {self.port}"))

                while not self._stop.is_set():
                    line = self._ser.readline().decode("utf-8", "ignore").strip()
                    if line:
                        self.last_rx_time = time.time()
                        self.q.put(("data", line))

            except Exception as e:
                self.connected = False
                self.last_disconnect_reason = str(e)
                self.q.put(("status", f"Disconnected: {e}"))
                time.sleep(1.0)