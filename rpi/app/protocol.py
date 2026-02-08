# app/protocol.py
import json
from typing import Any, Dict, Optional
from app.state import AppModel

def parse_line(line: str) -> Optional[Dict[str, Any]]:
    """
    Returns a dict if line is valid JSON object, otherwise None.
    """
    try:
        msg = json.loads(line)
        if isinstance(msg, dict):
            return msg
        return None
    except Exception:
        return None

def get(msg: Dict[str, Any], key: str, default=None):
    """
    Convenience getter. Avoids KeyError and keeps main code cleaner.
    """
    return msg.get(key, default)

def apply_message(model: AppModel, m: Dict[str, Any]) -> None:
    # status/ack
    if "ack" in m:
        ack = f"ACK: {m.get('ack')} {m.get('cmd','')}".strip()
        model.last_ack = ack
        model.status_line = ack

    # telemetry
    if not m.get("ok"):
        return

    t = model.telemetry
    t.ok = True
    t.raw = m

    t.C = m.get("C", "")
    t.F = float(m.get("F", 0) or 0)
    t.flowLpm = float(m.get("flowLpm", 0) or 0)
    t.turbidity_pct = float(m.get("% Turbidity", 0) or 0)
    t.NTU = float(m.get("NTU", 0) or 0)

    t.heaterEnable = m.get("heaterEnable", None)
    t.setTempF = m.get("setTempF", None)
    t.heaterDemand = m.get("heaterDemand", None)
    t.heaterOn = m.get("heaterOn", None)
    t.overTemp = m.get("overTemp", None)
    t.secOver = m.get("secOver", None)
    t.heaterStop = m.get("heaterStop", None)
    t.levelOk = m.get("levelOk", None)
    t.lidClosed = m.get("lidClosed", None)

    t.cycleState = m.get("cycleState", None)
    t.cycleRemainingS = m.get("cycleRemainingS", None)
    t.cycleTempSetF = m.get("cycleTempSetF", None)