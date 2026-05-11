"""Helpers for parsing and applying ESP32 protocol messages."""
import json
from typing import Any, Dict, Optional
from app.state import AppModel

_JSON_DECODER = json.JSONDecoder()

def parse_line(line: str) -> Optional[Dict[str, Any]]:
    """Return a dict from a serial line, tolerating startup noise before JSON."""
    try:
        start = line.find("{")
        if start < 0:
            return None

        msg, _ = _JSON_DECODER.raw_decode(line[start:])
        if isinstance(msg, dict):
            return msg
        return None
    except Exception:
        return None

def get(msg: Dict[str, Any], key: str, default=None):
    """Return a key from the message with a default fallback."""
    return msg.get(key, default)

def apply_message(model: AppModel, m: Dict[str, Any]) -> None:
    """Apply a decoded message to the in-memory model."""
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
    t.flowLow = m.get("flowLow")
    t.secLowFlow = m.get("secLowFlow")
    t.flowFault = m.get("flowFault")

    t.heaterEnable = m.get("heaterEnable", None)
    t.setTempF = m.get("setTempF", None)
    t.heaterDemand = m.get("heaterDemand", None)
    t.heaterOn = m.get("heaterOn", None)
    t.overTemp = m.get("overTemp", None)
    t.secOver = m.get("secOver", None)
    t.heaterStop = m.get("heaterStop", None)
    t.overTempThresholdF = m.get("overTempThresholdF", None)
    t.overTempLimitS = m.get("overTempLimitS", None)
    t.heatLowerSetpointF = m.get("heatLowerSetpointF", None)
    t.heatUpperSetpointF = m.get("heatUpperSetpointF", None)
    t.levelOk = m.get("levelOk", None)
    t.lidClosed = m.get("lidClosed", None)

    t.cycleState = m.get("cycleState", None)
    t.cycleRemainingS = m.get("cycleRemainingS", None)
    t.cycleTempSetF = m.get("cycleTempSetF", None)
    t.flowLowThresholdLpm = m.get("flowLowThresholdLpm", None)
    t.flowLowLimitS = m.get("flowLowLimitS", None)
    t.faultActive = m.get("faultActive", None)
    t.faultFlags = m.get("faultFlags", None)
    t.faultCode = m.get("faultCode", None)
    t.activeFaults = list(m.get("activeFaults", []) or [])
