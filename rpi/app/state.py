# app/state.py
from dataclasses import dataclass, field
from typing import Optional, Any, Dict

@dataclass
class Telemetry:
    # core
    ok: bool = False
    C: str = ""
    F: float = 0.0
    flowLpm: float = 0.0
    turbidity_pct: float = 0.0
    NTU: float = 0.0

    # heater / safety
    heaterEnable: Optional[bool] = None
    setTempF: Optional[float] = None
    heaterDemand: Optional[bool] = None
    heaterOn: Optional[bool] = None
    overTemp: Optional[bool] = None
    secOver: Optional[float] = None
    heaterStop: Optional[bool] = None
    levelOk: Optional[bool] = None
    lidClosed: Optional[bool] = None

    # cycle
    cycleState: Optional[str] = None
    cycleRemainingS: Optional[float] = None
    cycleTempSetF: Optional[float] = None

    raw: Dict[str, Any] = field(default_factory=dict)

@dataclass
class AppModel:
    status_line: str = ""
    last_ack: str = ""
    telemetry: Telemetry = field(default_factory=Telemetry)
