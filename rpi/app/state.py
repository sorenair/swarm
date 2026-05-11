"""State containers for telemetry and application status."""
from dataclasses import dataclass, field
from typing import Optional, Any, Dict

@dataclass
class Telemetry:
    """Live telemetry values received from the controller."""
    # core
    ok: bool = False
    C: str = ""
    F: float = 0.0
    flowLpm: float = 0.0

    # heater / safety
    heaterEnable: Optional[bool] = None
    setTempF: Optional[float] = None
    heaterDemand: Optional[bool] = None
    heaterOn: Optional[bool] = None
    overTemp: Optional[bool] = None
    secOver: Optional[float] = None
    heaterStop: Optional[bool] = None
    overTempThresholdF: Optional[float] = None
    overTempLimitS: Optional[int] = None
    heatLowerSetpointF: Optional[float] = None
    heatUpperSetpointF: Optional[float] = None
    levelOk: Optional[bool] = None
    lidClosed: Optional[bool] = None

    # flow sensor / saftey
    flowLow: Optional[bool] = None
    secLowFlow: Optional[int] = None
    flowFault: Optional[bool] = None
    flowLowThresholdLpm: Optional[float] = None
    flowLowLimitS: Optional[int] = None

    # controller-defined fault status
    faultActive: Optional[bool] = None
    faultFlags: Optional[int] = None
    faultCode: Optional[str] = None
    activeFaults: list[str] = field(default_factory=list)

    # cycle
    cycleState: Optional[str] = None
    cycleRemainingS: Optional[float] = None
    cycleTempSetF: Optional[float] = None

    raw: Dict[str, Any] = field(default_factory=dict)

@dataclass
class AppModel:
    """Top-level model for UI rendering and status tracking."""
    status_line: str = ""
    last_ack: str = ""
    telemetry: Telemetry = field(default_factory=Telemetry)
