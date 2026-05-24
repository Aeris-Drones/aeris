"""Typed pod lifecycle models used by the device manager."""

from dataclasses import dataclass, field
from enum import Enum


class PodLifecycleState(str, Enum):
    DETECTED = "detected"
    VALIDATING = "validating"
    POWER_CHECK = "power_check"
    SOFT_START = "soft_start"
    ENUMERATING = "enumerating"
    REGISTERED = "registered"
    REJECTED = "rejected"
    DISCONNECTED = "disconnected"
    FAULTED = "faulted"


@dataclass(frozen=True)
class DetectedPod:
    vehicle_id: str
    slot_id: str
    one_wire_id: str


@dataclass(frozen=True)
class PodCalibrationMetadata:
    last_calibration_sec: float | None = None
    next_calibration_due_sec: float | None = None


@dataclass(frozen=True)
class PodMetadata:
    serial: str
    pod_type: str
    capabilities: tuple[str, ...] = ()
    nominal_power_watts: float = 0.0
    calibration: PodCalibrationMetadata = field(default_factory=PodCalibrationMetadata)


@dataclass(frozen=True)
class PodLinkInfo:
    transport: str
    capabilities: tuple[str, ...] = ()
    detail: str = ""


@dataclass(frozen=True)
class PodStageTiming:
    stage: str
    elapsed_sec: float


@dataclass(frozen=True)
class PodStatusSnapshot:
    vehicle_id: str
    slot_id: str
    one_wire_id: str
    pod_serial: str = ""
    pod_type: str = ""
    lifecycle_state: PodLifecycleState = PodLifecycleState.DETECTED
    capabilities: tuple[str, ...] = ()
    connected: bool = True
    power_ready: bool = False
    link_ready: bool = False
    first_seen_sec: float = 0.0
    last_seen_sec: float = 0.0
    enumeration_elapsed_sec: float = 0.0
    stage_timings: tuple[PodStageTiming, ...] = ()
    rejection_code: str = ""
    rejection_detail: str = ""
    fault_code: str = ""
    fault_detail: str = ""
    last_calibration_sec: float | None = None
    next_calibration_due_sec: float | None = None
