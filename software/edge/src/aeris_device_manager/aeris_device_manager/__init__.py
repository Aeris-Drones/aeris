"""Aeris device manager package."""

from .adapters import (
    CalibrationWriteError,
    DeviceManagerError,
    InvalidEepromError,
    LinkEnumerationError,
    NullPodHardwareAdapter,
    PodHardwareAdapter,
    PowerBudgetDeniedError,
    SoftStartError,
    UnsupportedPodError,
)
from .inventory import PodCalibrationState, PodInventoryRecord, PodInventoryRegistry
from .models import (
    DetectedPod,
    PodCalibrationMetadata,
    PodLifecycleState,
    PodLinkInfo,
    PodMetadata,
    PodStageTiming,
    PodStatusSnapshot,
)
from .state_machine import DeviceManagerStateMachine, ReconcileResult

__all__ = [
    "CalibrationWriteError",
    "DetectedPod",
    "DeviceManagerError",
    "DeviceManagerStateMachine",
    "InvalidEepromError",
    "LinkEnumerationError",
    "NullPodHardwareAdapter",
    "PodCalibrationMetadata",
    "PodCalibrationState",
    "PodHardwareAdapter",
    "PodInventoryRecord",
    "PodInventoryRegistry",
    "PodLifecycleState",
    "PodLinkInfo",
    "PodMetadata",
    "PodStageTiming",
    "PodStatusSnapshot",
    "PowerBudgetDeniedError",
    "ReconcileResult",
    "SoftStartError",
    "UnsupportedPodError",
]
