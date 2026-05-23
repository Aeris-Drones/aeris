"""Aeris device manager package."""

from .adapters import (
    DeviceManagerError,
    InvalidEepromError,
    LinkEnumerationError,
    NullPodHardwareAdapter,
    PodHardwareAdapter,
    PowerBudgetDeniedError,
    SoftStartError,
    UnsupportedPodError,
)
from .models import (
    DetectedPod,
    PodLifecycleState,
    PodLinkInfo,
    PodMetadata,
    PodStageTiming,
    PodStatusSnapshot,
)
from .state_machine import DeviceManagerStateMachine, ReconcileResult

__all__ = [
    "DetectedPod",
    "DeviceManagerError",
    "DeviceManagerStateMachine",
    "InvalidEepromError",
    "LinkEnumerationError",
    "NullPodHardwareAdapter",
    "PodHardwareAdapter",
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
