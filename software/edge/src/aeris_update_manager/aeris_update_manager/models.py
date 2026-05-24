"""Typed firmware update models used by the update coordinator."""

from dataclasses import dataclass
from enum import Enum


class FirmwareUpdateLifecycleState(str, Enum):
    UNKNOWN = "unknown"
    IDLE = "idle"
    DOWNLOADING = "downloading"
    VALIDATING = "validating"
    APPLYING = "applying"
    VERIFYING = "verifying"
    COMPLETE = "complete"
    FAILED = "failed"
    ROLLING_BACK = "rolling_back"
    ROLLED_BACK = "rolled_back"


@dataclass(frozen=True)
class FirmwarePartitionState:
    active_slot: str
    inactive_slot: str
    current_version: str


@dataclass(frozen=True)
class FirmwareUpdateCommand:
    vehicle_id: str
    package_id: str
    target_version: str
    package_uri: str
    package_signature: str


@dataclass(frozen=True)
class StagedFirmwarePackage:
    package_id: str
    target_version: str
    package_uri: str
    signature: str


@dataclass(frozen=True)
class FirmwareUpdateStatusSnapshot:
    vehicle_id: str
    package_id: str
    current_version: str
    target_version: str
    lifecycle_state: FirmwareUpdateLifecycleState
    progress_percent: float
    active_slot: str
    inactive_slot: str
    rollback_performed: bool = False
    status_detail: str = ""
    error_code: str = ""
    error_detail: str = ""
