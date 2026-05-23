"""Tracks pod-backed capability availability for the orchestrator."""

from dataclasses import dataclass
from enum import Enum

from .vehicle_ids import normalize_vehicle_id


class PodCapabilityState(str, Enum):
    REGISTERED = "registered"
    REJECTED = "rejected"
    DISCONNECTED = "disconnected"
    FAULTED = "faulted"


@dataclass(frozen=True)
class PodCapabilityRecord:
    vehicle_id: str
    slot_id: str
    pod_serial: str
    lifecycle_state: PodCapabilityState
    capabilities: tuple[str, ...]
    rejection_code: str = ""
    fault_code: str = ""


class PodCapabilityRegistry:
    """Stores the current set of available and unavailable pod capabilities."""

    def __init__(self) -> None:
        self._available_by_vehicle: dict[str, set[str]] = {}
        self._unavailable_by_vehicle: dict[str, dict[str, str]] = {}

    def apply_records(self, records: list[PodCapabilityRecord]) -> None:
        available: dict[str, set[str]] = {}
        unavailable: dict[str, dict[str, str]] = {}

        for record in records:
            vehicle_id = normalize_vehicle_id(record.vehicle_id)
            if not vehicle_id:
                continue
            capabilities = self._normalized_capabilities(record.capabilities)
            if record.lifecycle_state is PodCapabilityState.REGISTERED:
                available.setdefault(vehicle_id, set()).update(capabilities)
                continue

            reason = self._reason_for_record(record)
            bucket = unavailable.setdefault(vehicle_id, {})
            for capability in capabilities:
                bucket[capability] = reason

        for vehicle_id, capabilities in available.items():
            unavailable_bucket = unavailable.get(vehicle_id)
            if unavailable_bucket is None:
                continue
            for capability in capabilities:
                unavailable_bucket.pop(capability, None)
            if not unavailable_bucket:
                unavailable.pop(vehicle_id, None)

        self._available_by_vehicle = available
        self._unavailable_by_vehicle = unavailable

    def available_capabilities_for_vehicle(self, vehicle_id: str) -> set[str]:
        return set(self._available_by_vehicle.get(normalize_vehicle_id(vehicle_id), set()))

    def unavailable_capabilities_for_vehicle(self, vehicle_id: str) -> set[str]:
        bucket = self._unavailable_by_vehicle.get(normalize_vehicle_id(vehicle_id), {})
        return set(bucket.keys())

    def unavailable_reason(self, vehicle_id: str, capability: str) -> str | None:
        bucket = self._unavailable_by_vehicle.get(normalize_vehicle_id(vehicle_id), {})
        return bucket.get(str(capability).strip().lower())

    def effective_slam_mode(self, vehicle_id: str, requested_mode: str) -> str:
        normalized_mode = str(requested_mode).strip().lower()
        if normalized_mode in {"lio_sam", "liosam"}:
            if "lidar" not in self.available_capabilities_for_vehicle(vehicle_id):
                return "vio"
            return "liosam"
        return normalized_mode

    @staticmethod
    def _normalized_capabilities(capabilities: tuple[str, ...]) -> set[str]:
        values: set[str] = set()
        for capability in capabilities:
            normalized = str(capability).strip().lower()
            if normalized:
                values.add(normalized)
        return values

    @staticmethod
    def _reason_for_record(record: PodCapabilityRecord) -> str:
        if record.lifecycle_state is PodCapabilityState.DISCONNECTED:
            return "disconnected"
        if record.lifecycle_state is PodCapabilityState.FAULTED:
            return (
                f"faulted:{record.fault_code}"
                if record.fault_code
                else "faulted"
            )
        if record.lifecycle_state is PodCapabilityState.REJECTED:
            return (
                f"rejected:{record.rejection_code}"
                if record.rejection_code
                else "rejected"
            )
        return record.lifecycle_state.value
