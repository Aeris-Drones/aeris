"""Durable pod inventory registry and calibration state projection."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from enum import Enum
from pathlib import Path

from .models import PodCalibrationMetadata, PodStatusSnapshot


class PodCalibrationState(str, Enum):
    UNKNOWN = "unknown"
    CURRENT = "current"
    DUE_SOON = "due_soon"
    OVERDUE = "overdue"


@dataclass(frozen=True)
class PodInventoryRecord:
    pod_serial: str
    pod_type: str
    one_wire_id: str = ""
    attached: bool = False
    vehicle_id: str = ""
    slot_id: str = ""
    lifecycle_state_label: str = "unknown"
    connected: bool = False
    power_ready: bool = False
    link_ready: bool = False
    capabilities: tuple[str, ...] = ()
    first_seen_sec: float = 0.0
    last_seen_sec: float = 0.0
    last_calibration_sec: float | None = None
    next_calibration_due_sec: float | None = None
    calibration_state: PodCalibrationState = PodCalibrationState.UNKNOWN
    calibration_detail: str = "Calibration schedule unavailable"


@dataclass
class _StoredPodInventoryRecord:
    pod_serial: str
    pod_type: str
    one_wire_id: str = ""
    attached: bool = False
    vehicle_id: str = ""
    slot_id: str = ""
    lifecycle_state_label: str = "unknown"
    connected: bool = False
    power_ready: bool = False
    link_ready: bool = False
    capabilities: tuple[str, ...] = ()
    first_seen_sec: float = 0.0
    last_seen_sec: float = 0.0
    last_calibration_sec: float | None = None
    next_calibration_due_sec: float | None = None


class PodInventoryRegistry:
    """Tracks all known pods, including detached rows, in a small JSON registry."""

    def __init__(
        self,
        *,
        registry_path: str | Path | None,
        due_soon_window_sec: float = 14 * 24 * 60 * 60,
    ) -> None:
        self._registry_path = Path(registry_path).expanduser() if registry_path else None
        self._due_soon_window_sec = float(max(0.0, due_soon_window_sec))
        self._records: dict[str, _StoredPodInventoryRecord] = {}
        self._load()

    def sync_live_snapshots(
        self, snapshots: tuple[PodStatusSnapshot, ...], *, now_sec: float
    ) -> None:
        for snapshot in snapshots:
            key = self._record_key(snapshot.pod_serial, snapshot.one_wire_id)
            current = self._records.get(key)
            attached = bool(snapshot.connected)
            calibration = self._merge_calibration(
                current=current,
                snapshot_last=snapshot.last_calibration_sec,
                snapshot_next=snapshot.next_calibration_due_sec,
            )
            self._records[key] = _StoredPodInventoryRecord(
                pod_serial=snapshot.pod_serial or (current.pod_serial if current else ""),
                pod_type=snapshot.pod_type or (current.pod_type if current else ""),
                one_wire_id=snapshot.one_wire_id or (current.one_wire_id if current else ""),
                attached=attached,
                vehicle_id=snapshot.vehicle_id if attached else "",
                slot_id=snapshot.slot_id if attached else "",
                lifecycle_state_label=snapshot.lifecycle_state.value,
                connected=snapshot.connected,
                power_ready=snapshot.power_ready,
                link_ready=snapshot.link_ready,
                capabilities=tuple(snapshot.capabilities) or (current.capabilities if current else ()),
                first_seen_sec=(
                    current.first_seen_sec
                    if current and current.first_seen_sec > 0.0
                    else snapshot.first_seen_sec
                ),
                last_seen_sec=max(snapshot.last_seen_sec, now_sec if attached else snapshot.last_seen_sec),
                last_calibration_sec=calibration.last_calibration_sec,
                next_calibration_due_sec=calibration.next_calibration_due_sec,
            )
        self._save()

    def apply_calibration_update(
        self,
        *,
        pod_serial: str,
        calibration: PodCalibrationMetadata,
        now_sec: float,
    ) -> PodInventoryRecord:
        key = self._record_key(pod_serial, "")
        stored = self._records.get(key)
        if stored is None:
            raise KeyError(pod_serial)
        stored.last_calibration_sec = calibration.last_calibration_sec
        stored.next_calibration_due_sec = calibration.next_calibration_due_sec
        stored.last_seen_sec = max(stored.last_seen_sec, now_sec)
        self._save()
        return self.snapshot(now_sec=now_sec, pod_serial=pod_serial)[0]

    def snapshot(
        self, *, now_sec: float, pod_serial: str | None = None
    ) -> tuple[PodInventoryRecord, ...]:
        rows = [
            self._to_inventory_record(record, now_sec=now_sec)
            for record in self._records.values()
            if pod_serial is None or record.pod_serial == pod_serial
        ]
        rows.sort(key=lambda row: (row.pod_type, row.pod_serial, row.one_wire_id))
        return tuple(rows)

    def lookup(self, pod_serial: str) -> PodInventoryRecord | None:
        rows = self.snapshot(now_sec=0.0, pod_serial=pod_serial)
        return rows[0] if rows else None

    def _load(self) -> None:
        if self._registry_path is None or not self._registry_path.exists():
            return
        payload = json.loads(self._registry_path.read_text(encoding="utf-8"))
        records = payload.get("records", [])
        for item in records:
            row = _StoredPodInventoryRecord(
                pod_serial=str(item.get("pod_serial", "")).strip(),
                pod_type=str(item.get("pod_type", "")).strip(),
                one_wire_id=str(item.get("one_wire_id", "")).strip(),
                attached=bool(item.get("attached", False)),
                vehicle_id=str(item.get("vehicle_id", "")).strip(),
                slot_id=str(item.get("slot_id", "")).strip(),
                lifecycle_state_label=str(item.get("lifecycle_state_label", "unknown")).strip()
                or "unknown",
                connected=bool(item.get("connected", False)),
                power_ready=bool(item.get("power_ready", False)),
                link_ready=bool(item.get("link_ready", False)),
                capabilities=tuple(item.get("capabilities", []) or ()),
                first_seen_sec=float(item.get("first_seen_sec", 0.0) or 0.0),
                last_seen_sec=float(item.get("last_seen_sec", 0.0) or 0.0),
                last_calibration_sec=self._optional_float(item.get("last_calibration_sec")),
                next_calibration_due_sec=self._optional_float(item.get("next_calibration_due_sec")),
            )
            self._records[self._record_key(row.pod_serial, row.one_wire_id)] = row

    def _save(self) -> None:
        if self._registry_path is None:
            return
        self._registry_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "records": [asdict(record) for _, record in sorted(self._records.items())]
        }
        self._registry_path.write_text(
            json.dumps(payload, indent=2, sort_keys=True),
            encoding="utf-8",
        )

    def _to_inventory_record(
        self, record: _StoredPodInventoryRecord, *, now_sec: float
    ) -> PodInventoryRecord:
        state, detail = self._derive_calibration_projection(
            last_calibration_sec=record.last_calibration_sec,
            next_calibration_due_sec=record.next_calibration_due_sec,
            now_sec=now_sec,
        )
        return PodInventoryRecord(
            pod_serial=record.pod_serial,
            pod_type=record.pod_type,
            one_wire_id=record.one_wire_id,
            attached=record.attached,
            vehicle_id=record.vehicle_id if record.attached else "",
            slot_id=record.slot_id if record.attached else "",
            lifecycle_state_label=record.lifecycle_state_label,
            connected=record.connected,
            power_ready=record.power_ready,
            link_ready=record.link_ready,
            capabilities=tuple(record.capabilities),
            first_seen_sec=record.first_seen_sec,
            last_seen_sec=record.last_seen_sec,
            last_calibration_sec=record.last_calibration_sec,
            next_calibration_due_sec=record.next_calibration_due_sec,
            calibration_state=state,
            calibration_detail=detail,
        )

    def _derive_calibration_projection(
        self,
        *,
        last_calibration_sec: float | None,
        next_calibration_due_sec: float | None,
        now_sec: float,
    ) -> tuple[PodCalibrationState, str]:
        if not last_calibration_sec or not next_calibration_due_sec:
            return (
                PodCalibrationState.UNKNOWN,
                "Calibration schedule unavailable",
            )
        if next_calibration_due_sec < now_sec:
            overdue_sec = now_sec - next_calibration_due_sec
            return (
                PodCalibrationState.OVERDUE,
                f"Overdue by {self._format_days(overdue_sec)}",
            )
        if next_calibration_due_sec - now_sec <= self._due_soon_window_sec:
            return (
                PodCalibrationState.DUE_SOON,
                f"Due in {self._format_days(next_calibration_due_sec - now_sec)}",
            )
        return (
            PodCalibrationState.CURRENT,
            f"Current until {self._format_date(next_calibration_due_sec)}",
        )

    @staticmethod
    def _merge_calibration(
        *,
        current: _StoredPodInventoryRecord | None,
        snapshot_last: float | None,
        snapshot_next: float | None,
    ) -> PodCalibrationMetadata:
        if current is None:
            return PodCalibrationMetadata(snapshot_last, snapshot_next)
        current_last = current.last_calibration_sec
        if snapshot_last is None:
            return PodCalibrationMetadata(current_last, current.next_calibration_due_sec)
        if current_last is not None and current_last > snapshot_last:
            return PodCalibrationMetadata(current_last, current.next_calibration_due_sec)
        return PodCalibrationMetadata(snapshot_last, snapshot_next)

    @staticmethod
    def _record_key(pod_serial: str, one_wire_id: str) -> str:
        serial = str(pod_serial).strip()
        if serial:
            return serial
        one_wire = str(one_wire_id).strip()
        return f"unknown::{one_wire}"

    @staticmethod
    def _optional_float(value: object) -> float | None:
        if value is None or value == "":
            return None
        numeric = float(value)
        return numeric if numeric > 0.0 else None

    @staticmethod
    def _format_days(seconds: float) -> str:
        whole_days = max(1, round(float(seconds) / 86400))
        return f"{whole_days} day" if whole_days == 1 else f"{whole_days} days"

    @staticmethod
    def _format_date(seconds: float) -> str:
        return datetime.fromtimestamp(seconds, tz=UTC).date().isoformat()
