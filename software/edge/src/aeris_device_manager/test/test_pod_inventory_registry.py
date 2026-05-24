from pathlib import Path

from aeris_device_manager.inventory import (
    PodCalibrationMetadata,
    PodCalibrationState,
    PodInventoryRegistry,
)
from aeris_device_manager.models import PodLifecycleState, PodStatusSnapshot


def _connected_snapshot(**overrides) -> PodStatusSnapshot:
    values = {
        "vehicle_id": "scout_2",
        "slot_id": "belly",
        "one_wire_id": "28-0018",
        "pod_serial": "GAS-118",
        "pod_type": "hazmat",
        "lifecycle_state": PodLifecycleState.REGISTERED,
        "capabilities": ("gas", "hazmat"),
        "connected": True,
        "power_ready": True,
        "link_ready": True,
        "first_seen_sec": 1_700_000_000.0,
        "last_seen_sec": 1_700_000_010.0,
        "last_calibration_sec": 1_700_000_000.0,
        "next_calibration_due_sec": 1_700_864_000.0,
    }
    values.update(overrides)
    return PodStatusSnapshot(**values)


def test_registry_projects_due_soon_and_persists_detached_known_pods(tmp_path: Path) -> None:
    registry_path = tmp_path / "pod-inventory.json"
    registry = PodInventoryRegistry(
        registry_path=registry_path,
        due_soon_window_sec=14 * 24 * 60 * 60,
    )

    registry.sync_live_snapshots((_connected_snapshot(),), now_sec=1_700_000_020.0)
    due_soon = registry.snapshot(now_sec=1_700_000_020.0)

    assert len(due_soon) == 1
    assert due_soon[0].attached is True
    assert due_soon[0].calibration_state is PodCalibrationState.DUE_SOON
    assert "Due in" in due_soon[0].calibration_detail

    registry.sync_live_snapshots(
        (
            _connected_snapshot(
                connected=False,
                power_ready=False,
                link_ready=False,
                lifecycle_state=PodLifecycleState.DISCONNECTED,
            ),
        ),
        now_sec=1_700_000_120.0,
    )
    detached = registry.snapshot(now_sec=1_700_000_120.0)

    assert detached[0].attached is False
    assert detached[0].vehicle_id == ""
    assert detached[0].slot_id == ""
    assert detached[0].pod_serial == "GAS-118"

    reloaded = PodInventoryRegistry(
        registry_path=registry_path,
        due_soon_window_sec=14 * 24 * 60 * 60,
    )
    reloaded_rows = reloaded.snapshot(now_sec=1_700_000_120.0)
    assert reloaded_rows[0].pod_serial == "GAS-118"
    assert reloaded_rows[0].attached is False


def test_registry_marks_missing_schedule_unknown_and_overdue_dates_blocking(
    tmp_path: Path,
) -> None:
    registry = PodInventoryRegistry(
        registry_path=tmp_path / "pod-inventory.json",
        due_soon_window_sec=7 * 24 * 60 * 60,
    )

    registry.sync_live_snapshots(
        (
            _connected_snapshot(
                pod_serial="THM-204",
                pod_type="thermal",
                last_calibration_sec=None,
                next_calibration_due_sec=None,
            ),
            _connected_snapshot(
                pod_serial="LDR-551",
                pod_type="lidar",
                one_wire_id="28-0055",
                slot_id="rear-bay",
                next_calibration_due_sec=1_699_900_000.0,
            ),
        ),
        now_sec=1_700_000_000.0,
    )

    rows = {row.pod_serial: row for row in registry.snapshot(now_sec=1_700_000_000.0)}
    assert rows["THM-204"].calibration_state is PodCalibrationState.UNKNOWN
    assert "unavailable" in rows["THM-204"].calibration_detail.lower()
    assert rows["LDR-551"].calibration_state is PodCalibrationState.OVERDUE
    assert "Overdue by" in rows["LDR-551"].calibration_detail


def test_registry_applies_verified_calibration_updates(tmp_path: Path) -> None:
    registry = PodInventoryRegistry(
        registry_path=tmp_path / "pod-inventory.json",
        due_soon_window_sec=14 * 24 * 60 * 60,
    )
    registry.sync_live_snapshots((_connected_snapshot(),), now_sec=1_700_000_000.0)

    updated = registry.apply_calibration_update(
        pod_serial="GAS-118",
        calibration=PodCalibrationMetadata(
            last_calibration_sec=1_700_000_000.0,
            next_calibration_due_sec=1_730_000_000.0,
        ),
        now_sec=1_700_000_005.0,
    )

    assert updated.calibration_state is PodCalibrationState.CURRENT
    assert "Current until" in updated.calibration_detail
