from pathlib import Path
import json

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


def test_registry_skips_save_when_live_snapshot_is_unchanged(tmp_path: Path, monkeypatch) -> None:
    registry = PodInventoryRegistry(
        registry_path=tmp_path / "pod-inventory.json",
        due_soon_window_sec=14 * 24 * 60 * 60,
    )
    saves = []
    original_save = registry._save

    def tracked_save():
        saves.append("save")
        original_save()

    monkeypatch.setattr(registry, "_save", tracked_save)

    registry.sync_live_snapshots((_connected_snapshot(),), now_sec=1_700_000_000.0)
    registry.sync_live_snapshots((_connected_snapshot(),), now_sec=1_700_000_100.0)

    assert saves == ["save"]
    assert registry.snapshot(now_sec=1_700_000_100.0)[0].last_seen_sec == 1_700_000_100.0


def test_registry_warns_and_recovers_from_corrupted_file(tmp_path: Path) -> None:
    registry_path = tmp_path / "pod-inventory.json"
    registry_path.write_text("{not-json", encoding="utf-8")
    warnings = []

    registry = PodInventoryRegistry(
        registry_path=registry_path,
        due_soon_window_sec=14 * 24 * 60 * 60,
        on_warning=warnings.append,
    )

    assert registry.snapshot(now_sec=1_700_000_000.0) == ()
    assert warnings
    assert "starting empty" in warnings[0]


def test_registry_ignores_invalid_numeric_values_in_persisted_rows(tmp_path: Path) -> None:
    registry_path = tmp_path / "pod-inventory.json"
    registry_path.write_text(
        json.dumps(
            {
                "records": [
                    {
                        "pod_serial": "GAS-118",
                        "pod_type": "hazmat",
                        "one_wire_id": "28-0018",
                        "attached": False,
                        "vehicle_id": "",
                        "slot_id": "",
                        "lifecycle_state_label": "disconnected",
                        "connected": False,
                        "power_ready": False,
                        "link_ready": False,
                        "capabilities": ["gas"],
                        "first_seen_sec": "bad",
                        "last_seen_sec": "NaN",
                        "last_calibration_sec": "oops",
                        "next_calibration_due_sec": "Infinity",
                    }
                ]
            }
        ),
        encoding="utf-8",
    )

    registry = PodInventoryRegistry(
        registry_path=registry_path,
        due_soon_window_sec=14 * 24 * 60 * 60,
    )

    row = registry.snapshot(now_sec=1_700_000_000.0)[0]
    assert row.first_seen_sec == 0.0
    assert row.last_seen_sec == 0.0
    assert row.last_calibration_sec is None
    assert row.next_calibration_due_sec is None


def test_registry_saves_with_atomic_replace(tmp_path: Path, monkeypatch) -> None:
    registry_path = tmp_path / "pod-inventory.json"
    registry = PodInventoryRegistry(
        registry_path=registry_path,
        due_soon_window_sec=14 * 24 * 60 * 60,
    )
    replace_calls = []
    original_replace = Path.replace

    def tracked_replace(self, target):
        replace_calls.append((self.name, Path(target).name))
        return original_replace(self, target)

    monkeypatch.setattr(Path, "replace", tracked_replace)

    registry.sync_live_snapshots((_connected_snapshot(),), now_sec=1_700_000_000.0)

    assert replace_calls == [("pod-inventory.json.tmp", "pod-inventory.json")]
    assert registry_path.exists()
    assert not (tmp_path / "pod-inventory.json.tmp").exists()
    payload = json.loads(registry_path.read_text(encoding="utf-8"))
    assert payload["records"][0]["pod_serial"] == "GAS-118"


def test_registry_migrates_unknown_key_rows_when_serial_becomes_known(tmp_path: Path) -> None:
    registry = PodInventoryRegistry(
        registry_path=tmp_path / "pod-inventory.json",
        due_soon_window_sec=14 * 24 * 60 * 60,
    )

    registry.sync_live_snapshots(
        (
            _connected_snapshot(
                pod_serial="",
                pod_type="hazmat",
                one_wire_id="28-0018",
            ),
        ),
        now_sec=1_700_000_000.0,
    )
    registry.sync_live_snapshots((_connected_snapshot(),), now_sec=1_700_000_010.0)

    rows = registry.snapshot(now_sec=1_700_000_010.0)
    assert len(rows) == 1
    assert rows[0].pod_serial == "GAS-118"
