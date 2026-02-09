from pathlib import Path


MISSION_CONTROL_HOOK = Path("software/viewer/src/hooks/useMissionControl.ts")
MISSION_CONTROL_PANEL = Path("software/viewer/src/components/mission/MissionControlPanel.tsx")


def _read(path: Path) -> str:
    assert path.is_file(), f"missing file: {path}"
    return path.read_text()


def test_abort_mission_tracks_explicit_error_state_and_avoids_local_success_override() -> None:
    source = _read(MISSION_CONTROL_HOOK)

    assert "const [abortMissionError, setAbortMissionError]" in source
    assert "ROS is disconnected" in source
    assert "abort_mission rejected" in source
    assert "contextAbort();" not in source


def test_mission_control_panel_displays_abort_error_feedback() -> None:
    source = _read(MISSION_CONTROL_PANEL)

    assert "abortMissionError" in source
    assert "text-xs text-danger" in source
