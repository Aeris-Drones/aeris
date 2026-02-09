from pathlib import Path
import subprocess


MISSION_CONTROL_HOOK = Path("software/viewer/src/hooks/useMissionControl.ts")
MISSION_CONTROL_PANEL = Path("software/viewer/src/components/mission/MissionControlPanel.tsx")
MISSION_CONTROL_BEHAVIOR_TEST = Path(
    "software/viewer/src/lib/missionControlBehavior.test.mjs"
)


def _read(path: Path) -> str:
    assert path.is_file(), f"missing file: {path}"
    return path.read_text()


def test_abort_mission_hook_uses_timeout_and_validation_helpers() -> None:
    source = _read(MISSION_CONTROL_HOOK)

    assert "const MISSION_SERVICE_TIMEOUT_MS = 8000;" in source
    assert "withServiceTimeout(" in source
    assert "computeMissionControlFlags({" in source
    assert "getAbortMissionValidationError({" in source
    assert "canAbort: controlFlags.canAbort" in source
    assert "contextAbort();" not in source


def test_mission_control_behavior_runtime_contracts() -> None:
    assert MISSION_CONTROL_BEHAVIOR_TEST.is_file()
    result = subprocess.run(
        ["node", "--test", str(MISSION_CONTROL_BEHAVIOR_TEST)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert (
        result.returncode == 0
    ), f"node runtime tests failed\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}"


def test_mission_control_panel_displays_abort_error_feedback() -> None:
    source = _read(MISSION_CONTROL_PANEL)

    assert "abortMissionError" in source
    assert "text-xs text-danger" in source
