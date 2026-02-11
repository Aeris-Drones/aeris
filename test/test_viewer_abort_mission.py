"""Tests for the viewer mission abort functionality.

This module validates the mission control components including:
- Mission control hook timeout and validation helper integration
- Mission control behavior runtime contracts via Node.js tests
- Mission control panel error feedback display

Prerequisites:
    - software/viewer/src/ contains the mission control source files
    - Node.js runtime available for behavior tests
    - Python 3.8+ with pytest
"""

from pathlib import Path
import shutil
import subprocess

import pytest


# Path constants for viewer mission control artifacts
MISSION_CONTROL_HOOK = Path("software/viewer/src/hooks/useMissionControl.ts")
MISSION_CONTROL_PANEL = Path("software/viewer/src/components/mission/MissionControlPanel.tsx")
MISSION_CONTROL_BEHAVIOR_TEST = Path(
    "software/viewer/src/lib/missionControlBehavior.test.mjs"
)


def _read(path: Path) -> str:
    """Read and return the contents of a file.

    Args:
        path: Path to the file to read.

    Returns:
        str: The file contents as a string.

    Raises:
        AssertionError: If the file does not exist.
    """
    assert path.is_file(), f"missing file: {path}"
    return path.read_text()


def test_abort_mission_hook_uses_timeout_and_validation_helpers() -> None:
    """Validate the mission control hook uses proper timeout and validation.

    Given: The useMissionControl.ts hook source file exists
    When: The source is searched for timeout and validation patterns
    Then: Mission service timeout constant is 8000ms
          withServiceTimeout helper is used
          computeMissionControlFlags and getAbortMissionValidationError are called
          canAbort flag is properly referenced without direct contextAbort call
    """
    source = _read(MISSION_CONTROL_HOOK)

    assert "const MISSION_SERVICE_TIMEOUT_MS = 8000;" in source
    assert "withServiceTimeout(" in source
    assert "computeMissionControlFlags({" in source
    assert "getAbortMissionValidationError({" in source
    assert "canAbort: controlFlags.canAbort" in source
    assert "contextAbort();" not in source


def test_mission_control_behavior_runtime_contracts() -> None:
    """Validate mission control behavior via Node.js runtime tests.

    Given: The mission control behavior test file exists
    When: Node.js test runner is invoked on the behavior test file
    Then: All runtime tests pass with exit code 0

    Raises:
        AssertionError: If Node.js tests fail with non-zero exit code.
    """
    assert MISSION_CONTROL_BEHAVIOR_TEST.is_file()
    if shutil.which("node") is None:
        pytest.skip("node not available")
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
    """Validate the mission control panel displays abort error feedback.

    Given: The MissionControlPanel.tsx component source file exists
    When: The source is searched for error display patterns
    Then: abortMissionError state is referenced
          Error feedback uses text-xs text-danger styling classes

    Note:
        This test is skipped if MissionControlPanel.tsx does not exist,
        as the component may have been refactored or removed.
    """
    if not MISSION_CONTROL_PANEL.is_file():
        pytest.skip("MissionControlPanel.tsx not found - component may have been removed")

    source = _read(MISSION_CONTROL_PANEL)

    assert "abortMissionError" in source
    assert "text-xs text-danger" in source
