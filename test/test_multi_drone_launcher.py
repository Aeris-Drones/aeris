"""Tests for the multi-drone SITL launcher module.

This module validates the run_multi_drone_sitl.py launcher behavior including:
- Process execution and error handling
- PX4 SITL process lifecycle management
- Signal handling for graceful shutdown
- Exit code validation

Prerequisites:
    - software/sim/tools/run_multi_drone_sitl.py exists
    - Python 3.8+ with pytest
"""

import importlib.util
from pathlib import Path

import pytest


# Path to the launcher script under test
SCRIPT_PATH = Path(__file__).resolve().parents[1] / "software/sim/tools/run_multi_drone_sitl.py"


def _load_launcher_module():
    """Dynamically load the launcher module for testing.

    Returns:
        module: The loaded run_multi_drone_sitl module.

    Raises:
        AssertionError: If the module spec or loader is None.
    """
    spec = importlib.util.spec_from_file_location("run_multi_drone_sitl", SCRIPT_PATH)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class _FakeProcess:
    """Mock process object for simulating subprocess behavior in tests.

    Attributes:
        _returncode: The exit code to return from wait().
        signals: List of signal numbers sent to this process.
    """

    def __init__(self, returncode: int) -> None:
        """Initialize the fake process with a predetermined return code.

        Args:
            returncode: The exit code to return from wait().
        """
        self._returncode = returncode
        self.signals = []

    def send_signal(self, signum):
        """Record a signal sent to this process.

        Args:
            signum: The signal number being sent.
        """
        self.signals.append(signum)

    def wait(self) -> int:
        """Wait for process completion and return the exit code.

        Returns:
            int: The predetermined return code.
        """
        return self._returncode


def test_execute_raises_when_px4_process_fails(monkeypatch) -> None:
    """Validate that execute() raises RuntimeError when a PX4 process fails.

    Given: Two vehicles configured for simulation
    When: The second PX4 process exits with return code 1
    Then: RuntimeError is raised with details of the failed process
    """
    launcher = _load_launcher_module()

    monkeypatch.setenv("PX4_BIN", "/tmp/px4")
    monkeypatch.setattr(launcher.shutil, "which", lambda _: "/tmp/px4")

    processes = [_FakeProcess(0), _FakeProcess(1)]
    process_iter = iter(processes)
    monkeypatch.setattr(
        launcher.subprocess, "Popen", lambda cmd, env: next(process_iter)
    )

    vehicles = [
        {"name": "scout1", "pose": "0 0 0 0 0 0", "mavlink_udp_port": 14540},
        {"name": "scout2", "pose": "50 0 0 0 0 0", "mavlink_udp_port": 14541},
    ]

    with pytest.raises(RuntimeError, match="PX4 SITL process failed: scout2=1"):
        launcher.execute(vehicles)


def test_execute_succeeds_when_all_px4_processes_exit_zero(monkeypatch) -> None:
    """Validate that execute() completes successfully when all processes exit cleanly.

    Given: Two vehicles configured for simulation
    When: All PX4 processes exit with return code 0
    Then: execute() returns without raising an exception
    """
    launcher = _load_launcher_module()

    monkeypatch.setenv("PX4_BIN", "/tmp/px4")
    monkeypatch.setattr(launcher.shutil, "which", lambda _: "/tmp/px4")

    processes = [_FakeProcess(0), _FakeProcess(0)]
    process_iter = iter(processes)
    monkeypatch.setattr(
        launcher.subprocess, "Popen", lambda cmd, env: next(process_iter)
    )

    vehicles = [
        {"name": "scout1", "pose": "0 0 0 0 0 0", "mavlink_udp_port": 14540},
        {"name": "scout2", "pose": "50 0 0 0 0 0", "mavlink_udp_port": 14541},
    ]

    launcher.execute(vehicles)


def test_execute_does_not_raise_on_user_interrupt_shutdown(monkeypatch) -> None:
    """Validate graceful shutdown handling on SIGINT (Ctrl+C).

    Given: Two vehicles configured for simulation
    When: A process triggers SIGINT during wait()
    Then: execute() handles the interrupt gracefully without raising
    """
    launcher = _load_launcher_module()

    monkeypatch.setenv("PX4_BIN", "/tmp/px4")
    monkeypatch.setattr(launcher.shutil, "which", lambda _: "/tmp/px4")

    handlers = {}

    def _register_handler(signum, handler):
        handlers[signum] = handler

    monkeypatch.setattr(launcher.signal, "signal", _register_handler)

    class _InterruptingProcess(_FakeProcess):
        """Fake process that triggers signal handlers during wait()."""

        def __init__(self, returncode: int, trigger_interrupt: bool = False) -> None:
            """Initialize with optional interrupt trigger.

            Args:
                returncode: The exit code to return.
                trigger_interrupt: Whether to trigger SIGINT during wait.
            """
            super().__init__(returncode)
            self._trigger_interrupt = trigger_interrupt

        def wait(self) -> int:
            """Wait for process, optionally triggering interrupt handler.

            Returns:
                int: The process return code.
            """
            if self._trigger_interrupt:
                handlers[launcher.signal.SIGINT](launcher.signal.SIGINT, None)
                self._trigger_interrupt = False
            return super().wait()

    processes = [
        _InterruptingProcess(-launcher.signal.SIGINT, trigger_interrupt=True),
        _InterruptingProcess(-launcher.signal.SIGINT),
    ]
    process_iter = iter(processes)
    monkeypatch.setattr(
        launcher.subprocess, "Popen", lambda cmd, env: next(process_iter)
    )

    vehicles = [
        {"name": "scout1", "pose": "0 0 0 0 0 0", "mavlink_udp_port": 14540},
        {"name": "scout2", "pose": "50 0 0 0 0 0", "mavlink_udp_port": 14541},
    ]

    launcher.execute(vehicles)
