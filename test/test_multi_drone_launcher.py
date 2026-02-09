import importlib.util
from pathlib import Path

import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "software/sim/tools/run_multi_drone_sitl.py"


def _load_launcher_module():
    spec = importlib.util.spec_from_file_location("run_multi_drone_sitl", SCRIPT_PATH)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class _FakeProcess:
    def __init__(self, returncode: int) -> None:
        self._returncode = returncode
        self.signals = []

    def send_signal(self, signum):
        self.signals.append(signum)

    def wait(self) -> int:
        return self._returncode


def test_execute_raises_when_px4_process_fails(monkeypatch) -> None:
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
    launcher = _load_launcher_module()

    monkeypatch.setenv("PX4_BIN", "/tmp/px4")
    monkeypatch.setattr(launcher.shutil, "which", lambda _: "/tmp/px4")

    handlers = {}

    def _register_handler(signum, handler):
        handlers[signum] = handler

    monkeypatch.setattr(launcher.signal, "signal", _register_handler)

    class _InterruptingProcess(_FakeProcess):
        def __init__(self, returncode: int, trigger_interrupt: bool = False) -> None:
            super().__init__(returncode)
            self._trigger_interrupt = trigger_interrupt

        def wait(self) -> int:
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
