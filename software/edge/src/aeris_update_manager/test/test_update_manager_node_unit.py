from __future__ import annotations

import threading
from types import ModuleType, SimpleNamespace
import sys


def _install_ros_stubs() -> None:
    rclpy_module = ModuleType("rclpy")
    rclpy_module.init = lambda *args, **kwargs: None
    rclpy_module.shutdown = lambda *args, **kwargs: None
    rclpy_module.spin = lambda *args, **kwargs: None

    rclpy_node_module = ModuleType("rclpy.node")

    class FakeNode:
        def __init__(self, *args, **kwargs) -> None:
            pass

    rclpy_node_module.Node = FakeNode

    rclpy_parameter_module = ModuleType("rclpy.parameter")

    class FakeParameter:
        pass

    rclpy_parameter_module.Parameter = FakeParameter

    builtin_msg_module = ModuleType("builtin_interfaces.msg")

    class FakeTime:
        def __init__(self) -> None:
            self.sec = 0
            self.nanosec = 0

    builtin_msg_module.Time = FakeTime

    aeris_msg_module = ModuleType("aeris_msgs.msg")

    class FakeFirmwareUpdateStatus:
        STATE_UNKNOWN = 0
        STATE_IDLE = 1
        STATE_DOWNLOADING = 2
        STATE_VALIDATING = 3
        STATE_APPLYING = 4
        STATE_VERIFYING = 5
        STATE_COMPLETE = 6
        STATE_FAILED = 7
        STATE_ROLLING_BACK = 8
        STATE_ROLLED_BACK = 9

        def __init__(self) -> None:
            self.vehicle_id = ""
            self.package_id = ""
            self.current_version = ""
            self.target_version = ""
            self.lifecycle_state = 0
            self.lifecycle_state_label = ""
            self.progress_percent = 0.0
            self.active_slot = ""
            self.inactive_slot = ""
            self.rollback_performed = False
            self.status_detail = ""
            self.error_code = ""
            self.error_detail = ""

    class FakeFirmwareUpdateStatusArray:
        def __init__(self) -> None:
            self.observed_at = None
            self.updates = []

    aeris_msg_module.FirmwareUpdateStatus = FakeFirmwareUpdateStatus
    aeris_msg_module.FirmwareUpdateStatusArray = FakeFirmwareUpdateStatusArray

    aeris_srv_module = ModuleType("aeris_msgs.srv")

    class FakeFirmwareUpdateCommandService:
        class Request:
            pass

        class Response:
            def __init__(self) -> None:
                self.accepted = False
                self.message = ""

    aeris_srv_module.FirmwareUpdateCommand = FakeFirmwareUpdateCommandService

    sys.modules.setdefault("rclpy", rclpy_module)
    sys.modules.setdefault("rclpy.node", rclpy_node_module)
    sys.modules.setdefault("rclpy.parameter", rclpy_parameter_module)
    sys.modules.setdefault("builtin_interfaces.msg", builtin_msg_module)
    sys.modules.setdefault("aeris_msgs.msg", aeris_msg_module)
    sys.modules.setdefault("aeris_msgs.srv", aeris_srv_module)


_install_ros_stubs()

from aeris_update_manager.models import (  # noqa: E402
    FirmwareUpdateCommand,
    FirmwareUpdateLifecycleState,
    FirmwareUpdateStatusSnapshot,
)
from aeris_update_manager.node import FirmwareUpdateManagerNode  # noqa: E402


class StreamingCoordinator:
    def __init__(self, published: list) -> None:
        self.published = published
        self.publish_counts_during_execution: list[int] = []

    def execute_update(self, command: FirmwareUpdateCommand, *, on_snapshot=None):
        snapshots = (
            FirmwareUpdateStatusSnapshot(
                vehicle_id=command.vehicle_id,
                package_id=command.package_id,
                current_version="2026.04.9",
                target_version=command.target_version,
                lifecycle_state=FirmwareUpdateLifecycleState.DOWNLOADING,
                progress_percent=10.0,
                active_slot="A",
                inactive_slot="B",
                status_detail="Staging signed package",
            ),
            FirmwareUpdateStatusSnapshot(
                vehicle_id=command.vehicle_id,
                package_id=command.package_id,
                current_version=command.target_version,
                target_version=command.target_version,
                lifecycle_state=FirmwareUpdateLifecycleState.COMPLETE,
                progress_percent=100.0,
                active_slot="B",
                inactive_slot="A",
                status_detail="Vehicle healthy on slot B",
            ),
        )
        history = []
        for snapshot in snapshots:
            history.append(snapshot)
            if on_snapshot is not None:
                on_snapshot(snapshot)
            self.publish_counts_during_execution.append(len(self.published))
        return tuple(history)


def _build_node(coordinator, published) -> FirmwareUpdateManagerNode:
    node = FirmwareUpdateManagerNode.__new__(FirmwareUpdateManagerNode)
    node._coordinator = coordinator
    node._latest_statuses = {}
    node._status_lock = threading.Lock()
    node._active_updates = set()
    node._active_updates_lock = threading.Lock()
    node._worker_threads = {}
    node._worker_commands = {}
    node._worker_threads_lock = threading.Lock()
    node._status_publisher = SimpleNamespace(publish=published.append)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=1_234_567_890)
    )
    node.get_logger = lambda: SimpleNamespace(
        error=lambda *_args, **_kwargs: None,
        info=lambda *_args, **_kwargs: None,
        warning=lambda *_args, **_kwargs: None,
    )
    return node


def _request():
    return SimpleNamespace(
        vehicle_id="scout_2",
        package_id="fw-2026.05.23",
        target_version="2026.05.23",
        package_uri="s3://updates/fw-2026.05.23.bin",
        package_signature="signed-manifest",
    )


def test_handle_command_returns_prompt_started_response_and_background_worker_streams_status() -> None:
    published = []
    coordinator = StreamingCoordinator(published)
    node = _build_node(coordinator, published)
    pending = []
    node._start_update_worker = lambda command: pending.append(  # type: ignore[method-assign]
        lambda: node._run_update(command)
    )

    request = _request()
    response = SimpleNamespace(accepted=False, message="")

    result = node._handle_command(request, response)

    assert result is response
    assert result.accepted is True
    assert result.message == "Firmware update started for scout_2"
    assert published == []
    assert pending
    assert "scout_2" in node._active_updates

    pending[0]()

    assert coordinator.publish_counts_during_execution == [1, 2]
    assert [message.updates[0].lifecycle_state_label for message in published] == [
        "downloading",
        "complete",
    ]
    assert "scout_2" not in node._active_updates


def test_handle_command_rejects_duplicate_inflight_updates_for_same_vehicle() -> None:
    published = []
    coordinator = StreamingCoordinator(published)
    node = _build_node(coordinator, published)
    pending = []
    node._start_update_worker = lambda command: pending.append(  # type: ignore[method-assign]
        lambda: node._run_update(command)
    )

    first = node._handle_command(_request(), SimpleNamespace(accepted=False, message=""))
    second = node._handle_command(_request(), SimpleNamespace(accepted=False, message=""))

    assert first.accepted is True
    assert second.accepted is False
    assert second.message == "Firmware update already in progress for scout_2"
    assert len(pending) == 1

    pending[0]()

    third = node._handle_command(_request(), SimpleNamespace(accepted=False, message=""))
    assert third.accepted is True


def test_start_update_worker_uses_daemon_thread_and_tracks_it() -> None:
    published = []
    node = _build_node(StreamingCoordinator(published), published)
    created = []

    class FakeThread:
        def __init__(self, *, target, args, daemon, name) -> None:
            self.target = target
            self.args = args
            self.daemon = daemon
            self.name = name
            self.started = False
            created.append(self)

        def start(self) -> None:
            self.started = True

    original_thread = threading.Thread
    threading.Thread = FakeThread  # type: ignore[assignment]
    try:
        node._start_update_worker(_request())
    finally:
        threading.Thread = original_thread  # type: ignore[assignment]

    assert len(created) == 1
    assert created[0].daemon is True
    assert created[0].started is True
    assert node._worker_threads["scout_2"] is created[0]
    assert node._worker_commands["scout_2"].vehicle_id == "scout_2"


def test_start_update_worker_rolls_back_tracking_when_thread_start_fails() -> None:
    published = []
    node = _build_node(StreamingCoordinator(published), published)

    class FakeThread:
        def __init__(self, *, target, args, daemon, name) -> None:
            self.target = target
            self.args = args
            self.daemon = daemon
            self.name = name

        def start(self) -> None:
            raise RuntimeError("thread resources exhausted")

    original_thread = threading.Thread
    threading.Thread = FakeThread  # type: ignore[assignment]
    try:
        try:
            node._start_update_worker(_request())
        except RuntimeError as error:
            assert str(error) == "thread resources exhausted"
        else:
            raise AssertionError("expected worker start to fail")
    finally:
        threading.Thread = original_thread  # type: ignore[assignment]

    assert node._worker_threads == {}
    assert node._worker_commands == {}


def test_destroy_node_waits_with_timeout_for_worker_completion() -> None:
    published = []
    node = _build_node(StreamingCoordinator(published), published)
    join_calls = []
    infos = []

    class FakeThread:
        name = "firmware-update-scout_2"
        alive_checks = 0

        def join(self, timeout=None) -> None:
            join_calls.append(timeout)

        def is_alive(self) -> bool:
            self.alive_checks += 1
            return self.alive_checks == 1

    node._worker_threads["scout_2"] = FakeThread()
    node._worker_commands["scout_2"] = FirmwareUpdateCommand(
        vehicle_id="scout_2",
        package_id="fw-2026.05.23",
        target_version="2026.05.23",
        package_uri="s3://updates/fw-2026.05.23.bin",
        package_signature="signed-manifest",
    )
    node.get_logger = lambda: SimpleNamespace(  # type: ignore[method-assign]
        error=lambda *_args, **_kwargs: None,
        info=lambda message: infos.append(message),
        warning=lambda *_args, **_kwargs: None,
    )

    result = node.destroy_node()

    assert infos == [
        "Waiting for firmware update worker to finish before shutdown: firmware-update-scout_2"
    ]
    assert join_calls == [node.SHUTDOWN_WORKER_JOIN_TIMEOUT_SEC]
    assert published == []
    assert result is None


def test_handle_command_start_failure_cleans_worker_tracking_for_shutdown() -> None:
    published = []
    node = _build_node(StreamingCoordinator(published), published)
    node._start_update_worker = lambda _command: (_ for _ in ()).throw(  # type: ignore[method-assign]
        RuntimeError("thread resources exhausted")
    )

    response = node._handle_command(_request(), SimpleNamespace(accepted=False, message=""))

    assert response.accepted is False
    assert response.message == "thread resources exhausted"
    assert node._active_updates == set()
    assert node._worker_threads == {}
    assert node._worker_commands == {}
    assert node.destroy_node() is None


def test_destroy_node_publishes_failed_snapshot_for_worker_shutdown_timeout() -> None:
    published = []
    node = _build_node(StreamingCoordinator(published), published)
    warnings = []
    node._latest_statuses["scout_2"] = FirmwareUpdateStatusSnapshot(
        vehicle_id="scout_2",
        package_id="fw-2026.05.23",
        current_version="2026.04.9",
        target_version="2026.05.23",
        lifecycle_state=FirmwareUpdateLifecycleState.APPLYING,
        progress_percent=55.0,
        active_slot="A",
        inactive_slot="B",
        status_detail="Applying package to slot B",
    )
    node._active_updates.add("scout_2")

    class FakeThread:
        name = "firmware-update-scout_2"

        def join(self, timeout=None) -> None:
            self.timeout = timeout

        def is_alive(self) -> bool:
            return True

    worker = FakeThread()
    node._worker_threads["scout_2"] = worker
    node._worker_commands["scout_2"] = FirmwareUpdateCommand(
        vehicle_id="scout_2",
        package_id="fw-2026.05.23",
        target_version="2026.05.23",
        package_uri="s3://updates/fw-2026.05.23.bin",
        package_signature="signed-manifest",
    )
    node.get_logger = lambda: SimpleNamespace(  # type: ignore[method-assign]
        error=lambda *_args, **_kwargs: None,
        info=lambda *_args, **_kwargs: None,
        warning=lambda message: warnings.append(message),
    )

    result = node.destroy_node()

    assert worker.timeout == node.SHUTDOWN_WORKER_JOIN_TIMEOUT_SEC
    assert [message.updates[0].lifecycle_state_label for message in published] == ["failed"]
    failed = published[0].updates[0]
    assert failed.error_code == "shutdown_timeout"
    assert failed.status_detail == "Firmware update did not finish before manager shutdown"
    assert failed.progress_percent == 55.0
    assert failed.active_slot == "A"
    assert warnings == [
        "Firmware update worker did not finish within 30s during shutdown: firmware-update-scout_2"
    ]
    assert node._active_updates == set()
    assert node._worker_threads == {}
    assert node._worker_commands == {}
    assert result is None
