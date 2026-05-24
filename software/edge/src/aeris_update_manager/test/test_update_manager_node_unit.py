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
    node._status_publisher = SimpleNamespace(publish=published.append)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=1_234_567_890)
    )
    node.get_logger = lambda: SimpleNamespace(error=lambda *_args, **_kwargs: None)
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
