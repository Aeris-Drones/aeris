import threading
import time

import pytest
import rclpy
from aeris_msgs.msg import MissionProgress, MissionState
from aeris_msgs.srv import MissionCommand
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from aeris_orchestrator.mission_node import MissionNode


def _wait_until(predicate, timeout_sec: float = 5.0, sleep_sec: float = 0.02) -> bool:
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(sleep_sec)
    return False


class MissionObserver(Node):
    def __init__(self) -> None:
        super().__init__("aeris_mission_observer_test")
        self.states: list[str] = []
        self.progress_updates: list[MissionProgress] = []
        self._state_lock = threading.Lock()
        self._progress_lock = threading.Lock()

        self._state_subscription = self.create_subscription(
            MissionState, "/orchestrator/mission_state", self._on_state, 10
        )
        self._progress_subscription = self.create_subscription(
            MissionProgress, "/orchestrator/mission_progress", self._on_progress, 10
        )
        self.start_client = self.create_client(MissionCommand, "start_mission")
        self.abort_client = self.create_client(MissionCommand, "abort_mission")

    def _on_state(self, message: MissionState) -> None:
        with self._state_lock:
            self.states.append(message.state)

    def _on_progress(self, message: MissionProgress) -> None:
        with self._progress_lock:
            self.progress_updates.append(message)


@pytest.fixture(scope="module")
def ros_runtime():
    rclpy.init()
    executor = MultiThreadedExecutor()
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        yield executor
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        rclpy.shutdown()


@pytest.fixture
def mission_harness(ros_runtime: MultiThreadedExecutor):
    mission_node = MissionNode()
    observer = MissionObserver()
    ros_runtime.add_node(mission_node)
    ros_runtime.add_node(observer)

    assert observer.start_client.wait_for_service(timeout_sec=2.0)
    assert observer.abort_client.wait_for_service(timeout_sec=2.0)

    try:
        yield mission_node, observer
    finally:
        mission_node._control_timer.cancel()
        mission_node._progress_timer.cancel()
        ros_runtime.remove_node(observer)
        ros_runtime.remove_node(mission_node)
        observer.destroy_node()
        mission_node.destroy_node()


def test_start_mission_transitions_to_planning_then_searching(mission_harness) -> None:
    _, observer = mission_harness

    request = MissionCommand.Request()
    request.command = "START"
    request.mission_id = "integration-start"
    request.zone_geometry = "{}"
    result_future = observer.start_client.call_async(request)

    assert _wait_until(lambda: result_future.done())
    response = result_future.result()
    assert response is not None
    assert response.success

    assert _wait_until(
        lambda: "PLANNING" in observer.states and "SEARCHING" in observer.states
    )
    planning_index = observer.states.index("PLANNING")
    searching_index = observer.states.index("SEARCHING")
    assert planning_index < searching_index

    assert _wait_until(lambda: len(observer.progress_updates) > 0)
    assert _wait_until(lambda: "TRACKING" in observer.states)


def test_start_mission_rejects_invalid_command(mission_harness) -> None:
    _, observer = mission_harness

    request = MissionCommand.Request()
    request.command = "ABORT"
    request.mission_id = "invalid-start-command"
    request.zone_geometry = "{}"
    result_future = observer.start_client.call_async(request)

    assert _wait_until(lambda: result_future.done())
    response = result_future.result()
    assert response is not None
    assert not response.success
    assert "unsupported command" in response.message


def test_abort_mission_transitions_to_aborted(mission_harness) -> None:
    _, observer = mission_harness

    start_request = MissionCommand.Request()
    start_request.command = "START"
    start_request.mission_id = "integration-abort"
    start_request.zone_geometry = "{}"
    start_future = observer.start_client.call_async(start_request)

    assert _wait_until(lambda: start_future.done())
    assert start_future.result() is not None
    assert start_future.result().success
    assert _wait_until(lambda: "SEARCHING" in observer.states)

    abort_request = MissionCommand.Request()
    abort_request.command = "ABORT"
    abort_request.mission_id = "integration-abort"
    abort_request.zone_geometry = ""
    abort_future = observer.abort_client.call_async(abort_request)

    assert _wait_until(lambda: abort_future.done())
    assert abort_future.result() is not None
    assert abort_future.result().success
    assert _wait_until(lambda: "ABORTED" in observer.states)


def test_abort_mission_rejects_invalid_command(mission_harness) -> None:
    _, observer = mission_harness

    start_request = MissionCommand.Request()
    start_request.command = "START"
    start_request.mission_id = "invalid-abort-command"
    start_request.zone_geometry = "{}"
    start_future = observer.start_client.call_async(start_request)

    assert _wait_until(lambda: start_future.done())
    assert start_future.result() is not None
    assert start_future.result().success
    assert _wait_until(lambda: "SEARCHING" in observer.states)

    abort_request = MissionCommand.Request()
    abort_request.command = "START"
    abort_request.mission_id = "invalid-abort-command"
    abort_request.zone_geometry = ""
    abort_future = observer.abort_client.call_async(abort_request)

    assert _wait_until(lambda: abort_future.done())
    response = abort_future.result()
    assert response is not None
    assert not response.success
    assert "unsupported command" in response.message
