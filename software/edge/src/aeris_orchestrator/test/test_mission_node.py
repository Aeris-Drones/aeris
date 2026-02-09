import json
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from aeris_msgs.msg import (
    AcousticBearing,
    GasIsopleth,
    MissionProgress,
    MissionState,
    Telemetry,
    ThermalHotspot,
)
from aeris_msgs.srv import MissionCommand
from geometry_msgs.msg import Point32, Polygon
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from aeris_orchestrator.mission_node import MissionNode


def _wait_until(predicate, timeout_sec: float = 5.0, sleep_sec: float = 0.02) -> bool:
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(sleep_sec)
    return False


VALID_ZONE_GEOMETRY = json.dumps(
    {
        "pattern": "lawnmower",
        "zone": {
            "id": "zone-alpha",
            "polygon": [
                {"x": 0.0, "z": 0.0},
                {"x": 12.0, "z": 0.0},
                {"x": 12.0, "z": 12.0},
                {"x": 0.0, "z": 12.0},
            ],
        },
    }
)


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
        self.telemetry_publisher = self.create_publisher(
            Telemetry, "/vehicle/telemetry", 10
        )
        self.thermal_publisher = self.create_publisher(
            ThermalHotspot, "thermal/hotspots", 10
        )
        self.acoustic_publisher = self.create_publisher(
            AcousticBearing, "acoustic/bearing", 10
        )
        self.gas_publisher = self.create_publisher(
            GasIsopleth, "gas/isopleth", 10
        )
        self.tracking_resolution_publisher = self.create_publisher(
            String, "/mission/tracking_resolution", 10
        )
        self.start_client = self.create_client(MissionCommand, "start_mission")
        self.abort_client = self.create_client(MissionCommand, "abort_mission")

    def _on_state(self, message: MissionState) -> None:
        with self._state_lock:
            self.states.append(message.state)

    def _on_progress(self, message: MissionProgress) -> None:
        with self._progress_lock:
            self.progress_updates.append(message)


def _publish_scout_telemetry(
    observer: MissionObserver,
    mission_node: MissionNode,
    vehicle_id: str = "scout1",
    *,
    latitude: float = 0.0,
    longitude: float = 0.0,
) -> None:
    telemetry = Telemetry()
    telemetry.vehicle_id = vehicle_id
    telemetry.vehicle_type = "scout"
    telemetry.position.latitude = latitude
    telemetry.position.longitude = longitude
    telemetry.timestamp = observer.get_clock().now().to_msg()
    observer.telemetry_publisher.publish(telemetry)
    normalized_id = mission_node._normalize_vehicle_id(vehicle_id)
    assert _wait_until(
        lambda: normalized_id in mission_node._scout_last_seen_monotonic
    )


@pytest.fixture
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
        ros_runtime.remove_node(observer)
        ros_runtime.remove_node(mission_node)
        observer.destroy_node()
        mission_node.destroy_node()


def test_start_mission_transitions_to_planning_then_searching(mission_harness) -> None:
    mission_node, observer = mission_harness
    _publish_scout_telemetry(observer, mission_node)

    request = MissionCommand.Request()
    request.command = "START"
    request.mission_id = "integration-start"
    request.zone_geometry = VALID_ZONE_GEOMETRY
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
    assert _wait_until(lambda: mission_node._mavlink_adapter.is_streaming)  # noqa: SLF001


def test_start_mission_rejects_invalid_command(mission_harness) -> None:
    _, observer = mission_harness

    request = MissionCommand.Request()
    request.command = "ABORT"
    request.mission_id = "invalid-start-command"
    request.zone_geometry = VALID_ZONE_GEOMETRY
    result_future = observer.start_client.call_async(request)

    assert _wait_until(lambda: result_future.done())
    response = result_future.result()
    assert response is not None
    assert not response.success
    assert "unsupported command" in response.message


def test_abort_mission_transitions_to_aborted(mission_harness) -> None:
    mission_node, observer = mission_harness
    _publish_scout_telemetry(observer, mission_node)

    start_request = MissionCommand.Request()
    start_request.command = "START"
    start_request.mission_id = "integration-abort"
    start_request.zone_geometry = VALID_ZONE_GEOMETRY
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
    assert not mission_node._mavlink_adapter.is_streaming


def test_abort_mission_rejects_invalid_command(mission_harness) -> None:
    mission_node, observer = mission_harness
    _publish_scout_telemetry(observer, mission_node)

    start_request = MissionCommand.Request()
    start_request.command = "START"
    start_request.mission_id = "invalid-abort-command"
    start_request.zone_geometry = VALID_ZONE_GEOMETRY
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


def test_abort_mission_rejects_mission_id_mismatch(mission_harness) -> None:
    mission_node, observer = mission_harness
    _publish_scout_telemetry(observer, mission_node)

    start_request = MissionCommand.Request()
    start_request.command = "START"
    start_request.mission_id = "mismatch-abort"
    start_request.zone_geometry = VALID_ZONE_GEOMETRY
    start_future = observer.start_client.call_async(start_request)

    assert _wait_until(lambda: start_future.done())
    assert start_future.result() is not None
    assert start_future.result().success
    assert _wait_until(lambda: "SEARCHING" in observer.states)

    abort_request = MissionCommand.Request()
    abort_request.command = "ABORT"
    abort_request.mission_id = "other-mission-id"
    abort_request.zone_geometry = ""
    abort_future = observer.abort_client.call_async(abort_request)

    assert _wait_until(lambda: abort_future.done())
    response = abort_future.result()
    assert response is not None
    assert not response.success
    assert "mission_id mismatch" in response.message
    assert "ABORTED" not in observer.states


def test_start_mission_rejects_missing_zone_geometry(mission_harness) -> None:
    _, observer = mission_harness

    request = MissionCommand.Request()
    request.command = "START"
    request.mission_id = "missing-zone-geometry"
    request.zone_geometry = ""
    result_future = observer.start_client.call_async(request)

    assert _wait_until(lambda: result_future.done())
    response = result_future.result()
    assert response is not None
    assert not response.success
    assert "zone_geometry is required" in response.message


def test_start_mission_rejects_unsupported_pattern(mission_harness) -> None:
    _, observer = mission_harness

    request = MissionCommand.Request()
    request.command = "START"
    request.mission_id = "bad-pattern"
    request.zone_geometry = json.dumps(
        {
            "pattern": "figure8",
            "zone": {
                "id": "zone-bad-pattern",
                "polygon": [
                    {"x": 0.0, "z": 0.0},
                    {"x": 3.0, "z": 0.0},
                    {"x": 3.0, "z": 3.0},
                ],
            },
        }
    )
    result_future = observer.start_client.call_async(request)

    assert _wait_until(lambda: result_future.done())
    response = result_future.result()
    assert response is not None
    assert not response.success
    assert "unsupported pattern" in response.message


def test_start_mission_rejects_non_finite_polygon_coordinates(mission_harness) -> None:
    _, observer = mission_harness

    request = MissionCommand.Request()
    request.command = "START"
    request.mission_id = "non-finite"
    request.zone_geometry = json.dumps(
        {
            "pattern": "lawnmower",
            "zone": {
                "id": "zone-non-finite",
                "polygon": [
                    {"x": 0.0, "z": 0.0},
                    {"x": 10.0, "z": 0.0},
                    {"x": float("nan"), "z": 5.0},
                ],
            },
        }
    )
    result_future = observer.start_client.call_async(request)

    assert _wait_until(lambda: result_future.done())
    response = result_future.result()
    assert response is not None
    assert not response.success
    assert "invalid zone polygon" in response.message


def test_start_mission_prefers_recently_seen_scout_endpoint(mission_harness) -> None:
    mission_node, observer = mission_harness

    _publish_scout_telemetry(observer, mission_node, vehicle_id="scout2")

    request = MissionCommand.Request()
    request.command = "START"
    request.mission_id = "prefer-seen-scout"
    request.zone_geometry = VALID_ZONE_GEOMETRY
    result_future = observer.start_client.call_async(request)

    assert _wait_until(lambda: result_future.done())
    response = result_future.result()
    assert response is not None
    assert response.success
    assert "scout 'scout_2'" in response.message
    assert mission_node._active_scout_vehicle_id == "scout_2"  # noqa: SLF001


def test_abort_mission_dispatches_rtl_to_all_active_endpoints_best_effort(
    mission_harness, monkeypatch
) -> None:
    mission_node, observer = mission_harness
    _publish_scout_telemetry(observer, mission_node, vehicle_id="scout1")

    start_request = MissionCommand.Request()
    start_request.command = "START"
    start_request.mission_id = "fleet-abort"
    start_request.zone_geometry = VALID_ZONE_GEOMETRY
    start_future = observer.start_client.call_async(start_request)

    assert _wait_until(lambda: start_future.done())
    assert start_future.result() is not None
    assert start_future.result().success
    assert _wait_until(lambda: "SEARCHING" in observer.states)

    _publish_scout_telemetry(observer, mission_node, vehicle_id="scout2")

    endpoint_calls: list[tuple[str, int]] = []
    rtl_attempts: list[int] = []

    def _record_endpoint(host: str, port: int) -> None:
        endpoint_calls.append((host, int(port)))

    def _record_rtl_attempt() -> bool:
        rtl_attempts.append(1)
        if len(rtl_attempts) == 1:
            raise OSError("simulated endpoint write failure")
        return True

    monkeypatch.setattr(mission_node._mavlink_adapter, "set_endpoint", _record_endpoint)  # noqa: SLF001
    monkeypatch.setattr(  # noqa: SLF001
        mission_node._mavlink_adapter, "send_return_to_launch", _record_rtl_attempt
    )

    abort_request = MissionCommand.Request()
    abort_request.command = "ABORT"
    abort_request.mission_id = "fleet-abort"
    abort_request.zone_geometry = ""
    abort_future = observer.abort_client.call_async(abort_request)

    assert _wait_until(lambda: abort_future.done())
    response = abort_future.result()
    assert response is not None
    assert response.success
    assert _wait_until(lambda: "ABORTED" in observer.states)
    assert len(rtl_attempts) == 2
    assert sorted(endpoint_calls) == [("127.0.0.1", 14540), ("127.0.0.1", 14541)]


def test_start_mission_rejects_when_no_scout_is_online(mission_harness) -> None:
    _, observer = mission_harness

    request = MissionCommand.Request()
    request.command = "START"
    request.mission_id = "no-online-scout"
    request.zone_geometry = VALID_ZONE_GEOMETRY
    result_future = observer.start_client.call_async(request)

    assert _wait_until(lambda: result_future.done())
    response = result_future.result()
    assert response is not None
    assert not response.success
    assert "no scout endpoint reported telemetry recently" in response.message


@pytest.fixture
def mission_harness_detection(ros_runtime: MultiThreadedExecutor):
    mission_node = MissionNode(
        parameter_overrides=[
            Parameter("enable_tracking_simulation", value=False),
            Parameter("tracking_timeout_sec", value=0.25),
            Parameter("tracking_dispatch_budget_ms", value=200.0),
            Parameter("detection_stale_ms", value=500.0),
            Parameter("replan_cooldown_sec", value=1.0),
        ]
    )
    observer = MissionObserver()
    ros_runtime.add_node(mission_node)
    ros_runtime.add_node(observer)

    assert observer.start_client.wait_for_service(timeout_sec=2.0)
    assert observer.abort_client.wait_for_service(timeout_sec=2.0)

    try:
        yield mission_node, observer
    finally:
        ros_runtime.remove_node(observer)
        ros_runtime.remove_node(mission_node)
        observer.destroy_node()
        mission_node.destroy_node()


def _start_searching_mission(mission_node: MissionNode, observer: MissionObserver) -> None:
    _publish_scout_telemetry(observer, mission_node, vehicle_id="scout1")
    request = MissionCommand.Request()
    request.command = "START"
    request.mission_id = "detection-mission"
    request.zone_geometry = VALID_ZONE_GEOMETRY
    future = observer.start_client.call_async(request)
    assert _wait_until(lambda: future.done())
    response = future.result()
    assert response is not None
    assert response.success
    assert _wait_until(lambda: mission_node._state == "SEARCHING")  # noqa: SLF001


def _publish_thermal(
    observer: MissionObserver,
    *,
    confidence: float = 0.95,
    stamp_offset_sec: float = 0.0,
    bbox: tuple[int, int, int, int] = (300, 220, 340, 260),
) -> None:
    message = ThermalHotspot()
    now = observer.get_clock().now().nanoseconds / 1e9 + stamp_offset_sec
    seconds = max(0.0, now)
    sec = int(seconds)
    nanosec = int((seconds - sec) * 1e9)
    message.stamp.sec = sec
    message.stamp.nanosec = nanosec
    message.confidence = float(confidence)
    message.bbox_px = [int(value) for value in bbox]
    message.frame_id = "thermal_front"
    observer.thermal_publisher.publish(message)


def _publish_acoustic(
    observer: MissionObserver,
    *,
    confidence: float = 0.9,
    bearing_deg: float = 90.0,
    stamp_offset_sec: float = 0.0,
) -> None:
    message = AcousticBearing()
    now = observer.get_clock().now().nanoseconds / 1e9 + stamp_offset_sec
    seconds = max(0.0, now)
    sec = int(seconds)
    nanosec = int((seconds - sec) * 1e9)
    message.stamp.sec = sec
    message.stamp.nanosec = nanosec
    message.confidence = float(confidence)
    message.bearing_deg = float(bearing_deg)
    observer.acoustic_publisher.publish(message)


def _publish_gas(
    observer: MissionObserver,
    *,
    x: float = 10.0,
    y: float = 5.0,
    stamp_offset_sec: float = 0.0,
) -> None:
    message = GasIsopleth()
    now = observer.get_clock().now().nanoseconds / 1e9 + stamp_offset_sec
    seconds = max(0.0, now)
    sec = int(seconds)
    nanosec = int((seconds - sec) * 1e9)
    message.stamp.sec = sec
    message.stamp.nanosec = nanosec
    message.centerline = [
        Point32(x=float(x), y=float(y), z=0.0),
        Point32(x=float(x + 0.5), y=float(y + 0.2), z=0.0),
    ]
    message.polygons = [
        Polygon(
            points=[
                Point32(x=float(x - 1.0), y=float(y - 1.0), z=0.0),
                Point32(x=float(x + 1.0), y=float(y - 1.0), z=0.0),
                Point32(x=float(x + 1.0), y=float(y + 1.0), z=0.0),
                Point32(x=float(x - 1.0), y=float(y + 1.0), z=0.0),
            ]
        )
    ]
    observer.gas_publisher.publish(message)


def test_detection_acceptance_transitions_to_tracking_and_resumes_searching(
    mission_harness_detection,
) -> None:
    mission_node, observer = mission_harness_detection
    _start_searching_mission(mission_node, observer)

    _publish_thermal(observer, confidence=0.95)
    assert _wait_until(lambda: "TRACKING" in observer.states)
    assert mission_node._state == "TRACKING"  # noqa: SLF001

    signal = String()
    signal.data = "resolved"
    observer.tracking_resolution_publisher.publish(signal)
    assert _wait_until(lambda: mission_node._state == "SEARCHING")  # noqa: SLF001


def test_detection_rejects_low_confidence_stale_and_cooldown(mission_harness_detection) -> None:
    mission_node, observer = mission_harness_detection
    _start_searching_mission(mission_node, observer)
    mission_node._thermal_confidence_min = 0.9  # noqa: SLF001

    _publish_thermal(observer, confidence=0.5)
    assert _wait_until(
        lambda: "below threshold" in mission_node._last_detection_rejection_reason  # noqa: SLF001
    )
    assert mission_node._state == "SEARCHING"  # noqa: SLF001

    _publish_acoustic(observer, confidence=0.95, stamp_offset_sec=-10.0)
    assert _wait_until(
        lambda: "stale" in mission_node._last_detection_rejection_reason  # noqa: SLF001
    )
    assert mission_node._state == "SEARCHING"  # noqa: SLF001

    _publish_thermal(observer, confidence=0.99)
    assert _wait_until(lambda: mission_node._state == "TRACKING")  # noqa: SLF001
    signal = String()
    signal.data = "resolved"
    observer.tracking_resolution_publisher.publish(signal)
    assert _wait_until(lambda: mission_node._state == "SEARCHING")  # noqa: SLF001

    _publish_gas(observer, x=1.0, y=1.0)
    assert _wait_until(
        lambda: "cooldown active" in mission_node._last_detection_rejection_reason  # noqa: SLF001
    )
    assert mission_node._state == "SEARCHING"  # noqa: SLF001


def test_detection_selects_nearest_online_scout_and_records_dispatch_latency(
    mission_harness_detection,
) -> None:
    mission_node, observer = mission_harness_detection
    _publish_scout_telemetry(
        observer, mission_node, vehicle_id="scout1", latitude=0.0, longitude=0.0
    )
    _publish_scout_telemetry(
        observer, mission_node, vehicle_id="scout2", latitude=50.0, longitude=50.0
    )
    _start_searching_mission(mission_node, observer)

    _publish_gas(observer, x=49.8, y=50.2)
    assert _wait_until(lambda: mission_node._state == "TRACKING")  # noqa: SLF001
    assert mission_node._active_scout_vehicle_id == "scout_2"  # noqa: SLF001
    assert mission_node._last_tracking_dispatch_latency_ms >= 0.0  # noqa: SLF001
    assert (
        mission_node._last_tracking_dispatch_latency_ms  # noqa: SLF001
        <= mission_node._tracking_dispatch_budget_ms  # noqa: SLF001
    )


def test_detection_selection_falls_back_to_most_recent_seen_without_positions(
    mission_harness_detection,
) -> None:
    mission_node, observer = mission_harness_detection
    _publish_scout_telemetry(observer, mission_node, vehicle_id="scout1")
    time.sleep(0.02)
    _publish_scout_telemetry(observer, mission_node, vehicle_id="scout2")

    mission_node._scout_position_snapshot.clear()  # noqa: SLF001
    selected = mission_node._select_tracking_scout_endpoint({"x": 0.0, "z": 0.0})  # noqa: SLF001
    assert selected is not None
    assert selected.vehicle_id == "scout_2"
