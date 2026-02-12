"""Integration tests for mission_node state machine and vehicle coordination.

Test coverage includes:
- Mission lifecycle: START, SEARCHING, TRACKING, ABORTED
- Multi-vehicle assignment and zone partitioning
- Detection-triggered tracking with nearest-scout selection
- VIO odometry integration for GPS-denied navigation
- Return-to-launch path planning from breadcrumb trails
- Ranger overwatch orbit management and failover
- Vehicle command service (HOLD, RECALL, RESUME)
- Position source selection: telemetry vs VIO odometry
"""

import json
import math
import threading
import time
from types import SimpleNamespace

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
from aeris_msgs.srv import MissionCommand, VehicleCommand
from geometry_msgs.msg import Point32, Polygon
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from aeris_orchestrator.mission_node import MissionNode, MissionPlanState, ScoutEndpoint


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
        self.vehicle_command_client = self.create_client(
            VehicleCommand, "vehicle_command"
        )

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


def _publish_ranger_telemetry(
    observer: MissionObserver,
    mission_node: MissionNode,
    vehicle_id: str = "ranger1",
    *,
    latitude: float = 0.0,
    longitude: float = 0.0,
) -> None:
    telemetry = Telemetry()
    telemetry.vehicle_id = vehicle_id
    telemetry.vehicle_type = "ranger"
    telemetry.position.latitude = latitude
    telemetry.position.longitude = longitude
    telemetry.timestamp = observer.get_clock().now().to_msg()
    observer.telemetry_publisher.publish(telemetry)
    normalized_id = mission_node._normalize_vehicle_id(vehicle_id)
    assert _wait_until(
        lambda: normalized_id in mission_node._ranger_last_seen_monotonic
    )


def _publish_scout_odometry(
    observer: MissionObserver,
    mission_node: MissionNode,
    vehicle_id: str = "scout1",
    *,
    x_m: float = 0.0,
    y_m: float = 0.0,
) -> None:
    normalized_id = mission_node._normalize_vehicle_id(vehicle_id)
    topic_token = mission_node._vehicle_model_topic_token(vehicle_id)
    odometry_topic = mission_node._scout_odometry_topics.get(
        normalized_id, f"/{topic_token}/openvins/odom"
    )
    publisher = observer.create_publisher(Odometry, odometry_topic, 10)
    message = Odometry()
    message.header.stamp = observer.get_clock().now().to_msg()
    message.header.frame_id = "odom"
    message.child_frame_id = "base_link"
    message.pose.pose.position.x = float(x_m)
    message.pose.pose.position.y = float(y_m)
    message.pose.pose.position.z = 0.0
    publisher.publish(message)
    assert _wait_until(
        lambda: normalized_id in mission_node._scout_vio_last_seen_monotonic
    )


def _inject_scout_odometry(
    mission_node: MissionNode,
    observer: MissionObserver,
    vehicle_id: str,
    *,
    x_m: float,
    y_m: float,
) -> None:
    message = Odometry()
    message.header.stamp = observer.get_clock().now().to_msg()
    message.header.frame_id = "odom"
    message.child_frame_id = "base_link"
    message.pose.pose.position.x = float(x_m)
    message.pose.pose.position.y = float(y_m)
    message.pose.pose.position.z = 0.0
    mission_node._handle_scout_odometry(vehicle_id, message)


def _publish_occupancy_map(
    observer: MissionObserver,
    mission_node: MissionNode | None = None,
    *,
    width: int = 20,
    height: int = 20,
    resolution_m: float = 1.0,
    origin_x: float = -10.0,
    origin_y: float = -10.0,
    occupied_indices: set[int] | None = None,
) -> None:
    publisher = observer.create_publisher(OccupancyGrid, "/map", 10)
    message = OccupancyGrid()
    message.header.stamp = observer.get_clock().now().to_msg()
    message.header.frame_id = "map"
    message.info.resolution = float(resolution_m)
    message.info.width = int(width)
    message.info.height = int(height)
    message.info.origin.position.x = float(origin_x)
    message.info.origin.position.y = float(origin_y)
    total_cells = int(width) * int(height)
    data = [0] * total_cells
    for index in occupied_indices or set():
        if 0 <= index < total_cells:
            data[index] = 100
    message.data = data
    publisher.publish(message)
    if mission_node is not None:
        assert _wait_until(lambda: mission_node._occupancy_map_snapshot is not None)


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
    assert observer.vehicle_command_client.wait_for_service(timeout_sec=2.0)

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
    assert not _wait_until(lambda: "TRACKING" in observer.states, timeout_sec=1.5)
    assert _wait_until(lambda: mission_node._mavlink_adapter.is_streaming)


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
    assert mission_node._active_scout_vehicle_id == "scout_2"


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

    def _record_endpoint(
        host: str, port: int, command_port: int | None = None
    ) -> None:
        del command_port
        endpoint_calls.append((host, int(port)))

    def _record_rtl_attempt() -> bool:
        rtl_attempts.append(1)
        if len(rtl_attempts) == 1:
            raise OSError("simulated endpoint write failure")
        return True

    monkeypatch.setattr(mission_node._mavlink_adapter, "set_endpoint", _record_endpoint)
    monkeypatch.setattr(
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
    assert sorted(endpoint_calls) == [("127.0.0.1", 14541), ("127.0.0.1", 14542)]


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


@pytest.fixture
def mission_harness_multi_vehicle(ros_runtime: MultiThreadedExecutor):
    mission_node = MissionNode(
        parameter_overrides=[
            Parameter(
                "ranger_endpoints_json",
                value='[{"vehicle_id":"ranger1","host":"127.0.0.1","port":14543}]',
            ),
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


@pytest.fixture
def mission_harness_vio(ros_runtime: MultiThreadedExecutor):
    mission_node = MissionNode(
        parameter_overrides=[
            Parameter("navigation_position_source", value="vio_odometry"),
            Parameter("vio_odom_stale_sec", value=0.1),
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


@pytest.fixture
def mission_harness_auto(ros_runtime: MultiThreadedExecutor):
    mission_node = MissionNode(
        parameter_overrides=[
            Parameter("navigation_position_source", value="auto"),
            Parameter("vio_odom_stale_sec", value=0.1),
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


@pytest.fixture
def mission_harness_auto_gps_denied(ros_runtime: MultiThreadedExecutor):
    mission_node = MissionNode(
        parameter_overrides=[
            Parameter("navigation_position_source", value="auto"),
            Parameter("gps_denied_mode", value=True),
            Parameter("vio_odom_stale_sec", value=0.1),
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


@pytest.fixture
def mission_harness_vio_return(ros_runtime: MultiThreadedExecutor):
    mission_node = MissionNode(
        parameter_overrides=[
            Parameter("navigation_position_source", value="vio_odometry"),
            Parameter("gps_denied_mode", value=True),
            Parameter("vio_odom_stale_sec", value=0.25),
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


def _start_searching_mission(
    mission_node: MissionNode,
    observer: MissionObserver,
    *,
    publish_telemetry: bool = True,
) -> None:
    if publish_telemetry:
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
    assert _wait_until(lambda: mission_node._state == "SEARCHING")


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
    assert mission_node._state == "TRACKING"

    signal = String()
    signal.data = "resolved"
    observer.tracking_resolution_publisher.publish(signal)
    assert _wait_until(lambda: mission_node._state == "SEARCHING")
    assert mission_node._last_tracking_completion_reason == "resolution signal"


def test_detection_rejects_low_confidence_stale_and_cooldown(mission_harness_detection) -> None:
    mission_node, observer = mission_harness_detection
    _start_searching_mission(mission_node, observer)
    mission_node._thermal_confidence_min = 0.9

    _publish_thermal(observer, confidence=0.5)
    assert _wait_until(
        lambda: "below threshold" in mission_node._last_detection_rejection_reason
    )
    assert mission_node._state == "SEARCHING"

    _publish_acoustic(observer, confidence=0.95, stamp_offset_sec=-10.0)
    assert _wait_until(
        lambda: "stale" in mission_node._last_detection_rejection_reason
    )
    assert mission_node._state == "SEARCHING"

    _publish_thermal(observer, confidence=0.99)
    assert _wait_until(lambda: mission_node._state == "TRACKING")
    signal = String()
    signal.data = "resolved"
    observer.tracking_resolution_publisher.publish(signal)
    assert _wait_until(lambda: mission_node._state == "SEARCHING")

    _publish_gas(observer, x=1.0, y=1.0)
    assert _wait_until(
        lambda: "cooldown active" in mission_node._last_detection_rejection_reason
    )
    assert mission_node._state == "SEARCHING"


def test_detection_selects_nearest_online_scout_and_records_dispatch_latency(
    mission_harness_detection,
) -> None:
    mission_node, observer = mission_harness_detection
    _publish_scout_telemetry(
        observer, mission_node, vehicle_id="scout1", latitude=0.0, longitude=0.0
    )
    _publish_scout_telemetry(
        observer, mission_node, vehicle_id="scout2", latitude=0.00045, longitude=0.00045
    )
    _start_searching_mission(mission_node, observer, publish_telemetry=False)

    _publish_gas(observer, x=50.0, y=50.0)
    assert _wait_until(lambda: mission_node._state == "TRACKING")
    assert mission_node._active_scout_vehicle_id == "scout_2"
    assert mission_node._last_tracking_dispatch_latency_ms >= 0.0
    assert (
        mission_node._last_tracking_dispatch_latency_ms
        <= mission_node._tracking_dispatch_budget_ms
    )


def test_detection_dispatch_preserves_non_target_vehicle_assignments(
    mission_harness_detection, monkeypatch
) -> None:
    mission_node, observer = mission_harness_detection
    _publish_scout_telemetry(
        observer, mission_node, vehicle_id="scout1", latitude=0.0, longitude=0.0
    )
    _publish_scout_telemetry(
        observer, mission_node, vehicle_id="scout2", latitude=0.00045, longitude=0.00045
    )
    _start_searching_mission(mission_node, observer, publish_telemetry=False)
    assert mission_node._active_scout_vehicle_id == "scout_1"

    endpoint_calls: list[tuple[str, int]] = []
    original_set_endpoint = mission_node._mavlink_adapter.set_endpoint

    def _record_endpoint(host: str, port: int, command_port: int | None = None) -> None:
        endpoint_calls.append((host, int(port)))
        if command_port is None:
            original_set_endpoint(host, port)
        else:
            original_set_endpoint(host, port, command_port=command_port)

    monkeypatch.setattr(mission_node._mavlink_adapter, "set_endpoint", _record_endpoint)

    _publish_gas(observer, x=50.0, y=50.0)
    assert _wait_until(lambda: mission_node._state == "TRACKING")
    assert mission_node._tracking_context.uses_dedicated_adapter
    assert mission_node._tracking_context.assigned_scout_vehicle_id == "scout_2"
    assert "scout_1" in mission_node._tracking_context.preserved_non_target_vehicle_ids
    assert mission_node._vehicle_assignments.get("scout_1") == "SEARCHING"
    assert mission_node._vehicle_assignments.get("scout_2") == "TRACKING"
    assert ("127.0.0.1", 14541) not in endpoint_calls

    signal = String()
    signal.data = "resolved"
    observer.tracking_resolution_publisher.publish(signal)
    assert _wait_until(lambda: mission_node._state == "SEARCHING")
    assert mission_node._vehicle_assignments.get("scout_2") == "SEARCHING"


def test_detection_latency_uses_adapter_dispatch_timestamp(
    mission_harness_detection, monkeypatch
) -> None:
    mission_node, observer = mission_harness_detection
    _start_searching_mission(mission_node, observer)

    def _fake_wait_for_setpoint_dispatch(*, after_monotonic: float, timeout_sec: float):
        del timeout_sec
        return after_monotonic + 0.150

    monkeypatch.setattr(
        mission_node._mavlink_adapter,
        "wait_for_setpoint_dispatch",
        _fake_wait_for_setpoint_dispatch,
    )

    _publish_thermal(observer, confidence=0.98)
    assert _wait_until(lambda: mission_node._state == "TRACKING")
    assert _wait_until(lambda: mission_node._last_tracking_dispatch_latency_ms > 0.0)
    assert mission_node._last_tracking_dispatch_latency_ms == pytest.approx(150.0, abs=10.0)


def test_detection_selection_falls_back_to_most_recent_seen_without_positions(
    mission_harness_detection,
) -> None:
    mission_node, observer = mission_harness_detection
    _publish_scout_telemetry(observer, mission_node, vehicle_id="scout1")
    time.sleep(0.02)
    _publish_scout_telemetry(observer, mission_node, vehicle_id="scout2")

    mission_node._scout_position_snapshot.clear()
    selected = mission_node._select_tracking_scout_endpoint({"x": 0.0, "z": 0.0})
    assert selected is not None
    assert selected.vehicle_id == "scout_2"


def test_tracking_selection_uses_vio_distance_when_configured(
    mission_harness_vio,
) -> None:
    mission_node, observer = mission_harness_vio
    now = time.monotonic()
    mission_node._scout_last_seen_monotonic = {
        "scout_1": now,
        "scout_2": now,
    }
    # Telemetry suggests scout_1 is closer, but VIO should take precedence.
    mission_node._scout_position_snapshot = {
        "scout_1": {"x": 0.0, "z": 0.0},
        "scout_2": {"x": 100.0, "z": 100.0},
    }
    _publish_scout_odometry(observer, mission_node, vehicle_id="scout1", x_m=100.0, y_m=100.0)
    _publish_scout_odometry(observer, mission_node, vehicle_id="scout2", x_m=1.0, y_m=1.0)

    selected = mission_node._select_tracking_scout_endpoint({"x": 0.0, "z": 0.0})
    assert selected is not None
    assert selected.vehicle_id == "scout_2"


def test_auto_position_source_falls_back_to_telemetry_and_prefers_fresh_vio(
    mission_harness_auto,
) -> None:
    mission_node, observer = mission_harness_auto
    mission_node._state = "SEARCHING"
    mission_node._mission_id = "auto-source"
    mission_node._active_scout_vehicle_id = "scout_1"
    mission_node._scout_plans = {
        "scout_1": MissionPlanState(
            pattern_type="lawnmower",
            zone_id="zone-auto",
            waypoints=[
                {"x": 0.0, "z": 0.0, "altitude_m": 20.0},
                {"x": 5.0, "z": 0.0, "altitude_m": 20.0},
            ],
            current_waypoint_index=0,
            scout_position={"x": -1.0, "z": 0.0},
        )
    }

    # No VIO available yet -> auto falls back to telemetry behavior.
    mission_node._advance_search_plan_for_vehicle("scout_1")
    assert mission_node._vehicle_position_sources["scout_1"] == "telemetry_geodetic"

    # Fresh VIO sample available -> auto switches to VIO.
    _publish_scout_odometry(observer, mission_node, vehicle_id="scout1", x_m=0.0, y_m=0.0)
    mission_node._advance_search_plan_for_vehicle("scout_1")
    assert mission_node._vehicle_position_sources["scout_1"] == "vio_odometry"
    assert mission_node._scout_plans["scout_1"].current_waypoint_index == 1


def test_auto_position_source_respects_gps_denied_vio_requirement(
    mission_harness_auto_gps_denied,
) -> None:
    mission_node, observer = mission_harness_auto_gps_denied
    del observer
    mission_node._state = "SEARCHING"
    mission_node._mission_id = "auto-gps-denied"
    mission_node._active_scout_vehicle_id = "scout_1"
    mission_node._scout_plans = {
        "scout_1": MissionPlanState(
            pattern_type="lawnmower",
            zone_id="zone-auto-gps-denied",
            waypoints=[
                {"x": 0.0, "z": 0.0, "altitude_m": 20.0},
                {"x": 10.0, "z": 0.0, "altitude_m": 20.0},
            ],
            current_waypoint_index=0,
            scout_position={"x": 0.0, "z": 0.0},
        )
    }

    mission_node._advance_search_plan_for_vehicle("scout_1")
    assert mission_node._vehicle_position_sources["scout_1"] == "vio_odometry:unavailable"
    assert mission_node._scout_plans["scout_1"].current_waypoint_index == 0


def test_start_mission_creates_distinct_scout_zone_assignments(mission_harness) -> None:
    mission_node, observer = mission_harness
    _publish_scout_telemetry(observer, mission_node, vehicle_id="scout1")
    _publish_scout_telemetry(observer, mission_node, vehicle_id="scout2")

    request = MissionCommand.Request()
    request.command = "START"
    request.mission_id = "multi-zone-assignments"
    request.zone_geometry = VALID_ZONE_GEOMETRY
    result_future = observer.start_client.call_async(request)

    assert _wait_until(lambda: result_future.done())
    response = result_future.result()
    assert response is not None
    assert response.success
    assert _wait_until(lambda: mission_node._state == "SEARCHING")
    assert len(mission_node._scout_plans) == 2
    assert mission_node._assignment_labels["scout_1"].startswith("SEARCHING:zone-")
    assert mission_node._assignment_labels["scout_2"].startswith("SEARCHING:zone-")
    assert mission_node._assignment_labels["scout_1"] != mission_node._assignment_labels["scout_2"]


def test_start_mission_falls_back_to_single_scout_plan(mission_harness) -> None:
    mission_node, observer = mission_harness
    _publish_scout_telemetry(observer, mission_node, vehicle_id="scout1")

    request = MissionCommand.Request()
    request.command = "START"
    request.mission_id = "single-scout-fallback"
    request.zone_geometry = VALID_ZONE_GEOMETRY
    result_future = observer.start_client.call_async(request)

    assert _wait_until(lambda: result_future.done())
    response = result_future.result()
    assert response is not None
    assert response.success
    assert _wait_until(lambda: mission_node._state == "SEARCHING")
    assert len(mission_node._scout_plans) == 1
    assert "scout_1" in mission_node._scout_plans


def test_completed_scout_is_reassigned_to_remaining_work(mission_harness) -> None:
    mission_node, observer = mission_harness
    del observer
    mission_node._state = "SEARCHING"
    mission_node._mission_id = "reassignment"
    mission_node._scout_plans = {
        "scout_1": MissionPlanState(
            pattern_type="lawnmower",
            zone_id="zone-1",
            waypoints=[
                {"x": 0.0, "z": 0.0, "altitude_m": 20.0},
                {"x": 1.0, "z": 0.0, "altitude_m": 20.0},
            ],
            current_waypoint_index=2,
            scout_position={"x": 1.0, "z": 0.0},
        ),
        "scout_2": MissionPlanState(
            pattern_type="lawnmower",
            zone_id="zone-2",
            waypoints=[
                {"x": 0.0, "z": 1.0, "altitude_m": 20.0},
                {"x": 1.0, "z": 1.0, "altitude_m": 20.0},
                {"x": 2.0, "z": 1.0, "altitude_m": 20.0},
                {"x": 3.0, "z": 1.0, "altitude_m": 20.0},
                {"x": 4.0, "z": 1.0, "altitude_m": 20.0},
            ],
            current_waypoint_index=0,
            scout_position={"x": 0.0, "z": 1.0},
        ),
    }
    mission_node._set_scout_assignment("scout_1", "IDLE", label="IDLE:zone-1:complete")
    mission_node._set_scout_assignment("scout_2", "SEARCHING", label="SEARCHING:zone-2")

    mission_node._redistribute_completed_scout_work()

    assert mission_node._vehicle_assignments.get("scout_1") == "SEARCHING"
    assert mission_node._assignment_labels.get("scout_1", "").startswith("SEARCHING:")
    assert len(mission_node._scout_plans["scout_1"].waypoints) >= 2
    assert len(mission_node._scout_plans["scout_2"].waypoints) < 5


def test_search_plan_completion_waits_for_final_waypoint_arrival(mission_harness) -> None:
    mission_node, observer = mission_harness
    del observer

    mission_node._state = "SEARCHING"
    mission_node._active_scout_vehicle_id = "scout_1"
    plan = MissionPlanState(
        pattern_type="lawnmower",
        zone_id="zone-1",
        waypoints=[
            {"x": 0.0, "z": 0.0, "altitude_m": 20.0},
            {"x": 10.0, "z": 0.0, "altitude_m": 20.0},
        ],
        current_waypoint_index=1,
        scout_position={"x": 0.0, "z": 0.0},
    )
    mission_node._scout_plans = {"scout_1": plan}

    assert not mission_node._is_search_plan_complete(plan)
    assert mission_node._search_plan_progress_percent(plan) == pytest.approx(50.0)

    plan.scout_position = {"x": 10.0, "z": 0.0}
    mission_node._advance_search_plan_for_vehicle("scout_1")

    assert mission_node._is_search_plan_complete(plan)
    assert plan.current_waypoint_index == len(plan.waypoints)
    assert mission_node._search_plan_progress_percent(plan) == pytest.approx(100.0)


def test_vio_waypoint_progression_uses_live_odometry(mission_harness_vio) -> None:
    mission_node, observer = mission_harness_vio
    mission_node._state = "SEARCHING"
    mission_node._mission_id = "vio-progress"
    mission_node._active_scout_vehicle_id = "scout_1"
    mission_node._scout_plans = {
        "scout_1": MissionPlanState(
            pattern_type="lawnmower",
            zone_id="zone-vio",
            waypoints=[
                {"x": 0.0, "z": 0.0, "altitude_m": 20.0},
                {"x": 5.0, "z": 0.0, "altitude_m": 20.0},
            ],
            current_waypoint_index=0,
            scout_position={"x": -2.0, "z": 0.0},
        )
    }

    _publish_scout_odometry(observer, mission_node, vehicle_id="scout1", x_m=0.0, y_m=0.0)
    mission_node._advance_search_plan_for_vehicle("scout_1")

    plan = mission_node._scout_plans["scout_1"]
    assert plan.current_waypoint_index == 1
    assert plan.scout_position == pytest.approx({"x": 0.0, "z": 0.0})
    assert mission_node._vehicle_position_sources["scout_1"] == "vio_odometry"


def test_vio_waypoint_progression_halts_on_stale_odometry(mission_harness_vio) -> None:
    mission_node, observer = mission_harness_vio
    mission_node._state = "SEARCHING"
    mission_node._mission_id = "vio-stale"
    mission_node._active_scout_vehicle_id = "scout_1"
    mission_node._scout_plans = {
        "scout_1": MissionPlanState(
            pattern_type="lawnmower",
            zone_id="zone-vio",
            waypoints=[
                {"x": 0.0, "z": 0.0, "altitude_m": 20.0},
                {"x": 10.0, "z": 0.0, "altitude_m": 20.0},
            ],
            current_waypoint_index=1,
            scout_position={"x": 0.0, "z": 0.0},
        )
    }

    _publish_scout_odometry(observer, mission_node, vehicle_id="scout1", x_m=0.0, y_m=0.0)
    time.sleep(0.15)
    mission_node._advance_search_plan_for_vehicle("scout_1")

    plan = mission_node._scout_plans["scout_1"]
    assert plan.current_waypoint_index == 1
    assert mission_node._vehicle_position_sources["scout_1"] == "vio_odometry:unavailable"
    assert "stale" in mission_node._scout_vio_status.get("scout_1", "")


def test_vio_breadcrumb_capture_records_state_and_timestamp(mission_harness_vio) -> None:
    mission_node, observer = mission_harness_vio
    mission_node._state = "SEARCHING"
    mission_node._mission_id = "vio-breadcrumb-metadata"

    _publish_scout_odometry(observer, mission_node, vehicle_id="scout1", x_m=3.25, y_m=-1.5)

    breadcrumbs = mission_node._scout_vio_breadcrumbs.get("scout_1", [])
    assert len(breadcrumbs) == 1
    breadcrumb = breadcrumbs[0]
    assert breadcrumb["x"] == pytest.approx(3.25)
    assert breadcrumb["z"] == pytest.approx(-1.5)
    assert breadcrumb["missionState"] == "SEARCHING"
    assert breadcrumb["missionId"] == "vio-breadcrumb-metadata"
    assert isinstance(breadcrumb["timestampSec"], float)
    assert breadcrumb["timestampSec"] > 0.0


def test_vio_breadcrumb_capture_prunes_and_downsamples_deterministically(
    ros_runtime: MultiThreadedExecutor,
) -> None:
    mission_node = MissionNode(
        parameter_overrides=[
            Parameter("navigation_position_source", value="vio_odometry"),
            Parameter("return_breadcrumb_limit_per_vehicle", value=3),
            Parameter("return_breadcrumb_min_spacing_m", value=2.0),
        ]
    )
    observer = MissionObserver()
    ros_runtime.add_node(mission_node)
    ros_runtime.add_node(observer)
    try:
        mission_node._state = "SEARCHING"
        mission_node._mission_id = "vio-breadcrumb-bounds"

        _inject_scout_odometry(mission_node, observer, "scout1", x_m=0.0, y_m=0.0)
        _inject_scout_odometry(mission_node, observer, "scout1", x_m=0.6, y_m=0.2)
        _inject_scout_odometry(mission_node, observer, "scout1", x_m=3.0, y_m=0.0)
        _inject_scout_odometry(mission_node, observer, "scout1", x_m=6.0, y_m=0.0)
        _inject_scout_odometry(mission_node, observer, "scout1", x_m=9.0, y_m=0.0)

        breadcrumbs = mission_node._scout_vio_breadcrumbs.get("scout_1", [])
        assert len(breadcrumbs) == 3
        assert [round(entry["x"], 3) for entry in breadcrumbs] == [3.0, 6.0, 9.0]
    finally:
        ros_runtime.remove_node(observer)
        ros_runtime.remove_node(mission_node)
        observer.destroy_node()
        mission_node.destroy_node()


def test_vio_odometry_topic_map_accepts_per_scout_override(
    ros_runtime: MultiThreadedExecutor,
) -> None:
    mission_node = MissionNode(
        parameter_overrides=[
            Parameter("navigation_position_source", value="vio_odometry"),
            Parameter(
                "scout_odometry_topics_json",
                value='{"scout1":"/custom/scout1/odom","scout2":"/custom/scout2/odom"}',
            ),
        ]
    )
    try:
        ros_runtime.add_node(mission_node)
        assert mission_node._scout_odometry_topics["scout_1"] == "/custom/scout1/odom"
        assert mission_node._scout_odometry_topics["scout_2"] == "/custom/scout2/odom"
    finally:
        ros_runtime.remove_node(mission_node)
        mission_node.destroy_node()


def test_vio_recall_synthesizes_return_path_and_progress_payload(
    mission_harness_vio_return, monkeypatch
) -> None:
    mission_node, observer = mission_harness_vio_return
    mission_node._state = "SEARCHING"
    mission_node._mission_id = "vio-recall-success"
    mission_node._active_scout_vehicle_id = "scout_1"
    mission_node._scout_plans = {
        "scout_1": MissionPlanState(
            pattern_type="lawnmower",
            zone_id="zone-return",
            waypoints=[{"x": 100.0, "z": 0.0, "altitude_m": 20.0}],
            current_waypoint_index=0,
            scout_position={"x": 0.0, "z": 0.0},
        )
    }
    mission_node._set_scout_assignment("scout_1", "SEARCHING", label="SEARCHING:zone-return")
    _publish_scout_telemetry(observer, mission_node, vehicle_id="scout1")
    _publish_occupancy_map(observer, mission_node)
    _inject_scout_odometry(mission_node, observer, "scout1", x_m=0.0, y_m=0.0)
    _inject_scout_odometry(mission_node, observer, "scout1", x_m=3.0, y_m=0.0)
    _inject_scout_odometry(mission_node, observer, "scout1", x_m=6.0, y_m=0.0)

    uploaded: list[int] = []
    started: list[dict[str, float]] = []
    monkeypatch.setattr(
        mission_node._mavlink_adapter,
        "upload_mission_items_int",
        lambda mission_id, waypoints: uploaded.append(len(waypoints)),
    )
    monkeypatch.setattr(
        mission_node._mavlink_adapter,
        "start_stream",
        lambda mission_id, setpoint: started.append(dict(setpoint)),
    )

    request = SimpleNamespace(
        command="RECALL",
        vehicle_id="scout1",
        mission_id="vio-recall-success",
    )
    response = SimpleNamespace(success=False, message="")
    result = mission_node._handle_vehicle_command(request, response)

    assert result.success
    assert mission_node._vehicle_assignments["scout_1"] == "RETURNING"
    assert mission_node._return_paths_by_vehicle["scout_1"]
    assert not mission_node._return_fallback_reasons.get("scout_1")
    assert uploaded and uploaded[-1] >= 2
    assert started

    captured: list[str] = []
    monkeypatch.setattr(
        mission_node._progress_string_pub, "publish", lambda message: captured.append(message.data)
    )
    monkeypatch.setattr(mission_node._progress_pub, "publish", lambda _: None)
    mission_node._publish_progress_if_active()
    payload = json.loads(captured[-1])
    trajectory = payload["returnTrajectory"]
    assert trajectory["vehicleId"] == "scout_1"
    assert trajectory["state"] == "RETURNING"
    assert len(trajectory["points"]) >= 2
    assert "fallbackReason" not in trajectory


def test_vio_recall_falls_back_to_px4_native_rtl_when_map_is_stale(
    mission_harness_vio_return, monkeypatch
) -> None:
    mission_node, observer = mission_harness_vio_return
    mission_node._state = "SEARCHING"
    mission_node._mission_id = "vio-recall-fallback"
    mission_node._active_scout_vehicle_id = "scout_1"
    mission_node._scout_plans = {
        "scout_1": MissionPlanState(
            pattern_type="lawnmower",
            zone_id="zone-return",
            waypoints=[{"x": 100.0, "z": 0.0, "altitude_m": 20.0}],
            current_waypoint_index=0,
            scout_position={"x": 0.0, "z": 0.0},
        )
    }
    mission_node._set_scout_assignment("scout_1", "SEARCHING", label="SEARCHING:zone-return")
    _publish_scout_telemetry(observer, mission_node, vehicle_id="scout1")
    _inject_scout_odometry(mission_node, observer, "scout1", x_m=0.0, y_m=0.0)
    _inject_scout_odometry(mission_node, observer, "scout1", x_m=2.0, y_m=0.0)
    monkeypatch.setattr(mission_node._mavlink_adapter, "send_return_to_launch", lambda: True)

    request = SimpleNamespace(
        command="RECALL",
        vehicle_id="scout1",
        mission_id="vio-recall-fallback",
    )
    response = SimpleNamespace(success=False, message="")
    result = mission_node._handle_vehicle_command(request, response)

    assert result.success
    assert "scout_1" in mission_node._return_fallback_reasons
    assert not mission_node._return_paths_by_vehicle.get("scout_1")
    assert mission_node._vehicle_assignments["scout_1"] == "RETURNING"
    assert mission_node._assignment_labels["scout_1"] == "RETURNING:fallback:px4-native-rtl"

    captured: list[str] = []
    monkeypatch.setattr(
        mission_node._progress_string_pub, "publish", lambda message: captured.append(message.data)
    )
    monkeypatch.setattr(mission_node._progress_pub, "publish", lambda _: None)
    mission_node._publish_progress_if_active()
    payload = json.loads(captured[-1])
    trajectory = payload["returnTrajectory"]
    assert trajectory["state"] == "FALLBACK"
    assert isinstance(trajectory.get("fallbackReason"), str)
    assert trajectory["fallbackReason"]


def test_vio_return_waypoint_builder_filters_stale_and_off_mission_breadcrumbs(
    mission_harness_vio_return,
) -> None:
    mission_node, observer = mission_harness_vio_return
    mission_node._state = "SEARCHING"
    mission_node._mission_id = "vio-breadcrumb-filtering"
    mission_node._mission_started_sec = mission_node._now_seconds() - 30.0
    mission_node._active_scout_vehicle_id = "scout_1"

    _publish_occupancy_map(observer, mission_node)
    _inject_scout_odometry(mission_node, observer, "scout1", x_m=8.0, y_m=0.0)

    now_sec = mission_node._now_seconds()
    mission_node._scout_vio_breadcrumbs["scout_1"] = [
        {
            "x": 99.0,
            "z": 99.0,
            "timestampSec": now_sec - 4.0,
            "missionState": "IDLE",
            "missionId": "vio-breadcrumb-filtering",
        },
        {
            "x": 1.0,
            "z": 0.0,
            "timestampSec": now_sec - (mission_node._return_breadcrumb_max_age_sec + 1.0),
            "missionState": "SEARCHING",
            "missionId": "vio-breadcrumb-filtering",
        },
        {
            "x": 0.0,
            "z": 0.0,
            "timestampSec": now_sec - 10.0,
            "missionState": "SEARCHING",
            "missionId": "vio-breadcrumb-filtering",
        },
        {
            "x": 4.0,
            "z": 0.0,
            "timestampSec": now_sec - 6.0,
            "missionState": "TRACKING",
            "missionId": "vio-breadcrumb-filtering",
        },
        {
            "x": 7.0,
            "z": 0.0,
            "timestampSec": now_sec - 3.0,
            "missionState": "SEARCHING",
            "missionId": "other-mission",
        },
    ]

    waypoints, error = mission_node._build_vio_return_waypoints("scout_1")
    assert not error
    assert waypoints is not None
    assert [round(float(point["x"]), 2) for point in waypoints] == [8.0, 4.0, 0.0]


def test_vio_return_execution_falls_back_when_map_becomes_stale_mid_return(
    mission_harness_vio_return, monkeypatch
) -> None:
    mission_node, observer = mission_harness_vio_return
    mission_node._state = "SEARCHING"
    mission_node._mission_id = "vio-map-stale-mid-return"
    mission_node._active_scout_vehicle_id = "scout_1"
    mission_node._scout_plans = {
        "scout_1": MissionPlanState(
            pattern_type="lawnmower",
            zone_id="zone-stale-mid-return",
            waypoints=[{"x": 100.0, "z": 0.0, "altitude_m": 20.0}],
            current_waypoint_index=0,
            scout_position={"x": 0.0, "z": 0.0},
        )
    }
    mission_node._set_scout_assignment(
        "scout_1", "SEARCHING", label="SEARCHING:zone-stale-mid-return"
    )
    _publish_scout_telemetry(observer, mission_node, vehicle_id="scout1")
    _publish_occupancy_map(observer, mission_node)
    _inject_scout_odometry(mission_node, observer, "scout1", x_m=0.0, y_m=0.0)
    _inject_scout_odometry(mission_node, observer, "scout1", x_m=3.0, y_m=0.0)
    _inject_scout_odometry(mission_node, observer, "scout1", x_m=6.0, y_m=0.0)

    monkeypatch.setattr(
        mission_node._mavlink_adapter, "upload_mission_items_int", lambda mission_id, waypoints: None
    )
    monkeypatch.setattr(
        mission_node._mavlink_adapter, "start_stream", lambda mission_id, setpoint: None
    )
    monkeypatch.setattr(mission_node._mavlink_adapter, "send_return_to_launch", lambda: True)

    request = SimpleNamespace(
        command="RECALL",
        vehicle_id="scout1",
        mission_id="vio-map-stale-mid-return",
    )
    response = SimpleNamespace(success=False, message="")
    result = mission_node._handle_vehicle_command(request, response)
    assert result.success
    assert mission_node._return_paths_by_vehicle.get("scout_1")

    mission_node._occupancy_map_received_monotonic = time.monotonic() - (
        mission_node._return_required_freshness_sec + 0.5
    )
    mission_node._advance_return_plan_for_vehicle("scout_1")

    assert "occupancy map became stale during return execution" in mission_node._return_fallback_reasons.get(
        "scout_1", ""
    )
    assert not mission_node._return_paths_by_vehicle.get("scout_1")
    assert mission_node._assignment_labels["scout_1"] == "RETURNING:fallback:px4-native-rtl"


def test_enforce_mixed_returning_separation_applies_waypoint_mitigation(
    mission_harness_vio_return, monkeypatch
) -> None:
    mission_node, observer = mission_harness_vio_return
    mission_node._state = "SEARCHING"
    mission_node._mission_id = "mixed-returning-separation"
    mission_node._active_scout_vehicle_id = "scout_1"
    mission_node._return_separation_horizontal_m = 5.0
    mission_node._return_separation_vertical_m = 3.0
    mission_node._return_altitude_offset_m = 0.0
    mission_node._mavlink_adapter._running = True

    _publish_occupancy_map(observer, mission_node)

    mission_node._scout_plans = {
        "scout_1": MissionPlanState(
            pattern_type="lawnmower",
            zone_id="zone-returning",
            waypoints=[{"x": 0.0, "z": 0.0, "altitude_m": 20.0}],
            current_waypoint_index=0,
            scout_position={"x": 0.0, "z": 0.0},
        ),
        "scout_2": MissionPlanState(
            pattern_type="lawnmower",
            zone_id="zone-searching",
            waypoints=[{"x": 1.0, "z": 1.0, "altitude_m": 20.0}],
            current_waypoint_index=0,
            scout_position={"x": 1.0, "z": 1.0},
        ),
    }
    mission_node._scout_position_snapshot = {
        "scout_1": {"x": 0.0, "z": 0.0},
        "scout_2": {"x": 1.0, "z": 1.0},
    }
    mission_node._return_paths_by_vehicle["scout_1"] = [
        {"x": 0.0, "z": 0.0, "altitude_m": 20.0},
        {"x": -10.0, "z": 0.0, "altitude_m": 20.0},
    ]
    mission_node._return_path_index_by_vehicle["scout_1"] = 0
    mission_node._set_scout_assignment("scout_1", "RETURNING", label="RETURNING:scout_1")
    mission_node._set_scout_assignment("scout_2", "SEARCHING", label="SEARCHING:zone-searching")

    updates: list[dict[str, float]] = []
    monkeypatch.setattr(
        mission_node._mavlink_adapter, "update_setpoint", lambda setpoint: updates.append(dict(setpoint))
    )

    mission_node._enforce_mixed_returning_separation()

    waypoints = mission_node._return_paths_by_vehicle["scout_1"]
    assert len(waypoints) == 3
    mitigation = waypoints[0]
    assert (
        math.hypot(mitigation["x"] - 1.0, mitigation["z"] - 1.0)
        >= mission_node._return_separation_horizontal_m
    )
    assert mitigation["altitude_m"] >= 23.5
    assert updates


def test_vio_recall_preserves_non_target_search_assignments(
    mission_harness_vio_return, monkeypatch
) -> None:
    mission_node, observer = mission_harness_vio_return
    mission_node._state = "SEARCHING"
    mission_node._mission_id = "vio-recall-mixed-fleet"
    mission_node._active_scout_vehicle_id = "scout_1"
    mission_node._scout_plans = {
        "scout_1": MissionPlanState(
            pattern_type="lawnmower",
            zone_id="zone-1",
            waypoints=[{"x": 100.0, "z": 0.0, "altitude_m": 20.0}],
            current_waypoint_index=0,
            scout_position={"x": 0.0, "z": 0.0},
        ),
        "scout_2": MissionPlanState(
            pattern_type="lawnmower",
            zone_id="zone-2",
            waypoints=[{"x": 110.0, "z": 10.0, "altitude_m": 20.0}],
            current_waypoint_index=0,
            scout_position={"x": 10.0, "z": 10.0},
        ),
    }
    mission_node._set_scout_assignment("scout_1", "SEARCHING", label="SEARCHING:zone-1")
    mission_node._set_scout_assignment("scout_2", "SEARCHING", label="SEARCHING:zone-2")
    _publish_scout_telemetry(observer, mission_node, vehicle_id="scout1")
    _publish_scout_telemetry(observer, mission_node, vehicle_id="scout2")
    _publish_occupancy_map(observer, mission_node)
    _inject_scout_odometry(mission_node, observer, "scout2", x_m=7.0, y_m=7.0)
    _inject_scout_odometry(mission_node, observer, "scout2", x_m=5.0, y_m=5.0)
    monkeypatch.setattr(
        mission_node._mavlink_adapter, "upload_mission_items_int", lambda mission_id, waypoints: None
    )
    monkeypatch.setattr(
        mission_node._mavlink_adapter, "start_stream", lambda mission_id, setpoint: None
    )

    request = SimpleNamespace(
        command="RECALL",
        vehicle_id="scout2",
        mission_id="vio-recall-mixed-fleet",
    )
    response = SimpleNamespace(success=False, message="")
    result = mission_node._handle_vehicle_command(request, response)

    assert result.success
    assert mission_node._vehicle_assignments["scout_2"] == "RETURNING"
    assert mission_node._vehicle_assignments["scout_1"] == "SEARCHING"


def test_start_mission_enables_ranger_overwatch_from_role_data(
    mission_harness_multi_vehicle,
) -> None:
    mission_node, observer = mission_harness_multi_vehicle
    _publish_scout_telemetry(observer, mission_node, vehicle_id="scout1")
    _publish_scout_telemetry(observer, mission_node, vehicle_id="scout2")
    _publish_ranger_telemetry(observer, mission_node, vehicle_id="ranger1")

    request = MissionCommand.Request()
    request.command = "START"
    request.mission_id = "ranger-overwatch"
    request.zone_geometry = VALID_ZONE_GEOMETRY
    result_future = observer.start_client.call_async(request)

    assert _wait_until(lambda: result_future.done())
    response = result_future.result()
    assert response is not None
    assert response.success
    assert _wait_until(lambda: mission_node._state == "SEARCHING")
    assert mission_node._active_ranger_vehicle_id == "ranger_1"
    assert mission_node._vehicle_assignments.get("ranger_1") == "OVERWATCH"
    assert len(mission_node._ranger_orbit_waypoints) >= 4


def test_start_mission_dispatches_ranger_overwatch_stream_when_endpoint_is_configured(
    mission_harness_multi_vehicle, monkeypatch
) -> None:
    mission_node, observer = mission_harness_multi_vehicle
    _publish_scout_telemetry(observer, mission_node, vehicle_id="scout1")
    _publish_scout_telemetry(observer, mission_node, vehicle_id="scout2")
    _publish_ranger_telemetry(observer, mission_node, vehicle_id="ranger1")

    calls: dict[str, list] = {"upload": [], "start": [], "update": [], "close": []}

    class _FakeRangerAdapter:
        def __init__(self) -> None:
            self.is_streaming = False

        def upload_mission_items_int(
            self, mission_id: str, waypoints: list[dict[str, float]]
        ) -> None:
            calls["upload"].append((mission_id, len(waypoints)))

        def start_stream(self, mission_id: str, initial_setpoint: dict[str, float]) -> None:
            self.is_streaming = True
            calls["start"].append((mission_id, dict(initial_setpoint)))

        def update_setpoint(self, setpoint: dict[str, float]) -> None:
            calls["update"].append(dict(setpoint))

        def close(self) -> None:
            self.is_streaming = False
            calls["close"].append(True)

    fake_adapter = _FakeRangerAdapter()
    monkeypatch.setattr(
        MissionNode,
        "_build_ranger_adapter",
        lambda self, endpoint: fake_adapter,
    )

    request = MissionCommand.Request()
    request.command = "START"
    request.mission_id = "ranger-overwatch-dispatch"
    request.zone_geometry = VALID_ZONE_GEOMETRY
    result_future = observer.start_client.call_async(request)

    assert _wait_until(lambda: result_future.done())
    response = result_future.result()
    assert response is not None
    assert response.success
    assert _wait_until(lambda: mission_node._state == "SEARCHING")
    assert _wait_until(lambda: bool(calls["upload"]))
    assert _wait_until(lambda: bool(calls["start"]))
    assert calls["start"][-1][0] == "ranger-overwatch-dispatch"
    assert calls["start"][-1][1]["altitude_m"] == pytest.approx(
        mission_node._ranger_overwatch_altitude_m
    )


def test_sync_assignments_reassigns_active_ranger_when_current_goes_offline(
    mission_harness, monkeypatch
) -> None:
    mission_node, observer = mission_harness
    del observer

    mission_node._mission_id = "ranger-failover"
    mission_node._state = "SEARCHING"
    mission_node._active_ranger_vehicle_id = "ranger_1"
    mission_node._ranger_endpoints = [
        ScoutEndpoint(vehicle_id="ranger_1", host="127.0.0.1", port=14543),
        ScoutEndpoint(vehicle_id="ranger_2", host="127.0.0.1", port=14544),
    ]
    mission_node._ranger_orbit_waypoints = [{"x": 0.0, "z": 0.0, "altitude_m": 20.0}]
    mission_node._set_scout_assignment("ranger_1", "OVERWATCH", label="OVERWATCH")
    mission_node._set_scout_assignment("ranger_2", "IDLE", label="IDLE")
    now = time.monotonic()
    mission_node._ranger_last_seen_monotonic = {
        "ranger_1": now - 10.0,
        "ranger_2": now,
    }

    dispatch_calls: list[bool] = []
    monkeypatch.setattr(
        mission_node,
        "_start_ranger_overwatch_execution",
        lambda: dispatch_calls.append(True),
    )

    mission_node._sync_assignments_with_telemetry_freshness()

    assert mission_node._active_ranger_vehicle_id == "ranger_2"
    assert mission_node._vehicle_assignments["ranger_1"] == "IDLE"
    assert mission_node._assignment_labels["ranger_1"] == "IDLE:offline"
    assert mission_node._vehicle_assignments["ranger_2"] == "OVERWATCH"
    assert dispatch_calls == [True]


def test_sync_assignments_preserves_completed_idle_label_for_online_scout(
    mission_harness,
) -> None:
    mission_node, observer = mission_harness
    del observer

    mission_node._mission_id = "sync-complete-label"
    mission_node._scout_plans = {
        "scout_1": MissionPlanState(
            pattern_type="lawnmower",
            zone_id="zone-1",
            waypoints=[
                {"x": 0.0, "z": 0.0, "altitude_m": 20.0},
                {"x": 1.0, "z": 0.0, "altitude_m": 20.0},
            ],
            current_waypoint_index=2,
            scout_position={"x": 1.0, "z": 0.0},
        )
    }
    mission_node._set_scout_assignment("scout_1", "IDLE", label="IDLE:zone-1:complete")
    mission_node._scout_last_seen_monotonic["scout_1"] = time.monotonic()

    mission_node._sync_assignments_with_telemetry_freshness()

    assert mission_node._vehicle_assignments["scout_1"] == "IDLE"
    assert mission_node._assignment_labels["scout_1"] == "IDLE:zone-1:complete"


def test_progress_payload_includes_vehicle_assignment_labels_and_progress(
    mission_harness, monkeypatch
) -> None:
    mission_node, observer = mission_harness
    del observer

    mission_node._state = "SEARCHING"
    mission_node._mission_id = "progress-payload"
    mission_node._active_scout_vehicle_id = "scout_1"
    mission_node._scout_plans = {
        "scout_1": MissionPlanState(
            pattern_type="lawnmower",
            zone_id="zone-1",
            waypoints=[
                {"x": 0.0, "z": 0.0, "altitude_m": 20.0},
                {"x": 1.0, "z": 0.0, "altitude_m": 20.0},
            ],
            current_waypoint_index=2,
            scout_position={"x": 1.0, "z": 0.0},
        )
    }
    mission_node._set_scout_assignment("scout_1", "SEARCHING", label="SEARCHING:zone-1")
    mission_node._scout_last_seen_monotonic["scout_1"] = time.monotonic()

    captured: list[str] = []

    def _capture_publish(message: String) -> None:
        captured.append(message.data)

    monkeypatch.setattr(mission_node._progress_string_pub, "publish", _capture_publish)
    monkeypatch.setattr(mission_node._progress_pub, "publish", lambda _: None)

    mission_node._publish_progress_if_active()

    assert captured
    payload = json.loads(captured[-1])
    assert payload["vehicleAssignments"]["scout_1"] == "SEARCHING"
    assert payload["vehicleAssignmentLabels"]["scout_1"] == "SEARCHING:zone-1"
    assert payload["vehicleProgress"]["scout_1"] == pytest.approx(100.0, abs=1e-3)
    assert payload["positionSourceMode"] == mission_node._navigation_position_source
    assert "scout_1" in payload["vehiclePositionSources"]
    assert payload["vehicleSlamModes"]["scout_1"] == "vio"
    assert payload["vehicleSlamModes"]["scout_2"] == "vio"
    assert "vehicleOnline" in payload


def test_normalize_slam_mode_maps_legacy_aliases_and_logs_warning() -> None:
    warnings: list[str] = []
    mission_node = MissionNode.__new__(MissionNode)
    mission_node.get_logger = lambda: SimpleNamespace(
        warning=lambda message: warnings.append(str(message))
    )

    assert mission_node._normalize_slam_mode("rtabmap_vio") == "vio"
    assert mission_node._normalize_slam_mode("rtabmap") == "vio"
    assert mission_node._normalize_slam_mode("lio_sam") == "liosam"
    assert mission_node._normalize_slam_mode("LIO-SAM") == "liosam"
    assert len(warnings) == 4
    assert "Legacy slam_mode" in warnings[0]


def test_normalize_slam_mode_falls_back_to_unknown_and_logs_warning() -> None:
    warnings: list[str] = []
    mission_node = MissionNode.__new__(MissionNode)
    mission_node.get_logger = lambda: SimpleNamespace(
        warning=lambda message: warnings.append(str(message))
    )

    assert mission_node._normalize_slam_mode("unrecognized_mode") == "unknown"
    assert warnings
    assert "Unsupported slam_mode 'unrecognized_mode'" in warnings[-1]


def test_progress_payload_reports_unknown_slam_mode_when_mode_is_invalid(
    mission_harness, monkeypatch
) -> None:
    mission_node, observer = mission_harness
    del observer

    mission_node._state = "SEARCHING"
    mission_node._mission_id = "progress-payload-unknown-slam"
    mission_node._slam_mode = "unknown"
    mission_node._scout_last_seen_monotonic["scout_1"] = time.monotonic()

    captured: list[str] = []

    def _capture_publish(message: String) -> None:
        captured.append(message.data)

    monkeypatch.setattr(mission_node._progress_string_pub, "publish", _capture_publish)
    monkeypatch.setattr(mission_node._progress_pub, "publish", lambda _: None)

    mission_node._publish_progress_if_active()

    assert captured
    payload = json.loads(captured[-1])
    assert payload["vehicleSlamModes"]["scout_1"] == "unknown"
    assert payload["vehicleSlamModes"]["scout_2"] == "unknown"


def test_vehicle_slam_mode_snapshot_uses_vehicle_specific_slam_mode_overrides() -> None:
    mission_node = MissionNode.__new__(MissionNode)
    mission_node._normalize_vehicle_id = lambda value: str(value).strip().lower()
    mission_node._slam_mode = "vio"
    mission_node._vehicle_slam_mode_overrides = {
        "scout_1": "vio",
        "scout_2": "liosam",
        "ranger_1": "unknown",
    }
    mission_node._scout_endpoints = [
        ScoutEndpoint(vehicle_id="scout_1", host="127.0.0.1", port=14541),
        ScoutEndpoint(vehicle_id="scout_2", host="127.0.0.1", port=14542),
    ]
    mission_node._scout_plans = {}
    mission_node._active_scout_vehicle_id = ""
    mission_node._ranger_endpoints = [
        ScoutEndpoint(vehicle_id="ranger_1", host="127.0.0.1", port=14543),
    ]
    mission_node._active_ranger_vehicle_id = ""

    snapshot = mission_node._vehicle_slam_mode_snapshot()
    assert snapshot["scout_1"] == "vio"
    assert snapshot["scout_2"] == "liosam"
    assert snapshot["ranger_1"] == "unknown"


def test_vehicle_command_targets_only_requested_endpoint(
    mission_harness, monkeypatch
) -> None:
    mission_node, observer = mission_harness
    del observer

    now = time.monotonic()
    mission_node._state = "SEARCHING"
    mission_node._mission_id = "vehicle-command-mission"
    mission_node._scout_last_seen_monotonic = {
        "scout_1": now,
        "scout_2": now,
    }

    endpoint_calls: list[tuple[str, int]] = []
    hold_calls: list[int] = []

    def _record_endpoint(
        host: str, port: int, command_port: int | None = None
    ) -> None:
        del command_port
        endpoint_calls.append((host, int(port)))

    def _record_hold() -> bool:
        hold_calls.append(1)
        return True

    monkeypatch.setattr(mission_node._mavlink_adapter, "set_endpoint", _record_endpoint)
    monkeypatch.setattr(mission_node._mavlink_adapter, "send_hold_position", _record_hold)

    request = SimpleNamespace(
        command="HOLD",
        vehicle_id="scout2",
        mission_id="vehicle-command-mission",
    )
    response = SimpleNamespace(success=False, message="")
    result = mission_node._handle_vehicle_command(request, response)

    assert result.success
    assert hold_calls == [1]
    assert endpoint_calls == [("127.0.0.1", 14542), ("127.0.0.1", 14540)]
    assert mission_node._state == "SEARCHING"


def test_vehicle_command_rejects_unknown_vehicle(mission_harness) -> None:
    mission_node, observer = mission_harness
    del observer

    mission_node._state = "SEARCHING"
    mission_node._mission_id = "reject-unknown-vehicle"
    mission_node._scout_last_seen_monotonic = {}

    request = SimpleNamespace(
        command="HOLD",
        vehicle_id="unknown7",
        mission_id="reject-unknown-vehicle",
    )
    response = SimpleNamespace(success=False, message="")
    result = mission_node._handle_vehicle_command(request, response)

    assert not result.success
    assert "unknown vehicle_id" in result.message


def test_vehicle_command_rejects_offline_vehicle(mission_harness) -> None:
    mission_node, observer = mission_harness
    del observer

    mission_node._state = "SEARCHING"
    mission_node._mission_id = "reject-offline-vehicle"
    mission_node._scout_last_seen_monotonic = {
        "scout_2": time.monotonic() - 10.0,
    }

    request = SimpleNamespace(
        command="HOLD",
        vehicle_id="scout2",
        mission_id="reject-offline-vehicle",
    )
    response = SimpleNamespace(success=False, message="")
    result = mission_node._handle_vehicle_command(request, response)

    assert not result.success
    assert "offline vehicle_id" in result.message


def test_vehicle_command_rejects_unsupported_command(mission_harness) -> None:
    mission_node, observer = mission_harness
    del observer

    mission_node._state = "SEARCHING"
    mission_node._mission_id = "reject-command"
    mission_node._scout_last_seen_monotonic = {"scout_1": time.monotonic()}

    request = SimpleNamespace(
        command="LAND",
        vehicle_id="scout1",
        mission_id="reject-command",
    )
    response = SimpleNamespace(success=False, message="")
    result = mission_node._handle_vehicle_command(request, response)

    assert not result.success
    assert "unsupported command" in result.message


def test_vehicle_command_rejects_invalid_state_transition(mission_harness) -> None:
    mission_node, observer = mission_harness
    del observer

    mission_node._state = "IDLE"
    mission_node._mission_id = "state-gate"
    mission_node._scout_last_seen_monotonic = {"scout_1": time.monotonic()}

    request = SimpleNamespace(
        command="HOLD",
        vehicle_id="scout1",
        mission_id="state-gate",
    )
    response = SimpleNamespace(success=False, message="")
    result = mission_node._handle_vehicle_command(request, response)

    assert not result.success
    assert "rejected while in IDLE" in result.message


def test_vehicle_command_rejects_invalid_per_vehicle_transition(
    mission_harness, monkeypatch
) -> None:
    mission_node, observer = mission_harness
    del observer

    mission_node._state = "SEARCHING"
    mission_node._mission_id = "transition-gate"
    mission_node._scout_last_seen_monotonic = {"scout_1": time.monotonic()}

    endpoint_calls: list[tuple[str, int]] = []
    hold_calls: list[int] = []

    def _record_endpoint(
        host: str, port: int, command_port: int | None = None
    ) -> None:
        del command_port
        endpoint_calls.append((host, int(port)))

    def _record_hold() -> bool:
        hold_calls.append(1)
        return True

    monkeypatch.setattr(mission_node._mavlink_adapter, "set_endpoint", _record_endpoint)
    monkeypatch.setattr(mission_node._mavlink_adapter, "send_hold_position", _record_hold)

    request = SimpleNamespace(
        command="HOLD",
        vehicle_id="scout1",
        mission_id="transition-gate",
    )
    response = SimpleNamespace(success=False, message="")
    first_result = mission_node._handle_vehicle_command(request, response)
    assert first_result.success

    second_response = SimpleNamespace(success=False, message="")
    second_result = mission_node._handle_vehicle_command(request, second_response)
    assert not second_result.success
    assert "invalid transition" in second_result.message
    assert hold_calls == [1]
    assert endpoint_calls == [("127.0.0.1", 14541), ("127.0.0.1", 14540)]


def test_vehicle_command_service_accepts_online_target(mission_harness, monkeypatch) -> None:
    mission_node, observer = mission_harness
    _publish_scout_telemetry(observer, mission_node, vehicle_id="scout1")
    _publish_scout_telemetry(observer, mission_node, vehicle_id="scout2")

    start_request = MissionCommand.Request()
    start_request.command = "START"
    start_request.mission_id = "vehicle-service-accept"
    start_request.zone_geometry = VALID_ZONE_GEOMETRY
    start_future = observer.start_client.call_async(start_request)

    assert _wait_until(lambda: start_future.done())
    assert start_future.result() is not None
    assert start_future.result().success
    assert _wait_until(lambda: "SEARCHING" in observer.states)

    endpoint_calls: list[tuple[str, int]] = []
    hold_calls: list[int] = []

    def _record_endpoint(
        host: str, port: int, command_port: int | None = None
    ) -> None:
        del command_port
        endpoint_calls.append((host, int(port)))

    def _record_hold() -> bool:
        hold_calls.append(1)
        return True

    monkeypatch.setattr(mission_node._mavlink_adapter, "set_endpoint", _record_endpoint)
    monkeypatch.setattr(mission_node._mavlink_adapter, "send_hold_position", _record_hold)

    request = VehicleCommand.Request()
    request.command = "HOLD"
    request.vehicle_id = "scout2"
    request.mission_id = "vehicle-service-accept"
    result_future = observer.vehicle_command_client.call_async(request)

    assert _wait_until(lambda: result_future.done())
    response = result_future.result()
    assert response is not None
    assert response.success
    assert "accepted" in response.message
    assert endpoint_calls == [("127.0.0.1", 14542), ("127.0.0.1", 14541)]
    assert hold_calls == [1]
    assert mission_node._state == "SEARCHING"


def test_vehicle_command_service_rejects_unknown_target(mission_harness) -> None:
    mission_node, observer = mission_harness
    _publish_scout_telemetry(observer, mission_node, vehicle_id="scout1")

    start_request = MissionCommand.Request()
    start_request.command = "START"
    start_request.mission_id = "vehicle-service-reject"
    start_request.zone_geometry = VALID_ZONE_GEOMETRY
    start_future = observer.start_client.call_async(start_request)

    assert _wait_until(lambda: start_future.done())
    assert start_future.result() is not None
    assert start_future.result().success
    assert _wait_until(lambda: "SEARCHING" in observer.states)

    request = VehicleCommand.Request()
    request.command = "HOLD"
    request.vehicle_id = "unknown99"
    request.mission_id = "vehicle-service-reject"
    result_future = observer.vehicle_command_client.call_async(request)

    assert _wait_until(lambda: result_future.done())
    response = result_future.result()
    assert response is not None
    assert not response.success
    assert "unknown vehicle_id" in response.message


def test_vehicle_command_applies_endpoint_command_port_and_target_system(
    mission_harness, monkeypatch
) -> None:
    mission_node, observer = mission_harness
    del observer

    now = time.monotonic()
    mission_node._state = "SEARCHING"
    mission_node._mission_id = "vehicle-command-target-system"
    mission_node._active_scout_vehicle_id = "scout_1"
    mission_node._scout_endpoints = [
        ScoutEndpoint(
            vehicle_id="scout_1",
            host="127.0.0.1",
            port=14541,
            command_port=14581,
            target_system=2,
            target_component=1,
        ),
        ScoutEndpoint(
            vehicle_id="scout_2",
            host="127.0.0.1",
            port=14542,
            command_port=14582,
            target_system=3,
            target_component=1,
        ),
    ]
    mission_node._reset_vehicle_command_states()
    mission_node._scout_last_seen_monotonic = {
        "scout_1": now,
        "scout_2": now,
    }

    endpoint_calls: list[tuple[str, int, int | None]] = []
    target_calls: list[tuple[int, int]] = []
    hold_calls: list[int] = []

    def _record_endpoint(host: str, port: int, command_port: int | None = None) -> None:
        endpoint_calls.append(
            (
                host,
                int(port),
                None if command_port is None else int(command_port),
            )
        )

    def _record_target(system_id: int, component_id: int = 1) -> None:
        target_calls.append((int(system_id), int(component_id)))

    def _record_hold() -> bool:
        hold_calls.append(1)
        return True

    monkeypatch.setattr(mission_node._mavlink_adapter, "set_endpoint", _record_endpoint)
    monkeypatch.setattr(mission_node._mavlink_adapter, "set_target", _record_target)
    monkeypatch.setattr(mission_node._mavlink_adapter, "send_hold_position", _record_hold)

    request = SimpleNamespace(
        command="HOLD",
        vehicle_id="scout2",
        mission_id="vehicle-command-target-system",
    )
    response = SimpleNamespace(success=False, message="")
    result = mission_node._handle_vehicle_command(request, response)

    assert result.success
    assert hold_calls == [1]
    assert endpoint_calls == [
        ("127.0.0.1", 14542, 14582),
        ("127.0.0.1", 14541, 14581),
    ]
    assert target_calls == [(3, 1), (2, 1)]


def test_vehicle_command_pauses_stream_when_switching_endpoints(
    mission_harness, monkeypatch
) -> None:
    mission_node, observer = mission_harness
    del observer

    now = time.monotonic()
    mission_node._state = "SEARCHING"
    mission_node._mission_id = "vehicle-command-stream-guard"
    mission_node._active_scout_vehicle_id = "scout_1"
    mission_node._plan_state.waypoints = [
        {"x": 10.0, "z": 20.0, "altitude_m": 30.0}
    ]
    mission_node._plan_state.current_waypoint_index = 0
    mission_node._scout_endpoints = [
        ScoutEndpoint(
            vehicle_id="scout_1",
            host="127.0.0.1",
            port=14541,
            command_port=14581,
            target_system=2,
            target_component=1,
        ),
        ScoutEndpoint(
            vehicle_id="scout_2",
            host="127.0.0.1",
            port=14542,
            command_port=14582,
            target_system=3,
            target_component=1,
        ),
    ]
    mission_node._reset_vehicle_command_states()
    mission_node._scout_last_seen_monotonic = {
        "scout_1": now,
        "scout_2": now,
    }
    mission_node._mavlink_adapter._running = True

    endpoint_calls: list[tuple[str, int, int | None]] = []
    target_calls: list[tuple[int, int]] = []
    stop_stream_calls: list[int] = []
    start_stream_calls: list[tuple[str, dict[str, float]]] = []

    def _record_endpoint(host: str, port: int, command_port: int | None = None) -> None:
        endpoint_calls.append((host, int(port), command_port))

    def _record_target(system_id: int, component_id: int = 1) -> None:
        target_calls.append((int(system_id), int(component_id)))

    def _record_stop_stream() -> None:
        stop_stream_calls.append(1)
        mission_node._mavlink_adapter._running = False

    def _record_start_stream(mission_id: str, setpoint: dict[str, float]) -> None:
        start_stream_calls.append((mission_id, dict(setpoint)))
        mission_node._mavlink_adapter._running = True

    monkeypatch.setattr(mission_node._mavlink_adapter, "set_endpoint", _record_endpoint)
    monkeypatch.setattr(mission_node._mavlink_adapter, "set_target", _record_target)
    monkeypatch.setattr(mission_node._mavlink_adapter, "stop_stream", _record_stop_stream)
    monkeypatch.setattr(mission_node._mavlink_adapter, "start_stream", _record_start_stream)
    monkeypatch.setattr(mission_node._mavlink_adapter, "send_hold_position", lambda: True)

    request = SimpleNamespace(
        command="HOLD",
        vehicle_id="scout2",
        mission_id="vehicle-command-stream-guard",
    )
    response = SimpleNamespace(success=False, message="")
    result = mission_node._handle_vehicle_command(request, response)

    assert result.success
    assert stop_stream_calls == [1]
    assert start_stream_calls == [
        (
            "vehicle-command-stream-guard",
            {"x": 10.0, "z": 20.0, "altitude_m": 30.0},
        )
    ]
    assert endpoint_calls == [
        ("127.0.0.1", 14542, 14582),
        ("127.0.0.1", 14541, 14581),
    ]
    assert target_calls == [(3, 1), (2, 1)]


def test_reset_vehicle_command_states_includes_ranger_endpoints(
    mission_harness_multi_vehicle,
) -> None:
    mission_node, observer = mission_harness_multi_vehicle
    del observer

    mission_node._scout_endpoints = [
        ScoutEndpoint(vehicle_id="scout_1", host="127.0.0.1", port=14541),
        ScoutEndpoint(vehicle_id="scout_2", host="127.0.0.1", port=14542),
    ]
    mission_node._ranger_endpoints = [
        ScoutEndpoint(vehicle_id="ranger_1", host="127.0.0.1", port=14543),
    ]

    mission_node._reset_vehicle_command_states()

    assert mission_node._vehicle_command_states == {
        "ranger_1": mission_node._VEHICLE_COMMAND_STATE_ACTIVE,
        "scout_1": mission_node._VEHICLE_COMMAND_STATE_ACTIVE,
        "scout_2": mission_node._VEHICLE_COMMAND_STATE_ACTIVE,
    }
