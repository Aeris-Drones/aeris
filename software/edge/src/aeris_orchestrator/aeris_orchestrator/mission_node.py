import json
import math
import re
import time
from dataclasses import dataclass, field
from typing import Final

import rclpy
from aeris_msgs.msg import (
    AcousticBearing,
    GasIsopleth,
    MissionProgress,
    MissionState,
    Telemetry,
    ThermalHotspot,
)
from aeris_msgs.srv import MissionCommand
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from .mavlink_adapter import MavlinkAdapter
from .search_patterns import generate_waypoints, validate_polygon


@dataclass
class MissionPlanState:
    pattern_type: str = ""
    zone_id: str = ""
    waypoints: list[dict[str, float]] = field(default_factory=list)
    current_waypoint_index: int = 0
    scout_position: dict[str, float] = field(
        default_factory=lambda: {"x": 0.0, "z": 0.0}
    )


@dataclass(frozen=True)
class ScoutEndpoint:
    vehicle_id: str
    host: str
    port: int


@dataclass(frozen=True)
class DetectionEvent:
    sensor_type: str
    confidence: float
    timestamp_sec: float
    local_target: dict[str, float]
    mission_id: str


@dataclass
class TrackingContext:
    active: bool = False
    mission_id: str = ""
    trigger_sensor_type: str = ""
    trigger_confidence: float = 0.0
    trigger_timestamp_sec: float = 0.0
    trigger_target: dict[str, float] = field(
        default_factory=lambda: {"x": 0.0, "z": 0.0}
    )
    accepted_monotonic: float = 0.0
    dispatch_monotonic: float = 0.0
    dispatch_latency_ms: float = 0.0
    assigned_scout_vehicle_id: str = ""
    completion_reason: str = ""
    timeout_deadline_monotonic: float = 0.0
    previous_plan: MissionPlanState | None = None
    previous_active_scout_vehicle_id: str = ""
    selected_scout_previous_assignment: str = "SEARCHING"
    preserved_non_target_vehicle_ids: list[str] = field(default_factory=list)
    uses_dedicated_adapter: bool = False


class MissionNode(Node):
    """Mission lifecycle orchestrator for simulation and GCS integration."""

    _ACTIVE_PROGRESS_STATES: Final[set[str]] = {"SEARCHING", "TRACKING"}
    _AUTONOMOUS_STATES: Final[set[str]] = {"SEARCHING", "TRACKING"}
    _ABORTABLE_STATES: Final[set[str]] = {"SEARCHING", "TRACKING"}
    _SUPPORTED_PATTERNS: Final[set[str]] = {"lawnmower", "spiral"}
    _EARTH_RADIUS_M: Final[float] = 6_378_137.0

    def __init__(self, *, parameter_overrides: list[Parameter] | None = None) -> None:
        super().__init__(
            "aeris_mission_orchestrator", parameter_overrides=parameter_overrides
        )

        queue_depth = int(self.declare_parameter("queue_depth", 10).value)
        self._control_loop_period = float(
            self.declare_parameter("control_loop_period_sec", 0.02).value
        )
        self._progress_period = float(
            self.declare_parameter("progress_publish_period_sec", 1.0).value
        )
        self._loop_budget_ms = float(self.declare_parameter("loop_budget_ms", 20.0).value)
        self._planning_cycles = int(self.declare_parameter("planning_cycles", 1).value)
        # Kept for backward compatibility with older launch configs.
        self.declare_parameter("enable_tracking_simulation", False)
        self.declare_parameter("tracking_trigger_coverage_percent", 1.0)
        self.declare_parameter("tracking_hold_cycles", 50)
        self._search_area_km2 = float(self.declare_parameter("search_area_km2", 1.5).value)
        self._total_drones = int(self.declare_parameter("total_drones", 3).value)
        self._active_drones = int(self.declare_parameter("active_drones", 3).value)
        self._grid_total = int(self.declare_parameter("grid_total", 100).value)
        self._coverage_increment_per_tick = float(
            self.declare_parameter("coverage_increment_per_tick", 1.0).value
        )
        self._timing_log_interval_sec = float(
            self.declare_parameter("timing_log_interval_sec", 5.0).value
        )

        self._lawnmower_track_spacing_m = float(
            self.declare_parameter("lawnmower_track_spacing_m", 5.0).value
        )
        self._spiral_radial_step_m = float(
            self.declare_parameter("spiral_radial_step_m", 3.0).value
        )
        self._spiral_angular_step_rad = float(
            self.declare_parameter("spiral_angular_step_rad", 0.35).value
        )
        self._waypoint_reached_threshold_m = float(
            self.declare_parameter("waypoint_reached_threshold_m", 1.5).value
        )
        self._simulated_scout_speed_mps = float(
            self.declare_parameter("simulated_scout_speed_mps", 4.0).value
        )
        self._scout_altitude_m = float(self.declare_parameter("scout_altitude_m", 20.0).value)
        self._scout_mavlink_host = str(
            self.declare_parameter("scout_mavlink_host", "127.0.0.1").value
        )
        self._scout_mavlink_udp_port = int(
            self.declare_parameter("scout_mavlink_udp_port", 14540).value
        )
        self._setpoint_stream_hz = float(
            self.declare_parameter("setpoint_stream_hz", 10.0).value
        )
        self._scout_online_window_sec = float(
            self.declare_parameter("scout_online_window_sec", 5.0).value
        )
        self._telemetry_geodetic_to_local_enabled = bool(
            self.declare_parameter("telemetry_geodetic_to_local_enabled", True).value
        )
        self._scout_endpoints_json = str(
            self.declare_parameter(
                "scout_endpoints_json",
                '[{"vehicle_id":"scout1","host":"127.0.0.1","port":14540},'
                '{"vehicle_id":"scout2","host":"127.0.0.1","port":14541}]',
            ).value
        )
        self._thermal_confidence_min = float(
            self.declare_parameter("thermal_confidence_min", 0.75).value
        )
        self._acoustic_confidence_min = float(
            self.declare_parameter("acoustic_confidence_min", 0.65).value
        )
        self._gas_trigger_min = float(self.declare_parameter("gas_trigger_min", 0.5).value)
        self._replan_cooldown_sec = float(
            self.declare_parameter("replan_cooldown_sec", 5.0).value
        )
        self._detection_stale_ms = float(
            self.declare_parameter("detection_stale_ms", 2000.0).value
        )
        self._tracking_focus_radius_m = float(
            self.declare_parameter("tracking_focus_radius_m", 50.0).value
        )
        self._tracking_timeout_sec = float(
            self.declare_parameter("tracking_timeout_sec", 30.0).value
        )
        self._tracking_dispatch_budget_ms = float(
            self.declare_parameter("tracking_dispatch_budget_ms", 200.0).value
        )
        self._tracking_completion_coverage_percent = float(
            self.declare_parameter("tracking_completion_coverage_percent", 100.0).value
        )
        self._detection_retrigger_from_tracking = bool(
            self.declare_parameter("detection_retrigger_from_tracking", True).value
        )
        self._acoustic_projection_range_m = float(
            self.declare_parameter("acoustic_projection_range_m", 35.0).value
        )
        self._thermal_pixels_to_meters = float(
            self.declare_parameter("thermal_pixels_to_meters", 0.1).value
        )
        self._tracking_pattern = str(
            self.declare_parameter("tracking_pattern", "spiral").value
        ).strip().lower()
        if self._tracking_pattern not in self._SUPPORTED_PATTERNS:
            self._tracking_pattern = "spiral"

        self._state = "IDLE"
        self._mission_id = ""
        self._mission_started_sec = 0.0
        self._planning_countdown = 0
        self._tracking_seen_this_mission = False
        self._coverage_percent = 0.0
        self._grid_completed = 0
        self._loop_iteration = 0
        self._loop_stats_since_log = 0
        self._loop_elapsed_total_ms = 0.0
        self._loop_elapsed_max_ms = 0.0
        self._last_timing_log_monotonic = time.perf_counter()
        self._plan_state = MissionPlanState()
        self._scout_last_seen_monotonic: dict[str, float] = {}
        self._scout_position_snapshot: dict[str, dict[str, float]] = {}
        self._scout_geodetic_snapshot: dict[str, dict[str, float]] = {}
        self._telemetry_geodetic_origin: dict[str, float] | None = None
        self._active_scout_vehicle_id = ""
        self._scout_assignments: dict[str, str] = {}
        self._scout_endpoints = self._parse_scout_endpoints()
        self._tracking_context = TrackingContext()
        self._last_detection_event: DetectionEvent | None = None
        self._last_detection_rejection_reason = ""
        self._last_tracking_completion_reason = ""
        self._last_detection_accept_monotonic = -math.inf
        self._last_tracking_dispatch_latency_ms = 0.0

        self._mavlink_adapter = MavlinkAdapter(
            host=self._scout_mavlink_host,
            port=self._scout_mavlink_udp_port,
            stream_hz=self._setpoint_stream_hz,
            logger=lambda message: self.get_logger().info(message),
        )
        self._tracking_mavlink_adapter: MavlinkAdapter | None = None

        self._state_pub = self.create_publisher(
            MissionState, "/orchestrator/mission_state", queue_depth
        )
        self._state_string_pub = self.create_publisher(
            String, "/mission/state", queue_depth
        )
        self._progress_pub = self.create_publisher(
            MissionProgress, "/orchestrator/mission_progress", queue_depth
        )
        self._progress_string_pub = self.create_publisher(
            String, "/mission/progress", queue_depth
        )
        self._serialized_callback_group = MutuallyExclusiveCallbackGroup()
        self._telemetry_subscription = self.create_subscription(
            Telemetry,
            "/vehicle/telemetry",
            self._handle_vehicle_telemetry,
            queue_depth,
        )
        self._thermal_subscription = self.create_subscription(
            ThermalHotspot,
            "thermal/hotspots",
            self._handle_thermal_hotspot,
            queue_depth,
            callback_group=self._serialized_callback_group,
        )
        self._acoustic_subscription = self.create_subscription(
            AcousticBearing,
            "acoustic/bearing",
            self._handle_acoustic_bearing,
            queue_depth,
            callback_group=self._serialized_callback_group,
        )
        self._gas_subscription = self.create_subscription(
            GasIsopleth,
            "gas/isopleth",
            self._handle_gas_isopleth,
            queue_depth,
            callback_group=self._serialized_callback_group,
        )
        self._tracking_resolution_subscription = self.create_subscription(
            String,
            "/mission/tracking_resolution",
            self._handle_tracking_resolution_signal,
            queue_depth,
            callback_group=self._serialized_callback_group,
        )

        self._start_service = self.create_service(
            MissionCommand,
            "start_mission",
            self._handle_start_mission,
            callback_group=self._serialized_callback_group,
        )
        self._abort_service = self.create_service(
            MissionCommand,
            "abort_mission",
            self._handle_abort_mission,
            callback_group=self._serialized_callback_group,
        )

        self._control_timer = self.create_timer(
            self._control_loop_period,
            self._control_loop,
            callback_group=self._serialized_callback_group,
        )
        self._progress_timer = self.create_timer(
            self._progress_period,
            self._publish_progress_if_active,
            callback_group=self._serialized_callback_group,
        )

        self._publish_state(previous_state="")
        self.get_logger().info(
            "Mission node ready: state topic=/orchestrator/mission_state, "
            "progress topic=/orchestrator/mission_progress, "
            f"loop={self._control_loop_period:.3f}s budget={self._loop_budget_ms:.2f}ms, "
            f"mavlink={self._scout_mavlink_host}:{self._scout_mavlink_udp_port}, "
            f"scout_endpoints={len(self._scout_endpoints)}"
        )

    def destroy_node(self) -> bool:
        self._close_tracking_adapter()
        self._mavlink_adapter.close()
        return super().destroy_node()

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _transition_to(self, new_state: str) -> None:
        if new_state == self._state:
            return
        previous = self._state
        self._state = new_state
        self._publish_state(previous_state=previous)
        self.get_logger().info(f"Mission state transition: {previous} -> {new_state}")

        if new_state in self._AUTONOMOUS_STATES and self._plan_state.waypoints:
            self._start_autonomous_execution()

        if new_state in {"ABORTED", "COMPLETE", "IDLE"}:
            self._close_tracking_adapter()
            self._mavlink_adapter.stop_stream()
            self._mission_started_sec = 0.0
            if new_state in {"ABORTED", "IDLE"}:
                self._active_scout_vehicle_id = ""
            self._scout_assignments.clear()
            self._reset_tracking_context()

    def _publish_state(self, previous_state: str) -> None:
        state_message = MissionState()
        state_message.state = self._state
        state_message.mission_id = self._mission_id
        state_message.timestamp = self._now_seconds()
        state_message.previous_state = previous_state
        self._state_pub.publish(state_message)

        legacy_state_message = String()
        legacy_state_message.data = self._state
        self._state_string_pub.publish(legacy_state_message)

    def _reset_progress(self) -> None:
        self._coverage_percent = 0.0
        self._grid_completed = 0
        self._tracking_seen_this_mission = False
        self._last_tracking_dispatch_latency_ms = 0.0
        self._last_detection_accept_monotonic = -math.inf
        self._last_detection_rejection_reason = ""
        self._last_tracking_completion_reason = ""
        self._last_detection_event = None
        self._scout_assignments.clear()
        self._reset_tracking_context()

    def _reset_plan_state(self) -> None:
        self._plan_state = MissionPlanState()

    def _reset_tracking_context(self) -> None:
        self._tracking_context = TrackingContext()

    def _close_tracking_adapter(self) -> None:
        if self._tracking_mavlink_adapter is None:
            return
        self._tracking_mavlink_adapter.close()
        self._tracking_mavlink_adapter = None

    def _set_scout_assignment(self, vehicle_id: str, assignment: str) -> None:
        normalized_vehicle_id = self._normalize_vehicle_id(vehicle_id)
        if not normalized_vehicle_id:
            return
        self._scout_assignments[normalized_vehicle_id] = assignment

    def _initialize_search_assignments(self, active_scout_vehicle_id: str) -> None:
        for endpoint in self._scout_endpoints:
            self._set_scout_assignment(endpoint.vehicle_id, "SEARCHING")
        for vehicle_id in self._scout_last_seen_monotonic:
            self._set_scout_assignment(vehicle_id, "SEARCHING")
        self._set_scout_assignment(active_scout_vehicle_id, "SEARCHING")

    def _clone_plan_state(self, plan: MissionPlanState) -> MissionPlanState:
        return MissionPlanState(
            pattern_type=plan.pattern_type,
            zone_id=plan.zone_id,
            waypoints=[dict(waypoint) for waypoint in plan.waypoints],
            current_waypoint_index=int(plan.current_waypoint_index),
            scout_position={
                "x": float(plan.scout_position.get("x", 0.0)),
                "z": float(plan.scout_position.get("z", 0.0)),
            },
        )

    def _normalized_command(self, command: str) -> str:
        return command.strip().upper()

    def _normalize_vehicle_id(self, value: str) -> str:
        normalized = value.strip().lower().replace("-", "_")
        match = re.fullmatch(r"([a-z]+)(\d+)", normalized)
        if match:
            normalized = f"{match.group(1)}_{match.group(2)}"
        return normalized

    def _parse_scout_endpoints(self) -> list[ScoutEndpoint]:
        endpoints: list[ScoutEndpoint] = []
        raw_json = self._scout_endpoints_json.strip()
        if raw_json:
            try:
                payload = json.loads(raw_json)
                if isinstance(payload, list):
                    for item in payload:
                        if not isinstance(item, dict):
                            continue
                        vehicle_id_raw = str(item.get("vehicle_id", "")).strip()
                        host = str(item.get("host", "127.0.0.1")).strip() or "127.0.0.1"
                        try:
                            port = int(item.get("port", 0))
                        except (TypeError, ValueError):
                            continue
                        if not vehicle_id_raw or port <= 0:
                            continue
                        endpoints.append(
                            ScoutEndpoint(
                                vehicle_id=self._normalize_vehicle_id(vehicle_id_raw),
                                host=host,
                                port=port,
                            )
                        )
            except json.JSONDecodeError:
                self.get_logger().warning(
                    "Invalid scout_endpoints_json; falling back to single scout endpoint"
                )

        if not endpoints:
            endpoints.append(
                ScoutEndpoint(
                    vehicle_id="scout_1",
                    host=self._scout_mavlink_host,
                    port=self._scout_mavlink_udp_port,
                )
            )
        return endpoints

    def _handle_vehicle_telemetry(self, message: Telemetry) -> None:
        vehicle_type = str(message.vehicle_type).strip().lower()
        vehicle_id = self._normalize_vehicle_id(message.vehicle_id)
        if not vehicle_id:
            return
        if vehicle_type and vehicle_type != "scout" and not vehicle_id.startswith("scout"):
            return

        latitude = float(message.position.latitude)
        longitude = float(message.position.longitude)

        self._scout_last_seen_monotonic[vehicle_id] = time.monotonic()
        if math.isfinite(latitude) and math.isfinite(longitude):
            self._scout_geodetic_snapshot[vehicle_id] = {
                "latitude": latitude,
                "longitude": longitude,
            }
        local_position = self._telemetry_to_local_position(latitude, longitude)
        self._scout_position_snapshot[vehicle_id] = local_position
        if self._mission_id and vehicle_id not in self._scout_assignments:
            if (
                self._tracking_context.active
                and vehicle_id == self._tracking_context.assigned_scout_vehicle_id
            ):
                self._set_scout_assignment(vehicle_id, "TRACKING")
            else:
                self._set_scout_assignment(vehicle_id, "SEARCHING")

    def _handle_thermal_hotspot(self, message: ThermalHotspot) -> None:
        event = self._normalize_thermal_hotspot(message)
        self._process_detection_event(event)

    def _handle_acoustic_bearing(self, message: AcousticBearing) -> None:
        event = self._normalize_acoustic_bearing(message)
        self._process_detection_event(event)

    def _handle_gas_isopleth(self, message: GasIsopleth) -> None:
        event = self._normalize_gas_isopleth(message)
        self._process_detection_event(event)

    def _handle_tracking_resolution_signal(self, message: String) -> None:
        signal = message.data.strip().lower()
        if not signal:
            return
        if self._state != "TRACKING" or not self._tracking_context.active:
            return
        if signal in {"resolved", "clear", "cleared", "complete", "done"}:
            self._complete_tracking("resolution signal")

    def _time_message_to_seconds(self, stamp) -> float:
        sec = float(getattr(stamp, "sec", 0))
        nanosec = float(getattr(stamp, "nanosec", 0))
        return sec + (nanosec / 1e9)

    def _telemetry_to_local_position(
        self, latitude: float, longitude: float
    ) -> dict[str, float]:
        # Some simulation environments may already provide local coordinates in
        # GeoPosition fields. If values are outside valid geodetic bounds, keep
        # legacy passthrough behavior.
        if not math.isfinite(latitude) or not math.isfinite(longitude):
            return {"x": 0.0, "z": 0.0}
        if not (-90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0):
            return {"x": longitude, "z": latitude}
        if not self._telemetry_geodetic_to_local_enabled:
            return {"x": longitude, "z": latitude}

        if self._telemetry_geodetic_origin is None:
            self._telemetry_geodetic_origin = {
                "latitude": latitude,
                "longitude": longitude,
            }
        origin_lat = float(self._telemetry_geodetic_origin["latitude"])
        origin_lon = float(self._telemetry_geodetic_origin["longitude"])
        north_m = math.radians(latitude - origin_lat) * self._EARTH_RADIUS_M
        east_scale = math.cos(math.radians(origin_lat))
        east_m = math.radians(longitude - origin_lon) * self._EARTH_RADIUS_M * east_scale
        return {"x": east_m, "z": north_m}

    def _active_scout_position(self) -> dict[str, float]:
        if self._active_scout_vehicle_id:
            snapshot = self._scout_position_snapshot.get(self._active_scout_vehicle_id)
            if snapshot is not None:
                return {"x": snapshot["x"], "z": snapshot["z"]}
        if self._plan_state.scout_position:
            return {
                "x": float(self._plan_state.scout_position.get("x", 0.0)),
                "z": float(self._plan_state.scout_position.get("z", 0.0)),
            }
        return {"x": 0.0, "z": 0.0}

    def _normalize_thermal_hotspot(self, message: ThermalHotspot) -> DetectionEvent:
        bbox = list(message.bbox_px)
        center_x = 320.0
        center_y = 240.0
        if len(bbox) >= 4:
            center_x = (float(bbox[0]) + float(bbox[2])) / 2.0
            center_y = (float(bbox[1]) + float(bbox[3])) / 2.0

        base = self._active_scout_position()
        target = {
            "x": base["x"] + ((center_x - 320.0) * self._thermal_pixels_to_meters),
            "z": base["z"] + ((center_y - 240.0) * self._thermal_pixels_to_meters),
        }
        timestamp_sec = self._time_message_to_seconds(message.stamp)
        if timestamp_sec <= 0.0:
            timestamp_sec = self._now_seconds()
        return DetectionEvent(
            sensor_type="thermal",
            confidence=float(message.confidence),
            timestamp_sec=timestamp_sec,
            local_target=target,
            mission_id=self._mission_id,
        )

    def _normalize_acoustic_bearing(self, message: AcousticBearing) -> DetectionEvent:
        base = self._active_scout_position()
        bearing_rad = math.radians(float(message.bearing_deg))
        target = {
            "x": base["x"] + (math.sin(bearing_rad) * self._acoustic_projection_range_m),
            "z": base["z"] + (math.cos(bearing_rad) * self._acoustic_projection_range_m),
        }
        timestamp_sec = self._time_message_to_seconds(message.stamp)
        if timestamp_sec <= 0.0:
            timestamp_sec = self._now_seconds()
        return DetectionEvent(
            sensor_type="acoustic",
            confidence=float(message.confidence),
            timestamp_sec=timestamp_sec,
            local_target=target,
            mission_id=self._mission_id,
        )

    def _normalize_gas_isopleth(self, message: GasIsopleth) -> DetectionEvent:
        points: list[tuple[float, float]] = []
        for point in message.centerline:
            points.append((float(point.x), float(point.y)))
        if not points:
            for polygon in message.polygons:
                for point in polygon.points:
                    points.append((float(point.x), float(point.y)))

        if points:
            mean_x = sum(point[0] for point in points) / len(points)
            mean_z = sum(point[1] for point in points) / len(points)
            target = {"x": mean_x, "z": mean_z}
        else:
            target = self._active_scout_position()

        confidence = 0.0
        if message.centerline:
            confidence = 1.0
        elif message.polygons:
            confidence = 0.7

        timestamp_sec = self._time_message_to_seconds(message.stamp)
        if timestamp_sec <= 0.0:
            timestamp_sec = self._now_seconds()
        return DetectionEvent(
            sensor_type="gas",
            confidence=confidence,
            timestamp_sec=timestamp_sec,
            local_target=target,
            mission_id=self._mission_id,
        )

    def _online_scout_endpoints(self) -> list[ScoutEndpoint]:
        now = time.monotonic()
        online_window = max(self._scout_online_window_sec, 0.0)
        online: list[ScoutEndpoint] = []
        for endpoint in self._scout_endpoints:
            last_seen = self._scout_last_seen_monotonic.get(endpoint.vehicle_id)
            if last_seen is None:
                continue
            if now - last_seen <= online_window:
                online.append(endpoint)
        return online

    def _select_first_available_scout(self) -> ScoutEndpoint | None:
        online = self._online_scout_endpoints()
        if online:
            return online[0]
        return None

    def _select_tracking_scout_endpoint(
        self, target: dict[str, float]
    ) -> ScoutEndpoint | None:
        online = self._online_scout_endpoints()
        if not online:
            return None

        with_position: list[tuple[float, float, str, ScoutEndpoint]] = []
        without_position: list[tuple[float, str, ScoutEndpoint]] = []
        for endpoint in online:
            last_seen = self._scout_last_seen_monotonic.get(endpoint.vehicle_id, -math.inf)
            position = self._scout_position_snapshot.get(endpoint.vehicle_id)
            if position is None:
                without_position.append((-last_seen, endpoint.vehicle_id, endpoint))
                continue
            distance = math.hypot(
                float(position["x"]) - float(target["x"]),
                float(position["z"]) - float(target["z"]),
            )
            with_position.append((-last_seen, distance, endpoint.vehicle_id, endpoint))

        if with_position:
            with_position.sort(key=lambda item: (item[1], item[0], item[2]))
            return with_position[0][3]

        without_position.sort(key=lambda item: (item[0], item[1]))
        return without_position[0][2]

    def _set_default_mission_id_if_needed(self, mission_id: str) -> str:
        stripped = mission_id.strip()
        if stripped:
            return stripped
        return f"mission-{int(time.time())}"

    def _active_abort_endpoints(self) -> list[ScoutEndpoint]:
        if not self._scout_endpoints:
            return []

        now = time.monotonic()
        online_window = max(self._scout_online_window_sec, 0.0)
        selected: list[ScoutEndpoint] = []
        selected_ids: set[str] = set()

        for endpoint in self._scout_endpoints:
            last_seen = self._scout_last_seen_monotonic.get(endpoint.vehicle_id)
            if last_seen is None:
                continue
            if now - last_seen <= online_window:
                selected.append(endpoint)
                selected_ids.add(endpoint.vehicle_id)

        if self._active_scout_vehicle_id:
            for endpoint in self._scout_endpoints:
                if endpoint.vehicle_id != self._active_scout_vehicle_id:
                    continue
                if endpoint.vehicle_id not in selected_ids:
                    selected.append(endpoint)
                    selected_ids.add(endpoint.vehicle_id)
                break

        if selected:
            return selected
        return list(self._scout_endpoints)

    def _dispatch_abort_rtl(self, endpoints: list[ScoutEndpoint]) -> tuple[int, int]:
        success_count = 0
        total_count = len(endpoints)

        for endpoint in endpoints:
            try:
                self._mavlink_adapter.set_endpoint(endpoint.host, endpoint.port)
                sent = self._mavlink_adapter.send_return_to_launch()
                if sent:
                    success_count += 1
                else:
                    self.get_logger().warning(
                        "Abort RTL dispatch was not acknowledged for "
                        f"{endpoint.vehicle_id} ({endpoint.host}:{endpoint.port})"
                    )
            except OSError as error:
                self.get_logger().warning(
                    "Abort RTL dispatch failed for "
                    f"{endpoint.vehicle_id} ({endpoint.host}:{endpoint.port}): {error}"
                )

        return success_count, total_count

    def _confidence_threshold(self, sensor_type: str) -> float:
        if sensor_type == "thermal":
            return self._thermal_confidence_min
        if sensor_type == "acoustic":
            return self._acoustic_confidence_min
        if sensor_type == "gas":
            return self._gas_trigger_min
        return 1.0

    def _validate_detection_event(
        self, event: DetectionEvent
    ) -> tuple[bool, str]:
        if not self._mission_id:
            return False, "no active mission"
        if event.mission_id and event.mission_id != self._mission_id:
            return False, "mission_id mismatch"

        allowed_states = {"SEARCHING"}
        if self._detection_retrigger_from_tracking:
            allowed_states.add("TRACKING")
        if self._state not in allowed_states:
            return False, f"state {self._state} does not accept detection triggers"

        threshold = self._confidence_threshold(event.sensor_type)
        if event.confidence < threshold:
            return (
                False,
                f"{event.sensor_type} confidence {event.confidence:.3f} below threshold {threshold:.3f}",
            )

        now_sec = self._now_seconds()
        stale_window = max(self._detection_stale_ms, 0.0) / 1000.0
        age_sec = now_sec - event.timestamp_sec
        if age_sec > stale_window:
            return (
                False,
                f"detection is stale by {age_sec:.3f}s (window {stale_window:.3f}s)",
            )

        if self._mission_started_sec > 0.0 and event.timestamp_sec < self._mission_started_sec:
            return False, "detection predates active mission"

        cooldown_age = time.monotonic() - self._last_detection_accept_monotonic
        if cooldown_age < max(self._replan_cooldown_sec, 0.0):
            return (
                False,
                f"cooldown active ({cooldown_age:.3f}s < {self._replan_cooldown_sec:.3f}s)",
            )

        target_x = float(event.local_target.get("x", 0.0))
        target_z = float(event.local_target.get("z", 0.0))
        if not math.isfinite(target_x) or not math.isfinite(target_z):
            return False, "normalized local target is non-finite"

        return True, ""

    def _build_tracking_polygon(self, target: dict[str, float]) -> list[dict[str, float]]:
        radius = max(self._tracking_focus_radius_m, 1.0)
        vertices: list[dict[str, float]] = []
        for index in range(16):
            angle = (2.0 * math.pi * index) / 16.0
            vertices.append(
                {
                    "x": float(target["x"]) + (radius * math.cos(angle)),
                    "z": float(target["z"]) + (radius * math.sin(angle)),
                }
            )
        return vertices

    def _build_tracking_plan(
        self, target: dict[str, float], *, source: str
    ) -> MissionPlanState | None:
        polygon = self._build_tracking_polygon(target)
        waypoints_2d = generate_waypoints(
            self._tracking_pattern,
            polygon,
            lawnmower_track_spacing_m=self._lawnmower_track_spacing_m,
            spiral_radial_step_m=self._spiral_radial_step_m,
            spiral_angular_step_rad=self._spiral_angular_step_rad,
        )
        if not waypoints_2d:
            return None

        waypoints = [
            {
                "x": float(waypoint["x"]),
                "z": float(waypoint["z"]),
                "altitude_m": self._scout_altitude_m,
            }
            for waypoint in waypoints_2d
        ]
        return MissionPlanState(
            pattern_type=f"tracking-{self._tracking_pattern}",
            zone_id=f"tracking-{source}",
            waypoints=waypoints,
            current_waypoint_index=0,
            scout_position={"x": float(target["x"]), "z": float(target["z"])},
        )

    def _select_endpoint_by_vehicle_id(self, vehicle_id: str) -> ScoutEndpoint | None:
        for endpoint in self._scout_endpoints:
            if endpoint.vehicle_id == vehicle_id:
                return endpoint
        return None

    def _tracking_command_adapter(self) -> MavlinkAdapter:
        if (
            self._tracking_context.active
            and self._tracking_context.uses_dedicated_adapter
            and self._tracking_mavlink_adapter is not None
        ):
            return self._tracking_mavlink_adapter
        return self._mavlink_adapter

    def _build_tracking_adapter(self, endpoint: ScoutEndpoint) -> MavlinkAdapter:
        return MavlinkAdapter(
            host=endpoint.host,
            port=endpoint.port,
            stream_hz=self._setpoint_stream_hz,
            logger=lambda message: self.get_logger().info(message),
        )

    def _process_detection_event(self, event: DetectionEvent) -> bool:
        is_valid, reason = self._validate_detection_event(event)
        if not is_valid:
            self._last_detection_rejection_reason = reason
            self.get_logger().info(
                f"Detection rejected ({event.sensor_type}): {reason}"
            )
            return False

        tracking_plan = self._build_tracking_plan(
            event.local_target, source=event.sensor_type
        )
        if tracking_plan is None:
            self._last_detection_rejection_reason = "failed to generate focused tracking pattern"
            self.get_logger().warning(
                f"Detection rejected ({event.sensor_type}): failed to build tracking plan"
            )
            return False

        selected_endpoint = self._select_tracking_scout_endpoint(event.local_target)
        if selected_endpoint is None:
            self._last_detection_rejection_reason = "no scout available for tracking dispatch"
            self.get_logger().warning(
                f"Detection rejected ({event.sensor_type}): no online scout for dispatch"
            )
            return False

        selected_scout_previous_assignment = self._scout_assignments.get(
            selected_endpoint.vehicle_id, "SEARCHING"
        )
        preserved_non_target_vehicle_ids = sorted(
            vehicle_id
            for vehicle_id, assignment in self._scout_assignments.items()
            if vehicle_id != selected_endpoint.vehicle_id
            and assignment in {"SEARCHING", "TRACKING"}
        )

        accepted_monotonic = time.monotonic()
        self._tracking_seen_this_mission = True
        previous_plan = self._tracking_context.previous_plan
        previous_scout_vehicle_id = self._tracking_context.previous_active_scout_vehicle_id
        if previous_plan is None:
            previous_plan = self._clone_plan_state(self._plan_state)
        if not previous_scout_vehicle_id:
            previous_scout_vehicle_id = self._active_scout_vehicle_id

        use_dedicated_adapter = (
            bool(previous_scout_vehicle_id)
            and selected_endpoint.vehicle_id != previous_scout_vehicle_id
            and self._mavlink_adapter.is_streaming
        )
        if use_dedicated_adapter:
            self._close_tracking_adapter()
            self._tracking_mavlink_adapter = self._build_tracking_adapter(selected_endpoint)
        else:
            self._close_tracking_adapter()
            self._mavlink_adapter.set_endpoint(selected_endpoint.host, selected_endpoint.port)

        self._last_detection_accept_monotonic = accepted_monotonic
        self._last_detection_event = event
        self._last_detection_rejection_reason = ""

        self._active_scout_vehicle_id = selected_endpoint.vehicle_id
        self._set_scout_assignment(selected_endpoint.vehicle_id, "TRACKING")
        self._plan_state = tracking_plan

        self._tracking_context = TrackingContext(
            active=True,
            mission_id=self._mission_id,
            trigger_sensor_type=event.sensor_type,
            trigger_confidence=event.confidence,
            trigger_timestamp_sec=event.timestamp_sec,
            trigger_target=dict(event.local_target),
            accepted_monotonic=accepted_monotonic,
            assigned_scout_vehicle_id=selected_endpoint.vehicle_id,
            timeout_deadline_monotonic=accepted_monotonic
            + max(self._tracking_timeout_sec, 0.1),
            previous_plan=previous_plan,
            previous_active_scout_vehicle_id=previous_scout_vehicle_id,
            selected_scout_previous_assignment=selected_scout_previous_assignment,
            preserved_non_target_vehicle_ids=preserved_non_target_vehicle_ids,
            uses_dedicated_adapter=use_dedicated_adapter,
        )

        # Transition first so the execution path starts/adapts streaming; latency
        # measurement below intentionally times the first adapter-level setpoint emit.
        if self._state != "TRACKING":
            self._transition_to("TRACKING")
        else:
            self._start_autonomous_execution()

        dispatch_adapter = self._tracking_command_adapter()
        dispatch_timeout_sec = max(self._tracking_dispatch_budget_ms / 1000.0, 0.001)
        dispatch_monotonic = dispatch_adapter.wait_for_setpoint_dispatch(
            after_monotonic=accepted_monotonic,
            timeout_sec=dispatch_timeout_sec,
        )
        if dispatch_monotonic is None:
            dispatch_monotonic = time.monotonic()
            dispatch_latency_ms = (dispatch_monotonic - accepted_monotonic) * 1000.0
            self._tracking_context.dispatch_monotonic = dispatch_monotonic
            self._tracking_context.dispatch_latency_ms = dispatch_latency_ms
            self._last_tracking_dispatch_latency_ms = dispatch_latency_ms
            reason = (
                "tracking dispatch failed to emit adapter setpoint before latency budget "
                f"({dispatch_latency_ms:.3f}ms > {self._tracking_dispatch_budget_ms:.3f}ms)"
            )
            self._last_detection_rejection_reason = reason
            self.get_logger().warning(reason)
            self._complete_tracking("latency budget exceeded")
            return False

        dispatch_latency_ms = (dispatch_monotonic - accepted_monotonic) * 1000.0
        self._tracking_context.dispatch_monotonic = dispatch_monotonic
        self._tracking_context.dispatch_latency_ms = dispatch_latency_ms
        self._last_tracking_dispatch_latency_ms = dispatch_latency_ms
        if dispatch_latency_ms > self._tracking_dispatch_budget_ms:
            reason = (
                "dispatch latency exceeded budget "
                f"({dispatch_latency_ms:.3f}ms > {self._tracking_dispatch_budget_ms:.3f}ms)"
            )
            self._last_detection_rejection_reason = reason
            self.get_logger().warning(reason)
            self._complete_tracking("latency budget exceeded")
            return False

        self.get_logger().info(
            f"Detection accepted ({event.sensor_type}) confidence={event.confidence:.3f}, "
            f"assigned_scout={selected_endpoint.vehicle_id}, "
            f"dispatch_latency_ms={dispatch_latency_ms:.3f}, "
            f"preserved_non_targets={len(preserved_non_target_vehicle_ids)}"
        )
        return True

    def _complete_tracking(self, reason: str) -> None:
        if not self._tracking_context.active:
            return

        previous_plan = self._tracking_context.previous_plan
        previous_scout_vehicle_id = self._tracking_context.previous_active_scout_vehicle_id
        assigned_scout_vehicle_id = self._tracking_context.assigned_scout_vehicle_id
        selected_scout_previous_assignment = (
            self._tracking_context.selected_scout_previous_assignment or "SEARCHING"
        )
        if selected_scout_previous_assignment == "TRACKING":
            selected_scout_previous_assignment = "SEARCHING"
        self.get_logger().info(f"Tracking completed: {reason}")

        if previous_plan is not None:
            self._plan_state = self._clone_plan_state(previous_plan)

        self._last_tracking_completion_reason = reason
        self._reset_tracking_context()
        self._close_tracking_adapter()

        if assigned_scout_vehicle_id:
            self._set_scout_assignment(
                assigned_scout_vehicle_id, selected_scout_previous_assignment
            )

        if previous_scout_vehicle_id:
            self._active_scout_vehicle_id = previous_scout_vehicle_id
            endpoint = self._select_endpoint_by_vehicle_id(previous_scout_vehicle_id)
            if endpoint is not None:
                self._mavlink_adapter.set_endpoint(endpoint.host, endpoint.port)
        if self._state != "SEARCHING":
            self._transition_to("SEARCHING")
        else:
            self._start_autonomous_execution()

    def _parse_mission_plan(
        self, zone_geometry: str
    ) -> tuple[MissionPlanState | None, str]:
        raw_payload = zone_geometry.strip()
        if not raw_payload:
            return None, (
                "zone_geometry is required. Expected JSON payload: "
                '{"pattern":"lawnmower|spiral","zone":{"id":"...","polygon":[{"x":0,"z":0}]}}'
            )

        try:
            payload = json.loads(raw_payload)
        except json.JSONDecodeError as error:
            return None, f"zone_geometry is not valid JSON: {error.msg}"

        if not isinstance(payload, dict):
            return None, "zone_geometry root value must be a JSON object"

        pattern = str(payload.get("pattern", "")).strip().lower()
        if pattern not in self._SUPPORTED_PATTERNS:
            return (
                None,
                f"unsupported pattern '{pattern}'. Supported values: lawnmower, spiral",
            )

        zone_payload = payload.get("zone")
        if not isinstance(zone_payload, dict):
            return None, "zone object is required in zone_geometry"

        zone_id = str(zone_payload.get("id", "")).strip()
        if not zone_id:
            return None, "zone.id is required in zone_geometry"

        polygon = zone_payload.get("polygon")
        if not isinstance(polygon, list):
            return None, "zone.polygon must be an array of points"

        valid_polygon, polygon_error = validate_polygon(polygon)
        if not valid_polygon:
            return None, f"invalid zone polygon: {polygon_error}"

        waypoints_2d = generate_waypoints(
            pattern,
            polygon,
            lawnmower_track_spacing_m=self._lawnmower_track_spacing_m,
            spiral_radial_step_m=self._spiral_radial_step_m,
            spiral_angular_step_rad=self._spiral_angular_step_rad,
        )
        if not waypoints_2d:
            return None, "failed to generate waypoints for the provided zone and pattern"

        waypoints: list[dict[str, float]] = []
        for waypoint in waypoints_2d:
            waypoints.append(
                {
                    "x": float(waypoint["x"]),
                    "z": float(waypoint["z"]),
                    "altitude_m": self._scout_altitude_m,
                }
            )

        initial_position = {"x": waypoints[0]["x"], "z": waypoints[0]["z"]}
        return (
            MissionPlanState(
                pattern_type=pattern,
                zone_id=zone_id,
                waypoints=waypoints,
                current_waypoint_index=0,
                scout_position=initial_position,
            ),
            "",
        )

    def _current_waypoint(self) -> dict[str, float] | None:
        if not self._plan_state.waypoints:
            return None
        index = self._plan_state.current_waypoint_index
        if index < 0 or index >= len(self._plan_state.waypoints):
            return None
        return self._plan_state.waypoints[index]

    def _start_autonomous_execution(self) -> None:
        waypoint = self._current_waypoint()
        if waypoint is None:
            return
        adapter = self._tracking_command_adapter()
        if adapter.is_streaming:
            adapter.update_setpoint(waypoint)
            return

        adapter.upload_mission_items_int(self._mission_id, self._plan_state.waypoints)
        adapter.start_stream(self._mission_id, waypoint)

    def _advance_waypoint_index(self) -> None:
        current = self._plan_state.current_waypoint_index
        next_index = current + 1
        if next_index >= len(self._plan_state.waypoints):
            if self._state == "TRACKING" and self._tracking_context.active:
                self._complete_tracking("focused area covered")
                return
            self._transition_to("COMPLETE")
            return

        self._plan_state.current_waypoint_index = next_index
        next_waypoint = self._plan_state.waypoints[next_index]
        self._tracking_command_adapter().update_setpoint(next_waypoint)
        self.get_logger().info(
            f"Advancing to waypoint {next_index + 1}/{len(self._plan_state.waypoints)}"
        )

    def _update_scout_position_towards_waypoint(self) -> None:
        waypoint = self._current_waypoint()
        if waypoint is None:
            return

        position = self._plan_state.scout_position
        delta_x = waypoint["x"] - position["x"]
        delta_z = waypoint["z"] - position["z"]
        distance = math.hypot(delta_x, delta_z)

        if distance <= self._waypoint_reached_threshold_m:
            self._advance_waypoint_index()
            return

        step = self._simulated_scout_speed_mps * self._control_loop_period
        if step <= 0.0:
            return

        if distance <= step:
            position["x"] = waypoint["x"]
            position["z"] = waypoint["z"]
        else:
            scale = step / distance
            position["x"] += delta_x * scale
            position["z"] += delta_z * scale

    def _handle_start_mission(
        self, request: MissionCommand.Request, response: MissionCommand.Response
    ) -> MissionCommand.Response:
        if self._normalized_command(request.command) != "START":
            response.success = False
            response.message = (
                f"start_mission rejected due to unsupported command '{request.command}'"
            )
            return response

        if self._state != "IDLE":
            response.success = False
            response.message = f"start_mission rejected while in {self._state}"
            return response

        plan_state, plan_error = self._parse_mission_plan(request.zone_geometry)
        if plan_state is None:
            response.success = False
            response.message = f"start_mission validation error: {plan_error}"
            return response

        selected_scout = self._select_first_available_scout()
        if selected_scout is None:
            response.success = False
            response.message = (
                "start_mission rejected: no scout endpoint reported telemetry recently"
            )
            return response
        self._active_scout_vehicle_id = selected_scout.vehicle_id
        self._close_tracking_adapter()
        self._mavlink_adapter.set_endpoint(selected_scout.host, selected_scout.port)

        self._mission_id = self._set_default_mission_id_if_needed(request.mission_id)
        self._mission_started_sec = self._now_seconds()
        self._reset_progress()
        self._initialize_search_assignments(selected_scout.vehicle_id)
        self._plan_state = plan_state
        self._planning_countdown = max(self._planning_cycles, 1)
        self._transition_to("PLANNING")

        response.success = True
        response.message = (
            f"Mission {self._mission_id} accepted with pattern '{self._plan_state.pattern_type}' "
            f"({len(self._plan_state.waypoints)} waypoints) on scout '{self._active_scout_vehicle_id}'"
        )
        return response

    def _handle_abort_mission(
        self, request: MissionCommand.Request, response: MissionCommand.Response
    ) -> MissionCommand.Response:
        if self._normalized_command(request.command) != "ABORT":
            response.success = False
            response.message = (
                f"abort_mission rejected due to unsupported command '{request.command}'"
            )
            return response

        if self._state not in self._ABORTABLE_STATES:
            response.success = False
            response.message = f"abort_mission rejected while in {self._state}"
            return response

        if request.mission_id.strip() and request.mission_id.strip() != self._mission_id:
            response.success = False
            response.message = "abort_mission rejected due to mission_id mismatch"
            return response

        # Pause setpoint streaming before switching MAVLink endpoints for RTL fan-out.
        self._close_tracking_adapter()
        self._mavlink_adapter.stop_stream()
        target_endpoints = self._active_abort_endpoints()
        successful_dispatches, total_dispatches = self._dispatch_abort_rtl(target_endpoints)

        self._transition_to("ABORTED")
        self._reset_plan_state()
        response.success = True
        response.message = (
            f"Mission {self._mission_id} aborted; RTL dispatches "
            f"{successful_dispatches}/{total_dispatches}"
        )
        return response

    def _control_loop(self) -> None:
        start = time.perf_counter()

        if self._state == "PLANNING":
            self._planning_countdown -= 1
            if self._planning_countdown <= 0:
                self._transition_to("SEARCHING")
        elif self._state == "TRACKING":
            if self._tracking_context.active:
                if (
                    self._coverage_percent
                    >= self._tracking_completion_coverage_percent
                ):
                    self._complete_tracking("coverage threshold reached")
                elif time.monotonic() >= self._tracking_context.timeout_deadline_monotonic:
                    self._complete_tracking("tracking timeout reached")
            else:
                self.get_logger().warning(
                    "TRACKING state observed without active tracking context; "
                    "returning to SEARCHING"
                )
                self._transition_to("SEARCHING")

        if self._state in self._AUTONOMOUS_STATES:
            self._update_scout_position_towards_waypoint()

        elapsed_ms = (time.perf_counter() - start) * 1000.0
        self._loop_iteration += 1
        self._loop_stats_since_log += 1
        self._loop_elapsed_total_ms += elapsed_ms
        self._loop_elapsed_max_ms = max(self._loop_elapsed_max_ms, elapsed_ms)
        self.get_logger().debug(
            f"control_loop iteration={self._loop_iteration} elapsed={elapsed_ms:.3f}ms"
        )

        now = time.perf_counter()
        if now - self._last_timing_log_monotonic >= self._timing_log_interval_sec:
            average_ms = self._loop_elapsed_total_ms / max(self._loop_stats_since_log, 1)
            self.get_logger().info(
                "control_loop timing: "
                f"iterations={self._loop_stats_since_log} "
                f"avg={average_ms:.3f}ms "
                f"max={self._loop_elapsed_max_ms:.3f}ms "
                f"budget={self._loop_budget_ms:.3f}ms"
            )
            self._loop_stats_since_log = 0
            self._loop_elapsed_total_ms = 0.0
            self._loop_elapsed_max_ms = 0.0
            self._last_timing_log_monotonic = now

        if elapsed_ms > self._loop_budget_ms:
            self.get_logger().warning(
                f"control_loop budget exceeded: {elapsed_ms:.3f}ms > "
                f"{self._loop_budget_ms:.3f}ms"
            )

    def _publish_progress_if_active(self) -> None:
        if self._state not in self._ACTIVE_PROGRESS_STATES:
            return

        self._coverage_percent = min(
            100.0, self._coverage_percent + self._coverage_increment_per_tick
        )
        covered_area = (self._coverage_percent / 100.0) * self._search_area_km2
        self._grid_completed = int(round((self._coverage_percent / 100.0) * self._grid_total))
        estimated_ticks_remaining = (100.0 - self._coverage_percent) / max(
            self._coverage_increment_per_tick, 0.01
        )
        estimated_time_remaining = max(0.0, estimated_ticks_remaining * self._progress_period)

        progress_message = MissionProgress()
        progress_message.coverage_percent = float(self._coverage_percent)
        progress_message.search_area_km2 = float(self._search_area_km2)
        progress_message.covered_area_km2 = float(covered_area)
        progress_message.active_drones = int(self._active_drones)
        progress_message.total_drones = int(self._total_drones)
        progress_message.estimated_time_remaining = float(estimated_time_remaining)
        progress_message.grid_completed = int(self._grid_completed)
        progress_message.grid_total = int(self._grid_total)
        self._progress_pub.publish(progress_message)

        legacy_progress_message = String()
        legacy_progress_message.data = json.dumps(
            {
                "coveragePercent": progress_message.coverage_percent,
                "searchAreaKm2": progress_message.search_area_km2,
                "coveredAreaKm2": progress_message.covered_area_km2,
                "activeDrones": progress_message.active_drones,
                "totalDrones": progress_message.total_drones,
                "estimatedTimeRemaining": progress_message.estimated_time_remaining,
                "gridProgress": {
                    "completed": progress_message.grid_completed,
                    "total": progress_message.grid_total,
                },
                "activeScoutVehicleId": self._active_scout_vehicle_id,
                "trackingActive": self._tracking_context.active,
                "trackingDispatchLatencyMs": self._last_tracking_dispatch_latency_ms,
                "lastTrackingCompletionReason": self._last_tracking_completion_reason,
                "vehicleAssignments": dict(sorted(self._scout_assignments.items())),
                "trackingPreservedNonTargetVehicles": list(
                    self._tracking_context.preserved_non_target_vehicle_ids
                ),
                "lastDetectionRejection": self._last_detection_rejection_reason,
            }
        )
        self._progress_string_pub.publish(legacy_progress_message)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Mission node shutdown requested.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
