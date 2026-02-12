from collections import deque
import json
import math
import re
import time
from dataclasses import dataclass, field
from typing import Callable, Final

import rclpy
from aeris_msgs.msg import (
    AcousticBearing,
    GasIsopleth,
    MissionProgress,
    MissionState,
    Telemetry,
    ThermalHotspot,
)
from aeris_msgs.srv import MissionCommand, VehicleCommand
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from .mavlink_adapter import MavlinkAdapter
from .search_patterns import (
    generate_waypoints,
    partition_polygon_for_scouts,
    validate_polygon,
)
from .vehicle_ids import normalize_vehicle_id


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
    command_port: int = 0
    target_system: int = 1
    target_component: int = 1


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
    selected_scout_previous_plan: MissionPlanState | None = None
    previous_active_scout_vehicle_id: str = ""
    selected_scout_previous_assignment: str = "SEARCHING"
    preserved_non_target_vehicle_ids: list[str] = field(default_factory=list)
    uses_dedicated_adapter: bool = False


class MissionNode(Node):
    """Mission lifecycle orchestrator for simulation and GCS integration.

    Manages state machine transitions (IDLE -> PLANNING -> SEARCHING -> TRACKING -> COMPLETE/ABORTED)
    and coordinates multi-vehicle search patterns. Integrates thermal, acoustic, and gas
    sensor detections to trigger focused tracking behaviors while maintaining ranger
    overwatch orbits. Supports both GPS-based telemetry and GPS-denied VIO navigation.
    """

    _ACTIVE_PROGRESS_STATES: Final[set[str]] = {"SEARCHING", "TRACKING"}
    _AUTONOMOUS_STATES: Final[set[str]] = {"SEARCHING", "TRACKING"}
    _ABORTABLE_STATES: Final[set[str]] = {"SEARCHING", "TRACKING"}
    _SUPPORTED_PATTERNS: Final[set[str]] = {"lawnmower", "spiral"}
    _EARTH_RADIUS_M: Final[float] = 6_378_137.0
    _SUPPORTED_VEHICLE_COMMANDS: Final[set[str]] = {"HOLD", "RESUME", "RECALL"}
    _VEHICLE_COMMAND_STATE_ACTIVE: Final[str] = "ACTIVE"
    _VEHICLE_COMMAND_STATE_HOLDING: Final[str] = "HOLDING"
    _VEHICLE_COMMAND_STATE_RETURNING: Final[str] = "RETURNING"
    _POSITION_SOURCE_TELEMETRY: Final[str] = "telemetry_geodetic"
    _POSITION_SOURCE_VIO: Final[str] = "vio_odometry"
    _POSITION_SOURCE_AUTO: Final[str] = "auto"
    _SUPPORTED_POSITION_SOURCES: Final[set[str]] = {
        _POSITION_SOURCE_TELEMETRY,
        _POSITION_SOURCE_VIO,
        _POSITION_SOURCE_AUTO,
    }
    _SUPPORTED_SLAM_MODES: Final[set[str]] = {"vio", "liosam"}
    _SLAM_MODE_ALIASES: Final[dict[str, str]] = {
        "rtabmap_vio": "vio",
        "rtabmap": "vio",
        "lio_sam": "liosam",
    }
    _SLAM_MODE_UNKNOWN: Final[str] = "unknown"
    _RETURN_TRAJECTORY_STATE_ACTIVE: Final[str] = "RETURNING"
    _RETURN_TRAJECTORY_STATE_FALLBACK: Final[str] = "FALLBACK"
    _OCCUPANCY_BLOCKED_THRESHOLD: Final[int] = 50
    _RETURN_BREADCRUMB_ALLOWED_STATES: Final[set[str]] = {
        "SEARCHING",
        "TRACKING",
        "RETURNING",
    }

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
        # Legacy parameters retained for backward compatibility with launch configs
        # predating the unified tracking behavior implementation.
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
        self._gps_denied_mode = bool(
            self.declare_parameter("gps_denied_mode", False).value
        )
        self._navigation_position_source = self._normalize_position_source(
            str(
                self.declare_parameter(
                    "navigation_position_source", self._POSITION_SOURCE_TELEMETRY
                ).value
            )
        )
        if (
            self._gps_denied_mode
            and self._navigation_position_source == self._POSITION_SOURCE_AUTO
        ):
            self._navigation_position_source = self._POSITION_SOURCE_VIO
        if (
            self._gps_denied_mode
            and self._navigation_position_source == self._POSITION_SOURCE_TELEMETRY
        ):
            self._navigation_position_source = self._POSITION_SOURCE_VIO
        self._slam_mode = self._normalize_slam_mode(
            str(self.declare_parameter("slam_mode", "vio").value)
        )
        self._vio_odom_stale_sec = max(
            0.0, float(self.declare_parameter("vio_odom_stale_sec", 1.0).value)
        )
        self._return_breadcrumb_limit_per_vehicle = max(
            1, int(self.declare_parameter("return_breadcrumb_limit_per_vehicle", 600).value)
        )
        self._return_breadcrumb_min_spacing_m = max(
            0.0, float(self.declare_parameter("return_breadcrumb_min_spacing_m", 0.75).value)
        )
        self._return_required_freshness_sec = max(
            0.1, float(self.declare_parameter("return_required_freshness_sec", 2.0).value)
        )
        self._return_breadcrumb_max_age_sec = max(
            self._return_required_freshness_sec,
            float(self.declare_parameter("return_breadcrumb_max_age_sec", 1800.0).value),
        )
        self._return_separation_horizontal_m = max(
            0.0, float(self.declare_parameter("return_separation_horizontal_m", 5.0).value)
        )
        self._return_separation_vertical_m = max(
            0.0, float(self.declare_parameter("return_separation_vertical_m", 3.0).value)
        )
        self._return_altitude_offset_m = max(
            0.0, float(self.declare_parameter("return_altitude_offset_m", 5.0).value)
        )
        self._scout_odometry_topics_json = str(
            self.declare_parameter("scout_odometry_topics_json", "").value
        )
        self._scout_endpoints_json = str(
            self.declare_parameter(
                "scout_endpoints_json",
                '[{"vehicle_id":"scout1","host":"127.0.0.1","port":14541,'
                '"command_port":14581,"target_system":2},'
                '{"vehicle_id":"scout2","host":"127.0.0.1","port":14542,'
                '"command_port":14582,"target_system":3}]',
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
        self._ranger_endpoints_json = str(
            self.declare_parameter("ranger_endpoints_json", "[]").value
        )
        self._ranger_overwatch_radius_m = float(
            self.declare_parameter("ranger_overwatch_radius_m", 30.0).value
        )
        self._ranger_overwatch_altitude_m = float(
            self.declare_parameter("ranger_overwatch_altitude_m", 45.0).value
        )
        self._ranger_orbit_waypoint_count = int(
            self.declare_parameter("ranger_orbit_waypoint_count", 16).value
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
        self._scout_vio_position_snapshot: dict[str, dict[str, float]] = {}
        self._scout_vio_last_seen_monotonic: dict[str, float] = {}
        self._scout_vio_status: dict[str, str] = {}
        self._scout_vio_breadcrumbs: dict[str, list[dict[str, float | str]]] = {}
        self._return_paths_by_vehicle: dict[str, list[dict[str, float]]] = {}
        self._return_path_index_by_vehicle: dict[str, int] = {}
        self._return_fallback_reasons: dict[str, str] = {}
        self._return_last_updated_sec_by_vehicle: dict[str, float] = {}
        self._occupancy_map_snapshot: dict[str, object] | None = None
        self._occupancy_map_received_monotonic = -math.inf
        self._fused_hazard_polygons: list[list[dict[str, float]]] = []
        self._fused_hazard_received_monotonic = -math.inf
        self._vehicle_position_sources: dict[str, str] = {}
        self._ranger_position_snapshot: dict[str, dict[str, float]] = {}
        self._ranger_geodetic_snapshot: dict[str, dict[str, float]] = {}
        self._telemetry_geodetic_origin: dict[str, float] | None = None
        self._active_scout_vehicle_id = ""
        self._vehicle_assignments: dict[str, str] = {}
        self._assignment_labels: dict[str, str] = {}
        self._scout_plans: dict[str, MissionPlanState] = {}
        self._zone_polygons_by_vehicle: dict[str, list[dict[str, float]]] = {}
        self._scout_endpoints = self._parse_scout_endpoints()
        self._scout_odometry_topics = self._build_scout_odometry_topic_map()
        self._ranger_endpoints = self._parse_ranger_endpoints()
        self._ranger_last_seen_monotonic: dict[str, float] = {}
        self._active_ranger_vehicle_id = ""
        self._ranger_orbit_waypoints: list[dict[str, float]] = []
        self._ranger_orbit_index = 0
        self._ranger_mavlink_adapter: MavlinkAdapter | None = None
        self._ranger_adapter_vehicle_id = ""
        self._tracking_context = TrackingContext()
        self._last_detection_event: DetectionEvent | None = None
        self._last_detection_rejection_reason = ""
        self._last_tracking_completion_reason = ""
        self._last_detection_accept_monotonic = -math.inf
        self._last_tracking_dispatch_latency_ms = 0.0
        self._vehicle_command_states: dict[str, str] = {}
        self._reset_vehicle_command_states()

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
        self._scout_odometry_subscriptions = []
        for vehicle_id in sorted(self._scout_odometry_topics.keys()):
            odom_topic = self._scout_odometry_topics[vehicle_id]
            self._scout_odometry_subscriptions.append(
                self.create_subscription(
                    Odometry,
                    odom_topic,
                    lambda message, target_vehicle_id=vehicle_id: self._handle_scout_odometry(
                        target_vehicle_id, message
                    ),
                    queue_depth,
                )
            )
        self._occupancy_map_subscription = self.create_subscription(
            OccupancyGrid,
            "/map",
            self._handle_occupancy_map,
            queue_depth,
            callback_group=self._serialized_callback_group,
        )
        self._fused_hazard_subscription = self.create_subscription(
            String,
            "/detections/fused",
            self._handle_fused_hazards,
            queue_depth,
            callback_group=self._serialized_callback_group,
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
        self._vehicle_command_service = self.create_service(
            VehicleCommand,
            "vehicle_command",
            self._handle_vehicle_command,
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
            f"scout_endpoints={len(self._scout_endpoints)}, "
            f"position_source={self._navigation_position_source}, "
            f"slam_mode={self._slam_mode}, "
            f"vio_topics={len(self._scout_odometry_topics)}"
        )

    def destroy_node(self) -> bool:
        self._close_tracking_adapter()
        self._close_ranger_adapter()
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
            self._start_ranger_overwatch_execution()

        if new_state in {"ABORTED", "COMPLETE", "IDLE"}:
            self._close_tracking_adapter()
            self._close_ranger_adapter()
            self._mavlink_adapter.stop_stream()
            self._mission_started_sec = 0.0
            if new_state in {"ABORTED", "IDLE"}:
                self._active_scout_vehicle_id = ""
            self._vehicle_assignments.clear()
            self._assignment_labels.clear()
            self._reset_tracking_context()
            self._reset_vehicle_command_states()

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
        self._vehicle_assignments.clear()
        self._assignment_labels.clear()
        self._vehicle_position_sources.clear()
        self._scout_vio_breadcrumbs.clear()
        self._return_paths_by_vehicle.clear()
        self._return_path_index_by_vehicle.clear()
        self._return_fallback_reasons.clear()
        self._return_last_updated_sec_by_vehicle.clear()
        self._scout_plans.clear()
        self._zone_polygons_by_vehicle.clear()
        self._active_ranger_vehicle_id = ""
        self._ranger_orbit_waypoints = []
        self._ranger_orbit_index = 0
        self._close_ranger_adapter()
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

    def _close_ranger_adapter(self) -> None:
        if self._ranger_mavlink_adapter is None:
            self._ranger_adapter_vehicle_id = ""
            return
        self._ranger_mavlink_adapter.close()
        self._ranger_mavlink_adapter = None
        self._ranger_adapter_vehicle_id = ""

    def _set_scout_assignment(
        self, vehicle_id: str, assignment: str, *, label: str | None = None
    ) -> None:
        normalized_vehicle_id = self._normalize_vehicle_id(vehicle_id)
        if not normalized_vehicle_id:
            return
        self._vehicle_assignments[normalized_vehicle_id] = assignment
        self._assignment_labels[normalized_vehicle_id] = (
            label if label is not None else assignment
        )

    def _initialize_search_assignments(self, active_scout_vehicle_id: str) -> None:
        for endpoint in self._scout_endpoints:
            label = f"SEARCHING:{self._scout_plans[endpoint.vehicle_id].zone_id}" if endpoint.vehicle_id in self._scout_plans else "IDLE"
            assignment = "SEARCHING" if endpoint.vehicle_id in self._scout_plans else "IDLE"
            self._set_scout_assignment(endpoint.vehicle_id, assignment, label=label)
        for vehicle_id in self._scout_last_seen_monotonic:
            if vehicle_id in self._scout_plans:
                label = f"SEARCHING:{self._scout_plans[vehicle_id].zone_id}"
                self._set_scout_assignment(vehicle_id, "SEARCHING", label=label)
        if active_scout_vehicle_id in self._scout_plans:
            label = f"SEARCHING:{self._scout_plans[active_scout_vehicle_id].zone_id}"
            self._set_scout_assignment(active_scout_vehicle_id, "SEARCHING", label=label)

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
        return normalize_vehicle_id(value)

    def _normalize_position_source(self, value: str) -> str:
        normalized = value.strip().lower().replace("-", "_")
        aliases = {
            "telemetry": self._POSITION_SOURCE_TELEMETRY,
            "telemetry_geodetic": self._POSITION_SOURCE_TELEMETRY,
            "vio": self._POSITION_SOURCE_VIO,
            "vio_odometry": self._POSITION_SOURCE_VIO,
            "gps_denied_vio": self._POSITION_SOURCE_VIO,
            "auto": self._POSITION_SOURCE_AUTO,
        }
        candidate = aliases.get(normalized, normalized)
        if candidate in self._SUPPORTED_POSITION_SOURCES:
            return candidate
        self.get_logger().warning(
            "Invalid navigation_position_source '%s'; falling back to '%s'"
            % (value, self._POSITION_SOURCE_TELEMETRY)
        )
        return self._POSITION_SOURCE_TELEMETRY

    def _vehicle_model_topic_token(self, vehicle_id: str) -> str:
        token = self._normalize_vehicle_id(vehicle_id)
        return re.sub(r"_([0-9]+)$", r"\1", token)

    def _default_odometry_topic_for_vehicle(self, vehicle_id: str) -> str:
        return f"/{self._vehicle_model_topic_token(vehicle_id)}/openvins/odom"

    def _build_scout_odometry_topic_map(self) -> dict[str, str]:
        topics_by_vehicle = {
            endpoint.vehicle_id: self._default_odometry_topic_for_vehicle(endpoint.vehicle_id)
            for endpoint in self._scout_endpoints
        }
        raw_json = self._scout_odometry_topics_json.strip()
        if not raw_json:
            return topics_by_vehicle

        try:
            payload = json.loads(raw_json)
        except json.JSONDecodeError:
            self.get_logger().warning(
                "Invalid scout_odometry_topics_json; using default odometry topics"
            )
            return topics_by_vehicle

        if isinstance(payload, dict):
            for raw_vehicle_id, raw_topic in payload.items():
                vehicle_id = self._normalize_vehicle_id(str(raw_vehicle_id))
                topic = str(raw_topic).strip()
                if not vehicle_id or not topic:
                    continue
                topics_by_vehicle[vehicle_id] = topic
            return topics_by_vehicle

        if isinstance(payload, list):
            for item in payload:
                if not isinstance(item, dict):
                    continue
                vehicle_id = self._normalize_vehicle_id(str(item.get("vehicle_id", "")))
                topic = str(item.get("topic", "")).strip()
                if not vehicle_id or not topic:
                    continue
                topics_by_vehicle[vehicle_id] = topic
            return topics_by_vehicle

        self.get_logger().warning(
            "Unsupported scout_odometry_topics_json payload; using default odometry topics"
        )
        return topics_by_vehicle

    def _set_vio_status(self, vehicle_id: str, reason: str) -> None:
        normalized_vehicle_id = self._normalize_vehicle_id(vehicle_id)
        previous_reason = self._scout_vio_status.get(normalized_vehicle_id, "")
        if reason:
            self._scout_vio_status[normalized_vehicle_id] = reason
            if reason != previous_reason:
                self.get_logger().warning(
                    "VIO odometry unavailable for %s: %s" % (normalized_vehicle_id, reason)
                )
            return
        if previous_reason:
            self._scout_vio_status.pop(normalized_vehicle_id, None)
            self.get_logger().info(
                "VIO odometry restored for %s" % normalized_vehicle_id
            )

    @staticmethod
    def _stamp_to_seconds(stamp) -> float:
        sec = float(getattr(stamp, "sec", 0))
        nanosec = float(getattr(stamp, "nanosec", 0))
        return sec + (nanosec / 1e9)

    def _capture_vio_breadcrumb(
        self,
        vehicle_id: str,
        *,
        x_m: float,
        z_m: float,
        timestamp_sec: float,
    ) -> None:
        normalized_vehicle_id = self._normalize_vehicle_id(vehicle_id)
        history = self._scout_vio_breadcrumbs.setdefault(normalized_vehicle_id, [])
        mission_state = str(self._state)
        mission_id = str(self._mission_id)
        if history:
            last = history[-1]
            delta_x = float(last["x"]) - x_m
            delta_z = float(last["z"]) - z_m
            distance_m = math.hypot(delta_x, delta_z)
            if (
                distance_m < self._return_breadcrumb_min_spacing_m
                and str(last.get("missionState", "")) == mission_state
                and str(last.get("missionId", "")) == mission_id
            ):
                last["timestampSec"] = timestamp_sec
                return

        history.append(
            {
                "x": x_m,
                "z": z_m,
                "timestampSec": timestamp_sec,
                "missionState": mission_state,
                "missionId": mission_id,
            }
        )
        overflow = len(history) - self._return_breadcrumb_limit_per_vehicle
        if overflow > 0:
            del history[0:overflow]

    def _handle_occupancy_map(self, message: OccupancyGrid) -> None:
        resolution = float(message.info.resolution)
        if not math.isfinite(resolution) or resolution <= 0.0:
            return
        width = message.info.width
        height = message.info.height
        if width <= 0 or height <= 0:
            return
        data = tuple(int(cell) for cell in message.data)
        if len(data) != width * height:
            return

        timestamp_sec = self._stamp_to_seconds(message.header.stamp)
        if timestamp_sec <= 0.0:
            timestamp_sec = self._now_seconds()
        self._occupancy_map_snapshot = {
            "resolution": resolution,
            "width": width,
            "height": height,
            "origin_x": float(message.info.origin.position.x),
            "origin_z": float(message.info.origin.position.y),
            "data": data,
            "timestamp_sec": timestamp_sec,
        }
        self._occupancy_map_received_monotonic = time.monotonic()

    @staticmethod
    def _extract_local_point(value: object) -> dict[str, float] | None:
        if not isinstance(value, dict):
            return None
        if "x" not in value:
            return None
        z_candidate = value.get("z", value.get("y"))
        if z_candidate is None:
            return None
        try:
            x_value = float(value["x"])
            z_value = float(z_candidate)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(x_value) or not math.isfinite(z_value):
            return None
        return {"x": x_value, "z": z_value}

    @classmethod
    def _extract_hazard_polygons(cls, payload: object) -> list[list[dict[str, float]]]:
        polygons: list[list[dict[str, float]]] = []
        queue: deque[object] = deque([payload])
        while queue:
            current = queue.popleft()
            if isinstance(current, dict):
                for key in (
                    "polygons",
                    "hazards",
                    "zones",
                    "geometry",
                    "features",
                    "points",
                    "polygon",
                ):
                    child = current.get(key)
                    if child is not None:
                        queue.append(child)
                continue
            if isinstance(current, list):
                points = [cls._extract_local_point(item) for item in current]
                cleaned_points = [point for point in points if point is not None]
                if len(cleaned_points) >= 3 and len(cleaned_points) == len(current):
                    polygons.append(cleaned_points)
                    continue
                queue.extend(current)
        return polygons

    def _handle_fused_hazards(self, message: String) -> None:
        raw_data = message.data.strip()
        if not raw_data:
            self._fused_hazard_polygons = []
            self._fused_hazard_received_monotonic = time.monotonic()
            return
        try:
            payload = json.loads(raw_data)
        except json.JSONDecodeError:
            return
        self._fused_hazard_polygons = self._extract_hazard_polygons(payload)
        self._fused_hazard_received_monotonic = time.monotonic()

    def _map_freshness_age_sec(self) -> float:
        if self._occupancy_map_snapshot is None:
            return math.inf
        if not math.isfinite(self._occupancy_map_received_monotonic):
            return math.inf
        return max(0.0, time.monotonic() - self._occupancy_map_received_monotonic)

    def _occupancy_cell_at_local(self, x_m: float, z_m: float) -> int | None:
        snapshot = self._occupancy_map_snapshot
        if snapshot is None:
            return None
        resolution = float(snapshot["resolution"])
        if resolution <= 0.0:
            return None
        width = int(snapshot["width"])
        height = int(snapshot["height"])
        origin_x = float(snapshot["origin_x"])
        origin_z = float(snapshot["origin_z"])
        grid_x = int(math.floor((x_m - origin_x) / resolution))
        grid_z = int(math.floor((z_m - origin_z) / resolution))
        if grid_x < 0 or grid_z < 0 or grid_x >= width or grid_z >= height:
            return None
        index = (grid_z * width) + grid_x
        data = snapshot["data"]
        if not isinstance(data, tuple) or index < 0 or index >= len(data):
            return None
        return int(data[index])

    @staticmethod
    def _point_in_polygon(
        x_m: float, z_m: float, polygon: list[dict[str, float]]
    ) -> bool:
        inside = False
        count = len(polygon)
        if count < 3:
            return False
        previous = polygon[-1]
        for current in polygon:
            x0 = float(previous["x"])
            z0 = float(previous["z"])
            x1 = float(current["x"])
            z1 = float(current["z"])
            denominator = z1 - z0
            if abs(denominator) < 1e-9:
                previous = current
                continue
            intersects = ((z0 > z_m) != (z1 > z_m)) and (
                x_m < (((x1 - x0) * (z_m - z0)) / denominator) + x0
            )
            if intersects:
                inside = not inside
            previous = current
        return inside

    def _point_hits_hazard_polygon(self, x_m: float, z_m: float) -> bool:
        for polygon in self._fused_hazard_polygons:
            if self._point_in_polygon(x_m, z_m, polygon):
                return True
        return False

    def _segment_is_return_safe(
        self,
        start: dict[str, float],
        end: dict[str, float],
    ) -> tuple[bool, str]:
        snapshot = self._occupancy_map_snapshot
        if snapshot is None:
            return False, "occupancy map unavailable"

        resolution = float(snapshot["resolution"])
        sample_step = max(0.1, resolution * 0.5)
        dx = float(end["x"]) - float(start["x"])
        dz = float(end["z"]) - float(start["z"])
        distance_m = math.hypot(dx, dz)
        sample_count = max(1, int(math.ceil(distance_m / sample_step)))

        for index in range(sample_count + 1):
            ratio = index / sample_count
            sample_x = float(start["x"]) + (dx * ratio)
            sample_z = float(start["z"]) + (dz * ratio)
            if self._point_hits_hazard_polygon(sample_x, sample_z):
                return False, "hazard exclusion intersected"
            cell = self._occupancy_cell_at_local(sample_x, sample_z)
            if cell is None:
                return False, "route exits occupancy map bounds"
            if cell < 0 or cell >= self._OCCUPANCY_BLOCKED_THRESHOLD:
                return False, "occupancy map indicates blocked segment"
        return True, ""

    def _set_return_fallback_reason(self, vehicle_id: str, reason: str) -> None:
        normalized_vehicle_id = self._normalize_vehicle_id(vehicle_id)
        if not normalized_vehicle_id:
            return
        self._return_fallback_reasons[normalized_vehicle_id] = reason
        self._return_last_updated_sec_by_vehicle[normalized_vehicle_id] = self._now_seconds()
        self.get_logger().warning(
            "Return planner fallback for %s: %s" % (normalized_vehicle_id, reason)
        )

    def _clear_return_fallback_reason(self, vehicle_id: str) -> None:
        normalized_vehicle_id = self._normalize_vehicle_id(vehicle_id)
        if not normalized_vehicle_id:
            return
        self._return_fallback_reasons.pop(normalized_vehicle_id, None)

    def _filtered_mission_breadcrumbs_for_return(
        self, vehicle_id: str
    ) -> list[dict[str, float | str]]:
        normalized_vehicle_id = self._normalize_vehicle_id(vehicle_id)
        if not normalized_vehicle_id or not self._mission_id:
            return []

        now_sec = self._now_seconds()
        mission_started_sec = self._mission_started_sec
        filtered: list[dict[str, float | str]] = []
        for breadcrumb in self._scout_vio_breadcrumbs.get(normalized_vehicle_id, []):
            if str(breadcrumb.get("missionId", "")) != self._mission_id:
                continue
            mission_state = str(breadcrumb.get("missionState", "")).upper()
            if mission_state not in self._RETURN_BREADCRUMB_ALLOWED_STATES:
                continue
            try:
                timestamp_sec = float(breadcrumb.get("timestampSec", 0.0))
            except (TypeError, ValueError):
                continue
            if not math.isfinite(timestamp_sec) or timestamp_sec <= 0.0:
                continue
            if mission_started_sec > 0.0 and timestamp_sec + 1e-6 < mission_started_sec:
                continue
            if timestamp_sec > now_sec + 1.0:
                continue
            if (now_sec - timestamp_sec) > self._return_breadcrumb_max_age_sec:
                continue
            filtered.append(breadcrumb)

        filtered.sort(key=lambda entry: float(entry.get("timestampSec", 0.0)))
        return filtered

    def _build_vio_return_waypoints(
        self, vehicle_id: str
    ) -> tuple[list[dict[str, float]] | None, str]:
        normalized_vehicle_id = self._normalize_vehicle_id(vehicle_id)
        if not normalized_vehicle_id:
            return None, "vehicle_id is required"
        if self._map_freshness_age_sec() > self._return_required_freshness_sec:
            return (
                None,
                "occupancy map is stale or unavailable for return planning",
            )
        current_vio_position = self._fresh_vio_position_for_vehicle(
            normalized_vehicle_id, update_status=True
        )
        if current_vio_position is None:
            return None, "VIO odometry is stale or unavailable for return planning"

        mission_breadcrumbs = self._filtered_mission_breadcrumbs_for_return(
            normalized_vehicle_id
        )
        if len(mission_breadcrumbs) < 2:
            return (
                None,
                "insufficient fresh in-mission VIO breadcrumb history to synthesize return path",
            )

        route_points: list[dict[str, float]] = [
            {"x": float(current_vio_position["x"]), "z": float(current_vio_position["z"])}
        ]
        for breadcrumb in reversed(mission_breadcrumbs):
            candidate = {
                "x": float(breadcrumb["x"]),
                "z": float(breadcrumb["z"]),
            }
            previous = route_points[-1]
            if (
                math.hypot(
                    float(previous["x"]) - float(candidate["x"]),
                    float(previous["z"]) - float(candidate["z"]),
                )
                <= 0.001
            ):
                continue
            route_points.append(candidate)
        if len(route_points) < 2:
            return None, "return synthesis produced an empty route"

        for index in range(len(route_points) - 1):
            safe, reason = self._segment_is_return_safe(
                route_points[index],
                route_points[index + 1],
            )
            if not safe:
                return None, reason

        altitude_m = self._scout_altitude_m + self._return_altitude_offset_m
        waypoints = [
            {
                "x": float(point["x"]),
                "z": float(point["z"]),
                "altitude_m": altitude_m,
            }
            for point in route_points
        ]
        return waypoints, ""

    @staticmethod
    def _path_length_m(points: list[dict[str, float]]) -> float:
        total_distance = 0.0
        for index in range(1, len(points)):
            previous = points[index - 1]
            current = points[index]
            total_distance += math.hypot(
                float(current.get("x", 0.0)) - float(previous.get("x", 0.0)),
                float(current.get("z", 0.0)) - float(previous.get("z", 0.0)),
            )
        return total_distance

    def _active_return_trajectory_payload(self) -> dict[str, object] | None:
        candidate_vehicle_ids = sorted(
            {
                vehicle_id
                for vehicle_id, assignment in self._vehicle_assignments.items()
                if assignment == "RETURNING"
            }
            | set(self._return_paths_by_vehicle.keys())
            | set(self._return_fallback_reasons.keys())
        )
        if not candidate_vehicle_ids:
            return None
        selected_vehicle_id = candidate_vehicle_ids[0]
        if self._active_scout_vehicle_id in candidate_vehicle_ids:
            selected_vehicle_id = self._active_scout_vehicle_id

        points = [
            {
                "x": float(point.get("x", 0.0)),
                "z": float(point.get("z", 0.0)),
                "altitude_m": float(point.get("altitude_m", self._scout_altitude_m)),
            }
            for point in self._return_paths_by_vehicle.get(selected_vehicle_id, [])
        ]
        fallback_reason = self._return_fallback_reasons.get(selected_vehicle_id, "")
        state = self._RETURN_TRAJECTORY_STATE_ACTIVE
        if fallback_reason and not points:
            state = self._RETURN_TRAJECTORY_STATE_FALLBACK
        eta_sec = self._path_length_m(points) / max(self._simulated_scout_speed_mps, 0.1)
        last_updated_sec = self._return_last_updated_sec_by_vehicle.get(
            selected_vehicle_id, self._now_seconds()
        )
        payload: dict[str, object] = {
            "vehicleId": selected_vehicle_id,
            "state": state,
            "points": points,
            "etaSec": max(0.0, eta_sec),
            "lastUpdatedSec": float(last_updated_sec),
        }
        if fallback_reason:
            payload["fallbackReason"] = fallback_reason
        return payload

    def _handle_scout_odometry(self, vehicle_id: str, message: Odometry) -> None:
        normalized_vehicle_id = self._normalize_vehicle_id(vehicle_id)
        x_m = float(message.pose.pose.position.x)
        y_m = float(message.pose.pose.position.y)
        if not math.isfinite(x_m) or not math.isfinite(y_m):
            self._set_vio_status(normalized_vehicle_id, "invalid pose values")
            return
        # Mission search/tracking geometry stores planar coordinates as x/z.
        # ROS odometry publishes horizontal coordinates as x/y, so y maps to z.
        self._scout_vio_position_snapshot[normalized_vehicle_id] = {
            "x": x_m,
            "z": y_m,
        }
        self._scout_vio_last_seen_monotonic[normalized_vehicle_id] = time.monotonic()
        timestamp_sec = self._stamp_to_seconds(message.header.stamp)
        if timestamp_sec <= 0.0:
            timestamp_sec = self._now_seconds()
        self._capture_vio_breadcrumb(
            normalized_vehicle_id,
            x_m=x_m,
            z_m=y_m,
            timestamp_sec=timestamp_sec,
        )
        self._set_vio_status(normalized_vehicle_id, "")

    def _fresh_vio_position_for_vehicle(
        self, vehicle_id: str, *, update_status: bool = True
    ) -> dict[str, float] | None:
        normalized_vehicle_id = self._normalize_vehicle_id(vehicle_id)
        snapshot = self._scout_vio_position_snapshot.get(normalized_vehicle_id)
        if snapshot is None:
            if update_status:
                self._set_vio_status(normalized_vehicle_id, "no odometry samples received")
            return None
        last_seen = self._scout_vio_last_seen_monotonic.get(normalized_vehicle_id)
        if last_seen is None:
            if update_status:
                self._set_vio_status(normalized_vehicle_id, "odometry timestamp unavailable")
            return None
        age_sec = time.monotonic() - last_seen
        if age_sec > self._vio_odom_stale_sec:
            if update_status:
                self._set_vio_status(
                    normalized_vehicle_id,
                    f"stale odometry (window {self._vio_odom_stale_sec:.3f}s)",
                )
            return None
        if update_status:
            self._set_vio_status(normalized_vehicle_id, "")
        return {"x": float(snapshot["x"]), "z": float(snapshot["z"])}

    def _resolve_position_source_for_vehicle(
        self, vehicle_id: str, *, for_selection: bool = False
    ) -> tuple[str, dict[str, float] | None]:
        if self._navigation_position_source == self._POSITION_SOURCE_TELEMETRY:
            return self._POSITION_SOURCE_TELEMETRY, None
        if self._navigation_position_source == self._POSITION_SOURCE_VIO:
            return self._POSITION_SOURCE_VIO, self._fresh_vio_position_for_vehicle(
                vehicle_id, update_status=not for_selection
            )

        # AUTO mode: prefer fresh VIO when available, otherwise telemetry unless
        # GPS-denied mode explicitly requires VIO.
        vio_position = self._fresh_vio_position_for_vehicle(
            vehicle_id, update_status=not for_selection
        )
        if vio_position is not None:
            return self._POSITION_SOURCE_VIO, vio_position
        if self._gps_denied_mode:
            return self._POSITION_SOURCE_VIO, None
        return self._POSITION_SOURCE_TELEMETRY, None

    def _normalize_slam_mode(self, value: str) -> str:
        raw = str(value).strip()
        normalized = raw.lower().replace("-", "_")
        alias_target = self._SLAM_MODE_ALIASES.get(normalized)
        candidate = alias_target if alias_target is not None else normalized
        if candidate in self._SUPPORTED_SLAM_MODES:
            if alias_target is not None:
                self.get_logger().warning(
                    f"Legacy slam_mode '{value}' detected; normalizing to '{candidate}'."
                )
            return candidate
        self.get_logger().warning(
            f"Unsupported slam_mode '{value}'; using '{self._SLAM_MODE_UNKNOWN}'."
        )
        return self._SLAM_MODE_UNKNOWN

    def _vehicle_position_source_snapshot(self) -> dict[str, str]:
        sources: dict[str, str] = {}
        for endpoint in self._scout_endpoints:
            vehicle_id = endpoint.vehicle_id
            source = self._vehicle_position_sources.get(vehicle_id)
            if source is None:
                source = self._navigation_position_source
            sources[vehicle_id] = source
        return dict(sorted(sources.items()))

    def _vehicle_slam_mode_snapshot(self) -> dict[str, str]:
        slam_modes: dict[str, str] = {}

        for endpoint in self._scout_endpoints:
            slam_modes[endpoint.vehicle_id] = self._slam_mode
        for vehicle_id in self._scout_plans.keys():
            slam_modes[vehicle_id] = self._slam_mode
        if self._active_scout_vehicle_id:
            slam_modes[self._active_scout_vehicle_id] = self._slam_mode

        for endpoint in self._ranger_endpoints:
            slam_modes.setdefault(endpoint.vehicle_id, self._SLAM_MODE_UNKNOWN)
        if self._active_ranger_vehicle_id:
            slam_modes.setdefault(self._active_ranger_vehicle_id, self._SLAM_MODE_UNKNOWN)

        return dict(sorted(slam_modes.items()))

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
                        try:
                            command_port = int(item.get("command_port", port))
                        except (TypeError, ValueError):
                            continue
                        try:
                            target_system = int(item.get("target_system", 1))
                        except (TypeError, ValueError):
                            target_system = 1
                        try:
                            target_component = int(item.get("target_component", 1))
                        except (TypeError, ValueError):
                            target_component = 1
                        if not vehicle_id_raw or port <= 0:
                            continue
                        endpoints.append(
                            ScoutEndpoint(
                                vehicle_id=self._normalize_vehicle_id(vehicle_id_raw),
                                host=host,
                                port=port,
                                command_port=command_port if command_port > 0 else port,
                                target_system=target_system if target_system > 0 else 1,
                                target_component=(
                                    target_component if target_component > 0 else 1
                                ),
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
                    command_port=self._scout_mavlink_udp_port,
                    target_system=1,
                    target_component=1,
                )
            )
        return endpoints

    def _parse_ranger_endpoints(self) -> list[ScoutEndpoint]:
        endpoints: list[ScoutEndpoint] = []
        raw_json = self._ranger_endpoints_json.strip()
        if not raw_json:
            return endpoints
        try:
            payload = json.loads(raw_json)
        except json.JSONDecodeError:
            self.get_logger().warning(
                "Invalid ranger_endpoints_json; ranger overwatch will remain telemetry-only"
            )
            return endpoints
        if not isinstance(payload, list):
            return endpoints
        for item in payload:
            if not isinstance(item, dict):
                continue
            vehicle_id_raw = str(item.get("vehicle_id", "")).strip()
            host = str(item.get("host", "127.0.0.1")).strip() or "127.0.0.1"
            try:
                port = int(item.get("port", 0))
            except (TypeError, ValueError):
                continue
            try:
                command_port = int(item.get("command_port", port))
            except (TypeError, ValueError):
                command_port = port
            try:
                target_system = int(item.get("target_system", 1))
            except (TypeError, ValueError):
                target_system = 1
            try:
                target_component = int(item.get("target_component", 1))
            except (TypeError, ValueError):
                target_component = 1
            if not vehicle_id_raw or port <= 0:
                continue
            endpoints.append(
                ScoutEndpoint(
                    vehicle_id=self._normalize_vehicle_id(vehicle_id_raw),
                    host=host,
                    port=port,
                    command_port=command_port if command_port > 0 else port,
                    target_system=target_system if target_system > 0 else 1,
                    target_component=target_component if target_component > 0 else 1,
                )
            )
        return endpoints

    def _handle_vehicle_telemetry(self, message: Telemetry) -> None:
        vehicle_type = str(message.vehicle_type).strip().lower()
        vehicle_id = self._normalize_vehicle_id(message.vehicle_id)
        if not vehicle_id:
            return
        is_scout = vehicle_type == "scout" or vehicle_id.startswith("scout")
        is_ranger = vehicle_type == "ranger" or vehicle_id.startswith("ranger")
        if vehicle_type and not is_scout and not is_ranger:
            return

        latitude = float(message.position.latitude)
        longitude = float(message.position.longitude)

        now_monotonic = time.monotonic()
        if is_scout:
            self._scout_last_seen_monotonic[vehicle_id] = now_monotonic
        elif is_ranger:
            self._ranger_last_seen_monotonic[vehicle_id] = now_monotonic
        if math.isfinite(latitude) and math.isfinite(longitude):
            if is_scout:
                self._scout_geodetic_snapshot[vehicle_id] = {
                    "latitude": latitude,
                    "longitude": longitude,
                }
            elif is_ranger:
                self._ranger_geodetic_snapshot[vehicle_id] = {
                    "latitude": latitude,
                    "longitude": longitude,
                }
        local_position = self._telemetry_to_local_position(latitude, longitude)
        if is_scout:
            self._scout_position_snapshot[vehicle_id] = local_position
        elif is_ranger:
            self._ranger_position_snapshot[vehicle_id] = local_position
        if not self._mission_id:
            return

        if is_ranger:
            if self._active_ranger_vehicle_id and vehicle_id == self._active_ranger_vehicle_id:
                self._set_scout_assignment(vehicle_id, "OVERWATCH", label="OVERWATCH")
            else:
                self._set_scout_assignment(vehicle_id, "IDLE", label="IDLE")
            return

        if vehicle_id in self._vehicle_assignments:
            return
        if (
            self._tracking_context.active
            and vehicle_id == self._tracking_context.assigned_scout_vehicle_id
        ):
            self._set_scout_assignment(vehicle_id, "TRACKING")
            return
        if vehicle_id in self._scout_plans:
            label = f"SEARCHING:{self._scout_plans[vehicle_id].zone_id}"
            self._set_scout_assignment(vehicle_id, "SEARCHING", label=label)
        else:
            self._set_scout_assignment(vehicle_id, "IDLE", label="IDLE")

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
            source_mode, vio_snapshot = self._resolve_position_source_for_vehicle(
                self._active_scout_vehicle_id
            )
            if source_mode == self._POSITION_SOURCE_VIO:
                if vio_snapshot is not None:
                    return {"x": vio_snapshot["x"], "z": vio_snapshot["z"]}
            else:
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

    def _online_ranger_endpoints(self) -> list[ScoutEndpoint]:
        now = time.monotonic()
        online_window = max(self._scout_online_window_sec, 0.0)
        online: list[ScoutEndpoint] = []
        for endpoint in self._ranger_endpoints:
            last_seen = self._ranger_last_seen_monotonic.get(endpoint.vehicle_id)
            if last_seen is None:
                continue
            if now - last_seen <= online_window:
                online.append(endpoint)
        return online

    def _online_ranger_vehicle_ids(self) -> list[str]:
        online_ids = {endpoint.vehicle_id for endpoint in self._online_ranger_endpoints()}
        now = time.monotonic()
        online_window = max(self._scout_online_window_sec, 0.0)
        for vehicle_id, last_seen in self._ranger_last_seen_monotonic.items():
            if now - last_seen <= online_window:
                online_ids.add(vehicle_id)
        return sorted(online_ids)

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
            position_source, vio_position = self._resolve_position_source_for_vehicle(
                endpoint.vehicle_id, for_selection=True
            )
            if position_source == self._POSITION_SOURCE_VIO:
                last_seen = self._scout_vio_last_seen_monotonic.get(
                    endpoint.vehicle_id, -math.inf
                )
                position = vio_position
            else:
                last_seen = self._scout_last_seen_monotonic.get(
                    endpoint.vehicle_id, -math.inf
                )
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
        if not self._scout_endpoints and not self._ranger_endpoints:
            return []

        now = time.monotonic()
        online_window = max(self._scout_online_window_sec, 0.0)
        selected: list[ScoutEndpoint] = []
        selected_ids: set[str] = set()

        for endpoint in [*self._scout_endpoints, *self._ranger_endpoints]:
            last_seen = self._scout_last_seen_monotonic.get(endpoint.vehicle_id)
            if last_seen is None:
                last_seen = self._ranger_last_seen_monotonic.get(endpoint.vehicle_id)
            if last_seen is None:
                continue
            if now - last_seen <= online_window:
                selected.append(endpoint)
                selected_ids.add(endpoint.vehicle_id)

        if self._active_scout_vehicle_id:
            for endpoint in [*self._scout_endpoints, *self._ranger_endpoints]:
                if endpoint.vehicle_id != self._active_scout_vehicle_id:
                    continue
                if endpoint.vehicle_id not in selected_ids:
                    selected.append(endpoint)
                    selected_ids.add(endpoint.vehicle_id)
                break

        if selected:
            return selected
        fallback = [*self._scout_endpoints, *self._ranger_endpoints]
        deduped: dict[str, ScoutEndpoint] = {}
        for endpoint in fallback:
            deduped[endpoint.vehicle_id] = endpoint
        return list(deduped.values())

    def _reset_vehicle_command_states(self) -> None:
        all_vehicle_ids = {
            endpoint.vehicle_id for endpoint in self._scout_endpoints
        } | {
            endpoint.vehicle_id for endpoint in self._ranger_endpoints
        }
        self._vehicle_command_states = {
            vehicle_id: self._VEHICLE_COMMAND_STATE_ACTIVE
            for vehicle_id in sorted(all_vehicle_ids)
        }

    def _next_vehicle_command_state(self, command: str) -> str:
        if command == "HOLD":
            return self._VEHICLE_COMMAND_STATE_HOLDING
        if command == "RESUME":
            return self._VEHICLE_COMMAND_STATE_ACTIVE
        if command == "RECALL":
            return self._VEHICLE_COMMAND_STATE_RETURNING
        return self._VEHICLE_COMMAND_STATE_ACTIVE

    def _validate_vehicle_command_transition(
        self, *, vehicle_id: str, command: str
    ) -> tuple[bool, str]:
        current_state = self._vehicle_command_states.get(
            vehicle_id, self._VEHICLE_COMMAND_STATE_ACTIVE
        )
        allowed_by_command: dict[str, set[str]] = {
            "HOLD": {self._VEHICLE_COMMAND_STATE_ACTIVE},
            "RESUME": {self._VEHICLE_COMMAND_STATE_HOLDING},
            "RECALL": {
                self._VEHICLE_COMMAND_STATE_ACTIVE,
                self._VEHICLE_COMMAND_STATE_HOLDING,
            },
        }
        allowed_states = allowed_by_command.get(command, set())
        if current_state in allowed_states:
            return True, ""
        allowed_str = ", ".join(sorted(allowed_states)) if allowed_states else "none"
        return (
            False,
            (
                f"vehicle_command '{command}' rejected for '{vehicle_id}': invalid transition "
                f"from {current_state}; allowed from {allowed_str}"
            ),
        )

    def _dispatch_abort_rtl(self, endpoints: list[ScoutEndpoint]) -> tuple[int, int]:
        success_count = 0
        total_count = len(endpoints)

        for endpoint in endpoints:
            try:
                self._apply_mavlink_endpoint(endpoint)
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
        scout_endpoint = self._resolve_scout_endpoint(vehicle_id)
        if scout_endpoint is not None:
            return scout_endpoint
        return self._resolve_ranger_endpoint(vehicle_id)

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
            command_port=endpoint.command_port if endpoint.command_port > 0 else endpoint.port,
            stream_hz=self._setpoint_stream_hz,
            target_system=endpoint.target_system,
            target_component=endpoint.target_component,
            logger=lambda message: self.get_logger().info(message),
        )

    def _build_ranger_adapter(self, endpoint: ScoutEndpoint) -> MavlinkAdapter:
        return MavlinkAdapter(
            host=endpoint.host,
            port=endpoint.port,
            command_port=endpoint.command_port if endpoint.command_port > 0 else endpoint.port,
            stream_hz=self._setpoint_stream_hz,
            target_system=endpoint.target_system,
            target_component=endpoint.target_component,
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

        selected_scout_previous_assignment = self._vehicle_assignments.get(
            selected_endpoint.vehicle_id, "SEARCHING"
        )
        selected_scout_previous_plan = self._scout_plans.get(selected_endpoint.vehicle_id)
        selected_scout_previous_plan_clone = (
            self._clone_plan_state(selected_scout_previous_plan)
            if selected_scout_previous_plan is not None
            else None
        )
        preserved_non_target_vehicle_ids = sorted(
            vehicle_id
            for vehicle_id, assignment in self._vehicle_assignments.items()
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
            self._apply_mavlink_endpoint(selected_endpoint)

        self._last_detection_accept_monotonic = accepted_monotonic
        self._last_detection_event = event
        self._last_detection_rejection_reason = ""

        self._active_scout_vehicle_id = selected_endpoint.vehicle_id
        self._set_scout_assignment(
            selected_endpoint.vehicle_id, "TRACKING", label="TRACKING"
        )
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
            selected_scout_previous_plan=selected_scout_previous_plan_clone,
            previous_active_scout_vehicle_id=previous_scout_vehicle_id,
            selected_scout_previous_assignment=selected_scout_previous_assignment,
            preserved_non_target_vehicle_ids=preserved_non_target_vehicle_ids,
            uses_dedicated_adapter=use_dedicated_adapter,
        )

        # State transition precedes latency measurement to include the first MAVLink
        # setpoint dispatch in the tracking response time budget.
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
        selected_scout_previous_plan = self._tracking_context.selected_scout_previous_plan
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
            if selected_scout_previous_plan is not None:
                self._scout_plans[assigned_scout_vehicle_id] = self._clone_plan_state(
                    selected_scout_previous_plan
                )
            restored_label = selected_scout_previous_assignment
            if (
                selected_scout_previous_assignment == "SEARCHING"
                and assigned_scout_vehicle_id in self._scout_plans
            ):
                restored_label = (
                    f"SEARCHING:{self._scout_plans[assigned_scout_vehicle_id].zone_id}"
                )
            self._set_scout_assignment(
                assigned_scout_vehicle_id,
                selected_scout_previous_assignment,
                label=restored_label,
            )

        if previous_scout_vehicle_id:
            self._active_scout_vehicle_id = previous_scout_vehicle_id
            if previous_scout_vehicle_id in self._scout_plans:
                self._plan_state = self._scout_plans[previous_scout_vehicle_id]
            endpoint = self._select_endpoint_by_vehicle_id(previous_scout_vehicle_id)
            if endpoint is not None:
                try:
                    self._apply_mavlink_endpoint(endpoint)
                except OSError as error:
                    self.get_logger().warning(
                        "Failed to restore scout endpoint after tracking complete: "
                        f"{endpoint.vehicle_id} ({endpoint.host}:{endpoint.port}): {error}"
                    )
        if self._state != "SEARCHING":
            self._transition_to("SEARCHING")
        else:
            self._start_autonomous_execution()

    def _resolve_scout_endpoint(self, vehicle_id: str) -> ScoutEndpoint | None:
        normalized_vehicle_id = self._normalize_vehicle_id(vehicle_id)
        for endpoint in self._scout_endpoints:
            if endpoint.vehicle_id == normalized_vehicle_id:
                return endpoint
        return None

    def _resolve_ranger_endpoint(self, vehicle_id: str) -> ScoutEndpoint | None:
        normalized_vehicle_id = self._normalize_vehicle_id(vehicle_id)
        for endpoint in self._ranger_endpoints:
            if endpoint.vehicle_id == normalized_vehicle_id:
                return endpoint
        return None

    def _is_endpoint_online(self, endpoint: ScoutEndpoint) -> bool:
        if self._scout_online_window_sec < 0:
            return False
        last_seen = self._scout_last_seen_monotonic.get(endpoint.vehicle_id)
        if last_seen is None:
            last_seen = self._ranger_last_seen_monotonic.get(endpoint.vehicle_id)
        if last_seen is None:
            return False
        return (time.monotonic() - last_seen) <= self._scout_online_window_sec

    def _current_vehicle_local_position(self, vehicle_id: str) -> dict[str, float] | None:
        normalized_vehicle_id = self._normalize_vehicle_id(vehicle_id)
        if not normalized_vehicle_id:
            return None
        fresh_vio = self._fresh_vio_position_for_vehicle(
            normalized_vehicle_id, update_status=False
        )
        if fresh_vio is not None:
            return {"x": float(fresh_vio["x"]), "z": float(fresh_vio["z"])}
        plan = self._scout_plans.get(normalized_vehicle_id)
        if plan is not None:
            return {
                "x": float(plan.scout_position.get("x", 0.0)),
                "z": float(plan.scout_position.get("z", 0.0)),
            }
        snapshot = self._scout_position_snapshot.get(normalized_vehicle_id)
        if snapshot is None:
            return None
        return {"x": float(snapshot["x"]), "z": float(snapshot["z"])}

    def _current_vehicle_altitude_m(self, vehicle_id: str) -> float:
        normalized_vehicle_id = self._normalize_vehicle_id(vehicle_id)
        if normalized_vehicle_id in self._return_paths_by_vehicle:
            waypoints = self._return_paths_by_vehicle.get(normalized_vehicle_id, [])
            default_altitude = self._scout_altitude_m + self._return_altitude_offset_m
            if not waypoints:
                return default_altitude
            current_index = self._return_path_index_by_vehicle.get(normalized_vehicle_id, 0)
            if current_index < 0:
                current_index = 0
            if current_index >= len(waypoints):
                current_index = len(waypoints) - 1
            return float(waypoints[current_index].get("altitude_m", default_altitude))
        plan = self._scout_plans.get(normalized_vehicle_id)
        if plan is not None:
            waypoint = self._current_waypoint_for_plan(plan)
            if waypoint is not None:
                return float(waypoint.get("altitude_m", self._scout_altitude_m))
        return self._scout_altitude_m

    def _apply_returning_separation_mitigation(
        self,
        *,
        returning_vehicle_id: str,
        searching_vehicle_id: str,
        returning_position: dict[str, float],
        searching_position: dict[str, float],
        searching_altitude_m: float,
    ) -> None:
        waypoints = self._return_paths_by_vehicle.get(returning_vehicle_id)
        if not waypoints:
            return
        current_index = self._return_path_index_by_vehicle.get(returning_vehicle_id, 0)
        if current_index < 0:
            current_index = 0
        if current_index >= len(waypoints):
            return

        away_x = float(returning_position["x"]) - float(searching_position["x"])
        away_z = float(returning_position["z"]) - float(searching_position["z"])
        norm = math.hypot(away_x, away_z)
        if norm < 1e-6:
            away_x, away_z, norm = 1.0, 0.0, 1.0
        unit_x = away_x / norm
        unit_z = away_z / norm
        target_horizontal_m = self._return_separation_horizontal_m + 1.0
        candidate_x = float(searching_position["x"]) + (unit_x * target_horizontal_m)
        candidate_z = float(searching_position["z"]) + (unit_z * target_horizontal_m)
        current_target = waypoints[current_index]
        candidate_altitude_m = max(
            float(current_target.get("altitude_m", self._scout_altitude_m)),
            float(searching_altitude_m) + self._return_separation_vertical_m + 0.5,
        )
        candidate_waypoint = {
            "x": candidate_x,
            "z": candidate_z,
            "altitude_m": candidate_altitude_m,
        }

        # Skip duplicate mitigation insertions when the current target already
        # provides adequate separation to prevent waypoint list bloat.
        if (
            math.hypot(
                float(current_target.get("x", 0.0)) - candidate_x,
                float(current_target.get("z", 0.0)) - candidate_z,
            )
            <= 0.25
            and abs(
                float(current_target.get("altitude_m", candidate_altitude_m))
                - candidate_altitude_m
            )
            <= 0.25
        ):
            return

        safe, reason = self._segment_is_return_safe(
            {
                "x": float(returning_position["x"]),
                "z": float(returning_position["z"]),
            },
            candidate_waypoint,
        )
        if not safe:
            self.get_logger().warning(
                "Unable to apply RETURNING separation mitigation waypoint for "
                f"{returning_vehicle_id} vs {searching_vehicle_id}: {reason}"
            )
            return

        waypoints.insert(current_index, candidate_waypoint)
        self._return_path_index_by_vehicle[returning_vehicle_id] = current_index
        self._return_last_updated_sec_by_vehicle[returning_vehicle_id] = self._now_seconds()
        if (
            self._active_scout_vehicle_id == returning_vehicle_id
            and self._mavlink_adapter.is_streaming
        ):
            self._mavlink_adapter.update_setpoint(dict(candidate_waypoint))
        self.get_logger().warning(
            "Applied RETURNING separation mitigation for "
            f"{returning_vehicle_id} away from {searching_vehicle_id} "
            f"toward ({candidate_x:.2f}, {candidate_z:.2f}, {candidate_altitude_m:.2f}m)"
        )

    def _enforce_mixed_returning_separation(self) -> None:
        returning_vehicle_ids = sorted(
            vehicle_id
            for vehicle_id, assignment in self._vehicle_assignments.items()
            if assignment == "RETURNING"
        )
        searching_vehicle_ids = sorted(
            vehicle_id
            for vehicle_id, assignment in self._vehicle_assignments.items()
            if assignment == "SEARCHING"
        )
        if not returning_vehicle_ids or not searching_vehicle_ids:
            return

        for returning_vehicle_id in returning_vehicle_ids:
            returning_position = self._current_vehicle_local_position(returning_vehicle_id)
            if returning_position is None:
                continue
            returning_altitude = self._current_vehicle_altitude_m(returning_vehicle_id)
            for searching_vehicle_id in searching_vehicle_ids:
                searching_position = self._current_vehicle_local_position(searching_vehicle_id)
                if searching_position is None:
                    continue
                searching_altitude = self._current_vehicle_altitude_m(searching_vehicle_id)
                horizontal_distance_m = math.hypot(
                    float(returning_position["x"]) - float(searching_position["x"]),
                    float(returning_position["z"]) - float(searching_position["z"]),
                )
                vertical_distance_m = abs(returning_altitude - searching_altitude)
                if (
                    horizontal_distance_m < self._return_separation_horizontal_m
                    and vertical_distance_m < self._return_separation_vertical_m
                ):
                    self.get_logger().warning(
                        "Mixed RETURNING/SEARCHING separation warning: "
                        f"{returning_vehicle_id} vs {searching_vehicle_id} "
                        f"(horizontal={horizontal_distance_m:.2f}m, "
                        f"vertical={vertical_distance_m:.2f}m)"
                    )
                    self._apply_returning_separation_mitigation(
                        returning_vehicle_id=returning_vehicle_id,
                        searching_vehicle_id=searching_vehicle_id,
                        returning_position=returning_position,
                        searching_position=searching_position,
                        searching_altitude_m=searching_altitude,
                    )

    def _dispatch_return_to_launch_for_endpoint(
        self, endpoint: ScoutEndpoint
    ) -> tuple[bool, str]:
        previous_host, previous_port = self._mavlink_adapter.endpoint
        previous_command_port = self._mavlink_adapter.command_port()
        previous_target_system, previous_target_component = self._mavlink_adapter.target
        restore_host, restore_port = previous_host, previous_port
        restore_command_port = previous_command_port
        restore_target_system, restore_target_component = (
            previous_target_system,
            previous_target_component,
        )
        if self._active_scout_vehicle_id:
            active_endpoint = self._resolve_scout_endpoint(self._active_scout_vehicle_id)
            if active_endpoint is not None:
                restore_host, restore_port = active_endpoint.host, active_endpoint.port
                restore_command_port = active_endpoint.command_port
                restore_target_system = active_endpoint.target_system
                restore_target_component = active_endpoint.target_component

        should_pause_stream = self._mavlink_adapter.is_streaming and (
            (endpoint.host, endpoint.port, endpoint.command_port)
            != (restore_host, restore_port, restore_command_port)
            or (endpoint.target_system, endpoint.target_component)
            != (restore_target_system, restore_target_component)
        )
        resume_setpoint: dict[str, float] | None = None
        if should_pause_stream:
            current_waypoint = self._current_waypoint()
            if current_waypoint is not None:
                resume_setpoint = dict(current_waypoint)
            self._mavlink_adapter.stop_stream()

        try:
            self._apply_mavlink_endpoint(endpoint)
        except OSError as error:
            return (
                False,
                f"px4 native RTL fallback dispatch failed for '{endpoint.vehicle_id}': {error}",
            )

        command_sent = False
        restore_error = ""
        try:
            command_sent = self._mavlink_adapter.send_return_to_launch()
        finally:
            if (
                (endpoint.host, endpoint.port, endpoint.command_port)
                != (restore_host, restore_port, restore_command_port)
                or (endpoint.target_system, endpoint.target_component)
                != (restore_target_system, restore_target_component)
            ):
                try:
                    if restore_command_port == restore_port:
                        self._mavlink_adapter.set_endpoint(restore_host, restore_port)
                    else:
                        self._mavlink_adapter.set_endpoint(
                            restore_host,
                            restore_port,
                            command_port=restore_command_port,
                        )
                    self._mavlink_adapter.set_target(
                        restore_target_system, restore_target_component
                    )
                except OSError as error:
                    restore_error = (
                        "px4 native RTL fallback failed to restore active endpoint "
                        f"{restore_host}:{restore_port} "
                        f"(command_port={restore_command_port}, "
                        f"target={restore_target_system}/{restore_target_component}): {error}"
                    )
            if (
                not restore_error
                and should_pause_stream
                and resume_setpoint is not None
                and self._state in self._AUTONOMOUS_STATES
            ):
                try:
                    self._mavlink_adapter.start_stream(self._mission_id, resume_setpoint)
                except OSError as error:
                    restore_error = (
                        "px4 native RTL fallback failed to resume mission streaming "
                        f"for '{self._mission_id}': {error}"
                    )

        if restore_error:
            return False, restore_error
        if not command_sent:
            return (
                False,
                f"px4 native RTL fallback for '{endpoint.vehicle_id}' was not acknowledged",
            )
        return True, ""

    def _activate_px4_return_fallback(
        self, vehicle_id: str, reason: str
    ) -> tuple[bool, str]:
        normalized_vehicle_id = self._normalize_vehicle_id(vehicle_id)
        if not normalized_vehicle_id:
            return False, "vehicle_id is required for px4 fallback"
        self._set_return_fallback_reason(normalized_vehicle_id, reason)
        self._set_scout_assignment(
            normalized_vehicle_id,
            "RETURNING",
            label="RETURNING:fallback:px4-native-rtl",
        )
        self._return_paths_by_vehicle.pop(normalized_vehicle_id, None)
        self._return_path_index_by_vehicle.pop(normalized_vehicle_id, None)
        self._return_last_updated_sec_by_vehicle[normalized_vehicle_id] = self._now_seconds()
        endpoint = self._resolve_scout_endpoint(normalized_vehicle_id)
        if endpoint is None:
            return (
                False,
                f"px4 native RTL fallback failed for '{normalized_vehicle_id}': unknown endpoint",
            )
        return self._dispatch_return_to_launch_for_endpoint(endpoint)

    def _start_vio_return_execution(
        self, endpoint: ScoutEndpoint
    ) -> tuple[bool, str]:
        normalized_vehicle_id = self._normalize_vehicle_id(endpoint.vehicle_id)
        waypoints, planning_error = self._build_vio_return_waypoints(normalized_vehicle_id)
        if waypoints is None:
            sent, fallback_error = self._activate_px4_return_fallback(
                normalized_vehicle_id, planning_error
            )
            if not sent:
                return False, fallback_error
            return True, ""

        self._clear_return_fallback_reason(normalized_vehicle_id)
        self._return_paths_by_vehicle[normalized_vehicle_id] = [dict(point) for point in waypoints]
        self._return_path_index_by_vehicle[normalized_vehicle_id] = 0
        self._return_last_updated_sec_by_vehicle[normalized_vehicle_id] = self._now_seconds()
        self._set_scout_assignment(
            normalized_vehicle_id,
            "RETURNING",
            label=f"RETURNING:{normalized_vehicle_id}",
        )
        return_target_plan = self._scout_plans.get(normalized_vehicle_id)
        if return_target_plan is not None:
            return_target_plan.current_waypoint_index = len(return_target_plan.waypoints)

        if self._active_scout_vehicle_id != normalized_vehicle_id:
            self._active_scout_vehicle_id = normalized_vehicle_id
        if self._mavlink_adapter.is_streaming:
            self._mavlink_adapter.stop_stream()
        try:
            self._apply_mavlink_endpoint(endpoint)
        except OSError as error:
            return (
                False,
                f"vehicle_command dispatch failed for '{endpoint.vehicle_id}': {error}",
            )

        self._mavlink_adapter.upload_mission_items_int(self._mission_id, waypoints)
        first_setpoint = dict(waypoints[0])
        if self._mavlink_adapter.is_streaming:
            self._mavlink_adapter.update_setpoint(first_setpoint)
        else:
            self._mavlink_adapter.start_stream(self._mission_id, first_setpoint)
        return True, ""

    def _advance_return_plan_for_vehicle(self, vehicle_id: str) -> None:
        normalized_vehicle_id = self._normalize_vehicle_id(vehicle_id)
        waypoints = self._return_paths_by_vehicle.get(normalized_vehicle_id)
        if not waypoints:
            return
        current_index = self._return_path_index_by_vehicle.get(normalized_vehicle_id, 0)
        if current_index < 0:
            current_index = 0
        if current_index >= len(waypoints):
            self._set_scout_assignment(
                normalized_vehicle_id,
                "IDLE",
                label="IDLE:return-complete",
            )
            self._return_paths_by_vehicle.pop(normalized_vehicle_id, None)
            self._return_path_index_by_vehicle.pop(normalized_vehicle_id, None)
            self._return_last_updated_sec_by_vehicle[normalized_vehicle_id] = (
                self._now_seconds()
            )
            return

        if self._map_freshness_age_sec() > self._return_required_freshness_sec:
            sent, fallback_error = self._activate_px4_return_fallback(
                normalized_vehicle_id,
                "occupancy map became stale during return execution",
            )
            if not sent:
                self.get_logger().warning(fallback_error)
            return

        source_mode, vio_position = self._resolve_position_source_for_vehicle(normalized_vehicle_id)
        if source_mode == self._POSITION_SOURCE_VIO:
            if vio_position is None:
                sent, fallback_error = self._activate_px4_return_fallback(
                    normalized_vehicle_id,
                    "VIO odometry became unavailable during return execution",
                )
                if not sent:
                    self.get_logger().warning(fallback_error)
                return
            position = {"x": float(vio_position["x"]), "z": float(vio_position["z"])}
        else:
            plan = self._scout_plans.get(normalized_vehicle_id)
            if plan is None:
                return
            position = {
                "x": float(plan.scout_position.get("x", 0.0)),
                "z": float(plan.scout_position.get("z", 0.0)),
            }
        target = waypoints[current_index]
        distance_m = math.hypot(
            float(target["x"]) - float(position["x"]),
            float(target["z"]) - float(position["z"]),
        )
        if distance_m <= self._waypoint_reached_threshold_m:
            next_index = current_index + 1
            self._return_path_index_by_vehicle[normalized_vehicle_id] = next_index
            self._return_last_updated_sec_by_vehicle[normalized_vehicle_id] = (
                self._now_seconds()
            )
            if (
                self._active_scout_vehicle_id == normalized_vehicle_id
                and next_index < len(waypoints)
                and self._mavlink_adapter.is_streaming
            ):
                self._mavlink_adapter.update_setpoint(dict(waypoints[next_index]))
            return

        if source_mode == self._POSITION_SOURCE_VIO:
            return

        # Telemetry-driven sim fallback path for non-VIO environments.
        plan = self._scout_plans.get(normalized_vehicle_id)
        if plan is None:
            return
        step = self._simulated_scout_speed_mps * self._control_loop_period
        if step <= 0.0:
            return
        delta_x = float(target["x"]) - float(position["x"])
        delta_z = float(target["z"]) - float(position["z"])
        if distance_m <= step:
            plan.scout_position["x"] = float(target["x"])
            plan.scout_position["z"] = float(target["z"])
        else:
            ratio = step / distance_m
            plan.scout_position["x"] = float(position["x"]) + (delta_x * ratio)
            plan.scout_position["z"] = float(position["z"]) + (delta_z * ratio)

    def _dispatch_vehicle_command(
        self, endpoint: ScoutEndpoint, command: str
    ) -> tuple[bool, str]:
        if command == "RECALL" and self._gps_denied_mode:
            return self._start_vio_return_execution(endpoint)

        dispatchers: dict[str, Callable[[], bool]] = {
            "HOLD": self._mavlink_adapter.send_hold_position,
            "RESUME": self._mavlink_adapter.send_resume_mission,
            "RECALL": self._mavlink_adapter.send_return_to_launch,
        }
        dispatch = dispatchers.get(command)
        if dispatch is None:
            return False, f"unsupported command '{command}'"

        previous_host, previous_port = self._mavlink_adapter.endpoint
        previous_command_port = self._mavlink_adapter.command_port
        previous_target_system, previous_target_component = self._mavlink_adapter.target
        restore_host, restore_port = previous_host, previous_port
        restore_command_port = previous_command_port
        restore_target_system, restore_target_component = (
            previous_target_system,
            previous_target_component,
        )
        if self._active_scout_vehicle_id:
            active_endpoint = self._resolve_scout_endpoint(self._active_scout_vehicle_id)
            if active_endpoint is not None:
                restore_host, restore_port = active_endpoint.host, active_endpoint.port
                restore_command_port = active_endpoint.command_port
                restore_target_system = active_endpoint.target_system
                restore_target_component = active_endpoint.target_component

        should_pause_stream = self._mavlink_adapter.is_streaming and (
            (endpoint.host, endpoint.port, endpoint.command_port)
            != (restore_host, restore_port, restore_command_port)
            or (endpoint.target_system, endpoint.target_component)
            != (restore_target_system, restore_target_component)
        )
        resume_setpoint: dict[str, float] | None = None
        if should_pause_stream:
            current_waypoint = self._current_waypoint()
            if current_waypoint is not None:
                resume_setpoint = dict(current_waypoint)
            self._mavlink_adapter.stop_stream()

        try:
            self._apply_mavlink_endpoint(endpoint)
        except OSError as error:
            return (
                False,
                f"vehicle_command dispatch failed for '{endpoint.vehicle_id}': {error}",
            )

        command_sent = False
        restore_error = ""
        try:
            command_sent = dispatch()
        finally:
            if (
                (endpoint.host, endpoint.port, endpoint.command_port)
                != (restore_host, restore_port, restore_command_port)
                or (endpoint.target_system, endpoint.target_component)
                != (restore_target_system, restore_target_component)
            ):
                try:
                    if restore_command_port == restore_port:
                        self._mavlink_adapter.set_endpoint(restore_host, restore_port)
                    else:
                        self._mavlink_adapter.set_endpoint(
                            restore_host,
                            restore_port,
                            command_port=restore_command_port,
                        )
                    self._mavlink_adapter.set_target(
                        restore_target_system, restore_target_component
                    )
                except OSError as error:
                    restore_error = (
                        "vehicle_command dispatch failed to restore active endpoint "
                        f"{restore_host}:{restore_port} "
                        f"(command_port={restore_command_port}, "
                        f"target={restore_target_system}/{restore_target_component}): {error}"
                    )
            if (
                not restore_error
                and should_pause_stream
                and resume_setpoint is not None
                and self._state in self._AUTONOMOUS_STATES
            ):
                try:
                    self._mavlink_adapter.start_stream(self._mission_id, resume_setpoint)
                except OSError as error:
                    restore_error = (
                        "vehicle_command dispatch failed to resume mission streaming "
                        f"for '{self._mission_id}': {error}"
                    )

        if restore_error:
            return False, restore_error

        if not command_sent:
            return (
                False,
                f"vehicle_command '{command}' was not acknowledged by '{endpoint.vehicle_id}'",
            )
        return True, ""

    def _apply_mavlink_endpoint(self, endpoint: ScoutEndpoint) -> None:
        command_port = endpoint.command_port if endpoint.command_port > 0 else endpoint.port
        if command_port == endpoint.port:
            self._mavlink_adapter.set_endpoint(endpoint.host, endpoint.port)
        else:
            self._mavlink_adapter.set_endpoint(
                endpoint.host,
                endpoint.port,
                command_port=command_port,
            )
        self._mavlink_adapter.set_target(
            endpoint.target_system,
            endpoint.target_component,
        )

    def _parse_mission_payload(
        self, zone_geometry: str
    ) -> tuple[str, str, list[dict[str, float]] | None, str]:
        raw_payload = zone_geometry.strip()
        if not raw_payload:
            return "", "", None, (
                "zone_geometry is required. Expected JSON payload: "
                '{"pattern":"lawnmower|spiral","zone":{"id":"...","polygon":[{"x":0,"z":0}]}}'
            )

        try:
            payload = json.loads(raw_payload)
        except json.JSONDecodeError as error:
            return "", "", None, f"zone_geometry is not valid JSON: {error.msg}"

        if not isinstance(payload, dict):
            return "", "", None, "zone_geometry root value must be a JSON object"

        pattern = str(payload.get("pattern", "")).strip().lower()
        if pattern not in self._SUPPORTED_PATTERNS:
            return (
                "",
                "",
                None,
                f"unsupported pattern '{pattern}'. Supported values: lawnmower, spiral",
            )

        zone_payload = payload.get("zone")
        if not isinstance(zone_payload, dict):
            return "", "", None, "zone object is required in zone_geometry"

        zone_id = str(zone_payload.get("id", "")).strip()
        if not zone_id:
            return "", "", None, "zone.id is required in zone_geometry"

        polygon = zone_payload.get("polygon")
        if not isinstance(polygon, list):
            return "", "", None, "zone.polygon must be an array of points"

        return pattern, zone_id, polygon, ""

    def _build_plan_from_polygon(
        self, *, pattern: str, zone_id: str, polygon: list[dict[str, float]]
    ) -> tuple[MissionPlanState | None, str]:
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

    def _parse_mission_plan(
        self, zone_geometry: str
    ) -> tuple[MissionPlanState | None, str]:
        pattern, zone_id, polygon, error = self._parse_mission_payload(zone_geometry)
        if error:
            return None, error
        assert polygon is not None
        return self._build_plan_from_polygon(
            pattern=pattern,
            zone_id=zone_id,
            polygon=polygon,
        )

    def _build_partitioned_scout_plans(
        self,
        *,
        pattern: str,
        zone_id: str,
        polygon: list[dict[str, float]],
        online_scouts: list[ScoutEndpoint],
        preferred_active_vehicle_id: str,
    ) -> tuple[
        dict[str, MissionPlanState],
        dict[str, list[dict[str, float]]],
        str,
        str,
    ]:
        if not online_scouts:
            return {}, {}, "", "no online scouts available"

        online_by_vehicle = {endpoint.vehicle_id: endpoint for endpoint in online_scouts}
        ordered_vehicle_ids = sorted(online_by_vehicle.keys())
        preferred_vehicle_id = self._normalize_vehicle_id(preferred_active_vehicle_id)

        # Fallback path for single-scout availability or partition failures.
        def _single_plan_fallback(
            vehicle_id: str,
        ) -> tuple[
            dict[str, MissionPlanState],
            dict[str, list[dict[str, float]]],
            str,
            str,
        ]:
            plan, plan_error = self._build_plan_from_polygon(
                pattern=pattern,
                zone_id=zone_id,
                polygon=polygon,
            )
            if plan is None:
                return {}, {}, "", plan_error
            return (
                {vehicle_id: plan},
                {vehicle_id: [dict(point) for point in polygon]},
                vehicle_id,
                "",
            )

        if len(ordered_vehicle_ids) < 2:
            active_vehicle = (
                preferred_vehicle_id
                if preferred_vehicle_id in online_by_vehicle
                else ordered_vehicle_ids[0]
            )
            return _single_plan_fallback(active_vehicle)

        partitions = partition_polygon_for_scouts(polygon, ordered_vehicle_ids)
        if len(partitions) < 2:
            active_vehicle = (
                preferred_vehicle_id
                if preferred_vehicle_id in online_by_vehicle
                else ordered_vehicle_ids[0]
            )
            return _single_plan_fallback(active_vehicle)

        plans: dict[str, MissionPlanState] = {}
        zone_polygons: dict[str, list[dict[str, float]]] = {}
        for partition in partitions:
            if not isinstance(partition, dict):
                continue
            vehicle_id = self._normalize_vehicle_id(str(partition.get("vehicle_id", "")))
            if not vehicle_id or vehicle_id not in online_by_vehicle:
                continue
            zone_polygon = partition.get("polygon")
            if not isinstance(zone_polygon, list):
                continue
            partition_zone_id = str(partition.get("zone_id", "")).strip()
            if not partition_zone_id:
                partition_zone_id = zone_id
            plan, plan_error = self._build_plan_from_polygon(
                pattern=pattern,
                zone_id=partition_zone_id,
                polygon=zone_polygon,
            )
            if plan is None:
                self.get_logger().warning(
                    f"Skipping partition for {vehicle_id}: {plan_error}"
                )
                continue
            plans[vehicle_id] = plan
            zone_polygons[vehicle_id] = [dict(point) for point in zone_polygon]

        if not plans:
            active_vehicle = (
                preferred_vehicle_id
                if preferred_vehicle_id in online_by_vehicle
                else ordered_vehicle_ids[0]
            )
            return _single_plan_fallback(active_vehicle)

        active_vehicle_id = preferred_vehicle_id
        if active_vehicle_id not in plans:
            active_vehicle_id = sorted(plans.keys())[0]
        return plans, zone_polygons, active_vehicle_id, ""

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

    def _current_waypoint_for_plan(
        self, plan: MissionPlanState
    ) -> dict[str, float] | None:
        if not plan.waypoints:
            return None
        index = plan.current_waypoint_index
        if index < 0:
            index = 0
            plan.current_waypoint_index = 0
        if index >= len(plan.waypoints):
            return None
        return plan.waypoints[index]

    def _is_search_plan_complete(self, plan: MissionPlanState) -> bool:
        if not plan.waypoints:
            return True
        return plan.current_waypoint_index >= len(plan.waypoints)

    def _search_plan_progress_percent(self, plan: MissionPlanState) -> float:
        if not plan.waypoints:
            return 100.0
        completed = max(0, min(plan.current_waypoint_index, len(plan.waypoints)))
        return min(100.0, max(0.0, (completed / len(plan.waypoints)) * 100.0))

    def _advance_search_plan_for_vehicle(self, vehicle_id: str) -> None:
        plan = self._scout_plans.get(vehicle_id)
        if plan is None:
            return
        waypoint = self._current_waypoint_for_plan(plan)
        if waypoint is None:
            return

        source_mode, vio_position = self._resolve_position_source_for_vehicle(vehicle_id)
        if source_mode == self._POSITION_SOURCE_VIO:
            position = vio_position
            if position is None:
                self._vehicle_position_sources[vehicle_id] = "vio_odometry:unavailable"
                return
            self._vehicle_position_sources[vehicle_id] = "vio_odometry"
            plan.scout_position["x"] = float(position["x"])
            plan.scout_position["z"] = float(position["z"])
        else:
            self._vehicle_position_sources[vehicle_id] = "telemetry_geodetic"
            position = plan.scout_position
        delta_x = waypoint["x"] - position["x"]
        delta_z = waypoint["z"] - position["z"]
        distance = math.hypot(delta_x, delta_z)

        if distance <= self._waypoint_reached_threshold_m:
            next_index = plan.current_waypoint_index + 1
            if next_index < len(plan.waypoints):
                plan.current_waypoint_index = next_index
                if (
                    vehicle_id == self._active_scout_vehicle_id
                    and self._state in self._AUTONOMOUS_STATES
                ):
                    self._tracking_command_adapter().update_setpoint(
                        plan.waypoints[next_index]
                    )
            else:
                plan.current_waypoint_index = len(plan.waypoints)
                self._set_scout_assignment(
                    vehicle_id,
                    "IDLE",
                    label=f"IDLE:{plan.zone_id}:complete",
                )
            return

        if source_mode == self._POSITION_SOURCE_VIO:
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

    def _all_search_plans_complete(self) -> bool:
        if not self._scout_plans:
            return False
        return all(self._is_search_plan_complete(plan) for plan in self._scout_plans.values())

    def _redistribute_completed_scout_work(self) -> None:
        if self._state != "SEARCHING":
            return
        if len(self._scout_plans) < 2:
            return

        completed_ids = sorted(
            vehicle_id
            for vehicle_id, plan in self._scout_plans.items()
            if self._is_search_plan_complete(plan)
        )
        if not completed_ids:
            return

        for completed_vehicle_id in completed_ids:
            donor_candidates: list[tuple[int, str]] = []
            for vehicle_id, plan in self._scout_plans.items():
                if vehicle_id == completed_vehicle_id:
                    continue
                remaining = len(plan.waypoints) - plan.current_waypoint_index
                if remaining > 2:
                    donor_candidates.append((remaining, vehicle_id))
            if not donor_candidates:
                continue
            donor_candidates.sort(key=lambda item: (-item[0], item[1]))
            donor_vehicle_id = donor_candidates[0][1]
            donor_plan = self._scout_plans[donor_vehicle_id]
            remaining = len(donor_plan.waypoints) - donor_plan.current_waypoint_index
            split_offset = max(1, remaining // 2)
            split_index = donor_plan.current_waypoint_index + split_offset
            if split_index >= len(donor_plan.waypoints) - 1:
                continue

            reassigned_tail = [dict(point) for point in donor_plan.waypoints[split_index:]]
            if len(reassigned_tail) < 2:
                continue

            donor_plan.waypoints = [dict(point) for point in donor_plan.waypoints[:split_index]]
            if donor_plan.current_waypoint_index >= len(donor_plan.waypoints):
                donor_plan.current_waypoint_index = len(donor_plan.waypoints)
                self._set_scout_assignment(
                    donor_vehicle_id,
                    "IDLE",
                    label=f"IDLE:{donor_plan.zone_id}:complete",
                )

            new_zone_id = f"{donor_plan.zone_id}-assist-{completed_vehicle_id}"
            self._scout_plans[completed_vehicle_id] = MissionPlanState(
                pattern_type=donor_plan.pattern_type,
                zone_id=new_zone_id,
                waypoints=reassigned_tail,
                current_waypoint_index=0,
                scout_position={
                    "x": float(reassigned_tail[0]["x"]),
                    "z": float(reassigned_tail[0]["z"]),
                },
            )
            self._zone_polygons_by_vehicle[completed_vehicle_id] = []
            self._set_scout_assignment(
                completed_vehicle_id,
                "SEARCHING",
                label=f"SEARCHING:{new_zone_id}",
            )
            self.get_logger().info(
                "Reassigned uncovered work from "
                f"{donor_vehicle_id} to {completed_vehicle_id} ({new_zone_id})"
            )

    def _sync_assignments_with_telemetry_freshness(self) -> None:
        if not self._mission_id:
            return

        for endpoint in self._scout_endpoints:
            vehicle_id = endpoint.vehicle_id
            assignment = self._vehicle_assignments.get(vehicle_id, "IDLE")
            assignment_label = self._assignment_labels.get(vehicle_id, assignment)
            if self._is_endpoint_online(endpoint):
                if assignment == "IDLE" and vehicle_id in self._scout_plans:
                    if "complete" in assignment_label.lower():
                        continue
                    label = f"SEARCHING:{self._scout_plans[vehicle_id].zone_id}"
                    self._set_scout_assignment(vehicle_id, "SEARCHING", label=label)
                continue
            if (
                self._tracking_context.active
                and vehicle_id == self._tracking_context.assigned_scout_vehicle_id
            ):
                continue
            self._set_scout_assignment(vehicle_id, "IDLE", label="IDLE:offline")

        online_rangers = set(self._online_ranger_vehicle_ids())
        if (
            self._active_ranger_vehicle_id
            and self._active_ranger_vehicle_id not in online_rangers
        ):
            if self._active_ranger_vehicle_id in self._vehicle_assignments:
                self._set_scout_assignment(
                    self._active_ranger_vehicle_id,
                    "IDLE",
                    label="IDLE:offline",
                )
            self._close_ranger_adapter()
            self._active_ranger_vehicle_id = ""

        if not self._active_ranger_vehicle_id and online_rangers:
            self._active_ranger_vehicle_id = sorted(online_rangers)[0]
            self._set_scout_assignment(
                self._active_ranger_vehicle_id,
                "OVERWATCH",
                label="OVERWATCH",
            )
            self._start_ranger_overwatch_execution()
        elif self._active_ranger_vehicle_id in online_rangers:
            self._set_scout_assignment(
                self._active_ranger_vehicle_id,
                "OVERWATCH",
                label="OVERWATCH",
            )
        for endpoint in self._ranger_endpoints:
            if endpoint.vehicle_id in online_rangers:
                continue
            if endpoint.vehicle_id in self._vehicle_assignments:
                self._set_scout_assignment(
                    endpoint.vehicle_id,
                    "IDLE",
                    label="IDLE:offline",
                )

    def _vehicle_progress_snapshot(self) -> dict[str, float]:
        progress: dict[str, float] = {}
        for vehicle_id, plan in self._scout_plans.items():
            progress[vehicle_id] = round(self._search_plan_progress_percent(plan), 2)
        if (
            self._tracking_context.active
            and self._tracking_context.assigned_scout_vehicle_id
            and self._plan_state.waypoints
        ):
            progress[self._tracking_context.assigned_scout_vehicle_id] = round(
                self._search_plan_progress_percent(self._plan_state), 2
            )
        if self._active_ranger_vehicle_id:
            if self._ranger_orbit_waypoints:
                progress[self._active_ranger_vehicle_id] = round(
                    (
                        (self._ranger_orbit_index % len(self._ranger_orbit_waypoints))
                        / len(self._ranger_orbit_waypoints)
                    )
                    * 100.0,
                    2,
                )
            else:
                progress[self._active_ranger_vehicle_id] = 0.0
        return dict(sorted(progress.items()))

    def _vehicle_online_snapshot(self) -> dict[str, bool]:
        online: dict[str, bool] = {}
        for endpoint in self._scout_endpoints:
            online[endpoint.vehicle_id] = self._is_endpoint_online(endpoint)
        ranger_online = set(self._online_ranger_vehicle_ids())
        for endpoint in self._ranger_endpoints:
            online[endpoint.vehicle_id] = endpoint.vehicle_id in ranger_online
        for ranger_vehicle_id in ranger_online:
            online[ranger_vehicle_id] = True
        if self._active_ranger_vehicle_id and self._active_ranger_vehicle_id not in online:
            online[self._active_ranger_vehicle_id] = (
                self._active_ranger_vehicle_id in ranger_online
            )
        return dict(sorted(online.items()))

    def _polygon_centroid(self, polygon: list[dict[str, float]]) -> dict[str, float]:
        if not polygon:
            return {"x": 0.0, "z": 0.0}
        count = len(polygon)
        return {
            "x": sum(float(point.get("x", 0.0)) for point in polygon) / count,
            "z": sum(float(point.get("z", 0.0)) for point in polygon) / count,
        }

    def _build_ranger_orbit_waypoints(
        self, centroid: dict[str, float]
    ) -> list[dict[str, float]]:
        count = max(self._ranger_orbit_waypoint_count, 4)
        radius = max(self._ranger_overwatch_radius_m, 1.0)
        waypoints: list[dict[str, float]] = []
        for index in range(count):
            theta = (2.0 * math.pi * index) / count
            waypoints.append(
                {
                    "x": float(centroid["x"]) + (radius * math.cos(theta)),
                    "z": float(centroid["z"]) + (radius * math.sin(theta)),
                    "altitude_m": float(self._ranger_overwatch_altitude_m),
                }
            )
        return waypoints

    def _activate_ranger_overwatch(self, mission_polygon: list[dict[str, float]]) -> None:
        self._active_ranger_vehicle_id = ""
        self._ranger_orbit_waypoints = []
        self._ranger_orbit_index = 0
        self._close_ranger_adapter()

        online_ranger_vehicle_ids = self._online_ranger_vehicle_ids()
        if not online_ranger_vehicle_ids:
            return
        ranger_vehicle_id = online_ranger_vehicle_ids[0]
        centroid = self._polygon_centroid(mission_polygon)
        self._ranger_orbit_waypoints = self._build_ranger_orbit_waypoints(centroid)
        self._active_ranger_vehicle_id = ranger_vehicle_id
        self._set_scout_assignment(ranger_vehicle_id, "OVERWATCH", label="OVERWATCH")
        self._start_ranger_overwatch_execution()

    def _start_ranger_overwatch_execution(self) -> None:
        if self._state not in self._AUTONOMOUS_STATES:
            return
        if not self._mission_id or not self._active_ranger_vehicle_id:
            return
        if not self._ranger_orbit_waypoints:
            return

        endpoint = self._resolve_ranger_endpoint(self._active_ranger_vehicle_id)
        if endpoint is None:
            return

        current_waypoint = self._ranger_orbit_waypoints[
            self._ranger_orbit_index % len(self._ranger_orbit_waypoints)
        ]
        if (
            self._ranger_mavlink_adapter is None
            or self._ranger_adapter_vehicle_id != endpoint.vehicle_id
        ):
            self._close_ranger_adapter()
            self._ranger_mavlink_adapter = self._build_ranger_adapter(endpoint)
            self._ranger_adapter_vehicle_id = endpoint.vehicle_id

        if self._ranger_mavlink_adapter.is_streaming:
            self._ranger_mavlink_adapter.update_setpoint(current_waypoint)
            return

        self._ranger_mavlink_adapter.upload_mission_items_int(
            self._mission_id, self._ranger_orbit_waypoints
        )
        self._ranger_mavlink_adapter.start_stream(self._mission_id, current_waypoint)

    def _update_ranger_overwatch_orbit(self) -> None:
        if not self._active_ranger_vehicle_id:
            return
        if not self._ranger_orbit_waypoints:
            return
        self._start_ranger_overwatch_execution()
        current_index = self._ranger_orbit_index % len(self._ranger_orbit_waypoints)
        current_waypoint = self._ranger_orbit_waypoints[current_index]
        ranger_position = self._ranger_position_snapshot.get(self._active_ranger_vehicle_id)
        if ranger_position is None:
            return
        delta_x = float(current_waypoint["x"]) - float(ranger_position["x"])
        delta_z = float(current_waypoint["z"]) - float(ranger_position["z"])
        distance = math.hypot(delta_x, delta_z)
        if distance <= self._waypoint_reached_threshold_m:
            self._ranger_orbit_index = (current_index + 1) % len(self._ranger_orbit_waypoints)
            self._start_ranger_overwatch_execution()

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

        tracking_vehicle_id = self._tracking_context.assigned_scout_vehicle_id
        if not tracking_vehicle_id:
            tracking_vehicle_id = self._active_scout_vehicle_id
        if not tracking_vehicle_id:
            return

        source_mode, vio_position = self._resolve_position_source_for_vehicle(
            tracking_vehicle_id
        )
        if source_mode == self._POSITION_SOURCE_VIO:
            position = vio_position
            if position is None:
                self._vehicle_position_sources[tracking_vehicle_id] = (
                    "vio_odometry:unavailable"
                )
                return
            self._vehicle_position_sources[tracking_vehicle_id] = "vio_odometry"
            self._plan_state.scout_position["x"] = float(position["x"])
            self._plan_state.scout_position["z"] = float(position["z"])
        else:
            if self._active_scout_vehicle_id:
                self._vehicle_position_sources[self._active_scout_vehicle_id] = (
                    "telemetry_geodetic"
                )
            position = self._plan_state.scout_position
        delta_x = waypoint["x"] - position["x"]
        delta_z = waypoint["z"] - position["z"]
        distance = math.hypot(delta_x, delta_z)

        if distance <= self._waypoint_reached_threshold_m:
            self._advance_waypoint_index()
            return

        if source_mode == self._POSITION_SOURCE_VIO:
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

        pattern, zone_id, polygon, payload_error = self._parse_mission_payload(
            request.zone_geometry
        )
        if payload_error:
            response.success = False
            response.message = f"start_mission validation error: {payload_error}"
            return response
        assert polygon is not None
        valid_polygon, polygon_error = validate_polygon(polygon)
        if not valid_polygon:
            response.success = False
            response.message = f"start_mission validation error: invalid zone polygon: {polygon_error}"
            return response

        selected_scout = self._select_first_available_scout()
        if selected_scout is None:
            response.success = False
            response.message = (
                "start_mission rejected: no scout endpoint reported telemetry recently"
            )
            return response
        online_scouts = self._online_scout_endpoints()
        scout_plans, zone_polygons, active_vehicle_id, plan_error = (
            self._build_partitioned_scout_plans(
                pattern=pattern,
                zone_id=zone_id,
                polygon=polygon,
                online_scouts=online_scouts,
                preferred_active_vehicle_id=selected_scout.vehicle_id,
            )
        )
        if not scout_plans:
            response.success = False
            response.message = f"start_mission validation error: {plan_error}"
            return response

        self._active_scout_vehicle_id = active_vehicle_id
        self._close_tracking_adapter()
        active_endpoint = self._resolve_scout_endpoint(self._active_scout_vehicle_id)
        if active_endpoint is not None:
            self._apply_mavlink_endpoint(active_endpoint)
        self._reset_vehicle_command_states()

        self._mission_id = self._set_default_mission_id_if_needed(request.mission_id)
        self._mission_started_sec = self._now_seconds()
        self._reset_progress()
        self._active_scout_vehicle_id = active_vehicle_id
        self._scout_plans = scout_plans
        self._zone_polygons_by_vehicle = zone_polygons
        self._plan_state = self._scout_plans[self._active_scout_vehicle_id]
        self._initialize_search_assignments(self._active_scout_vehicle_id)
        self._activate_ranger_overwatch(polygon)
        self._planning_countdown = max(self._planning_cycles, 1)
        self._transition_to("PLANNING")

        response.success = True
        response.message = (
            f"Mission {self._mission_id} accepted with pattern '{self._plan_state.pattern_type}' "
            f"({len(self._plan_state.waypoints)} waypoints) on scout '{self._active_scout_vehicle_id}' "
            f"with {len(self._scout_plans)} scout assignments"
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

        # Suspend active streaming to prevent setpoint race conditions during
        # multi-endpoint RTL command dispatch.
        self._close_tracking_adapter()
        self._close_ranger_adapter()
        self._mavlink_adapter.stop_stream()
        target_endpoints = self._active_abort_endpoints()
        successful_dispatches, total_dispatches = self._dispatch_abort_rtl(target_endpoints)
        for endpoint in target_endpoints:
            self._vehicle_command_states[endpoint.vehicle_id] = (
                self._VEHICLE_COMMAND_STATE_RETURNING
            )

        self._transition_to("ABORTED")
        self._reset_plan_state()
        self._return_paths_by_vehicle.clear()
        self._return_path_index_by_vehicle.clear()
        self._return_fallback_reasons.clear()
        response.success = True
        response.message = (
            f"Mission {self._mission_id} aborted; RTL dispatches "
            f"{successful_dispatches}/{total_dispatches}"
        )
        return response

    def _handle_vehicle_command(
        self, request: VehicleCommand.Request, response: VehicleCommand.Response
    ) -> VehicleCommand.Response:
        command = self._normalized_command(request.command)
        if command not in self._SUPPORTED_VEHICLE_COMMANDS:
            response.success = False
            response.message = (
                f"vehicle_command rejected due to unsupported command '{request.command}'"
            )
            return response

        if self._state not in self._AUTONOMOUS_STATES:
            response.success = False
            response.message = f"vehicle_command '{command}' rejected while in {self._state}"
            return response

        if request.mission_id.strip() and request.mission_id.strip() != self._mission_id:
            response.success = False
            response.message = "vehicle_command rejected due to mission_id mismatch"
            return response

        normalized_vehicle_id = self._normalize_vehicle_id(request.vehicle_id)
        if not normalized_vehicle_id:
            response.success = False
            response.message = "vehicle_command rejected due to missing vehicle_id"
            return response

        endpoint = self._select_endpoint_by_vehicle_id(normalized_vehicle_id)
        if endpoint is None:
            response.success = False
            response.message = (
                f"vehicle_command rejected due to unknown vehicle_id '{request.vehicle_id}'"
            )
            return response

        if not self._is_endpoint_online(endpoint):
            response.success = False
            response.message = (
                f"vehicle_command rejected due to offline vehicle_id '{request.vehicle_id}'"
            )
            return response

        transition_valid, transition_error = self._validate_vehicle_command_transition(
            vehicle_id=endpoint.vehicle_id, command=command
        )
        if not transition_valid:
            response.success = False
            response.message = transition_error
            return response

        command_sent, command_error = self._dispatch_vehicle_command(endpoint, command)
        if not command_sent:
            response.success = False
            response.message = command_error
            return response

        self._vehicle_command_states[endpoint.vehicle_id] = self._next_vehicle_command_state(
            command
        )

        response.success = True
        response.message = (
            f"vehicle_command '{command}' accepted for target '{endpoint.vehicle_id}'"
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
            if self._state == "SEARCHING" and self._scout_plans:
                for vehicle_id in sorted(self._scout_plans.keys()):
                    if self._vehicle_assignments.get(vehicle_id) == "RETURNING":
                        self._advance_return_plan_for_vehicle(vehicle_id)
                    else:
                        self._advance_search_plan_for_vehicle(vehicle_id)
                if (
                    self._active_scout_vehicle_id in self._scout_plans
                    and self._vehicle_assignments.get(self._active_scout_vehicle_id)
                    != "RETURNING"
                ):
                    self._plan_state = self._scout_plans[self._active_scout_vehicle_id]
                self._redistribute_completed_scout_work()
                self._enforce_mixed_returning_separation()
                if self._all_search_plans_complete():
                    self._transition_to("COMPLETE")
                self._update_ranger_overwatch_orbit()
            elif self._state == "TRACKING":
                # Parallel execution: continue search coverage with non-tracking scouts
                # while the assigned tracking scout handles the focused investigation.
                tracked_vehicle_id = self._tracking_context.assigned_scout_vehicle_id
                for vehicle_id in sorted(self._scout_plans.keys()):
                    if vehicle_id == tracked_vehicle_id:
                        continue
                    self._advance_search_plan_for_vehicle(vehicle_id)
                self._update_scout_position_towards_waypoint()
                self._update_ranger_overwatch_orbit()
            else:
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
        self._sync_assignments_with_telemetry_freshness()

        self._coverage_percent = min(
            100.0, self._coverage_percent + self._coverage_increment_per_tick
        )
        covered_area = (self._coverage_percent / 100.0) * self._search_area_km2
        self._grid_completed = int(round((self._coverage_percent / 100.0) * self._grid_total))
        estimated_ticks_remaining = (100.0 - self._coverage_percent) / max(
            self._coverage_increment_per_tick, 0.01
        )
        estimated_time_remaining = max(0.0, estimated_ticks_remaining * self._progress_period)
        assignment_items = dict(sorted(self._vehicle_assignments.items()))
        assignment_labels = dict(sorted(self._assignment_labels.items()))
        vehicle_progress = self._vehicle_progress_snapshot()
        vehicle_online = self._vehicle_online_snapshot()
        active_drones = sum(
            1
            for assignment in assignment_items.values()
            if assignment not in {"IDLE"}
        )
        total_tracked_drones = len(
            {endpoint.vehicle_id for endpoint in self._scout_endpoints}
            | {endpoint.vehicle_id for endpoint in self._ranger_endpoints}
        )

        progress_message = MissionProgress()
        progress_message.coverage_percent = float(self._coverage_percent)
        progress_message.search_area_km2 = float(self._search_area_km2)
        progress_message.covered_area_km2 = float(covered_area)
        progress_message.active_drones = int(active_drones)
        progress_message.total_drones = int(
            max(self._total_drones, total_tracked_drones, len(assignment_items), 1)
        )
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
                "activeRangerVehicleId": self._active_ranger_vehicle_id,
                "trackingActive": self._tracking_context.active,
                "trackingDispatchLatencyMs": self._last_tracking_dispatch_latency_ms,
                "lastTrackingCompletionReason": self._last_tracking_completion_reason,
                "vehicleAssignments": assignment_items,
                "vehicleAssignmentLabels": assignment_labels,
                "vehicleProgress": vehicle_progress,
                "vehicleOnline": vehicle_online,
                "positionSourceMode": self._navigation_position_source,
                "vehiclePositionSources": self._vehicle_position_source_snapshot(),
                "vehicleSlamModes": self._vehicle_slam_mode_snapshot(),
                "trackingPreservedNonTargetVehicles": list(
                    self._tracking_context.preserved_non_target_vehicle_ids
                ),
                "lastDetectionRejection": self._last_detection_rejection_reason,
                "returnTrajectory": self._active_return_trajectory_payload(),
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
