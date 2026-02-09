import json
import math
import re
import time
from dataclasses import dataclass, field
from typing import Callable, Final

import rclpy
from aeris_msgs.msg import MissionProgress, MissionState, Telemetry
from aeris_msgs.srv import MissionCommand, VehicleCommand
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
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
    command_port: int = 0
    target_system: int = 1
    target_component: int = 1


class MissionNode(Node):
    """Mission lifecycle orchestrator for simulation and GCS integration."""

    _ACTIVE_PROGRESS_STATES: Final[set[str]] = {"SEARCHING", "TRACKING"}
    _AUTONOMOUS_STATES: Final[set[str]] = {"SEARCHING", "TRACKING"}
    _ABORTABLE_STATES: Final[set[str]] = {"SEARCHING", "TRACKING"}
    _SUPPORTED_PATTERNS: Final[set[str]] = {"lawnmower", "spiral"}
    _SUPPORTED_VEHICLE_COMMANDS: Final[set[str]] = {"HOLD", "RESUME", "RECALL"}
    _VEHICLE_COMMAND_STATE_ACTIVE: Final[str] = "ACTIVE"
    _VEHICLE_COMMAND_STATE_HOLDING: Final[str] = "HOLDING"
    _VEHICLE_COMMAND_STATE_RETURNING: Final[str] = "RETURNING"

    def __init__(self) -> None:
        super().__init__("aeris_mission_orchestrator")

        queue_depth = int(self.declare_parameter("queue_depth", 10).value)
        self._control_loop_period = float(
            self.declare_parameter("control_loop_period_sec", 0.02).value
        )
        self._progress_period = float(
            self.declare_parameter("progress_publish_period_sec", 1.0).value
        )
        self._loop_budget_ms = float(self.declare_parameter("loop_budget_ms", 20.0).value)
        self._planning_cycles = int(self.declare_parameter("planning_cycles", 1).value)
        self._enable_tracking_simulation = bool(
            self.declare_parameter("enable_tracking_simulation", True).value
        )
        self._tracking_trigger_coverage_percent = float(
            self.declare_parameter("tracking_trigger_coverage_percent", 1.0).value
        )
        self._tracking_hold_cycles = int(
            self.declare_parameter("tracking_hold_cycles", 50).value
        )
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
        self._scout_endpoints_json = str(
            self.declare_parameter(
                "scout_endpoints_json",
                '[{"vehicle_id":"scout1","host":"127.0.0.1","port":14541,'
                '"command_port":14581,"target_system":2},'
                '{"vehicle_id":"scout2","host":"127.0.0.1","port":14542,'
                '"command_port":14582,"target_system":3}]',
            ).value
        )

        self._state = "IDLE"
        self._mission_id = ""
        self._planning_countdown = 0
        self._tracking_countdown = 0
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
        self._active_scout_vehicle_id = ""
        self._scout_endpoints = self._parse_scout_endpoints()
        self._vehicle_command_states: dict[str, str] = {}
        self._reset_vehicle_command_states()

        self._mavlink_adapter = MavlinkAdapter(
            host=self._scout_mavlink_host,
            port=self._scout_mavlink_udp_port,
            stream_hz=self._setpoint_stream_hz,
            logger=lambda message: self.get_logger().info(message),
        )

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
            f"scout_endpoints={len(self._scout_endpoints)}"
        )

    def destroy_node(self) -> bool:
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
            self._mavlink_adapter.stop_stream()
            if new_state in {"ABORTED", "IDLE"}:
                self._active_scout_vehicle_id = ""
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
        self._tracking_countdown = 0
        self._tracking_seen_this_mission = False

    def _reset_plan_state(self) -> None:
        self._plan_state = MissionPlanState()

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

    def _handle_vehicle_telemetry(self, message: Telemetry) -> None:
        vehicle_type = str(message.vehicle_type).strip().lower()
        vehicle_id = self._normalize_vehicle_id(message.vehicle_id)
        if not vehicle_id:
            return
        if vehicle_type and vehicle_type != "scout" and not vehicle_id.startswith("scout"):
            return
        self._scout_last_seen_monotonic[vehicle_id] = time.monotonic()

    def _select_first_available_scout(self) -> ScoutEndpoint | None:
        if not self._scout_endpoints:
            return None

        now = time.monotonic()
        availability_cutoff = max(self._scout_online_window_sec, 0.0)
        for endpoint in self._scout_endpoints:
            last_seen = self._scout_last_seen_monotonic.get(endpoint.vehicle_id)
            if last_seen is None:
                continue
            if now - last_seen <= availability_cutoff:
                return endpoint
        return None

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

    def _reset_vehicle_command_states(self) -> None:
        self._vehicle_command_states = {
            endpoint.vehicle_id: self._VEHICLE_COMMAND_STATE_ACTIVE
            for endpoint in self._scout_endpoints
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

    def _resolve_scout_endpoint(self, vehicle_id: str) -> ScoutEndpoint | None:
        normalized_vehicle_id = self._normalize_vehicle_id(vehicle_id)
        for endpoint in self._scout_endpoints:
            if endpoint.vehicle_id == normalized_vehicle_id:
                return endpoint
        return None

    def _is_endpoint_online(self, endpoint: ScoutEndpoint) -> bool:
        if self._scout_online_window_sec < 0:
            return False
        last_seen = self._scout_last_seen_monotonic.get(endpoint.vehicle_id)
        if last_seen is None:
            return False
        return (time.monotonic() - last_seen) <= self._scout_online_window_sec

    def _dispatch_vehicle_command(
        self, endpoint: ScoutEndpoint, command: str
    ) -> tuple[bool, str]:
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
                        f"{restore_host}:{restore_port}"
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
        if self._mavlink_adapter.is_streaming:
            self._mavlink_adapter.update_setpoint(waypoint)
            return

        self._mavlink_adapter.upload_mission_items_int(
            self._mission_id, self._plan_state.waypoints
        )
        self._mavlink_adapter.start_stream(self._mission_id, waypoint)

    def _advance_waypoint_index(self) -> None:
        current = self._plan_state.current_waypoint_index
        next_index = current + 1
        if next_index >= len(self._plan_state.waypoints):
            self._transition_to("COMPLETE")
            return

        self._plan_state.current_waypoint_index = next_index
        next_waypoint = self._plan_state.waypoints[next_index]
        self._mavlink_adapter.update_setpoint(next_waypoint)
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
        self._apply_mavlink_endpoint(selected_scout)
        self._reset_vehicle_command_states()

        self._mission_id = self._set_default_mission_id_if_needed(request.mission_id)
        self._reset_progress()
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
        self._mavlink_adapter.stop_stream()
        target_endpoints = self._active_abort_endpoints()
        successful_dispatches, total_dispatches = self._dispatch_abort_rtl(target_endpoints)
        for endpoint in target_endpoints:
            self._vehicle_command_states[endpoint.vehicle_id] = (
                self._VEHICLE_COMMAND_STATE_RETURNING
            )

        self._transition_to("ABORTED")
        self._reset_plan_state()
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

        endpoint = self._resolve_scout_endpoint(normalized_vehicle_id)
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
        elif self._state == "SEARCHING":
            should_enter_tracking = (
                self._enable_tracking_simulation
                and not self._tracking_seen_this_mission
                and self._coverage_percent >= self._tracking_trigger_coverage_percent
            )
            if should_enter_tracking:
                self._tracking_seen_this_mission = True
                self._tracking_countdown = max(self._tracking_hold_cycles, 1)
                self._transition_to("TRACKING")
        elif self._state == "TRACKING":
            self._tracking_countdown -= 1
            if self._tracking_countdown <= 0 and self._coverage_percent < 100.0:
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
