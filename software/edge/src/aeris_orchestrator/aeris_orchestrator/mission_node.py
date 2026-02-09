import json
import math
import time
from dataclasses import dataclass, field
from typing import Final

import rclpy
from aeris_msgs.msg import MissionProgress, MissionState
from aeris_msgs.srv import MissionCommand
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


class MissionNode(Node):
    """Mission lifecycle orchestrator for simulation and GCS integration."""

    _ACTIVE_PROGRESS_STATES: Final[set[str]] = {"SEARCHING", "TRACKING"}
    _AUTONOMOUS_STATES: Final[set[str]] = {"SEARCHING", "TRACKING"}
    _ABORTABLE_STATES: Final[set[str]] = {"SEARCHING", "TRACKING"}
    _SUPPORTED_PATTERNS: Final[set[str]] = {"lawnmower", "spiral"}

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
            f"mavlink={self._scout_mavlink_host}:{self._scout_mavlink_udp_port}"
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

        if new_state == "SEARCHING" and self._plan_state.waypoints:
            self._start_autonomous_execution()

        if new_state in {"ABORTED", "COMPLETE", "IDLE"}:
            self._mavlink_adapter.stop_stream()

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

    def _set_default_mission_id_if_needed(self, mission_id: str) -> str:
        stripped = mission_id.strip()
        if stripped:
            return stripped
        return f"mission-{int(time.time())}"

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

        self._mission_id = self._set_default_mission_id_if_needed(request.mission_id)
        self._reset_progress()
        self._plan_state = plan_state
        self._planning_countdown = max(self._planning_cycles, 1)
        self._transition_to("PLANNING")

        response.success = True
        response.message = (
            f"Mission {self._mission_id} accepted with pattern '{self._plan_state.pattern_type}' "
            f"({len(self._plan_state.waypoints)} waypoints)"
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

        self._transition_to("ABORTED")
        self._reset_plan_state()
        response.success = True
        response.message = f"Mission {self._mission_id} aborted"
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

        if self._coverage_percent >= 100.0:
            self._transition_to("COMPLETE")


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
