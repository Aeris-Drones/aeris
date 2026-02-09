import json
import time
from typing import Final

import rclpy
from aeris_msgs.msg import MissionProgress, MissionState
from aeris_msgs.srv import MissionCommand
from rclpy.node import Node
from std_msgs.msg import String


class MissionNode(Node):
    """Mission lifecycle orchestrator for simulation and GCS integration."""

    _ACTIVE_PROGRESS_STATES: Final[set[str]] = {"SEARCHING", "TRACKING"}
    _ABORTABLE_STATES: Final[set[str]] = {"SEARCHING", "TRACKING"}

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

        self._start_service = self.create_service(
            MissionCommand, "start_mission", self._handle_start_mission
        )
        self._abort_service = self.create_service(
            MissionCommand, "abort_mission", self._handle_abort_mission
        )

        self._control_timer = self.create_timer(
            self._control_loop_period, self._control_loop
        )
        self._progress_timer = self.create_timer(
            self._progress_period, self._publish_progress_if_active
        )

        self._publish_state(previous_state="")
        self.get_logger().info(
            "Mission node ready: state topic=/orchestrator/mission_state, "
            "progress topic=/orchestrator/mission_progress, "
            f"loop={self._control_loop_period:.3f}s budget={self._loop_budget_ms:.2f}ms"
        )

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _transition_to(self, new_state: str) -> None:
        if new_state == self._state:
            return
        previous = self._state
        self._state = new_state
        self._publish_state(previous_state=previous)
        self.get_logger().info(f"Mission state transition: {previous} -> {new_state}")

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

    def _normalized_command(self, command: str) -> str:
        return command.strip().upper()

    def _set_default_mission_id_if_needed(self, mission_id: str) -> str:
        stripped = mission_id.strip()
        if stripped:
            return stripped
        return f"mission-{int(time.time())}"

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

        self._mission_id = self._set_default_mission_id_if_needed(request.mission_id)
        self._reset_progress()
        self._planning_countdown = max(self._planning_cycles, 1)
        self._transition_to("PLANNING")

        response.success = True
        response.message = f"Mission {self._mission_id} accepted"
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
            self.get_logger().warn(
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
