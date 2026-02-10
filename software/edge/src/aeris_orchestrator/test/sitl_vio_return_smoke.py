#!/usr/bin/env python3
"""Smoke validation for VIO-based return trajectory publication."""

from __future__ import annotations

import json
import math
import sys
import time
from dataclasses import dataclass

import rclpy
from aeris_msgs.msg import MissionState, Telemetry
from aeris_msgs.srv import MissionCommand, VehicleCommand
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import String


VALID_ZONE_GEOMETRY = json.dumps(
    {
        "pattern": "lawnmower",
        "zone": {
            "id": "vio-return-zone",
            "polygon": [
                {"x": -20.0, "z": -20.0},
                {"x": 20.0, "z": -20.0},
                {"x": 20.0, "z": 20.0},
                {"x": -20.0, "z": 20.0},
            ],
        },
    }
)


@dataclass
class ProgressSnapshot:
    payload: dict
    received_monotonic: float


class VioReturnSmokeNode(Node):
    def __init__(self) -> None:
        super().__init__("vio_return_sitl_smoke")
        self.state_history: list[str] = []
        self.latest_progress: ProgressSnapshot | None = None

        self._state_sub = self.create_subscription(
            MissionState, "/orchestrator/mission_state", self._on_state, 10
        )
        self._progress_sub = self.create_subscription(
            String, "/mission/progress", self._on_progress, 10
        )

        self.telemetry_pub = self.create_publisher(Telemetry, "/vehicle/telemetry", 10)
        self.odom_pub = self.create_publisher(Odometry, "/scout1/openvins/odom", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 10)

        self.start_client = self.create_client(MissionCommand, "start_mission")
        self.abort_client = self.create_client(MissionCommand, "abort_mission")
        self.vehicle_command_client = self.create_client(VehicleCommand, "vehicle_command")

    def _on_state(self, message: MissionState) -> None:
        self.state_history.append(message.state)

    def _on_progress(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(payload, dict):
            self.latest_progress = ProgressSnapshot(
                payload=payload,
                received_monotonic=time.monotonic(),
            )


def _wait_until(
    node: VioReturnSmokeNode, predicate, timeout_sec: float = 8.0, sleep_sec: float = 0.05
) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
        time.sleep(sleep_sec)
    return False


def _publish_scout_telemetry(
    node: VioReturnSmokeNode, *, vehicle_id: str, latitude: float, longitude: float
) -> None:
    msg = Telemetry()
    msg.vehicle_id = vehicle_id
    msg.vehicle_type = "scout"
    msg.position.latitude = float(latitude)
    msg.position.longitude = float(longitude)
    msg.timestamp = node.get_clock().now().to_msg()
    node.telemetry_pub.publish(msg)


def _publish_odom(node: VioReturnSmokeNode, *, x_m: float, y_m: float) -> None:
    msg = Odometry()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "odom"
    msg.child_frame_id = "base_link"
    msg.pose.pose.position.x = float(x_m)
    msg.pose.pose.position.y = float(y_m)
    msg.pose.pose.position.z = 0.0
    node.odom_pub.publish(msg)


def _publish_odom_sequence(
    node: VioReturnSmokeNode, points: list[tuple[float, float]], *, repeats: int = 2
) -> None:
    for x_m, y_m in points:
        for _ in range(max(repeats, 1)):
            _publish_odom(node, x_m=x_m, y_m=y_m)
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.05)


def _publish_clear_occupancy_map(node: VioReturnSmokeNode) -> None:
    msg = OccupancyGrid()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "map"
    msg.info.resolution = 1.0
    msg.info.width = 120
    msg.info.height = 120
    msg.info.origin.position.x = -60.0
    msg.info.origin.position.y = -60.0
    msg.data = [0] * (msg.info.width * msg.info.height)
    node.map_pub.publish(msg)


def _start_mission(node: VioReturnSmokeNode, mission_id: str) -> None:
    request = MissionCommand.Request()
    request.command = "START"
    request.mission_id = mission_id
    request.zone_geometry = VALID_ZONE_GEOMETRY
    future = node.start_client.call_async(request)
    if not _wait_until(node, lambda: future.done(), timeout_sec=8.0):
        raise RuntimeError("start_mission timed out")
    response = future.result()
    if response is None or not response.success:
        raise RuntimeError(f"start_mission failed: {getattr(response, 'message', '<none>')}")


def _send_recall(node: VioReturnSmokeNode, mission_id: str) -> None:
    request = VehicleCommand.Request()
    request.command = "RECALL"
    request.vehicle_id = "scout1"
    request.mission_id = mission_id
    future = node.vehicle_command_client.call_async(request)
    if not _wait_until(node, lambda: future.done(), timeout_sec=8.0):
        raise RuntimeError("vehicle_command RECALL timed out")
    response = future.result()
    if response is None or not response.success:
        raise RuntimeError(
            f"vehicle_command RECALL failed: {getattr(response, 'message', '<none>')}"
        )


def _abort_best_effort(node: VioReturnSmokeNode, mission_id: str) -> None:
    request = MissionCommand.Request()
    request.command = "ABORT"
    request.mission_id = mission_id
    request.zone_geometry = ""
    future = node.abort_client.call_async(request)
    _wait_until(node, lambda: future.done(), timeout_sec=3.0)


def run_smoke() -> int:
    rclpy.init()
    node = VioReturnSmokeNode()
    mission_id = f"sitl-vio-return-{int(time.time())}"
    start_monotonic = time.monotonic()
    try:
        if not node.start_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("start_mission service not available")
        if not node.abort_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("abort_mission service not available")
        if not node.vehicle_command_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("vehicle_command service not available")

        _publish_scout_telemetry(node, vehicle_id="scout1", latitude=0.0, longitude=0.0)
        _publish_scout_telemetry(node, vehicle_id="scout2", latitude=0.0003, longitude=0.0003)
        _publish_clear_occupancy_map(node)
        _publish_odom_sequence(node, [(0.0, 0.0), (4.0, 0.0), (8.0, 0.0)], repeats=2)

        _start_mission(node, mission_id)
        if not _wait_until(node, lambda: "SEARCHING" in node.state_history, timeout_sec=8.0):
            raise RuntimeError("Mission never reached SEARCHING")

        _publish_odom_sequence(
            node,
            [(0.0, 0.0), (4.0, 0.0), (8.0, 0.0), (10.0, 0.0), (12.0, 0.0)],
            repeats=2,
        )
        recall_sent_monotonic = time.monotonic()
        _send_recall(node, mission_id)

        if not _wait_until(
            node,
            lambda: node.latest_progress is not None
            and isinstance(node.latest_progress.payload.get("returnTrajectory"), dict),
            timeout_sec=8.0,
        ):
            raise RuntimeError("returnTrajectory was not published on /mission/progress")

        payload = node.latest_progress.payload if node.latest_progress else {}
        return_payload = payload.get("returnTrajectory")
        if not isinstance(return_payload, dict):
            raise RuntimeError("returnTrajectory payload type is invalid")
        required_keys = {"vehicleId", "state", "points", "etaSec", "lastUpdatedSec"}
        missing = [key for key in required_keys if key not in return_payload]
        if missing:
            raise RuntimeError(f"returnTrajectory is missing required keys: {missing}")
        if return_payload.get("vehicleId") != "scout_1":
            raise RuntimeError(
                f"returnTrajectory.vehicleId mismatch: {return_payload.get('vehicleId')}"
            )
        if return_payload.get("fallbackReason"):
            raise RuntimeError(
                f"Unexpected fallbackReason in success path: {return_payload.get('fallbackReason')}"
            )

        points = return_payload.get("points")
        if not isinstance(points, list) or len(points) < 2:
            raise RuntimeError("returnTrajectory.points must contain at least 2 points")
        final = points[-1]
        if not isinstance(final, dict):
            raise RuntimeError("returnTrajectory.points final item invalid")
        launch_error_m = math.hypot(float(final.get("x", 0.0)), float(final.get("z", 0.0)))
        if launch_error_m > 2.0:
            raise RuntimeError(f"Launch-point error too high: {launch_error_m:.3f}m > 2.0m")

        update_latency_sec = node.latest_progress.received_monotonic - recall_sent_monotonic
        if update_latency_sec > 1.0:
            raise RuntimeError(
                f"Trajectory update latency too high: {update_latency_sec:.3f}s > 1.0s"
            )

        total_elapsed = time.monotonic() - start_monotonic
        print(
            "SITL VIO return smoke passed: "
            f"launch_error_m={launch_error_m:.3f}, "
            f"update_latency_sec={update_latency_sec:.3f}, "
            f"elapsed_sec={total_elapsed:.3f}"
        )
        return 0
    except Exception as exc:  # noqa: BLE001
        print(f"SITL VIO return smoke failed: {exc}", file=sys.stderr)
        return 1
    finally:
        _abort_best_effort(node, mission_id)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(run_smoke())
