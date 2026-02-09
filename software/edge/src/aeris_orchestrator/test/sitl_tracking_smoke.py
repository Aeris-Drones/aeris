#!/usr/bin/env python3
"""SITL smoke harness for Story 1.5 tracking reassignment behavior."""

from __future__ import annotations

import json
import sys
import time
from dataclasses import dataclass

import rclpy
from aeris_msgs.msg import GasIsopleth, MissionState, Telemetry
from aeris_msgs.srv import MissionCommand
from geometry_msgs.msg import Point32, Polygon
from rclpy.node import Node
from std_msgs.msg import String


VALID_ZONE_GEOMETRY = json.dumps(
    {
        "pattern": "lawnmower",
        "zone": {
            "id": "sitl-zone",
            "polygon": [
                {"x": 0.0, "z": 0.0},
                {"x": 20.0, "z": 0.0},
                {"x": 20.0, "z": 20.0},
                {"x": 0.0, "z": 20.0},
            ],
        },
    }
)


@dataclass
class ProgressSnapshot:
    payload: dict
    received_monotonic: float


class TrackingSmokeNode(Node):
    def __init__(self) -> None:
        super().__init__("tracking_sitl_smoke")
        self.state_history: list[str] = []
        self.latest_progress: ProgressSnapshot | None = None

        self._state_sub = self.create_subscription(
            MissionState, "/orchestrator/mission_state", self._on_state, 10
        )
        self._progress_sub = self.create_subscription(
            String, "/mission/progress", self._on_progress, 10
        )
        self.telemetry_pub = self.create_publisher(Telemetry, "/vehicle/telemetry", 10)
        self.gas_pub = self.create_publisher(GasIsopleth, "gas/isopleth", 10)
        self.tracking_resolution_pub = self.create_publisher(
            String, "/mission/tracking_resolution", 10
        )

        self.start_client = self.create_client(MissionCommand, "start_mission")
        self.abort_client = self.create_client(MissionCommand, "abort_mission")

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
    node: TrackingSmokeNode, predicate, timeout_sec: float = 8.0, sleep_sec: float = 0.05
) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
        time.sleep(sleep_sec)
    return False


def _publish_scout_telemetry(
    node: TrackingSmokeNode, vehicle_id: str, *, latitude: float, longitude: float
) -> None:
    msg = Telemetry()
    msg.vehicle_id = vehicle_id
    msg.vehicle_type = "scout"
    msg.position.latitude = float(latitude)
    msg.position.longitude = float(longitude)
    msg.timestamp = node.get_clock().now().to_msg()
    node.telemetry_pub.publish(msg)


def _publish_ranger_telemetry(
    node: TrackingSmokeNode, vehicle_id: str, *, latitude: float, longitude: float
) -> None:
    msg = Telemetry()
    msg.vehicle_id = vehicle_id
    msg.vehicle_type = "ranger"
    msg.position.latitude = float(latitude)
    msg.position.longitude = float(longitude)
    msg.timestamp = node.get_clock().now().to_msg()
    node.telemetry_pub.publish(msg)


def _publish_gas_detection(node: TrackingSmokeNode, *, x: float, y: float) -> None:
    msg = GasIsopleth()
    now = node.get_clock().now().nanoseconds / 1e9
    sec = int(now)
    msg.stamp.sec = sec
    msg.stamp.nanosec = int((now - sec) * 1e9)
    msg.centerline = [
        Point32(x=float(x), y=float(y), z=0.0),
        Point32(x=float(x + 0.4), y=float(y + 0.2), z=0.0),
    ]
    msg.polygons = [
        Polygon(
            points=[
                Point32(x=float(x - 1.0), y=float(y - 1.0), z=0.0),
                Point32(x=float(x + 1.0), y=float(y - 1.0), z=0.0),
                Point32(x=float(x + 1.0), y=float(y + 1.0), z=0.0),
                Point32(x=float(x - 1.0), y=float(y + 1.0), z=0.0),
            ]
        )
    ]
    node.gas_pub.publish(msg)


def _start_mission(node: TrackingSmokeNode, mission_id: str) -> None:
    request = MissionCommand.Request()
    request.command = "START"
    request.mission_id = mission_id
    request.zone_geometry = VALID_ZONE_GEOMETRY
    future = node.start_client.call_async(request)
    if not _wait_until(node, lambda: future.done(), timeout_sec=8.0):
        raise RuntimeError("start_mission timed out")
    response = future.result()
    if response is None or not response.success:
        message = "<none>" if response is None else response.message
        raise RuntimeError(f"start_mission failed: {message}")


def _abort_best_effort(node: TrackingSmokeNode, mission_id: str) -> None:
    request = MissionCommand.Request()
    request.command = "ABORT"
    request.mission_id = mission_id
    request.zone_geometry = ""
    future = node.abort_client.call_async(request)
    _wait_until(node, lambda: future.done(), timeout_sec=3.0)


def run_smoke() -> int:
    rclpy.init()
    node = TrackingSmokeNode()
    mission_id = f"sitl-tracking-{int(time.time())}"
    try:
        if not node.start_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("start_mission service not available")
        if not node.abort_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("abort_mission service not available")

        _publish_scout_telemetry(node, "scout1", latitude=0.0, longitude=0.0)
        _publish_scout_telemetry(
            node, "scout2", latitude=0.00045, longitude=0.00045
        )
        _publish_ranger_telemetry(node, "ranger1", latitude=0.0002, longitude=0.0001)
        if not _wait_until(node, lambda: node.latest_progress is not None, timeout_sec=2.0):
            node.get_logger().info("No mission progress yet; continuing after telemetry warmup")

        _start_mission(node, mission_id)
        if not _wait_until(node, lambda: "SEARCHING" in node.state_history, timeout_sec=8.0):
            raise RuntimeError("Mission never reached SEARCHING")

        if not _wait_until(
            node,
            lambda: node.latest_progress is not None
            and isinstance(node.latest_progress.payload.get("vehicleAssignmentLabels"), dict),
            timeout_sec=8.0,
        ):
            raise RuntimeError("Mission progress is missing vehicle assignment labels")

        labels = (
            node.latest_progress.payload.get("vehicleAssignmentLabels", {})
            if node.latest_progress
            else {}
        )
        if not str(labels.get("scout_1", "")).startswith("SEARCHING:"):
            raise RuntimeError("scout_1 did not receive partition assignment label")
        if not str(labels.get("scout_2", "")).startswith("SEARCHING:"):
            raise RuntimeError("scout_2 did not receive partition assignment label")
        if labels.get("scout_1") == labels.get("scout_2"):
            raise RuntimeError("scout partition labels are not distinct")
        if labels.get("ranger_1") != "OVERWATCH":
            raise RuntimeError("ranger_1 did not receive OVERWATCH assignment label")

        _publish_gas_detection(node, x=50.0, y=50.0)
        if not _wait_until(
            node, lambda: "TRACKING" in node.state_history, timeout_sec=8.0
        ):
            raise RuntimeError("Detection did not trigger TRACKING")

        if not _wait_until(
            node,
            lambda: node.latest_progress is not None
            and node.latest_progress.payload.get("trackingActive") is True
            and node.latest_progress.payload.get("activeScoutVehicleId") == "scout_2",
            timeout_sec=8.0,
        ):
            raise RuntimeError("Tracking progress payload did not reflect scout_2 dispatch")

        progress_payload = node.latest_progress.payload if node.latest_progress else {}
        preserved = progress_payload.get("trackingPreservedNonTargetVehicles", [])
        assignments = progress_payload.get("vehicleAssignments", {})
        if "scout_1" not in preserved:
            raise RuntimeError("Non-target scout_1 was not reported as preserved")
        if assignments.get("scout_1") != "SEARCHING":
            raise RuntimeError("Non-target scout_1 assignment changed unexpectedly")
        labels = progress_payload.get("vehicleAssignmentLabels", {})
        if labels.get("ranger_1") != "OVERWATCH":
            raise RuntimeError("Ranger overwatch assignment did not persist through tracking")

        resolution = String()
        resolution.data = "resolved"
        node.tracking_resolution_pub.publish(resolution)
        if not _wait_until(
            node,
            lambda: node.state_history.count("SEARCHING") >= 2,
            timeout_sec=8.0,
        ):
            raise RuntimeError("Tracking did not return to SEARCHING after resolution")

        print("SITL tracking smoke passed: target scout replanned, non-target preserved.")
        return 0
    except Exception as exc:  # noqa: BLE001
        print(f"SITL tracking smoke failed: {exc}", file=sys.stderr)
        return 1
    finally:
        _abort_best_effort(node, mission_id)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(run_smoke())
