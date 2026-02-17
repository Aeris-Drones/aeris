from contextlib import suppress
import json
import math
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("aeris_msgs.msg")
from aeris_msgs.msg import (
    AcousticBearing,
    FusedDetection,
    GasIsopleth,
    ThermalHotspot,
)
from geometry_msgs.msg import Point32, Polygon
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from aeris_perception.fusion_node import FusionNode


class _FusionObserver(Node):
    def __init__(self, topic: str = "/detections/fused") -> None:
        super().__init__("fusion_observer_test")
        self.messages: list[FusedDetection] = []
        self._subscription = self.create_subscription(
            FusedDetection,
            topic,
            self._on_message,
            10,
        )

    def _on_message(self, message: FusedDetection) -> None:
        self.messages.append(message)


class _FusionStimulus(Node):
    def __init__(self) -> None:
        super().__init__("fusion_stimulus_test")
        self.thermal_pub = self.create_publisher(ThermalHotspot, "thermal/hotspots", 10)
        self.acoustic_pub = self.create_publisher(AcousticBearing, "acoustic/bearing", 10)
        self.gas_pub = self.create_publisher(GasIsopleth, "gas/isopleth", 10)


def _now_stamp(node: Node):
    now_sec = node.get_clock().now().nanoseconds / 1e9
    sec = int(now_sec)
    nanosec = int((now_sec - sec) * 1e9)
    return sec, nanosec


def test_fusion_node_publishes_upgrades_for_correlated_modalities() -> None:
    rclpy.init()
    executor = SingleThreadedExecutor()
    node = FusionNode(
        parameter_overrides=[
            Parameter("acoustic_projection_range_m", value=2.0),
        ]
    )
    observer = _FusionObserver()
    stimulus = _FusionStimulus()

    try:
        executor.add_node(node)
        executor.add_node(observer)
        executor.add_node(stimulus)

        sec, nanosec = _now_stamp(stimulus)
        thermal = ThermalHotspot()
        thermal.stamp.sec = sec
        thermal.stamp.nanosec = nanosec
        thermal.bbox_px = [300, 220, 340, 260]
        thermal.confidence = 0.82
        thermal.frame_id = "map"
        stimulus.thermal_pub.publish(thermal)

        acoustic = AcousticBearing()
        acoustic.stamp.sec = sec
        acoustic.stamp.nanosec = nanosec
        acoustic.bearing_deg = 2.0
        acoustic.confidence = 0.85
        stimulus.acoustic_pub.publish(acoustic)

        deadline = time.monotonic() + 2.5
        while time.monotonic() < deadline and len(observer.messages) < 2:
            executor.spin_once(timeout_sec=0.1)

        assert len(observer.messages) >= 2
        first = observer.messages[0]
        second = observer.messages[-1]
        assert first.confidence_level in {"LOW", "MEDIUM"}
        assert second.confidence_level == "HIGH"
        assert first.candidate_id == second.candidate_id
        assert second.source_modalities == ["acoustic", "thermal"]
        assert second.frame_id == "map"
        assert second.mission_id == ""
        assert math.isfinite(second.local_target.x)
        assert math.isfinite(second.local_target.y)
        assert len(second.local_geometry) >= 3
        payload = json.loads(second.hazard_payload_json)
        assert isinstance(payload, dict)
        assert "polygons" in payload
        assert len(payload["polygons"]) >= 1
        assert len(payload["polygons"][0]) >= 3
    finally:
        for active in (stimulus, observer, node):
            with suppress(RuntimeError):
                executor.remove_node(active)
        executor.shutdown()
        stimulus.destroy_node()
        observer.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def test_fusion_node_rejects_future_samples() -> None:
    rclpy.init()
    node = FusionNode(
        parameter_overrides=[
            Parameter("max_future_skew_sec", value=0.1),
        ]
    )
    published: list[FusedDetection] = []

    class _Recorder:
        def publish(self, message: FusedDetection) -> None:
            published.append(message)

    node._publisher = _Recorder()

    try:
        future = GasIsopleth()
        now = node._now_seconds()
        future.stamp.sec = int(now + 2.0)
        future.stamp.nanosec = 0
        future.centerline = [
            Point32(x=2.0, y=1.0, z=0.0),
            Point32(x=3.0, y=1.1, z=0.0),
        ]
        future.polygons = [
            Polygon(
                points=[
                    Point32(x=1.0, y=0.0, z=0.0),
                    Point32(x=4.0, y=0.0, z=0.0),
                    Point32(x=4.0, y=3.0, z=0.0),
                ]
            )
        ]
        node._handle_gas_isopleth(future)
        assert published == []
    finally:
        node.destroy_node()
        rclpy.shutdown()
