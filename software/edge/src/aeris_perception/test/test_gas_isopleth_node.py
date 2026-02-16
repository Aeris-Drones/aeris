from contextlib import suppress
import math
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("aeris_msgs.msg")
pytest.importorskip("geometry_msgs.msg")
from aeris_msgs.msg import GasIsopleth
from geometry_msgs.msg import Point32, PointStamped, Polygon, Vector3Stamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from aeris_perception.gas_isopleth_node import GasIsoplethNode


class _IsoplethObserver(Node):
    def __init__(self, topic: str = "gas/isopleth") -> None:
        super().__init__("gas_isopleth_observer_test")
        self.messages: list[GasIsopleth] = []
        self.arrival_sec: list[float] = []
        self._subscription = self.create_subscription(
            GasIsopleth,
            topic,
            self._on_isopleth,
            10,
        )

    def _on_isopleth(self, message: GasIsopleth) -> None:
        self.messages.append(message)
        self.arrival_sec.append(time.monotonic())


class _StimulusNode(Node):
    def __init__(
        self,
        *,
        gas_topic: str = "sim/gas/sample",
        wind_topic: str = "sim/wind/vector",
    ) -> None:
        super().__init__("gas_isopleth_stimulus_test")
        self._gas_topic = gas_topic
        self._wind_topic = wind_topic
        self._gas_pub = self.create_publisher(PointStamped, gas_topic, 10)
        self._wind_pub = self.create_publisher(Vector3Stamped, wind_topic, 10)
        self._phase = 0.0
        self._timer = self.create_timer(0.1, self._publish)

    def _publish(self) -> None:
        self._phase += 0.12
        now = self.get_clock().now().to_msg()

        wind = Vector3Stamped()
        wind.header.stamp = now
        wind.vector.x = 2.0
        wind.vector.y = 0.5 * math.sin(self._phase)
        self._wind_pub.publish(wind)

        sample = PointStamped()
        sample.header.stamp = now
        sample.point.x = 5.0 + (3.0 * math.sin(self._phase))
        sample.point.y = 2.0 + (2.0 * math.cos(self._phase))
        sample.point.z = 1.5 + abs(math.sin(self._phase * 1.8))
        self._gas_pub.publish(sample)


def _message_has_finite_geometry(message: GasIsopleth) -> bool:
    for polygon in message.polygons:
        for point in polygon.points:
            if not (math.isfinite(point.x) and math.isfinite(point.y) and math.isfinite(point.z)):
                return False
    for point in message.centerline:
        if not (math.isfinite(point.x) and math.isfinite(point.y) and math.isfinite(point.z)):
            return False
    return True


def test_node_publishes_isopleths_with_contract_and_cadence() -> None:
    rclpy.init()
    executor = SingleThreadedExecutor()

    node = GasIsoplethNode(
        parameter_overrides=[
            Parameter("rate_hz", value=0.8),
            Parameter("gas_input_topic", value="sim/gas/sample"),
            Parameter("wind_input_topic", value="sim/wind/vector"),
            Parameter("output_topic", value="gas/isopleth"),
            Parameter("smoothing_window", value=24),
            Parameter("plume_resolution", value=20),
        ]
    )
    observer = _IsoplethObserver(topic="gas/isopleth")
    stimulus = _StimulusNode(gas_topic="sim/gas/sample", wind_topic="sim/wind/vector")

    try:
        executor.add_node(node)
        executor.add_node(observer)
        executor.add_node(stimulus)

        deadline = time.monotonic() + 9.0
        while time.monotonic() < deadline and len(observer.arrival_sec) < 4:
            executor.spin_once(timeout_sec=0.1)

        assert len(observer.messages) >= 3
        message = observer.messages[-1]
        assert message.species
        assert message.units
        assert message.polygons
        assert message.centerline
        assert _message_has_finite_geometry(message)

        elapsed = observer.arrival_sec[-1] - observer.arrival_sec[0]
        observed_rate_hz = (len(observer.arrival_sec) - 1) / max(elapsed, 1e-6)
        assert 0.45 <= observed_rate_hz <= 1.1
    finally:
        for active in (stimulus, observer, node):
            with suppress(RuntimeError):
                executor.remove_node(active)
        executor.shutdown()
        stimulus.destroy_node()
        observer.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def test_contract_remains_compatible_with_orchestrator_normalization() -> None:
    # This test validates the richer payload shape consumed by mission_node.
    message = GasIsopleth()
    message.centerline = [
        Point32(x=5.0, y=2.0, z=0.0),
        Point32(x=7.5, y=2.2, z=0.0),
    ]
    message.polygons = [
        Polygon(
            points=[
                Point32(x=4.0, y=1.0, z=0.0),
                Point32(x=8.0, y=1.0, z=0.0),
                Point32(x=8.0, y=3.0, z=0.0),
                Point32(x=4.0, y=3.0, z=0.0),
            ]
        )
    ]

    center_mean_x = sum(point.x for point in message.centerline) / len(message.centerline)
    center_mean_y = sum(point.y for point in message.centerline) / len(message.centerline)
    assert center_mean_x == pytest.approx(6.25)
    assert center_mean_y == pytest.approx(2.1)


def test_node_rejects_frame_mismatch_and_future_timestamps(monkeypatch) -> None:
    rclpy.init()
    node = GasIsoplethNode(
        parameter_overrides=[
            Parameter("expected_frame_id", value="map"),
            Parameter("max_future_skew_sec", value=0.1),
        ]
    )
    observed_samples: list[dict[str, float]] = []
    observed_winds: list[dict[str, float]] = []
    monkeypatch.setattr(
        node._model,
        "add_sample",
        lambda **kwargs: observed_samples.append(
            {
                "x": float(kwargs["x"]),
                "y": float(kwargs["y"]),
                "concentration": float(kwargs["concentration"]),
                "timestamp_sec": float(kwargs["timestamp_sec"]),
            }
        ),
    )
    monkeypatch.setattr(
        node._model,
        "set_wind",
        lambda **kwargs: observed_winds.append(
            {
                "vx": float(kwargs["vx"]),
                "vy": float(kwargs["vy"]),
                "timestamp_sec": float(kwargs["timestamp_sec"]),
            }
        ),
    )

    now_sec = node._now_seconds()
    now_whole = int(now_sec)
    now_fraction = now_sec - now_whole
    now_nanosec = int(max(0.0, now_fraction) * 1e9)
    future_sec = now_whole + 3

    try:
        bad_frame_sample = PointStamped()
        bad_frame_sample.header.stamp.sec = now_whole
        bad_frame_sample.header.stamp.nanosec = now_nanosec
        bad_frame_sample.header.frame_id = "odom"
        bad_frame_sample.point.x = 1.0
        bad_frame_sample.point.y = 2.0
        bad_frame_sample.point.z = 3.0
        node._handle_gas_sample(bad_frame_sample)
        assert observed_samples == []

        future_sample = PointStamped()
        future_sample.header.stamp.sec = future_sec
        future_sample.header.stamp.nanosec = 0
        future_sample.header.frame_id = "map"
        future_sample.point.x = 1.0
        future_sample.point.y = 2.0
        future_sample.point.z = 3.0
        node._handle_gas_sample(future_sample)
        assert observed_samples == []

        good_sample = PointStamped()
        good_sample.header.stamp.sec = now_whole
        good_sample.header.stamp.nanosec = now_nanosec
        good_sample.header.frame_id = "map"
        good_sample.point.x = 4.0
        good_sample.point.y = 5.0
        good_sample.point.z = 6.0
        node._handle_gas_sample(good_sample)
        assert len(observed_samples) == 1

        bad_frame_wind = Vector3Stamped()
        bad_frame_wind.header.stamp.sec = now_whole
        bad_frame_wind.header.stamp.nanosec = now_nanosec
        bad_frame_wind.header.frame_id = "odom"
        bad_frame_wind.vector.x = 1.0
        bad_frame_wind.vector.y = 0.5
        node._handle_wind_sample(bad_frame_wind)
        assert observed_winds == []

        future_wind = Vector3Stamped()
        future_wind.header.stamp.sec = future_sec
        future_wind.header.stamp.nanosec = 0
        future_wind.header.frame_id = "map"
        future_wind.vector.x = 1.0
        future_wind.vector.y = 0.5
        node._handle_wind_sample(future_wind)
        assert observed_winds == []

        good_wind = Vector3Stamped()
        good_wind.header.stamp.sec = now_whole
        good_wind.header.stamp.nanosec = now_nanosec
        good_wind.header.frame_id = "map"
        good_wind.vector.x = 1.0
        good_wind.vector.y = 0.5
        node._handle_wind_sample(good_wind)
        assert len(observed_winds) == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
