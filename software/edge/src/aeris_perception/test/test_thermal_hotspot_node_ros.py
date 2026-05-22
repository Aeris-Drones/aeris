import time
from types import SimpleNamespace

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
sensor_msgs = pytest.importorskip("sensor_msgs.msg")
import aeris_perception.thermal_hotspot_node as thermal_hotspot_node_module
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image

from aeris_perception.thermal_hotspot_node import ThermalHotspotNode


class _RecorderPublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


def _make_thermal_image_message(frame: np.ndarray) -> Image:
    assert frame.ndim == 2
    assert frame.dtype == np.uint16

    message = Image()
    message.height = int(frame.shape[0])
    message.width = int(frame.shape[1])
    message.encoding = "mono16"
    message.is_bigendian = 0
    message.step = int(frame.shape[1] * frame.itemsize)
    message.data = frame.tobytes()
    message.header.frame_id = "thermal_front"
    return message


def test_thermal_hotspot_node_pipeline_meets_minimum_rate_target() -> None:
    rclpy.init()
    node = ThermalHotspotNode()
    recorder = _RecorderPublisher()
    try:
        # Benchmark the full node processing callback path (decode -> detect -> msg publish)
        node._target_publish_rate_hz = 0.0
        node._publisher = recorder

        frame = np.full((120, 160), 23, dtype=np.uint16)
        frame[35:70, 60:95] = 42
        message = _make_thermal_image_message(frame)

        iterations = 120
        start = time.perf_counter()
        for _ in range(iterations):
            node._handle_thermal_image(message)
        elapsed = max(time.perf_counter() - start, 1e-6)
        fps = iterations / elapsed

        assert fps >= 5.0
        assert recorder.messages
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_rate_limiter_applies_even_when_no_hotspots_detected() -> None:
    rclpy.init()
    node = ThermalHotspotNode()
    try:
        node._target_publish_rate_hz = 1.0
        calls = 0

        def fake_image_to_temperature(_message: Image):
            nonlocal calls
            calls += 1
            return np.full((8, 8), 20.0, dtype=np.float32)

        node._image_to_temperature_celsius = fake_image_to_temperature
        node._detector.detect = lambda _frame: []
        message = _make_thermal_image_message(np.full((8, 8), 20, dtype=np.uint16))

        node._handle_thermal_image(message)
        node._handle_thermal_image(message)

        assert calls == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_publish_budget_is_consumed_only_when_hotspot_is_published(
    monkeypatch,
) -> None:
    rclpy.init()
    node = ThermalHotspotNode()
    recorder = _RecorderPublisher()
    try:
        node._target_publish_rate_hz = 2.0
        node._publisher = recorder

        times = iter((10.0, 10.2, 10.6))
        monkeypatch.setattr(
            thermal_hotspot_node_module.time,
            "monotonic",
            lambda: next(times),
        )

        detect_calls = 0

        def fake_image_to_temperature(_message: Image):
            return np.full((8, 8), 20.0, dtype=np.float32)

        def fake_detect(_frame: np.ndarray):
            nonlocal detect_calls
            detect_calls += 1
            if detect_calls == 1:
                return []
            return [
                SimpleNamespace(
                    bbox_xyxy=(1, 2, 3, 4),
                    temp_c=42.0,
                    confidence=0.9,
                )
            ]

        node._image_to_temperature_celsius = fake_image_to_temperature
        node._detector.detect = fake_detect
        message = _make_thermal_image_message(np.full((8, 8), 20, dtype=np.uint16))

        node._handle_thermal_image(message)
        assert node._last_publish_monotonic == pytest.approx(0.0)

        node._handle_thermal_image(message)
        assert detect_calls == 1
        assert recorder.messages == []

        node._handle_thermal_image(message)
        assert detect_calls == 2
        assert len(recorder.messages) == 1
        assert node._last_publish_monotonic == pytest.approx(10.6)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_rate_hz_alias_updates_target_publish_rate() -> None:
    rclpy.init()
    node = ThermalHotspotNode()
    try:
        result = node._handle_param_update(
            [Parameter("rate_hz", value=7.5)]
        )
        assert result.successful
        assert node._target_publish_rate_hz == pytest.approx(7.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()
