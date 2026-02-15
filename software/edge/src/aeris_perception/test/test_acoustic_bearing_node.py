import math

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
std_msgs = pytest.importorskip("std_msgs.msg")
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from aeris_perception.acoustic_bearing_node import AcousticBearingNode
from aeris_perception.acoustic_localization import synthesize_planar_wave


class _RecorderPublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


def _circular_delta_deg(a: float, b: float) -> float:
    return abs(((a - b + 180.0) % 360.0) - 180.0)


def _make_message(frame: np.ndarray) -> Float32MultiArray:
    message = Float32MultiArray()
    channels, samples = frame.shape
    message.layout.dim = [
        MultiArrayDimension(label="channels", size=channels, stride=channels * samples),
        MultiArrayDimension(label="samples", size=samples, stride=samples),
    ]
    message.data = frame.astype(np.float32).reshape(-1).tolist()
    return message


def test_node_publishes_contract_fields_with_bounded_metrics() -> None:
    rclpy.init()
    node = AcousticBearingNode()
    recorder = _RecorderPublisher()
    try:
        node._publisher = recorder
        node._rate_hz = 1.0

        frame = synthesize_planar_wave(
            mic_positions_m=node._mic_positions_m,
            sample_rate_hz=node._sample_rate_hz,
            source_bearings_deg=[75.0],
            frequencies_hz=[550.0],
            amplitudes=[1.0],
            window_size_samples=node._window_size_samples,
            speed_of_sound_mps=node._speed_of_sound_mps,
        )
        node._handle_audio_frame(_make_message(frame))
        node._publish_next_estimate()

        assert recorder.messages
        message = recorder.messages[-1]
        assert _circular_delta_deg(float(message.bearing_deg), 75.0) <= 12.0
        assert 0.0 <= float(message.confidence) <= 1.0
        assert math.isfinite(float(message.snr_db))
        assert message.vehicle_id
        assert message.classification
        assert message.mic_array
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_node_rotates_between_multiple_candidates_over_time() -> None:
    rclpy.init()
    node = AcousticBearingNode()
    recorder = _RecorderPublisher()
    try:
        node._publisher = recorder

        frame = synthesize_planar_wave(
            mic_positions_m=node._mic_positions_m,
            sample_rate_hz=node._sample_rate_hz,
            source_bearings_deg=[30.0, 210.0],
            frequencies_hz=[400.0, 760.0],
            amplitudes=[1.0, 0.8],
            window_size_samples=node._window_size_samples,
            speed_of_sound_mps=node._speed_of_sound_mps,
        )
        node._handle_audio_frame(_make_message(frame))
        node._publish_next_estimate()
        node._publish_next_estimate()

        assert len(recorder.messages) >= 2
        first = float(recorder.messages[-2].bearing_deg)
        second = float(recorder.messages[-1].bearing_deg)
        assert _circular_delta_deg(first, second) >= 20.0
    finally:
        node.destroy_node()
        rclpy.shutdown()
