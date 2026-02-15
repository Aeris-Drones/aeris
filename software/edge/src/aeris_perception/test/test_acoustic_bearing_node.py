import math
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("aeris_msgs.msg")
std_msgs = pytest.importorskip("std_msgs.msg")
from aeris_msgs.msg import AcousticBearing
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from aeris_perception.acoustic_audio_sim_node import AcousticAudioSimNode
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
        MultiArrayDimension(label="channels", size=channels, stride=samples),
        MultiArrayDimension(label="samples", size=samples, stride=1),
    ]
    message.data = frame.astype(np.float32).reshape(-1).tolist()
    return message


def _split_frame_messages(frame: np.ndarray, chunk_samples: int) -> list[Float32MultiArray]:
    _, total_samples = frame.shape
    chunk = max(1, int(chunk_samples))
    messages: list[Float32MultiArray] = []
    for start in range(0, total_samples, chunk):
        messages.append(_make_message(frame[:, start : start + chunk]))
    return messages


class _BearingObserver(Node):
    def __init__(self) -> None:
        super().__init__("acoustic_bearing_observer_test")
        self.messages: list[AcousticBearing] = []
        self.arrival_sec: list[float] = []
        self._subscription = self.create_subscription(
            AcousticBearing,
            "acoustic/bearing",
            self._on_bearing,
            10,
        )

    def _on_bearing(self, message: AcousticBearing) -> None:
        self.messages.append(message)
        self.arrival_sec.append(time.monotonic())


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


def test_node_buffers_chunked_audio_frames_until_window_complete() -> None:
    rclpy.init()
    node = AcousticBearingNode()
    recorder = _RecorderPublisher()
    try:
        node._publisher = recorder
        frame = synthesize_planar_wave(
            mic_positions_m=node._mic_positions_m,
            sample_rate_hz=node._sample_rate_hz,
            source_bearings_deg=[95.0],
            frequencies_hz=[520.0],
            amplitudes=[1.0],
            window_size_samples=node._window_size_samples,
            speed_of_sound_mps=node._speed_of_sound_mps,
        )
        messages = _split_frame_messages(frame, node._window_size_samples // 4)
        assert len(messages) >= 4

        for index, message in enumerate(messages):
            node._handle_audio_frame(message)
            node._publish_next_estimate()
            if index < len(messages) - 1:
                assert not recorder.messages

        assert recorder.messages
        latest = recorder.messages[-1]
        assert _circular_delta_deg(float(latest.bearing_deg), 95.0) <= 12.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_node_decoder_respects_multiarray_data_offset() -> None:
    rclpy.init()
    node = AcousticBearingNode()
    try:
        frame = synthesize_planar_wave(
            mic_positions_m=node._mic_positions_m,
            sample_rate_hz=node._sample_rate_hz,
            source_bearings_deg=[110.0],
            frequencies_hz=[530.0],
            amplitudes=[1.0],
            window_size_samples=node._window_size_samples,
            speed_of_sound_mps=node._speed_of_sound_mps,
        )
        message = _make_message(frame)
        prefix = np.linspace(-0.5, 0.5, 17, dtype=np.float32).tolist()
        message.layout.data_offset = len(prefix)
        message.data = prefix + message.data

        decoded = node._decode_audio_frame(message)
        assert decoded is not None
        assert decoded.shape == frame.shape
        np.testing.assert_allclose(decoded, frame.astype(np.float32), atol=1e-6, rtol=0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_node_parameter_update_is_atomic_on_failure() -> None:
    rclpy.init()
    node = AcousticBearingNode()
    try:
        initial_rate_hz = node._rate_hz
        initial_sample_rate_hz = node._sample_rate_hz
        initial_window_size = node._window_size_samples

        result = node.set_parameters_atomically(
            [
                Parameter("sample_rate_hz", value=32000.0),
                Parameter("window_size", value=4096),
                Parameter("rate_hz", value=0.0),
            ]
        )
        assert not result.successful
        assert node._rate_hz == initial_rate_hz
        assert node._sample_rate_hz == initial_sample_rate_hz
        assert node._window_size_samples == initial_window_size
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_audio_sim_node_applies_runtime_parameter_updates() -> None:
    rclpy.init()
    node = AcousticAudioSimNode()
    try:
        results = node.set_parameters(
            [
                Parameter("publish_rate_hz", value=7.0),
                Parameter("array_geometry_m", value=[0.0, 0.0, 0.2, 0.0, 0.0, 0.2, 0.2, 0.2]),
                Parameter("source_bearings_deg", value=[15.0, 195.0]),
                Parameter("source_frequencies_hz", value=[300.0, 650.0]),
                Parameter("source_amplitudes", value=[1.0, 0.5]),
            ]
        )
        assert all(result.successful for result in results)
        assert node._publish_rate_hz == pytest.approx(7.0)
        assert node._mic_positions_m.shape == (4, 2)
        assert node._source_bearings_deg == [15.0, 195.0]
        assert node._source_frequencies_hz == [300.0, 650.0]
        assert node._source_amplitudes == [1.0, 0.5]

        restart_required = node.set_parameters(
            [
                Parameter("output_topic", value="acoustic/alt_audio"),
            ]
        )
        assert len(restart_required) == 1
        assert not restart_required[0].successful
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_audio_sim_node_uses_row_major_multiarray_strides() -> None:
    rclpy.init()
    node = AcousticAudioSimNode()
    recorder = _RecorderPublisher()
    try:
        node._publisher = recorder
        node._publish_frame()
        assert recorder.messages
        message = recorder.messages[-1]
        assert len(message.layout.dim) >= 2
        channel_dim = message.layout.dim[0]
        sample_dim = message.layout.dim[1]
        assert channel_dim.stride == sample_dim.size
        assert sample_dim.stride == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_pipeline_publish_rate_is_near_one_hz() -> None:
    rclpy.init()
    executor = SingleThreadedExecutor()
    sim_node = AcousticAudioSimNode()
    bearing_node = AcousticBearingNode()
    observer = _BearingObserver()
    try:
        sim_results = sim_node.set_parameters(
            [
                Parameter("publish_rate_hz", value=20.0),
                Parameter("window_size", value=256),
            ]
        )
        bearing_results = bearing_node.set_parameters(
            [
                Parameter("rate_hz", value=1.0),
                Parameter("window_size", value=1024),
                Parameter("frame_stale_sec", value=2.5),
            ]
        )
        assert all(result.successful for result in sim_results)
        assert all(result.successful for result in bearing_results)

        executor.add_node(sim_node)
        executor.add_node(bearing_node)
        executor.add_node(observer)

        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline and len(observer.arrival_sec) < 4:
            executor.spin_once(timeout_sec=0.1)

        assert len(observer.arrival_sec) >= 3
        duration = observer.arrival_sec[-1] - observer.arrival_sec[0]
        observed_rate_hz = (len(observer.arrival_sec) - 1) / max(duration, 1e-6)
        assert 0.7 <= observed_rate_hz <= 1.3
    finally:
        for node in (observer, bearing_node, sim_node):
            try:
                executor.remove_node(node)
            except Exception as exc:
                print(f"Ignoring exception while removing node from executor: {exc}")
        executor.shutdown()
        observer.destroy_node()
        bearing_node.destroy_node()
        sim_node.destroy_node()
        rclpy.shutdown()
