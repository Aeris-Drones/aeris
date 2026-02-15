"""Acoustic bearing node with deterministic multichannel localization."""

from __future__ import annotations

from typing import Any
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32MultiArray

from aeris_msgs.msg import AcousticBearing

from .acoustic_localization import estimate_acoustic_candidates


class AcousticBearingNode(Node):
    """Estimate and publish acoustic source bearings from 4-channel audio frames."""

    def __init__(self) -> None:
        super().__init__("acoustic_bearing")

        self._audio_topic = str(
            self.declare_parameter("audio_topic", "acoustic/audio").value
        )
        self._publish_topic = str(
            self.declare_parameter("publish_topic", "acoustic/bearing").value
        )
        self._rate_hz = float(self.declare_parameter("rate_hz", 1.0).value)
        self._sample_rate_hz = float(
            self.declare_parameter("sample_rate_hz", 16000.0).value
        )
        self._window_size_samples = int(
            self.declare_parameter("window_size", 2048).value
        )
        self._speed_of_sound_mps = float(
            self.declare_parameter("speed_of_sound_mps", 343.0).value
        )
        self._source_separation_deg = float(
            self.declare_parameter("source_separation_deg", 25.0).value
        )
        self._max_sources_per_cycle = int(
            self.declare_parameter("max_sources_per_cycle", 2).value
        )
        self._frame_stale_sec = float(
            self.declare_parameter("frame_stale_sec", 3.0).value
        )
        self._mic_array_id = str(self.declare_parameter("mic_array", "4ch_v2").value)
        self._vehicle_id = str(self.declare_parameter("vehicle_id", "scout_1").value)
        self._classification = str(
            self.declare_parameter("classification", "unknown").value
        )

        default_geometry = [-0.06, -0.03, 0.05, -0.04, -0.01, 0.07, 0.07, 0.06]
        self._mic_positions_m = self._parse_array_geometry(
            self.declare_parameter("array_geometry_m", default_geometry).value
        )

        self._publisher = self.create_publisher(
            AcousticBearing,
            self._publish_topic,
            10,
        )
        self._subscription = self.create_subscription(
            Float32MultiArray,
            self._audio_topic,
            self._handle_audio_frame,
            10,
        )
        self._timer = self.create_timer(self._period_sec, self._publish_next_estimate)

        self._latest_frame: np.ndarray | None = None
        self._latest_frame_monotonic = -np.inf
        self._candidate_cursor = 0
        self._last_stale_warning_monotonic = -np.inf
        self.add_on_set_parameters_callback(self._handle_param_update)

    @property
    def _period_sec(self) -> float:
        return 1.0 / max(self._rate_hz, 0.1)

    def _parse_array_geometry(self, raw_value: Any) -> np.ndarray:
        try:
            if isinstance(raw_value, str):
                tokens = [
                    token.strip() for token in raw_value.replace(";", ",").split(",")
                ]
                values = [float(token) for token in tokens if token]
            else:
                values = [float(item) for item in list(raw_value)]
        except (TypeError, ValueError):
            values = []

        if len(values) < 8 or len(values) % 2 != 0:
            values = [-0.05, -0.05, 0.05, -0.05, -0.05, 0.05, 0.05, 0.05]

        geometry = np.asarray(values, dtype=np.float64).reshape(-1, 2)
        if geometry.shape[0] < 4:
            geometry = np.asarray(
                [[-0.06, -0.03], [0.05, -0.04], [-0.01, 0.07], [0.07, 0.06]],
                dtype=np.float64,
            )
        return geometry

    def _decode_audio_frame(self, message: Float32MultiArray) -> np.ndarray | None:
        samples = np.asarray(message.data, dtype=np.float32)
        if samples.size == 0:
            return None

        channel_count = int(self._mic_positions_m.shape[0])
        if channel_count <= 1:
            return None

        frame: np.ndarray
        if len(message.layout.dim) >= 2:
            first_size = int(message.layout.dim[0].size)
            second_size = int(message.layout.dim[1].size)
            expected = first_size * second_size
            if expected <= 0 or expected > samples.size:
                return None

            trimmed = samples[:expected]
            matrix = trimmed.reshape(first_size, second_size)
            if first_size == channel_count:
                frame = matrix
            elif second_size == channel_count:
                frame = matrix.T
            else:
                return None
        else:
            if samples.size % channel_count != 0:
                return None
            frame = samples.reshape(channel_count, -1)

        if frame.shape[1] < self._window_size_samples:
            return None

        clipped = frame[:, -self._window_size_samples :]
        if not np.all(np.isfinite(clipped)):
            return None
        return clipped.astype(np.float32)

    def _handle_audio_frame(self, message: Float32MultiArray) -> None:
        frame = self._decode_audio_frame(message)
        if frame is None:
            self.get_logger().warning(
                "dropping malformed acoustic frame: invalid shape/channel metadata"
            )
            return

        self._latest_frame = frame
        self._latest_frame_monotonic = time.monotonic()

    def _publish_next_estimate(self) -> None:
        if self._latest_frame is None:
            return

        now_monotonic = time.monotonic()
        if now_monotonic - self._latest_frame_monotonic > self._frame_stale_sec:
            if now_monotonic - self._last_stale_warning_monotonic >= 5.0:
                self.get_logger().warning(
                    "acoustic input is stale; skipping bearing publication"
                )
                self._last_stale_warning_monotonic = now_monotonic
            return

        candidates = estimate_acoustic_candidates(
            frame=self._latest_frame,
            sample_rate_hz=self._sample_rate_hz,
            mic_positions_m=self._mic_positions_m,
            speed_of_sound_mps=self._speed_of_sound_mps,
            min_source_separation_deg=self._source_separation_deg,
            max_sources_per_cycle=self._max_sources_per_cycle,
        )
        if not candidates:
            return

        selected = candidates[self._candidate_cursor % len(candidates)]
        self._candidate_cursor += 1

        message = AcousticBearing()
        message.stamp = self.get_clock().now().to_msg()
        message.bearing_deg = float(selected.bearing_deg)
        message.confidence = float(selected.confidence)
        message.snr_db = float(selected.snr_db)
        message.mic_array = self._mic_array_id
        message.vehicle_id = self._vehicle_id
        message.classification = self._classification
        self._publisher.publish(message)

    def _handle_param_update(self, params: list[Parameter]) -> SetParametersResult:
        try:
            for parameter in params:
                if parameter.name == "rate_hz":
                    updated_rate_hz = float(parameter.value)
                    if updated_rate_hz <= 0.0:
                        return SetParametersResult(
                            successful=False,
                            reason="rate_hz must be > 0",
                        )
                    self._rate_hz = updated_rate_hz
                    self._timer.cancel()
                    self._timer = self.create_timer(
                        self._period_sec,
                        self._publish_next_estimate,
                    )
                elif parameter.name == "sample_rate_hz":
                    self._sample_rate_hz = max(1000.0, float(parameter.value))
                elif parameter.name == "window_size":
                    self._window_size_samples = max(16, int(parameter.value))
                elif parameter.name == "speed_of_sound_mps":
                    self._speed_of_sound_mps = max(1.0, float(parameter.value))
                elif parameter.name == "source_separation_deg":
                    self._source_separation_deg = max(1.0, float(parameter.value))
                elif parameter.name == "max_sources_per_cycle":
                    self._max_sources_per_cycle = max(1, int(parameter.value))
                elif parameter.name == "frame_stale_sec":
                    self._frame_stale_sec = max(0.1, float(parameter.value))
                elif parameter.name == "mic_array":
                    self._mic_array_id = str(parameter.value)
                elif parameter.name == "vehicle_id":
                    self._vehicle_id = str(parameter.value)
                elif parameter.name == "classification":
                    self._classification = str(parameter.value)
                elif parameter.name in {"audio_topic", "publish_topic"}:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            f"{parameter.name} requires node restart "
                            "to rebind subscriptions/publishers"
                        ),
                    )
                elif parameter.name == "array_geometry_m":
                    self._mic_positions_m = self._parse_array_geometry(parameter.value)
        except (TypeError, ValueError):
            return SetParametersResult(
                successful=False,
                reason="invalid acoustic parameter value",
            )

        return SetParametersResult(successful=True)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = AcousticBearingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
