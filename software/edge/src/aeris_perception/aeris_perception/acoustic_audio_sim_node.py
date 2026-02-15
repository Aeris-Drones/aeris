"""Simulation helper that publishes deterministic multichannel acoustic frames."""

from __future__ import annotations

from typing import Any

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from .acoustic_localization import synthesize_planar_wave


class AcousticAudioSimNode(Node):
    """Publish synthetic 4-channel audio frames for acoustic pipeline smoke tests."""

    def __init__(self) -> None:
        super().__init__("acoustic_audio_sim")

        self._output_topic = str(
            self.declare_parameter("output_topic", "acoustic/audio").value
        )
        self._publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 5.0).value
        )
        self._sample_rate_hz = float(
            self.declare_parameter("sample_rate_hz", 16000.0).value
        )
        self._window_size = int(self.declare_parameter("window_size", 2048).value)
        self._speed_of_sound_mps = float(
            self.declare_parameter("speed_of_sound_mps", 343.0).value
        )

        default_geometry = [-0.06, -0.03, 0.05, -0.04, -0.01, 0.07, 0.07, 0.06]
        self._mic_positions_m = self._parse_array_geometry(
            self.declare_parameter("array_geometry_m", default_geometry).value
        )

        default_source_bearings = [35.0, 210.0]
        default_source_frequencies = [430.0, 760.0]
        default_source_amplitudes = [1.0, 0.7]
        self._source_bearings_deg = self._parse_float_sequence(
            self.declare_parameter("source_bearings_deg", default_source_bearings).value,
            default_source_bearings,
        )
        self._source_frequencies_hz = self._parse_float_sequence(
            self.declare_parameter("source_frequencies_hz", default_source_frequencies).value,
            default_source_frequencies,
        )
        self._source_amplitudes = self._parse_float_sequence(
            self.declare_parameter("source_amplitudes", default_source_amplitudes).value,
            default_source_amplitudes,
        )

        self._publisher = self.create_publisher(Float32MultiArray, self._output_topic, 10)
        self._elapsed_sec = 0.0
        self._timer = self.create_timer(self._period_sec, self._publish_frame)
        self.add_on_set_parameters_callback(self._handle_param_update)

    @property
    def _period_sec(self) -> float:
        return 1.0 / max(self._publish_rate_hz, 0.1)

    def _parse_array_geometry(self, raw_value: Any) -> np.ndarray:
        if isinstance(raw_value, str):
            tokens = [token.strip() for token in raw_value.replace(";", ",").split(",")]
            values = [float(token) for token in tokens if token]
        else:
            values = [float(item) for item in list(raw_value)]

        if len(values) < 8 or len(values) % 2 != 0:
            raise ValueError("array_geometry_m must contain at least 4 (x,z) pairs")

        geometry = np.asarray(values, dtype=np.float64).reshape(-1, 2)
        if geometry.shape[0] < 4:
            raise ValueError("array_geometry_m must describe at least four microphones")
        return geometry

    def _parse_float_sequence(
        self,
        value: Any,
        fallback: list[float],
    ) -> list[float]:
        try:
            if isinstance(value, str):
                tokens = [token.strip() for token in value.replace(";", ",").split(",")]
                parsed = [float(token) for token in tokens if token]
            else:
                parsed = [float(item) for item in list(value)]
        except (TypeError, ValueError):
            parsed = []

        if not parsed:
            return list(fallback)
        return parsed

    def _normalized_source_configuration(self) -> tuple[list[float], list[float], list[float]]:
        source_count = min(
            len(self._source_bearings_deg),
            len(self._source_frequencies_hz),
            len(self._source_amplitudes),
        )
        if source_count <= 0:
            return [45.0], [500.0], [1.0]
        return (
            self._source_bearings_deg[:source_count],
            self._source_frequencies_hz[:source_count],
            self._source_amplitudes[:source_count],
        )

    def _publish_frame(self) -> None:
        bearings, frequencies, amplitudes = self._normalized_source_configuration()

        frame = synthesize_planar_wave(
            mic_positions_m=self._mic_positions_m,
            sample_rate_hz=self._sample_rate_hz,
            source_bearings_deg=bearings,
            frequencies_hz=frequencies,
            amplitudes=amplitudes,
            window_size_samples=self._window_size,
            speed_of_sound_mps=self._speed_of_sound_mps,
            time_offset_sec=self._elapsed_sec,
        )

        message = Float32MultiArray()
        channel_count, sample_count = frame.shape
        message.layout.dim = [
            MultiArrayDimension(
                label="channels",
                size=int(channel_count),
                stride=int(channel_count * sample_count),
            ),
            MultiArrayDimension(
                label="samples",
                size=int(sample_count),
                stride=int(sample_count),
            ),
        ]
        message.data = frame.reshape(-1).astype(np.float32).tolist()
        self._publisher.publish(message)

        self._elapsed_sec += float(sample_count) / max(self._sample_rate_hz, 1.0)

    def _handle_param_update(self, params: list[Parameter]) -> SetParametersResult:
        try:
            for parameter in params:
                if parameter.name == "publish_rate_hz":
                    new_rate = float(parameter.value)
                    if new_rate <= 0.0:
                        return SetParametersResult(
                            successful=False,
                            reason="publish_rate_hz must be > 0",
                        )
                    self._publish_rate_hz = new_rate
                    self._timer.cancel()
                    self._timer = self.create_timer(self._period_sec, self._publish_frame)
                elif parameter.name == "window_size":
                    self._window_size = max(16, int(parameter.value))
                elif parameter.name == "sample_rate_hz":
                    self._sample_rate_hz = max(1000.0, float(parameter.value))
                elif parameter.name == "speed_of_sound_mps":
                    self._speed_of_sound_mps = max(1.0, float(parameter.value))
        except (TypeError, ValueError):
            return SetParametersResult(
                successful=False,
                reason="invalid acoustic audio simulation parameter value",
            )

        return SetParametersResult(successful=True)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = AcousticAudioSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
