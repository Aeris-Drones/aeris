from __future__ import annotations

import json

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("sensor_msgs.msg")
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_msgs.msg import String

from aeris_perception.rgb_ingest import RgbFrame, RgbFrameMetadata
from aeris_perception.rgb_ingest_node import RgbIngestNode


class _RecorderPublisher:
    def __init__(self) -> None:
        self.messages: list[object] = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


class _StaticSource:
    def __init__(self, frames: list[RgbFrame]) -> None:
        self._frames = list(frames)
        self.closed = False

    def read(self) -> RgbFrame | None:
        if not self._frames:
            return None
        return self._frames.pop(0)

    def close(self) -> None:
        self.closed = True


def test_rgb_ingest_node_publishes_image_and_metadata_payload() -> None:
    rclpy.init()
    source = _StaticSource(
        [
            RgbFrame(
                image_bgr=np.full((3, 5, 3), 17, dtype=np.uint8),
                metadata=RgbFrameMetadata(
                    source_type="replay",
                    source_name="halo_replay",
                    source_uri="/tmp/halo.mp4",
                    frame_id="halo_rgb",
                    frame_index=4,
                    monotonic_timestamp_ns=1_250_000_000,
                    source_timestamp_ns=1_200_000_000,
                    replayed=True,
                ),
            )
        ]
    )
    node = RgbIngestNode(
        parameter_overrides=[
            Parameter("source_type", value="replay"),
            Parameter("source_uri", value="/tmp/halo.mp4"),
            Parameter("frame_id", value="halo_rgb"),
        ],
        source_factory=lambda _config: source,
    )
    image_recorder = _RecorderPublisher()
    metadata_recorder = _RecorderPublisher()

    try:
        node._timer.cancel()
        node._image_publisher = image_recorder
        node._metadata_publisher = metadata_recorder

        node._publish_next_frame()

        assert len(image_recorder.messages) == 1
        assert len(metadata_recorder.messages) == 1

        image_message = image_recorder.messages[0]
        metadata_message = metadata_recorder.messages[0]
        assert isinstance(image_message, Image)
        assert isinstance(metadata_message, String)
        assert image_message.header.frame_id == "halo_rgb"
        assert image_message.header.stamp.sec == 1
        assert image_message.header.stamp.nanosec == 250_000_000
        assert image_message.height == 3
        assert image_message.width == 5
        assert image_message.encoding == "bgr8"

        payload = json.loads(metadata_message.data)
        assert payload["frame_index"] == 4
        assert payload["monotonic_timestamp_ns"] == 1_250_000_000
        assert payload["source_timestamp_ns"] == 1_200_000_000
        assert payload["replayed"] is True
        assert payload["height"] == 3
        assert payload["width"] == 5
        assert payload["channels"] == 3
    finally:
        node.destroy_node()
        rclpy.shutdown()
