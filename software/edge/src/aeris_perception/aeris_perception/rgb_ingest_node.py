"""ROS 2 node that publishes RGB frames and source metadata for Halo."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Callable

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .rgb_dataset import RgbDatasetCaptureConfig, RgbFrameCaptureWriter
from .rgb_ingest import (
    RgbFrame,
    RgbReadableSource,
    RgbSourceConfig,
    RgbSourceError,
    build_rgb_frame_source,
)


class RgbIngestNode(Node):
    """Publish RGB frames from a configured live or replay source."""

    def __init__(
        self,
        *,
        parameter_overrides: list[Parameter] | None = None,
        source_factory: Callable[[RgbSourceConfig], RgbReadableSource] | None = None,
        capture_writer_factory: (
            Callable[[RgbDatasetCaptureConfig], RgbFrameCaptureWriter] | None
        ) = None,
    ) -> None:
        super().__init__("rgb_ingest", parameter_overrides=parameter_overrides)

        self._source_factory = source_factory or build_rgb_frame_source
        self._capture_writer_factory = capture_writer_factory or RgbFrameCaptureWriter
        self._image_topic = str(
            self.declare_parameter("image_topic", "rgb/image_raw").value
        )
        self._metadata_topic = str(
            self.declare_parameter("metadata_topic", "rgb/source_metadata").value
        )
        self._publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 15.0).value
        )
        self._source_type = str(self.declare_parameter("source_type", "camera").value)
        self._source_uri = str(self.declare_parameter("source_uri", "0").value)
        self._frame_id = str(self.declare_parameter("frame_id", "halo_rgb").value)
        self._source_name = str(self.declare_parameter("source_name", "").value)
        self._loop_replay = bool(
            self.declare_parameter("loop_replay", False).value
        )
        self._capture_enabled = bool(
            self.declare_parameter("capture_enabled", False).value
        )
        self._capture_output_dir = str(
            self.declare_parameter("capture_output_dir", "").value
        )
        self._capture_manifest_path = str(
            self.declare_parameter("capture_manifest_path", "").value
        )
        self._capture_image_format = str(
            self.declare_parameter("capture_image_format", "png").value
        )
        self._capture_reason = str(
            self.declare_parameter("capture_reason", "continuous_capture").value
        )
        self._ros_stamp_provider = lambda: self.get_clock().now().to_msg()

        if self._publish_rate_hz <= 0.0:
            raise RgbSourceError("publish_rate_hz must be > 0 for RGB ingest")

        self._source = self._source_factory(self._build_source_config())
        self._capture_writer = self._build_capture_writer()
        self._image_publisher = self.create_publisher(Image, self._image_topic, 10)
        self._metadata_publisher = self.create_publisher(
            String, self._metadata_topic, 10
        )
        self._timer = self.create_timer(
            1.0 / self._publish_rate_hz, self._publish_next_frame
        )
        self._source_exhausted = False
        self.add_on_set_parameters_callback(self._handle_param_update)

    def destroy_node(self) -> bool:
        self._source.close()
        return super().destroy_node()

    def _build_source_config(self) -> RgbSourceConfig:
        return RgbSourceConfig(
            source_type=self._source_type,
            source_uri=self._source_uri,
            frame_id=self._frame_id,
            source_name=self._source_name,
            loop_replay=self._loop_replay,
        )

    def _build_capture_writer(self) -> RgbFrameCaptureWriter | None:
        if not self._capture_enabled:
            return None

        output_dir = self._capture_output_dir.strip()
        if not output_dir:
            raise RgbSourceError(
                "capture_output_dir must be configured when RGB capture is enabled"
            )

        manifest_path = self._capture_manifest_path.strip()
        if not manifest_path:
            manifest_path = str(Path(output_dir) / "manifest.jsonl")

        return self._capture_writer_factory(
            RgbDatasetCaptureConfig(
                output_dir=output_dir,
                manifest_path=manifest_path,
                image_format=self._capture_image_format,
                capture_reason=self._capture_reason,
            )
        )

    def _handle_param_update(self, params: list[Parameter]) -> SetParametersResult:
        updated_rate_hz = self._publish_rate_hz

        for parameter in params:
            try:
                if parameter.name == "publish_rate_hz":
                    updated_rate_hz = float(parameter.value)
                    continue
                if parameter.name in {
                    "image_topic",
                    "metadata_topic",
                    "source_type",
                    "source_uri",
                    "frame_id",
                    "source_name",
                    "loop_replay",
                    "capture_enabled",
                    "capture_output_dir",
                    "capture_manifest_path",
                    "capture_image_format",
                    "capture_reason",
                }:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            f"{parameter.name} updates require node restart "
                            "to rebuild the RGB ingest source or publishers"
                        ),
                    )
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False,
                    reason=f"invalid value for '{parameter.name}'",
                )

        if updated_rate_hz <= 0.0:
            return SetParametersResult(
                successful=False, reason="publish_rate_hz must be > 0"
            )

        if updated_rate_hz != self._publish_rate_hz:
            self._publish_rate_hz = updated_rate_hz
            self._timer.cancel()
            self._timer = self.create_timer(
                1.0 / self._publish_rate_hz, self._publish_next_frame
            )

        return SetParametersResult(successful=True)

    def _publish_next_frame(self) -> None:
        if self._source_exhausted:
            return

        try:
            frame = self._source.read()
        except RgbSourceError as error:
            self._source_exhausted = True
            self._timer.cancel()
            self.get_logger().error(f"RGB ingest stopped: {error}")
            return

        if frame is None:
            if self._source.config.normalized_source_type == "camera":
                self.get_logger().debug("RGB camera frame read missed; waiting for next frame.")
                return
            self._source_exhausted = True
            self._timer.cancel()
            self.get_logger().info(
                f"RGB {self._source.config.normalized_source_type} source exhausted."
            )
            return

        if self._capture_writer is not None:
            try:
                self._capture_writer.capture(frame)
            except RgbSourceError as error:
                self._source_exhausted = True
                self._timer.cancel()
                self.get_logger().error(f"RGB capture stopped: {error}")
                return

        self._image_publisher.publish(self._rgb_frame_to_image_message(frame))
        self._metadata_publisher.publish(String(data=_metadata_payload_json(frame)))

    def _rgb_frame_to_image_message(self, frame: RgbFrame) -> Image:
        return _rgb_frame_to_image_message(frame, self._ros_stamp_provider())


def _rgb_frame_to_image_message(frame: RgbFrame, ros_stamp) -> Image:
    image = frame.image_bgr
    if image.dtype != np.uint8:
        raise RgbSourceError("RGB ingest frames must be uint8 BGR images")
    if image.ndim != 3 or image.shape[2] != 3:
        raise RgbSourceError("RGB ingest frames must use HxWx3 BGR layout")

    message = Image()
    message.header.stamp = ros_stamp
    message.header.frame_id = frame.metadata.frame_id
    message.height = int(image.shape[0])
    message.width = int(image.shape[1])
    message.encoding = "bgr8"
    message.is_bigendian = 0
    message.step = int(image.shape[1] * image.shape[2] * image.itemsize)
    message.data = image.tobytes()
    return message


def _metadata_payload_json(frame: RgbFrame) -> str:
    payload = frame.metadata.to_dict()
    payload["height"] = int(frame.image_bgr.shape[0])
    payload["width"] = int(frame.image_bgr.shape[1])
    payload["channels"] = int(frame.image_bgr.shape[2])
    return json.dumps(payload, separators=(",", ":"), sort_keys=True)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RgbIngestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C is expected; cleanup runs in finally.
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
