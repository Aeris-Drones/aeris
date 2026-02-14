"""Thermal hotspot detector node backed by real image processing."""

from __future__ import annotations

import time
from collections import deque
from typing import Any

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image

from aeris_msgs.msg import ThermalHotspot

from .thermal_detection import (
    ThermalDetectionConfig,
    ThermalHotspotDetector,
)

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - cv_bridge is expected in ROS runtime
    CvBridge = None


def _decode_image_without_cv_bridge(message: Image) -> np.ndarray | None:
    width = int(message.width)
    height = int(message.height)
    if width <= 0 or height <= 0 or not message.data:
        return None

    encoding = message.encoding.strip().lower()
    expected = width * height
    data = bytes(message.data)

    if encoding in {"mono8", "8uc1"}:
        array = np.frombuffer(data, dtype=np.uint8, count=expected)
    elif encoding in {"mono16", "16uc1"}:
        array = np.frombuffer(data, dtype=np.uint16, count=expected)
    elif encoding in {"32fc1", "32fc"}:
        array = np.frombuffer(data, dtype=np.float32, count=expected)
    else:
        return None

    if array.size != expected:
        return None
    return array.reshape((height, width))


class ThermalHotspotNode(Node):
    """Subscribe to thermal images and publish derived hotspot detections."""

    def __init__(self) -> None:
        super().__init__("thermal_hotspot")
        self._bridge = CvBridge() if CvBridge is not None else None
        if self._bridge is None:
            self.get_logger().warning(
                "cv_bridge unavailable; using fallback image decoding for limited encodings."
            )

        thermal_image_topic = str(
            self.declare_parameter(
                "thermal_image_topic", "/scout1/thermal/image_raw"
            ).value
        )
        hotspot_topic = str(
            self.declare_parameter("hotspot_topic", "thermal/hotspots").value
        )
        self._target_publish_rate_hz = float(
            self.declare_parameter("target_publish_rate_hz", 10.0).value
        )
        self._temperature_scale = float(
            self.declare_parameter("temperature_scale", 1.0).value
        )
        self._temperature_offset_c = float(
            self.declare_parameter("temperature_offset_c", 0.0).value
        )
        self._frame_id_override = str(
            self.declare_parameter("frame_id_override", "").value
        )

        config = ThermalDetectionConfig(
            threshold_min_c=float(
                self.declare_parameter("threshold_min_c", 30.0).value
            ),
            threshold_max_c=float(
                self.declare_parameter("threshold_max_c", 120.0).value
            ),
            min_hotspot_area_px=int(
                self.declare_parameter("min_hotspot_area_px", 25).value
            ),
            min_temp_delta_c=float(
                self.declare_parameter("min_temp_delta_c", 4.0).value
            ),
            temporal_iou_gate=float(
                self.declare_parameter("temporal_iou_gate", 0.2).value
            ),
            temporal_history_size=int(
                self.declare_parameter("temporal_history_size", 4).value
            ),
            max_hotspots_per_frame=int(
                self.declare_parameter("max_hotspots_per_frame", 5).value
            ),
        )
        self._detector = ThermalHotspotDetector(config)

        self._publisher = self.create_publisher(ThermalHotspot, hotspot_topic, 10)
        self._subscription = self.create_subscription(
            Image, thermal_image_topic, self._handle_thermal_image, 10
        )
        self._last_publish_monotonic = 0.0
        self._process_times = deque(maxlen=60)
        self._last_rate_log_monotonic = time.monotonic()
        self.add_on_set_parameters_callback(self._handle_param_update)

    def _handle_param_update(self, params: list[Parameter]) -> SetParametersResult:
        config = self._detector.config
        update = {
            "threshold_min_c": config.threshold_min_c,
            "threshold_max_c": config.threshold_max_c,
            "min_hotspot_area_px": config.min_hotspot_area_px,
            "min_temp_delta_c": config.min_temp_delta_c,
            "min_aspect_ratio": config.min_aspect_ratio,
            "max_aspect_ratio": config.max_aspect_ratio,
            "temporal_iou_gate": config.temporal_iou_gate,
            "temporal_history_size": config.temporal_history_size,
            "max_hotspots_per_frame": config.max_hotspots_per_frame,
            "local_background_margin_px": config.local_background_margin_px,
        }
        for parameter in params:
            try:
                if parameter.name in update:
                    value = parameter.value
                    if isinstance(update[parameter.name], int):
                        update[parameter.name] = int(value)
                    else:
                        update[parameter.name] = float(value)
                    continue
                if parameter.name == "target_publish_rate_hz":
                    self._target_publish_rate_hz = float(parameter.value)
                    continue
                if parameter.name == "temperature_scale":
                    self._temperature_scale = float(parameter.value)
                    continue
                if parameter.name == "temperature_offset_c":
                    self._temperature_offset_c = float(parameter.value)
                    continue
                if parameter.name == "frame_id_override":
                    self._frame_id_override = str(parameter.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False,
                    reason=f"invalid value for parameter '{parameter.name}'",
                )

        if update["threshold_min_c"] >= update["threshold_max_c"]:
            return SetParametersResult(
                successful=False,
                reason="threshold_min_c must be < threshold_max_c",
            )
        if update["min_hotspot_area_px"] <= 0:
            return SetParametersResult(
                successful=False, reason="min_hotspot_area_px must be > 0"
            )
        if update["min_temp_delta_c"] <= 0.0:
            return SetParametersResult(
                successful=False, reason="min_temp_delta_c must be > 0"
            )

        self._detector.update_config(ThermalDetectionConfig(**update))
        return SetParametersResult(successful=True)

    def _image_to_temperature_celsius(self, message: Image) -> np.ndarray | None:
        image_array: np.ndarray | None
        if self._bridge is not None:
            try:
                image_array = np.asarray(
                    self._bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
                )
            except Exception as error:  # pragma: no cover - ROS/cv_bridge runtime path
                self.get_logger().warning(
                    f"failed to decode thermal image with cv_bridge: {error}"
                )
                return None
        else:
            image_array = _decode_image_without_cv_bridge(message)
            if image_array is None:
                self.get_logger().warning(
                    f"unsupported thermal image encoding '{message.encoding}' without cv_bridge"
                )
                return None

        return (image_array.astype(np.float32) * self._temperature_scale) + float(
            self._temperature_offset_c
        )

    def _extract_stamp(self, message: Image):
        if message.header.stamp.sec > 0 or message.header.stamp.nanosec > 0:
            return message.header.stamp
        return self.get_clock().now().to_msg()

    def _handle_thermal_image(self, message: Image) -> None:
        now = time.monotonic()
        if self._target_publish_rate_hz > 0.0:
            min_interval = 1.0 / self._target_publish_rate_hz
            if now - self._last_publish_monotonic < min_interval:
                return

        frame_celsius = self._image_to_temperature_celsius(message)
        if frame_celsius is None:
            return

        detections = self._detector.detect(frame_celsius)
        if not detections:
            return

        stamp = self._extract_stamp(message)
        frame_id = self._frame_id_override or message.header.frame_id or "thermal_front"
        for detection in detections:
            output = ThermalHotspot()
            output.stamp = stamp
            output.bbox_px = [
                int(detection.bbox_xyxy[0]),
                int(detection.bbox_xyxy[1]),
                int(detection.bbox_xyxy[2]),
                int(detection.bbox_xyxy[3]),
            ]
            output.temp_c = float(detection.temp_c)
            output.confidence = float(detection.confidence)
            output.frame_id = frame_id
            self._publisher.publish(output)

        self._last_publish_monotonic = now
        self._process_times.append(now)
        if now - self._last_rate_log_monotonic >= 5.0 and len(self._process_times) > 2:
            elapsed = self._process_times[-1] - self._process_times[0]
            if elapsed > 0.0:
                fps = (len(self._process_times) - 1) / elapsed
                self.get_logger().info(f"thermal detection publish rate {fps:.2f} fps")
            self._last_rate_log_monotonic = now


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = ThermalHotspotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
