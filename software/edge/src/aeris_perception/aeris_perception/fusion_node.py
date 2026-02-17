"""ROS 2 node for spatial-temporal fusion across thermal, acoustic, and gas detections."""

from __future__ import annotations

from typing import Any
import json
import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point32
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter

from aeris_msgs.msg import (
    AcousticBearing,
    FusedDetection,
    GasIsopleth,
    ThermalHotspot,
)

from .fusion_engine import FusionConfig, FusionEngine, NormalizedDetection


class FusionNode(Node):
    """Fuse modality detections into unified confidence-ranked alerts."""

    def __init__(
        self,
        *,
        parameter_overrides: list[Parameter] | None = None,
    ) -> None:
        super().__init__("multi_modal_fusion", parameter_overrides=parameter_overrides)

        self._thermal_topic = str(
            self.declare_parameter("thermal_topic", "thermal/hotspots").value
        )
        self._acoustic_topic = str(
            self.declare_parameter("acoustic_topic", "acoustic/bearing").value
        )
        self._gas_topic = str(self.declare_parameter("gas_topic", "gas/isopleth").value)
        self._output_topic = str(
            self.declare_parameter("output_topic", "/detections/fused").value
        )

        self._frame_id = str(self.declare_parameter("frame_id", "map").value)
        self._image_center_x = float(self.declare_parameter("image_center_x", 320.0).value)
        self._image_center_y = float(self.declare_parameter("image_center_y", 240.0).value)
        self._thermal_pixels_to_meters = float(
            self.declare_parameter("thermal_pixels_to_meters", 0.1).value
        )
        self._acoustic_projection_range_m = float(
            self.declare_parameter("acoustic_projection_range_m", 35.0).value
        )
        self._expected_frame_id = str(
            self.declare_parameter("expected_frame_id", "").value
        ).strip()
        self._mission_id = str(self.declare_parameter("mission_id", "").value).strip()
        self._max_future_skew_sec = max(
            0.0, float(self.declare_parameter("max_future_skew_sec", 0.25).value)
        )

        self._fusion_config = self._load_fusion_config()
        self._engine = FusionEngine(config=self._fusion_config)

        self._publisher = self.create_publisher(FusedDetection, self._output_topic, 10)
        self._thermal_subscription = self.create_subscription(
            ThermalHotspot,
            self._thermal_topic,
            self._handle_thermal_hotspot,
            10,
        )
        self._acoustic_subscription = self.create_subscription(
            AcousticBearing,
            self._acoustic_topic,
            self._handle_acoustic_bearing,
            10,
        )
        self._gas_subscription = self.create_subscription(
            GasIsopleth,
            self._gas_topic,
            self._handle_gas_isopleth,
            10,
        )
        self.add_on_set_parameters_callback(self._handle_param_update)

    def _load_fusion_config(self) -> FusionConfig:
        return FusionConfig(
            correlation_window_sec=float(
                self.declare_parameter("correlation_window_sec", 3.0).value
            ),
            spatial_gate_m=float(self.declare_parameter("spatial_gate_m", 12.0).value),
            candidate_ttl_sec=float(
                self.declare_parameter("candidate_ttl_sec", 20.0).value
            ),
            strong_confidence_min=float(
                self.declare_parameter("strong_confidence_min", 0.75).value
            ),
            weak_confidence_min=float(
                self.declare_parameter("weak_confidence_min", 0.40).value
            ),
            max_future_skew_sec=float(
                self.declare_parameter("max_future_skew_sec", 0.25).value
            ),
        )

    def _handle_param_update(self, params: list[Parameter]) -> SetParametersResult:
        updated_frame_id = self._frame_id
        updated_expected_frame_id = self._expected_frame_id
        updated_mission_id = self._mission_id
        updated_image_center_x = self._image_center_x
        updated_image_center_y = self._image_center_y
        updated_thermal_pixels_to_meters = self._thermal_pixels_to_meters
        updated_acoustic_projection_range_m = self._acoustic_projection_range_m
        updated_max_future_skew_sec = self._max_future_skew_sec
        fusion_update = {
            "correlation_window_sec": self._fusion_config.correlation_window_sec,
            "spatial_gate_m": self._fusion_config.spatial_gate_m,
            "candidate_ttl_sec": self._fusion_config.candidate_ttl_sec,
            "strong_confidence_min": self._fusion_config.strong_confidence_min,
            "weak_confidence_min": self._fusion_config.weak_confidence_min,
            "max_future_skew_sec": self._fusion_config.max_future_skew_sec,
        }

        for parameter in params:
            try:
                if parameter.name == "frame_id":
                    updated_frame_id = str(parameter.value).strip() or "map"
                elif parameter.name == "expected_frame_id":
                    updated_expected_frame_id = str(parameter.value).strip()
                elif parameter.name == "mission_id":
                    updated_mission_id = str(parameter.value).strip()
                elif parameter.name == "image_center_x":
                    updated_image_center_x = float(parameter.value)
                elif parameter.name == "image_center_y":
                    updated_image_center_y = float(parameter.value)
                elif parameter.name == "thermal_pixels_to_meters":
                    updated_thermal_pixels_to_meters = float(parameter.value)
                elif parameter.name == "acoustic_projection_range_m":
                    updated_acoustic_projection_range_m = float(parameter.value)
                elif parameter.name == "max_future_skew_sec":
                    updated_max_future_skew_sec = max(0.0, float(parameter.value))
                    fusion_update["max_future_skew_sec"] = updated_max_future_skew_sec
                elif parameter.name in fusion_update:
                    fusion_update[parameter.name] = float(parameter.value)
                elif parameter.name in {
                    "thermal_topic",
                    "acoustic_topic",
                    "gas_topic",
                    "output_topic",
                }:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            f"{parameter.name} updates require node restart "
                            "to rebuild ROS publishers/subscriptions"
                        ),
                    )
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False, reason=f"invalid value for '{parameter.name}'"
                )

        if fusion_update["correlation_window_sec"] <= 0.0:
            return SetParametersResult(
                successful=False, reason="correlation_window_sec must be > 0"
            )
        if fusion_update["spatial_gate_m"] <= 0.0:
            return SetParametersResult(
                successful=False, reason="spatial_gate_m must be > 0"
            )
        if fusion_update["candidate_ttl_sec"] <= 0.0:
            return SetParametersResult(
                successful=False, reason="candidate_ttl_sec must be > 0"
            )
        if fusion_update["weak_confidence_min"] < 0.0:
            return SetParametersResult(
                successful=False, reason="weak_confidence_min must be >= 0"
            )
        if fusion_update["strong_confidence_min"] < fusion_update["weak_confidence_min"]:
            return SetParametersResult(
                successful=False,
                reason="strong_confidence_min must be >= weak_confidence_min",
            )

        self._frame_id = updated_frame_id
        self._expected_frame_id = updated_expected_frame_id
        self._mission_id = updated_mission_id
        self._image_center_x = updated_image_center_x
        self._image_center_y = updated_image_center_y
        self._thermal_pixels_to_meters = updated_thermal_pixels_to_meters
        self._acoustic_projection_range_m = updated_acoustic_projection_range_m
        self._max_future_skew_sec = updated_max_future_skew_sec
        self._fusion_config = FusionConfig(
            correlation_window_sec=fusion_update["correlation_window_sec"],
            spatial_gate_m=fusion_update["spatial_gate_m"],
            candidate_ttl_sec=fusion_update["candidate_ttl_sec"],
            strong_confidence_min=fusion_update["strong_confidence_min"],
            weak_confidence_min=fusion_update["weak_confidence_min"],
            max_future_skew_sec=fusion_update["max_future_skew_sec"],
        )
        self._engine = FusionEngine(config=self._fusion_config)
        return SetParametersResult(successful=True)

    def _handle_thermal_hotspot(self, message: ThermalHotspot) -> None:
        if not self._frame_is_valid(message.frame_id):
            return
        timestamp_sec = self._resolve_stamp_seconds(message.stamp)
        if timestamp_sec is None:
            return

        bbox = list(message.bbox_px)
        center_x = self._image_center_x
        center_y = self._image_center_y
        if len(bbox) >= 4:
            center_x = (float(bbox[0]) + float(bbox[2])) * 0.5
            center_y = (float(bbox[1]) + float(bbox[3])) * 0.5
        local_x = (center_x - self._image_center_x) * self._thermal_pixels_to_meters
        local_z = (center_y - self._image_center_y) * self._thermal_pixels_to_meters
        half_extent_x = 2.0 * self._thermal_pixels_to_meters
        half_extent_z = 2.0 * self._thermal_pixels_to_meters
        geometry = (
            (local_x - half_extent_x, local_z - half_extent_z),
            (local_x + half_extent_x, local_z - half_extent_z),
            (local_x + half_extent_x, local_z + half_extent_z),
            (local_x - half_extent_x, local_z + half_extent_z),
        )
        normalized = NormalizedDetection(
            modality="thermal",
            timestamp_sec=timestamp_sec,
            confidence=max(0.0, min(1.0, float(message.confidence))),
            local_x=float(local_x),
            local_z=float(local_z),
            geometry=geometry,
        )
        self._fuse_detection(normalized)

    def _handle_acoustic_bearing(self, message: AcousticBearing) -> None:
        timestamp_sec = self._resolve_stamp_seconds(message.stamp)
        if timestamp_sec is None:
            return

        bearing_rad = math.radians(float(message.bearing_deg))
        local_x = math.sin(bearing_rad) * self._acoustic_projection_range_m
        local_z = math.cos(bearing_rad) * self._acoustic_projection_range_m
        normalized = NormalizedDetection(
            modality="acoustic",
            timestamp_sec=timestamp_sec,
            confidence=max(0.0, min(1.0, float(message.confidence))),
            local_x=float(local_x),
            local_z=float(local_z),
            geometry=((0.0, 0.0), (float(local_x), float(local_z))),
        )
        self._fuse_detection(normalized)

    def _handle_gas_isopleth(self, message: GasIsopleth) -> None:
        timestamp_sec = self._resolve_stamp_seconds(message.stamp)
        if timestamp_sec is None:
            return

        points: list[tuple[float, float]] = []
        for point in message.centerline:
            points.append((float(point.x), float(point.y)))
        if not points:
            for polygon in message.polygons:
                for point in polygon.points:
                    points.append((float(point.x), float(point.y)))
        if not points:
            return

        local_x = sum(point[0] for point in points) / len(points)
        local_z = sum(point[1] for point in points) / len(points)
        if message.centerline:
            confidence = 1.0
        elif message.polygons:
            confidence = 0.7
        else:
            confidence = 0.0
        normalized = NormalizedDetection(
            modality="gas",
            timestamp_sec=timestamp_sec,
            confidence=confidence,
            local_x=float(local_x),
            local_z=float(local_z),
            geometry=tuple(points[:24]),
        )
        self._fuse_detection(normalized)

    def _fuse_detection(self, detection: NormalizedDetection) -> None:
        result = self._engine.ingest(detection, now_sec=self._now_seconds())
        if result is None:
            return

        message = FusedDetection()
        message.stamp = self._seconds_to_time(result.timestamp_sec)
        message.candidate_id = result.candidate_id
        message.mission_id = self._mission_id
        message.confidence_level = result.confidence_level
        message.confidence = float(result.confidence)
        message.source_modalities = list(result.source_modalities)
        message.local_target = Point32(
            x=float(result.local_x),
            y=float(result.local_z),
            z=0.0,
        )
        message.local_geometry = [
            Point32(x=float(x_value), y=float(z_value), z=0.0)
            for x_value, z_value in result.geometry
        ]
        message.frame_id = self._frame_id
        if len(result.geometry) >= 3:
            message.hazard_payload_json = json.dumps(
                {
                    "polygons": [
                        [
                            {"x": float(point[0]), "z": float(point[1])}
                            for point in result.geometry
                        ]
                    ]
                }
            )
        else:
            message.hazard_payload_json = ""
        self._publisher.publish(message)

    def _resolve_stamp_seconds(self, stamp) -> float | None:
        timestamp_sec = self._time_message_to_seconds(stamp)
        now_sec = self._now_seconds()
        if timestamp_sec <= 0.0:
            return now_sec
        if timestamp_sec > now_sec + self._max_future_skew_sec:
            self.get_logger().warning(
                f"Ignoring detection with future timestamp {timestamp_sec:.3f}s"
            )
            return None
        return timestamp_sec

    def _frame_is_valid(self, frame_id: str) -> bool:
        if not self._expected_frame_id:
            return True
        incoming = str(frame_id).strip()
        if incoming == self._expected_frame_id:
            return True
        self.get_logger().warning(
            f"Ignoring detection with frame_id '{incoming or '<empty>'}'; "
            f"expected '{self._expected_frame_id}'"
        )
        return False

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _time_message_to_seconds(stamp) -> float:
        sec = float(getattr(stamp, "sec", 0.0))
        nanosec = float(getattr(stamp, "nanosec", 0.0))
        return sec + (nanosec / 1e9)

    @staticmethod
    def _seconds_to_time(timestamp_sec: float):
        bounded = max(0.0, float(timestamp_sec))
        sec = int(math.floor(bounded))
        nanosec = int((bounded - sec) * 1e9)
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        message = Time()
        message.sec = sec
        message.nanosec = nanosec
        return message


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
