"""Reading-driven gas plume node with wind-aware isopleth publishing."""

from __future__ import annotations

from typing import Any

import rclpy
from geometry_msgs.msg import Point32, PointStamped, Polygon, Vector3Stamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter

from aeris_msgs.msg import GasIsopleth

from .gas_plume_model import GasPlumeModel


class GasIsoplethNode(Node):
    """Publish gas isopleths from gas concentration and wind input streams."""

    def __init__(
        self,
        *,
        parameter_overrides: list[Parameter] | None = None,
    ) -> None:
        super().__init__("gas_isopleth", parameter_overrides=parameter_overrides)

        self._rate_hz = float(self.declare_parameter("rate_hz", 0.5).value)
        self._species = str(self.declare_parameter("species", "VOC").value)
        self._units = str(self.declare_parameter("units", "ppm").value)

        self._gas_input_topic = str(
            self.declare_parameter("gas_input_topic", "sim/gas/sample").value
        )
        self._wind_input_topic = str(
            self.declare_parameter("wind_input_topic", "sim/wind/vector").value
        )
        self._output_topic = str(
            self.declare_parameter("output_topic", "gas/isopleth").value
        )

        self._smoothing_window = int(self.declare_parameter("smoothing_window", 30).value)
        self._plume_resolution = int(self.declare_parameter("plume_resolution", 24).value)
        self._sample_stale_sec = float(self.declare_parameter("sample_stale_sec", 4.0).value)
        self._wind_stale_sec = float(self.declare_parameter("wind_stale_sec", 2.0).value)
        self._hold_last_sec = float(self.declare_parameter("hold_last_sec", 5.0).value)
        self._min_wind_speed_mps = float(
            self.declare_parameter("min_wind_speed_mps", 0.1).value
        )
        self._max_extent_m = float(self.declare_parameter("max_extent_m", 250.0).value)

        self._model = self._create_model()

        self._publisher = self.create_publisher(GasIsopleth, self._output_topic, 10)
        self._gas_subscription = self.create_subscription(
            PointStamped,
            self._gas_input_topic,
            self._handle_gas_sample,
            10,
        )
        self._wind_subscription = self.create_subscription(
            Vector3Stamped,
            self._wind_input_topic,
            self._handle_wind_sample,
            10,
        )
        self._timer = self.create_timer(self._period_sec, self._publish_sample)

        self.add_on_set_parameters_callback(self._handle_param_update)

    @property
    def _period_sec(self) -> float:
        return 1.0 / max(self._rate_hz, 0.1)

    def _create_model(self) -> GasPlumeModel:
        return GasPlumeModel(
            smoothing_window=self._smoothing_window,
            plume_resolution=self._plume_resolution,
            sample_stale_sec=self._sample_stale_sec,
            wind_stale_sec=self._wind_stale_sec,
            hold_last_sec=self._hold_last_sec,
            min_wind_speed_mps=self._min_wind_speed_mps,
            max_extent_m=self._max_extent_m,
        )

    def _handle_param_update(self, params: list[Parameter]) -> SetParametersResult:
        rate_hz = self._rate_hz
        species = self._species
        units = self._units

        smoothing_window = self._smoothing_window
        plume_resolution = self._plume_resolution
        sample_stale_sec = self._sample_stale_sec
        wind_stale_sec = self._wind_stale_sec
        hold_last_sec = self._hold_last_sec
        min_wind_speed_mps = self._min_wind_speed_mps
        max_extent_m = self._max_extent_m

        requires_timer_reset = False
        requires_model_reset = False

        for parameter in params:
            if parameter.name == "rate_hz":
                try:
                    rate_candidate = float(parameter.value)
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False,
                        reason="rate_hz must be numeric",
                    )
                if rate_candidate <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="rate_hz must be > 0",
                    )
                rate_hz = rate_candidate
                requires_timer_reset = True
            elif parameter.name == "species":
                species = str(parameter.value).strip() or "VOC"
            elif parameter.name == "units":
                units = str(parameter.value).strip() or "ppm"
            elif parameter.name == "smoothing_window":
                try:
                    smoothing_window = max(3, int(parameter.value))
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False,
                        reason="smoothing_window must be an integer",
                    )
                requires_model_reset = True
            elif parameter.name == "plume_resolution":
                try:
                    plume_resolution = max(8, int(parameter.value))
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False,
                        reason="plume_resolution must be an integer",
                    )
                requires_model_reset = True
            elif parameter.name == "sample_stale_sec":
                try:
                    sample_stale_sec = max(0.1, float(parameter.value))
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False,
                        reason="sample_stale_sec must be numeric",
                    )
                requires_model_reset = True
            elif parameter.name == "wind_stale_sec":
                try:
                    wind_stale_sec = max(0.1, float(parameter.value))
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False,
                        reason="wind_stale_sec must be numeric",
                    )
                requires_model_reset = True
            elif parameter.name == "hold_last_sec":
                try:
                    hold_last_sec = max(0.0, float(parameter.value))
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False,
                        reason="hold_last_sec must be numeric",
                    )
                requires_model_reset = True
            elif parameter.name == "min_wind_speed_mps":
                try:
                    min_wind_speed_mps = max(0.0, float(parameter.value))
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False,
                        reason="min_wind_speed_mps must be numeric",
                    )
                requires_model_reset = True
            elif parameter.name == "max_extent_m":
                try:
                    max_extent_m = max(1.0, float(parameter.value))
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False,
                        reason="max_extent_m must be numeric",
                    )
                requires_model_reset = True
            elif parameter.name in {
                "gas_input_topic",
                "wind_input_topic",
                "output_topic",
            }:
                if str(parameter.value) != self.get_parameter(parameter.name).value:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            f"{parameter.name} updates require node restart "
                            "to rebuild ROS publishers/subscriptions"
                        ),
                    )

        self._rate_hz = rate_hz
        self._species = species
        self._units = units
        self._smoothing_window = smoothing_window
        self._plume_resolution = plume_resolution
        self._sample_stale_sec = sample_stale_sec
        self._wind_stale_sec = wind_stale_sec
        self._hold_last_sec = hold_last_sec
        self._min_wind_speed_mps = min_wind_speed_mps
        self._max_extent_m = max_extent_m

        if requires_model_reset:
            self._model = self._create_model()

        if requires_timer_reset:
            self._timer.cancel()
            self._timer = self.create_timer(self._period_sec, self._publish_sample)

        return SetParametersResult(successful=True)

    def _publish_sample(self) -> None:
        now_sec = self._now_seconds()
        estimate = self._model.estimate(now_sec=now_sec)
        if estimate is None:
            return

        msg = GasIsopleth()
        msg.stamp = self.get_clock().now().to_msg()
        msg.species = self._species
        msg.units = self._units
        msg.polygons = [self._to_polygon(polygon) for polygon in estimate.polygons]
        msg.centerline = [
            Point32(x=float(x), y=float(y), z=0.0) for x, y in estimate.centerline
        ]
        self._publisher.publish(msg)

    def _handle_gas_sample(self, message: PointStamped) -> None:
        stamp_sec = self._stamp_to_seconds(message.header.stamp)
        if stamp_sec <= 0.0:
            stamp_sec = self._now_seconds()

        concentration = float(message.point.z)
        self._model.add_sample(
            x=float(message.point.x),
            y=float(message.point.y),
            concentration=concentration,
            timestamp_sec=stamp_sec,
        )

    def _handle_wind_sample(self, message: Vector3Stamped) -> None:
        stamp_sec = self._stamp_to_seconds(message.header.stamp)
        if stamp_sec <= 0.0:
            stamp_sec = self._now_seconds()

        self._model.set_wind(
            vx=float(message.vector.x),
            vy=float(message.vector.y),
            timestamp_sec=stamp_sec,
        )

    @staticmethod
    def _to_polygon(points: list[tuple[float, float]]) -> Polygon:
        return Polygon(
            points=[Point32(x=float(x), y=float(y), z=0.0) for x, y in points]
        )

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _stamp_to_seconds(stamp) -> float:
        sec = float(getattr(stamp, "sec", 0.0))
        nanosec = float(getattr(stamp, "nanosec", 0.0))
        return sec + (nanosec / 1e9)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = GasIsoplethNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
