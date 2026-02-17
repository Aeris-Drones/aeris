"""Simulated gas concentration and wind topic publisher for local demos."""

from __future__ import annotations

from typing import Any
import math

import rclpy
from geometry_msgs.msg import PointStamped, Vector3Stamped
from rclpy.node import Node


class GasInputSimNode(Node):
    """Publish deterministic gas sample + wind vector streams for simulation."""

    def __init__(self) -> None:
        super().__init__("gas_input_sim")

        self._gas_output_topic = str(
            self.declare_parameter("gas_output_topic", "sim/gas/sample").value
        )
        self._wind_output_topic = str(
            self.declare_parameter("wind_output_topic", "sim/wind/vector").value
        )
        self._sample_rate_hz = max(
            0.2, float(self.declare_parameter("sample_rate_hz", 4.0).value)
        )
        self._wind_rate_hz = max(
            0.2, float(self.declare_parameter("wind_rate_hz", 2.0).value)
        )

        self._source_x = float(self.declare_parameter("source_x", 0.0).value)
        self._source_y = float(self.declare_parameter("source_y", 0.0).value)
        self._base_concentration = float(
            self.declare_parameter("base_concentration", 3.0).value
        )
        self._plume_sigma_m = max(
            1.0, float(self.declare_parameter("plume_sigma_m", 18.0).value)
        )
        self._wind_speed_mps = max(
            0.0, float(self.declare_parameter("wind_speed_mps", 2.0).value)
        )
        self._frame_id = str(self.declare_parameter("frame_id", "map").value).strip()

        self._gas_publisher = self.create_publisher(
            PointStamped, self._gas_output_topic, 10
        )
        self._wind_publisher = self.create_publisher(
            Vector3Stamped, self._wind_output_topic, 10
        )

        self._sample_phase = 0.0
        self._wind_phase = 0.0
        self._latest_wind = (self._wind_speed_mps, 0.0)

        self._sample_timer = self.create_timer(
            1.0 / self._sample_rate_hz,
            self._publish_gas_sample,
        )
        self._wind_timer = self.create_timer(
            1.0 / self._wind_rate_hz,
            self._publish_wind,
        )

    def _publish_wind(self) -> None:
        self._wind_phase += 0.08
        direction = self._wind_phase
        speed = self._wind_speed_mps * (0.85 + (0.15 * math.sin(self._wind_phase * 0.7)))
        vx = speed * math.cos(direction)
        vy = speed * math.sin(direction)
        self._latest_wind = (vx, vy)

        message = Vector3Stamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.vector.x = float(vx)
        message.vector.y = float(vy)
        message.vector.z = 0.0
        self._wind_publisher.publish(message)

    def _publish_gas_sample(self) -> None:
        self._sample_phase += 0.15
        now = self.get_clock().now().to_msg()

        # Probe path sweeps through plume crosswind/downwind to provide
        # incremental geometry updates to the plume estimator.
        probe_x = self._source_x + (14.0 * math.sin(self._sample_phase))
        probe_y = self._source_y + (10.0 * math.sin(self._sample_phase * 0.6))

        vx, vy = self._latest_wind
        wind_norm = math.hypot(vx, vy)
        if wind_norm <= 1e-6:
            wind_dir = (1.0, 0.0)
        else:
            wind_dir = (vx / wind_norm, vy / wind_norm)

        dx = probe_x - self._source_x
        dy = probe_y - self._source_y
        along = (dx * wind_dir[0]) + (dy * wind_dir[1])
        cross = (-dx * wind_dir[1]) + (dy * wind_dir[0])

        sigma = self._plume_sigma_m
        downwind_gain = 1.0 + (0.2 * max(0.0, along / max(sigma, 1.0)))
        concentration = self._base_concentration * downwind_gain * math.exp(
            -((along * along) / (2.0 * sigma * sigma))
            - ((cross * cross) / (2.0 * (0.55 * sigma) ** 2))
        )

        message = PointStamped()
        message.header.stamp = now
        message.header.frame_id = self._frame_id
        message.point.x = float(probe_x)
        message.point.y = float(probe_y)
        message.point.z = float(max(0.0, concentration))
        self._gas_publisher.publish(message)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = GasInputSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received; shutting down gas_input_sim.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
