"""Synthetic gas plume simulator for testing map visualization pipelines.

Generates time-varying isopleth polygons that mimic drifting gas plumes,
allowing validation of gas concentration overlays without requiring
actual chemical sensors.
"""

import math
from typing import List

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

from aeris_msgs.msg import GasIsopleth
from geometry_msgs.msg import Point32, Polygon


class GasIsoplethNode(Node):
    """Generates synthetic gas plume contours with realistic drift and diffusion.

    Produces time-varying rectangular polygons that simulate plume boundaries
    moving with wind drift. Used to validate gas visualization overlays
    and downstream alerting logic without hardware dependencies.
    """

    def __init__(self) -> None:
        super().__init__("gas_isopleth")
        self._rate_hz = self.declare_parameter("rate_hz", 0.5).value
        self._species = self.declare_parameter("species", "VOC").value
        self._units = self.declare_parameter("units", "ppm").value
        self._counter = 0
        self._publisher = self.create_publisher(GasIsopleth, "gas/isopleth", 10)
        self._timer = self.create_timer(self._period_sec, self._publish_sample)
        self.add_on_set_parameters_callback(self._handle_param_update)

    @property
    def _period_sec(self) -> float:
        rate = max(self._rate_hz, 0.1)
        return 1.0 / rate

    def _handle_param_update(self, params):
        updated = False
        for param in params:
            if param.name == "rate_hz":
                try:
                    value = float(param.value)
                except (TypeError, ValueError):
                    return SetParametersResult(successful=False)
                if value <= 0.0:
                    return SetParametersResult(successful=False)
                self._rate_hz = value
                updated = True
            elif param.name == "species":
                self._species = str(param.value)
            elif param.name == "units":
                self._units = str(param.value)
        if updated:
            self._timer.cancel()
            self._timer = self.create_timer(self._period_sec, self._publish_sample)
            self.get_logger().info(f"gas isopleth rate set to {self._rate_hz:.2f} Hz")
        return SetParametersResult(successful=True)

    def _rectangle(self) -> Polygon:
        self._counter += 1
        drift = math.sin(self._counter * 0.1) * 1.5
        width = 2.0
        height = 1.0 + 0.5 * math.sin(self._counter * 0.07)
        x_min = drift - width / 2.0
        x_max = drift + width / 2.0
        y_min = -height / 2.0
        y_max = height / 2.0
        points = [
            Point32(x=x_min, y=y_min, z=0.0),
            Point32(x=x_max, y=y_min, z=0.0),
            Point32(x=x_max, y=y_max, z=0.0),
            Point32(x=x_min, y=y_max, z=0.0),
        ]
        return Polygon(points=points)

    def _centerline(self) -> List[Point32]:
        drift = math.sin(self._counter * 0.1) * 1.5
        return [
            Point32(x=drift, y=-1.0, z=0.0),
            Point32(x=drift + 0.2, y=0.0, z=0.0),
            Point32(x=drift, y=1.0, z=0.0),
        ]

    def _publish_sample(self) -> None:
        msg = GasIsopleth()
        msg.stamp = self.get_clock().now().to_msg()
        msg.species = self._species
        msg.units = self._units
        msg.polygons = [self._rectangle()]
        msg.centerline = self._centerline()
        self._publisher.publish(msg)


def main() -> None:
    rclpy.init()
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
