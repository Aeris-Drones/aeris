"""Synthetic thermal hotspot generator for testing detection pipelines.

Simulates thermal camera detections by publishing randomized bounding
boxes with temperature and confidence values. Enables validation of
hotspot tracking and alerting without thermal hardware.
"""

import random
from typing import List

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

from aeris_msgs.msg import ThermalHotspot


class ThermalHotspotNode(Node):
    """Generates synthetic thermal detections with randomized geometry.

    Publishes bounding boxes with randomized positions, sizes, and temperatures
    to simulate thermal camera output. Configurable rate allows stress-testing
    of downstream consumers under varying detection frequencies.
    """

    def __init__(self) -> None:
        super().__init__("thermal_hotspot")
        self._rate_hz = self.declare_parameter("rate_hz", 5.0).value
        self._publisher = self.create_publisher(
            ThermalHotspot, "thermal/hotspots", 10
        )
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
                    self.get_logger().warning("rate_hz must be > 0.0")
                    return SetParametersResult(successful=False)
                self._rate_hz = value
                updated = True
        if updated:
            self._timer.cancel()
            self._timer = self.create_timer(self._period_sec, self._publish_sample)
            self.get_logger().info(f"thermal rate set to {self._rate_hz:.2f} Hz")
        return SetParametersResult(successful=True)

    def _random_bbox(self) -> List[int]:
        x_min = random.randint(0, 400)
        y_min = random.randint(0, 200)
        width = random.randint(40, 160)
        height = random.randint(40, 160)
        x_max = min(x_min + width, 639)
        y_max = min(y_min + height, 479)
        return [x_min, y_min, x_max, y_max]

    def _publish_sample(self) -> None:
        msg = ThermalHotspot()
        msg.stamp = self.get_clock().now().to_msg()
        msg.bbox_px = self._random_bbox()
        msg.temp_c = random.uniform(30.0, 45.0)
        msg.confidence = random.uniform(0.6, 0.95)
        msg.frame_id = "thermal_front"
        self._publisher.publish(msg)


def main() -> None:
    rclpy.init()
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
