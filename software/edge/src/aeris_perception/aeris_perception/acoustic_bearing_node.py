"""Publishes synthetic AcousticBearing detections at ~1 Hz."""

import random

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

from aeris_msgs.msg import AcousticBearing


class AcousticBearingNode(Node):
    """Publishes bearings with configurable frequency."""

    def __init__(self) -> None:
        super().__init__("acoustic_bearing")
        self._rate_hz = self.declare_parameter("rate_hz", 1.0).value
        self._publisher = self.create_publisher(AcousticBearing, "acoustic/bearing", 10)
        self._timer = self.create_timer(self._period_sec, self._publish_sample)
        self.add_on_set_parameters_callback(self._handle_param_update)

    @property
    def _period_sec(self) -> float:
        rate = max(self._rate_hz, 0.1)
        return 1.0 / rate

    def _handle_param_update(self, params):
        for param in params:
            if param.name == "rate_hz":
                try:
                    value = float(param.value)
                except (TypeError, ValueError):
                    return SetParametersResult(successful=False)
                if value <= 0.0:
                    self.get_logger().warning("rate_hz must be positive")
                    return SetParametersResult(successful=False)
                self._rate_hz = value
                self._timer.cancel()
                self._timer = self.create_timer(self._period_sec, self._publish_sample)
                self.get_logger().info(f"acoustic rate set to {self._rate_hz:.2f} Hz")
        return SetParametersResult(successful=True)

    def _publish_sample(self) -> None:
        msg = AcousticBearing()
        msg.stamp = self.get_clock().now().to_msg()
        msg.bearing_deg = random.uniform(0.0, 360.0)
        msg.confidence = random.uniform(0.5, 0.9)
        msg.snr_db = random.uniform(5.0, 12.0)
        msg.mic_array = "4ch_v1"
        self._publisher.publish(msg)


def main() -> None:
    rclpy.init()
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
