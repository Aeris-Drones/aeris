"""Synthetic thermal hotspot generator for testing detection pipelines.

Simulates thermal camera detections by publishing randomized bounding
boxes with temperature and confidence values. Enables validation of
hotspot tracking and alerting without thermal hardware.
"""

import random
from typing import Any, List

import rclpy
from rcl_interfaces.msg import Parameter, SetParametersResult
from rclpy.node import Node

from aeris_msgs.msg import ThermalHotspot


class ThermalHotspotNode(Node):
    """Generates synthetic thermal detections with randomized geometry.

    Publishes bounding boxes with randomized positions, sizes, and temperatures
    to simulate thermal camera output. Configurable rate allows stress-testing
    of downstream consumers under varying detection frequencies.

    The simulated bounding boxes are constrained to a 640x480 pixel frame
    to represent a typical thermal camera resolution.

    Attributes:
        _rate_hz: Publication rate in Hz.
        _publisher: ROS publisher for ThermalHotspot messages.
        _timer: ROS timer triggering periodic publications.

    ROS Parameters:
        rate_hz (float): Publication frequency in Hz. Must be positive.
            Default: 5.0

    ROS Topics:
        Published:
            thermal/hotspots (aeris_msgs/msg/ThermalHotspot): Thermal
                detections with bounding boxes, temperature, and confidence.
    """

    def __init__(self) -> None:
        """Initialize the thermal hotspot node.

        Declares parameters, creates the publisher, and starts the timer.
        """
        super().__init__("thermal_hotspot")
        self._rate_hz: float = self.declare_parameter("rate_hz", 5.0).value
        self._publisher = self.create_publisher(
            ThermalHotspot, "thermal/hotspots", 10
        )
        self._timer = self.create_timer(self._period_sec, self._publish_sample)
        self.add_on_set_parameters_callback(self._handle_param_update)

    @property
    def _period_sec(self) -> float:
        """Calculate timer period from current rate.

        Returns:
            Timer period in seconds, clamped to prevent excessive rates.
        """
        rate = max(self._rate_hz, 0.1)
        return 1.0 / rate

    def _handle_param_update(self, params: List[Parameter]) -> SetParametersResult:
        """Handle runtime parameter updates.

        Args:
            params: List of Parameter objects being set.

        Returns:
            SetParametersResult indicating success or failure.

        Note:
            When rate_hz is updated, the timer is cancelled and recreated
            with the new period.
        """
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
            # Recreate timer with new period
            self._timer.cancel()
            self._timer = self.create_timer(
                self._period_sec, self._publish_sample
            )
            self.get_logger().info(
                f"thermal rate set to {self._rate_hz:.2f} Hz"
            )
        return SetParametersResult(successful=True)

    def _random_bbox(self) -> List[int]:
        """Generate a random bounding box within frame constraints.

        Creates a randomized bounding box [x_min, y_min, x_max, y_max]
        constrained to a 640x480 pixel frame. The box dimensions vary
        between 40-160 pixels in each direction.

        Returns:
            List of 4 integers [x_min, y_min, x_max, y_max] representing
            the bounding box in pixel coordinates.

        Algorithm:
            1. Randomly select top-left corner (x_min, y_min)
            2. Randomly select width and height (40-160 pixels)
            3. Clamp bottom-right corner to frame bounds (639, 479)
        """
        # Random top-left corner in upper-left quadrant to allow room for size
        x_min = random.randint(0, 400)
        y_min = random.randint(0, 200)
        # Random dimensions
        width = random.randint(40, 160)
        height = random.randint(40, 160)
        # Calculate bottom-right with frame boundary clamping
        x_max = min(x_min + width, 639)  # Frame width - 1
        y_max = min(y_min + height, 479)  # Frame height - 1
        return [x_min, y_min, x_max, y_max]

    def _publish_sample(self) -> None:
        """Generate and publish a synthetic thermal hotspot sample.

        Creates a ThermalHotspot message with:
        - Current timestamp
        - Randomized bounding box within frame bounds
        - Temperature between 30-45°C (typical detection range)
        - Confidence between 0.6-0.95 (moderate to high)
        - Fixed frame_id representing front thermal camera
        """
        msg = ThermalHotspot()
        msg.stamp = self.get_clock().now().to_msg()
        msg.bbox_px = self._random_bbox()
        # Temperature range represents typical hotspot detection
        # (above ambient, below saturation)
        msg.temp_c = random.uniform(30.0, 45.0)
        # Confidence varies between moderate and high detection certainty
        msg.confidence = random.uniform(0.6, 0.95)
        msg.frame_id = "thermal_front"
        self._publisher.publish(msg)


def main(args: Any = None) -> None:
    """Entry point for the thermal hotspot node.

    Args:
        args: Command line arguments (passed to rclpy.init).
    """
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
