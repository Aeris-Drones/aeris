"""Synthetic acoustic bearing publisher for testing sound source localization.

Generates random bearing measurements simulating a 4-channel microphone array.
Used in SITL scenarios to validate tracking behavior triggered by acoustic
detections. Supports runtime frequency adjustment via ROS parameters.
"""

import random
from typing import Any, List

import rclpy
from rcl_interfaces.msg import Parameter, SetParametersResult
from rclpy.node import Node

from aeris_msgs.msg import AcousticBearing


class AcousticBearingNode(Node):
    """Publishes synthetic acoustic bearing measurements.

    This node simulates a 4-channel microphone array by publishing randomized
    bearing measurements at a configurable rate. It is designed for SITL
    (Software-In-The-Loop) testing of acoustic tracking algorithms.

    Attributes:
        _rate_hz: Current publication rate in Hz.
        _publisher: ROS publisher for AcousticBearing messages.
        _timer: ROS timer triggering periodic publications.

    ROS Parameters:
        rate_hz (float): Publication frequency in Hz. Must be positive.
            Default: 1.0

    ROS Topics:
        Published:
            acoustic/bearing (aeris_msgs/msg/AcousticBearing): Bearing
                measurements with confidence and SNR values.
    """

    def __init__(self) -> None:
        """Initialize the acoustic bearing node.

        Declares parameters, creates the publisher, and starts the timer.
        """
        super().__init__("acoustic_bearing")
        self._rate_hz: float = self.declare_parameter("rate_hz", 1.0).value
        self._publisher = self.create_publisher(
            AcousticBearing, "acoustic/bearing", 10
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
                # Recreate timer with new period
                self._timer.cancel()
                self._timer = self.create_timer(
                    self._period_sec, self._publish_sample
                )
                self.get_logger().info(
                    f"acoustic rate set to {self._rate_hz:.2f} Hz"
                )
        return SetParametersResult(successful=True)

    def _publish_sample(self) -> None:
        """Generate and publish a synthetic acoustic bearing sample.

        Creates a randomized bearing measurement with:
        - Bearing: Uniform random angle 0-360 degrees
        - Confidence: Random value between 0.5-0.9
        - SNR: Random signal-to-noise ratio 5-12 dB
        """
        msg = AcousticBearing()
        msg.stamp = self.get_clock().now().to_msg()
        # Generate random bearing uniformly distributed around full circle
        msg.bearing_deg = random.uniform(0.0, 360.0)
        # Confidence varies between moderate and high
        msg.confidence = random.uniform(0.5, 0.9)
        # SNR represents typical detection range for distant sources
        msg.snr_db = random.uniform(5.0, 12.0)
        msg.mic_array = "4ch_v1"
        self._publisher.publish(msg)


def main(args: Any = None) -> None:
    """Entry point for the acoustic bearing node.

    Args:
        args: Command line arguments (passed to rclpy.init).
    """
    rclpy.init(args=args)
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
