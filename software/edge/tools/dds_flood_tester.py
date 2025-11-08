#!/usr/bin/env python3
"""High-rate ROS 2 publisher for transport validation."""

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FloodPublisher(Node):
    def __init__(self, topic: str, rate_hz: float, payload_size: int) -> None:
        super().__init__("dds_flood_publisher")
        self._publisher = self.create_publisher(String, topic, 10)
        self._period = 1.0 / rate_hz
        self._payload = "X" * max(1, payload_size)
        self._sent = 0
        self._last_log = self.get_clock().now()
        self._timer = self.create_timer(self._period, self._publish)

    def _publish(self) -> None:
        msg = String()
        msg.data = f"{self._sent}:{self._payload}"
        self._publisher.publish(msg)
        self._sent += 1

        now = self.get_clock().now()
        if (now - self._last_log).nanoseconds > 5_000_000_000:
            self.get_logger().info(f"Sent {self._sent} samples")
            self._last_log = now


def main() -> None:
    parser = argparse.ArgumentParser(description="DDS flood generator")
    parser.add_argument("--topic", default="flood/test")
    parser.add_argument("--rate", type=float, default=500.0)
    parser.add_argument("--payload-bytes", type=int, default=1024)
    args = parser.parse_args()

    rclpy.init()
    node = FloodPublisher(args.topic, args.rate, args.payload_bytes)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
