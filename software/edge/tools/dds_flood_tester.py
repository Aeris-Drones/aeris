#!/usr/bin/env python3
"""DDS transport stress tester for validating message throughput.

Publishes high-rate messages with configurable payload sizes to measure
DDS layer performance, identify dropped messages, and characterize
latency under load. Used for network capacity planning and QoS tuning.

Usage:
    ros2 run aeris_tools dds_flood_tester [options]
    python3 dds_flood_tester.py [--topic TOPIC] [--rate HZ] [--payload-bytes BYTES]

Examples:
    # Default 500Hz flood with 1KB payloads on 'flood/test'
    ros2 run aeris_tools dds_flood_tester

    # High-rate 2KHz flood with 100-byte payloads on custom topic
    ros2 run aeris_tools dds_flood_tester --topic /stress/test --rate 2000 --payload-bytes 100

    # Large payload test (10KB) at 100Hz to test fragmentation
    ros2 run aeris_tools dds_flood_tester --rate 100 --payload-bytes 10240

Exit Codes:
    0: Normal shutdown (Ctrl+C)
    1: ROS initialization error

Debugging:
    - Monitor with `ros2 topic hz /flood/test` to verify receive rate
    - Check `ros2 topic echo /flood/test | head` to inspect message format
    - Use wireshark with DDS dissector to analyze packet fragmentation
"""

import argparse
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FloodPublisher(Node):
    """Publishes messages at sustained high rates to stress-test DDS transport.

    Generates sequential messages with fixed-size payloads to measure
    throughput and detect message loss in the DDS layer.

    Attributes:
        _publisher: ROS2 publisher for String messages.
        _period: Interval between publishes (seconds).
        _payload: Fixed-size payload string.
        _sent: Total messages published.
        _last_log: Timestamp of last status log.
        _timer: ROS2 timer for periodic publishing.
    """

    def __init__(self, topic: str, rate_hz: float, payload_size: int) -> None:
        super().__init__("dds_flood_publisher")
        self._publisher = self.create_publisher(String, topic, 10)
        self._period = 1.0 / rate_hz
        self._payload = "X" * max(1, payload_size)
        self._sent = 0
        self._last_log = self.get_clock().now()
        self._timer = self.create_timer(self._period, self._publish)

    def _publish(self) -> None:
        """Publish a single message and update counters."""
        msg = String()
        msg.data = f"{self._sent}:{self._payload}"
        self._publisher.publish(msg)
        self._sent += 1

        now = self.get_clock().now()
        if (now - self._last_log).nanoseconds > 5_000_000_000:
            self.get_logger().info(f"Sent {self._sent} samples")
            self._last_log = now


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments.

    Returns:
        Parsed arguments namespace.
    """
    parser = argparse.ArgumentParser(
        description="DDS flood generator for transport stress testing"
    )
    parser.add_argument(
        "--topic",
        default="flood/test",
        help="ROS2 topic to publish on (default: %(default)s)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=500.0,
        help="Publication rate in Hz (default: %(default)s)",
    )
    parser.add_argument(
        "--payload-bytes",
        type=int,
        default=1024,
        help="Payload size in bytes (default: %(default)s)",
    )
    return parser.parse_args()


def main() -> int:
    """Main entry point.

    Returns:
        Exit code (0 for success, non-zero for error).
    """
    args = parse_args()

    rclpy.init()
    node = FloodPublisher(args.topic, args.rate, args.payload_bytes)
    exit_code = 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        print(f"Error: {exc}", file=__import__("sys").stderr)
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    import sys

    sys.exit(main())
