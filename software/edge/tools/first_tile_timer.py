#!/usr/bin/env python3
"""Utility that reports the elapsed time until the first /map/tiles sample."""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node

from aeris_msgs.msg import MapTile


class FirstTileTimer(Node):
    """Measures wall clock time until the first MapTile arrives."""

    def __init__(self, timeout_sec: float) -> None:
        super().__init__("first_tile_timer")
        self._start = time.monotonic()
        self._timeout = timeout_sec
        self.exit_code = 1
        self._completed = False

        self.create_subscription(
            MapTile,
            "/map/tiles",
            self._tile_callback,
            10,
        )
        self.create_timer(0.5, self._timeout_callback)

    def _tile_callback(self, msg: MapTile) -> None:  # noqa: ARG002
        if self._completed:
            return
        elapsed = time.monotonic() - self._start
        self.get_logger().info(f"First tile in {elapsed:.3f} seconds")
        self.exit_code = 0
        self._completed = True
        if rclpy.ok():
            rclpy.shutdown()

    def _timeout_callback(self) -> None:
        if self._completed:
            return
        elapsed = time.monotonic() - self._start
        if elapsed >= self._timeout:
            self.get_logger().error(
                f"No /map/tiles message received within {self._timeout:.1f} seconds"
            )
            self.exit_code = 2
            self._completed = True
            if rclpy.ok():
                rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Prints how long it took to observe the first /map/tiles sample."
    )
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=120.0,
        help="Abort if no tile is seen within this many seconds (default: 120)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = FirstTileTimer(args.timeout_sec)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted before first tile")
        node.exit_code = 130
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()
    sys.exit(node.exit_code)


if __name__ == "__main__":
    main()
