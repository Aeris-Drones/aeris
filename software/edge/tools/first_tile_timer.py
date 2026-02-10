#!/usr/bin/env python3
"""Measure elapsed time to first /map/tiles sample, anchored to mission liftoff."""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node

from aeris_msgs.msg import MapTile, MissionState


class FirstTileTimer(Node):
    """Measures wall clock time until the first MapTile arrives."""

    def __init__(
        self,
        timeout_sec: float,
        start_mode: str,
        liftoff_topic: str,
        liftoff_state: str,
    ) -> None:
        super().__init__("first_tile_timer")
        self._boot = time.monotonic()
        self._timeout = timeout_sec
        self._start_mode = start_mode
        self._liftoff_topic = liftoff_topic
        self._liftoff_state = liftoff_state.strip().upper()

        self.exit_code = 1
        self._completed = False
        self._liftoff_seen = start_mode == "immediate"
        self._start = self._boot if self._liftoff_seen else None

        self.create_subscription(
            MapTile,
            "/map/tiles",
            self._tile_callback,
            10,
        )

        if start_mode == "liftoff":
            self.create_subscription(
                MissionState,
                liftoff_topic,
                self._liftoff_callback,
                10,
            )
            self.get_logger().info(
                f"Waiting for liftoff state '{self._liftoff_state}' on "
                f"{self._liftoff_topic} before timing first tile"
            )
        else:
            self.get_logger().info("Timing starts immediately at node startup")

        self.create_timer(0.5, self._timeout_callback)

    def _liftoff_callback(self, msg: MissionState) -> None:
        if self._liftoff_seen or self._completed:
            return
        state = msg.state.strip().upper()
        if state != self._liftoff_state:
            return

        self._liftoff_seen = True
        self._start = time.monotonic()
        self.get_logger().info(
            f"Liftoff detected from {self._liftoff_topic} state "
            f"'{self._liftoff_state}'; started first-tile timer"
        )

    def _tile_callback(self, msg: MapTile) -> None:  # noqa: ARG002
        if self._completed:
            return
        if not self._liftoff_seen:
            return
        if self._start is None:
            return

        elapsed = time.monotonic() - self._start
        baseline = "liftoff" if self._start_mode == "liftoff" else "startup"
        self.get_logger().info(f"First tile in {elapsed:.3f} seconds from {baseline}")
        self.exit_code = 0
        self._completed = True
        if rclpy.ok():
            rclpy.shutdown()

    def _timeout_callback(self) -> None:
        if self._completed:
            return

        elapsed = time.monotonic() - self._boot
        if elapsed < self._timeout:
            return

        if not self._liftoff_seen:
            self.get_logger().error(
                f"No liftoff state '{self._liftoff_state}' on {self._liftoff_topic} "
                f"within {self._timeout:.1f} seconds"
            )
            self.exit_code = 3
        elif self._start is not None:
            since_liftoff = time.monotonic() - self._start
            self.get_logger().error(
                f"No /map/tiles message received within {since_liftoff:.1f} seconds "
                "after liftoff"
            )
            self.exit_code = 2
        else:
            self.get_logger().error(
                f"Timer start condition not met within {self._timeout:.1f} seconds"
            )
            self.exit_code = 4

        self._completed = True
        if rclpy.ok():
            rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Print how long it took to observe the first /map/tiles sample."
    )
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=120.0,
        help="Abort if the required event does not complete within this many seconds (default: 120)",
    )
    parser.add_argument(
        "--start-mode",
        choices=["liftoff", "immediate"],
        default="liftoff",
        help="Start timer at mission liftoff or immediately (default: liftoff)",
    )
    parser.add_argument(
        "--liftoff-topic",
        default="/orchestrator/mission_state",
        help="MissionState topic used to detect liftoff (default: /orchestrator/mission_state)",
    )
    parser.add_argument(
        "--liftoff-state",
        default="SEARCHING",
        help="MissionState.state value that marks liftoff (default: SEARCHING)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = FirstTileTimer(
        args.timeout_sec,
        args.start_mode,
        args.liftoff_topic,
        args.liftoff_state,
    )
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
