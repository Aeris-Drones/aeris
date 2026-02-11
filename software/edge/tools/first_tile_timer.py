#!/usr/bin/env python3
"""Mission startup latency benchmark for map tile generation.

Measures wall-clock time from mission liftoff (or immediate start) to
the first MapTile publication. Used to characterize end-to-end system
startup performance and identify bottlenecks in the mapping pipeline.

Usage:
    ros2 run aeris_tools first_tile_timer [options]
    python3 first_tile_timer.py [--timeout-sec SEC] [--start-mode MODE]

Examples:
    # Default: wait for SEARCHING state on /orchestrator/mission_state
    ros2 run aeris_tools first_tile_timer

    # Immediate start mode (no liftoff wait)
    ros2 run aeris_tools first_tile_timer --start-mode immediate

    # Custom mission state trigger with extended timeout
    ros2 run aeris_tools first_tile_timer \
        --liftoff-state RUNNING \
        --liftoff-topic /mission/state \
        --timeout-sec 300

Exit Codes:
    0: First tile received within timeout
    2: Timeout waiting for first tile after liftoff
    3: Timeout waiting for liftoff state
    4: Unknown timer start condition failure
    130: Interrupted by user (Ctrl+C)

Debugging:
    - Verify /map/tiles is publishing: `ros2 topic hz /map/tiles`
    - Check mission state: `ros2 topic echo /orchestrator/mission_state`
    - Use --start-mode immediate to bypass liftoff detection
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node

from aeris_msgs.msg import MapTile, MissionState


class FirstTileTimer(Node):
    """Measures latency from mission start to first map tile availability.

    Supports two trigger modes: immediate (timer starts at node startup)
    or liftoff (timer starts when a specific MissionState is observed).
    Exits with code 0 on success, non-zero on timeout or error.
    """

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
        """Handle mission state change and start timer on liftoff.

        Args:
            msg: MissionState message containing current state.
        """
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
        """Handle incoming tile message and record latency.

        Args:
            msg: Received MapTile message (unused, required by callback signature).
        """
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
        """Check for timeout and report failure if threshold exceeded."""
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
    """Parse command-line arguments.

    Returns:
        Parsed arguments namespace.
    """
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


def main() -> int:
    """Main entry point.

    Returns:
        Exit code (0 for success, non-zero for error).
    """
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
    return node.exit_code


if __name__ == "__main__":
    sys.exit(main())


if __name__ == "__main__":
    main()
