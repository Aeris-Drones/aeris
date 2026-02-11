#!/usr/bin/env python3
"""End-to-end map tile latency profiler for viewer performance validation.

Measures the distribution of latencies from MapTile publication to
successful byte retrieval via the GetMapTileBytes service. Reports
average and P95 latencies to characterize viewer responsiveness.

Usage:
    ros2 run aeris_tools tile_latency_probe [options]
    python3 tile_latency_probe.py [--samples N] [--timeout-sec SEC] [--service NAME]

Examples:
    # Default: collect 40 samples with 180s timeout
    ros2 run aeris_tools tile_latency_probe

    # Quick test with 10 samples
    ros2 run aeris_tools tile_latency_probe --samples 10 --timeout-sec 60

    # Custom service endpoint
    ros2 run aeris_tools tile_latency_probe --service /custom/get_tile

Exit Codes:
    0: P95 latency within threshold (<= 2000ms)
    2: No latency samples collected
    3: P95 latency exceeded threshold (> 2000ms)
    4: Tile byte service not available
    130: Interrupted by user (Ctrl+C)

Debugging:
    - Verify /map/tiles is publishing: `ros2 topic hz /map/tiles`
    - Check service availability: `ros2 service list | grep get_tile`
    - Monitor viewer logs for tile fetch errors

Performance Thresholds:
    The probe reports success only if P95 latency stays within 2 seconds,
    which is the target for interactive viewer responsiveness.
"""

import argparse
import statistics
import sys
import time
from typing import List

import rclpy
from rclpy.node import Node

from aeris_msgs.msg import MapTile
from aeris_msgs.srv import GetMapTileBytes


class TileLatencyProbe(Node):
    """Profiles tile retrieval latency by sampling descriptor-to-byte delays.

    Subscribes to tile announcements, then measures round-trip time to fetch
    the actual tile bytes via service call. Tracks latency distribution and
    exits with success only if P95 latency stays within threshold (2s).

    Attributes:
        _sample_count: Target number of latency samples to collect.
        _timeout_sec: Maximum time to wait for samples.
        _start: Timestamp when probing started.
        _latencies_ms: List of collected latency measurements in milliseconds.
        _pending: Set of tile IDs awaiting service response.
        exit_code: Exit code to return on completion.
        _client: Service client for GetMapTileBytes.
    """

    def __init__(self, sample_count: int, timeout_sec: float, service_name: str) -> None:
        super().__init__("tile_latency_probe")
        self._sample_count = sample_count
        self._timeout_sec = timeout_sec
        self._start = time.monotonic()
        self._latencies_ms: List[float] = []
        self._pending: set[str] = set()
        self.exit_code = 1

        self._client = self.create_client(GetMapTileBytes, service_name)
        self.create_subscription(MapTile, "/map/tiles", self._on_tile, 20)
        self.create_timer(0.25, self._tick)

    def wait_for_service_ready(self, timeout_sec: float) -> bool:
        """Wait for the tile bytes service to become available.

        Args:
            timeout_sec: Maximum time to wait in seconds.

        Returns:
            True if service is ready, False if timeout occurred.
        """
        return self._client.wait_for_service(timeout_sec=timeout_sec)

    def _now_ms(self) -> float:
        return self.get_clock().now().nanoseconds / 1e6

    def _on_tile(self, msg: MapTile) -> None:
        """Handle tile announcement and initiate service call.

        Args:
            msg: MapTile message containing tile metadata.
        """
        if len(self._latencies_ms) >= self._sample_count:
            return
        if msg.tile_id in self._pending:
            return
        if not self._client.service_is_ready():
            return

        req = GetMapTileBytes.Request()
        req.tile_id = msg.tile_id
        self._pending.add(msg.tile_id)
        future = self._client.call_async(req)

        def _done(fut):
            self._pending.discard(msg.tile_id)
            try:
                resp = fut.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"service call failed for {msg.tile_id}: {exc}")
                return
            if not resp.found:
                return
            published_ms = (resp.published_at.sec * 1000.0) + (resp.published_at.nanosec / 1e6)
            receive_complete_ms = self._now_ms()
            latency = max(0.0, receive_complete_ms - published_ms)
            self._latencies_ms.append(latency)

        future.add_done_callback(_done)

    def _tick(self) -> None:
        """Periodic timer callback to check for completion or timeout."""
        elapsed = time.monotonic() - self._start
        if elapsed >= self._timeout_sec:
            self._finish(timeout=True)
            return

        if len(self._latencies_ms) >= self._sample_count:
            self._finish(timeout=False)

    def _finish(self, timeout: bool) -> None:
        """Complete the probe and report results.

        Args:
            timeout: True if probe finished due to timeout.
        """
        if not self._latencies_ms:
            self.get_logger().error("No latency samples collected")
            self.exit_code = 2
        else:
            p95 = percentile(self._latencies_ms, 95)
            avg = statistics.mean(self._latencies_ms)
            self.get_logger().info(
                f"samples={len(self._latencies_ms)} avg={avg:.1f}ms p95={p95:.1f}ms"
            )
            if p95 <= 2000.0:
                self.exit_code = 0
            else:
                self.exit_code = 3
        if timeout:
            self.get_logger().warning("probe timeout reached")
        if rclpy.ok():
            rclpy.shutdown()


def percentile(values: List[float], p: int) -> float:
    """Calculate the p-th percentile of a list of values.

    Args:
        values: List of numeric values.
        p: Percentile to calculate (0-100).

    Returns:
        The p-th percentile value.
    """
    data = sorted(values)
    idx = max(0, min(len(data) - 1, int(((p / 100) * len(data)) + 0.999999) - 1))
    return data[idx]


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments.

    Returns:
        Parsed arguments namespace.
    """
    parser = argparse.ArgumentParser(
        description="Measure map tile publish-to-viewer latency",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=40,
        help="Number of latency samples to collect (default: %(default)s)",
    )
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=180.0,
        help="Maximum time to wait for samples in seconds (default: %(default)s)",
    )
    parser.add_argument(
        "--service",
        type=str,
        default="/map/get_tile_bytes",
        help="Service name for tile byte retrieval (default: %(default)s)",
    )
    return parser.parse_args()


def main() -> int:
    """Main entry point.

    Returns:
        Exit code (0 for success, non-zero for error).
    """
    args = parse_args()
    rclpy.init()
    node = TileLatencyProbe(args.samples, args.timeout_sec, args.service)

    deadline = time.monotonic() + 15.0
    while not node.wait_for_service_ready(timeout_sec=0.5):
        if time.monotonic() >= deadline:
            node.get_logger().error("tile byte service not available")
            node.exit_code = 4
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            return node.exit_code

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warning("interrupted")
        node.exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return node.exit_code


if __name__ == "__main__":
    sys.exit(main())
