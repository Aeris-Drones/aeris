#!/usr/bin/env python3
"""Measure /map/tiles descriptor-to-byte-fetch latency distribution."""

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
        return self._client.wait_for_service(timeout_sec=timeout_sec)

    def _now_ms(self) -> float:
        return self.get_clock().now().nanoseconds / 1e6

    def _on_tile(self, msg: MapTile) -> None:
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
        elapsed = time.monotonic() - self._start
        if elapsed >= self._timeout_sec:
            self._finish(timeout=True)
            return

        if len(self._latencies_ms) >= self._sample_count:
            self._finish(timeout=False)

    def _finish(self, timeout: bool) -> None:
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
    data = sorted(values)
    idx = max(0, min(len(data) - 1, int(((p / 100) * len(data)) + 0.999999) - 1))
    return data[idx]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Measure map tile publish-to-viewer latency")
    parser.add_argument("--samples", type=int, default=40)
    parser.add_argument("--timeout-sec", type=float, default=180.0)
    parser.add_argument("--service", type=str, default="/map/get_tile_bytes")
    return parser.parse_args()


def main() -> None:
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
            sys.exit(node.exit_code)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warning("interrupted")
        node.exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(node.exit_code)


if __name__ == "__main__":
    main()
