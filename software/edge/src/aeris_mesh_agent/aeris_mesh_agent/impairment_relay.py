"""Applies probabilistic drop/delay to emulate a lossy mesh link."""

import collections
import random
import time
from typing import Deque, Tuple

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import String


class ImpairmentRelay(Node):
    """Relays String topics with tunable drop probability and delay."""

    def __init__(self) -> None:
        super().__init__("impairment_relay")
        self._input_topic = self.declare_parameter(
            "input_topic", "orchestrator/heartbeat"
        ).value
        self._output_topic = self.declare_parameter(
            "output_topic", "mesh/heartbeat_imp"
        ).value
        self._drop_prob = float(self.declare_parameter("drop_prob", 0.01).value)
        self._delay_ms = int(self.declare_parameter("delay_ms", 10).value)

        self._pending: Deque[Tuple[float, str]] = collections.deque()
        self._received = 0
        self._forwarded = 0
        self._dropped = 0

        self._publisher = self.create_publisher(String, self._output_topic, 10)
        self._subscription = self.create_subscription(  # noqa: F841
            String, self._input_topic, self._handle_message, 10
        )
        self._dispatch_timer = self.create_timer(0.01, self._flush_pending)
        self._stats_timer = self.create_timer(5.0, self._log_stats)
        self.add_on_set_parameters_callback(self._handle_param_update)

    @property
    def _delay_sec(self) -> float:
        return max(self._delay_ms, 0) / 1000.0

    def _handle_param_update(self, params):
        for param in params:
            if param.name == "drop_prob":
                value = float(param.value)
                if value < 0.0 or value > 1.0:
                    return SetParametersResult(successful=False)
                self._drop_prob = value
                self.get_logger().info(f"drop_prob set to {self._drop_prob:.3f}")
            elif param.name == "delay_ms":
                self._delay_ms = max(int(param.value), 0)
                self.get_logger().info(f"delay_ms set to {self._delay_ms}")
            elif param.name in {"input_topic", "output_topic"}:
                self.get_logger().warning(
                    f"{param.name} is fixed for this launch; restart to change topics"
                )
                return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def _handle_message(self, msg: String) -> None:
        self._received += 1
        if random.random() < self._drop_prob:
            self._dropped += 1
            return
        publish_time = time.monotonic() + self._delay_sec
        self._pending.append((publish_time, msg.data))

    def _flush_pending(self) -> None:
        now = time.monotonic()
        while self._pending and self._pending[0][0] <= now:
            _, data = self._pending.popleft()
            out_msg = String()
            out_msg.data = data
            self._publisher.publish(out_msg)
            self._forwarded += 1

    def _log_stats(self) -> None:
        total = self._received
        pass_rate = (self._forwarded / total) if total else 0.0
        self.get_logger().info(
            f"impairment relay stats in last window — received: {self._received}, "
            f"forwarded: {self._forwarded}, dropped: {self._dropped}, pass_rate: {pass_rate:.2f}"
        )
        self._received = 0
        self._forwarded = 0
        self._dropped = 0


def main() -> None:
    rclpy.init()
    node = ImpairmentRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
