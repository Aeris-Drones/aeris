import os
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HeartbeatNode(Node):
    """Minimal orchestrator heartbeat publisher for early bring-up."""

    def __init__(self) -> None:
        super().__init__("aeris_orchestrator_heartbeat")
        qos_depth = self.declare_parameter("heartbeat_queue_depth", 10).value
        topic = self.declare_parameter("heartbeat_topic", "orchestrator/heartbeat").value
        period = self.declare_parameter("heartbeat_period_sec", 1.0).value

        self._publisher = self.create_publisher(String, topic, qos_depth)
        self._timer = self.create_timer(period, self._publish_heartbeat)
        self._sequence = 0

        self.get_logger().info(
            f"Heartbeat publishing to '{topic}' every {period:.2f}s with depth {qos_depth}."
        )

    def _publish_heartbeat(self) -> None:
        timestamp = datetime.now(timezone.utc).isoformat()
        host_id = os.uname().nodename

        message = String()
        message.data = f"{timestamp}::{host_id}::{self._sequence}"

        self._publisher.publish(message)
        self._sequence += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeartbeatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Heartbeat shutdown requested.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
