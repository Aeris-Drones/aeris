"""Minimal orchestrator heartbeat publisher for early bring-up verification.

Provides a lightweight liveness signal used by system health monitors
to verify orchestrator process availability before full mission
capabilities are initialized. The heartbeat format includes host
identity to aid distributed debugging across multi-node deployments.
"""

import os
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HeartbeatNode(Node):
    """ROS 2 node that publishes periodic heartbeat messages.

    Publishes timestamped heartbeat messages containing host identity
    and sequence numbers to enable external health monitoring and
    debugging of distributed orchestrator deployments.

    Attributes:
        _publisher: ROS publisher for heartbeat String messages.
        _timer: Periodic timer triggering heartbeat publications.
        _sequence: Monotonic sequence counter for message ordering.
    """

    def __init__(self) -> None:
        """Initialize the heartbeat node with configurable parameters.

        Declares and retrieves parameters for queue depth, topic name,
        and publication period. Creates the publisher and timer.
        """
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
        """Publish a heartbeat message with timestamp and host identity.

        Constructs a heartbeat message containing UTC timestamp, host
        nodename, and sequence number. Publishes via ROS and increments
        the sequence counter.
        """
        timestamp = datetime.now(timezone.utc).isoformat()
        host_id = os.uname().nodename

        message = String()
        message.data = f"{timestamp}::{host_id}::{self._sequence}"

        self._publisher.publish(message)
        self._sequence += 1


def main(args=None) -> None:
    """Entry point for the heartbeat node executable.

    Initializes ROS 2, creates the HeartbeatNode, and spins until
    shutdown is requested via interrupt or signal.

    Args:
        args: Optional command-line arguments passed to rclpy.init().
    """
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
