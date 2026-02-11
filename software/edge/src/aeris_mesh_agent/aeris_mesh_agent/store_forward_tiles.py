"""Store-and-forward buffer for map tiles during mesh outages.

Maintains a local buffer of MapTile messages when the mesh link is
disconnected and automatically replays them upon reconnection.
Prevents map data loss during transient network failures.
"""

import copy
from typing import List

import rclpy
from rcl_interfaces.msg import Parameter, SetParametersResult
from rclpy.node import Node

from aeris_msgs.msg import MapTile


class StoreForwardTiles(Node):
    """Buffers and replays map tiles based on link state.

    The link_up parameter controls forwarding behavior:
    - When True: tiles pass through immediately
    - When False: tiles are buffered for later replay
    """

    def __init__(self) -> None:
        super().__init__("store_forward_tiles")
        self._input_topic = self.declare_parameter("input_topic", "map/tiles").value
        self._output_topic = self.declare_parameter("output_topic", "map/tiles_out").value
        self._link_up = bool(self.declare_parameter("link_up", True).value)

        self._buffer: List[MapTile] = []
        self._publisher = self.create_publisher(MapTile, self._output_topic, 10)
        self._subscription = self.create_subscription(  # noqa: F841
            MapTile, self._input_topic, self._handle_tile, 10
        )
        self.add_on_set_parameters_callback(self._handle_param_update)

    def _handle_param_update(self, params: List[Parameter]) -> SetParametersResult:
        """Handles dynamic parameter updates from ROS parameter interface.

        Args:
            params: List of parameter objects to update.

        Returns:
            SetParametersResult indicating success or failure of the update.

        Note:
            Topics (input_topic, output_topic) cannot be changed at runtime
            and will return failure if attempted.
        """
        for param in params:
            if param.name == "link_up":
                self._set_link_state(bool(param.value))
            elif param.name in {"input_topic", "output_topic"}:
                self.get_logger().warning(
                    f"{param.name} is fixed for this process; restart to change topics"
                )
                return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def _set_link_state(self, new_state: bool) -> None:
        """Updates the mesh link state and triggers buffer flush on reconnect.

        Args:
            new_state: The new link state (True for up, False for down).

        Note:
            When the link transitions from down to up, any buffered tiles
            are automatically flushed to the output topic.
        """
        if self._link_up == new_state:
            return

        self._link_up = new_state
        state = "up" if self._link_up else "down"
        self.get_logger().info(f"link state set to {state}")

        # Flush buffered messages when link comes back online
        if self._link_up:
            self._flush_buffer()

    def _handle_tile(self, msg: MapTile) -> None:
        """Processes incoming map tiles based on current link state.

        Args:
            msg: The incoming MapTile message to process.

        Note:
            When link is up, tiles pass through immediately.
            When link is down, tiles are deep-copied and buffered for later replay.
        """
        if self._link_up:
            # Link healthy: forward immediately
            self._publisher.publish(copy.deepcopy(msg))
            return

        # Link down: buffer tile for later transmission
        self._buffer.append(copy.deepcopy(msg))
        self.get_logger().info(
            f"link down; buffered tile count={len(self._buffer)}"
        )

    def _flush_buffer(self) -> None:
        """Publishes all buffered tiles and clears the buffer.

        Called automatically when the link state transitions from down to up.
        Ensures no map data is lost during transient network failures.
        """
        if not self._buffer:
            self.get_logger().info("link restored; nothing to flush")
            return

        queued = len(self._buffer)

        # Replay all buffered tiles in order
        for tile in self._buffer:
            self._publisher.publish(tile)

        self._buffer.clear()
        self.get_logger().info(f"link restored; flushed {queued} buffered tiles")


def main() -> None:
    """Entry point for the store-forward tiles node.

    Initializes ROS 2, creates the StoreForwardTiles node, and spins until
    interrupted. Performs proper cleanup on shutdown.
    """
    rclpy.init()
    node = StoreForwardTiles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
