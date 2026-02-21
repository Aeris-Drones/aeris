"""Store-and-forward buffering for map tiles, detections, and telemetry streams."""

from __future__ import annotations

from typing import Any

import rclpy
from rcl_interfaces.msg import Parameter, SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Float32, String

from aeris_msgs.msg import FusedDetection, MapTile, Telemetry

from .message_envelope import (
    build_dedupe_key,
    compute_payload_hash,
    deserialize_message_payload,
    extract_event_timestamp,
    serialize_message_payload,
)
from .store_forward_core import StoreForwardController
from .store_forward_store import StoreForwardStore


class StoreForwardTiles(Node):
    """Durably buffers outbound traffic during outages and replays in order."""

    def __init__(self) -> None:
        super().__init__("store_forward_tiles")

        # Topic wiring
        self._map_input_topic = str(self.declare_parameter("map_input_topic", "map/tiles").value)
        self._map_output_topic = str(
            self.declare_parameter("map_output_topic", "map/tiles_out").value
        )
        self._detection_input_topic = str(
            self.declare_parameter("detection_input_topic", "detections/fused").value
        )
        self._detection_output_topic = str(
            self.declare_parameter("detection_output_topic", "detections/fused_out").value
        )
        self._heartbeat_input_topic = str(
            self.declare_parameter("heartbeat_input_topic", "orchestrator/heartbeat").value
        )
        self._connectivity_heartbeat_topic = str(
            self.declare_parameter("connectivity_heartbeat_topic", "mesh/heartbeat_imp").value
        )
        self._heartbeat_output_topic = str(
            self.declare_parameter("heartbeat_output_topic", "mesh/heartbeat_out").value
        )
        self._telemetry_input_topic = str(
            self.declare_parameter("telemetry_input_topic", "telemetry").value
        )
        self._telemetry_output_topic = str(
            self.declare_parameter("telemetry_output_topic", "telemetry_out").value
        )
        self._pdr_topic = str(self.declare_parameter("pdr_topic", "").value)

        # Connectivity + storage controls required by Story 4.2.
        self._heartbeat_timeout_sec = float(
            self.declare_parameter("heartbeat_timeout_sec", 2.0).value
        )
        self._pdr_threshold = float(self.declare_parameter("pdr_threshold", 0.0).value)
        self._storage_path = str(
            self.declare_parameter(
                "storage_path", "/var/lib/aeris/store-forward/store-forward.db"
            ).value
        )
        self._max_bytes = int(self.declare_parameter("max_bytes", 256 * 1024 * 1024).value)
        self._link_up_override_value = bool(self.declare_parameter("link_up", True).value)
        self._link_up_override_enabled = bool(
            self.declare_parameter("link_up_override", False).value
        )

        self._store = StoreForwardStore(db_path=self._storage_path, max_bytes=self._max_bytes)
        self._controller = StoreForwardController(
            store=self._store,
            heartbeat_timeout_sec=self._heartbeat_timeout_sec,
            pdr_threshold=self._pdr_threshold,
        )
        self._controller.set_connectivity_override(
            enabled=self._link_up_override_enabled,
            link_up=self._link_up_override_value,
        )

        self._map_publisher = self.create_publisher(MapTile, self._map_output_topic, 10)
        self._detection_publisher = self.create_publisher(
            FusedDetection, self._detection_output_topic, 10
        )
        self._heartbeat_publisher = self.create_publisher(String, self._heartbeat_output_topic, 10)
        self._telemetry_publisher = self.create_publisher(
            Telemetry, self._telemetry_output_topic, 10
        )
        self._route_specs: dict[str, tuple[type[Any], Any]] = {
            "map_tile": (MapTile, self._map_publisher),
            "fused_detection": (FusedDetection, self._detection_publisher),
            "heartbeat": (String, self._heartbeat_publisher),
            "telemetry": (Telemetry, self._telemetry_publisher),
        }

        self._map_subscription = self.create_subscription(
            MapTile, self._map_input_topic, self._handle_map_tile, 10
        )
        self._detection_subscription = self.create_subscription(
            FusedDetection, self._detection_input_topic, self._handle_detection, 10
        )
        self._heartbeat_subscription = self.create_subscription(
            String, self._heartbeat_input_topic, self._handle_heartbeat, 10
        )
        self._telemetry_subscription = self.create_subscription(
            Telemetry, self._telemetry_input_topic, self._handle_telemetry, 10
        )
        self._connectivity_heartbeat_subscription = self.create_subscription(
            String,
            self._connectivity_heartbeat_topic,
            self._handle_connectivity_heartbeat,
            10,
        )
        self._pdr_subscription = None
        if self._pdr_topic:
            self._pdr_subscription = self.create_subscription(
                Float32, self._pdr_topic, self._handle_pdr, 10
            )

        self._link_monitor_timer = self.create_timer(0.25, self._sync_connectivity)
        self._metrics_timer = self.create_timer(5.0, self._log_metrics)
        self.add_on_set_parameters_callback(self._handle_param_update)

    def _handle_param_update(self, params: list[Parameter]) -> SetParametersResult:
        immutable_topics = {
            "map_input_topic",
            "map_output_topic",
            "detection_input_topic",
            "detection_output_topic",
            "heartbeat_input_topic",
            "connectivity_heartbeat_topic",
            "heartbeat_output_topic",
            "telemetry_input_topic",
            "telemetry_output_topic",
            "pdr_topic",
            "storage_path",
        }

        for param in params:
            if param.name in immutable_topics:
                self.get_logger().warning(
                    f"{param.name} is fixed for this process; restart to change topics"
                )
                return SetParametersResult(successful=False)

            if param.name == "heartbeat_timeout_sec":
                value = float(param.value)
                if value <= 0.0:
                    return SetParametersResult(successful=False)
                self._heartbeat_timeout_sec = value
                self._controller.connectivity.heartbeat_timeout_sec = value
                continue

            if param.name == "pdr_threshold":
                value = float(param.value)
                if value < 0.0 or value > 1.0:
                    return SetParametersResult(successful=False)
                self._pdr_threshold = value
                self._controller.connectivity.pdr_threshold = value
                continue

            if param.name == "max_bytes":
                value = int(param.value)
                if value <= 0:
                    return SetParametersResult(successful=False)
                self._max_bytes = value
                self._store.set_max_bytes(value)
                continue

            if param.name == "link_up":
                self._link_up_override_value = bool(param.value)
                self._controller.set_connectivity_override(
                    enabled=self._link_up_override_enabled,
                    link_up=self._link_up_override_value,
                )
                continue

            if param.name == "link_up_override":
                self._link_up_override_enabled = bool(param.value)
                self._controller.set_connectivity_override(
                    enabled=self._link_up_override_enabled,
                    link_up=self._link_up_override_value,
                )
                continue

            self.get_logger().warning(f"unsupported parameter '{param.name}'")
            return SetParametersResult(successful=False)

        return SetParametersResult(successful=True)

    def _handle_map_tile(self, msg: MapTile) -> None:
        self._handle_outbound(
            message=msg,
            input_topic=self._map_input_topic,
            output_topic=self._map_output_topic,
            route_key="map_tile",
            message_kind="map_tile",
        )

    def _handle_detection(self, msg: FusedDetection) -> None:
        self._handle_outbound(
            message=msg,
            input_topic=self._detection_input_topic,
            output_topic=self._detection_output_topic,
            route_key="fused_detection",
            message_kind="fused_detection",
        )

    def _handle_heartbeat(self, msg: String) -> None:
        self._handle_outbound(
            message=msg,
            input_topic=self._heartbeat_input_topic,
            output_topic=self._heartbeat_output_topic,
            route_key="heartbeat",
            message_kind="heartbeat",
        )

    def _handle_connectivity_heartbeat(self, _msg: String) -> None:
        self._controller.connectivity.observe_heartbeat()

    def _handle_telemetry(self, msg: Telemetry) -> None:
        self._handle_outbound(
            message=msg,
            input_topic=self._telemetry_input_topic,
            output_topic=self._telemetry_output_topic,
            route_key="telemetry",
            message_kind="telemetry",
        )

    def _handle_pdr(self, msg: Float32) -> None:
        self._controller.connectivity.observe_pdr(float(msg.data))

    def _handle_outbound(
        self,
        *,
        message: Any,
        input_topic: str,
        output_topic: str,
        route_key: str,
        message_kind: str,
    ) -> None:
        payload = serialize_message_payload(message)
        payload_hash = compute_payload_hash(payload)
        dedupe_key = build_dedupe_key(
            message_kind=message_kind,
            topic=input_topic,
            message=message,
            payload_hash=payload_hash,
        )
        event_ts = extract_event_timestamp(message)
        disposition = self._controller.handle_outbound(
            topic=output_topic,
            route_key=route_key,
            message_kind=message_kind,
            event_ts=event_ts,
            dedupe_key=dedupe_key,
            payload=payload,
            payload_hash=payload_hash,
            publish_callback=self._publish_record,
        )
        if disposition != "published-live":
            metrics = self._store.metrics()
            self.get_logger().debug(
                "buffer disposition="
                f"{disposition} queued={metrics.queued_count} "
                f"bytes={metrics.bytes_on_disk} dedupe_hits={metrics.dedupe_hits} "
                f"evicted={metrics.evicted_count}"
            )

    def _publish_record(
        self,
        ingest_seq: int,
        topic: str,
        route_key: str,
        message_kind: str,
        payload: bytes,
    ) -> None:
        del topic
        del message_kind

        spec = self._route_specs.get(route_key)
        if spec is None:
            self.get_logger().warning(f"no route configured for replay key '{route_key}'")
            return
        message_type, publisher = spec
        message = deserialize_message_payload(message_type, payload)
        publisher.publish(message)

        if ingest_seq > 0:
            self.get_logger().debug(
                f"replayed buffered message ingest_seq={ingest_seq} route={route_key}"
            )

    def _sync_connectivity(self) -> None:
        self._controller.sync_connectivity_and_flush(self._publish_record)

    def _log_metrics(self) -> None:
        metrics = self._store.metrics()
        link_state = "up" if self._controller.connectivity.is_link_up() else "down"
        self.get_logger().info(
            "store-forward metrics "
            f"link={link_state} queued={metrics.queued_count} "
            f"bytes={metrics.bytes_on_disk} dedupe_hits={metrics.dedupe_hits} "
            f"evicted={metrics.evicted_count} "
            f"oldest_age_sec={metrics.oldest_buffered_age_sec:.2f}"
        )

    def destroy_node(self) -> bool:
        self._store.close()
        return super().destroy_node()


def main() -> None:
    """Entry point for the store-forward mesh buffering node."""
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
