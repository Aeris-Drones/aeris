"""Store-and-forward buffering for map tiles, detections, and telemetry streams."""

from __future__ import annotations

import json
import time
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
from .relay_routing import RelayEnvelope, RelayLatencyTracker, RelayRouteDecision, RelayRouteSelector
from .store_forward_core import ReplayMetadata, StoreForwardController
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
        self._vehicle_id = str(self.declare_parameter("vehicle_id", "").value)
        self._source_vehicle_id = str(
            self.declare_parameter("source_vehicle_id", self._vehicle_id).value
        )
        self._relay_vehicle_id = str(self.declare_parameter("relay_vehicle_id", "ranger1").value)
        self._relay_enabled = bool(self.declare_parameter("relay_enabled", False).value)
        self._relay_activation_policy = str(
            self.declare_parameter("relay_activation_policy", "auto").value
        )
        self._relay_heartbeat_topic = str(
            self.declare_parameter("relay_heartbeat_topic", "").value
        )
        self._relay_pdr_topic = str(self.declare_parameter("relay_pdr_topic", "").value)
        self._relay_heartbeat_timeout_sec = float(
            self.declare_parameter("relay_heartbeat_timeout_sec", 2.0).value
        )
        self._relay_pdr_threshold = float(
            self.declare_parameter("relay_pdr_threshold", 0.0).value
        )
        self._map_relay_output_topic = str(
            self.declare_parameter("map_relay_output_topic", "relay/map/tiles_out").value
        )
        self._detection_relay_output_topic = str(
            self.declare_parameter(
                "detection_relay_output_topic", "relay/detections/fused_out"
            ).value
        )
        self._heartbeat_relay_output_topic = str(
            self.declare_parameter("heartbeat_relay_output_topic", "relay/heartbeat_out").value
        )
        self._telemetry_relay_output_topic = str(
            self.declare_parameter("telemetry_relay_output_topic", "relay/telemetry_out").value
        )
        self._relay_map_input_topic = str(self.declare_parameter("relay_map_input_topic", "").value)
        self._relay_detection_input_topic = str(
            self.declare_parameter("relay_detection_input_topic", "").value
        )
        self._relay_heartbeat_input_topic = str(
            self.declare_parameter("relay_heartbeat_input_topic", "").value
        )
        self._relay_telemetry_input_topic = str(
            self.declare_parameter("relay_telemetry_input_topic", "").value
        )

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
        self._replay_batch_size = int(self.declare_parameter("replay_batch_size", 64).value)
        self._max_replay_per_cycle = int(
            self.declare_parameter("max_replay_per_cycle", 256).value
        )
        self._replay_annotation_topic = str(
            self.declare_parameter("replay_annotation_topic", "mesh/replay_annotations").value
        )
        self._publish_live_annotations = bool(
            self.declare_parameter("publish_live_annotations", True).value
        )
        self._link_up_override_value = bool(self.declare_parameter("link_up", True).value)
        self._link_up_override_enabled = bool(
            self.declare_parameter("link_up_override", False).value
        )

        self._store = StoreForwardStore(db_path=self._storage_path, max_bytes=self._max_bytes)
        self._controller = StoreForwardController(
            store=self._store,
            heartbeat_timeout_sec=self._heartbeat_timeout_sec,
            pdr_threshold=self._pdr_threshold,
            replay_batch_size=self._replay_batch_size,
            max_replay_per_cycle=self._max_replay_per_cycle,
        )
        self._route_selector = RelayRouteSelector(
            relay_enabled=self._relay_enabled,
            activation_policy=self._relay_activation_policy,
            direct_heartbeat_timeout_sec=self._heartbeat_timeout_sec,
            direct_pdr_threshold=self._pdr_threshold,
            relay_heartbeat_timeout_sec=self._relay_heartbeat_timeout_sec,
            relay_pdr_threshold=self._relay_pdr_threshold,
        )
        self._relay_latency = RelayLatencyTracker(window_size=512)
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
        self._map_relay_publisher = self.create_publisher(
            MapTile, self._map_relay_output_topic, 10
        )
        self._detection_relay_publisher = self.create_publisher(
            FusedDetection, self._detection_relay_output_topic, 10
        )
        self._heartbeat_relay_publisher = self.create_publisher(
            String, self._heartbeat_relay_output_topic, 10
        )
        self._telemetry_relay_publisher = self.create_publisher(
            Telemetry, self._telemetry_relay_output_topic, 10
        )
        self._replay_annotation_publisher = self.create_publisher(
            String, self._replay_annotation_topic, 10
        )
        self._route_specs: dict[str, tuple[type[Any], Any]] = {
            "map_tile": (MapTile, self._map_publisher),
            "relay_map_tile": (MapTile, self._map_relay_publisher),
            "fused_detection": (FusedDetection, self._detection_publisher),
            "relay_fused_detection": (FusedDetection, self._detection_relay_publisher),
            "heartbeat": (String, self._heartbeat_publisher),
            "relay_heartbeat": (String, self._heartbeat_relay_publisher),
            "telemetry": (Telemetry, self._telemetry_publisher),
            "relay_telemetry": (Telemetry, self._telemetry_relay_publisher),
        }
        self._topic_specs: dict[str, tuple[type[Any], Any]] = {
            self._map_output_topic: (MapTile, self._map_publisher),
            self._detection_output_topic: (FusedDetection, self._detection_publisher),
            self._heartbeat_output_topic: (String, self._heartbeat_publisher),
            self._telemetry_output_topic: (Telemetry, self._telemetry_publisher),
            self._map_relay_output_topic: (MapTile, self._map_relay_publisher),
            self._detection_relay_output_topic: (
                FusedDetection,
                self._detection_relay_publisher,
            ),
            self._heartbeat_relay_output_topic: (String, self._heartbeat_relay_publisher),
            self._telemetry_relay_output_topic: (Telemetry, self._telemetry_relay_publisher),
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
        self._relay_heartbeat_subscription = None
        if self._relay_heartbeat_topic:
            self._relay_heartbeat_subscription = self.create_subscription(
                String, self._relay_heartbeat_topic, self._handle_relay_connectivity_heartbeat, 10
            )
        self._relay_pdr_subscription = None
        if self._relay_pdr_topic:
            self._relay_pdr_subscription = self.create_subscription(
                Float32, self._relay_pdr_topic, self._handle_relay_pdr, 10
            )

        self._relay_map_subscription = None
        if self._relay_map_input_topic:
            self._relay_map_subscription = self.create_subscription(
                MapTile, self._relay_map_input_topic, self._handle_map_tile_relay_ingress, 10
            )
        self._relay_detection_subscription = None
        if self._relay_detection_input_topic:
            self._relay_detection_subscription = self.create_subscription(
                FusedDetection,
                self._relay_detection_input_topic,
                self._handle_detection_relay_ingress,
                10,
            )
        self._relay_heartbeat_input_subscription = None
        if self._relay_heartbeat_input_topic:
            self._relay_heartbeat_input_subscription = self.create_subscription(
                String,
                self._relay_heartbeat_input_topic,
                self._handle_heartbeat_relay_ingress,
                10,
            )
        self._relay_telemetry_subscription = None
        if self._relay_telemetry_input_topic:
            self._relay_telemetry_subscription = self.create_subscription(
                Telemetry,
                self._relay_telemetry_input_topic,
                self._handle_telemetry_relay_ingress,
                10,
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
            "map_relay_output_topic",
            "detection_relay_output_topic",
            "heartbeat_relay_output_topic",
            "telemetry_relay_output_topic",
            "relay_map_input_topic",
            "relay_detection_input_topic",
            "relay_heartbeat_input_topic",
            "relay_telemetry_input_topic",
            "relay_heartbeat_topic",
            "relay_pdr_topic",
            "replay_annotation_topic",
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
                self._route_selector.set_direct_thresholds(
                    heartbeat_timeout_sec=value,
                    pdr_threshold=self._pdr_threshold,
                )
                continue

            if param.name == "pdr_threshold":
                value = float(param.value)
                if value < 0.0 or value > 1.0:
                    return SetParametersResult(successful=False)
                self._pdr_threshold = value
                self._controller.connectivity.pdr_threshold = value
                self._route_selector.set_direct_thresholds(
                    heartbeat_timeout_sec=self._heartbeat_timeout_sec,
                    pdr_threshold=value,
                )
                continue

            if param.name == "relay_enabled":
                self._relay_enabled = bool(param.value)
                self._route_selector.relay_enabled = self._relay_enabled
                continue

            if param.name == "relay_activation_policy":
                self._relay_activation_policy = str(param.value)
                self._route_selector.activation_policy = self._relay_activation_policy
                continue

            if param.name == "relay_heartbeat_timeout_sec":
                value = float(param.value)
                if value <= 0.0:
                    return SetParametersResult(successful=False)
                self._relay_heartbeat_timeout_sec = value
                self._route_selector.set_relay_thresholds(
                    heartbeat_timeout_sec=value,
                    pdr_threshold=self._relay_pdr_threshold,
                )
                continue

            if param.name == "relay_pdr_threshold":
                value = float(param.value)
                if value < 0.0 or value > 1.0:
                    return SetParametersResult(successful=False)
                self._relay_pdr_threshold = value
                self._route_selector.set_relay_thresholds(
                    heartbeat_timeout_sec=self._relay_heartbeat_timeout_sec,
                    pdr_threshold=value,
                )
                continue

            if param.name == "max_bytes":
                value = int(param.value)
                if value <= 0:
                    return SetParametersResult(successful=False)
                self._max_bytes = value
                self._store.set_max_bytes(value)
                continue

            if param.name == "replay_batch_size":
                value = int(param.value)
                if value <= 0:
                    return SetParametersResult(successful=False)
                self._replay_batch_size = value
                self._controller.set_replay_limits(
                    replay_batch_size=self._replay_batch_size,
                    max_replay_per_cycle=self._max_replay_per_cycle,
                )
                continue

            if param.name == "max_replay_per_cycle":
                value = int(param.value)
                if value <= 0:
                    return SetParametersResult(successful=False)
                self._max_replay_per_cycle = value
                self._controller.set_replay_limits(
                    replay_batch_size=self._replay_batch_size,
                    max_replay_per_cycle=self._max_replay_per_cycle,
                )
                continue

            if param.name == "publish_live_annotations":
                self._publish_live_annotations = bool(param.value)
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
        self._route_selector.observe_direct_heartbeat()

    def _handle_relay_connectivity_heartbeat(self, _msg: String) -> None:
        self._route_selector.observe_relay_heartbeat()

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
        self._route_selector.observe_direct_pdr(float(msg.data))

    def _handle_relay_pdr(self, msg: Float32) -> None:
        self._route_selector.observe_relay_pdr(float(msg.data))

    def _handle_map_tile_relay_ingress(self, msg: MapTile) -> None:
        self._handle_outbound(
            message=msg,
            input_topic=self._relay_map_input_topic,
            output_topic=self._map_output_topic,
            route_key="relay_map_tile",
            message_kind="map_tile",
            route_decision=self._relay_ingress_decision(),
            relay_hop=1,
        )

    def _handle_detection_relay_ingress(self, msg: FusedDetection) -> None:
        self._handle_outbound(
            message=msg,
            input_topic=self._relay_detection_input_topic,
            output_topic=self._detection_output_topic,
            route_key="relay_fused_detection",
            message_kind="fused_detection",
            route_decision=self._relay_ingress_decision(),
            relay_hop=1,
        )

    def _handle_heartbeat_relay_ingress(self, msg: String) -> None:
        self._handle_outbound(
            message=msg,
            input_topic=self._relay_heartbeat_input_topic,
            output_topic=self._heartbeat_output_topic,
            route_key="relay_heartbeat",
            message_kind="heartbeat",
            route_decision=self._relay_ingress_decision(),
            relay_hop=1,
        )

    def _handle_telemetry_relay_ingress(self, msg: Telemetry) -> None:
        self._handle_outbound(
            message=msg,
            input_topic=self._relay_telemetry_input_topic,
            output_topic=self._telemetry_output_topic,
            route_key="relay_telemetry",
            message_kind="telemetry",
            route_decision=self._relay_ingress_decision(),
            relay_hop=1,
        )

    def _handle_outbound(
        self,
        *,
        message: Any,
        input_topic: str,
        output_topic: str,
        route_key: str,
        message_kind: str,
        route_decision: RelayRouteDecision | None = None,
        relay_hop: int = 0,
    ) -> None:
        decision = route_decision or self._select_route_decision()
        target_topic, target_route_key = self._resolve_route_target(
            delivery_mode=decision.delivery_mode,
            output_topic=output_topic,
            route_key=route_key,
        )
        payload = serialize_message_payload(message)
        payload_hash = compute_payload_hash(payload)
        dedupe_key = build_dedupe_key(
            message_kind=message_kind,
            topic=input_topic,
            message=message,
            payload_hash=payload_hash,
        )
        event_ts = extract_event_timestamp(message)
        source_vehicle_id = self._source_vehicle_id_from_message(
            message,
            input_topic=input_topic,
            is_relay_ingress=route_key.startswith("relay_"),
        )
        if route_key.startswith("relay_") and not source_vehicle_id:
            self.get_logger().warning(
                "relay ingress message missing source vehicle metadata "
                f"(topic={input_topic}, route={route_key})"
            )
        relay_vehicle_id = self._relay_vehicle_id if decision.delivery_mode == "relay" else ""
        effective_relay_hop = max(relay_hop, 1 if decision.delivery_mode == "relay" else 0)

        if self._link_up_override_enabled:
            selected_link_up = self._link_up_override_value
        else:
            selected_link_up = decision.selected_link_up

        if self._relay_enabled and not self._link_up_override_enabled:
            self._controller.set_connectivity_override(enabled=True, link_up=selected_link_up)

        disposition = self._controller.handle_outbound(
            topic=target_topic,
            route_key=target_route_key,
            message_kind=message_kind,
            event_ts=event_ts,
            dedupe_key=dedupe_key,
            payload=payload,
            payload_hash=payload_hash,
            source_vehicle_id=source_vehicle_id,
            relay_vehicle_id=relay_vehicle_id,
            relay_hop=effective_relay_hop,
            relay_delivery_mode=decision.delivery_mode,
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

    def _select_route_decision(self) -> RelayRouteDecision:
        if not self._relay_enabled:
            return RelayRouteDecision(
                delivery_mode="direct",
                selected_link_up=self._controller.connectivity.is_link_up(),
                reason="relay-disabled",
            )
        return self._route_selector.select_route()

    def _relay_ingress_decision(self) -> RelayRouteDecision:
        """Relay ingress still depends on Ranger->GCS connectivity for buffering."""
        return RelayRouteDecision(
            delivery_mode="relay",
            selected_link_up=self._controller.connectivity.is_link_up(),
            reason="relay-ingress",
        )

    def _resolve_route_target(
        self, *, delivery_mode: str, output_topic: str, route_key: str
    ) -> tuple[str, str]:
        mode = delivery_mode.strip().lower()
        if mode != "relay":
            return output_topic, route_key

        relay_topic_by_route = {
            "map_tile": self._map_relay_output_topic,
            "fused_detection": self._detection_relay_output_topic,
            "heartbeat": self._heartbeat_relay_output_topic,
            "telemetry": self._telemetry_relay_output_topic,
        }
        relay_route_key_by_route = {
            "map_tile": "relay_map_tile",
            "fused_detection": "relay_fused_detection",
            "heartbeat": "relay_heartbeat",
            "telemetry": "relay_telemetry",
        }
        return (
            relay_topic_by_route.get(route_key, output_topic),
            relay_route_key_by_route.get(route_key, route_key),
        )

    def _source_vehicle_id_from_message(
        self,
        message: Any,
        *,
        input_topic: str,
        is_relay_ingress: bool,
    ) -> str:
        candidate = getattr(message, "vehicle_id", "")
        if isinstance(candidate, str) and candidate:
            return candidate

        hazard_payload_json = getattr(message, "hazard_payload_json", "")
        if isinstance(hazard_payload_json, str) and hazard_payload_json:
            try:
                payload = json.loads(hazard_payload_json)
                if isinstance(payload, dict):
                    payload_source = payload.get("source_vehicle_id") or payload.get(
                        "vehicle_id"
                    )
                    if isinstance(payload_source, str) and payload_source:
                        return payload_source
            except json.JSONDecodeError:
                # Not all detections carry JSON payload metadata.
                pass

        topic_source = self._extract_vehicle_id_from_topic(input_topic)
        if topic_source:
            return topic_source

        # For relay ingress, unknown provenance is safer than wrong provenance.
        if is_relay_ingress:
            return ""
        if self._source_vehicle_id:
            return self._source_vehicle_id
        return self._vehicle_id

    @staticmethod
    def _extract_vehicle_id_from_topic(topic: str) -> str:
        for part in topic.split("/"):
            candidate = part.strip()
            if not candidate:
                continue
            lower = candidate.lower()
            if lower.startswith(("scout", "ranger", "uav", "drone", "vehicle")) and any(
                char.isdigit() for char in candidate
            ):
                return candidate
        return ""

    def _publish_record(
        self,
        ingest_seq: int,
        topic: str,
        route_key: str,
        message_kind: str,
        payload: bytes,
        metadata: ReplayMetadata,
    ) -> None:
        spec = self._topic_specs.get(topic) or self._route_specs.get(route_key)
        if spec is None:
            self.get_logger().warning(f"no route configured for replay key '{route_key}'")
            return
        message_type, publisher = spec
        message = deserialize_message_payload(message_type, payload)
        publisher.publish(message)
        published_at_ts = time.time()
        relay_envelope = RelayEnvelope(
            source_vehicle_id=metadata.source_vehicle_id,
            relay_vehicle_id=metadata.relay_vehicle_id,
            relay_hop=metadata.relay_hop,
            delivery_mode=metadata.relay_delivery_mode,
        )
        self._relay_latency.record(
            envelope=relay_envelope,
            original_event_ts=metadata.original_event_ts,
            replayed_at_ts=metadata.replayed_at_ts,
            published_at_ts=published_at_ts,
        )
        if metadata.delivery_mode == "replayed" or self._publish_live_annotations:
            annotation = {
                "delivery_mode": metadata.delivery_mode,
                "original_event_ts": metadata.original_event_ts,
                "replayed_at_ts": metadata.replayed_at_ts,
                "published_at_ts": published_at_ts,
                "ingest_seq": ingest_seq if ingest_seq > 0 else None,
                "topic": topic,
                "route_key": route_key,
                "message_kind": message_kind,
                "priority_class": metadata.priority_class,
                "dedupe_key": metadata.dedupe_key,
                "payload_hash": metadata.payload_hash,
                "relay_envelope": {
                    "source_vehicle_id": relay_envelope.source_vehicle_id,
                    "relay_vehicle_id": relay_envelope.relay_vehicle_id,
                    "relay_hop": relay_envelope.relay_hop,
                    "delivery_mode": relay_envelope.delivery_mode,
                },
            }
            self._replay_annotation_publisher.publish(
                String(data=json.dumps(annotation, separators=(",", ":")))
            )

        if ingest_seq > 0:
            self.get_logger().debug(
                "replayed buffered message "
                f"ingest_seq={ingest_seq} route={route_key} "
                f"priority={metadata.priority_class}"
            )

    def _sync_connectivity(self) -> None:
        if self._relay_enabled and not self._link_up_override_enabled:
            decision = self._route_selector.select_route()
            self._controller.set_connectivity_override(
                enabled=True,
                link_up=decision.selected_link_up,
            )
        self._controller.sync_connectivity_and_flush(self._publish_record)

    def _log_metrics(self) -> None:
        metrics = self._store.metrics()
        replay_metrics = self._controller.replay_metrics
        relay_snapshot = self._relay_latency.snapshot()
        current_route = self._select_route_decision()
        link_state = "up" if self._controller.connectivity.is_link_up() else "down"
        self.get_logger().info(
            "store-forward metrics "
            f"link={link_state} queued={metrics.queued_count} "
            f"(control={metrics.queued_count_by_class.get('control', 0)} "
            f"telemetry={metrics.queued_count_by_class.get('telemetry', 0)} "
            f"tiles={metrics.queued_count_by_class.get('tiles', 0)} "
            f"bulk={metrics.queued_count_by_class.get('bulk', 0)}) "
            f"(direct_backlog={metrics.queued_count_by_delivery_mode.get('direct', 0)} "
            f"relay_backlog={metrics.queued_count_by_delivery_mode.get('relay', 0)}) "
            f"bytes={metrics.bytes_on_disk} dedupe_hits={metrics.dedupe_hits} "
            f"evicted={metrics.evicted_count} "
            f"oldest_age_sec={metrics.oldest_buffered_age_sec:.2f} "
            f"replay_total={replay_metrics.replayed_total} "
            f"replay_last_batch={replay_metrics.last_flush_count} "
            f"replay_rate={replay_metrics.last_flush_rate_msgs_per_sec:.1f}/s "
            f"relay_mode={current_route.delivery_mode} "
            f"relay_forwarded_total={relay_snapshot.relay_forwarded_total} "
            f"relay_latency_p50={relay_snapshot.relay_latency_p50_sec:.3f}s "
            f"relay_latency_p95={relay_snapshot.relay_latency_p95_sec:.3f}s "
            f"replay_lag_p50={relay_snapshot.replay_lag_p50_sec:.3f}s "
            f"replay_lag_p95={relay_snapshot.replay_lag_p95_sec:.3f}s"
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
