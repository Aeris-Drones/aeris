"""Unit tests for store-and-forward persistence and link-state buffering behavior."""

from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
MESH_AGENT_SRC = REPO_ROOT / "software" / "edge" / "src" / "aeris_mesh_agent"
if str(MESH_AGENT_SRC) not in sys.path:
    sys.path.insert(0, str(MESH_AGENT_SRC))

from aeris_mesh_agent.store_forward_core import (
    ConnectivityState,
    ReplayMetadata,
    StoreForwardController,
)
from aeris_mesh_agent.relay_routing import RelayEnvelope, RelayLatencyTracker, RelayRouteSelector
from aeris_mesh_agent.store_forward_store import StoreForwardStore


def _make_payload(size: int) -> bytes:
    return ("x" * size).encode("utf-8")


def test_controller_disconnect_buffer_reconnect_flushes_in_order(tmp_path: Path) -> None:
    db_path = tmp_path / "buffer" / "store-forward.db"
    store = StoreForwardStore(db_path=str(db_path), max_bytes=1024 * 1024)
    controller = StoreForwardController(store=store, heartbeat_timeout_sec=5.0)

    published: list[tuple[int, str, str, str, bytes, ReplayMetadata]] = []

    def _publish(
        ingest_seq: int,
        topic: str,
        route_key: str,
        message_kind: str,
        payload: bytes,
        metadata: ReplayMetadata,
    ) -> None:
        published.append((ingest_seq, topic, route_key, message_kind, payload, metadata))

    controller.set_connectivity_override(enabled=True, link_up=False)
    for i in range(3):
        controller.handle_outbound(
            topic="map/tiles_out",
            route_key="map_tile",
            message_kind="map_tile",
            event_ts=1000.0 + i,
            dedupe_key=f"map:tile-{i}",
            payload=_make_payload(16 + i),
            payload_hash=f"hash-{i}",
            publish_callback=_publish,
        )

    assert store.pending_count() == 3

    controller.set_connectivity_override(enabled=True, link_up=True)
    controller.flush_pending(_publish)

    assert [seq for seq, _, _, _, _, _ in published] == [1, 2, 3]
    assert [topic for _, topic, _, _, _, _ in published] == [
        "map/tiles_out",
        "map/tiles_out",
        "map/tiles_out",
    ]
    assert all(meta.delivery_mode == "replayed" for _, _, _, _, _, meta in published)
    assert store.pending_count() == 0


def test_controller_replay_prioritizes_classes_and_preserves_order(tmp_path: Path) -> None:
    db_path = tmp_path / "buffer" / "store-forward.db"
    store = StoreForwardStore(db_path=str(db_path), max_bytes=1024 * 1024)
    controller = StoreForwardController(store=store, heartbeat_timeout_sec=5.0)

    published: list[tuple[int, str, str, str, bytes, ReplayMetadata]] = []

    def _publish(
        ingest_seq: int,
        topic: str,
        route_key: str,
        message_kind: str,
        payload: bytes,
        metadata: ReplayMetadata,
    ) -> None:
        published.append((ingest_seq, topic, route_key, message_kind, payload, metadata))

    controller.set_connectivity_override(enabled=True, link_up=False)
    queued = [
        ("detections/fused_out", "fused_detection", "fused_detection"),
        ("telemetry_out", "telemetry", "telemetry"),
        ("map/tiles_out", "map_tile", "map_tile"),
        ("mesh/heartbeat_out", "heartbeat", "heartbeat"),
        ("telemetry_out", "telemetry", "telemetry"),
        ("detections/fused_out", "fused_detection", "fused_detection"),
    ]
    for idx, (topic, route_key, message_kind) in enumerate(queued):
        controller.handle_outbound(
            topic=topic,
            route_key=route_key,
            message_kind=message_kind,
            event_ts=2000.0 + idx,
            dedupe_key=f"{route_key}:{idx}",
            payload=_make_payload(10 + idx),
            payload_hash=f"hash-{idx}",
            publish_callback=_publish,
        )

    controller.set_connectivity_override(enabled=True, link_up=True)
    controller.flush_pending(_publish, batch_size=32, max_records=32)

    assert [seq for seq, _, _, _, _, _ in published] == [4, 2, 5, 3, 1, 6]
    assert [meta.priority_class for _, _, _, _, _, meta in published] == [
        "control",
        "telemetry",
        "telemetry",
        "tiles",
        "bulk",
        "bulk",
    ]
    assert all(meta.delivery_mode == "replayed" for _, _, _, _, _, meta in published)
    assert store.pending_count() == 0


def test_controller_flush_recovers_after_link_drop_mid_replay(tmp_path: Path) -> None:
    db_path = tmp_path / "buffer" / "store-forward.db"
    store = StoreForwardStore(db_path=str(db_path), max_bytes=1024 * 1024)
    controller = StoreForwardController(store=store, heartbeat_timeout_sec=5.0)

    published: list[int] = []
    dropped_once = False

    def _publish(
        ingest_seq: int,
        topic: str,
        route_key: str,
        message_kind: str,
        payload: bytes,
        metadata: ReplayMetadata,
    ) -> None:
        nonlocal dropped_once
        del topic, route_key, message_kind, payload, metadata
        published.append(ingest_seq)
        if not dropped_once:
            dropped_once = True
            controller.set_connectivity_override(enabled=True, link_up=False)

    controller.set_connectivity_override(enabled=True, link_up=False)
    for idx in range(3):
        controller.handle_outbound(
            topic="telemetry_out",
            route_key="telemetry",
            message_kind="telemetry",
            event_ts=3000.0 + idx,
            dedupe_key=f"telemetry:{idx}",
            payload=_make_payload(16 + idx),
            payload_hash=f"t-hash-{idx}",
            publish_callback=_publish,
        )

    controller.set_connectivity_override(enabled=True, link_up=True)
    flushed_first = controller.flush_pending(_publish, batch_size=8, max_records=8)
    assert flushed_first == 1
    assert store.pending_count() == 2

    controller.set_connectivity_override(enabled=True, link_up=True)
    flushed_second = controller.flush_pending(_publish, batch_size=8, max_records=8)
    assert flushed_second == 2
    assert store.pending_count() == 0
    assert published == [1, 2, 3]


def test_controller_keeps_live_telemetry_flowing_while_backlog_drains(tmp_path: Path) -> None:
    db_path = tmp_path / "buffer" / "store-forward.db"
    store = StoreForwardStore(db_path=str(db_path), max_bytes=1024 * 1024)
    controller = StoreForwardController(
        store=store,
        heartbeat_timeout_sec=5.0,
        replay_batch_size=8,
        max_replay_per_cycle=8,
    )

    published: list[tuple[int, ReplayMetadata]] = []

    def _publish(
        ingest_seq: int,
        topic: str,
        route_key: str,
        message_kind: str,
        payload: bytes,
        metadata: ReplayMetadata,
    ) -> None:
        del topic, route_key, message_kind, payload
        published.append((ingest_seq, metadata))

    controller.set_connectivity_override(enabled=True, link_up=False)
    controller.handle_outbound(
        topic="map/tiles_out",
        route_key="map_tile",
        message_kind="map_tile",
        event_ts=4000.0,
        dedupe_key="map:stale",
        payload=_make_payload(20),
        payload_hash="stale-hash",
        publish_callback=_publish,
    )
    assert store.pending_count() == 1

    controller.set_connectivity_override(enabled=True, link_up=True)
    result = controller.handle_outbound(
        topic="telemetry_out",
        route_key="telemetry",
        message_kind="telemetry",
        event_ts=4001.0,
        dedupe_key="telemetry:live",
        payload=_make_payload(22),
        payload_hash="live-hash",
        publish_callback=_publish,
    )

    assert result == "published-live"
    assert published[0][0] == 0
    assert published[0][1].delivery_mode == "live"
    assert published[0][1].priority_class == "telemetry"
    assert any(meta.delivery_mode == "replayed" for _, meta in published[1:])
    assert store.pending_count() == 0


def test_controller_flush_handles_publish_exceptions_without_data_loss(tmp_path: Path) -> None:
    db_path = tmp_path / "buffer" / "store-forward.db"
    store = StoreForwardStore(db_path=str(db_path), max_bytes=1024 * 1024)
    controller = StoreForwardController(store=store, heartbeat_timeout_sec=5.0)

    controller.set_connectivity_override(enabled=True, link_up=False)
    for idx in range(3):
        controller.handle_outbound(
            topic="telemetry_out",
            route_key="telemetry",
            message_kind="telemetry",
            event_ts=5000.0 + idx,
            dedupe_key=f"telemetry:{idx}",
            payload=_make_payload(18 + idx),
            payload_hash=f"hash-{idx}",
            publish_callback=lambda *args, **kwargs: None,
        )

    published: list[int] = []
    attempts: dict[int, int] = {}

    def _publish_fail_once(
        ingest_seq: int,
        topic: str,
        route_key: str,
        message_kind: str,
        payload: bytes,
        metadata: ReplayMetadata,
    ) -> None:
        del topic, route_key, message_kind, payload, metadata
        attempts[ingest_seq] = attempts.get(ingest_seq, 0) + 1
        if ingest_seq == 2 and attempts[ingest_seq] == 1:
            raise RuntimeError("transient publish failure")
        published.append(ingest_seq)

    controller.set_connectivity_override(enabled=True, link_up=True)
    flushed_first = controller.flush_pending(_publish_fail_once, batch_size=8, max_records=8)
    assert flushed_first == 1
    assert store.pending_count() == 2

    flushed_second = controller.flush_pending(_publish_fail_once, batch_size=8, max_records=8)
    assert flushed_second == 2
    assert store.pending_count() == 0
    assert published == [1, 2, 3]


def test_store_recovers_pending_records_after_restart(tmp_path: Path) -> None:
    db_path = tmp_path / "buffer" / "store-forward.db"
    first = StoreForwardStore(db_path=str(db_path), max_bytes=1024 * 1024)
    first.enqueue(
        topic="detections/fused_out",
        event_ts=1.0,
        dedupe_key="detection:candidate-1",
        payload=_make_payload(24),
        payload_hash="d-hash-1",
    )
    first.enqueue(
        topic="orchestrator/heartbeat_out",
        event_ts=2.0,
        dedupe_key="heartbeat:vehicle-1",
        payload=_make_payload(25),
        payload_hash="d-hash-2",
    )
    first.close()

    second = StoreForwardStore(db_path=str(db_path), max_bytes=1024 * 1024)
    rows = second.peek_pending(limit=10)
    assert [row.ingest_seq for row in rows] == [1, 2]
    assert [row.topic for row in rows] == ["detections/fused_out", "orchestrator/heartbeat_out"]


def test_store_deduplicates_and_tracks_dedupe_hits(tmp_path: Path) -> None:
    db_path = tmp_path / "buffer" / "store-forward.db"
    store = StoreForwardStore(db_path=str(db_path), max_bytes=1024 * 1024)

    first = store.enqueue(
        topic="map/tiles_out",
        event_ts=10.0,
        dedupe_key="map:1/2/3",
        payload=_make_payload(12),
        payload_hash="payload-hash",
    )
    second = store.enqueue(
        topic="map/tiles_out",
        event_ts=11.0,
        dedupe_key="map:1/2/3",
        payload=_make_payload(12),
        payload_hash="payload-hash",
    )

    assert first.accepted is True
    assert second.accepted is False
    assert store.pending_count() == 1
    metrics = store.metrics(now_wallclock=20.0)
    assert metrics.dedupe_hits == 1


def test_store_allows_reenqueue_after_ack(tmp_path: Path) -> None:
    db_path = tmp_path / "buffer" / "store-forward.db"
    store = StoreForwardStore(db_path=str(db_path), max_bytes=1024 * 1024)

    first = store.enqueue(
        topic="map/tiles_out",
        event_ts=1.0,
        dedupe_key="map:stable",
        payload=_make_payload(12),
        payload_hash="payload-hash",
    )
    assert first.accepted is True
    assert first.ingest_seq is not None

    store.ack_through(first.ingest_seq)

    replay = store.enqueue(
        topic="map/tiles_out",
        event_ts=2.0,
        dedupe_key="map:stable",
        payload=_make_payload(12),
        payload_hash="payload-hash",
    )
    assert replay.accepted is True
    assert store.pending_count() == 1


def test_store_allows_reenqueue_after_eviction(tmp_path: Path) -> None:
    db_path = tmp_path / "buffer" / "store-forward.db"
    store = StoreForwardStore(db_path=str(db_path), max_bytes=3)

    first = store.enqueue(
        topic="map/tiles_out",
        event_ts=1.0,
        dedupe_key="map:stable",
        payload=b"aaa",
        payload_hash="payload-hash",
    )
    assert first.accepted is True

    store.enqueue(
        topic="map/tiles_out",
        event_ts=2.0,
        dedupe_key="map:other",
        payload=b"bbb",
        payload_hash="payload-hash-2",
    )

    replay = store.enqueue(
        topic="map/tiles_out",
        event_ts=3.0,
        dedupe_key="map:stable",
        payload=b"aaa",
        payload_hash="payload-hash",
    )
    assert replay.accepted is True


def test_store_rejects_enqueue_when_record_exceeds_capacity(tmp_path: Path) -> None:
    db_path = tmp_path / "buffer" / "store-forward.db"
    store = StoreForwardStore(db_path=str(db_path), max_bytes=2)

    result = store.enqueue(
        topic="map/tiles_out",
        event_ts=1.0,
        dedupe_key="map:oversized",
        payload=b"abc",
        payload_hash="oversized-hash",
    )

    assert result.accepted is False
    assert result.reason == "dropped-over-capacity"
    assert result.ingest_seq is None
    assert store.pending_count() == 0


def test_store_evicts_oldest_when_size_limit_exceeded(tmp_path: Path) -> None:
    db_path = tmp_path / "buffer" / "store-forward.db"
    store = StoreForwardStore(db_path=str(db_path), max_bytes=80)

    store.enqueue(
        topic="map/tiles_out",
        event_ts=1.0,
        dedupe_key="map:1",
        payload=_make_payload(40),
        payload_hash="h1",
    )
    store.enqueue(
        topic="map/tiles_out",
        event_ts=2.0,
        dedupe_key="map:2",
        payload=_make_payload(40),
        payload_hash="h2",
    )
    store.enqueue(
        topic="map/tiles_out",
        event_ts=3.0,
        dedupe_key="map:3",
        payload=_make_payload(40),
        payload_hash="h3",
    )

    pending = store.peek_pending(limit=10)
    assert len(pending) < 3
    assert pending[-1].dedupe_key == "map:3"
    metrics = store.metrics(now_wallclock=10.0)
    assert metrics.evicted_count >= 1


def test_store_eviction_prefers_lower_priority_before_control(tmp_path: Path) -> None:
    db_path = tmp_path / "buffer" / "store-forward.db"
    store = StoreForwardStore(db_path=str(db_path), max_bytes=80)

    store.enqueue(
        topic="mesh/heartbeat_out",
        route_key="heartbeat",
        message_kind="heartbeat",
        event_ts=1.0,
        dedupe_key="control:1",
        payload=_make_payload(30),
        payload_hash="control-hash",
    )
    store.enqueue(
        topic="detections/fused_out",
        route_key="fused_detection",
        message_kind="fused_detection",
        event_ts=2.0,
        dedupe_key="bulk:1",
        payload=_make_payload(30),
        payload_hash="bulk-hash-1",
    )
    store.enqueue(
        topic="detections/fused_out",
        route_key="fused_detection",
        message_kind="fused_detection",
        event_ts=3.0,
        dedupe_key="bulk:2",
        payload=_make_payload(30),
        payload_hash="bulk-hash-2",
    )

    pending = store.peek_pending(limit=10)
    dedupe_keys = {row.dedupe_key for row in pending}
    assert "control:1" in dedupe_keys
    assert "bulk:2" in dedupe_keys
    assert "bulk:1" not in dedupe_keys


def test_store_metrics_include_priority_class_counts(tmp_path: Path) -> None:
    db_path = tmp_path / "buffer" / "store-forward.db"
    store = StoreForwardStore(db_path=str(db_path), max_bytes=1024 * 1024)
    store.enqueue(
        topic="mesh/heartbeat_out",
        route_key="heartbeat",
        message_kind="heartbeat",
        event_ts=1.0,
        dedupe_key="heartbeat:a",
        payload=b"a",
        payload_hash="h-a",
    )
    store.enqueue(
        topic="telemetry_out",
        route_key="telemetry",
        message_kind="telemetry",
        event_ts=2.0,
        dedupe_key="telemetry:a",
        payload=b"b",
        payload_hash="h-b",
    )
    store.enqueue(
        topic="map/tiles_out",
        route_key="map_tile",
        message_kind="map_tile",
        event_ts=3.0,
        dedupe_key="tile:a",
        payload=b"c",
        payload_hash="h-c",
    )
    store.enqueue(
        topic="detections/fused_out",
        route_key="fused_detection",
        message_kind="fused_detection",
        event_ts=4.0,
        dedupe_key="detection:a",
        payload=b"d",
        payload_hash="h-d",
    )
    metrics = store.metrics(now_wallclock=10.0)
    assert metrics.queued_count == 4
    assert metrics.queued_count_by_class == {
        "control": 1,
        "telemetry": 1,
        "tiles": 1,
        "bulk": 1,
    }


def test_connectivity_state_uses_heartbeat_timeout_and_pdr_threshold() -> None:
    state = ConnectivityState(
        heartbeat_timeout_sec=2.0,
        pdr_threshold=0.8,
        now_fn=lambda: 100.0,
    )

    state.observe_heartbeat(99.5)
    state.observe_pdr(0.9)
    assert state.is_link_up() is True

    state.observe_pdr(0.2)
    assert state.is_link_up() is False


def test_relay_route_selector_prefers_direct_then_falls_back_to_relay() -> None:
    current_time = {"value": 100.0}
    selector = RelayRouteSelector(
        relay_enabled=True,
        activation_policy="auto",
        direct_heartbeat_timeout_sec=2.0,
        direct_pdr_threshold=0.0,
        relay_heartbeat_timeout_sec=2.0,
        relay_pdr_threshold=0.0,
        now_fn=lambda: current_time["value"],
    )

    selector.observe_direct_heartbeat()
    selector.observe_relay_heartbeat()
    assert selector.select_route().delivery_mode == "direct"

    current_time["value"] = 103.1
    selector.observe_relay_heartbeat()
    decision = selector.select_route()
    assert decision.delivery_mode == "relay"
    assert decision.selected_link_up is True

    current_time["value"] = 106.5
    decision = selector.select_route()
    assert decision.delivery_mode == "direct"
    assert decision.selected_link_up is False


def test_controller_replays_relay_metadata_fields(tmp_path: Path) -> None:
    db_path = tmp_path / "buffer" / "store-forward.db"
    store = StoreForwardStore(db_path=str(db_path), max_bytes=1024 * 1024)
    controller = StoreForwardController(store=store, heartbeat_timeout_sec=5.0)
    controller.set_connectivity_override(enabled=True, link_up=False)

    published: list[ReplayMetadata] = []

    def _publish(
        ingest_seq: int,
        topic: str,
        route_key: str,
        message_kind: str,
        payload: bytes,
        metadata: ReplayMetadata,
    ) -> None:
        del ingest_seq, topic, route_key, message_kind, payload
        published.append(metadata)

    controller.handle_outbound(
        topic="relay/telemetry_out",
        route_key="relay_telemetry",
        message_kind="telemetry",
        event_ts=10.0,
        dedupe_key="telemetry:relay:1",
        payload=b"payload",
        payload_hash="relay-hash",
        source_vehicle_id="scout1",
        relay_vehicle_id="ranger1",
        relay_hop=1,
        relay_delivery_mode="relay",
        publish_callback=_publish,
    )
    metrics = store.metrics(now_wallclock=20.0)
    assert metrics.queued_count_by_delivery_mode["relay"] == 1

    controller.set_connectivity_override(enabled=True, link_up=True)
    controller.flush_pending(_publish, batch_size=8, max_records=8)

    assert len(published) == 1
    assert published[0].source_vehicle_id == "scout1"
    assert published[0].relay_vehicle_id == "ranger1"
    assert published[0].relay_hop == 1
    assert published[0].relay_delivery_mode == "relay"


def test_store_dedupe_stays_stable_across_direct_and_relay_modes(tmp_path: Path) -> None:
    db_path = tmp_path / "buffer" / "store-forward.db"
    store = StoreForwardStore(db_path=str(db_path), max_bytes=1024 * 1024)

    first = store.enqueue(
        topic="telemetry_out",
        route_key="telemetry",
        message_kind="telemetry",
        event_ts=1.0,
        dedupe_key="telemetry:scout1:1:0",
        payload=b"same-payload",
        payload_hash="same-hash",
        relay_delivery_mode="direct",
    )
    second = store.enqueue(
        topic="relay/telemetry_out",
        route_key="relay_telemetry",
        message_kind="telemetry",
        event_ts=1.1,
        dedupe_key="telemetry:scout1:1:0",
        payload=b"same-payload",
        payload_hash="same-hash",
        relay_delivery_mode="relay",
    )

    assert first.accepted is True
    assert second.accepted is False
    assert store.pending_count() == 1
    assert store.metrics(now_wallclock=10.0).dedupe_hits == 1


def test_relay_latency_tracker_reports_percentiles() -> None:
    tracker = RelayLatencyTracker(window_size=16)
    envelope = RelayEnvelope(
        source_vehicle_id="scout1",
        relay_vehicle_id="ranger1",
        relay_hop=1,
        delivery_mode="relay",
    )

    tracker.record(
        envelope=envelope,
        original_event_ts=10.0,
        replayed_at_ts=10.4,
        published_at_ts=10.8,
    )
    tracker.record(
        envelope=envelope,
        original_event_ts=20.0,
        replayed_at_ts=20.2,
        published_at_ts=20.5,
    )

    snapshot = tracker.snapshot()
    assert snapshot.relay_forwarded_total == 2
    assert snapshot.relay_latency_p95_sec >= snapshot.relay_latency_p50_sec
    assert snapshot.replay_lag_p95_sec >= snapshot.replay_lag_p50_sec
