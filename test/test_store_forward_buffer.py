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
