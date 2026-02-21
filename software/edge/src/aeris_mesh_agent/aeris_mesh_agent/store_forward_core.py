"""Connectivity and replay orchestration for store-and-forward behavior."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import time

from .store_forward_store import StoreForwardStore


@dataclass(frozen=True)
class ReplayMetadata:
    """Delivery provenance attached to each publish operation."""

    delivery_mode: str
    original_event_ts: float
    replayed_at_ts: float | None
    dedupe_key: str
    payload_hash: str
    priority_class: str


@dataclass(frozen=True)
class ReplayMetrics:
    """Replay throughput and progress counters for observability."""

    replayed_total: int
    last_flush_count: int
    last_flush_duration_sec: float
    last_flush_rate_msgs_per_sec: float


PublishCallback = Callable[[int, str, str, str, bytes, ReplayMetadata], None]


@dataclass
class ConnectivityState:
    """Evaluates link health from heartbeat freshness and optional PDR thresholds."""

    heartbeat_timeout_sec: float
    pdr_threshold: float
    now_fn: Callable[[], float] = time.monotonic
    _last_heartbeat_monotonic: float | None = None
    _latest_pdr: float | None = None
    _override_enabled: bool = False
    _override_link_up: bool = True

    def observe_heartbeat(self, timestamp_monotonic: float | None = None) -> None:
        self._last_heartbeat_monotonic = (
            self.now_fn() if timestamp_monotonic is None else float(timestamp_monotonic)
        )

    def observe_pdr(self, pdr_value: float | None) -> None:
        self._latest_pdr = None if pdr_value is None else float(pdr_value)

    def set_override(self, *, enabled: bool, link_up: bool) -> None:
        self._override_enabled = bool(enabled)
        self._override_link_up = bool(link_up)

    def is_link_up(self) -> bool:
        if self._override_enabled:
            return self._override_link_up

        if self._last_heartbeat_monotonic is None:
            return False

        heartbeat_age = self.now_fn() - self._last_heartbeat_monotonic
        heartbeat_ok = heartbeat_age <= self.heartbeat_timeout_sec

        pdr_ok = True
        if self.pdr_threshold > 0.0 and self._latest_pdr is not None:
            pdr_ok = self._latest_pdr >= self.pdr_threshold

        return heartbeat_ok and pdr_ok


class StoreForwardController:
    """Coordinates write buffering and ordered replay based on connectivity."""

    def __init__(
        self,
        *,
        store: StoreForwardStore,
        heartbeat_timeout_sec: float,
        pdr_threshold: float = 0.0,
        now_fn: Callable[[], float] = time.monotonic,
        replay_batch_size: int = 64,
        max_replay_per_cycle: int = 256,
    ) -> None:
        self._store = store
        self._connectivity = ConnectivityState(
            heartbeat_timeout_sec=float(heartbeat_timeout_sec),
            pdr_threshold=float(pdr_threshold),
            now_fn=now_fn,
        )
        self._last_link_up = self._connectivity.is_link_up()
        self._replay_batch_size = max(1, int(replay_batch_size))
        self._max_replay_per_cycle = max(
            self._replay_batch_size, int(max_replay_per_cycle)
        )
        self._replayed_total = 0
        self._last_flush_count = 0
        self._last_flush_duration_sec = 0.0

    @property
    def connectivity(self) -> ConnectivityState:
        return self._connectivity

    @property
    def replay_metrics(self) -> ReplayMetrics:
        duration = self._last_flush_duration_sec
        rate = (
            float(self._last_flush_count) / duration
            if duration > 0.0
            else 0.0
        )
        return ReplayMetrics(
            replayed_total=self._replayed_total,
            last_flush_count=self._last_flush_count,
            last_flush_duration_sec=duration,
            last_flush_rate_msgs_per_sec=rate,
        )

    def set_connectivity_override(self, *, enabled: bool, link_up: bool) -> None:
        self._connectivity.set_override(enabled=enabled, link_up=link_up)

    def set_replay_limits(self, *, replay_batch_size: int, max_replay_per_cycle: int) -> None:
        self._replay_batch_size = max(1, int(replay_batch_size))
        self._max_replay_per_cycle = max(
            self._replay_batch_size, int(max_replay_per_cycle)
        )

    def handle_outbound(
        self,
        *,
        topic: str,
        route_key: str,
        message_kind: str,
        event_ts: float,
        dedupe_key: str,
        payload: bytes,
        payload_hash: str,
        publish_callback: PublishCallback,
    ) -> str:
        """Publish immediately when healthy, otherwise persist for replay."""
        link_up = self._connectivity.is_link_up()
        has_backlog = self._store.pending_count() > 0
        priority_class = self._store.classify_priority(
            route_key=route_key,
            message_kind=message_kind,
        )

        # Keep live control traffic flowing even while replay backlog exists.
        if link_up and (not has_backlog or priority_class == "control"):
            publish_callback(
                0,
                topic,
                route_key,
                message_kind,
                payload,
                ReplayMetadata(
                    delivery_mode="live",
                    original_event_ts=float(event_ts),
                    replayed_at_ts=None,
                    dedupe_key=dedupe_key,
                    payload_hash=payload_hash,
                    priority_class=priority_class,
                ),
            )
            if has_backlog:
                self.flush_pending(
                    publish_callback,
                    batch_size=self._replay_batch_size,
                    max_records=self._max_replay_per_cycle,
                )
            return "published-live"

        result = self._store.enqueue(
            topic=topic,
            route_key=route_key,
            message_kind=message_kind,
            event_ts=event_ts,
            dedupe_key=dedupe_key,
            payload=payload,
            payload_hash=payload_hash,
        )
        if result.accepted and link_up:
            self.flush_pending(
                publish_callback,
                batch_size=self._replay_batch_size,
                max_records=self._max_replay_per_cycle,
            )
            return "buffered-and-flushed"
        if result.accepted:
            return "buffered"
        return "deduplicated"

    def sync_connectivity_and_flush(self, publish_callback: PublishCallback) -> bool:
        """Flush queued records exactly when connectivity transitions up."""
        link_up = self._connectivity.is_link_up()
        reconnected = link_up and not self._last_link_up
        self._last_link_up = link_up
        if link_up and (reconnected or self._store.pending_count() > 0):
            self.flush_pending(
                publish_callback,
                batch_size=self._replay_batch_size,
                max_records=self._max_replay_per_cycle,
            )
        return reconnected

    def flush_pending(
        self,
        publish_callback: PublishCallback,
        *,
        batch_size: int | None = None,
        max_records: int | None = None,
    ) -> int:
        """Replay buffered records by priority class with bounded cycle limits."""
        effective_batch_size = (
            self._replay_batch_size if batch_size is None else max(1, int(batch_size))
        )
        effective_max_records = (
            self._max_replay_per_cycle
            if max_records is None
            else max(1, int(max_records))
        )

        flushed = 0
        start_mono = time.monotonic()
        while self._connectivity.is_link_up() and flushed < effective_max_records:
            remaining_budget = effective_max_records - flushed
            batch_limit = min(effective_batch_size, remaining_budget)
            batch = self._store.peek_pending_prioritized(limit=batch_limit)
            if not batch:
                break

            published_seqs: list[int] = []
            for record in batch:
                if not self._connectivity.is_link_up():
                    break
                publish_callback(
                    record.ingest_seq,
                    record.topic,
                    record.route_key,
                    record.message_kind,
                    record.payload,
                    ReplayMetadata(
                        delivery_mode="replayed",
                        original_event_ts=record.event_ts,
                        replayed_at_ts=time.time(),
                        dedupe_key=record.dedupe_key,
                        payload_hash=record.payload_hash,
                        priority_class=record.priority_class,
                    ),
                )
                published_seqs.append(record.ingest_seq)

            if published_seqs:
                self._store.ack_records(published_seqs)
                flushed += len(published_seqs)

            # Connectivity dropped during this batch: leave unpublished messages queued.
            if len(published_seqs) < len(batch):
                break

        duration = max(time.monotonic() - start_mono, 0.0)
        self._last_flush_count = flushed
        self._last_flush_duration_sec = duration
        if flushed > 0:
            self._replayed_total += flushed
        return flushed
