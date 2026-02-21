"""Connectivity and replay orchestration for store-and-forward behavior."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import time

from .store_forward_store import QueueRecord, StoreForwardStore


PublishCallback = Callable[[int, str, str, str, bytes], None]


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
    ) -> None:
        self._store = store
        self._connectivity = ConnectivityState(
            heartbeat_timeout_sec=float(heartbeat_timeout_sec),
            pdr_threshold=float(pdr_threshold),
            now_fn=now_fn,
        )
        self._last_link_up = self._connectivity.is_link_up()

    @property
    def connectivity(self) -> ConnectivityState:
        return self._connectivity

    def set_connectivity_override(self, *, enabled: bool, link_up: bool) -> None:
        self._connectivity.set_override(enabled=enabled, link_up=link_up)

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

        if link_up and not has_backlog:
            publish_callback(0, topic, route_key, message_kind, payload)
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
            self.flush_pending(publish_callback)
            return "buffered-and-flushed"
        if result.accepted:
            return "buffered"
        return "deduplicated"

    def sync_connectivity_and_flush(self, publish_callback: PublishCallback) -> bool:
        """Flush queued records exactly when connectivity transitions up."""
        link_up = self._connectivity.is_link_up()
        reconnected = link_up and not self._last_link_up
        self._last_link_up = link_up
        if reconnected:
            self.flush_pending(publish_callback)
        return reconnected

    def flush_pending(self, publish_callback: PublishCallback, *, batch_size: int = 256) -> int:
        """Replay buffered records in ingest order and acknowledge committed replay."""
        flushed = 0
        while self._connectivity.is_link_up():
            batch = self._store.peek_pending(limit=batch_size)
            if not batch:
                break

            last_seq = self._publish_batch(batch, publish_callback)
            self._store.ack_through(last_seq)
            flushed += len(batch)
        return flushed

    def _publish_batch(self, batch: list[QueueRecord], publish_callback: PublishCallback) -> int:
        last_seq = batch[-1].ingest_seq
        for record in batch:
            publish_callback(
                record.ingest_seq,
                record.topic,
                record.route_key,
                record.message_kind,
                record.payload,
            )
        return last_seq
