"""Relay routing primitives for Scout->Ranger->GCS path selection."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import time
from typing import Callable

_VALID_POLICIES = {"auto", "force-direct", "force-relay"}


@dataclass(frozen=True)
class RelayEnvelope:
    """Metadata carried with replay annotations for relay-aware delivery."""

    source_vehicle_id: str
    relay_vehicle_id: str
    relay_hop: int
    delivery_mode: str


@dataclass(frozen=True)
class RelayRouteDecision:
    """Selected transport route for the next outbound message."""

    delivery_mode: str
    selected_link_up: bool
    reason: str


@dataclass
class _LinkHealth:
    """Heartbeat + optional PDR based link health evaluator."""

    heartbeat_timeout_sec: float
    pdr_threshold: float
    now_fn: Callable[[], float]
    _last_heartbeat_monotonic: float | None = None
    _latest_pdr: float | None = None

    def observe_heartbeat(self, timestamp_monotonic: float | None = None) -> None:
        self._last_heartbeat_monotonic = (
            self.now_fn() if timestamp_monotonic is None else float(timestamp_monotonic)
        )

    def observe_pdr(self, pdr: float | None) -> None:
        self._latest_pdr = None if pdr is None else float(pdr)

    def is_up(self) -> bool:
        if self._last_heartbeat_monotonic is None:
            return False
        heartbeat_age = self.now_fn() - self._last_heartbeat_monotonic
        if heartbeat_age > self.heartbeat_timeout_sec:
            return False
        if self.pdr_threshold > 0.0 and self._latest_pdr is not None:
            return self._latest_pdr >= self.pdr_threshold
        return True


class RelayRouteSelector:
    """Direct-vs-relay route chooser with policy and health gating."""

    def __init__(
        self,
        *,
        relay_enabled: bool,
        activation_policy: str,
        direct_heartbeat_timeout_sec: float,
        direct_pdr_threshold: float,
        relay_heartbeat_timeout_sec: float,
        relay_pdr_threshold: float,
        now_fn: Callable[[], float] = time.monotonic,
    ) -> None:
        policy = activation_policy.strip().lower()
        if policy not in _VALID_POLICIES:
            policy = "auto"
        self._relay_enabled = bool(relay_enabled)
        self._activation_policy = policy
        self._direct = _LinkHealth(
            heartbeat_timeout_sec=float(direct_heartbeat_timeout_sec),
            pdr_threshold=float(direct_pdr_threshold),
            now_fn=now_fn,
        )
        self._relay = _LinkHealth(
            heartbeat_timeout_sec=float(relay_heartbeat_timeout_sec),
            pdr_threshold=float(relay_pdr_threshold),
            now_fn=now_fn,
        )

    @property
    def relay_enabled(self) -> bool:
        return self._relay_enabled

    @relay_enabled.setter
    def relay_enabled(self, value: bool) -> None:
        self._relay_enabled = bool(value)

    @property
    def activation_policy(self) -> str:
        return self._activation_policy

    @activation_policy.setter
    def activation_policy(self, value: str) -> None:
        policy = value.strip().lower()
        self._activation_policy = policy if policy in _VALID_POLICIES else "auto"

    def set_direct_thresholds(self, *, heartbeat_timeout_sec: float, pdr_threshold: float) -> None:
        self._direct.heartbeat_timeout_sec = float(heartbeat_timeout_sec)
        self._direct.pdr_threshold = float(pdr_threshold)

    def set_relay_thresholds(self, *, heartbeat_timeout_sec: float, pdr_threshold: float) -> None:
        self._relay.heartbeat_timeout_sec = float(heartbeat_timeout_sec)
        self._relay.pdr_threshold = float(pdr_threshold)

    def observe_direct_heartbeat(self) -> None:
        self._direct.observe_heartbeat()

    def observe_relay_heartbeat(self) -> None:
        self._relay.observe_heartbeat()

    def observe_direct_pdr(self, pdr: float) -> None:
        self._direct.observe_pdr(pdr)

    def observe_relay_pdr(self, pdr: float) -> None:
        self._relay.observe_pdr(pdr)

    def select_route(self) -> RelayRouteDecision:
        direct_up = self._direct.is_up()
        relay_up = self._relay.is_up()

        if not self._relay_enabled:
            return RelayRouteDecision(
                delivery_mode="direct",
                selected_link_up=direct_up,
                reason="relay-disabled",
            )

        if self._activation_policy == "force-direct":
            return RelayRouteDecision(
                delivery_mode="direct",
                selected_link_up=direct_up,
                reason="force-direct",
            )

        if self._activation_policy == "force-relay":
            if relay_up:
                return RelayRouteDecision(
                    delivery_mode="relay",
                    selected_link_up=True,
                    reason="force-relay",
                )
            return RelayRouteDecision(
                delivery_mode="direct",
                selected_link_up=direct_up,
                reason="force-relay-fallback-direct",
            )

        if direct_up:
            return RelayRouteDecision(
                delivery_mode="direct",
                selected_link_up=True,
                reason="direct-healthy",
            )
        if relay_up:
            return RelayRouteDecision(
                delivery_mode="relay",
                selected_link_up=True,
                reason="direct-unhealthy-relay-healthy",
            )
        return RelayRouteDecision(
            delivery_mode="direct",
            selected_link_up=False,
            reason="all-paths-unhealthy",
        )


@dataclass(frozen=True)
class RelayLatencySnapshot:
    """Windowed relay observability values."""

    relay_forwarded_total: int
    relay_latency_p50_sec: float
    relay_latency_p95_sec: float
    replay_lag_p50_sec: float
    replay_lag_p95_sec: float


class RelayLatencyTracker:
    """Tracks relay latency and replay lag percentiles for logging."""

    def __init__(self, window_size: int = 256) -> None:
        self._relay_forwarded_total = 0
        self._relay_latencies: deque[float] = deque(maxlen=max(8, int(window_size)))
        self._replay_lags: deque[float] = deque(maxlen=max(8, int(window_size)))

    def record(
        self,
        *,
        envelope: RelayEnvelope,
        original_event_ts: float,
        replayed_at_ts: float | None,
        published_at_ts: float,
    ) -> None:
        if envelope.delivery_mode != "relay":
            return

        self._relay_forwarded_total += 1
        self._relay_latencies.append(max(float(published_at_ts) - float(original_event_ts), 0.0))
        if replayed_at_ts is not None:
            self._replay_lags.append(max(float(replayed_at_ts) - float(original_event_ts), 0.0))

    def snapshot(self) -> RelayLatencySnapshot:
        return RelayLatencySnapshot(
            relay_forwarded_total=self._relay_forwarded_total,
            relay_latency_p50_sec=self._percentile(self._relay_latencies, 50),
            relay_latency_p95_sec=self._percentile(self._relay_latencies, 95),
            replay_lag_p50_sec=self._percentile(self._replay_lags, 50),
            replay_lag_p95_sec=self._percentile(self._replay_lags, 95),
        )

    @staticmethod
    def _percentile(values: deque[float], percentile: int) -> float:
        if not values:
            return 0.0
        rank = max(0.0, min(100.0, float(percentile))) / 100.0
        ordered = sorted(float(v) for v in values)
        idx = int(round((len(ordered) - 1) * rank))
        return ordered[idx]
