"""Deterministic ABR policy primitives for mesh-agent video adaptation."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import time
from typing import Callable


@dataclass(frozen=True)
class AbrTier:
    """One ladder level that can be applied by the encoder pipeline."""

    name: str
    resolution: str
    target_bitrate_kbps: int
    max_bitrate_kbps: int
    gop: int
    fps: int


@dataclass(frozen=True)
class LinkHealthSample:
    """Point-in-time network health sample consumed by the policy."""

    timestamp_monotonic: float
    pdr: float | None = None
    rtt_ms: float | None = None
    packet_loss: float | None = None
    throughput_kbps: float | None = None
    heartbeat_age_sec: float | None = None


@dataclass
class AbrPolicyConfig:
    """Thresholds and guards for ABR decisions."""

    pdr_degrade_threshold: float = 0.75
    pdr_severe_threshold: float = 0.35
    pdr_hysteresis: float = 0.08
    rtt_degrade_threshold_ms: float = 250.0
    rtt_severe_threshold_ms: float = 600.0
    rtt_hysteresis_ms: float = 40.0
    packet_loss_degrade_threshold: float = 0.10
    packet_loss_severe_threshold: float = 0.35
    heartbeat_severe_age_sec: float = 0.90
    min_dwell_sec: float = 2.0
    min_downgrade_interval_sec: float = 0.25
    severe_hold_sec: float = 2.0
    reaction_window_sec: float = 1.0
    allow_video_suspend: bool = True
    severe_floor_profile: str = "V0"
    downgrade_step: int = 1
    upgrade_step: int = 1


@dataclass(frozen=True)
class AbrDecision:
    """A policy transition recommendation."""

    timestamp_monotonic: float
    action: str
    from_profile: str
    to_profile: str
    reason: str
    degraded: bool
    severe: bool
    decision_latency_ms: float
    reaction_latency_ms: float | None
    sample: LinkHealthSample

    def to_dict(self) -> dict[str, object]:
        payload = asdict(self)
        # Flatten nested sample for easier telemetry pipelines.
        sample = payload.pop("sample")
        payload["sample"] = sample
        return payload


@dataclass(frozen=True)
class AbrPolicySnapshot:
    """Current policy state + counters for observability."""

    current_profile: str
    video_suspended: bool
    transition_count: int
    downgrade_count: int
    upgrade_count: int
    severe_floor_entries: int
    suspend_count: int


class AbrPolicy:
    """Deterministic ABR finite state machine."""

    def __init__(
        self,
        *,
        tiers: list[AbrTier],
        config: AbrPolicyConfig,
        now_fn: Callable[[], float] = time.monotonic,
    ) -> None:
        if not tiers:
            raise ValueError("ABR policy requires at least one ladder tier")
        self._tiers = list(tiers)
        self._tier_index_by_name = {tier.name: idx for idx, tier in enumerate(self._tiers)}
        self._config = config
        self._now_fn = now_fn
        self._current_index = len(self._tiers) - 1
        self._current_profile = self._tiers[self._current_index].name
        self._video_suspended = False
        now = float(self._now_fn())
        self._last_transition_monotonic = now
        self._severe_started_monotonic: float | None = None
        self._impairment_started_monotonic: float | None = None
        self._impairment_reaction_recorded = True
        self._transition_count = 0
        self._downgrade_count = 0
        self._upgrade_count = 0
        self._severe_floor_entries = 0
        self._suspend_count = 0

    @property
    def config(self) -> AbrPolicyConfig:
        return self._config

    def reconfigure(self, config: AbrPolicyConfig) -> None:
        self._config = config

    def snapshot(self) -> AbrPolicySnapshot:
        return AbrPolicySnapshot(
            current_profile=self._current_profile,
            video_suspended=self._video_suspended,
            transition_count=self._transition_count,
            downgrade_count=self._downgrade_count,
            upgrade_count=self._upgrade_count,
            severe_floor_entries=self._severe_floor_entries,
            suspend_count=self._suspend_count,
        )

    def evaluate(self, sample: LinkHealthSample) -> AbrDecision | None:
        now = float(sample.timestamp_monotonic)
        severe_threshold = self._is_severe(sample)
        severe = severe_threshold
        degraded = severe or self._is_degraded(sample)
        self._track_impairment_window(now=now, degraded=degraded)
        reaction_window_escalation = self._reaction_window_exceeded(
            now=now,
            degraded=degraded,
            severe=severe_threshold,
        )
        if reaction_window_escalation:
            severe = True
            degraded = True
        healthy_for_upgrade = self._is_upgrade_healthy(sample)

        if self._video_suspended:
            if severe:
                return None
            return self._transition(
                now=now,
                sample=sample,
                action="resume_video",
                to_profile=self._config.severe_floor_profile,
                reason="link-recovered-resume-v0",
                degraded=degraded,
                severe=severe,
            )

        if severe:
            if self._severe_started_monotonic is None:
                self._severe_started_monotonic = now
            if self._current_profile != self._config.severe_floor_profile:
                return self._transition(
                    now=now,
                    sample=sample,
                    action="switch_profile",
                    to_profile=self._config.severe_floor_profile,
                    reason=(
                        "reaction-window-force-v0"
                        if reaction_window_escalation
                        else "severe-impairment-force-v0"
                    ),
                    degraded=degraded,
                    severe=severe,
                )
            if self._config.allow_video_suspend and not reaction_window_escalation:
                severe_elapsed = now - self._severe_started_monotonic
                if severe_elapsed >= self._config.severe_hold_sec:
                    return self._transition(
                        now=now,
                        sample=sample,
                        action="suspend_video",
                        to_profile=self._config.severe_floor_profile,
                        reason="severe-impairment-suspend-after-v0-hold",
                        degraded=degraded,
                        severe=severe,
                    )
            return None
        self._severe_started_monotonic = None

        if self._current_profile == self._config.severe_floor_profile and (
            self._current_profile not in self._tier_index_by_name
        ):
            if healthy_for_upgrade and self._dwell_elapsed(now=now):
                return self._transition(
                    now=now,
                    sample=sample,
                    action="switch_profile",
                    to_profile=self._tiers[0].name,
                    reason="recovered-v0-to-ladder-floor",
                    degraded=degraded,
                    severe=severe,
                )
            return None

        current_index = self._tier_index_by_name.get(self._current_profile, self._current_index)
        self._current_index = current_index

        if degraded:
            if not self._downgrade_interval_elapsed(now=now):
                return None
            target_index = max(0, current_index - max(1, int(self._config.downgrade_step)))
            if target_index != current_index:
                return self._transition(
                    now=now,
                    sample=sample,
                    action="switch_profile",
                    to_profile=self._tiers[target_index].name,
                    reason="link-degraded-step-down",
                    degraded=degraded,
                    severe=severe,
                )
            return None

        if healthy_for_upgrade and self._dwell_elapsed(now=now):
            target_index = min(
                len(self._tiers) - 1, current_index + max(1, int(self._config.upgrade_step))
            )
            if target_index != current_index:
                return self._transition(
                    now=now,
                    sample=sample,
                    action="switch_profile",
                    to_profile=self._tiers[target_index].name,
                    reason="link-stable-step-up",
                    degraded=degraded,
                    severe=severe,
                )

        return None

    def _track_impairment_window(self, *, now: float, degraded: bool) -> None:
        if degraded:
            if self._impairment_started_monotonic is None:
                self._impairment_started_monotonic = now
                self._impairment_reaction_recorded = False
            return
        self._impairment_started_monotonic = None
        self._impairment_reaction_recorded = True

    def _reaction_window_exceeded(self, *, now: float, degraded: bool, severe: bool) -> bool:
        if severe or not degraded:
            return False
        if self._impairment_started_monotonic is None:
            return False
        reaction_window_sec = max(0.0, float(self._config.reaction_window_sec))
        if reaction_window_sec <= 0.0:
            return False
        return (now - self._impairment_started_monotonic) >= reaction_window_sec

    def _is_degraded(self, sample: LinkHealthSample) -> bool:
        cfg = self._config
        if sample.pdr is not None and sample.pdr < cfg.pdr_degrade_threshold:
            return True
        if sample.rtt_ms is not None and sample.rtt_ms > cfg.rtt_degrade_threshold_ms:
            return True
        if sample.packet_loss is not None and sample.packet_loss > cfg.packet_loss_degrade_threshold:
            return True
        return False

    def _is_upgrade_healthy(self, sample: LinkHealthSample) -> bool:
        cfg = self._config
        if sample.pdr is not None and sample.pdr < (cfg.pdr_degrade_threshold + cfg.pdr_hysteresis):
            return False
        if sample.rtt_ms is not None and sample.rtt_ms > (
            cfg.rtt_degrade_threshold_ms - cfg.rtt_hysteresis_ms
        ):
            return False
        if (
            sample.packet_loss is not None
            and sample.packet_loss > max(cfg.packet_loss_degrade_threshold * 0.5, 0.0)
        ):
            return False
        if sample.heartbeat_age_sec is not None and sample.heartbeat_age_sec > cfg.heartbeat_severe_age_sec:
            return False
        return True

    def _is_severe(self, sample: LinkHealthSample) -> bool:
        cfg = self._config
        if sample.heartbeat_age_sec is not None and sample.heartbeat_age_sec >= cfg.heartbeat_severe_age_sec:
            return True
        if sample.pdr is not None and sample.pdr <= cfg.pdr_severe_threshold:
            return True
        if sample.rtt_ms is not None and sample.rtt_ms >= cfg.rtt_severe_threshold_ms:
            return True
        if sample.packet_loss is not None and sample.packet_loss >= cfg.packet_loss_severe_threshold:
            return True
        return False

    def _dwell_elapsed(self, *, now: float) -> bool:
        return (now - self._last_transition_monotonic) >= self._config.min_dwell_sec

    def _downgrade_interval_elapsed(self, *, now: float) -> bool:
        return (now - self._last_transition_monotonic) >= self._config.min_downgrade_interval_sec

    def _transition(
        self,
        *,
        now: float,
        sample: LinkHealthSample,
        action: str,
        to_profile: str,
        reason: str,
        degraded: bool,
        severe: bool,
    ) -> AbrDecision:
        from_profile = self._current_profile
        if action == "suspend_video":
            self._video_suspended = True
            self._suspend_count += 1
        elif action == "resume_video":
            self._video_suspended = False
            self._current_profile = to_profile
            if to_profile in self._tier_index_by_name:
                self._current_index = self._tier_index_by_name[to_profile]
        else:
            self._current_profile = to_profile
            self._video_suspended = False
            if to_profile in self._tier_index_by_name:
                target_index = self._tier_index_by_name[to_profile]
                if target_index < self._current_index:
                    self._downgrade_count += 1
                elif target_index > self._current_index:
                    self._upgrade_count += 1
                self._current_index = target_index
            elif to_profile == self._config.severe_floor_profile:
                self._severe_floor_entries += 1

        self._transition_count += 1
        self._last_transition_monotonic = now

        reaction_latency_ms: float | None = None
        if (
            self._impairment_started_monotonic is not None
            and not self._impairment_reaction_recorded
            and action in {"switch_profile", "resume_video", "suspend_video"}
        ):
            reaction_latency_ms = max((now - self._impairment_started_monotonic) * 1000.0, 0.0)
            self._impairment_reaction_recorded = True

        decision_latency_ms = max((self._now_fn() - sample.timestamp_monotonic) * 1000.0, 0.0)
        return AbrDecision(
            timestamp_monotonic=now,
            action=action,
            from_profile=from_profile,
            to_profile=to_profile,
            reason=reason,
            degraded=degraded,
            severe=severe,
            decision_latency_ms=decision_latency_ms,
            reaction_latency_ms=reaction_latency_ms,
            sample=sample,
        )
