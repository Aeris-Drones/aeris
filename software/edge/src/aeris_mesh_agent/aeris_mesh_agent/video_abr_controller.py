"""Runtime ABR controller that parses ladder config and drives encoder commands."""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import re
import time
from typing import Callable, Protocol

import yaml

from .abr_policy import (
    AbrDecision,
    AbrPolicy,
    AbrPolicyConfig,
    AbrTier,
    LinkHealthSample,
)

_RESOLUTION_RE = re.compile(r"^\s*(\d+)x(\d+)\s*$")


@dataclass(frozen=True)
class EncoderProfile:
    """Encoder profile payload resolved from ABR ladder tier."""

    name: str
    width: int
    height: int
    fps: int
    gop: int
    target_bitrate_kbps: int
    max_bitrate_kbps: int

    @property
    def resolution(self) -> str:
        return f"{self.width}x{self.height}"


@dataclass
class VideoAbrControllerConfig:
    """Controller-level settings including retries + V0 profile behavior."""

    ack_timeout_sec: float = 0.5
    ack_max_retries: int = 2
    reaction_window_sec: float = 1.0
    v0_profile_name: str = "V0"
    v0_resolution: str = "320x180"
    v0_target_bitrate_kbps: int = 220
    v0_max_bitrate_kbps: int = 320
    v0_fps: int = 10
    v0_gop: int = 30
    policy: AbrPolicyConfig = field(default_factory=AbrPolicyConfig)


@dataclass(frozen=True)
class EncoderCommand:
    """Command sent from ABR to encoder/video subsystem."""

    command_id: int
    action: str
    profile_name: str
    request_ts: float
    reason: str
    width: int | None = None
    height: int | None = None
    fps: int | None = None
    gop: int | None = None
    target_bitrate_kbps: int | None = None
    max_bitrate_kbps: int | None = None
    retry_count: int = 0

    def to_dict(self) -> dict[str, object]:
        return {
            "command_id": self.command_id,
            "action": self.action,
            "profile_name": self.profile_name,
            "request_ts": self.request_ts,
            "reason": self.reason,
            "resolution": (
                None
                if self.width is None or self.height is None
                else f"{self.width}x{self.height}"
            ),
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "gop": self.gop,
            "target_bitrate_kbps": self.target_bitrate_kbps,
            "max_bitrate_kbps": self.max_bitrate_kbps,
            "retry_count": self.retry_count,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), separators=(",", ":"))


@dataclass(frozen=True)
class EncoderAck:
    """Encoder profile application acknowledgement."""

    command_id: int
    profile_name: str
    applied_at_ts: float
    status: str = "ok"

    @staticmethod
    def from_payload(payload: str | dict[str, object]) -> "EncoderAck":
        raw = json.loads(payload) if isinstance(payload, str) else payload
        if not isinstance(raw, dict):
            raise ValueError("encoder ack payload must be a JSON object")
        command_id_raw = raw.get("command_id")
        if command_id_raw is None:
            raise ValueError("encoder ack is missing command_id")
        try:
            command_id = int(command_id_raw)
        except (TypeError, ValueError) as exc:
            raise ValueError("encoder ack command_id is invalid") from exc
        profile_name = str(raw.get("profile_name", "")).strip()
        applied_raw = raw.get("applied_at_ts")
        if applied_raw is None:
            applied_at_ts = time.time()
        else:
            try:
                applied_at_ts = float(applied_raw)
            except (TypeError, ValueError) as exc:
                raise ValueError("encoder ack applied_at_ts is invalid") from exc
        status = str(raw.get("status", "ok")).strip().lower() or "ok"
        if not profile_name:
            raise ValueError("encoder ack is missing profile_name")
        return EncoderAck(
            command_id=command_id,
            profile_name=profile_name,
            applied_at_ts=applied_at_ts,
            status=status,
        )


@dataclass(frozen=True)
class VideoAbrSnapshot:
    """Combined ABR policy/controller snapshot."""

    current_profile: str
    video_suspended: bool
    pending_command_id: int | None
    pending_retry_count: int
    transition_count: int
    downgrade_count: int
    upgrade_count: int
    severe_floor_entries: int
    suspend_count: int
    command_failures: int
    decision_latency_p50_ms: float
    decision_latency_p95_ms: float
    reaction_latency_p50_ms: float
    reaction_latency_p95_ms: float


class EncoderTransport(Protocol):
    """Encoder command transport contract."""

    def send_command(self, command: EncoderCommand) -> None:
        pass


@dataclass
class _PendingCommand:
    command: EncoderCommand
    retries: int
    deadline_monotonic: float


class VideoAbrController:
    """Orchestrates ABR policy decisions and encoder command reliability."""

    def __init__(
        self,
        *,
        ladder_path: str,
        config: VideoAbrControllerConfig,
        transport: EncoderTransport,
        now_mono_fn: Callable[[], float] = time.monotonic,
        now_wall_fn: Callable[[], float] = time.time,
    ) -> None:
        self._config = config
        self._now_mono_fn = now_mono_fn
        self._now_wall_fn = now_wall_fn
        self._transport = transport

        self._tier_by_name, tiers = self.load_ladder_tiers(ladder_path)
        if self._config.policy.severe_floor_profile != self._config.v0_profile_name:
            self._config.policy.severe_floor_profile = self._config.v0_profile_name
        self._config.policy.reaction_window_sec = self._config.reaction_window_sec

        self._policy = AbrPolicy(tiers=tiers, config=self._config.policy, now_fn=self._now_mono_fn)
        self._pending_command: _PendingCommand | None = None
        self._next_command_id = 1
        self._command_failures = 0
        self._decision_latencies_ms: list[float] = []
        self._reaction_latencies_ms: list[float] = []
        self._last_ack_command_id: int | None = None
        self._last_ack_status: str | None = None
        self._last_ack_matched_pending = False

    @property
    def config(self) -> VideoAbrControllerConfig:
        return self._config

    @staticmethod
    def load_ladder_tiers(path: str) -> tuple[dict[str, EncoderProfile], list[AbrTier]]:
        with open(path, "r", encoding="utf-8") as fp:
            raw = yaml.safe_load(fp)

        if not isinstance(raw, dict):
            raise ValueError("ABR ladder config must be a mapping")
        ladder = raw.get("ladder")
        if not isinstance(ladder, list) or not ladder:
            raise ValueError("ABR ladder config requires a non-empty 'ladder' list")

        tier_by_name: dict[str, EncoderProfile] = {}
        abr_tiers: list[AbrTier] = []
        previous_target = -1
        for idx, item in enumerate(ladder):
            if not isinstance(item, dict):
                raise ValueError(f"invalid ladder tier at index {idx}: expected map")
            name = str(item.get("name", "")).strip()
            resolution = str(item.get("resolution", "")).strip()
            match = _RESOLUTION_RE.match(resolution)
            if not name or not match:
                raise ValueError(f"invalid ladder tier '{name or idx}': missing name/resolution")
            width = int(match.group(1))
            height = int(match.group(2))
            target = int(item.get("target_bitrate_kbps", 0))
            max_bitrate = int(item.get("max_bitrate_kbps", 0))
            gop = int(item.get("gop", 0))
            fps = int(item.get("fps", 0))
            if target <= 0 or max_bitrate <= 0 or gop <= 0 or fps <= 0:
                raise ValueError(f"invalid ladder tier '{name}': non-positive numeric value")
            if target > max_bitrate:
                raise ValueError(f"invalid ladder tier '{name}': target bitrate > max bitrate")
            if target <= previous_target:
                raise ValueError(
                    "ABR ladder target_bitrate_kbps values must be strictly increasing"
                )
            previous_target = target
            if name in tier_by_name:
                raise ValueError(f"duplicate ABR tier name: {name}")

            profile = EncoderProfile(
                name=name,
                width=width,
                height=height,
                fps=fps,
                gop=gop,
                target_bitrate_kbps=target,
                max_bitrate_kbps=max_bitrate,
            )
            tier_by_name[name] = profile
            abr_tiers.append(
                AbrTier(
                    name=name,
                    resolution=profile.resolution,
                    target_bitrate_kbps=target,
                    max_bitrate_kbps=max_bitrate,
                    gop=gop,
                    fps=fps,
                )
            )
        return tier_by_name, abr_tiers

    def reconfigure_policy(self, config: AbrPolicyConfig) -> None:
        if config.severe_floor_profile != self._config.v0_profile_name:
            config.severe_floor_profile = self._config.v0_profile_name
        self._config.reaction_window_sec = float(config.reaction_window_sec)
        self._config.policy = config
        self._policy.reconfigure(config)

    def update_ack_timeout(self, timeout_sec: float) -> None:
        self._config.ack_timeout_sec = timeout_sec
        if self._pending_command is None:
            return
        self._pending_command = _PendingCommand(
            command=self._pending_command.command,
            retries=self._pending_command.retries,
            deadline_monotonic=self._next_ack_deadline(),
        )

    def last_ack_details(self) -> tuple[int | None, str | None, bool]:
        return (
            self._last_ack_command_id,
            self._last_ack_status,
            self._last_ack_matched_pending,
        )

    def reconfigure_v0_profile(
        self,
        *,
        profile_name: str,
        resolution: str,
        target_bitrate_kbps: int,
        max_bitrate_kbps: int,
        fps: int,
        gop: int,
    ) -> None:
        match = _RESOLUTION_RE.match(str(resolution).strip())
        if match is None:
            raise ValueError("invalid V0 resolution; expected WIDTHxHEIGHT")
        if target_bitrate_kbps <= 0 or max_bitrate_kbps <= 0:
            raise ValueError("V0 bitrate values must be positive")
        if target_bitrate_kbps > max_bitrate_kbps:
            raise ValueError("V0 target bitrate must be <= max bitrate")
        if fps <= 0 or gop <= 0:
            raise ValueError("V0 fps/gop must be positive")

        self._config.v0_profile_name = profile_name
        self._config.v0_resolution = f"{int(match.group(1))}x{int(match.group(2))}"
        self._config.v0_target_bitrate_kbps = int(target_bitrate_kbps)
        self._config.v0_max_bitrate_kbps = int(max_bitrate_kbps)
        self._config.v0_fps = int(fps)
        self._config.v0_gop = int(gop)
        self._config.policy.severe_floor_profile = profile_name
        self._policy.reconfigure(self._config.policy)

    def observe_encoder_ack(self, payload: str | dict[str, object]) -> bool:
        ack = EncoderAck.from_payload(payload)
        self._last_ack_command_id = ack.command_id
        self._last_ack_status = ack.status
        pending = self._pending_command
        if pending is None:
            self._last_ack_matched_pending = False
            return False
        if ack.command_id != pending.command.command_id:
            self._last_ack_matched_pending = False
            return False
        self._last_ack_matched_pending = True
        if ack.status == "ok":
            self._pending_command = None
            return True
        # Rejected acks are actionable failures: keep the command pending and
        # trigger immediate retry processing on the next tick/evaluate cycle.
        self._pending_command = _PendingCommand(
            command=pending.command,
            retries=pending.retries,
            deadline_monotonic=self._now_mono_fn(),
        )
        return False

    def tick(self) -> None:
        self._process_pending_timeout()

    def evaluate(self, sample: LinkHealthSample) -> dict[str, object] | None:
        self._process_pending_timeout()
        decision = self._policy.evaluate(sample)
        if decision is None:
            return None

        self._record_latencies(decision)
        command = self._build_command(decision)
        self._send_command(command)
        payload = decision.to_dict()
        payload["command"] = command.to_dict()
        return payload

    def snapshot(self) -> VideoAbrSnapshot:
        policy_snapshot = self._policy.snapshot()
        pending_id = None
        pending_retry_count = 0
        if self._pending_command is not None:
            pending_id = self._pending_command.command.command_id
            pending_retry_count = self._pending_command.retries

        return VideoAbrSnapshot(
            current_profile=policy_snapshot.current_profile,
            video_suspended=policy_snapshot.video_suspended,
            pending_command_id=pending_id,
            pending_retry_count=pending_retry_count,
            transition_count=policy_snapshot.transition_count,
            downgrade_count=policy_snapshot.downgrade_count,
            upgrade_count=policy_snapshot.upgrade_count,
            severe_floor_entries=policy_snapshot.severe_floor_entries,
            suspend_count=policy_snapshot.suspend_count,
            command_failures=self._command_failures,
            decision_latency_p50_ms=self._percentile(self._decision_latencies_ms, 50),
            decision_latency_p95_ms=self._percentile(self._decision_latencies_ms, 95),
            reaction_latency_p50_ms=self._percentile(self._reaction_latencies_ms, 50),
            reaction_latency_p95_ms=self._percentile(self._reaction_latencies_ms, 95),
        )

    def _record_latencies(self, decision: AbrDecision) -> None:
        if decision.decision_latency_ms >= 0.0:
            self._decision_latencies_ms.append(decision.decision_latency_ms)
            if len(self._decision_latencies_ms) > 512:
                self._decision_latencies_ms = self._decision_latencies_ms[-512:]
        if decision.reaction_latency_ms is not None and decision.reaction_latency_ms >= 0.0:
            self._reaction_latencies_ms.append(decision.reaction_latency_ms)
            if len(self._reaction_latencies_ms) > 512:
                self._reaction_latencies_ms = self._reaction_latencies_ms[-512:]

    def _build_command(self, decision: AbrDecision) -> EncoderCommand:
        action = str(decision.action)
        to_profile = str(decision.to_profile)
        profile = self._resolve_profile(to_profile)

        command = EncoderCommand(
            command_id=self._next_command_id,
            action=action,
            profile_name=to_profile,
            request_ts=self._now_wall_fn(),
            reason=decision.reason,
            width=profile.width if profile else None,
            height=profile.height if profile else None,
            fps=profile.fps if profile else None,
            gop=profile.gop if profile else None,
            target_bitrate_kbps=profile.target_bitrate_kbps if profile else None,
            max_bitrate_kbps=profile.max_bitrate_kbps if profile else None,
            retry_count=0,
        )
        self._next_command_id += 1
        return command

    def _resolve_profile(self, name: str) -> EncoderProfile | None:
        if name in self._tier_by_name:
            return self._tier_by_name[name]
        if name == self._config.v0_profile_name:
            match = _RESOLUTION_RE.match(self._config.v0_resolution)
            if match is None:
                raise ValueError("invalid v0_resolution in controller config")
            return EncoderProfile(
                name=name,
                width=int(match.group(1)),
                height=int(match.group(2)),
                fps=int(self._config.v0_fps),
                gop=int(self._config.v0_gop),
                target_bitrate_kbps=int(self._config.v0_target_bitrate_kbps),
                max_bitrate_kbps=int(self._config.v0_max_bitrate_kbps),
            )
        return None

    def _send_command(self, command: EncoderCommand) -> None:
        self._transport.send_command(command)
        self._pending_command = _PendingCommand(
            command=command,
            retries=0,
            deadline_monotonic=self._next_ack_deadline(),
        )

    def _process_pending_timeout(self) -> None:
        pending = self._pending_command
        if pending is None:
            return
        if self._now_mono_fn() < pending.deadline_monotonic:
            return

        if pending.retries >= max(0, int(self._config.ack_max_retries)):
            self._command_failures += 1
            # Do not abandon adaptation after retry exhaustion; keep retrying in
            # bounded cycles so severe-floor commands eventually apply.
            retried = EncoderCommand(
                command_id=pending.command.command_id,
                action=pending.command.action,
                profile_name=pending.command.profile_name,
                request_ts=pending.command.request_ts,
                reason=pending.command.reason,
                width=pending.command.width,
                height=pending.command.height,
                fps=pending.command.fps,
                gop=pending.command.gop,
                target_bitrate_kbps=pending.command.target_bitrate_kbps,
                max_bitrate_kbps=pending.command.max_bitrate_kbps,
                retry_count=pending.command.retry_count + 1,
            )
            self._transport.send_command(retried)
            self._pending_command = _PendingCommand(
                command=retried,
                retries=0,
                deadline_monotonic=self._next_ack_deadline(),
            )
            return

        retried = EncoderCommand(
            command_id=pending.command.command_id,
            action=pending.command.action,
            profile_name=pending.command.profile_name,
            request_ts=pending.command.request_ts,
            reason=pending.command.reason,
            width=pending.command.width,
            height=pending.command.height,
            fps=pending.command.fps,
            gop=pending.command.gop,
            target_bitrate_kbps=pending.command.target_bitrate_kbps,
            max_bitrate_kbps=pending.command.max_bitrate_kbps,
            retry_count=pending.retries + 1,
        )
        self._transport.send_command(retried)
        self._pending_command = _PendingCommand(
            command=retried,
            retries=pending.retries + 1,
            deadline_monotonic=self._next_ack_deadline(),
        )

    def _next_ack_deadline(self) -> float:
        return self._now_mono_fn() + max(0.05, self._config.ack_timeout_sec)

    @staticmethod
    def _percentile(values: list[float], percentile: int) -> float:
        if not values:
            return 0.0
        if len(values) == 1:
            return float(values[0])
        clipped = max(0.0, min(100.0, float(percentile))) / 100.0
        # Deterministic quantile estimate for short windows.
        sorted_values = sorted(float(v) for v in values)
        index = round((len(sorted_values) - 1) * clipped)
        return float(sorted_values[index])
