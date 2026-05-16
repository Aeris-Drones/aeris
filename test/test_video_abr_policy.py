"""ABR policy/controller tests for adaptive mesh video streaming behavior."""

from __future__ import annotations

from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
MESH_AGENT_SRC = REPO_ROOT / "software" / "edge" / "src" / "aeris_mesh_agent"
STORE_FORWARD_FILE = MESH_AGENT_SRC / "aeris_mesh_agent" / "store_forward_tiles.py"
if str(MESH_AGENT_SRC) not in sys.path:
    sys.path.insert(0, str(MESH_AGENT_SRC))

from aeris_mesh_agent.abr_policy import AbrPolicy, AbrPolicyConfig, AbrTier, LinkHealthSample
from aeris_mesh_agent.video_abr_controller import (
    EncoderCommand,
    EncoderTransport,
    VideoAbrController,
    VideoAbrControllerConfig,
)


class _CaptureTransport(EncoderTransport):
    def __init__(self) -> None:
        self.sent: list[EncoderCommand] = []

    def send_command(self, command: EncoderCommand) -> None:
        self.sent.append(command)


def _ladder_tiers() -> list[AbrTier]:
    return [
        AbrTier(
            name="base_360p",
            resolution="640x360",
            target_bitrate_kbps=900,
            max_bitrate_kbps=1200,
            gop=60,
            fps=30,
        ),
        AbrTier(
            name="perf_540p",
            resolution="960x540",
            target_bitrate_kbps=1800,
            max_bitrate_kbps=2400,
            gop=60,
            fps=30,
        ),
        AbrTier(
            name="detail_720p",
            resolution="1280x720",
            target_bitrate_kbps=3200,
            max_bitrate_kbps=4200,
            gop=60,
            fps=30,
        ),
    ]


def _write_ladder(path: Path) -> Path:
    ladder_path = path / "abr_ladder.yaml"
    ladder_path.write_text(
        "\n".join(
            [
                "version: 1",
                "ladder:",
                "  - name: base_360p",
                "    resolution: 640x360",
                "    target_bitrate_kbps: 900",
                "    max_bitrate_kbps: 1200",
                "    gop: 60",
                "    fps: 30",
                "  - name: perf_540p",
                "    resolution: 960x540",
                "    target_bitrate_kbps: 1800",
                "    max_bitrate_kbps: 2400",
                "    gop: 60",
                "    fps: 30",
                "  - name: detail_720p",
                "    resolution: 1280x720",
                "    target_bitrate_kbps: 3200",
                "    max_bitrate_kbps: 4200",
                "    gop: 60",
                "    fps: 30",
            ]
        ),
        encoding="utf-8",
    )
    return ladder_path


def test_policy_threshold_crossing_hysteresis_and_dwell() -> None:
    now = {"value": 100.2}
    policy = AbrPolicy(
        tiers=_ladder_tiers(),
        config=AbrPolicyConfig(
            pdr_degrade_threshold=0.80,
            pdr_severe_threshold=0.35,
            pdr_hysteresis=0.10,
            min_dwell_sec=2.0,
            min_downgrade_interval_sec=0.10,
            severe_hold_sec=2.0,
        ),
        now_fn=lambda: now["value"],
    )
    now["value"] = 100.4

    # First impairment triggers one-step downgrade.
    first = policy.evaluate(
        LinkHealthSample(timestamp_monotonic=100.4, pdr=0.70, rtt_ms=200.0, heartbeat_age_sec=0.1)
    )
    assert first is not None
    assert first.action == "switch_profile"
    assert first.from_profile == "detail_720p"
    assert first.to_profile == "perf_540p"

    # Dwell prevents immediate rebound.
    now["value"] = 100.6
    rebound_too_early = policy.evaluate(
        LinkHealthSample(timestamp_monotonic=100.6, pdr=0.96, rtt_ms=120.0, heartbeat_age_sec=0.1)
    )
    assert rebound_too_early is None

    # After dwell window, healthy link steps up.
    now["value"] = 103.0
    recovered = policy.evaluate(
        LinkHealthSample(timestamp_monotonic=103.0, pdr=0.97, rtt_ms=110.0, heartbeat_age_sec=0.1)
    )
    assert recovered is not None
    assert recovered.action == "switch_profile"
    assert recovered.from_profile == "perf_540p"
    assert recovered.to_profile == "detail_720p"


def test_policy_enforces_v0_before_optional_suspend() -> None:
    now = {"value": 10.0}
    policy = AbrPolicy(
        tiers=_ladder_tiers(),
        config=AbrPolicyConfig(
            heartbeat_severe_age_sec=0.8,
            severe_hold_sec=1.5,
            allow_video_suspend=True,
            severe_floor_profile="V0",
        ),
        now_fn=lambda: now["value"],
    )

    first = policy.evaluate(
        LinkHealthSample(timestamp_monotonic=10.0, heartbeat_age_sec=1.1, pdr=0.95)
    )
    assert first is not None
    assert first.action == "switch_profile"
    assert first.to_profile == "V0"

    now["value"] = 11.0
    hold = policy.evaluate(
        LinkHealthSample(timestamp_monotonic=11.0, heartbeat_age_sec=1.2, pdr=0.95)
    )
    assert hold is None

    now["value"] = 11.7
    suspended = policy.evaluate(
        LinkHealthSample(timestamp_monotonic=11.7, heartbeat_age_sec=1.3, pdr=0.95)
    )
    assert suspended is not None
    assert suspended.action == "suspend_video"
    assert suspended.from_profile == "V0"


def test_policy_forces_v0_when_reaction_window_exceeded() -> None:
    now = {"value": 5.0}
    policy = AbrPolicy(
        tiers=_ladder_tiers(),
        config=AbrPolicyConfig(
            pdr_degrade_threshold=0.80,
            pdr_severe_threshold=0.20,
            min_downgrade_interval_sec=10.0,
            reaction_window_sec=0.5,
            severe_floor_profile="V0",
            allow_video_suspend=False,
        ),
        now_fn=lambda: now["value"],
    )

    first = policy.evaluate(
        LinkHealthSample(timestamp_monotonic=5.0, pdr=0.70, rtt_ms=220.0, heartbeat_age_sec=0.1)
    )
    assert first is None

    now["value"] = 5.7
    forced = policy.evaluate(
        LinkHealthSample(timestamp_monotonic=5.7, pdr=0.70, rtt_ms=220.0, heartbeat_age_sec=0.1)
    )
    assert forced is not None
    assert forced.action == "switch_profile"
    assert forced.to_profile == "V0"
    assert forced.reason == "reaction-window-force-v0"


def test_controller_retries_timed_out_encoder_commands(tmp_path: Path) -> None:
    ladder_path = _write_ladder(tmp_path)
    now = {"mono": 50.0, "wall": 1000.0}
    transport = _CaptureTransport()
    controller = VideoAbrController(
        ladder_path=str(ladder_path),
        config=VideoAbrControllerConfig(
            ack_timeout_sec=0.5,
            ack_max_retries=2,
            policy=AbrPolicyConfig(
                heartbeat_severe_age_sec=0.7,
                severe_hold_sec=3.0,
                allow_video_suspend=False,
            ),
        ),
        transport=transport,
        now_mono_fn=lambda: now["mono"],
        now_wall_fn=lambda: now["wall"],
    )

    decision = controller.evaluate(
        LinkHealthSample(timestamp_monotonic=50.0, heartbeat_age_sec=1.0, pdr=0.95)
    )
    assert decision is not None
    assert len(transport.sent) == 1
    assert transport.sent[0].profile_name == "V0"
    assert transport.sent[0].retry_count == 0

    now["mono"] = 50.6
    controller.tick()
    assert len(transport.sent) == 2
    assert transport.sent[1].command_id == transport.sent[0].command_id
    assert transport.sent[1].retry_count == 1

    now["mono"] = 51.2
    controller.tick()
    assert len(transport.sent) == 3
    assert transport.sent[2].retry_count == 2

    now["mono"] = 51.8
    controller.tick()
    assert len(transport.sent) == 4
    assert transport.sent[3].retry_count == 3
    assert controller.snapshot().command_failures == 1
    assert controller.snapshot().pending_command_id == transport.sent[0].command_id


def test_controller_ack_prevents_retries(tmp_path: Path) -> None:
    ladder_path = _write_ladder(tmp_path)
    now = {"mono": 10.0, "wall": 20.0}
    transport = _CaptureTransport()
    controller = VideoAbrController(
        ladder_path=str(ladder_path),
        config=VideoAbrControllerConfig(
            ack_timeout_sec=0.3,
            ack_max_retries=2,
            policy=AbrPolicyConfig(heartbeat_severe_age_sec=0.7),
        ),
        transport=transport,
        now_mono_fn=lambda: now["mono"],
        now_wall_fn=lambda: now["wall"],
    )

    decision = controller.evaluate(
        LinkHealthSample(timestamp_monotonic=10.0, heartbeat_age_sec=0.9, pdr=0.95)
    )
    assert decision is not None
    assert len(transport.sent) == 1
    command_id = transport.sent[0].command_id
    assert controller.observe_encoder_ack(
        {
            "command_id": command_id,
            "profile_name": transport.sent[0].profile_name,
            "applied_at_ts": 20.1,
            "status": "ok",
        }
    )

    now["mono"] = 10.6
    controller.tick()
    assert len(transport.sent) == 1
    assert controller.snapshot().pending_command_id is None


def test_controller_rejects_ack_without_command_id(tmp_path: Path) -> None:
    ladder_path = _write_ladder(tmp_path)
    now = {"mono": 5.0, "wall": 6.0}
    transport = _CaptureTransport()
    controller = VideoAbrController(
        ladder_path=str(ladder_path),
        config=VideoAbrControllerConfig(policy=AbrPolicyConfig(heartbeat_severe_age_sec=0.7)),
        transport=transport,
        now_mono_fn=lambda: now["mono"],
        now_wall_fn=lambda: now["wall"],
    )
    decision = controller.evaluate(
        LinkHealthSample(timestamp_monotonic=5.0, heartbeat_age_sec=0.9, pdr=0.95)
    )
    assert decision is not None

    try:
        controller.observe_encoder_ack(
            {
                "profile_name": transport.sent[0].profile_name,
                "applied_at_ts": 6.1,
                "status": "ok",
            }
        )
    except ValueError as exc:
        assert "command_id" in str(exc)
    else:
        raise AssertionError("expected ValueError for ack payload missing command_id")


def test_controller_retries_when_encoder_ack_rejects(tmp_path: Path) -> None:
    ladder_path = _write_ladder(tmp_path)
    now = {"mono": 20.0, "wall": 30.0}
    transport = _CaptureTransport()
    controller = VideoAbrController(
        ladder_path=str(ladder_path),
        config=VideoAbrControllerConfig(
            ack_timeout_sec=0.5,
            ack_max_retries=1,
            policy=AbrPolicyConfig(heartbeat_severe_age_sec=0.7),
        ),
        transport=transport,
        now_mono_fn=lambda: now["mono"],
        now_wall_fn=lambda: now["wall"],
    )

    decision = controller.evaluate(
        LinkHealthSample(timestamp_monotonic=20.0, heartbeat_age_sec=0.9, pdr=0.95)
    )
    assert decision is not None
    assert len(transport.sent) == 1
    command_id = transport.sent[0].command_id

    accepted = controller.observe_encoder_ack(
        {
            "command_id": command_id,
            "profile_name": transport.sent[0].profile_name,
            "applied_at_ts": 30.1,
            "status": "error",
        }
    )
    assert not accepted

    controller.tick()
    assert len(transport.sent) == 2
    assert transport.sent[1].command_id == command_id


def test_controller_reconfigures_pending_ack_deadline(tmp_path: Path) -> None:
    ladder_path = _write_ladder(tmp_path)
    now = {"mono": 100.0, "wall": 200.0}
    transport = _CaptureTransport()
    controller = VideoAbrController(
        ladder_path=str(ladder_path),
        config=VideoAbrControllerConfig(
            ack_timeout_sec=2.0,
            ack_max_retries=1,
            policy=AbrPolicyConfig(heartbeat_severe_age_sec=0.7),
        ),
        transport=transport,
        now_mono_fn=lambda: now["mono"],
        now_wall_fn=lambda: now["wall"],
    )

    decision = controller.evaluate(
        LinkHealthSample(timestamp_monotonic=100.0, heartbeat_age_sec=0.9, pdr=0.95)
    )
    assert decision is not None
    assert len(transport.sent) == 1

    controller.update_ack_timeout(0.2)
    now["mono"] = 100.3
    controller.tick()
    assert len(transport.sent) == 2


def test_store_forward_wires_abr_metrics_decisions_commands_and_acks() -> None:
    text = STORE_FORWARD_FILE.read_text(encoding="utf-8")
    assert "def _active_link_health_sample(self)" in text
    assert "self._abr_controller.evaluate(sample)" in text
    assert "self._abr_decision_publisher.publish(" in text
    assert "self._abr_ack_subscription = self.create_subscription(" in text
    assert "encoder rejected ABR command" in text
    assert "self._abr_controller.update_ack_timeout(value)" in text


def test_store_forward_prefers_existing_map_tile_provenance_before_fallback() -> None:
    text = STORE_FORWARD_FILE.read_text(encoding="utf-8")
    assert 'if message_kind == "map_tile":' in text
    assert 'getattr(message, "source_vehicle_id", "")' in text
    assert "if not source_vehicle_id:" in text
    assert "source_vehicle_id = self._source_vehicle_id_from_message(" in text


def test_ladder_parser_fails_on_unsorted_target_bitrate(tmp_path: Path) -> None:
    ladder_path = tmp_path / "abr_bad.yaml"
    ladder_path.write_text(
        "\n".join(
            [
                "version: 1",
                "ladder:",
                "  - name: high",
                "    resolution: 1280x720",
                "    target_bitrate_kbps: 2200",
                "    max_bitrate_kbps: 2600",
                "    gop: 60",
                "    fps: 30",
                "  - name: low",
                "    resolution: 640x360",
                "    target_bitrate_kbps: 900",
                "    max_bitrate_kbps: 1200",
                "    gop: 60",
                "    fps: 30",
            ]
        ),
        encoding="utf-8",
    )

    try:
        VideoAbrController.load_ladder_tiers(str(ladder_path))
    except ValueError as exc:
        assert "strictly increasing" in str(exc)
    else:
        raise AssertionError("expected ValueError for unsorted ladder bitrate values")
