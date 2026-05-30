from __future__ import annotations

from types import SimpleNamespace

import pytest

from aeris_perception.halo_confidence_event import (
    halo_confidence_event_from_candidate,
    parse_halo_confidence_event,
)


def _candidate(**overrides: object) -> SimpleNamespace:
    payload = {
        "source_type": "camera",
        "source_name": "halo_front_camera",
        "source_uri": "/captures/front.mp4",
        "frame_id": "halo_rgb_front",
        "frame_index": 17,
        "monotonic_timestamp_ns": 1_717_200_123_456_789,
        "source_timestamp_ns": 1_717_200_123_000_000,
        "detection_type": "candidate_human_presence",
        "confidence": 0.72,
        "region": {"x": 44, "y": 18, "width": 32, "height": 56},
        "baseline_name": "halo_rgb_region_baseline",
        "baseline_version": "0.1.0",
        "latency_ms": 8.4,
        "confidence_components": {
            "brightness": 0.74,
            "verticality": 0.66,
            "size": 0.59,
            "fill": 0.92,
        },
    }
    payload.update(overrides)
    return SimpleNamespace(**payload)


def test_halo_confidence_event_from_candidate_preserves_halo_metadata() -> None:
    event = halo_confidence_event_from_candidate(_candidate())

    assert event.event_id.startswith("halo-confidence-")
    assert event.source_id == "camera:halo_front_camera"
    assert event.source_name == "halo_front_camera"
    assert event.source_uri == "/captures/front.mp4"
    assert event.frame_id == "halo_rgb_front"
    assert event.frame_index == 17
    assert event.timestamp_ns == 1_717_200_123_456_789
    assert event.label == "Candidate Human Presence"
    assert event.detection_type == "candidate_human_presence"
    assert event.confidence == pytest.approx(0.72)
    assert event.confidence_level == "MEDIUM"
    assert event.region == {"x": 44, "y": 18, "width": 32, "height": 56}
    assert event.recognition["baseline_name"] == "halo_rgb_region_baseline"
    assert event.recognition["baseline_version"] == "0.1.0"
    assert event.recognition["confidence_components"]["fill"] == pytest.approx(0.92)


def test_parse_halo_confidence_event_derives_defaults_and_clamps_confidence() -> None:
    event = parse_halo_confidence_event(
        {
            "source_id": "camera:halo_front_camera",
            "source_name": "halo_front_camera",
            "frame_id": "halo_rgb_front",
            "frame_index": 8,
            "timestamp_ns": 123_456_789,
            "detection_type": "candidate_human_presence",
            "confidence": 1.4,
            "recognition": {
                "baseline_name": "halo_rgb_region_baseline",
                "baseline_version": "0.1.0",
            },
        }
    )

    assert event.event_id.startswith("halo-confidence-")
    assert event.label == "Candidate Human Presence"
    assert event.confidence == 1.0
    assert event.confidence_level == "HIGH"
    assert event.source_uri is None
    assert event.region is None
    assert event.location_hint is None
    assert event.evidence_ref is None
    assert event.evidence_uri is None


def test_parse_halo_confidence_event_round_trip_preserves_optional_fields() -> None:
    payload = {
        "event_id": "halo-confidence-explicit",
        "source_id": "camera:halo_front_camera",
        "source_name": "halo_front_camera",
        "source_uri": "rtsp://halo/front",
        "frame_id": "halo_rgb_front",
        "frame_index": 21,
        "timestamp_ns": 999_000_321,
        "label": "Possible Survivor",
        "detection_type": "candidate_human_presence",
        "confidence": 0.88,
        "confidence_level": "HIGH",
        "region": {"x": 11, "y": 15, "width": 20, "height": 28},
        "location_hint": {"label": "Zone E-2", "x": 12.5, "y": 0.0, "z": -6.0},
        "evidence_ref": "evidence-017",
        "evidence_uri": "file:///tmp/halo/evidence-017.json",
        "recognition": {
            "baseline_name": "halo_rgb_region_baseline",
            "baseline_version": "0.1.0",
            "confidence_components": {"brightness": 0.91},
        },
    }

    event = parse_halo_confidence_event(payload)

    assert event.to_dict() == payload


@pytest.mark.parametrize(
    "payload, message",
    [
        (
            {
                "source_name": "halo_front_camera",
                "frame_id": "halo_rgb_front",
                "frame_index": 1,
                "timestamp_ns": 10,
                "detection_type": "candidate_human_presence",
                "confidence": 0.5,
            },
            "source_id",
        ),
        (
            {
                "source_id": "camera:halo_front_camera",
                "source_name": "halo_front_camera",
                "frame_id": "",
                "frame_index": 1,
                "timestamp_ns": 10,
                "detection_type": "candidate_human_presence",
                "confidence": 0.5,
            },
            "frame_id",
        ),
        (
            {
                "source_id": "camera:halo_front_camera",
                "source_name": "halo_front_camera",
                "frame_id": "halo_rgb_front",
                "frame_index": 1,
                "timestamp_ns": 10,
                "detection_type": "candidate_human_presence",
                "confidence": 0.5,
                "region": {"x": 1, "y": 2, "width": 0, "height": 3},
                "recognition": {
                    "baseline_name": "halo_rgb_region_baseline",
                    "baseline_version": "0.1.0",
                },
            },
            "width",
        ),
        (
            {
                "source_id": "camera:halo_front_camera",
                "source_name": "halo_front_camera",
                "source_uri": 123,
                "frame_id": "halo_rgb_front",
                "frame_index": 1,
                "timestamp_ns": 10,
                "detection_type": "candidate_human_presence",
                "confidence": 0.5,
                "recognition": {
                    "baseline_name": "halo_rgb_region_baseline",
                    "baseline_version": "0.1.0",
                },
            },
            "source_uri",
        ),
        (
            {
                "source_id": "camera:halo_front_camera",
                "source_name": "halo_front_camera",
                "frame_id": "halo_rgb_front",
                "frame_index": 1.9,
                "timestamp_ns": 10,
                "detection_type": "candidate_human_presence",
                "confidence": 0.5,
                "recognition": {
                    "baseline_name": "halo_rgb_region_baseline",
                    "baseline_version": "0.1.0",
                },
            },
            "frame_index",
        ),
        (
            {
                "source_id": "camera:halo_front_camera",
                "source_name": "halo_front_camera",
                "frame_id": "halo_rgb_front",
                "frame_index": 1,
                "timestamp_ns": 10.2,
                "detection_type": "candidate_human_presence",
                "confidence": 0.5,
                "recognition": {
                    "baseline_name": "halo_rgb_region_baseline",
                    "baseline_version": "0.1.0",
                },
            },
            "timestamp_ns",
        ),
        (
            {
                "source_id": "camera:halo_front_camera",
                "source_name": "halo_front_camera",
                "frame_id": "halo_rgb_front",
                "frame_index": 1,
                "timestamp_ns": 10,
                "detection_type": "candidate_human_presence",
                "confidence": 0.5,
                "region": {"x": 1.4, "y": 2, "width": 3, "height": 4},
                "recognition": {
                    "baseline_name": "halo_rgb_region_baseline",
                    "baseline_version": "0.1.0",
                },
            },
            "x",
        ),
    ],
)
def test_parse_halo_confidence_event_rejects_invalid_payloads(
    payload: dict[str, object], message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        parse_halo_confidence_event(payload)
