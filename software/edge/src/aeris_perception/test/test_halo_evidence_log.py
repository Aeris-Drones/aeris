from __future__ import annotations

import fcntl
import json
from pathlib import Path

import pytest

from aeris_perception.halo_confidence_event import (
    HaloConfidenceEvent,
    parse_halo_confidence_event,
)
from aeris_perception.halo_evidence_log import (
    HALO_EVIDENCE_LOG_SCHEMA_VERSION,
    HaloEvidenceLogError,
    append_halo_evidence_log_record,
    read_halo_evidence_log,
    replay_halo_evidence_log,
)


def _event(
    *,
    event_id: str = "halo-confidence-001",
    frame_index: int = 17,
    timestamp_ns: str = "1717200123456789",
    evidence_ref: str | None = "capture-017",
    evidence_uri: str | None = "file:///tmp/halo/capture-017.png",
) -> HaloConfidenceEvent:
    event: HaloConfidenceEvent = parse_halo_confidence_event(
        {
            "event_id": event_id,
            "source_id": "camera:halo_front_camera",
            "source_name": "halo_front_camera",
            "source_uri": "rtsp://halo/front",
            "frame_id": "halo_rgb_front",
            "frame_index": frame_index,
            "timestamp_ns": timestamp_ns,
            "label": "Candidate Human Presence",
            "detection_type": "candidate_human_presence",
            "confidence": 0.72,
            "confidence_level": "MEDIUM",
            "region": {"x": 11, "y": 15, "width": 20, "height": 28},
            "recognition": {
                "baseline_name": "halo_rgb_region_baseline",
                "baseline_version": "0.1.0",
                "source_timestamp_ns": "1717200123000000",
                "confidence_components": {"brightness": 0.91, "fill": 0.87},
            },
            "evidence_ref": evidence_ref,
            "evidence_uri": evidence_uri,
        }
    )
    return event


def test_append_halo_evidence_log_record_persists_canonical_event_jsonl(
    tmp_path: Path,
) -> None:
    log_path = tmp_path / "logs" / "halo_evidence.jsonl"
    evidence_path = tmp_path / "captures" / "frame-017.png"
    evidence_path.parent.mkdir(parents=True)
    evidence_path.write_bytes(b"png")

    record = append_halo_evidence_log_record(
        log_path,
        _event(),
        run_id="halo-demo-run",
        mode="evaluation",
        evidence_path=evidence_path,
        recorded_at_ns=1_717_200_555_000_000_001,
    )

    assert record.schema_version == HALO_EVIDENCE_LOG_SCHEMA_VERSION
    assert record.sequence == 0
    assert record.recorded_at_ns == 1_717_200_555_000_000_001
    assert record.run_id == "halo-demo-run"
    assert record.mode == "evaluation"
    assert record.evidence_path == str(evidence_path.resolve())
    assert record.evidence_ref == "capture-017"
    assert record.evidence_uri == "file:///tmp/halo/capture-017.png"
    assert record.event.timestamp_ns == 1_717_200_123_456_789

    raw_records = [
        json.loads(line)
        for line in log_path.read_text(encoding="utf-8").splitlines()
        if line
    ]
    assert raw_records == [
        {
            "schema_version": HALO_EVIDENCE_LOG_SCHEMA_VERSION,
            "sequence": 0,
            "recorded_at_ns": "1717200555000000001",
            "run_id": "halo-demo-run",
            "mode": "evaluation",
            "event": record.event.to_dict(),
            "evidence_path": str(evidence_path.resolve()),
            "evidence_ref": "capture-017",
            "evidence_uri": "file:///tmp/halo/capture-017.png",
        }
    ]


def test_append_halo_evidence_log_record_increments_sequence_and_replays_in_file_order(
    tmp_path: Path,
) -> None:
    log_path = tmp_path / "halo_evidence.jsonl"
    first_evidence = tmp_path / "captures" / "frame-001.png"
    second_evidence = tmp_path / "captures" / "frame-002.png"
    first_evidence.parent.mkdir(parents=True)
    first_evidence.write_bytes(b"frame-1")
    second_evidence.write_bytes(b"frame-2")

    append_halo_evidence_log_record(
        log_path,
        _event(event_id="halo-confidence-001", frame_index=9, timestamp_ns="300"),
        run_id="halo-live-run",
        mode="live",
        evidence_path=first_evidence,
        recorded_at_ns=900,
    )
    append_halo_evidence_log_record(
        log_path,
        _event(event_id="halo-confidence-002", frame_index=4, timestamp_ns="100"),
        run_id="halo-live-run",
        mode="replay",
        evidence_path=second_evidence,
        recorded_at_ns=950,
    )

    records = read_halo_evidence_log(log_path)
    replay_records = replay_halo_evidence_log(log_path)

    assert [record.sequence for record in records] == [0, 1]
    assert [record.event.event_id for record in records] == [
        "halo-confidence-001",
        "halo-confidence-002",
    ]
    assert [record.event.timestamp_ns for record in replay_records] == [300, 100]


def test_append_halo_evidence_log_record_uses_advisory_lock_for_sequence_allocation(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    log_path = tmp_path / "halo_evidence.jsonl"
    evidence_path = tmp_path / "captures" / "frame-017.png"
    evidence_path.parent.mkdir(parents=True)
    evidence_path.write_bytes(b"png")
    log_path.write_text(
        json.dumps(
            {
                "schema_version": HALO_EVIDENCE_LOG_SCHEMA_VERSION,
                "sequence": 0,
                "recorded_at_ns": "1717200555000000001",
                "run_id": "halo-demo-run",
                "mode": "evaluation",
                "event": _event().to_dict(),
                "evidence_path": str(evidence_path.resolve()),
            }
        )
        + "\n",
        encoding="utf-8",
    )
    lock_modes: list[int] = []

    def _fake_flock(file_descriptor: int, operation: int) -> None:
        assert file_descriptor >= 0
        lock_modes.append(operation)

    monkeypatch.setattr(fcntl, "flock", _fake_flock)

    record = append_halo_evidence_log_record(
        log_path,
        _event(event_id="halo-confidence-002"),
        run_id="halo-demo-run",
        mode="evaluation",
        evidence_path=evidence_path,
        recorded_at_ns=1_717_200_555_000_000_002,
    )

    assert record.sequence == 1
    assert lock_modes == [
        fcntl.LOCK_EX,
        fcntl.LOCK_UN,
    ]


def test_append_halo_evidence_log_record_rejects_missing_local_evidence_path(
    tmp_path: Path,
) -> None:
    log_path = tmp_path / "halo_evidence.jsonl"
    missing_path = tmp_path / "captures" / "missing.png"

    with pytest.raises(HaloEvidenceLogError, match="does not exist"):
        append_halo_evidence_log_record(
            log_path,
            _event(),
            run_id="halo-demo-run",
            mode="evaluation",
            evidence_path=missing_path,
        )


def test_append_halo_evidence_log_record_requires_at_least_one_evidence_handle(
    tmp_path: Path,
) -> None:
    log_path = tmp_path / "halo_evidence.jsonl"

    with pytest.raises(HaloEvidenceLogError, match="evidence handle"):
        append_halo_evidence_log_record(
            log_path,
            _event(evidence_ref=None, evidence_uri=None),
            run_id="halo-demo-run",
            mode="evaluation",
        )


def test_read_halo_evidence_log_rejects_unsupported_schema_version(
    tmp_path: Path,
) -> None:
    log_path = tmp_path / "halo_evidence.jsonl"
    evidence_path = tmp_path / "captures" / "frame-017.png"
    evidence_path.parent.mkdir(parents=True)
    evidence_path.write_bytes(b"png")
    payload = {
        "schema_version": HALO_EVIDENCE_LOG_SCHEMA_VERSION + 1,
        "sequence": 0,
        "recorded_at_ns": "1717200555000000001",
        "run_id": "halo-demo-run",
        "mode": "evaluation",
        "event": _event().to_dict(),
        "evidence_path": str(evidence_path.resolve()),
    }
    log_path.write_text(json.dumps(payload) + "\n", encoding="utf-8")

    with pytest.raises(
        HaloEvidenceLogError,
        match="Unsupported Halo evidence log schema version",
    ):
        read_halo_evidence_log(log_path)


def test_read_halo_evidence_log_rejects_missing_evidence_but_replay_keeps_record(
    tmp_path: Path,
) -> None:
    log_path = tmp_path / "halo_evidence.jsonl"
    payload = {
        "schema_version": HALO_EVIDENCE_LOG_SCHEMA_VERSION,
        "sequence": 0,
        "recorded_at_ns": "1717200555000000001",
        "run_id": "halo-demo-run",
        "mode": "evaluation",
        "event": _event().to_dict(),
        "evidence_path": str((tmp_path / "captures" / "deleted.png").resolve()),
    }
    log_path.write_text(json.dumps(payload) + "\n", encoding="utf-8")

    with pytest.raises(HaloEvidenceLogError, match="does not exist"):
        read_halo_evidence_log(log_path)

    replay_records = replay_halo_evidence_log(log_path)

    assert len(replay_records) == 1
    assert replay_records[0].evidence_path == payload["evidence_path"]
    assert replay_records[0].evidence_ref == "capture-017"
    assert replay_records[0].evidence_uri == "file:///tmp/halo/capture-017.png"


def test_read_halo_evidence_log_requires_first_sequence_to_start_at_zero(
    tmp_path: Path,
) -> None:
    log_path = tmp_path / "halo_evidence.jsonl"
    evidence_path = tmp_path / "captures" / "frame-017.png"
    evidence_path.parent.mkdir(parents=True)
    evidence_path.write_bytes(b"png")
    log_path.write_text(
        json.dumps(
            {
                "schema_version": HALO_EVIDENCE_LOG_SCHEMA_VERSION,
                "sequence": 5,
                "recorded_at_ns": "1717200555000000001",
                "run_id": "halo-demo-run",
                "mode": "evaluation",
                "event": _event().to_dict(),
                "evidence_path": str(evidence_path.resolve()),
            }
        )
        + "\n",
        encoding="utf-8",
    )

    with pytest.raises(HaloEvidenceLogError, match="start at sequence 0"):
        read_halo_evidence_log(log_path)


def test_read_halo_evidence_log_wraps_invalid_nested_event_with_line_context(
    tmp_path: Path,
) -> None:
    log_path = tmp_path / "halo_evidence.jsonl"
    evidence_path = tmp_path / "captures" / "frame-017.png"
    evidence_path.parent.mkdir(parents=True)
    evidence_path.write_bytes(b"png")
    invalid_event = _event().to_dict()
    invalid_event.pop("source_id")
    log_path.write_text(
        json.dumps(
            {
                "schema_version": HALO_EVIDENCE_LOG_SCHEMA_VERSION,
                "sequence": 0,
                "recorded_at_ns": "1717200555000000001",
                "run_id": "halo-demo-run",
                "mode": "evaluation",
                "event": invalid_event,
                "evidence_path": str(evidence_path.resolve()),
            }
        )
        + "\n",
        encoding="utf-8",
    )

    with pytest.raises(HaloEvidenceLogError, match="line 1"):
        read_halo_evidence_log(log_path)


def test_append_halo_evidence_log_record_ignores_missing_historical_evidence_for_sequence(
    tmp_path: Path,
) -> None:
    log_path = tmp_path / "halo_evidence.jsonl"
    current_evidence_path = tmp_path / "captures" / "frame-018.png"
    current_evidence_path.parent.mkdir(parents=True)
    current_evidence_path.write_bytes(b"png")
    log_path.write_text(
        json.dumps(
            {
                "schema_version": HALO_EVIDENCE_LOG_SCHEMA_VERSION,
                "sequence": 0,
                "recorded_at_ns": "1717200555000000001",
                "run_id": "halo-demo-run",
                "mode": "evaluation",
                "event": _event().to_dict(),
                "evidence_path": str((tmp_path / "captures" / "deleted.png").resolve()),
            }
        )
        + "\n",
        encoding="utf-8",
    )

    record = append_halo_evidence_log_record(
        log_path,
        _event(event_id="halo-confidence-002"),
        run_id="halo-demo-run",
        mode="evaluation",
        evidence_path=current_evidence_path,
        recorded_at_ns=1_717_200_555_000_000_002,
    )

    assert record.sequence == 1
    assert len(log_path.read_text(encoding="utf-8").splitlines()) == 2
    with pytest.raises(HaloEvidenceLogError, match="does not exist"):
        read_halo_evidence_log(log_path)


def test_read_halo_evidence_log_rejects_partial_record_payload(
    tmp_path: Path,
) -> None:
    log_path = tmp_path / "halo_evidence.jsonl"
    log_path.write_text(
        json.dumps(
            {
                "schema_version": HALO_EVIDENCE_LOG_SCHEMA_VERSION,
                "sequence": 0,
                "recorded_at_ns": "1717200555000000001",
                "run_id": "halo-demo-run",
                "event": _event().to_dict(),
            }
        )
        + "\n",
        encoding="utf-8",
    )

    with pytest.raises(HaloEvidenceLogError, match="mode"):
        read_halo_evidence_log(log_path)
