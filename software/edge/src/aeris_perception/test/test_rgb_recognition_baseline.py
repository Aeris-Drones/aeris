from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from aeris_perception.rgb_dataset import RgbDatasetCaptureConfig, RgbFrameCaptureWriter
from aeris_perception.rgb_ingest import RgbFrame, RgbFrameMetadata, RgbSourceError


def _frame_metadata(*, frame_index: int) -> RgbFrameMetadata:
    return RgbFrameMetadata(
        source_type="camera",
        source_name="halo_front_camera",
        source_uri="0",
        frame_id="halo_rgb_front",
        frame_index=frame_index,
        monotonic_timestamp_ns=1_000_000_000 + frame_index,
        source_timestamp_ns=None,
        replayed=False,
    )


def _candidate_image() -> np.ndarray:
    image = np.zeros((12, 12, 3), dtype=np.uint8)
    image[2:10, 4:8] = np.array([225, 225, 225], dtype=np.uint8)
    return image


def _background_image() -> np.ndarray:
    return np.full((12, 12, 3), 20, dtype=np.uint8)


def _frame(image_bgr: np.ndarray, *, frame_index: int) -> RgbFrame:
    return RgbFrame(image_bgr=image_bgr, metadata=_frame_metadata(frame_index=frame_index))


def _write_manifest(tmp_path: Path) -> Path:
    output_dir = tmp_path / "capture"
    manifest_path = output_dir / "manifest.jsonl"
    writer = RgbFrameCaptureWriter(
        RgbDatasetCaptureConfig(
            output_dir=output_dir,
            manifest_path=manifest_path,
            image_format="ppm",
            capture_reason="continuous_capture",
        )
    )
    writer.capture(_frame(_candidate_image(), frame_index=0), label="person")
    writer.capture(_frame(_background_image(), frame_index=1), label="background")
    writer.capture(
        _frame(_background_image(), frame_index=2),
        review={"human_present": True, "notes": "label pending"},
    )
    return manifest_path


def test_generate_baseline_candidates_emits_bounded_confidence_and_bbox() -> None:
    from aeris_perception.rgb_recognition_baseline import generate_baseline_candidates

    candidates = generate_baseline_candidates(
        _frame(_candidate_image(), frame_index=7)
    )

    assert len(candidates) == 1
    candidate = candidates[0]
    assert candidate.detection_type == "candidate_human_presence"
    assert candidate.source_name == "halo_front_camera"
    assert candidate.frame_id == "halo_rgb_front"
    assert candidate.frame_index == 7
    assert 0.0 <= candidate.confidence <= 1.0
    assert candidate.region is not None
    assert candidate.region["width"] > 0
    assert candidate.region["height"] > 0
    assert candidate.baseline_name
    assert candidate.baseline_version


def test_evaluate_rgb_manifest_writes_detections_and_summary(tmp_path) -> None:
    from aeris_perception.rgb_recognition_baseline import evaluate_rgb_dataset_manifest

    manifest_path = _write_manifest(tmp_path)
    detections_path = tmp_path / "detections.jsonl"
    summary_path = tmp_path / "summary.json"

    clock_values = iter(
        (
            1_000_000,
            2_500_000,
            5_000_000,
            6_500_000,
            9_000_000,
            10_500_000,
        )
    )

    summary = evaluate_rgb_dataset_manifest(
        manifest_path,
        detections_output_path=detections_path,
        summary_output_path=summary_path,
        latency_clock_ns=lambda: next(clock_values),
    )

    assert summary.frames_processed == 3
    assert summary.frames_skipped == 0
    assert summary.detections_emitted == 1
    assert summary.labeled_frame_count == 3
    assert summary.unlabeled_frame_count == 0
    assert summary.false_positive_frames == 0
    assert summary.false_negative_frames == 1
    assert summary.dataset_coverage["processed_ratio"] == 1.0
    assert summary.confidence_summary["count"] == 1
    assert summary.latency_summary_ms["p95"] == 1.5

    detection_records = [
        json.loads(line)
        for line in detections_path.read_text(encoding="utf-8").splitlines()
        if line
    ]
    assert len(detection_records) == 1
    assert detection_records[0]["confidence"] == pytest.approx(
        summary.confidence_summary["max"]
    )
    assert detection_records[0]["latency_ms"] == pytest.approx(1.5)
    assert detection_records[0]["region"]["height"] > 0

    summary_record = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary_record["frames_processed"] == 3
    assert summary_record["false_negative_frames"] == 1


def test_evaluate_rgb_manifest_handles_unlabeled_entries(tmp_path) -> None:
    from aeris_perception.rgb_recognition_baseline import evaluate_rgb_dataset_manifest

    output_dir = tmp_path / "capture"
    manifest_path = output_dir / "manifest.jsonl"
    writer = RgbFrameCaptureWriter(
        RgbDatasetCaptureConfig(
            output_dir=output_dir,
            manifest_path=manifest_path,
            image_format="ppm",
            capture_reason="continuous_capture",
        )
    )
    writer.capture(_frame(_candidate_image(), frame_index=0))
    writer.capture(_frame(_background_image(), frame_index=1))

    summary = evaluate_rgb_dataset_manifest(manifest_path)

    assert summary.frames_processed == 2
    assert summary.labeled_frame_count == 0
    assert summary.unlabeled_frame_count == 2
    assert summary.false_positive_frames == 0
    assert summary.false_negative_frames == 0


def test_evaluate_rgb_manifest_rejects_empty_manifest(tmp_path) -> None:
    from aeris_perception.rgb_recognition_baseline import evaluate_rgb_dataset_manifest

    manifest_path = tmp_path / "manifest.jsonl"
    manifest_path.write_text("", encoding="utf-8")

    with pytest.raises(RgbSourceError, match="no capture entries"):
        evaluate_rgb_dataset_manifest(manifest_path)


def test_evaluate_rgb_manifest_rejects_malformed_manifest(tmp_path) -> None:
    from aeris_perception.rgb_recognition_baseline import evaluate_rgb_dataset_manifest

    manifest_path = tmp_path / "manifest.jsonl"
    manifest_path.write_text("{not-json}\n", encoding="utf-8")

    with pytest.raises(RgbSourceError, match="Malformed RGB dataset manifest"):
        evaluate_rgb_dataset_manifest(manifest_path)
