"""Deterministic RGB candidate detection baseline and manifest evaluator."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, replace
import json
from math import ceil
from pathlib import Path
import time
from typing import Any, Callable, Sequence

import numpy as np

from .rgb_dataset import (
    ManifestReplayFrameSource,
    RgbDatasetManifestEntry,
    load_rgb_dataset_manifest,
)
from .rgb_ingest import RgbFrame, RgbSourceConfig, RgbSourceError


BASELINE_NAME = "halo_rgb_region_baseline"
BASELINE_VERSION = "0.1.0"
DETECTION_TYPE = "candidate_human_presence"
MIN_CANDIDATE_CONFIDENCE = 0.5

_POSITIVE_LABEL_TOKENS = {
    "candidate_human_presence",
    "contains_people",
    "contains_person",
    "human",
    "human_present",
    "human_presence",
    "people",
    "person",
    "person_present",
    "survivor",
    "victim",
}
_NEGATIVE_LABEL_TOKENS = {
    "background",
    "clear",
    "empty",
    "negative",
    "no_human",
    "no_person",
    "none",
}
_REVIEW_BOOLEAN_KEYS = (
    "contains_people",
    "contains_person",
    "expected_presence",
    "human_present",
    "human_presence",
    "person_present",
)


@dataclass(frozen=True)
class RecognitionCandidate:
    """Inspectable output record for one baseline candidate detection."""

    source_type: str
    source_name: str
    source_uri: str
    frame_id: str
    frame_index: int
    monotonic_timestamp_ns: int
    source_timestamp_ns: int | None
    detection_type: str
    confidence: float
    region: dict[str, int] | None
    baseline_name: str
    baseline_version: str
    latency_ms: float = 0.0
    confidence_components: dict[str, float] | None = None

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["confidence"] = _round_float(self.confidence)
        payload["latency_ms"] = _round_float(self.latency_ms)
        if self.confidence_components is not None:
            payload["confidence_components"] = {
                key: _round_float(value)
                for key, value in self.confidence_components.items()
            }
        return payload


@dataclass(frozen=True)
class FrameEvaluationResult:
    """Per-frame evaluation output kept in the summary JSON."""

    source_name: str
    source_uri: str
    frame_id: str
    frame_index: int
    monotonic_timestamp_ns: int
    latency_ms: float
    detections_emitted: int
    label: str | None
    review: dict[str, Any] | None
    expected_human_presence: bool | None

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["latency_ms"] = _round_float(self.latency_ms)
        return payload


@dataclass(frozen=True)
class RecognitionEvaluationSummary:
    """Aggregate evaluation output for one manifest run."""

    manifest_path: str
    baseline_name: str
    baseline_version: str
    frames_processed: int
    frames_skipped: int
    detections_emitted: int
    confidence_summary: dict[str, Any]
    latency_summary_ms: dict[str, Any]
    dataset_coverage: dict[str, Any]
    labeled_frame_count: int
    unlabeled_frame_count: int
    reviewed_frame_count: int
    false_positive_frames: int
    false_negative_frames: int
    label_metrics_available: bool
    frame_results: list[FrameEvaluationResult]

    def to_dict(self) -> dict[str, Any]:
        return {
            "manifest_path": self.manifest_path,
            "baseline_name": self.baseline_name,
            "baseline_version": self.baseline_version,
            "frames_processed": self.frames_processed,
            "frames_skipped": self.frames_skipped,
            "detections_emitted": self.detections_emitted,
            "confidence_summary": self.confidence_summary,
            "latency_summary_ms": self.latency_summary_ms,
            "dataset_coverage": self.dataset_coverage,
            "labeled_frame_count": self.labeled_frame_count,
            "unlabeled_frame_count": self.unlabeled_frame_count,
            "reviewed_frame_count": self.reviewed_frame_count,
            "false_positive_frames": self.false_positive_frames,
            "false_negative_frames": self.false_negative_frames,
            "label_metrics_available": self.label_metrics_available,
            "frame_results": [result.to_dict() for result in self.frame_results],
        }


def generate_baseline_candidates(frame: RgbFrame) -> list[RecognitionCandidate]:
    """Generate deterministic candidate detections from one RGB frame."""

    image_bgr = frame.image_bgr
    if image_bgr.dtype != np.uint8:
        raise RgbSourceError("RGB recognition baseline requires uint8 images")
    if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise RgbSourceError("RGB recognition baseline requires HxWx3 images")

    luminance = _image_luminance(image_bgr)
    threshold = max(32.0, float(luminance.mean()) + (float(luminance.std()) * 0.60))
    bright_mask = luminance >= threshold
    components = _connected_components(bright_mask)
    if not components:
        return []

    total_pixels = int(image_bgr.shape[0] * image_bgr.shape[1])
    min_component_area = max(4, total_pixels // 80)

    candidates: list[RecognitionCandidate] = []
    for component in components:
        area = len(component)
        if area < min_component_area:
            continue

        points = np.asarray(component, dtype=int)
        ys = points[:, 0]
        xs = points[:, 1]
        min_y = int(ys.min())
        max_y = int(ys.max())
        min_x = int(xs.min())
        max_x = int(xs.max())
        width = (max_x - min_x) + 1
        height = (max_y - min_y) + 1
        if width <= 0 or height <= 0:
            continue

        bbox_area = width * height
        area_ratio = area / total_pixels
        aspect_ratio = height / max(width, 1)
        fill_ratio = area / bbox_area
        component_mean = float(luminance[ys, xs].mean())
        brightness_score = _bounded((component_mean - float(luminance.mean())) / 128.0)
        verticality_score = _bounded((aspect_ratio - 0.75) / 2.5)
        size_score = _bounded(area_ratio * 24.0)
        fill_score = _bounded(fill_ratio)
        confidence = _bounded(
            0.15
            + (0.35 * brightness_score)
            + (0.25 * verticality_score)
            + (0.15 * size_score)
            + (0.10 * fill_score)
        )
        if confidence < MIN_CANDIDATE_CONFIDENCE:
            continue

        candidates.append(
            RecognitionCandidate(
                source_type=frame.metadata.source_type,
                source_name=frame.metadata.source_name,
                source_uri=frame.metadata.source_uri,
                frame_id=frame.metadata.frame_id,
                frame_index=frame.metadata.frame_index,
                monotonic_timestamp_ns=frame.metadata.monotonic_timestamp_ns,
                source_timestamp_ns=frame.metadata.source_timestamp_ns,
                detection_type=DETECTION_TYPE,
                confidence=confidence,
                region={
                    "x": int(min_x),
                    "y": int(min_y),
                    "width": int(width),
                    "height": int(height),
                },
                baseline_name=BASELINE_NAME,
                baseline_version=BASELINE_VERSION,
                confidence_components={
                    "brightness": brightness_score,
                    "verticality": verticality_score,
                    "size": size_score,
                    "fill": fill_score,
                },
            )
        )

    candidates.sort(key=lambda candidate: candidate.confidence, reverse=True)
    return candidates[:8]


def evaluate_rgb_dataset_manifest(
    manifest_path: Path | str,
    *,
    detections_output_path: Path | str | None = None,
    summary_output_path: Path | str | None = None,
    latency_clock_ns: Callable[[], int] | None = None,
    candidate_generator: Callable[[RgbFrame], list[RecognitionCandidate]] | None = None,
) -> RecognitionEvaluationSummary:
    """Run the recognition baseline over a dataset manifest."""

    resolved_manifest_path = Path(manifest_path).expanduser().resolve()
    entries = _load_validated_manifest_entries(resolved_manifest_path)
    source = ManifestReplayFrameSource(
        config=RgbSourceConfig(
            source_type="manifest_replay",
            source_uri=str(resolved_manifest_path),
            frame_id=entries[0].metadata.frame_id,
            source_name=entries[0].metadata.source_name,
        ),
        entries=entries,
        manifest_path=resolved_manifest_path,
    )
    detector = candidate_generator or generate_baseline_candidates
    clock = latency_clock_ns or time.perf_counter_ns

    detections: list[RecognitionCandidate] = []
    frame_results: list[FrameEvaluationResult] = []
    latency_values_ms: list[float] = []
    frames_processed = 0
    false_positive_frames = 0
    false_negative_frames = 0
    labeled_frame_count = 0
    reviewed_frame_count = 0

    try:
        for entry in entries:
            start_ns = int(clock())
            frame = source.read()
            if frame is None:
                raise RgbSourceError(
                    "RGB dataset manifest replay ended before all frames were read"
                )

            frame_candidates = detector(frame)
            end_ns = int(clock())
            latency_ms = _round_float(max(0, end_ns - start_ns) / 1_000_000.0)
            latency_values_ms.append(latency_ms)
            frames_processed += 1

            expected_human_presence = _expected_human_presence(entry)
            if expected_human_presence is not None:
                labeled_frame_count += 1
            if entry.review is not None:
                reviewed_frame_count += 1

            if expected_human_presence is True and not frame_candidates:
                false_negative_frames += 1
            if expected_human_presence is False and frame_candidates:
                false_positive_frames += 1

            for candidate in frame_candidates:
                detections.append(replace(candidate, latency_ms=latency_ms))

            frame_results.append(
                FrameEvaluationResult(
                    source_name=frame.metadata.source_name,
                    source_uri=frame.metadata.source_uri,
                    frame_id=frame.metadata.frame_id,
                    frame_index=frame.metadata.frame_index,
                    monotonic_timestamp_ns=frame.metadata.monotonic_timestamp_ns,
                    latency_ms=latency_ms,
                    detections_emitted=len(frame_candidates),
                    label=entry.label,
                    review=entry.review,
                    expected_human_presence=expected_human_presence,
                )
            )
    finally:
        source.close()

    if frames_processed == 0:
        raise RgbSourceError(
            f"RGB dataset manifest has no readable frames: '{resolved_manifest_path}'"
        )

    summary = RecognitionEvaluationSummary(
        manifest_path=str(resolved_manifest_path),
        baseline_name=BASELINE_NAME,
        baseline_version=BASELINE_VERSION,
        frames_processed=frames_processed,
        frames_skipped=0,
        detections_emitted=len(detections),
        confidence_summary=_confidence_summary(detections),
        latency_summary_ms=_latency_summary(latency_values_ms),
        dataset_coverage={
            "frames_in_manifest": len(entries),
            "frames_processed": frames_processed,
            "frames_skipped": 0,
            "processed_ratio": _round_float(frames_processed / len(entries)),
        },
        labeled_frame_count=labeled_frame_count,
        unlabeled_frame_count=len(entries) - labeled_frame_count,
        reviewed_frame_count=reviewed_frame_count,
        false_positive_frames=false_positive_frames,
        false_negative_frames=false_negative_frames,
        label_metrics_available=labeled_frame_count > 0,
        frame_results=frame_results,
    )

    if detections_output_path is not None:
        _write_jsonl(Path(detections_output_path), [item.to_dict() for item in detections])
    if summary_output_path is not None:
        _write_json(Path(summary_output_path), summary.to_dict())
    return summary


def main(argv: Sequence[str] | None = None) -> int:
    """Run the manifest-backed recognition baseline from the command line."""

    parser = argparse.ArgumentParser(
        description=(
            "Run the Halo RGB recognition baseline over a dataset manifest and "
            "write inspectable detection/evaluation outputs."
        )
    )
    parser.add_argument("manifest_path", help="Path to the RGB dataset manifest JSONL.")
    parser.add_argument(
        "--detections-output",
        dest="detections_output",
        help="JSONL path for emitted candidate detections.",
    )
    parser.add_argument(
        "--summary-output",
        dest="summary_output",
        help="JSON path for aggregate evaluation output.",
    )
    args = parser.parse_args(argv)

    manifest_path = Path(args.manifest_path).expanduser().resolve()
    detections_output_path = Path(args.detections_output).expanduser().resolve() if args.detections_output else manifest_path.with_name(f"{manifest_path.stem}_recognition_detections.jsonl")
    summary_output_path = Path(args.summary_output).expanduser().resolve() if args.summary_output else manifest_path.with_name(f"{manifest_path.stem}_recognition_summary.json")

    summary = evaluate_rgb_dataset_manifest(
        manifest_path,
        detections_output_path=detections_output_path,
        summary_output_path=summary_output_path,
    )
    print(json.dumps(summary.to_dict(), indent=2, sort_keys=True))
    return 0


def _load_validated_manifest_entries(
    manifest_path: Path,
) -> list[RgbDatasetManifestEntry]:
    if not manifest_path.exists():
        raise RgbSourceError(
            f"RGB dataset manifest path does not exist: '{manifest_path}'"
        )

    entries = load_rgb_dataset_manifest(manifest_path)
    if not entries:
        raise RgbSourceError(
            f"RGB dataset manifest has no capture entries: '{manifest_path}'"
        )

    for entry in entries:
        image_path = (manifest_path.parent / entry.relative_image_path).resolve()
        if not image_path.exists():
            raise RgbSourceError(
                f"RGB dataset capture file does not exist: '{image_path}'"
            )
        _validated_review_payload(entry)
    return entries


def _image_luminance(image_bgr: np.ndarray) -> np.ndarray:
    return (
        (0.114 * image_bgr[:, :, 0])
        + (0.587 * image_bgr[:, :, 1])
        + (0.299 * image_bgr[:, :, 2])
    )


def _connected_components(mask: np.ndarray) -> list[list[tuple[int, int]]]:
    components: list[list[tuple[int, int]]] = []
    height, width = mask.shape
    visited = np.zeros_like(mask, dtype=bool)

    for y_index in range(height):
        for x_index in range(width):
            if not bool(mask[y_index, x_index]) or bool(visited[y_index, x_index]):
                continue

            component: list[tuple[int, int]] = []
            stack = [(y_index, x_index)]
            visited[y_index, x_index] = True
            while stack:
                current_y, current_x = stack.pop()
                component.append((current_y, current_x))
                for next_y, next_x in (
                    (current_y - 1, current_x),
                    (current_y + 1, current_x),
                    (current_y, current_x - 1),
                    (current_y, current_x + 1),
                ):
                    if next_y < 0 or next_x < 0 or next_y >= height or next_x >= width:
                        continue
                    if not bool(mask[next_y, next_x]) or bool(visited[next_y, next_x]):
                        continue
                    visited[next_y, next_x] = True
                    stack.append((next_y, next_x))
            components.append(component)

    return components


def _expected_human_presence(entry: RgbDatasetManifestEntry) -> bool | None:
    review_payload = _validated_review_payload(entry)
    if review_payload is not None:
        for key in _REVIEW_BOOLEAN_KEYS:
            value = review_payload.get(key)
            if isinstance(value, bool):
                return value

    if entry.label is None:
        return None

    token = _normalize_label(entry.label)
    if token in _POSITIVE_LABEL_TOKENS:
        return True
    if token in _NEGATIVE_LABEL_TOKENS:
        return False
    return None


def _normalize_label(label: str) -> str:
    return str(label).strip().lower().replace("-", "_").replace(" ", "_")


def _validated_review_payload(
    entry: RgbDatasetManifestEntry,
) -> dict[str, Any] | None:
    if entry.review is None:
        return None
    if isinstance(entry.review, dict):
        return entry.review

    metadata = entry.metadata
    raise RgbSourceError(
        "Malformed review payload for RGB dataset manifest entry "
        f"source '{metadata.source_name}' frame_index {metadata.frame_index}: "
        "expected object or null"
    )


def _confidence_summary(
    detections: list[RecognitionCandidate],
) -> dict[str, Any]:
    if not detections:
        return {
            "count": 0,
            "min": None,
            "max": None,
            "mean": None,
            "buckets": {"low": 0, "medium": 0, "high": 0},
        }

    confidences = [candidate.confidence for candidate in detections]
    return {
        "count": len(confidences),
        "min": _round_float(min(confidences)),
        "max": _round_float(max(confidences)),
        "mean": _round_float(sum(confidences) / len(confidences)),
        "buckets": {
            "low": sum(confidence < 0.34 for confidence in confidences),
            "medium": sum(0.34 <= confidence < 0.67 for confidence in confidences),
            "high": sum(confidence >= 0.67 for confidence in confidences),
        },
    }


def _latency_summary(latency_values_ms: list[float]) -> dict[str, Any]:
    if not latency_values_ms:
        return {"min": None, "p50": None, "p95": None, "max": None, "mean": None}

    return {
        "min": _round_float(min(latency_values_ms)),
        "p50": _percentile(latency_values_ms, 50),
        "p95": _percentile(latency_values_ms, 95),
        "max": _round_float(max(latency_values_ms)),
        "mean": _round_float(sum(latency_values_ms) / len(latency_values_ms)),
    }


def _percentile(values: list[float], percentile: int) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    rank = max(0, ceil((percentile / 100.0) * len(ordered)) - 1)
    return _round_float(ordered[rank])


def _bounded(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def _round_float(value: float) -> float:
    return round(float(value), 4)


def _write_jsonl(path: Path, records: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as output_file:
        for record in records:
            output_file.write(json.dumps(record, sort_keys=True))
            output_file.write("\n")


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
