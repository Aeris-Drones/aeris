"""Deterministic spatial-temporal fusion for thermal, acoustic, and gas detections."""

from __future__ import annotations

from dataclasses import dataclass, field
import math


@dataclass(frozen=True)
class FusionConfig:
    correlation_window_sec: float = 3.0
    spatial_gate_m: float = 12.0
    candidate_ttl_sec: float = 20.0
    strong_confidence_min: float = 0.75
    weak_confidence_min: float = 0.40
    max_future_skew_sec: float = 0.25


@dataclass(frozen=True)
class NormalizedDetection:
    modality: str
    timestamp_sec: float
    confidence: float
    local_x: float
    local_z: float
    geometry: tuple[tuple[float, float], ...] = ()


@dataclass(frozen=True)
class FusedDetectionResult:
    candidate_id: str
    timestamp_sec: float
    confidence: float
    confidence_level: str
    source_modalities: tuple[str, ...]
    local_x: float
    local_z: float
    geometry: tuple[tuple[float, float], ...]


@dataclass
class _Candidate:
    candidate_id: str
    created_sec: float
    last_update_sec: float
    local_x: float
    local_z: float
    detections_by_modality: dict[str, NormalizedDetection] = field(default_factory=dict)
    max_emitted_level_rank: int = 0


class FusionEngine:
    """Associate modality detections into unified confidence-ranked alerts."""

    _LEVEL_RANK = {
        "LOW": 1,
        "MEDIUM": 2,
        "HIGH": 3,
    }

    def __init__(self, config: FusionConfig | None = None) -> None:
        self._config = config or FusionConfig()
        self._candidates: list[_Candidate] = []
        self._next_candidate_index = 1

    def ingest(
        self, detection: NormalizedDetection, *, now_sec: float | None = None
    ) -> FusedDetectionResult | None:
        now = float(now_sec) if now_sec is not None else float(detection.timestamp_sec)
        if not self._detection_is_usable(detection, now_sec=now):
            return None

        self._prune_expired_candidates(now_sec=now)

        candidate = self._select_candidate(detection)
        if candidate is None:
            candidate = self._create_candidate(detection)
            self._candidates.append(candidate)

        self._update_candidate(candidate, detection)
        level, score = self._classify_candidate(candidate)
        if level is None:
            return None

        rank = self._LEVEL_RANK[level]
        if rank <= candidate.max_emitted_level_rank:
            return None

        candidate.max_emitted_level_rank = rank
        geometry = self._combined_geometry(candidate)
        modalities = tuple(sorted(candidate.detections_by_modality.keys()))
        latest_ts = max(
            event.timestamp_sec for event in candidate.detections_by_modality.values()
        )
        return FusedDetectionResult(
            candidate_id=candidate.candidate_id,
            timestamp_sec=float(latest_ts),
            confidence=float(score),
            confidence_level=level,
            source_modalities=modalities,
            local_x=float(candidate.local_x),
            local_z=float(candidate.local_z),
            geometry=geometry,
        )

    def _detection_is_usable(
        self, detection: NormalizedDetection, *, now_sec: float
    ) -> bool:
        if detection.modality.strip() == "":
            return False
        for value in (
            detection.timestamp_sec,
            detection.confidence,
            detection.local_x,
            detection.local_z,
            now_sec,
        ):
            if not math.isfinite(float(value)):
                return False

        if float(detection.confidence) < 0.0:
            return False
        if float(detection.timestamp_sec) > (
            now_sec + max(self._config.max_future_skew_sec, 0.0)
        ):
            return False

        stale_age_sec = now_sec - float(detection.timestamp_sec)
        if stale_age_sec > max(self._config.correlation_window_sec, 0.0):
            return False
        return True

    def _prune_expired_candidates(self, *, now_sec: float) -> None:
        ttl_sec = max(self._config.candidate_ttl_sec, 0.0)
        self._candidates = [
            candidate
            for candidate in self._candidates
            if (now_sec - candidate.last_update_sec) <= ttl_sec
        ]

    def _select_candidate(self, detection: NormalizedDetection) -> _Candidate | None:
        best_candidate: _Candidate | None = None
        best_score = math.inf
        for candidate in self._candidates:
            age_delta = abs(detection.timestamp_sec - candidate.last_update_sec)
            if age_delta > max(self._config.correlation_window_sec, 0.0):
                continue
            distance_m = math.hypot(
                detection.local_x - candidate.local_x,
                detection.local_z - candidate.local_z,
            )
            if distance_m > max(self._config.spatial_gate_m, 0.0):
                continue
            # Deterministic score favors closer candidates, then tighter temporal overlap.
            score = distance_m + (age_delta * 0.1)
            if score < best_score:
                best_score = score
                best_candidate = candidate
        return best_candidate

    def _create_candidate(self, detection: NormalizedDetection) -> _Candidate:
        candidate = _Candidate(
            candidate_id=f"cand-{self._next_candidate_index:05d}",
            created_sec=float(detection.timestamp_sec),
            last_update_sec=float(detection.timestamp_sec),
            local_x=float(detection.local_x),
            local_z=float(detection.local_z),
        )
        self._next_candidate_index += 1
        return candidate

    def _update_candidate(
        self, candidate: _Candidate, detection: NormalizedDetection
    ) -> None:
        previous = candidate.detections_by_modality.get(detection.modality)
        if previous is not None and previous.timestamp_sec > detection.timestamp_sec:
            return

        candidate.detections_by_modality[detection.modality] = detection
        candidate.last_update_sec = max(candidate.last_update_sec, detection.timestamp_sec)

        weighted_x = 0.0
        weighted_z = 0.0
        weight_sum = 0.0
        for entry in candidate.detections_by_modality.values():
            weight = max(float(entry.confidence), 0.01)
            weighted_x += float(entry.local_x) * weight
            weighted_z += float(entry.local_z) * weight
            weight_sum += weight
        if weight_sum > 0.0:
            candidate.local_x = weighted_x / weight_sum
            candidate.local_z = weighted_z / weight_sum

    def _classify_candidate(self, candidate: _Candidate) -> tuple[str | None, float]:
        if not candidate.detections_by_modality:
            return None, 0.0

        strongest = max(
            float(entry.confidence)
            for entry in candidate.detections_by_modality.values()
        )
        modality_count = len(candidate.detections_by_modality)
        strong_min = max(self._config.strong_confidence_min, 0.0)
        weak_min = max(self._config.weak_confidence_min, 0.0)
        if modality_count >= 2:
            confidence = min(1.0, strongest + (0.1 * (modality_count - 1)))
            return "HIGH", confidence
        if strongest >= strong_min:
            return "MEDIUM", min(1.0, strongest)
        if strongest >= weak_min:
            return "LOW", min(1.0, strongest)
        return None, strongest

    def _combined_geometry(self, candidate: _Candidate) -> tuple[tuple[float, float], ...]:
        points: list[tuple[float, float]] = []
        for modality in sorted(candidate.detections_by_modality.keys()):
            detection = candidate.detections_by_modality[modality]
            for point in detection.geometry:
                x_value = float(point[0])
                z_value = float(point[1])
                if not (math.isfinite(x_value) and math.isfinite(z_value)):
                    continue
                points.append((x_value, z_value))

        if len(points) >= 3:
            return tuple(points[:24])
        return (
            (float(candidate.local_x), float(candidate.local_z)),
        )
