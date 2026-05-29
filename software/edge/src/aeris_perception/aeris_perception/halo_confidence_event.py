"""Canonical Halo confidence event contract for RGB recognition output."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import TYPE_CHECKING, Any, Mapping

if TYPE_CHECKING:
    from .rgb_recognition_baseline import RecognitionCandidate


_HIGH_CONFIDENCE_MIN = 0.85
_MEDIUM_CONFIDENCE_MIN = 0.60
_LOW_CONFIDENCE_MIN = 0.30


@dataclass(frozen=True)
class HaloConfidenceEvent:
    """Stable event payload shared between Halo perception and the viewer."""

    event_id: str
    source_id: str
    source_name: str
    source_uri: str | None
    frame_id: str
    frame_index: int
    timestamp_ns: int
    label: str
    detection_type: str
    confidence: float
    confidence_level: str
    recognition: dict[str, Any]
    region: dict[str, int] | None = None
    location_hint: str | dict[str, Any] | None = None
    evidence_ref: str | None = None
    evidence_uri: str | None = None

    def to_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "event_id": self.event_id,
            "source_id": self.source_id,
            "source_name": self.source_name,
            "frame_id": self.frame_id,
            "frame_index": self.frame_index,
            "timestamp_ns": self.timestamp_ns,
            "label": self.label,
            "detection_type": self.detection_type,
            "confidence": _round_float(self.confidence),
            "confidence_level": self.confidence_level,
            "recognition": _serialize_recognition(self.recognition),
        }
        if self.source_uri is not None:
            payload["source_uri"] = self.source_uri
        if self.region is not None:
            payload["region"] = dict(self.region)
        if self.location_hint is not None:
            payload["location_hint"] = _serialize_location_hint(self.location_hint)
        if self.evidence_ref is not None:
            payload["evidence_ref"] = self.evidence_ref
        if self.evidence_uri is not None:
            payload["evidence_uri"] = self.evidence_uri
        return payload


def halo_confidence_event_from_candidate(
    candidate: RecognitionCandidate,
    *,
    location_hint: str | Mapping[str, Any] | None = None,
    evidence_ref: str | None = None,
    evidence_uri: str | None = None,
) -> HaloConfidenceEvent:
    """Convert a baseline recognition candidate into the canonical Halo event."""

    source_name = _require_non_empty_string(candidate.source_name, "source_name")
    source_id = f"{_require_non_empty_string(candidate.source_type, 'source_type')}:{source_name}"
    detection_type = _require_non_empty_string(candidate.detection_type, "detection_type")
    frame_id = _require_non_empty_string(candidate.frame_id, "frame_id")
    region = _normalize_region(candidate.region)
    confidence = _clamp01(candidate.confidence)

    return HaloConfidenceEvent(
        event_id=_build_event_id(
            source_id=source_id,
            frame_id=frame_id,
            frame_index=int(candidate.frame_index),
            timestamp_ns=int(candidate.monotonic_timestamp_ns),
            detection_type=detection_type,
            region=region,
        ),
        source_id=source_id,
        source_name=source_name,
        source_uri=_optional_string(candidate.source_uri),
        frame_id=frame_id,
        frame_index=int(candidate.frame_index),
        timestamp_ns=int(candidate.monotonic_timestamp_ns),
        label=_humanize_detection_type(detection_type),
        detection_type=detection_type,
        confidence=confidence,
        confidence_level=confidence_level_for_score(confidence),
        region=region,
        location_hint=_normalize_location_hint(location_hint),
        evidence_ref=_optional_string(evidence_ref),
        evidence_uri=_optional_string(evidence_uri),
        recognition=_normalize_recognition(
            {
                "baseline_name": candidate.baseline_name,
                "baseline_version": candidate.baseline_version,
                "latency_ms": candidate.latency_ms,
                "source_timestamp_ns": candidate.source_timestamp_ns,
                "confidence_components": candidate.confidence_components,
            }
        ),
    )


def serialize_halo_confidence_event(event: HaloConfidenceEvent) -> dict[str, Any]:
    """Serialize a Halo confidence event into a JSON-safe dictionary."""

    return event.to_dict()


def parse_halo_confidence_event(payload: Mapping[str, Any]) -> HaloConfidenceEvent:
    """Parse and validate an inbound Halo confidence event payload."""

    if not isinstance(payload, Mapping):
        raise ValueError("Halo confidence event payload must be a mapping")

    source_id = _require_mapping_string(payload, "source_id")
    source_name = _require_mapping_string(payload, "source_name")
    frame_id = _require_mapping_string(payload, "frame_id")
    frame_index = _require_mapping_int(payload, "frame_index", minimum=0)
    timestamp_ns = _require_mapping_int(payload, "timestamp_ns", minimum=0)
    detection_type = _optional_string(payload.get("detection_type"))
    label = _optional_string(payload.get("label"))
    if detection_type is None and label is None:
        raise ValueError("Halo confidence event requires detection_type or label")
    resolved_detection_type = detection_type or _slugify_label(label or "")
    resolved_label = label or _humanize_detection_type(resolved_detection_type)
    confidence = _clamp01(_require_mapping_float(payload, "confidence"))
    recognition = _normalize_recognition(payload.get("recognition"))
    region = _normalize_region(payload.get("region"))

    return HaloConfidenceEvent(
        event_id=_optional_string(payload.get("event_id"))
        or _build_event_id(
            source_id=source_id,
            frame_id=frame_id,
            frame_index=frame_index,
            timestamp_ns=timestamp_ns,
            detection_type=resolved_detection_type,
            region=region,
        ),
        source_id=source_id,
        source_name=source_name,
        source_uri=_optional_string(payload.get("source_uri")),
        frame_id=frame_id,
        frame_index=frame_index,
        timestamp_ns=timestamp_ns,
        label=resolved_label,
        detection_type=resolved_detection_type,
        confidence=confidence,
        confidence_level=_normalize_confidence_level(
            payload.get("confidence_level"), confidence
        ),
        region=region,
        location_hint=_normalize_location_hint(payload.get("location_hint")),
        evidence_ref=_optional_string(payload.get("evidence_ref")),
        evidence_uri=_optional_string(payload.get("evidence_uri")),
        recognition=recognition,
    )


def confidence_level_for_score(confidence: float) -> str:
    """Map a bounded confidence score onto the Halo display tiers."""

    score = _clamp01(confidence)
    if score >= _HIGH_CONFIDENCE_MIN:
        return "HIGH"
    if score >= _MEDIUM_CONFIDENCE_MIN:
        return "MEDIUM"
    if score >= _LOW_CONFIDENCE_MIN:
        return "LOW"
    return "UNKNOWN"


def _build_event_id(
    *,
    source_id: str,
    frame_id: str,
    frame_index: int,
    timestamp_ns: int,
    detection_type: str,
    region: Mapping[str, int] | None,
) -> str:
    region_key = (
        f"{region['x']}:{region['y']}:{region['width']}:{region['height']}"
        if region is not None
        else "none"
    )
    return (
        "halo-confidence-"
        f"{source_id}:{frame_id}:{frame_index}:{timestamp_ns}:{detection_type}:{region_key}"
    )


def _normalize_confidence_level(raw_level: Any, confidence: float) -> str:
    level = _optional_string(raw_level)
    if level is None:
        return confidence_level_for_score(confidence)
    normalized = level.upper()
    if normalized in {"HIGH", "MEDIUM", "LOW", "UNKNOWN"}:
        return normalized
    return confidence_level_for_score(confidence)


def _normalize_recognition(raw_value: Any) -> dict[str, Any]:
    if not isinstance(raw_value, Mapping):
        raise ValueError("recognition must be an object")

    baseline_name = _require_non_empty_string(raw_value.get("baseline_name"), "baseline_name")
    baseline_version = _require_non_empty_string(
        raw_value.get("baseline_version"), "baseline_version"
    )
    payload: dict[str, Any] = {
        "baseline_name": baseline_name,
        "baseline_version": baseline_version,
    }

    latency_ms = raw_value.get("latency_ms")
    if latency_ms is not None:
        payload["latency_ms"] = _round_float(_require_finite_float(latency_ms, "latency_ms"))

    source_timestamp_ns = raw_value.get("source_timestamp_ns")
    if source_timestamp_ns is not None:
        payload["source_timestamp_ns"] = _require_int(
            source_timestamp_ns, "source_timestamp_ns", minimum=0
        )

    components = raw_value.get("confidence_components")
    if components is not None:
        if not isinstance(components, Mapping):
            raise ValueError("confidence_components must be an object")
        normalized_components: dict[str, float] = {}
        for key, value in components.items():
            component_name = _require_non_empty_string(key, "confidence_components key")
            normalized_components[component_name] = _round_float(
                _clamp01(_require_finite_float(value, component_name))
            )
        payload["confidence_components"] = normalized_components

    return payload


def _serialize_recognition(recognition: Mapping[str, Any]) -> dict[str, Any]:
    payload = {
        "baseline_name": recognition["baseline_name"],
        "baseline_version": recognition["baseline_version"],
    }
    if "latency_ms" in recognition:
        payload["latency_ms"] = _round_float(float(recognition["latency_ms"]))
    if "source_timestamp_ns" in recognition:
        payload["source_timestamp_ns"] = int(recognition["source_timestamp_ns"])
    if "confidence_components" in recognition:
        payload["confidence_components"] = {
            str(key): _round_float(float(value))
            for key, value in recognition["confidence_components"].items()
        }
    return payload


def _normalize_region(raw_value: Any) -> dict[str, int] | None:
    if raw_value is None:
        return None
    if not isinstance(raw_value, Mapping):
        raise ValueError("region must be an object")

    region = {
        axis: _require_int(raw_value.get(axis), axis, minimum=0)
        for axis in ("x", "y")
    }
    for dimension in ("width", "height"):
        region[dimension] = _require_int(raw_value.get(dimension), dimension, minimum=1)
    return region


def _normalize_location_hint(raw_value: Any) -> str | dict[str, Any] | None:
    if raw_value is None:
        return None
    if isinstance(raw_value, str):
        return _require_non_empty_string(raw_value, "location_hint")
    if not isinstance(raw_value, Mapping):
        raise ValueError("location_hint must be a string or object")

    payload: dict[str, Any] = {}
    label = raw_value.get("label")
    if label is not None:
        payload["label"] = _require_non_empty_string(label, "location_hint.label")
    for axis in ("x", "y", "z"):
        axis_value = raw_value.get(axis)
        if axis_value is None:
            continue
        payload[axis] = _require_finite_float(axis_value, f"location_hint.{axis}")
    if not payload:
        raise ValueError("location_hint must include a label or coordinates")
    return payload


def _serialize_location_hint(location_hint: str | Mapping[str, Any]) -> str | dict[str, Any]:
    if isinstance(location_hint, str):
        return location_hint
    payload: dict[str, Any] = {}
    if "label" in location_hint:
        payload["label"] = str(location_hint["label"])
    for axis in ("x", "y", "z"):
        if axis in location_hint:
            payload[axis] = float(location_hint[axis])
    return payload


def _slugify_label(label: str) -> str:
    normalized = "_".join(label.strip().lower().split())
    return _require_non_empty_string(normalized, "label")


def _humanize_detection_type(detection_type: str) -> str:
    parts = [segment for segment in detection_type.strip().split("_") if segment]
    if not parts:
        raise ValueError("detection_type must not be empty")
    return " ".join(segment.capitalize() for segment in parts)


def _require_mapping_string(payload: Mapping[str, Any], key: str) -> str:
    return _require_non_empty_string(payload.get(key), key)


def _require_mapping_int(payload: Mapping[str, Any], key: str, *, minimum: int) -> int:
    return _require_int(payload.get(key), key, minimum=minimum)


def _require_mapping_float(payload: Mapping[str, Any], key: str) -> float:
    return _require_finite_float(payload.get(key), key)


def _require_non_empty_string(raw_value: Any, field_name: str) -> str:
    if not isinstance(raw_value, str) or raw_value.strip() == "":
        raise ValueError(f"{field_name} must be a non-empty string")
    return raw_value.strip()


def _optional_string(raw_value: Any) -> str | None:
    if raw_value is None:
        return None
    if not isinstance(raw_value, str):
        raise ValueError("optional string field must be a string when provided")
    stripped = raw_value.strip()
    return stripped or None


def _require_int(raw_value: Any, field_name: str, *, minimum: int) -> int:
    if isinstance(raw_value, bool):
        raise ValueError(f"{field_name} must be an integer")
    try:
        value = int(raw_value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} must be an integer") from exc
    if value < minimum:
        raise ValueError(f"{field_name} must be >= {minimum}")
    return value


def _require_finite_float(raw_value: Any, field_name: str) -> float:
    try:
        value = float(raw_value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} must be numeric") from exc
    if not math.isfinite(value):
        raise ValueError(f"{field_name} must be numeric")
    return value


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def _round_float(value: float) -> float:
    return round(float(value), 6)
