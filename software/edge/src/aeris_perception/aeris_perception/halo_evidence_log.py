"""Append-only Halo evidence log records for replayable confidence events."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import time
from typing import Any, Mapping

from .halo_confidence_event import (
    HaloConfidenceEvent,
    parse_halo_confidence_event,
    serialize_halo_confidence_event,
)


HALO_EVIDENCE_LOG_SCHEMA_VERSION = 1
SUPPORTED_HALO_EVIDENCE_LOG_MODES = {"live", "replay", "evaluation"}


class HaloEvidenceLogError(ValueError):
    """Raised when a Halo evidence log record cannot be written or replayed."""


@dataclass(frozen=True)
class HaloEvidenceLogRecord:
    """One persisted Halo evidence event plus replay metadata."""

    schema_version: int
    sequence: int
    recorded_at_ns: int
    run_id: str
    mode: str
    event: HaloConfidenceEvent
    evidence_path: str | None = None
    evidence_ref: str | None = None
    evidence_uri: str | None = None

    def to_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "schema_version": self.schema_version,
            "sequence": self.sequence,
            "recorded_at_ns": str(self.recorded_at_ns),
            "run_id": self.run_id,
            "mode": self.mode,
            "event": serialize_halo_confidence_event(self.event),
        }
        if self.evidence_path is not None:
            payload["evidence_path"] = self.evidence_path
        if self.evidence_ref is not None:
            payload["evidence_ref"] = self.evidence_ref
        if self.evidence_uri is not None:
            payload["evidence_uri"] = self.evidence_uri
        return payload


def append_halo_evidence_log_record(
    log_path: Path | str,
    event: HaloConfidenceEvent,
    *,
    run_id: str,
    mode: str,
    evidence_path: Path | str | None = None,
    recorded_at_ns: int | None = None,
) -> HaloEvidenceLogRecord:
    """Append one Halo evidence record to a JSONL log."""

    resolved_log_path = Path(log_path).expanduser().resolve()
    resolved_evidence_path = _normalize_evidence_path(evidence_path)
    existing_records = (
        _read_halo_evidence_log(
            resolved_log_path,
            validate_evidence_paths=False,
        )
        if resolved_log_path.exists()
        else []
    )
    _require_evidence_handle(
        evidence_path=resolved_evidence_path,
        evidence_ref=event.evidence_ref,
        evidence_uri=event.evidence_uri,
    )

    record = HaloEvidenceLogRecord(
        schema_version=HALO_EVIDENCE_LOG_SCHEMA_VERSION,
        sequence=len(existing_records),
        recorded_at_ns=_require_int(
            time.time_ns() if recorded_at_ns is None else recorded_at_ns,
            "recorded_at_ns",
            minimum=0,
        ),
        run_id=_require_non_empty_string(run_id, "run_id"),
        mode=_normalize_mode(mode),
        event=event,
        evidence_path=resolved_evidence_path,
        evidence_ref=event.evidence_ref,
        evidence_uri=event.evidence_uri,
    )

    resolved_log_path.parent.mkdir(parents=True, exist_ok=True)
    with resolved_log_path.open("a", encoding="utf-8") as log_file:
        log_file.write(json.dumps(record.to_dict(), sort_keys=True))
        log_file.write("\n")

    return record


def read_halo_evidence_log(log_path: Path | str) -> list[HaloEvidenceLogRecord]:
    """Load Halo evidence log records in persisted order."""

    resolved_log_path = Path(log_path).expanduser().resolve()
    return _read_halo_evidence_log(
        resolved_log_path,
        validate_evidence_paths=True,
    )


def _read_halo_evidence_log(
    resolved_log_path: Path,
    *,
    validate_evidence_paths: bool,
) -> list[HaloEvidenceLogRecord]:
    records: list[HaloEvidenceLogRecord] = []
    with resolved_log_path.open("r", encoding="utf-8") as log_file:
        for line_number, line in enumerate(log_file, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                raw_payload = json.loads(stripped)
            except json.JSONDecodeError as error:
                raise HaloEvidenceLogError(
                    f"Malformed Halo evidence log record at line {line_number}"
                ) from error
            record = _parse_halo_evidence_log_record(
                raw_payload,
                line_number=line_number,
                validate_evidence_path=validate_evidence_paths,
            )
            if not records and record.sequence != 0:
                raise HaloEvidenceLogError(
                    "Halo evidence log must start at sequence 0 "
                    f"at line {line_number}"
                )
            if records and record.sequence != records[-1].sequence + 1:
                raise HaloEvidenceLogError(
                    "Halo evidence log sequence must increase by 1 in persisted order "
                    f"at line {line_number}"
                )
            records.append(record)
    return records


def replay_halo_evidence_log(log_path: Path | str) -> list[HaloEvidenceLogRecord]:
    """Replay persisted Halo evidence events without rerunning recognition."""

    return read_halo_evidence_log(log_path)


def _parse_halo_evidence_log_record(
    payload: Any,
    *,
    line_number: int,
    validate_evidence_path: bool,
) -> HaloEvidenceLogRecord:
    if not isinstance(payload, Mapping):
        raise HaloEvidenceLogError(
            f"Halo evidence log record at line {line_number} must be an object"
        )

    schema_version = _require_mapping_int(payload, "schema_version", minimum=1)
    if schema_version != HALO_EVIDENCE_LOG_SCHEMA_VERSION:
        raise HaloEvidenceLogError(
            "Unsupported Halo evidence log schema version "
            f"{schema_version} at line {line_number}"
        )

    try:
        event = parse_halo_confidence_event(_require_mapping_object(payload, "event"))
    except ValueError as error:
        raise HaloEvidenceLogError(
            f"Malformed Halo confidence event in evidence log at line {line_number}: {error}"
        ) from error
    evidence_path = _normalize_evidence_path(
        payload.get("evidence_path"),
        validate_exists=validate_evidence_path,
    )
    evidence_ref = _optional_string(payload.get("evidence_ref"), "evidence_ref")
    evidence_uri = _optional_string(payload.get("evidence_uri"), "evidence_uri")

    if (
        evidence_ref is not None
        and event.evidence_ref is not None
        and evidence_ref != event.evidence_ref
    ):
        raise HaloEvidenceLogError(
            f"Halo evidence log evidence_ref mismatch at line {line_number}"
        )
    if (
        evidence_uri is not None
        and event.evidence_uri is not None
        and evidence_uri != event.evidence_uri
    ):
        raise HaloEvidenceLogError(
            f"Halo evidence log evidence_uri mismatch at line {line_number}"
        )

    return HaloEvidenceLogRecord(
        schema_version=schema_version,
        sequence=_require_mapping_int(payload, "sequence", minimum=0),
        recorded_at_ns=_require_mapping_int(payload, "recorded_at_ns", minimum=0),
        run_id=_require_mapping_string(payload, "run_id"),
        mode=_normalize_mode(payload.get("mode")),
        event=event,
        evidence_path=evidence_path,
        evidence_ref=evidence_ref or event.evidence_ref,
        evidence_uri=evidence_uri or event.evidence_uri,
    )


def _normalize_evidence_path(
    raw_value: Any,
    *,
    validate_exists: bool = True,
) -> str | None:
    if raw_value is None:
        return None
    if isinstance(raw_value, Path):
        evidence_path = str(raw_value)
    elif isinstance(raw_value, str):
        evidence_path = raw_value
    else:
        raise HaloEvidenceLogError("evidence_path must be a string")
    evidence_path = evidence_path.strip()
    if not evidence_path:
        return None

    resolved_path = Path(evidence_path).expanduser().resolve()
    if validate_exists and not resolved_path.exists():
        raise HaloEvidenceLogError(
            f"Halo evidence path does not exist: '{resolved_path}'"
        )
    if validate_exists and not resolved_path.is_file():
        raise HaloEvidenceLogError(
            f"Halo evidence path is not a file: '{resolved_path}'"
        )
    return str(resolved_path)


def _normalize_mode(raw_value: Any) -> str:
    mode = _require_non_empty_string(raw_value, "mode").lower()
    if mode not in SUPPORTED_HALO_EVIDENCE_LOG_MODES:
        supported_modes = ", ".join(sorted(SUPPORTED_HALO_EVIDENCE_LOG_MODES))
        raise HaloEvidenceLogError(f"mode must be one of {supported_modes}")
    return mode


def _require_evidence_handle(
    *,
    evidence_path: str | None,
    evidence_ref: str | None,
    evidence_uri: str | None,
) -> None:
    if (
        evidence_path is None
        and evidence_ref is None
        and evidence_uri is None
    ):
        raise HaloEvidenceLogError(
            "Halo evidence log record requires at least one evidence handle"
        )


def _require_mapping_object(
    payload: Mapping[str, Any],
    field_name: str,
) -> Mapping[str, Any]:
    raw_value = payload.get(field_name)
    if not isinstance(raw_value, Mapping):
        raise HaloEvidenceLogError(f"{field_name} must be an object")
    return raw_value


def _require_mapping_string(payload: Mapping[str, Any], field_name: str) -> str:
    return _require_non_empty_string(payload.get(field_name), field_name)


def _require_mapping_int(
    payload: Mapping[str, Any], field_name: str, *, minimum: int = 0
) -> int:
    return _require_int(payload.get(field_name), field_name, minimum=minimum)


def _require_non_empty_string(raw_value: Any, field_name: str) -> str:
    if not isinstance(raw_value, str) or not raw_value.strip():
        raise HaloEvidenceLogError(f"{field_name} must be a non-empty string")
    return raw_value.strip()


def _optional_string(raw_value: Any, field_name: str) -> str | None:
    if raw_value is None:
        return None
    if not isinstance(raw_value, str):
        raise HaloEvidenceLogError(f"{field_name} must be a string")
    normalized = raw_value.strip()
    return normalized or None


def _require_int(raw_value: Any, field_name: str, *, minimum: int = 0) -> int:
    if isinstance(raw_value, bool):
        raise HaloEvidenceLogError(f"{field_name} must be an integer")
    if isinstance(raw_value, int):
        value = raw_value
    elif isinstance(raw_value, str) and raw_value.strip():
        try:
            value = int(raw_value.strip())
        except ValueError as error:
            raise HaloEvidenceLogError(f"{field_name} must be an integer") from error
    else:
        raise HaloEvidenceLogError(f"{field_name} must be an integer")

    if value < minimum:
        raise HaloEvidenceLogError(f"{field_name} must be >= {minimum}")
    return value
