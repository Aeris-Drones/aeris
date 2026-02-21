"""Message envelope helpers for deterministic buffering and replay metadata."""

from __future__ import annotations

from base64 import b64decode, b64encode
from collections.abc import Mapping
from hashlib import sha256
import json
import time
from typing import Any


def serialize_message_payload(message: Any) -> bytes:
    """Serialize a ROS-like message object into stable JSON bytes."""
    primitive = _to_primitive(message)
    return json.dumps(primitive, separators=(",", ":"), sort_keys=True).encode("utf-8")


def deserialize_message_payload(message_type: type[Any], payload_bytes: bytes) -> Any:
    """Deserialize JSON payload bytes back into the target message type."""
    payload = json.loads(payload_bytes.decode("utf-8"))
    message = message_type()

    # Preferred ROS helper path for nested typed message reconstruction.
    try:
        from rosidl_runtime_py.set_message import set_message_fields

        set_message_fields(message, payload)
        return message
    except Exception:
        pass

    _populate_object(message, payload)
    return message


def compute_payload_hash(payload_bytes: bytes) -> str:
    return sha256(payload_bytes).hexdigest()


def extract_event_timestamp(message: Any, *, fallback: float | None = None) -> float:
    """Extract event timestamp from common ROS stamp fields."""
    if fallback is None:
        fallback = time.time()

    stamp = getattr(message, "stamp", None)
    if stamp is None:
        stamp = getattr(message, "timestamp", None)
    if stamp is None:
        header = getattr(message, "header", None)
        if header is not None:
            stamp = getattr(header, "stamp", None)
    if stamp is None:
        return float(fallback)

    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return float(fallback)
    return float(sec) + float(nanosec) / 1_000_000_000.0


def build_dedupe_key(
    *,
    message_kind: str,
    topic: str,
    message: Any,
    payload_hash: str,
) -> str:
    """Build deterministic dedupe keys aligned with story requirements."""
    kind = message_kind.lower()
    if kind == "map_tile":
        tile_id = getattr(message, "tile_id", "")
        tile_hash = getattr(message, "hash_sha256", "") or payload_hash
        return f"map_tile:{tile_id}:{tile_hash}"
    if kind == "fused_detection":
        candidate_id = getattr(message, "candidate_id", "")
        stamp = getattr(message, "stamp", None)
        sec = getattr(stamp, "sec", 0) if stamp is not None else 0
        nsec = getattr(stamp, "nanosec", 0) if stamp is not None else 0
        return f"fused_detection:{candidate_id}:{sec}:{nsec}"
    if kind == "telemetry":
        vehicle_id = getattr(message, "vehicle_id", "")
        stamp = getattr(message, "timestamp", None)
        sec = getattr(stamp, "sec", 0) if stamp is not None else 0
        nsec = getattr(stamp, "nanosec", 0) if stamp is not None else 0
        return f"telemetry:{vehicle_id}:{sec}:{nsec}"
    if kind == "heartbeat":
        data = getattr(message, "data", "")
        return _heartbeat_dedupe(data=data, payload_hash=payload_hash)
    return f"{topic}:{payload_hash}"


def _heartbeat_dedupe(*, data: str, payload_hash: str) -> str:
    if not data:
        return f"heartbeat:{payload_hash}"
    try:
        maybe_json = json.loads(data)
        if isinstance(maybe_json, Mapping):
            vehicle_id = str(maybe_json.get("vehicle_id", ""))
            ts = str(maybe_json.get("timestamp", maybe_json.get("stamp", "")))
            if vehicle_id or ts:
                return f"heartbeat:{vehicle_id}:{ts}:{payload_hash[:12]}"
    except json.JSONDecodeError:
        pass
    return f"heartbeat:{payload_hash}"


def _to_primitive(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, bytes):
        return {"__bytes__": b64encode(value).decode("ascii")}
    if isinstance(value, bytearray):
        return {"__bytes__": b64encode(bytes(value)).decode("ascii")}
    if isinstance(value, (list, tuple)):
        return [_to_primitive(item) for item in value]
    if isinstance(value, Mapping):
        return {str(k): _to_primitive(v) for k, v in value.items()}

    if hasattr(value, "get_fields_and_field_types"):
        out: dict[str, Any] = {}
        fields = value.get_fields_and_field_types().keys()
        for field in sorted(fields):
            out[field] = _to_primitive(getattr(value, field))
        return out

    if hasattr(value, "__dict__"):
        return {str(k): _to_primitive(v) for k, v in value.__dict__.items()}

    return repr(value)


def _populate_object(target: Any, payload: Any) -> Any:
    if isinstance(payload, Mapping):
        for field, raw in payload.items():
            if not hasattr(target, field):
                continue
            current = getattr(target, field)
            converted = _convert_for_target(current, raw)
            try:
                setattr(target, field, converted)
            except Exception:
                # Fall back to in-place update for nested ROS-like structs.
                _populate_object(current, raw)
    return target


def _convert_for_target(current: Any, raw: Any) -> Any:
    if isinstance(raw, Mapping) and "__bytes__" in raw:
        return b64decode(str(raw["__bytes__"]).encode("ascii"))

    if hasattr(current, "get_fields_and_field_types") and isinstance(raw, Mapping):
        _populate_object(current, raw)
        return current

    if isinstance(current, list) and isinstance(raw, list):
        return [_convert_list_item(item) for item in raw]

    return raw


def _convert_list_item(raw: Any) -> Any:
    if isinstance(raw, Mapping) and "__bytes__" in raw:
        return b64decode(str(raw["__bytes__"]).encode("ascii"))
    return raw
