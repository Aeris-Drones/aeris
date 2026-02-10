"""Shared vehicle-id normalization for orchestrator modules."""

from __future__ import annotations

import re


def normalize_vehicle_id(value: str) -> str:
    normalized = value.strip().lower().replace("-", "_")
    if not normalized:
        return ""
    normalized = re.sub(r"([a-z]+)(\d+)", r"\1_\2", normalized)
    normalized = re.sub(r"_+", "_", normalized).strip("_")
    return normalized
