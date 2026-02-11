"""Shared vehicle-id normalization for orchestrator modules.

Enforces consistent vehicle identifier formatting across ROS topic names,
configuration payloads, and inter-node communication. Handles common
variations (case, hyphen/underscore, concatenated numbers) to ensure
reliable endpoint matching regardless of input source conventions.
"""

from __future__ import annotations

import re


def normalize_vehicle_id(value: str) -> str:
    normalized = value.strip().lower().replace("-", "_")
    if not normalized:
        return ""
    normalized = re.sub(r"([a-z]+)(\d+)", r"\1_\2", normalized)
    normalized = re.sub(r"_+", "_", normalized).strip("_")
    return normalized
