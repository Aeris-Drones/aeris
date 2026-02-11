"""Shared vehicle-id normalization for orchestrator modules.

Enforces consistent vehicle identifier formatting across ROS topic names,
configuration payloads, and inter-node communication. Handles common
variations (case, hyphen/underscore, concatenated numbers) to ensure
reliable endpoint matching regardless of input source conventions.
"""

from __future__ import annotations

import re


def normalize_vehicle_id(value: str) -> str:
    """Normalize a vehicle identifier to standard format.

    Converts input to lowercase, replaces hyphens with underscores,
    inserts underscores between alphabetic and numeric characters,
    and collapses multiple consecutive underscores.

    Args:
        value: Raw vehicle identifier string to normalize.

    Returns:
        Normalized vehicle identifier or empty string if input is empty
        or contains only whitespace/underscores.

    Examples:
        >>> normalize_vehicle_id(" scout1 ")
        'scout_1'
        >>> normalize_vehicle_id("RANGER-2")
        'ranger_2'
        >>> normalize_vehicle_id("alpha__03")
        'alpha_03'
        >>> normalize_vehicle_id("___")
        ''
    """
    normalized = value.strip().lower().replace("-", "_")
    if not normalized:
        return ""
    normalized = re.sub(r"([a-z]+)(\d+)", r"\1_\2", normalized)
    normalized = re.sub(r"_+", "_").strip("_")
    return normalized
