"""Shared helpers for simulation SLAM validation scripts and tests."""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

_AVERAGE_RATE_RE = re.compile(r"average rate:\s*([0-9]+(?:\.[0-9]+)?)")
_FLOAT_RE = r"[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?"
_POSITION_BLOCK_RE = re.compile(
    rf"position:\s*\n\s*x:\s*({_FLOAT_RE})\s*\n\s*y:\s*({_FLOAT_RE})\s*\n\s*z:\s*({_FLOAT_RE})",
    re.MULTILINE,
)


@dataclass(frozen=True)
class Point3D:
    x: float
    y: float
    z: float


def parse_average_rate(hz_output: str) -> float | None:
    """Return the last reported average rate from ros2 topic hz output."""
    matches = _AVERAGE_RATE_RE.findall(hz_output)
    if not matches:
        return None
    return float(matches[-1])


def has_loop_closure_signal(info_log: str, pattern: str) -> bool:
    """Return True if loop-closure indicators are present in RTAB-Map info output."""
    return re.search(pattern, info_log, re.IGNORECASE) is not None


def load_trajectory_waypoints(path: Path) -> list[Point3D]:
    """Load trajectory waypoints from JSON and normalize coordinates to float."""
    data = json.loads(path.read_text())
    waypoints = data.get("waypoints")
    if not isinstance(waypoints, list) or len(waypoints) < 2:
        raise ValueError("trajectory must include at least two waypoints")

    points: list[Point3D] = []
    for idx, waypoint in enumerate(waypoints):
        if not isinstance(waypoint, dict):
            raise ValueError(f"waypoint {idx} must be an object")
        try:
            points.append(
                Point3D(
                    x=float(waypoint["x"]),
                    y=float(waypoint["y"]),
                    z=float(waypoint["z"]),
                )
            )
        except KeyError as exc:
            raise ValueError(f"waypoint {idx} missing coordinate: {exc}") from exc
        except (TypeError, ValueError) as exc:
            raise ValueError(f"waypoint {idx} has invalid numeric coordinate") from exc
    return points


def extract_odom_positions(odom_echo_log: str) -> list[Point3D]:
    """Extract all position samples from ros2 topic echo output for nav_msgs/Odometry."""
    samples: list[Point3D] = []
    for match in _POSITION_BLOCK_RE.finditer(odom_echo_log):
        samples.append(
            Point3D(
                x=float(match.group(1)),
                y=float(match.group(2)),
                z=float(match.group(3)),
            )
        )
    return samples


def distance(point_a: Point3D, point_b: Point3D) -> float:
    return math.dist((point_a.x, point_a.y, point_a.z), (point_b.x, point_b.y, point_b.z))


def is_closed_loop(points: Iterable[Point3D], tolerance_m: float) -> bool:
    point_list = list(points)
    if len(point_list) < 2:
        return False
    return distance(point_list[0], point_list[-1]) <= tolerance_m


def is_pose_near(sample: Point3D, target: Point3D, tolerance_m: float) -> bool:
    return distance(sample, target) <= tolerance_m
