"""Shared helpers for simulation SLAM validation scripts and tests.

This module provides utility functions and data structures for parsing
ROS 2 topic output, loading trajectory waypoints, and computing geometric
relationships for SLAM validation tasks.

Typical usage example:
    # Parse topic hz output
    rate = parse_average_rate(hz_output)

    # Load and check trajectory
    waypoints = load_trajectory_waypoints(path)
    is_loop = is_closed_loop(waypoints, tolerance_m=1.0)
"""

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
    """Immutable 3D point with x, y, z coordinates."""

    x: float
    y: float
    z: float


def parse_average_rate(hz_output: str) -> float | None:
    """Extract the last reported average rate from ros2 topic hz output.

    Args:
        hz_output: Raw output string from 'ros2 topic hz' command.

    Returns:
        The last average rate as a float, or None if no rate found.
    """
    matches = _AVERAGE_RATE_RE.findall(hz_output)
    if not matches:
        return None
    return float(matches[-1])


def has_loop_closure_signal(info_log: str, pattern: str) -> bool:
    """Check for loop-closure indicators in RTAB-Map info output.

    Args:
        info_log: Log output from RTAB-Map info topic.
        pattern: Regular expression pattern to search for.

    Returns:
        True if the pattern is found (case-insensitive), False otherwise.
    """
    return re.search(pattern, info_log, re.IGNORECASE) is not None


def load_trajectory_waypoints(path: Path) -> list[Point3D]:
    """Load trajectory waypoints from a JSON file.

    Args:
        path: Path to the JSON waypoint file.

    Returns:
        List of Point3D objects representing the trajectory.

    Raises:
        ValueError: If the file has fewer than 2 waypoints or invalid coordinates.
    """
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
    """Extract all position samples from ros2 topic echo output.

    Parses nav_msgs/Odometry formatted output to extract position coordinates.

    Args:
        odom_echo_log: Raw output from 'ros2 topic echo' on an Odometry topic.

    Returns:
        List of Point3D objects representing extracted positions.
    """
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
    """Calculate Euclidean distance between two 3D points.

    Args:
        point_a: First point.
        point_b: Second point.

    Returns:
        Euclidean distance in the same units as the points.
    """
    return math.dist((point_a.x, point_a.y, point_a.z), (point_b.x, point_b.y, point_b.z))


def is_closed_loop(points: Iterable[Point3D], tolerance_m: float) -> bool:
    """Check if a trajectory forms a closed loop.

    Args:
        points: Iterable of trajectory points.
        tolerance_m: Maximum distance between start and end to be considered closed.

    Returns:
        True if start and end points are within tolerance, False otherwise.
    """
    point_list = list(points)
    if len(point_list) < 2:
        return False
    return distance(point_list[0], point_list[-1]) <= tolerance_m


def is_pose_near(sample: Point3D, target: Point3D, tolerance_m: float) -> bool:
    """Check if a sample point is near a target within tolerance.

    Args:
        sample: The point to check.
        target: The reference point.
        tolerance_m: Maximum allowed distance.

    Returns:
        True if sample is within tolerance of target, False otherwise.
    """
    return distance(sample, target) <= tolerance_m
