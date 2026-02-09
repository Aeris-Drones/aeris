"""Deterministic search pattern planning utilities for mission orchestration."""

from __future__ import annotations

import math
from typing import Iterable

Waypoint = dict[str, float]

_EPSILON = 1e-6


def validate_polygon(polygon: Iterable[dict[str, float]]) -> tuple[bool, str]:
    normalized = _normalize_polygon(polygon)
    if len(normalized) < 3:
        return False, "polygon must contain at least 3 points"

    area = _polygon_area(normalized)
    if area <= _EPSILON:
        return False, "polygon area must be greater than zero"

    return True, ""


def generate_waypoints(
    pattern: str,
    polygon: Iterable[dict[str, float]],
    *,
    lawnmower_track_spacing_m: float = 5.0,
    spiral_radial_step_m: float = 3.0,
    spiral_angular_step_rad: float = 0.35,
) -> list[Waypoint]:
    normalized = _normalize_polygon(polygon)
    if pattern == "lawnmower":
        return generate_lawnmower_waypoints(
            normalized, track_spacing_m=lawnmower_track_spacing_m
        )
    if pattern == "spiral":
        return generate_spiral_waypoints(
            normalized,
            radial_step_m=spiral_radial_step_m,
            angular_step_rad=spiral_angular_step_rad,
        )
    return []


def partition_polygon_for_scouts(
    polygon: Iterable[dict[str, float]], scout_vehicle_ids: Iterable[str]
) -> list[dict[str, object]]:
    normalized = _normalize_polygon(polygon)
    if len(normalized) < 3:
        return []

    ordered_vehicle_ids = sorted(
        {
            _normalize_vehicle_id(vehicle_id)
            for vehicle_id in scout_vehicle_ids
            if _normalize_vehicle_id(vehicle_id)
        }
    )
    if not ordered_vehicle_ids:
        return []

    if len(ordered_vehicle_ids) == 1:
        return [
            {
                "vehicle_id": ordered_vehicle_ids[0],
                "zone_id": "zone-1",
                "polygon": normalized,
            }
        ]

    min_x, max_x, min_z, max_z = _bounding_box(normalized)
    span_x = max_x - min_x
    span_z = max_z - min_z
    axis = "x" if span_x >= span_z else "z"
    axis_min = min_x if axis == "x" else min_z
    axis_max = max_x if axis == "x" else max_z
    span = axis_max - axis_min

    if span <= _EPSILON:
        return [
            {
                "vehicle_id": ordered_vehicle_ids[0],
                "zone_id": "zone-1",
                "polygon": normalized,
            }
        ]

    partitions: list[dict[str, object]] = []
    vehicle_count = len(ordered_vehicle_ids)
    for index, vehicle_id in enumerate(ordered_vehicle_ids):
        lower = axis_min + (span * index / vehicle_count)
        upper = (
            axis_max
            if index == vehicle_count - 1
            else axis_min + (span * (index + 1) / vehicle_count)
        )
        clipped = _clip_polygon_axis_range(
            normalized,
            axis=axis,
            lower=lower,
            upper=upper,
            include_upper=index == vehicle_count - 1,
        )
        if len(clipped) < 3 or _polygon_area(clipped) <= _EPSILON:
            continue
        partitions.append(
            {
                "vehicle_id": vehicle_id,
                "zone_id": f"zone-{index + 1}",
                "polygon": clipped,
            }
        )

    if not partitions:
        return [
            {
                "vehicle_id": ordered_vehicle_ids[0],
                "zone_id": "zone-1",
                "polygon": normalized,
            }
        ]
    return partitions


def generate_lawnmower_waypoints(
    polygon: Iterable[dict[str, float]], *, track_spacing_m: float = 5.0
) -> list[Waypoint]:
    normalized = _normalize_polygon(polygon)
    if track_spacing_m <= 0:
        track_spacing_m = 5.0

    _min_x, _max_x, min_z, max_z = _bounding_box(normalized)
    z = min_z
    row_index = 0
    waypoints: list[Waypoint] = []
    row_limit = math.ceil((max_z - min_z) / track_spacing_m) + 2

    for _ in range(max(row_limit, 1)):
        intersections = _line_intersections_with_polygon(
            normalized, horizontal_line_z=z
        )
        if intersections:
            row_segments: list[tuple[Waypoint, Waypoint]] = []
            for index in range(0, len(intersections) - 1, 2):
                x_start = intersections[index]
                x_end = intersections[index + 1]
                if x_end - x_start <= _EPSILON:
                    continue
                start = {"x": x_start, "z": z}
                end = {"x": x_end, "z": z}
                if _point_in_polygon_or_boundary(start, normalized) and _point_in_polygon_or_boundary(
                    end, normalized
                ):
                    row_segments.append((start, end))

            if row_segments:
                if row_index % 2 == 1:
                    row_segments = list(reversed(row_segments))
                    row_segments = [(end, start) for (start, end) in row_segments]

                for segment_start, segment_end in row_segments:
                    waypoints.append(segment_start)
                    waypoints.append(segment_end)
                row_index += 1

        z += track_spacing_m
        if z > max_z + _EPSILON:
            break

    return _deduplicate_sequential_points(waypoints)


def generate_spiral_waypoints(
    polygon: Iterable[dict[str, float]],
    *,
    radial_step_m: float = 3.0,
    angular_step_rad: float = 0.35,
) -> list[Waypoint]:
    normalized = _normalize_polygon(polygon)
    if radial_step_m <= 0:
        radial_step_m = 3.0
    if angular_step_rad <= 0:
        angular_step_rad = 0.35

    centroid = _centroid(normalized)
    min_x, max_x, min_z, max_z = _bounding_box(normalized)
    max_radius = max(
        math.dist((centroid["x"], centroid["z"]), (min_x, min_z)),
        math.dist((centroid["x"], centroid["z"]), (min_x, max_z)),
        math.dist((centroid["x"], centroid["z"]), (max_x, min_z)),
        math.dist((centroid["x"], centroid["z"]), (max_x, max_z)),
    )

    waypoints: list[Waypoint] = []
    theta = 0.0
    theta_limit = 24.0 * math.pi

    while theta <= theta_limit:
        radius = radial_step_m * theta / (2.0 * math.pi)
        if radius > max_radius + radial_step_m:
            break

        candidate = {
            "x": centroid["x"] + radius * math.cos(theta),
            "z": centroid["z"] + radius * math.sin(theta),
        }
        if _point_in_polygon_or_boundary(candidate, normalized):
            waypoints.append(candidate)
        theta += angular_step_rad

    if not waypoints and _point_in_polygon_or_boundary(centroid, normalized):
        waypoints.append(centroid)

    return _deduplicate_sequential_points(waypoints)


def point_in_polygon(point: dict[str, float], polygon: Iterable[dict[str, float]]) -> bool:
    return _point_in_polygon_or_boundary(point, _normalize_polygon(polygon))


def _normalize_polygon(polygon: Iterable[dict[str, float]]) -> list[Waypoint]:
    normalized: list[Waypoint] = []
    for point in polygon:
        if not isinstance(point, dict):
            continue
        x = point.get("x")
        z = point.get("z")
        if isinstance(x, (int, float)) and isinstance(z, (int, float)):
            x_value = float(x)
            z_value = float(z)
            if not math.isfinite(x_value) or not math.isfinite(z_value):
                continue
            normalized.append({"x": x_value, "z": z_value})

    if len(normalized) >= 2:
        first = normalized[0]
        last = normalized[-1]
        if abs(first["x"] - last["x"]) <= _EPSILON and abs(first["z"] - last["z"]) <= _EPSILON:
            normalized = normalized[:-1]
    return normalized


def _normalize_vehicle_id(value: str) -> str:
    normalized = value.strip().lower().replace("-", "_")
    if not normalized:
        return ""
    return normalized


def _polygon_area(polygon: list[Waypoint]) -> float:
    area = 0.0
    size = len(polygon)
    if size < 3:
        return 0.0
    for index in range(size):
        nxt = (index + 1) % size
        area += polygon[index]["x"] * polygon[nxt]["z"]
        area -= polygon[nxt]["x"] * polygon[index]["z"]
    return abs(area) / 2.0


def _bounding_box(polygon: list[Waypoint]) -> tuple[float, float, float, float]:
    xs = [point["x"] for point in polygon]
    zs = [point["z"] for point in polygon]
    return min(xs), max(xs), min(zs), max(zs)


def _line_intersections_with_polygon(
    polygon: list[Waypoint], *, horizontal_line_z: float
) -> list[float]:
    intersections: list[float] = []
    for index, current in enumerate(polygon):
        nxt = polygon[(index + 1) % len(polygon)]
        z1 = current["z"]
        z2 = nxt["z"]
        x1 = current["x"]
        x2 = nxt["x"]

        if abs(z1 - z2) <= _EPSILON:
            if abs(z1 - horizontal_line_z) <= _EPSILON:
                intersections.extend((min(x1, x2), max(x1, x2)))
            continue

        crossing = (z1 <= horizontal_line_z < z2) or (z2 <= horizontal_line_z < z1)
        if crossing:
            t = (horizontal_line_z - z1) / (z2 - z1)
            intersections.append(x1 + t * (x2 - x1))

    intersections.sort()
    return intersections


def _point_in_polygon_or_boundary(point: Waypoint, polygon: list[Waypoint]) -> bool:
    for index, current in enumerate(polygon):
        nxt = polygon[(index + 1) % len(polygon)]
        if _point_on_segment(point, current, nxt):
            return True

    inside = False
    x = point["x"]
    z = point["z"]
    for index, current in enumerate(polygon):
        nxt = polygon[(index + 1) % len(polygon)]
        x1, z1 = current["x"], current["z"]
        x2, z2 = nxt["x"], nxt["z"]

        intersects = ((z1 > z) != (z2 > z)) and (
            x < ((x2 - x1) * (z - z1) / (z2 - z1 + _EPSILON)) + x1
        )
        if intersects:
            inside = not inside
    return inside


def _point_on_segment(point: Waypoint, start: Waypoint, end: Waypoint) -> bool:
    cross = (point["z"] - start["z"]) * (end["x"] - start["x"]) - (
        point["x"] - start["x"]
    ) * (end["z"] - start["z"])
    if abs(cross) > _EPSILON:
        return False

    dot = (point["x"] - start["x"]) * (end["x"] - start["x"]) + (
        point["z"] - start["z"]
    ) * (end["z"] - start["z"])
    if dot < -_EPSILON:
        return False

    sq_len = (end["x"] - start["x"]) ** 2 + (end["z"] - start["z"]) ** 2
    if dot - sq_len > _EPSILON:
        return False
    return True


def _centroid(polygon: list[Waypoint]) -> Waypoint:
    sum_x = sum(point["x"] for point in polygon)
    sum_z = sum(point["z"] for point in polygon)
    count = max(len(polygon), 1)
    return {"x": sum_x / count, "z": sum_z / count}


def _deduplicate_sequential_points(waypoints: list[Waypoint]) -> list[Waypoint]:
    if not waypoints:
        return []

    deduped = [waypoints[0]]
    for point in waypoints[1:]:
        previous = deduped[-1]
        if (
            abs(previous["x"] - point["x"]) > _EPSILON
            or abs(previous["z"] - point["z"]) > _EPSILON
        ):
            deduped.append(point)
    return deduped


def _clip_polygon_axis_range(
    polygon: list[Waypoint],
    *,
    axis: str,
    lower: float,
    upper: float,
    include_upper: bool,
) -> list[Waypoint]:
    if not polygon:
        return []
    clipped = _clip_polygon_half_plane(
        polygon,
        axis=axis,
        boundary=lower,
        keep_greater=True,
        include_boundary=True,
    )
    if not clipped:
        return []
    clipped = _clip_polygon_half_plane(
        clipped,
        axis=axis,
        boundary=upper,
        keep_greater=False,
        include_boundary=include_upper,
    )
    if not clipped:
        return []
    return _deduplicate_sequential_points(clipped)


def _clip_polygon_half_plane(
    polygon: list[Waypoint],
    *,
    axis: str,
    boundary: float,
    keep_greater: bool,
    include_boundary: bool,
) -> list[Waypoint]:
    if axis not in {"x", "z"}:
        return []
    if not polygon:
        return []

    output: list[Waypoint] = []
    previous = polygon[-1]
    previous_inside = _is_inside_half_plane(
        previous,
        axis=axis,
        boundary=boundary,
        keep_greater=keep_greater,
        include_boundary=include_boundary,
    )

    for current in polygon:
        current_inside = _is_inside_half_plane(
            current,
            axis=axis,
            boundary=boundary,
            keep_greater=keep_greater,
            include_boundary=include_boundary,
        )

        if current_inside:
            if not previous_inside:
                intersection = _segment_axis_intersection(
                    previous, current, axis=axis, boundary=boundary
                )
                if intersection is not None:
                    output.append(intersection)
            output.append({"x": current["x"], "z": current["z"]})
        elif previous_inside:
            intersection = _segment_axis_intersection(
                previous, current, axis=axis, boundary=boundary
            )
            if intersection is not None:
                output.append(intersection)

        previous = current
        previous_inside = current_inside

    return _deduplicate_sequential_points(output)


def _is_inside_half_plane(
    point: Waypoint,
    *,
    axis: str,
    boundary: float,
    keep_greater: bool,
    include_boundary: bool,
) -> bool:
    value = point[axis]
    if keep_greater:
        if include_boundary:
            return value >= boundary - _EPSILON
        return value > boundary + _EPSILON
    if include_boundary:
        return value <= boundary + _EPSILON
    return value < boundary - _EPSILON


def _segment_axis_intersection(
    start: Waypoint, end: Waypoint, *, axis: str, boundary: float
) -> Waypoint | None:
    start_axis = start[axis]
    end_axis = end[axis]
    delta = end_axis - start_axis
    if abs(delta) <= _EPSILON:
        return None
    t = (boundary - start_axis) / delta
    t = max(0.0, min(1.0, t))
    return {
        "x": start["x"] + ((end["x"] - start["x"]) * t),
        "z": start["z"] + ((end["z"] - start["z"]) * t),
    }
