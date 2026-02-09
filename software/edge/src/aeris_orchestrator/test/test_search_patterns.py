import pytest

from aeris_orchestrator.search_patterns import (
    generate_waypoints,
    partition_polygon_for_scouts,
    point_in_polygon,
    validate_polygon,
)
from aeris_orchestrator.vehicle_ids import normalize_vehicle_id


def _rectangle_polygon() -> list[dict[str, float]]:
    return [
        {"x": 0.0, "z": 0.0},
        {"x": 20.0, "z": 0.0},
        {"x": 20.0, "z": 10.0},
        {"x": 0.0, "z": 10.0},
    ]


def test_validate_polygon_rejects_too_few_points() -> None:
    valid, reason = validate_polygon([{"x": 0.0, "z": 0.0}, {"x": 1.0, "z": 1.0}])
    assert not valid
    assert "at least 3 points" in reason


def test_validate_polygon_rejects_zero_area() -> None:
    valid, reason = validate_polygon(
        [
            {"x": 0.0, "z": 0.0},
            {"x": 1.0, "z": 1.0},
            {"x": 2.0, "z": 2.0},
        ]
    )
    assert not valid
    assert "greater than zero" in reason


def test_validate_polygon_rejects_non_finite_points() -> None:
    valid, reason = validate_polygon(
        [
            {"x": 0.0, "z": 0.0},
            {"x": 10.0, "z": 0.0},
            {"x": float("nan"), "z": 5.0},
            {"x": float("inf"), "z": 1.0},
        ]
    )
    assert not valid
    assert "at least 3 points" in reason


def test_lawnmower_waypoints_stay_inside_polygon() -> None:
    polygon = _rectangle_polygon()
    waypoints = generate_waypoints("lawnmower", polygon, lawnmower_track_spacing_m=2.5)
    assert len(waypoints) > 2
    assert all(point_in_polygon(point, polygon) for point in waypoints)


def test_spiral_waypoints_stay_inside_polygon() -> None:
    polygon = _rectangle_polygon()
    waypoints = generate_waypoints("spiral", polygon, spiral_radial_step_m=1.5)
    assert len(waypoints) > 2
    assert all(point_in_polygon(point, polygon) for point in waypoints)


def test_waypoints_are_deterministic_for_same_input() -> None:
    polygon = _rectangle_polygon()
    first = generate_waypoints("lawnmower", polygon, lawnmower_track_spacing_m=2.5)
    second = generate_waypoints("lawnmower", polygon, lawnmower_track_spacing_m=2.5)
    assert first == second


def _polygon_area(polygon: list[dict[str, float]]) -> float:
    if len(polygon) < 3:
        return 0.0
    area = 0.0
    for index, current in enumerate(polygon):
        nxt = polygon[(index + 1) % len(polygon)]
        area += current["x"] * nxt["z"] - nxt["x"] * current["z"]
    return abs(area) / 2.0


def test_partition_polygon_for_scouts_is_deterministic_and_stably_ordered() -> None:
    polygon = _rectangle_polygon()
    first = partition_polygon_for_scouts(polygon, ["scout2", "scout-1"])
    second = partition_polygon_for_scouts(polygon, ["scout2", "scout-1"])

    assert first == second
    assert [part["vehicle_id"] for part in first] == ["scout_1", "scout_2"]


def test_partition_polygon_for_scouts_non_overlap_and_area_coverage() -> None:
    polygon = _rectangle_polygon()
    partitions = partition_polygon_for_scouts(polygon, ["scout1", "scout2"])

    assert len(partitions) == 2
    left = partitions[0]["polygon"]
    right = partitions[1]["polygon"]
    assert left and right
    assert all(point_in_polygon(point, polygon) for point in left)
    assert all(point_in_polygon(point, polygon) for point in right)

    left_max_x = max(point["x"] for point in left)
    right_min_x = min(point["x"] for point in right)
    assert left_max_x <= right_min_x + 1e-6

    total_area = _polygon_area(left) + _polygon_area(right)
    assert total_area == pytest.approx(_polygon_area(polygon), rel=1e-3)


def test_normalize_vehicle_id_applies_shared_rules() -> None:
    assert normalize_vehicle_id(" scout1 ") == "scout_1"
    assert normalize_vehicle_id("RANGER-2") == "ranger_2"
    assert normalize_vehicle_id("alpha__03") == "alpha_03"
    assert normalize_vehicle_id("___") == ""
