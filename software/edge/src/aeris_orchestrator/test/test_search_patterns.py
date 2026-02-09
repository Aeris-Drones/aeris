from aeris_orchestrator.search_patterns import (
    generate_waypoints,
    point_in_polygon,
    validate_polygon,
)


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
