import math

from aeris_perception.gas_plume_model import GasPlumeModel


def _direction(centerline: list[tuple[float, float]]) -> tuple[float, float]:
    start_x, start_y = centerline[0]
    end_x, end_y = centerline[-1]
    dx = end_x - start_x
    dy = end_y - start_y
    norm = math.hypot(dx, dy)
    if norm <= 1e-9:
        return (1.0, 0.0)
    return (dx / norm, dy / norm)


def _dot(a: tuple[float, float], b: tuple[float, float]) -> float:
    return (a[0] * b[0]) + (a[1] * b[1])


def test_model_emits_finite_polygons_and_wind_aligned_centerline() -> None:
    model = GasPlumeModel(
        smoothing_window=20,
        plume_resolution=24,
        sample_stale_sec=4.0,
        wind_stale_sec=2.0,
    )
    t0 = 50.0
    for index in range(8):
        model.add_sample(
            x=-2.0 + index * 0.4,
            y=1.5,
            concentration=2.0 + index * 0.1,
            timestamp_sec=t0 + (index * 0.05),
        )
    model.set_wind(vx=3.5, vy=0.0, timestamp_sec=t0 + 0.2)

    estimate = model.estimate(now_sec=t0 + 0.3)

    assert estimate is not None
    assert len(estimate.polygons) == 3
    assert all(len(polygon) == 24 for polygon in estimate.polygons)
    assert all(
        math.isfinite(coord)
        for polygon in estimate.polygons
        for point in polygon
        for coord in point
    )
    direction = _direction(estimate.centerline)
    assert _dot(direction, (1.0, 0.0)) >= 0.8


def test_model_refines_centerline_position_with_new_samples() -> None:
    model = GasPlumeModel(
        smoothing_window=30,
        plume_resolution=18,
        sample_stale_sec=5.0,
        wind_stale_sec=2.0,
    )
    t0 = 120.0
    model.set_wind(vx=0.0, vy=2.5, timestamp_sec=t0)

    for x in (-1.0, 0.0, 1.0):
        model.add_sample(x=x, y=0.0, concentration=2.0, timestamp_sec=t0)

    first = model.estimate(now_sec=t0 + 0.1)
    assert first is not None

    for x in (-1.0, 0.0, 1.0):
        model.add_sample(x=x, y=6.0, concentration=3.0, timestamp_sec=t0 + 0.2)

    second = model.estimate(now_sec=t0 + 0.3)
    assert second is not None

    first_mid_y = sum(point[1] for point in first.centerline) / len(first.centerline)
    second_mid_y = sum(point[1] for point in second.centerline) / len(second.centerline)
    assert second_mid_y > first_mid_y + 1.0


def test_model_handles_stale_wind_with_direction_fallback_and_hold_limit() -> None:
    model = GasPlumeModel(
        smoothing_window=12,
        plume_resolution=16,
        sample_stale_sec=0.6,
        wind_stale_sec=0.4,
        hold_last_sec=1.5,
    )
    t0 = 10.0
    model.set_wind(vx=2.0, vy=0.0, timestamp_sec=t0)
    model.add_sample(x=0.0, y=0.0, concentration=1.0, timestamp_sec=t0)
    fresh = model.estimate(now_sec=t0 + 0.1)
    assert fresh is not None

    # Wind and samples are stale now, but estimate should be held briefly.
    held = model.estimate(now_sec=t0 + 1.0)
    assert held is not None
    held_dir = _direction(held.centerline)
    assert held_dir[0] > 0.0

    expired = model.estimate(now_sec=t0 + 2.5)
    assert expired is None


def test_model_rejects_samples_and_wind_far_in_the_future() -> None:
    model = GasPlumeModel(
        smoothing_window=10,
        plume_resolution=16,
        sample_stale_sec=3.0,
        wind_stale_sec=2.0,
        future_tolerance_sec=0.2,
    )
    t0 = 25.0

    # Far-future timestamps should not be treated as valid fresh inputs.
    model.add_sample(x=0.0, y=0.0, concentration=2.0, timestamp_sec=t0 + 5.0)
    model.set_wind(vx=2.0, vy=0.0, timestamp_sec=t0 + 5.0)
    assert model.estimate(now_sec=t0 + 0.1) is None

    # In-window timestamps should produce a normal estimate.
    model.add_sample(x=0.5, y=0.2, concentration=2.3, timestamp_sec=t0 + 0.1)
    model.set_wind(vx=2.0, vy=0.0, timestamp_sec=t0 + 0.1)
    estimate = model.estimate(now_sec=t0 + 0.2)
    assert estimate is not None
