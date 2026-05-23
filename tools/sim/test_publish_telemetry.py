from tools.sim.publish_telemetry import compute_remaining_flight_time_seconds


def test_compute_remaining_flight_time_seconds_uses_explicit_unavailable_sentinel():
    assert compute_remaining_flight_time_seconds(42.0, 0.45, False) == -1.0


def test_compute_remaining_flight_time_seconds_uses_battery_drain_rate_when_available():
    assert compute_remaining_flight_time_seconds(45.0, 0.45, True) == 100.0


def test_compute_remaining_flight_time_seconds_guards_invalid_drain_rates():
    assert compute_remaining_flight_time_seconds(45.0, 0.0, True) == -1.0
    assert compute_remaining_flight_time_seconds(45.0, -0.3, True) == -1.0
