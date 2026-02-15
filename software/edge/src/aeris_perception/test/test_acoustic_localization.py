import math

import numpy as np

from aeris_perception.acoustic_localization import (
    estimate_acoustic_candidates,
    synthesize_planar_wave,
)


def _array_geometry() -> np.ndarray:
    return np.array(
        [
            [-0.06, -0.03],
            [0.05, -0.04],
            [-0.01, 0.07],
            [0.07, 0.06],
        ],
        dtype=np.float64,
    )


def _circular_delta_deg(a: float, b: float) -> float:
    return abs(((a - b + 180.0) % 360.0) - 180.0)


def test_estimation_wraps_bearing_in_0_to_360_and_hits_near_true_direction() -> None:
    sample_rate_hz = 16000.0
    mic_positions = _array_geometry()
    frame = synthesize_planar_wave(
        mic_positions_m=mic_positions,
        sample_rate_hz=sample_rate_hz,
        source_bearings_deg=[358.0],
        frequencies_hz=[500.0],
        amplitudes=[1.0],
        window_size_samples=4096,
        speed_of_sound_mps=343.0,
    )

    candidates = estimate_acoustic_candidates(
        frame=frame,
        sample_rate_hz=sample_rate_hz,
        mic_positions_m=mic_positions,
        speed_of_sound_mps=343.0,
        min_source_separation_deg=20.0,
        max_sources_per_cycle=2,
    )

    assert candidates
    first = candidates[0]
    assert 0.0 <= first.bearing_deg < 360.0
    assert _circular_delta_deg(first.bearing_deg, 358.0) <= 10.0


def test_confidence_and_snr_outputs_are_deterministic_and_bounded() -> None:
    sample_rate_hz = 16000.0
    mic_positions = _array_geometry()
    frame = synthesize_planar_wave(
        mic_positions_m=mic_positions,
        sample_rate_hz=sample_rate_hz,
        source_bearings_deg=[45.0],
        frequencies_hz=[440.0],
        amplitudes=[1.0],
        window_size_samples=4096,
        speed_of_sound_mps=343.0,
    )

    first = estimate_acoustic_candidates(
        frame=frame,
        sample_rate_hz=sample_rate_hz,
        mic_positions_m=mic_positions,
    )
    second = estimate_acoustic_candidates(
        frame=frame,
        sample_rate_hz=sample_rate_hz,
        mic_positions_m=mic_positions,
    )

    assert first
    assert second
    assert len(first) == len(second)
    for left, right in zip(first, second):
        assert left.bearing_deg == right.bearing_deg
        assert left.confidence == right.confidence
        assert left.snr_db == right.snr_db
        assert 0.0 <= left.confidence <= 1.0
        assert math.isfinite(left.snr_db)
        assert -30.0 <= left.snr_db <= 80.0


def test_multi_source_returns_separated_candidates() -> None:
    sample_rate_hz = 16000.0
    mic_positions = _array_geometry()
    frame = synthesize_planar_wave(
        mic_positions_m=mic_positions,
        sample_rate_hz=sample_rate_hz,
        source_bearings_deg=[30.0, 210.0],
        frequencies_hz=[420.0, 760.0],
        amplitudes=[1.0, 0.85],
        window_size_samples=4096,
        speed_of_sound_mps=343.0,
    )

    candidates = estimate_acoustic_candidates(
        frame=frame,
        sample_rate_hz=sample_rate_hz,
        mic_positions_m=mic_positions,
        min_source_separation_deg=35.0,
        max_sources_per_cycle=2,
    )

    assert len(candidates) >= 2
    first = candidates[0].bearing_deg
    second = candidates[1].bearing_deg
    separation = abs(((first - second + 180.0) % 360.0) - 180.0)
    assert separation >= 35.0
