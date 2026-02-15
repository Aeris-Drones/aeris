"""Deterministic acoustic localization helpers for multichannel microphone arrays."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

_EPSILON = 1e-12


@dataclass(frozen=True)
class AcousticCandidate:
    """Estimated acoustic source direction and quality metrics."""

    bearing_deg: float
    confidence: float
    snr_db: float
    score: float


def normalize_bearing_deg(value: float) -> float:
    """Normalize a bearing to [0, 360)."""
    wrapped = math.fmod(float(value), 360.0)
    if wrapped < 0.0:
        wrapped += 360.0
    if wrapped >= 360.0:
        wrapped -= 360.0
    return float(wrapped)


def _bearing_unit_vector(bearing_deg: float) -> np.ndarray:
    radians = math.radians(float(bearing_deg))
    return np.asarray([math.sin(radians), math.cos(radians)], dtype=np.float64)


def _circular_distance_deg(left: float, right: float) -> float:
    return abs(((left - right + 180.0) % 360.0) - 180.0)


def _pair_indices(channel_count: int) -> list[tuple[int, int]]:
    pairs: list[tuple[int, int]] = []
    for left in range(channel_count):
        for right in range(left + 1, channel_count):
            pairs.append((left, right))
    return pairs


def gcc_phat_delay_seconds(
    signal_a: np.ndarray,
    signal_b: np.ndarray,
    sample_rate_hz: float,
    max_delay_sec: float,
    *,
    interp_factor: int = 8,
) -> float:
    """Estimate TDOA between two channels using GCC-PHAT."""
    if sample_rate_hz <= 0.0:
        return 0.0
    if max_delay_sec <= 0.0:
        return 0.0

    reference = np.asarray(signal_a, dtype=np.float64).reshape(-1)
    target = np.asarray(signal_b, dtype=np.float64).reshape(-1)
    if reference.size == 0 or target.size == 0:
        return 0.0

    if reference.size != target.size:
        length = min(reference.size, target.size)
        reference = reference[:length]
        target = target[:length]

    reference = reference - float(np.mean(reference))
    target = target - float(np.mean(target))

    transform_length = reference.size + target.size
    reference_fft = np.fft.rfft(reference, n=transform_length)
    target_fft = np.fft.rfft(target, n=transform_length)
    cross_power = reference_fft * np.conjugate(target_fft)

    magnitude = np.abs(cross_power)
    cross_power /= np.where(magnitude > _EPSILON, magnitude, 1.0)

    interpolation = max(1, int(interp_factor))
    correlation = np.fft.irfft(cross_power, n=transform_length * interpolation)

    max_shift = (transform_length * interpolation) // 2
    bounded_shift = int(interpolation * sample_rate_hz * max_delay_sec)
    max_shift = max(1, min(max_shift, bounded_shift))

    centered = np.concatenate((correlation[-max_shift:], correlation[: max_shift + 1]))
    delay_index = int(np.argmax(np.abs(centered))) - max_shift
    return float(delay_index) / (float(interpolation) * float(sample_rate_hz))


def _compute_snr_db(frame: np.ndarray) -> float:
    mono = np.mean(frame, axis=0)
    signal_rms = float(np.sqrt(np.mean(np.square(mono)) + _EPSILON))
    residual = frame - mono[np.newaxis, :]
    noise_rms = float(np.sqrt(np.mean(np.square(residual)) + _EPSILON))
    snr_db = 20.0 * math.log10((signal_rms + _EPSILON) / (noise_rms + _EPSILON))
    return float(np.clip(snr_db, -30.0, 80.0))


def _select_candidate_indices(
    scores: np.ndarray,
    angles_deg: np.ndarray,
    min_source_separation_deg: float,
    max_sources_per_cycle: int,
) -> list[int]:
    if scores.size == 0:
        return []

    min_separation = max(1.0, float(min_source_separation_deg))
    max_candidates = max(1, int(max_sources_per_cycle))

    ranked_indices = np.argsort(scores)[::-1]
    selected: list[int] = []
    for candidate_index in ranked_indices:
        candidate_angle = float(angles_deg[candidate_index])
        if any(
            _circular_distance_deg(candidate_angle, float(angles_deg[index]))
            < min_separation
            for index in selected
        ):
            continue
        selected.append(int(candidate_index))
        if len(selected) >= max_candidates:
            break
    return selected


def estimate_acoustic_candidates(
    frame: np.ndarray,
    sample_rate_hz: float,
    mic_positions_m: np.ndarray,
    *,
    speed_of_sound_mps: float = 343.0,
    min_source_separation_deg: float = 25.0,
    max_sources_per_cycle: int = 2,
) -> list[AcousticCandidate]:
    """Estimate one or more acoustic source bearings from a multichannel frame."""
    if sample_rate_hz <= 0.0:
        return []
    if speed_of_sound_mps <= 0.0:
        return []

    channels = np.asarray(frame, dtype=np.float64)
    geometry = np.asarray(mic_positions_m, dtype=np.float64)

    if channels.ndim != 2 or geometry.ndim != 2 or geometry.shape[1] != 2:
        return []
    if channels.shape[0] != geometry.shape[0] or channels.shape[0] < 2:
        return []
    if channels.shape[1] < 16:
        return []
    if not np.all(np.isfinite(channels)):
        return []

    pair_indices = _pair_indices(channels.shape[0])
    if not pair_indices:
        return []

    delays: list[float] = []
    pair_vectors: list[np.ndarray] = []
    for left, right in pair_indices:
        displacement = geometry[right] - geometry[left]
        max_delay_sec = float(np.linalg.norm(displacement) / speed_of_sound_mps)
        delay_sec = gcc_phat_delay_seconds(
            channels[left],
            channels[right],
            sample_rate_hz,
            max_delay_sec,
        )
        delays.append(delay_sec)
        pair_vectors.append(displacement)

    delay_vector = np.asarray(delays, dtype=np.float64)
    vectors = np.asarray(pair_vectors, dtype=np.float64)

    angles_deg = np.arange(0.0, 360.0, 1.0, dtype=np.float64)
    sigma = max(1.0 / float(sample_rate_hz), 1e-5)
    scores = np.zeros_like(angles_deg)

    for index, angle_deg in enumerate(angles_deg):
        direction = _bearing_unit_vector(float(angle_deg))
        expected_delays = (vectors @ direction) / float(speed_of_sound_mps)
        delay_error = delay_vector - expected_delays
        scores[index] = float(
            np.sum(np.exp(-0.5 * np.square(delay_error / sigma)))
        )

    selected_indices = _select_candidate_indices(
        scores=scores,
        angles_deg=angles_deg,
        min_source_separation_deg=min_source_separation_deg,
        max_sources_per_cycle=max_sources_per_cycle,
    )
    if not selected_indices:
        return []

    score_floor = float(np.percentile(scores, 25.0))
    score_span = max(float(np.max(scores) - score_floor), _EPSILON)
    snr_db = _compute_snr_db(channels)

    candidates: list[AcousticCandidate] = []
    for selected_index in selected_indices:
        angle_deg = float(angles_deg[selected_index])
        direction = _bearing_unit_vector(angle_deg)
        expected_delays = (vectors @ direction) / float(speed_of_sound_mps)
        delay_error = delay_vector - expected_delays
        residual_rms = float(np.sqrt(np.mean(np.square(delay_error))))

        relative_score = float(
            np.clip((float(scores[selected_index]) - score_floor) / score_span, 0.0, 1.0)
        )
        coherence = float(np.exp(-0.5 * np.square(residual_rms / (sigma * 3.0))))
        confidence = float(np.clip(0.15 + (0.85 * relative_score * coherence), 0.0, 1.0))

        candidates.append(
            AcousticCandidate(
                bearing_deg=normalize_bearing_deg(angle_deg),
                confidence=confidence,
                snr_db=snr_db,
                score=float(scores[selected_index]),
            )
        )

    candidates.sort(key=lambda candidate: candidate.score, reverse=True)
    return candidates


def synthesize_planar_wave(
    *,
    mic_positions_m: np.ndarray,
    sample_rate_hz: float,
    source_bearings_deg: Sequence[float],
    frequencies_hz: Sequence[float],
    amplitudes: Sequence[float],
    window_size_samples: int,
    speed_of_sound_mps: float,
    time_offset_sec: float = 0.0,
) -> np.ndarray:
    """Generate deterministic multichannel audio with one or more far-field sources."""
    if sample_rate_hz <= 0.0:
        raise ValueError("sample_rate_hz must be positive")
    if speed_of_sound_mps <= 0.0:
        raise ValueError("speed_of_sound_mps must be positive")

    geometry = np.asarray(mic_positions_m, dtype=np.float64)
    if geometry.ndim != 2 or geometry.shape[1] != 2 or geometry.shape[0] < 2:
        raise ValueError("mic_positions_m must be shape (channels, 2)")

    sample_count = max(16, int(window_size_samples))
    channel_count = geometry.shape[0]
    if not (
        len(source_bearings_deg)
        == len(frequencies_hz)
        == len(amplitudes)
        and len(source_bearings_deg) > 0
    ):
        raise ValueError("source arrays must have identical non-zero lengths")

    timeline = (
        np.arange(sample_count, dtype=np.float64) / float(sample_rate_hz)
    ) + float(time_offset_sec)
    frame = np.zeros((channel_count, sample_count), dtype=np.float64)

    for bearing_deg, frequency_hz, amplitude in zip(
        source_bearings_deg, frequencies_hz, amplitudes
    ):
        direction = _bearing_unit_vector(float(bearing_deg))
        delays = (geometry @ direction) / float(speed_of_sound_mps)
        angular_rate = 2.0 * math.pi * float(frequency_hz)
        source_amplitude = float(amplitude)
        for channel_index in range(channel_count):
            phase = angular_rate * (timeline - delays[channel_index])
            frame[channel_index, :] += source_amplitude * np.sin(phase)

    if len(source_bearings_deg) > 1:
        frame *= 0.8
    return frame.astype(np.float32)
