"""Gas plume estimation utilities for gas isopleth publishing.

Implements a deterministic, wind-aware plume model inspired by Kernel DM+V/W
behavior: concentration samples drive plume geometry, wind vectors steer plume
orientation, and state is updated incrementally as new samples arrive.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Deque


@dataclass(frozen=True)
class PlumeEstimate:
    """Output geometry for GasIsopleth publication."""

    polygons: list[list[tuple[float, float]]]
    centerline: list[tuple[float, float]]


class GasPlumeModel:
    """Deterministic gas plume estimator using concentration + wind samples."""

    _POLYGON_SCALES = (1.0, 0.72, 0.45)
    _MAX_LENGTH_RATIO = 0.6
    _LENGTH_BASE_M = 2.5
    _LENGTH_FLOOR_M = 2.0
    _MAJOR_SIGMA_SCALE = 2.6
    _WIND_SPEED_SCALE = 1.8
    _CENTER_SHIFT_RATIO = 0.15
    _WIDTH_TO_LENGTH_RATIO = 0.85
    _WIDTH_BASE_M = 1.0
    _WIDTH_FLOOR_M = 0.8
    _MIN_EFFECTIVE_HALF_WIDTH_M = 0.9
    _MINOR_SIGMA_SCALE = 2.0
    _CONCENTRATION_WIDTH_SCALE = 0.35
    _DOWNWIND_STRETCH = 0.40
    _UPWIND_STRETCH = 0.15

    def __init__(
        self,
        *,
        smoothing_window: int = 30,
        plume_resolution: int = 24,
        sample_stale_sec: float = 4.0,
        wind_stale_sec: float = 2.0,
        hold_last_sec: float = 5.0,
        direction_smoothing_alpha: float = 0.35,
        min_wind_speed_mps: float = 0.1,
        max_extent_m: float = 250.0,
        future_tolerance_sec: float = 0.25,
    ) -> None:
        self._smoothing_window = max(3, int(smoothing_window))
        self._plume_resolution = max(8, int(plume_resolution))
        self._sample_stale_sec = max(0.1, float(sample_stale_sec))
        self._wind_stale_sec = max(0.1, float(wind_stale_sec))
        self._hold_last_sec = max(0.0, float(hold_last_sec))
        self._direction_alpha = min(1.0, max(0.0, float(direction_smoothing_alpha)))
        self._min_wind_speed_mps = max(0.0, float(min_wind_speed_mps))
        self._max_extent_m = max(1.0, float(max_extent_m))
        self._future_tolerance_sec = max(0.0, float(future_tolerance_sec))

        self._samples: Deque[tuple[float, float, float, float]] = deque(
            maxlen=self._smoothing_window
        )
        self._wind: tuple[float, float, float] | None = None
        self._prev_direction = (1.0, 0.0)
        self._last_estimate: tuple[PlumeEstimate, float] | None = None

    def add_sample(
        self,
        *,
        x: float,
        y: float,
        concentration: float,
        timestamp_sec: float,
    ) -> None:
        """Add one gas sample in local XY coordinates."""
        if not (
            math.isfinite(x)
            and math.isfinite(y)
            and math.isfinite(concentration)
            and math.isfinite(timestamp_sec)
        ):
            return
        self._samples.append(
            (float(x), float(y), max(0.0, float(concentration)), float(timestamp_sec))
        )

    def set_wind(self, *, vx: float, vy: float, timestamp_sec: float) -> None:
        """Set latest wind vector sample in local XY coordinates."""
        if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(timestamp_sec)):
            return
        self._wind = (float(vx), float(vy), float(timestamp_sec))

    def estimate(self, *, now_sec: float) -> PlumeEstimate | None:
        """Estimate plume geometry for the current time."""
        if not math.isfinite(now_sec):
            return self._held_estimate(now_sec=now_sec)

        fresh = self._fresh_samples(now_sec=now_sec)
        if not fresh:
            return self._held_estimate(now_sec=now_sec)

        weight_sum = sum(sample[2] for sample in fresh)
        if weight_sum <= 1e-9:
            return self._held_estimate(now_sec=now_sec)

        centroid_x = sum(x * concentration for x, _, concentration in fresh) / weight_sum
        centroid_y = sum(y * concentration for _, y, concentration in fresh) / weight_sum

        cov_xx = 0.0
        cov_xy = 0.0
        cov_yy = 0.0
        for x, y, concentration in fresh:
            dx = x - centroid_x
            dy = y - centroid_y
            cov_xx += concentration * dx * dx
            cov_xy += concentration * dx * dy
            cov_yy += concentration * dy * dy
        cov_xx /= weight_sum
        cov_xy /= weight_sum
        cov_yy /= weight_sum

        major_axis, sigma_major, sigma_minor = self._principal_axes(
            cov_xx=cov_xx,
            cov_xy=cov_xy,
            cov_yy=cov_yy,
        )
        target_direction = self._resolve_direction(
            now_sec=now_sec,
            fallback_axis=major_axis,
        )

        if self._dot(target_direction, self._prev_direction) < 0.0:
            target_direction = (-target_direction[0], -target_direction[1])

        smoothed = self._normalize(
            (
                ((1.0 - self._direction_alpha) * self._prev_direction[0])
                + (self._direction_alpha * target_direction[0]),
                ((1.0 - self._direction_alpha) * self._prev_direction[1])
                + (self._direction_alpha * target_direction[1]),
            )
        )
        self._prev_direction = smoothed
        cross_axis = (-smoothed[1], smoothed[0])

        wind_speed = self._fresh_wind_speed(now_sec=now_sec)
        max_concentration = max(sample[2] for sample in fresh)
        half_length = min(
            self._max_extent_m * self._MAX_LENGTH_RATIO,
            max(
                self._LENGTH_FLOOR_M,
                self._LENGTH_BASE_M
                + (sigma_major * self._MAJOR_SIGMA_SCALE)
                + (wind_speed * self._WIND_SPEED_SCALE),
            ),
        )
        half_width = min(
            max(self._MIN_EFFECTIVE_HALF_WIDTH_M, half_length * self._WIDTH_TO_LENGTH_RATIO),
            max(
                self._WIDTH_FLOOR_M,
                self._WIDTH_BASE_M
                + (sigma_minor * self._MINOR_SIGMA_SCALE)
                + (
                    math.sqrt(max_concentration)
                    * self._CONCENTRATION_WIDTH_SCALE
                ),
            ),
        )

        plume_center = (
            centroid_x + (smoothed[0] * (self._CENTER_SHIFT_RATIO * half_length)),
            centroid_y + (smoothed[1] * (self._CENTER_SHIFT_RATIO * half_length)),
        )

        polygons: list[list[tuple[float, float]]] = []
        for scale in self._POLYGON_SCALES:
            polygons.append(
                self._oriented_polygon(
                    center=plume_center,
                    direction=smoothed,
                    cross=cross_axis,
                    half_length=half_length * scale,
                    half_width=half_width * scale,
                )
            )

        centerline = self._centerline(
            center=plume_center,
            direction=smoothed,
            half_length=half_length,
        )

        estimate = PlumeEstimate(polygons=polygons, centerline=centerline)
        self._last_estimate = (estimate, float(now_sec))
        return estimate

    def _fresh_samples(self, *, now_sec: float) -> list[tuple[float, float, float]]:
        fresh: list[tuple[float, float, float]] = []
        for x, y, concentration, ts in self._samples:
            age_sec = now_sec - ts
            if age_sec < -self._future_tolerance_sec:
                continue
            if age_sec > self._sample_stale_sec:
                continue
            if concentration <= 0.0:
                continue
            fresh.append((x, y, concentration))
        return fresh

    def _fresh_wind_speed(self, *, now_sec: float) -> float:
        if self._wind is None:
            return 0.0
        vx, vy, ts = self._wind
        wind_age = now_sec - ts
        if wind_age < -self._future_tolerance_sec:
            return 0.0
        if wind_age > self._wind_stale_sec:
            return 0.0
        speed = math.hypot(vx, vy)
        if not math.isfinite(speed):
            return 0.0
        return speed

    def _resolve_direction(
        self,
        *,
        now_sec: float,
        fallback_axis: tuple[float, float],
    ) -> tuple[float, float]:
        if self._wind is not None:
            vx, vy, ts = self._wind
            wind_age = now_sec - ts
            wind_speed = math.hypot(vx, vy)
            if (
                wind_age >= -self._future_tolerance_sec
                and wind_age <= self._wind_stale_sec
                and wind_speed >= self._min_wind_speed_mps
            ):
                return self._normalize((vx, vy))

        if self._norm(fallback_axis) > 1e-6:
            return self._normalize(fallback_axis)
        return self._prev_direction

    def _principal_axes(
        self,
        *,
        cov_xx: float,
        cov_xy: float,
        cov_yy: float,
    ) -> tuple[tuple[float, float], float, float]:
        if not all(math.isfinite(value) for value in (cov_xx, cov_xy, cov_yy)):
            return self._prev_direction, 1.0, 0.5

        trace = cov_xx + cov_yy
        determinant = (cov_xx * cov_yy) - (cov_xy * cov_xy)
        disc = max(0.0, (trace * trace * 0.25) - determinant)
        root = math.sqrt(disc)

        lambda_major = max(0.0, (trace * 0.5) + root)
        lambda_minor = max(0.0, (trace * 0.5) - root)

        if abs(cov_xy) > 1e-9:
            axis = (lambda_major - cov_yy, cov_xy)
        elif cov_xx >= cov_yy:
            axis = (1.0, 0.0)
        else:
            axis = (0.0, 1.0)

        major_axis = self._normalize(axis)
        sigma_major = math.sqrt(lambda_major)
        sigma_minor = math.sqrt(lambda_minor)
        return major_axis, sigma_major, sigma_minor

    def _oriented_polygon(
        self,
        *,
        center: tuple[float, float],
        direction: tuple[float, float],
        cross: tuple[float, float],
        half_length: float,
        half_width: float,
    ) -> list[tuple[float, float]]:
        points: list[tuple[float, float]] = []
        for index in range(self._plume_resolution):
            theta = (2.0 * math.pi * index) / self._plume_resolution
            forward = math.cos(theta)
            lateral = math.sin(theta)

            if forward >= 0.0:
                long_scale = 1.0 + (self._DOWNWIND_STRETCH * forward)
            else:
                long_scale = 1.0 + (self._UPWIND_STRETCH * forward)

            long_component = half_length * forward * long_scale
            lateral_component = half_width * lateral

            offset_x = (direction[0] * long_component) + (cross[0] * lateral_component)
            offset_y = (direction[1] * long_component) + (cross[1] * lateral_component)

            point_x = self._clip(center[0] + offset_x)
            point_y = self._clip(center[1] + offset_y)
            points.append((point_x, point_y))
        return points

    def _centerline(
        self,
        *,
        center: tuple[float, float],
        direction: tuple[float, float],
        half_length: float,
    ) -> list[tuple[float, float]]:
        points: list[tuple[float, float]] = []
        for scale in (-0.35, 0.05, 0.55, 1.0):
            offset_x = direction[0] * (half_length * scale)
            offset_y = direction[1] * (half_length * scale)
            points.append(
                (
                    self._clip(center[0] + offset_x),
                    self._clip(center[1] + offset_y),
                )
            )
        return points

    def _held_estimate(self, *, now_sec: float) -> PlumeEstimate | None:
        if self._last_estimate is None:
            return None
        estimate, estimate_time = self._last_estimate
        if not math.isfinite(now_sec):
            return estimate
        if now_sec - estimate_time <= self._hold_last_sec:
            return estimate
        return None

    def _clip(self, value: float) -> float:
        return max(-self._max_extent_m, min(self._max_extent_m, value))

    @staticmethod
    def _normalize(vector: tuple[float, float]) -> tuple[float, float]:
        norm = math.hypot(vector[0], vector[1])
        if norm <= 1e-9:
            return (1.0, 0.0)
        return (vector[0] / norm, vector[1] / norm)

    @staticmethod
    def _dot(a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] * b[0]) + (a[1] * b[1])

    @staticmethod
    def _norm(vector: tuple[float, float]) -> float:
        return math.hypot(vector[0], vector[1])
