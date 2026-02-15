"""Thermal hotspot extraction and suppression utilities."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque

import numpy as np

try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover - optional acceleration path
    cv2 = None


@dataclass(frozen=True)
class ThermalDetectionConfig:
    """Configuration for thermal hotspot extraction and filtering."""

    threshold_min_c: float = 30.0
    threshold_max_c: float = 120.0
    min_hotspot_area_px: int = 25
    min_temp_delta_c: float = 4.0
    min_aspect_ratio: float = 0.2
    max_aspect_ratio: float = 5.0
    temporal_iou_gate: float = 0.2
    temporal_history_size: int = 4
    max_hotspots_per_frame: int = 5
    local_background_margin_px: int = 6


@dataclass(frozen=True)
class ThermalDetection:
    """Single thermal hotspot derived from image processing."""

    bbox_xyxy: tuple[int, int, int, int]
    temp_c: float
    confidence: float


@dataclass(frozen=True)
class _Component:
    bbox_xyxy: tuple[int, int, int, int]
    area_px: int
    local_mask: np.ndarray


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _bbox_iou(lhs: tuple[int, int, int, int], rhs: tuple[int, int, int, int]) -> float:
    x0 = max(lhs[0], rhs[0])
    y0 = max(lhs[1], rhs[1])
    x1 = min(lhs[2], rhs[2])
    y1 = min(lhs[3], rhs[3])
    if x1 < x0 or y1 < y0:
        return 0.0
    intersection = float((x1 - x0 + 1) * (y1 - y0 + 1))
    lhs_area = float((lhs[2] - lhs[0] + 1) * (lhs[3] - lhs[1] + 1))
    rhs_area = float((rhs[2] - rhs[0] + 1) * (rhs[3] - rhs[1] + 1))
    union = lhs_area + rhs_area - intersection
    if union <= 0.0:
        return 0.0
    return intersection / union


def _extract_components_with_cv2(mask: np.ndarray) -> list[_Component]:
    mask_u8 = mask.astype(np.uint8)
    labels_count, labels, stats, _centroids = cv2.connectedComponentsWithStats(
        mask_u8, connectivity=8
    )
    components: list[_Component] = []
    for label in range(1, labels_count):
        x = int(stats[label, cv2.CC_STAT_LEFT])
        y = int(stats[label, cv2.CC_STAT_TOP])
        width = int(stats[label, cv2.CC_STAT_WIDTH])
        height = int(stats[label, cv2.CC_STAT_HEIGHT])
        area = int(stats[label, cv2.CC_STAT_AREA])
        if width <= 0 or height <= 0 or area <= 0:
            continue
        region = labels[y : y + height, x : x + width]
        components.append(
            _Component(
                bbox_xyxy=(x, y, x + width - 1, y + height - 1),
                area_px=area,
                local_mask=(region == label),
            )
        )
    return components


def _extract_components_without_cv2(mask: np.ndarray) -> list[_Component]:
    height, width = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    components: list[_Component] = []
    neighbors = (
        (-1, -1),
        (0, -1),
        (1, -1),
        (-1, 0),
        (1, 0),
        (-1, 1),
        (0, 1),
        (1, 1),
    )

    for y_start in range(height):
        for x_start in range(width):
            if not mask[y_start, x_start] or visited[y_start, x_start]:
                continue

            queue = deque([(x_start, y_start)])
            visited[y_start, x_start] = True
            pixels: list[tuple[int, int]] = []
            min_x = max_x = x_start
            min_y = max_y = y_start

            while queue:
                x, y = queue.popleft()
                pixels.append((x, y))
                min_x = min(min_x, x)
                min_y = min(min_y, y)
                max_x = max(max_x, x)
                max_y = max(max_y, y)
                for dx, dy in neighbors:
                    nx = x + dx
                    ny = y + dy
                    if nx < 0 or ny < 0 or nx >= width or ny >= height:
                        continue
                    if visited[ny, nx] or not mask[ny, nx]:
                        continue
                    visited[ny, nx] = True
                    queue.append((nx, ny))

            local_h = max_y - min_y + 1
            local_w = max_x - min_x + 1
            local_mask = np.zeros((local_h, local_w), dtype=bool)
            for px, py in pixels:
                local_mask[py - min_y, px - min_x] = True

            components.append(
                _Component(
                    bbox_xyxy=(min_x, min_y, max_x, max_y),
                    area_px=len(pixels),
                    local_mask=local_mask,
                )
            )
    return components


def _extract_components(mask: np.ndarray) -> list[_Component]:
    if cv2 is not None:
        return _extract_components_with_cv2(mask)
    return _extract_components_without_cv2(mask)


class ThermalHotspotDetector:
    """Detect thermal hotspots with deterministic filtering heuristics."""

    def __init__(self, config: ThermalDetectionConfig | None = None) -> None:
        self._config = config or ThermalDetectionConfig()
        history_size = max(1, int(self._config.temporal_history_size))
        self._history: Deque[list[tuple[int, int, int, int]]] = deque(maxlen=history_size)

    @property
    def config(self) -> ThermalDetectionConfig:
        return self._config

    def update_config(self, config: ThermalDetectionConfig) -> None:
        self._config = config
        history_size = max(1, int(config.temporal_history_size))
        if self._history.maxlen != history_size:
            self._history = deque(self._history, maxlen=history_size)

    def detect(self, frame_celsius: np.ndarray) -> list[ThermalDetection]:
        frame = np.asarray(frame_celsius, dtype=np.float32)
        if frame.ndim != 2 or frame.size == 0:
            self._history.append([])
            return []

        cfg = self._config
        mask = np.logical_and(frame >= cfg.threshold_min_c, frame <= cfg.threshold_max_c)
        if not np.any(mask):
            self._history.append([])
            return []

        previous_boxes = [
            box for frame_boxes in self._history for box in frame_boxes
        ]

        candidates: list[ThermalDetection] = []
        for component in _extract_components(mask):
            if component.area_px < cfg.min_hotspot_area_px:
                continue
            x0, y0, x1, y1 = component.bbox_xyxy
            width = max(1, x1 - x0 + 1)
            height = max(1, y1 - y0 + 1)
            aspect_ratio = float(width) / float(height)
            if aspect_ratio < cfg.min_aspect_ratio or aspect_ratio > cfg.max_aspect_ratio:
                continue

            local_region = frame[y0 : y1 + 1, x0 : x1 + 1]
            hotspot_pixels = local_region[component.local_mask]
            if hotspot_pixels.size == 0:
                continue

            hotspot_peak = float(np.max(hotspot_pixels))
            hotspot_mean = float(np.mean(hotspot_pixels))

            margin = max(1, int(cfg.local_background_margin_px))
            bg_x0 = max(0, x0 - margin)
            bg_y0 = max(0, y0 - margin)
            bg_x1 = min(frame.shape[1] - 1, x1 + margin)
            bg_y1 = min(frame.shape[0] - 1, y1 + margin)

            background_window = frame[bg_y0 : bg_y1 + 1, bg_x0 : bg_x1 + 1]
            exclusion_mask = np.zeros_like(background_window, dtype=bool)
            offset_x = x0 - bg_x0
            offset_y = y0 - bg_y0
            exclusion_mask[
                offset_y : offset_y + component.local_mask.shape[0],
                offset_x : offset_x + component.local_mask.shape[1],
            ] = component.local_mask

            background_pixels = background_window[~exclusion_mask]
            if background_pixels.size == 0:
                background_temp = float(np.median(frame))
            else:
                background_temp = float(np.median(background_pixels))

            temp_delta = hotspot_peak - background_temp
            if temp_delta < cfg.min_temp_delta_c:
                continue

            max_iou = 0.0
            for previous in previous_boxes:
                max_iou = max(max_iou, _bbox_iou(component.bbox_xyxy, previous))

            if previous_boxes and max_iou < cfg.temporal_iou_gate:
                # Keep stable detections, suppress one-off spatial outliers unless they are
                # very strong thermal deltas compared to local background.
                if temp_delta < (cfg.min_temp_delta_c * 1.5):
                    continue

            area_score = _clamp(
                component.area_px / float(max(cfg.min_hotspot_area_px * 4, 1)), 0.0, 1.0
            )
            fill_ratio = component.area_px / float(max(width * height, 1))
            fill_score = _clamp(fill_ratio, 0.0, 1.0)
            delta_score = _clamp(
                temp_delta / float(max(cfg.min_temp_delta_c * 3.0, 1.0)), 0.0, 1.0
            )
            thermal_span = max(cfg.threshold_max_c - cfg.threshold_min_c, 1.0)
            temp_score = _clamp(
                (hotspot_mean - cfg.threshold_min_c) / thermal_span, 0.0, 1.0
            )
            stability_score = max_iou if previous_boxes else 1.0
            confidence = _clamp(
                (0.30 * temp_score)
                + (0.25 * delta_score)
                + (0.20 * area_score)
                + (0.15 * fill_score)
                + (0.10 * stability_score),
                0.0,
                1.0,
            )
            candidates.append(
                ThermalDetection(
                    bbox_xyxy=component.bbox_xyxy,
                    temp_c=round(hotspot_peak, 3),
                    confidence=round(confidence, 4),
                )
            )

        candidates.sort(key=lambda item: (-item.confidence, item.bbox_xyxy))
        limited = candidates[: max(1, int(cfg.max_hotspots_per_frame))]
        self._history.append([item.bbox_xyxy for item in limited])
        return limited
