import numpy as np

from aeris_perception.thermal_detection import (
    ThermalDetectionConfig,
    ThermalHotspotDetector,
)


def _base_frame(height: int = 80, width: int = 100, ambient_c: float = 24.0) -> np.ndarray:
    return np.full((height, width), ambient_c, dtype=np.float32)


def test_extracts_hotspot_with_image_derived_bbox_temp_and_confidence() -> None:
    detector = ThermalHotspotDetector(
        ThermalDetectionConfig(
            threshold_min_c=30.0,
            threshold_max_c=120.0,
            min_hotspot_area_px=20,
            min_temp_delta_c=4.0,
        )
    )
    frame = _base_frame()
    frame[20:35, 40:60] = 47.5

    detections = detector.detect(frame)

    assert len(detections) == 1
    detection = detections[0]
    assert detection.bbox_xyxy == (40, 20, 59, 34)
    assert detection.temp_c == 47.5
    assert 0.0 <= detection.confidence <= 1.0


def test_rejects_small_or_low_delta_components() -> None:
    detector = ThermalHotspotDetector(
        ThermalDetectionConfig(
            threshold_min_c=29.0,
            threshold_max_c=120.0,
            min_hotspot_area_px=25,
            min_temp_delta_c=5.0,
        )
    )
    frame = _base_frame(ambient_c=26.0)
    frame[10:13, 15:19] = 50.0
    frame[40:52, 55:67] = 30.5

    detections = detector.detect(frame)

    assert detections == []


def test_temporal_gate_suppresses_transient_hotspot_like_noise() -> None:
    detector = ThermalHotspotDetector(
        ThermalDetectionConfig(
            threshold_min_c=30.0,
            threshold_max_c=120.0,
            min_hotspot_area_px=20,
            min_temp_delta_c=3.0,
            temporal_iou_gate=0.2,
            temporal_history_size=3,
        )
    )
    stable = _base_frame(ambient_c=25.0)
    stable[18:32, 22:42] = 41.0
    assert len(detector.detect(stable)) == 1

    transient = _base_frame(ambient_c=25.0)
    transient[55:70, 70:86] = 29.0
    detections = detector.detect(transient)
    assert detections == []


def test_detector_output_is_deterministic_for_same_input() -> None:
    config = ThermalDetectionConfig(
        threshold_min_c=30.0,
        threshold_max_c=120.0,
        min_hotspot_area_px=12,
        min_temp_delta_c=2.5,
    )
    frame = _base_frame(ambient_c=23.0)
    frame[8:18, 10:26] = 43.0
    frame[48:62, 60:81] = 45.0

    result_a = ThermalHotspotDetector(config).detect(frame)
    result_b = ThermalHotspotDetector(config).detect(frame)

    assert result_a == result_b
