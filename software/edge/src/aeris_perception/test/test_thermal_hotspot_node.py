import time

import numpy as np

from aeris_perception.thermal_detection import (
    ThermalDetectionConfig,
    ThermalHotspotDetector,
)


def test_hotspot_detection_pipeline_meets_minimum_rate_target() -> None:
    config = ThermalDetectionConfig(
        threshold_min_c=30.0,
        threshold_max_c=120.0,
        min_hotspot_area_px=20,
        min_temp_delta_c=3.0,
    )
    detector = ThermalHotspotDetector(config)
    frame = np.full((120, 160), 23.0, dtype=np.float32)
    frame[35:70, 60:95] = 42.0

    iterations = 150
    start = time.perf_counter()
    for _ in range(iterations):
        detector.detect(frame)
    elapsed = max(time.perf_counter() - start, 1e-6)
    fps = iterations / elapsed

    assert fps >= 5.0
