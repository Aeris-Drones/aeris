import os
import time

import numpy as np
import pytest

from aeris_perception.thermal_detection import (
    ThermalDetectionConfig,
    ThermalHotspotDetector,
)


def test_detector_core_processing_is_fast_enough_for_target_rate() -> None:
    if os.getenv("AERIS_RUN_PERF_TESTS", "").lower() not in {"1", "true", "yes"}:
        pytest.skip("Set AERIS_RUN_PERF_TESTS=1 to run detector throughput checks.")

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

    # Detector-core benchmark only; end-to-end ROS throughput is validated
    # separately with simulation smoke tooling.
    assert fps >= 5.0
