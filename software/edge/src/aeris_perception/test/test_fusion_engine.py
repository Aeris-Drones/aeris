import math

from aeris_perception.fusion_engine import (
    FusionConfig,
    FusionEngine,
    NormalizedDetection,
)


def _make_detection(
    *,
    modality: str,
    timestamp_sec: float,
    confidence: float,
    x: float,
    z: float,
    geometry: tuple[tuple[float, float], ...] | None = None,
) -> NormalizedDetection:
    if geometry is None:
        normalized_geometry = ((float(x), float(z)),)
    else:
        normalized_geometry = tuple((float(px), float(pz)) for px, pz in geometry)
    return NormalizedDetection(
        modality=modality,
        timestamp_sec=float(timestamp_sec),
        confidence=float(confidence),
        local_x=float(x),
        local_z=float(z),
        geometry=normalized_geometry,
    )


def test_correlation_upgrades_confidence_when_second_modality_arrives() -> None:
    engine = FusionEngine(config=FusionConfig())
    first = engine.ingest(
        _make_detection(
            modality="thermal",
            timestamp_sec=10.0,
            confidence=0.80,
            x=3.0,
            z=2.0,
        ),
        now_sec=10.0,
    )
    assert first is not None
    assert first.confidence_level == "MEDIUM"
    assert first.source_modalities == ("thermal",)

    second = engine.ingest(
        _make_detection(
            modality="acoustic",
            timestamp_sec=11.0,
            confidence=0.85,
            x=5.0,
            z=3.0,
        ),
        now_sec=11.0,
    )
    assert second is not None
    assert second.confidence_level == "HIGH"
    assert second.candidate_id == first.candidate_id
    assert second.source_modalities == ("acoustic", "thermal")


def test_dedup_does_not_reemit_same_confidence_level_for_same_candidate() -> None:
    engine = FusionEngine(config=FusionConfig())
    emitted = engine.ingest(
        _make_detection(
            modality="thermal",
            timestamp_sec=20.0,
            confidence=0.82,
            x=1.0,
            z=1.0,
        ),
        now_sec=20.0,
    )
    assert emitted is not None
    assert emitted.confidence_level == "MEDIUM"

    duplicate = engine.ingest(
        _make_detection(
            modality="thermal",
            timestamp_sec=20.5,
            confidence=0.86,
            x=1.5,
            z=1.2,
        ),
        now_sec=20.5,
    )
    assert duplicate is None


def test_weak_to_strong_single_modality_upgrade_emits_low_then_medium() -> None:
    engine = FusionEngine(config=FusionConfig())
    low_event = engine.ingest(
        _make_detection(
            modality="thermal",
            timestamp_sec=30.0,
            confidence=0.45,
            x=0.0,
            z=0.0,
        ),
        now_sec=30.0,
    )
    assert low_event is not None
    assert low_event.confidence_level == "LOW"

    medium_event = engine.ingest(
        _make_detection(
            modality="thermal",
            timestamp_sec=30.7,
            confidence=0.80,
            x=0.3,
            z=0.1,
        ),
        now_sec=30.7,
    )
    assert medium_event is not None
    assert medium_event.confidence_level == "MEDIUM"
    assert medium_event.candidate_id == low_event.candidate_id


def test_spatial_gate_and_ttl_split_candidates() -> None:
    config = FusionConfig(spatial_gate_m=4.0, candidate_ttl_sec=2.0)
    engine = FusionEngine(config=config)
    first = engine.ingest(
        _make_detection(
            modality="thermal",
            timestamp_sec=40.0,
            confidence=0.82,
            x=0.0,
            z=0.0,
        ),
        now_sec=40.0,
    )
    assert first is not None

    far = engine.ingest(
        _make_detection(
            modality="acoustic",
            timestamp_sec=40.2,
            confidence=0.90,
            x=50.0,
            z=50.0,
        ),
        now_sec=40.2,
    )
    assert far is not None
    assert far.candidate_id != first.candidate_id

    # Expired candidate should not be reused even if spatially nearby.
    later = engine.ingest(
        _make_detection(
            modality="thermal",
            timestamp_sec=43.5,
            confidence=0.90,
            x=0.5,
            z=0.4,
        ),
        now_sec=43.5,
    )
    assert later is not None
    assert later.candidate_id != first.candidate_id


def test_future_and_stale_samples_are_rejected() -> None:
    engine = FusionEngine(config=FusionConfig(correlation_window_sec=3.0))
    future = engine.ingest(
        _make_detection(
            modality="gas",
            timestamp_sec=100.6,
            confidence=0.9,
            x=1.0,
            z=2.0,
        ),
        now_sec=100.0,
    )
    assert future is None

    stale = engine.ingest(
        _make_detection(
            modality="gas",
            timestamp_sec=90.0,
            confidence=0.9,
            x=1.0,
            z=2.0,
        ),
        now_sec=95.0,
    )
    assert stale is None

    valid = engine.ingest(
        _make_detection(
            modality="gas",
            timestamp_sec=95.0,
            confidence=0.9,
            x=1.0,
            z=2.0,
        ),
        now_sec=95.0,
    )
    assert valid is not None
    assert math.isfinite(valid.local_x)


def test_geometry_is_normalized_to_convex_hull() -> None:
    engine = FusionEngine(config=FusionConfig())
    first = engine.ingest(
        _make_detection(
            modality="thermal",
            timestamp_sec=10.0,
            confidence=0.80,
            x=1.0,
            z=0.0,
            geometry=((-1.0, -1.0), (3.0, -1.0), (3.0, 1.0), (-1.0, 1.0)),
        ),
        now_sec=10.0,
    )
    assert first is not None

    second = engine.ingest(
        _make_detection(
            modality="acoustic",
            timestamp_sec=10.2,
            confidence=0.90,
            x=1.0,
            z=0.0,
            geometry=((0.0, 0.0), (2.0, 0.0)),
        ),
        now_sec=10.2,
    )
    assert second is not None
    assert second.confidence_level == "HIGH"
    assert len(second.geometry) == 4
    assert set(second.geometry) == {(-1.0, -1.0), (3.0, -1.0), (3.0, 1.0), (-1.0, 1.0)}


def test_candidate_pool_is_bounded_under_spike_load() -> None:
    engine = FusionEngine(
        config=FusionConfig(correlation_window_sec=3.0, candidate_ttl_sec=60.0, max_candidates=2)
    )
    for index in range(3):
        emitted = engine.ingest(
            _make_detection(
                modality="thermal",
                timestamp_sec=20.0 + index,
                confidence=0.95,
                x=100.0 * index,
                z=100.0 * index,
            ),
            now_sec=20.0 + index,
        )
        assert emitted is not None
    assert len(engine._candidates) == 2
    assert [candidate.candidate_id for candidate in engine._candidates] == [
        "cand-00003",
        "cand-00002",
    ]
