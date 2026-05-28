from __future__ import annotations

import numpy as np
import pytest

from aeris_perception.rgb_ingest import (
    MonotonicTimestampGenerator,
    RgbFrameSource,
    RgbSourceConfig,
    RgbSourceError,
    build_rgb_frame_source,
)


class _FakeCapture:
    def __init__(
        self,
        reads: list[tuple[bool, np.ndarray | None]],
        *,
        opened: bool = True,
        fps: float = 30.0,
        pos_msec: list[float] | None = None,
    ) -> None:
        self._reads = list(reads)
        self._opened = opened
        self._fps = fps
        self._pos_msec = list(pos_msec or [])
        self.release_calls = 0
        self.set_calls: list[tuple[int, float]] = []

    def isOpened(self) -> bool:
        return self._opened

    def read(self) -> tuple[bool, np.ndarray | None]:
        if not self._reads:
            return False, None
        return self._reads.pop(0)

    def release(self) -> None:
        self.release_calls += 1

    def get(self, prop_id: int) -> float:
        if prop_id == 5:
            return self._fps
        if prop_id == 0:
            if self._pos_msec:
                return self._pos_msec.pop(0)
            return 0.0
        return 0.0

    def set(self, prop_id: int, value: float) -> bool:
        self.set_calls.append((prop_id, value))
        return True


def _sample_frame(fill: int) -> np.ndarray:
    return np.full((4, 6, 3), fill, dtype=np.uint8)


def test_rgb_source_config_requires_uri_for_file_backed_sources() -> None:
    with pytest.raises(RgbSourceError, match="source_uri"):
        RgbSourceConfig(source_type="video_file", source_uri="", frame_id="halo_rgb")

    with pytest.raises(RgbSourceError, match="source_uri"):
        RgbSourceConfig(source_type="replay", source_uri="", frame_id="halo_rgb")


def test_rgb_source_config_rejects_unknown_source_types() -> None:
    with pytest.raises(RgbSourceError, match="unsupported"):
        RgbSourceConfig(source_type="lidar", source_uri="0", frame_id="halo_rgb")


def test_monotonic_timestamp_generator_never_regresses() -> None:
    generator = MonotonicTimestampGenerator()

    first = generator.normalize(200)
    second = generator.normalize(200)
    third = generator.normalize(150)

    assert first == 200
    assert second == 201
    assert third == 202


def test_video_file_source_uses_clock_time_for_monotonic_stamps() -> None:
    capture = _FakeCapture([(True, _sample_frame(10)), (True, _sample_frame(20))])
    clock_values = iter((1_000, 1_000))
    source = RgbFrameSource(
        config=RgbSourceConfig(
            source_type="video_file",
            source_uri="/tmp/halo.mp4",
            frame_id="halo_rgb",
        ),
        capture=capture,
        clock_ns=lambda: next(clock_values),
    )

    first = source.read()
    second = source.read()

    assert first is not None
    assert second is not None
    assert first.metadata.monotonic_timestamp_ns == 1_000
    assert second.metadata.monotonic_timestamp_ns == 1_001
    assert first.metadata.source_timestamp_ns is None
    assert second.metadata.source_timestamp_ns is None


def test_camera_source_returns_none_for_transient_miss_without_exhausting_capture() -> None:
    capture = _FakeCapture(
        [(False, None), (True, _sample_frame(33))],
        opened=True,
    )
    source = RgbFrameSource(
        config=RgbSourceConfig(
            source_type="camera",
            source_uri="0",
            frame_id="halo_rgb",
        ),
        capture=capture,
        clock_ns=lambda: 5_000,
    )

    missed = source.read()
    recovered = source.read()

    assert missed is None
    assert recovered is not None
    assert recovered.metadata.frame_index == 0
    assert recovered.metadata.monotonic_timestamp_ns == 5_000
    assert capture.release_calls == 0


def test_replay_source_uses_capture_timeline_and_loops() -> None:
    first_capture = _FakeCapture(
        [(True, _sample_frame(1)), (False, None)],
        pos_msec=[0.0, 0.0],
        fps=25.0,
    )
    second_capture = _FakeCapture(
        [(True, _sample_frame(2))],
        pos_msec=[0.0],
        fps=25.0,
    )
    captures = iter((first_capture, second_capture))

    source = build_rgb_frame_source(
        RgbSourceConfig(
            source_type="replay",
            source_uri="/tmp/replay.mp4",
            frame_id="halo_rgb",
            loop_replay=True,
        ),
        capture_factory=lambda _target: next(captures),
        path_exists=lambda _path: True,
        clock_ns=lambda: 9_999,
    )

    first = source.read()
    second = source.read()

    assert first is not None
    assert second is not None
    assert first.metadata.replayed is True
    assert first.metadata.source_timestamp_ns == 0
    assert first.metadata.monotonic_timestamp_ns == 0
    assert second.metadata.frame_index == 1
    assert second.metadata.source_timestamp_ns == 0
    assert second.metadata.monotonic_timestamp_ns == 1
    assert first_capture.release_calls == 1


def test_replay_reopen_failure_releases_replacement_capture() -> None:
    first_capture = _FakeCapture(
        [(True, _sample_frame(1)), (False, None)],
        pos_msec=[0.0, 0.0],
        fps=25.0,
    )
    failed_replacement = _FakeCapture([], opened=False)
    captures = iter((first_capture, failed_replacement))

    source = build_rgb_frame_source(
        RgbSourceConfig(
            source_type="replay",
            source_uri="/tmp/replay.mp4",
            frame_id="halo_rgb",
            loop_replay=True,
        ),
        capture_factory=lambda _target: next(captures),
        path_exists=lambda _path: True,
    )

    assert source.read() is not None

    with pytest.raises(RgbSourceError, match="Unable to reopen"):
        source.read()

    assert first_capture.release_calls == 1
    assert failed_replacement.release_calls == 1


def test_build_rgb_frame_source_rejects_missing_file_inputs() -> None:
    with pytest.raises(RgbSourceError, match="does not exist"):
        build_rgb_frame_source(
            RgbSourceConfig(
                source_type="replay",
                source_uri="/tmp/missing.mp4",
                frame_id="halo_rgb",
            ),
            capture_factory=lambda _target: _FakeCapture([]),
            path_exists=lambda _path: False,
        )


def test_build_rgb_frame_source_rejects_unopenable_capture() -> None:
    with pytest.raises(RgbSourceError, match="Unable to open"):
        build_rgb_frame_source(
            RgbSourceConfig(
                source_type="camera",
                source_uri="0",
                frame_id="halo_rgb",
            ),
            capture_factory=lambda _target: _FakeCapture([], opened=False),
        )
