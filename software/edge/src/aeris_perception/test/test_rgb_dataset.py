from __future__ import annotations

import json

import numpy as np
import pytest

from aeris_perception.rgb_ingest import RgbFrame, RgbFrameMetadata, RgbSourceConfig


def _sample_frame(fill: int, *, frame_index: int = 0) -> RgbFrame:
    return RgbFrame(
        image_bgr=np.full((3, 4, 3), fill, dtype=np.uint8),
        metadata=RgbFrameMetadata(
            source_type="camera",
            source_name="halo_front_camera",
            source_uri="0",
            frame_id="halo_rgb_front",
            frame_index=frame_index,
            monotonic_timestamp_ns=1_000_000_000 + frame_index,
            source_timestamp_ns=None,
            replayed=False,
        ),
    )


def test_capture_writer_saves_frame_and_manifest_entry(tmp_path) -> None:
    from aeris_perception.rgb_dataset import (
        RgbDatasetCaptureConfig,
        RgbFrameCaptureWriter,
    )

    output_dir = tmp_path / "capture"
    manifest_path = output_dir / "manifest.jsonl"
    writer = RgbFrameCaptureWriter(
        RgbDatasetCaptureConfig(
            output_dir=output_dir,
            manifest_path=manifest_path,
            image_format="ppm",
            capture_reason="continuous_capture",
        )
    )

    saved_entry = writer.capture(_sample_frame(17))

    assert saved_entry.capture_reason == "continuous_capture"
    assert saved_entry.label is None
    assert saved_entry.review is None
    assert saved_entry.height == 3
    assert saved_entry.width == 4
    assert saved_entry.channels == 3

    saved_image = output_dir / saved_entry.relative_image_path
    assert saved_image.exists()

    manifest_lines = manifest_path.read_text(encoding="utf-8").splitlines()
    assert len(manifest_lines) == 1

    manifest_record = json.loads(manifest_lines[0])
    assert manifest_record["capture_reason"] == "continuous_capture"
    assert manifest_record["label"] is None
    assert manifest_record["review"] is None
    assert manifest_record["height"] == 3
    assert manifest_record["width"] == 4
    assert manifest_record["channels"] == 3
    assert manifest_record["metadata"]["frame_id"] == "halo_rgb_front"
    assert manifest_record["metadata"]["source_name"] == "halo_front_camera"


def test_capture_manifest_path_can_live_outside_output_dir(tmp_path) -> None:
    from aeris_perception.rgb_dataset import (
        RgbDatasetCaptureConfig,
        RgbFrameCaptureWriter,
    )
    from aeris_perception.rgb_ingest import build_rgb_frame_source

    output_dir = tmp_path / "capture"
    manifest_path = tmp_path / "manifests" / "halo.jsonl"
    writer = RgbFrameCaptureWriter(
        RgbDatasetCaptureConfig(
            output_dir=output_dir,
            manifest_path=manifest_path,
            image_format="ppm",
            capture_reason="continuous_capture",
        )
    )
    frame = _sample_frame(19)

    saved_entry = writer.capture(frame)
    replay_source = build_rgb_frame_source(
        RgbSourceConfig(
            source_type="manifest_replay",
            source_uri=str(manifest_path),
            frame_id="halo_rgb_front",
        )
    )
    replayed = replay_source.read()

    assert saved_entry.relative_image_path.startswith("../capture/frames/")
    assert replayed is not None
    assert np.array_equal(replayed.image_bgr, frame.image_bgr)


def test_manifest_replay_rebuilds_frame_sequence(tmp_path) -> None:
    from aeris_perception.rgb_dataset import (
        RgbDatasetCaptureConfig,
        RgbFrameCaptureWriter,
    )
    from aeris_perception.rgb_ingest import build_rgb_frame_source

    output_dir = tmp_path / "capture"
    manifest_path = output_dir / "manifest.jsonl"
    writer = RgbFrameCaptureWriter(
        RgbDatasetCaptureConfig(
            output_dir=output_dir,
            manifest_path=manifest_path,
            image_format="ppm",
            capture_reason="continuous_capture",
        )
    )
    first_frame = _sample_frame(10, frame_index=0)
    second_frame = _sample_frame(20, frame_index=1)
    writer.capture(first_frame)
    writer.capture(second_frame)

    replay_source = build_rgb_frame_source(
        RgbSourceConfig(
            source_type="manifest_replay",
            source_uri=str(manifest_path),
            frame_id="halo_rgb_front",
            loop_replay=False,
        )
    )

    replayed_first = replay_source.read()
    replayed_second = replay_source.read()
    exhausted = replay_source.read()

    assert replayed_first is not None
    assert replayed_second is not None
    assert np.array_equal(replayed_first.image_bgr, first_frame.image_bgr)
    assert np.array_equal(replayed_second.image_bgr, second_frame.image_bgr)
    assert replayed_first.metadata.frame_index == 0
    assert replayed_second.metadata.frame_index == 1
    assert replayed_first.metadata.frame_id == "halo_rgb_front"
    assert replayed_first.metadata.replayed is True
    assert replayed_second.metadata.replayed is True
    assert exhausted is None


def test_manifest_replay_uses_configured_frame_id(tmp_path) -> None:
    from aeris_perception.rgb_dataset import (
        RgbDatasetCaptureConfig,
        RgbFrameCaptureWriter,
    )
    from aeris_perception.rgb_ingest import build_rgb_frame_source

    output_dir = tmp_path / "capture"
    manifest_path = output_dir / "manifest.jsonl"
    writer = RgbFrameCaptureWriter(
        RgbDatasetCaptureConfig(
            output_dir=output_dir,
            manifest_path=manifest_path,
            image_format="ppm",
            capture_reason="continuous_capture",
        )
    )
    writer.capture(_sample_frame(10, frame_index=0))

    replay_source = build_rgb_frame_source(
        RgbSourceConfig(
            source_type="manifest_replay",
            source_uri=str(manifest_path),
            frame_id="halo_replay_frame",
            loop_replay=False,
        )
    )

    replayed = replay_source.read()

    assert replayed is not None
    assert replayed.metadata.frame_id == "halo_replay_frame"


def test_capture_writer_rejects_existing_capture_path(tmp_path) -> None:
    from aeris_perception.rgb_dataset import (
        RgbDatasetCaptureConfig,
        RgbFrameCaptureWriter,
    )
    from aeris_perception.rgb_ingest import RgbSourceError

    output_dir = tmp_path / "capture"
    manifest_path = output_dir / "manifest.jsonl"
    writer = RgbFrameCaptureWriter(
        RgbDatasetCaptureConfig(
            output_dir=output_dir,
            manifest_path=manifest_path,
            image_format="ppm",
            capture_reason="continuous_capture",
        )
    )
    frame = _sample_frame(10, frame_index=0)

    writer.capture(frame)

    with pytest.raises(RgbSourceError, match="already exists"):
        writer.capture(frame)


def test_capture_writer_rejects_non_directory_output_path(tmp_path) -> None:
    from aeris_perception.rgb_dataset import (
        RgbDatasetCaptureConfig,
        RgbFrameCaptureWriter,
    )
    from aeris_perception.rgb_ingest import RgbSourceError

    output_dir = tmp_path / "not_a_directory"
    output_dir.write_text("halo", encoding="utf-8")

    with pytest.raises(RgbSourceError, match="output directory"):
        RgbFrameCaptureWriter(
            RgbDatasetCaptureConfig(
                output_dir=output_dir,
                manifest_path=tmp_path / "manifest.jsonl",
                image_format="ppm",
                capture_reason="continuous_capture",
            )
        )


def test_manifest_replay_rejects_missing_capture_file(tmp_path) -> None:
    from aeris_perception.rgb_ingest import RgbSourceError, build_rgb_frame_source

    manifest_path = tmp_path / "manifest.jsonl"
    manifest_path.write_text(
        json.dumps(
            {
                "manifest_version": 1,
                "relative_image_path": "frames/missing.ppm",
                "capture_reason": "continuous_capture",
                "label": None,
                "review": None,
                "height": 3,
                "width": 4,
                "channels": 3,
                "metadata": _sample_frame(5).metadata.to_dict(),
            }
        )
        + "\n",
        encoding="utf-8",
    )

    with pytest.raises(RgbSourceError, match="capture file does not exist"):
        build_rgb_frame_source(
            RgbSourceConfig(
                source_type="manifest_replay",
                source_uri=str(manifest_path),
                frame_id="halo_rgb_front",
            )
        )


def test_manifest_replay_rejects_empty_manifest(tmp_path) -> None:
    from aeris_perception.rgb_ingest import RgbSourceError, build_rgb_frame_source

    manifest_path = tmp_path / "manifest.jsonl"
    manifest_path.write_text("", encoding="utf-8")

    with pytest.raises(RgbSourceError, match="no capture entries"):
        build_rgb_frame_source(
            RgbSourceConfig(
                source_type="manifest_replay",
                source_uri=str(manifest_path),
                frame_id="halo_rgb_front",
            )
        )


def test_manifest_replay_rejects_malformed_manifest(tmp_path) -> None:
    from aeris_perception.rgb_ingest import RgbSourceError, build_rgb_frame_source

    manifest_path = tmp_path / "manifest.jsonl"
    manifest_path.write_text("{not-json}\n", encoding="utf-8")

    with pytest.raises(RgbSourceError, match="Malformed RGB dataset manifest"):
        build_rgb_frame_source(
            RgbSourceConfig(
                source_type="manifest_replay",
                source_uri=str(manifest_path),
                frame_id="halo_rgb_front",
            )
        )
