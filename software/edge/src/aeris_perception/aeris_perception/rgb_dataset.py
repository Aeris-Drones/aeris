"""RGB frame capture and manifest-backed replay for the Halo proof loop."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
from typing import Any, Callable

import numpy as np

from .rgb_ingest import (
    MonotonicTimestampGenerator,
    RgbFrame,
    RgbFrameMetadata,
    RgbSourceConfig,
    RgbSourceError,
)


MANIFEST_VERSION = 1
SUPPORTED_CAPTURE_IMAGE_FORMATS = {"jpeg", "jpg", "png", "ppm"}


@dataclass(frozen=True)
class RgbDatasetCaptureConfig:
    """Configures manifest-backed frame capture for RGB ingest."""

    output_dir: Path
    manifest_path: Path
    image_format: str = "png"
    capture_reason: str = "continuous_capture"

    def __post_init__(self) -> None:
        image_format = self.normalized_image_format
        if image_format not in SUPPORTED_CAPTURE_IMAGE_FORMATS:
            raise RgbSourceError(
                f"unsupported capture image format '{self.image_format}'"
            )
        if not str(self.capture_reason).strip():
            raise RgbSourceError(
                "capture_reason must be configured when capture is enabled"
            )

    @property
    def normalized_image_format(self) -> str:
        return self.image_format.strip().lower()


@dataclass(frozen=True)
class RgbDatasetManifestEntry:
    """One manifest entry describing a captured RGB frame."""

    relative_image_path: str
    capture_reason: str
    metadata: RgbFrameMetadata
    height: int
    width: int
    channels: int
    label: str | None = None
    review: dict[str, Any] | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "manifest_version": MANIFEST_VERSION,
            "relative_image_path": self.relative_image_path,
            "capture_reason": self.capture_reason,
            "label": self.label,
            "review": self.review,
            "height": self.height,
            "width": self.width,
            "channels": self.channels,
            "metadata": self.metadata.to_dict(),
        }

    @classmethod
    def from_dict(
        cls, payload: dict[str, Any], *, line_number: int
    ) -> "RgbDatasetManifestEntry":
        try:
            manifest_version = int(payload["manifest_version"])
            if manifest_version != MANIFEST_VERSION:
                raise RgbSourceError(
                    f"Unsupported RGB dataset manifest version {manifest_version} "
                    f"(expected {MANIFEST_VERSION}) at line {line_number}"
                )
            metadata_payload = payload["metadata"]
            metadata = RgbFrameMetadata(
                source_type=str(metadata_payload["source_type"]),
                source_name=str(metadata_payload["source_name"]),
                source_uri=str(metadata_payload["source_uri"]),
                frame_id=str(metadata_payload["frame_id"]),
                frame_index=int(metadata_payload["frame_index"]),
                monotonic_timestamp_ns=int(metadata_payload["monotonic_timestamp_ns"]),
                source_timestamp_ns=(
                    None
                    if metadata_payload["source_timestamp_ns"] is None
                    else int(metadata_payload["source_timestamp_ns"])
                ),
                replayed=bool(metadata_payload["replayed"]),
            )
            return cls(
                relative_image_path=str(payload["relative_image_path"]),
                capture_reason=str(payload["capture_reason"]),
                label=payload.get("label"),
                review=payload.get("review"),
                height=int(payload["height"]),
                width=int(payload["width"]),
                channels=int(payload["channels"]),
                metadata=metadata,
            )
        except (KeyError, TypeError, ValueError) as error:
            raise RgbSourceError(
                f"Malformed RGB dataset manifest at line {line_number}"
            ) from error


class RgbFrameCaptureWriter:
    """Persists RGB frames plus JSONL manifest entries for replay and evaluation."""

    def __init__(
        self,
        config: RgbDatasetCaptureConfig,
        *,
        image_writer: Callable[[Path, np.ndarray, str], None] | None = None,
    ) -> None:
        self._config = config
        self._image_writer = image_writer or _write_image_file
        self._output_dir = Path(config.output_dir)
        self._manifest_path = Path(config.manifest_path)
        self._frames_dir = self._output_dir / "frames"
        self._ensure_output_layout()

    def capture(
        self,
        frame: RgbFrame,
        *,
        label: str | None = None,
        review: dict[str, Any] | None = None,
    ) -> RgbDatasetManifestEntry:
        _validate_image(frame.image_bgr)

        relative_image_path = Path("frames") / _capture_filename(
            frame.metadata,
            image_format=self._config.normalized_image_format,
        )
        image_path = self._output_dir / relative_image_path
        if image_path.exists():
            raise RgbSourceError(
                f"RGB capture image already exists: '{image_path}'"
            )
        self._image_writer(
            image_path,
            frame.image_bgr,
            self._config.normalized_image_format,
        )

        entry = RgbDatasetManifestEntry(
            relative_image_path=os.path.relpath(
                image_path, start=self._manifest_path.parent
            ),
            capture_reason=self._config.capture_reason,
            metadata=frame.metadata,
            height=int(frame.image_bgr.shape[0]),
            width=int(frame.image_bgr.shape[1]),
            channels=int(frame.image_bgr.shape[2]),
            label=label,
            review=review,
        )
        with self._manifest_path.open("a", encoding="utf-8") as manifest_file:
            manifest_file.write(json.dumps(entry.to_dict(), sort_keys=True))
            manifest_file.write("\n")
        return entry

    def _ensure_output_layout(self) -> None:
        if self._output_dir.exists() and not self._output_dir.is_dir():
            raise RgbSourceError(
                f"RGB capture output directory is not a directory: '{self._output_dir}'"
            )
        if self._manifest_path.exists() and self._manifest_path.is_dir():
            raise RgbSourceError(
                f"RGB dataset manifest path is a directory: '{self._manifest_path}'"
            )
        try:
            self._output_dir.mkdir(parents=True, exist_ok=True)
            self._frames_dir.mkdir(parents=True, exist_ok=True)
            self._manifest_path.parent.mkdir(parents=True, exist_ok=True)
        except OSError as error:
            raise RgbSourceError(
                f"Unable to prepare RGB capture output path '{self._output_dir}'"
            ) from error


class ManifestReplayFrameSource:
    """Replays captured RGB frames from a manifest-backed image sequence."""

    def __init__(
        self,
        *,
        config: RgbSourceConfig,
        entries: list[RgbDatasetManifestEntry],
        manifest_path: Path,
        image_reader: Callable[[Path], np.ndarray] | None = None,
        path_exists: Callable[[Path], bool] | None = None,
        clock_ns: Callable[[], int] | None = None,
    ) -> None:
        self._config = config
        self._entries = list(entries)
        self._manifest_path = manifest_path
        self._image_reader = image_reader or _read_image_file
        self._path_exists = path_exists or Path.exists
        self._index = 0
        self._timestamp_generator = MonotonicTimestampGenerator(
            fallback_clock_ns=clock_ns
        )

    @property
    def config(self) -> RgbSourceConfig:
        return self._config

    def read(self) -> RgbFrame | None:
        if not self._entries:
            return None
        if self._index >= len(self._entries):
            if not self._config.loop_replay:
                return None
            self._index = 0

        entry = self._entries[self._index]
        self._index += 1

        image_path = self._resolve_image_path(entry)
        image = self._image_reader(image_path)
        _validate_image(image)

        metadata = entry.metadata
        return RgbFrame(
            image_bgr=image,
            metadata=RgbFrameMetadata(
                source_type=metadata.source_type,
                source_name=metadata.source_name,
                source_uri=metadata.source_uri,
                frame_id=self._config.frame_id,
                frame_index=metadata.frame_index,
                monotonic_timestamp_ns=self._timestamp_generator.normalize(
                    metadata.monotonic_timestamp_ns
                ),
                source_timestamp_ns=metadata.source_timestamp_ns,
                replayed=True,
            ),
        )

    def close(self) -> None:
        return None

    def _resolve_image_path(self, entry: RgbDatasetManifestEntry) -> Path:
        image_path = (self._manifest_path.parent / entry.relative_image_path).resolve()
        if not self._path_exists(image_path):
            raise RgbSourceError(
                f"RGB dataset capture file does not exist: '{image_path}'"
            )
        return image_path


def build_manifest_replay_frame_source(
    config: RgbSourceConfig,
    *,
    path_exists: Callable[[Path], bool] | None = None,
    clock_ns: Callable[[], int] | None = None,
) -> ManifestReplayFrameSource:
    """Build a manifest-backed RGB replay source."""

    manifest_path = Path(config.source_uri)
    exists = path_exists or Path.exists
    if not exists(manifest_path):
        raise RgbSourceError(
            f"RGB dataset manifest path does not exist: '{config.source_uri}'"
        )

    entries = load_rgb_dataset_manifest(manifest_path)
    if not entries:
        raise RgbSourceError(
            f"RGB dataset manifest has no capture entries: '{config.source_uri}'"
        )
    for entry in entries:
        image_path = manifest_path.parent / entry.relative_image_path
        if not exists(image_path):
            raise RgbSourceError(
                f"RGB dataset capture file does not exist: '{image_path}'"
            )

    return ManifestReplayFrameSource(
        config=config,
        entries=entries,
        manifest_path=manifest_path,
        path_exists=exists,
        clock_ns=clock_ns,
    )


def load_rgb_dataset_manifest(manifest_path: Path) -> list[RgbDatasetManifestEntry]:
    """Load and validate JSONL manifest entries."""

    entries: list[RgbDatasetManifestEntry] = []
    try:
        with manifest_path.open("r", encoding="utf-8") as manifest_file:
            for line_number, raw_line in enumerate(manifest_file, start=1):
                line = raw_line.strip()
                if not line:
                    continue
                try:
                    payload = json.loads(line)
                except json.JSONDecodeError as error:
                    raise RgbSourceError(
                        f"Malformed RGB dataset manifest at line {line_number}"
                    ) from error
                entries.append(
                    RgbDatasetManifestEntry.from_dict(
                        payload, line_number=line_number
                    )
                )
    except OSError as error:
        raise RgbSourceError(
            f"Unable to read RGB dataset manifest '{manifest_path}'"
        ) from error

    return entries


def _capture_filename(metadata: RgbFrameMetadata, *, image_format: str) -> str:
    source_slug = _slugify(metadata.source_name or metadata.frame_id)
    return (
        f"{source_slug}_{metadata.frame_index:06d}_"
        f"{metadata.monotonic_timestamp_ns}.{image_format}"
    )


def _slugify(value: str) -> str:
    normalized = re.sub(r"[^a-zA-Z0-9_-]+", "_", value.strip())
    return normalized.strip("_") or "frame"


def _validate_image(image: np.ndarray) -> None:
    if image.dtype != np.uint8:
        raise RgbSourceError("captured RGB frames must be uint8 BGR images")
    if image.ndim != 3 or image.shape[2] != 3:
        raise RgbSourceError("captured RGB frames must use HxWx3 BGR layout")


def _write_image_file(path: Path, image_bgr: np.ndarray, image_format: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if image_format == "ppm":
        _write_ppm(path, image_bgr)
        return

    try:
        import cv2
    except ImportError as error:  # pragma: no cover - depends on runtime image
        raise RgbSourceError(
            "OpenCV is required for non-PPM RGB capture formats"
        ) from error

    if not cv2.imwrite(str(path), image_bgr):
        raise RgbSourceError(f"Unable to write RGB capture image '{path}'")


def _read_image_file(path: Path) -> np.ndarray:
    if path.suffix.lower() == ".ppm":
        return _read_ppm(path)

    try:
        import cv2
    except ImportError as error:  # pragma: no cover - depends on runtime image
        raise RgbSourceError(
            "OpenCV is required for non-PPM RGB replay formats"
        ) from error

    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise RgbSourceError(f"Unable to read RGB capture image '{path}'")
    return image


def _write_ppm(path: Path, image_bgr: np.ndarray) -> None:
    _validate_image(image_bgr)
    image_rgb = image_bgr[:, :, ::-1]
    height, width = image_rgb.shape[:2]
    header = f"P6\n{width} {height}\n255\n".encode("ascii")
    try:
        with path.open("wb") as output_file:
            output_file.write(header)
            output_file.write(image_rgb.tobytes())
    except OSError as error:
        raise RgbSourceError(f"Unable to write RGB capture image '{path}'") from error


def _read_ppm(path: Path) -> np.ndarray:
    try:
        data = path.read_bytes()
    except OSError as error:
        raise RgbSourceError(f"Unable to read RGB capture image '{path}'") from error

    parts = data.split(b"\n", 3)
    if len(parts) != 4 or parts[0] != b"P6":
        raise RgbSourceError(f"Unable to read RGB capture image '{path}'")

    try:
        width_text, height_text = parts[1].split()
        width = int(width_text)
        height = int(height_text)
        max_value = int(parts[2])
    except ValueError as error:
        raise RgbSourceError(f"Unable to read RGB capture image '{path}'") from error

    if max_value != 255:
        raise RgbSourceError(f"Unable to read RGB capture image '{path}'")

    expected_size = width * height * 3
    payload = parts[3]
    if len(payload) != expected_size:
        raise RgbSourceError(f"Unable to read RGB capture image '{path}'")

    image_rgb = np.frombuffer(payload, dtype=np.uint8).reshape((height, width, 3))
    return image_rgb[:, :, ::-1].copy()
