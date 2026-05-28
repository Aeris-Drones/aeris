"""RGB ingest contract and source adapters for the Halo perception path."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time
from typing import Any, Callable, Protocol

import numpy as np


CAP_PROP_POS_MSEC = 0
CAP_PROP_POS_FRAMES = 1
CAP_PROP_FPS = 5

SUPPORTED_RGB_SOURCE_TYPES = {"camera", "video_file", "replay", "manifest_replay"}


class RgbSourceError(RuntimeError):
    """Raised when an RGB source cannot be configured or read safely."""


class CaptureLike(Protocol):
    def isOpened(self) -> bool:
        pass

    def read(self) -> tuple[bool, np.ndarray | None]:
        pass

    def release(self) -> None:
        pass

    def get(self, prop_id: int) -> float:
        pass

    def set(self, prop_id: int, value: float) -> bool:
        pass


class RgbReadableSource(Protocol):
    @property
    def config(self) -> "RgbSourceConfig":
        pass

    def read(self) -> "RgbFrame | None":
        pass

    def close(self) -> None:
        pass


@dataclass(frozen=True)
class RgbSourceConfig:
    """Configuration for one RGB ingest source."""

    source_type: str
    source_uri: str
    frame_id: str
    source_name: str = ""
    loop_replay: bool = False

    def __post_init__(self) -> None:
        normalized_type = self.source_type.strip().lower()
        if normalized_type not in SUPPORTED_RGB_SOURCE_TYPES:
            raise RgbSourceError(
                f"unsupported RGB source type '{self.source_type}'"
            )
        if not self.frame_id.strip():
            raise RgbSourceError("frame_id must be configured for RGB ingest")
        if not self.source_uri.strip():
            raise RgbSourceError(
                "source_uri must be configured for the selected RGB source"
            )

    @property
    def normalized_source_type(self) -> str:
        return self.source_type.strip().lower()

    @property
    def capture_target(self) -> int | str:
        target = self.source_uri.strip()
        if self.normalized_source_type == "camera" and target.isdigit():
            return int(target)
        return target

    @property
    def effective_source_name(self) -> str:
        if self.source_name.strip():
            return self.source_name.strip()
        target = self.source_uri.strip()
        if not target:
            return self.normalized_source_type
        return Path(target).name or target


@dataclass(frozen=True)
class RgbFrameMetadata:
    """Metadata carried alongside a decoded RGB frame."""

    source_type: str
    source_name: str
    source_uri: str
    frame_id: str
    frame_index: int
    monotonic_timestamp_ns: int
    source_timestamp_ns: int | None
    replayed: bool

    def to_dict(self) -> dict[str, Any]:
        return {
            "source_type": self.source_type,
            "source_name": self.source_name,
            "source_uri": self.source_uri,
            "frame_id": self.frame_id,
            "frame_index": self.frame_index,
            "monotonic_timestamp_ns": self.monotonic_timestamp_ns,
            "source_timestamp_ns": self.source_timestamp_ns,
            "replayed": self.replayed,
        }


@dataclass(frozen=True)
class RgbFrame:
    """Decoded RGB frame plus ingest metadata for downstream Halo stages."""

    image_bgr: np.ndarray
    metadata: RgbFrameMetadata


class MonotonicTimestampGenerator:
    """Normalizes candidate timestamps into a strictly monotonic sequence."""

    def __init__(self, *, fallback_clock_ns: Callable[[], int] | None = None) -> None:
        self._fallback_clock_ns = fallback_clock_ns or time.monotonic_ns
        self._last_timestamp_ns: int | None = None

    def normalize(self, candidate_ns: int | None) -> int:
        if candidate_ns is None:
            candidate_ns = int(self._fallback_clock_ns())
        else:
            candidate_ns = int(candidate_ns)

        if self._last_timestamp_ns is None:
            self._last_timestamp_ns = candidate_ns
            return candidate_ns

        if candidate_ns <= self._last_timestamp_ns:
            candidate_ns = self._last_timestamp_ns + 1

        self._last_timestamp_ns = candidate_ns
        return candidate_ns


class RgbFrameSource:
    """Reads frames from one configured RGB source."""

    def __init__(
        self,
        *,
        config: RgbSourceConfig,
        capture: CaptureLike,
        clock_ns: Callable[[], int] | None = None,
        capture_factory: Callable[[int | str], CaptureLike] | None = None,
        timestamp_generator: MonotonicTimestampGenerator | None = None,
    ) -> None:
        self._config = config
        self._capture = capture
        self._clock_ns = clock_ns or time.monotonic_ns
        self._capture_factory = capture_factory
        self._timestamp_generator = timestamp_generator or MonotonicTimestampGenerator(
            fallback_clock_ns=self._clock_ns
        )
        self._frame_index = 0
        self._source_frame_index = 0

    @property
    def config(self) -> RgbSourceConfig:
        return self._config

    def read(self) -> RgbFrame | None:
        success, image = self._capture.read()
        if not success or image is None:
            if (
                self._config.normalized_source_type == "replay"
                and self._config.loop_replay
            ):
                self._rewind_or_reopen()
                success, image = self._capture.read()
                if not success or image is None:
                    return None
            else:
                return None

        if not isinstance(image, np.ndarray):
            raise RgbSourceError("RGB source returned a non-array frame")
        if image.ndim != 3 or image.shape[2] != 3:
            raise RgbSourceError(
                "RGB source returned a frame without 3-channel BGR shape"
            )

        source_timestamp_ns = self._resolve_source_timestamp_ns()
        candidate_timestamp_ns = source_timestamp_ns
        if self._config.normalized_source_type != "replay":
            candidate_timestamp_ns = self._clock_ns()

        metadata = RgbFrameMetadata(
            source_type=self._config.normalized_source_type,
            source_name=self._config.effective_source_name,
            source_uri=self._config.source_uri,
            frame_id=self._config.frame_id,
            frame_index=self._frame_index,
            monotonic_timestamp_ns=self._timestamp_generator.normalize(
                candidate_timestamp_ns
            ),
            source_timestamp_ns=source_timestamp_ns,
            replayed=self._config.normalized_source_type == "replay",
        )
        self._frame_index += 1
        self._source_frame_index += 1
        return RgbFrame(image_bgr=image, metadata=metadata)

    def close(self) -> None:
        self._capture.release()

    def _resolve_source_timestamp_ns(self) -> int | None:
        if self._config.normalized_source_type != "replay":
            return None

        capture_position_ms = float(self._capture.get(CAP_PROP_POS_MSEC))
        if capture_position_ms > 0.0 or self._source_frame_index == 0:
            return max(0, round(capture_position_ms * 1_000_000.0))

        fps = float(self._capture.get(CAP_PROP_FPS))
        if fps > 0.0:
            return round((self._source_frame_index / fps) * 1_000_000_000.0)
        return None

    def _rewind_or_reopen(self) -> None:
        if self._capture_factory is not None:
            self._capture.release()
            replacement = self._capture_factory(self._config.capture_target)
            if not replacement.isOpened():
                replacement.release()
                raise RgbSourceError(
                    f"Unable to reopen replay source '{self._config.source_uri}'"
                )
            self._capture = replacement
            self._source_frame_index = 0
            return

        rewound = self._capture.set(CAP_PROP_POS_FRAMES, 0.0)
        if not rewound:
            raise RgbSourceError(
                f"Unable to rewind replay source '{self._config.source_uri}'"
            )
        self._source_frame_index = 0


def build_rgb_frame_source(
    config: RgbSourceConfig,
    *,
    capture_factory: Callable[[int | str], CaptureLike] | None = None,
    path_exists: Callable[[Path], bool] | None = None,
    clock_ns: Callable[[], int] | None = None,
) -> RgbReadableSource:
    """Build and validate an RGB source backed by OpenCV-style capture APIs."""

    if config.normalized_source_type == "manifest_replay":
        from .rgb_dataset import build_manifest_replay_frame_source

        return build_manifest_replay_frame_source(
            config,
            path_exists=path_exists,
            clock_ns=clock_ns,
        )

    if config.normalized_source_type in {"video_file", "replay"}:
        exists = path_exists or Path.exists
        source_path = Path(config.source_uri)
        if not exists(source_path):
            raise RgbSourceError(
                f"RGB source path does not exist: '{config.source_uri}'"
            )

    capture_builder = capture_factory or _build_opencv_capture
    capture = capture_builder(config.capture_target)
    if not capture.isOpened():
        capture.release()
        raise RgbSourceError(
            f"Unable to open RGB {config.normalized_source_type} source "
            f"'{config.source_uri}'"
        )

    return RgbFrameSource(
        config=config,
        capture=capture,
        clock_ns=clock_ns,
        capture_factory=(
            capture_builder if config.normalized_source_type == "replay" else None
        ),
    )


def _build_opencv_capture(target: int | str) -> CaptureLike:
    try:
        import cv2
    except ImportError as error:  # pragma: no cover - depends on ROS runtime image
        raise RgbSourceError(
            "OpenCV is required for RGB ingest but is not installed"
        ) from error

    return cv2.VideoCapture(target)
