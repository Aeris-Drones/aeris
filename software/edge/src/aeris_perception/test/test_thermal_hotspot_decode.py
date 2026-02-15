import numpy as np
import pytest

pytest.importorskip("rclpy")
sensor_msgs = pytest.importorskip("sensor_msgs.msg")
from sensor_msgs.msg import Image

from aeris_perception.thermal_hotspot_node import _decode_image_without_cv_bridge


def _make_image(
    *,
    array: np.ndarray,
    encoding: str,
    is_bigendian: bool,
    step: int,
    payload: bytes,
) -> Image:
    message = Image()
    message.height = int(array.shape[0])
    message.width = int(array.shape[1])
    message.encoding = encoding
    message.is_bigendian = int(is_bigendian)
    message.step = int(step)
    message.data = payload
    return message


def test_decode_without_cv_bridge_handles_row_padding_for_mono16() -> None:
    source = np.array([[100, 200], [300, 400]], dtype=np.uint16)
    row_payload = source.astype("<u2")
    padded_step = (source.shape[1] * source.itemsize) + 4
    payload = b"".join(
        row_payload[row].tobytes() + b"\x00" * (padded_step - (source.shape[1] * source.itemsize))
        for row in range(source.shape[0])
    )
    message = _make_image(
        array=source,
        encoding="mono16",
        is_bigendian=False,
        step=padded_step,
        payload=payload,
    )

    decoded = _decode_image_without_cv_bridge(message)

    assert decoded is not None
    assert np.array_equal(decoded, source)


def test_decode_without_cv_bridge_honors_big_endian_mono16() -> None:
    source = np.array([[512, 1024], [2048, 4096]], dtype=np.uint16)
    payload = source.astype(">u2").tobytes()
    step = source.shape[1] * source.itemsize
    message = _make_image(
        array=source,
        encoding="mono16",
        is_bigendian=True,
        step=step,
        payload=payload,
    )

    decoded = _decode_image_without_cv_bridge(message)

    assert decoded is not None
    assert np.array_equal(decoded.astype(np.uint16), source)


def test_decode_without_cv_bridge_rejects_invalid_step() -> None:
    source = np.array([[10, 20], [30, 40]], dtype=np.uint16)
    payload = source.tobytes()
    message = _make_image(
        array=source,
        encoding="mono16",
        is_bigendian=False,
        step=1,
        payload=payload,
    )

    assert _decode_image_without_cv_bridge(message) is None
