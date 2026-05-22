"""Source-level tests for thermal hotspot rate-limit behavior."""

from pathlib import Path


THERMAL_NODE = Path(
    "software/edge/src/aeris_perception/aeris_perception/thermal_hotspot_node.py"
)


def _read_handle_thermal_image() -> str:
    source = THERMAL_NODE.read_text(encoding="utf-8")
    start = source.index("    def _handle_thermal_image(self, message: Image) -> None:")
    end = source.index("\n\ndef main(")
    return source[start:end]


def test_processing_rate_limit_uses_dedicated_processed_timestamp() -> None:
    source = THERMAL_NODE.read_text(encoding="utf-8")

    assert "self._last_processed_monotonic = 0.0" in source

    function_source = _read_handle_thermal_image()
    assert "if now - self._last_processed_monotonic < min_interval:" in function_source
    assert "self._last_processed_monotonic = now" in function_source


def test_publish_timestamp_updates_only_after_publish_loop() -> None:
    function_source = _read_handle_thermal_image()

    publish_call = "self._publisher.publish(output)"
    publish_timestamp = "self._last_publish_monotonic = now"

    assert publish_call in function_source
    assert publish_timestamp in function_source
    assert function_source.index(publish_call) < function_source.index(publish_timestamp)
