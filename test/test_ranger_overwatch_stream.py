"""Source-level tests for ranger overwatch suppression ordering."""

from pathlib import Path


MISSION_NODE = Path(
    "software/edge/src/aeris_orchestrator/aeris_orchestrator/mission_node.py"
)


def _read_dispatch_vehicle_command() -> str:
    source = MISSION_NODE.read_text(encoding="utf-8")
    start = source.index("    def _dispatch_vehicle_command(")
    end = source.index("    def _is_ranger_overwatch_suppressed(")
    return source[start:end]


def test_ranger_stream_stop_happens_after_successful_dispatch() -> None:
    source = _read_dispatch_vehicle_command()

    stop_call = "self._ranger_mavlink_adapter.stop_stream()"
    dispatch_call = "command_sent = dispatch()"

    assert stop_call in source
    assert dispatch_call in source
    assert source.index(dispatch_call) < source.index(
        stop_call
    ), "ranger overwatch stream should only stop after vehicle command dispatch succeeds"


def test_ranger_suppression_uses_command_state_constants() -> None:
    source = MISSION_NODE.read_text(encoding="utf-8")
    start = source.index("    def _is_ranger_overwatch_suppressed(")
    end = source.index("    def _apply_mavlink_endpoint(")
    function_source = source[start:end]

    assert 'self._VEHICLE_COMMAND_STATE_HOLDING' in function_source
    assert 'self._VEHICLE_COMMAND_STATE_RETURNING' in function_source
