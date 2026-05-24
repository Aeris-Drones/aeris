from aeris_update_manager.time_utils import (
    is_successful_terminal_state,
    split_nanoseconds,
)
from aeris_update_manager.models import FirmwareUpdateLifecycleState


def test_split_nanoseconds_preserves_second_and_nanosecond_precision() -> None:
    assert split_nanoseconds(4_567_890_123) == (4, 567_890_123)


def test_only_complete_is_a_successful_terminal_state() -> None:
    assert is_successful_terminal_state(FirmwareUpdateLifecycleState.COMPLETE) is True
    assert is_successful_terminal_state(FirmwareUpdateLifecycleState.ROLLED_BACK) is False
    assert is_successful_terminal_state(FirmwareUpdateLifecycleState.FAILED) is False
