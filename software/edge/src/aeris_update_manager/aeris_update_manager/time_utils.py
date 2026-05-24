"""Small pure helpers shared by the firmware update node and tests."""

from .models import FirmwareUpdateLifecycleState


def split_nanoseconds(nanoseconds: int) -> tuple[int, int]:
    total = max(0, int(nanoseconds))
    return total // 1_000_000_000, total % 1_000_000_000


def is_successful_terminal_state(
    state: FirmwareUpdateLifecycleState,
) -> bool:
    return state is FirmwareUpdateLifecycleState.COMPLETE
