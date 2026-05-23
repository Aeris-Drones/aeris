import pytest

from aeris_device_manager.adapters import (
    InvalidEepromError,
    PodHardwareAdapter,
    PowerBudgetDeniedError,
    UnsupportedPodError,
)
from aeris_device_manager.models import (
    DetectedPod,
    PodLifecycleState,
    PodLinkInfo,
    PodMetadata,
)
from aeris_device_manager.state_machine import DeviceManagerStateMachine


class FakeClock:
    def __init__(self) -> None:
        self._monotonic = 100.0
        self._wall = 1_700_000_000.0

    def monotonic(self) -> float:
        return self._monotonic

    def wall_time(self) -> float:
        return self._wall

    def advance(self, seconds: float) -> None:
        self._monotonic += float(seconds)
        self._wall += float(seconds)


class FakeHardwareAdapter(PodHardwareAdapter):
    def __init__(
        self,
        clock: FakeClock,
        *,
        metadata: PodMetadata | None = None,
        capabilities: tuple[str, ...] | None = None,
        stage_durations: dict[str, float] | None = None,
        scan_pods: list[DetectedPod] | None = None,
        failure: str | None = None,
    ) -> None:
        self.clock = clock
        self.metadata = metadata or PodMetadata(
            serial="LDR-001",
            pod_type="lidar",
            capabilities=("lidar", "mapping"),
            nominal_power_watts=42.0,
        )
        self.capabilities = capabilities or self.metadata.capabilities
        self.stage_durations = {
            "eeprom_crc": 2.0,
            "power_budget": 0.5,
            "soft_start": 1.0,
            "link_enumeration": 4.0,
            "capability_registration": 3.5,
            **(stage_durations or {}),
        }
        self.scan_pods = scan_pods or [
            DetectedPod(vehicle_id="scout_1", slot_id="bay_a", one_wire_id="28-0001")
        ]
        self.failure = failure
        self.power_off_calls: list[tuple[str, str]] = []

    def scan(self) -> list[DetectedPod]:
        return list(self.scan_pods)

    def read_eeprom(self, pod: DetectedPod) -> PodMetadata:
        self.clock.advance(self.stage_durations["eeprom_crc"])
        if self.failure == "invalid_eeprom":
            raise InvalidEepromError(
                code="eeprom_crc_invalid",
                detail=f"CRC check failed for {pod.one_wire_id}",
            )
        if self.failure == "unsupported_pod":
            raise UnsupportedPodError(
                code="unsupported_pod",
                detail="pod type is not supported by this firmware build",
            )
        return self.metadata

    def check_power_budget(self, pod: DetectedPod, metadata: PodMetadata) -> None:
        self.clock.advance(self.stage_durations["power_budget"])
        if self.failure == "power_budget":
            raise PowerBudgetDeniedError(
                code="power_budget_denied",
                detail=f"pod requires {metadata.nominal_power_watts:.1f}W",
            )

    def soft_start(self, pod: DetectedPod, metadata: PodMetadata) -> None:
        self.clock.advance(self.stage_durations["soft_start"])

    def enumerate_link(self, pod: DetectedPod, metadata: PodMetadata) -> PodLinkInfo:
        self.clock.advance(self.stage_durations["link_enumeration"])
        return PodLinkInfo(transport="usb", capabilities=self.capabilities)

    def register_capabilities(
        self, pod: DetectedPod, metadata: PodMetadata, link: PodLinkInfo
    ) -> tuple[str, ...]:
        self.clock.advance(self.stage_durations["capability_registration"])
        return tuple(link.capabilities or metadata.capabilities)

    def power_off(self, pod: DetectedPod) -> None:
        self.power_off_calls.append((pod.vehicle_id, pod.slot_id))


def _state_sequence(result) -> list[PodLifecycleState]:
    return [
        snapshots[0].lifecycle_state for snapshots in result.transition_snapshots if snapshots
    ]


def test_registers_pod_with_typed_status_within_budget() -> None:
    clock = FakeClock()
    adapter = FakeHardwareAdapter(clock)
    machine = DeviceManagerStateMachine(adapter, clock=clock, enumeration_budget_sec=15.0)

    result = machine.reconcile()

    assert _state_sequence(result) == [
        PodLifecycleState.DETECTED,
        PodLifecycleState.VALIDATING,
        PodLifecycleState.POWER_CHECK,
        PodLifecycleState.SOFT_START,
        PodLifecycleState.ENUMERATING,
        PodLifecycleState.REGISTERED,
    ]
    assert len(result.current) == 1

    pod = result.current[0]
    assert pod.vehicle_id == "scout_1"
    assert pod.slot_id == "bay_a"
    assert pod.pod_serial == "LDR-001"
    assert pod.pod_type == "lidar"
    assert pod.lifecycle_state is PodLifecycleState.REGISTERED
    assert pod.capabilities == ("lidar", "mapping")
    assert pod.connected is True
    assert pod.power_ready is True
    assert pod.link_ready is True
    assert pod.enumeration_elapsed_sec == pytest.approx(11.0)
    assert [timing.stage for timing in pod.stage_timings] == [
        "detect",
        "eeprom_crc",
        "power_budget",
        "soft_start",
        "link_enumeration",
        "capability_registration",
    ]


def test_marks_registered_pod_disconnected_and_powers_it_off() -> None:
    clock = FakeClock()
    adapter = FakeHardwareAdapter(clock)
    machine = DeviceManagerStateMachine(adapter, clock=clock, enumeration_budget_sec=15.0)

    first_result = machine.reconcile()
    assert first_result.current[0].lifecycle_state is PodLifecycleState.REGISTERED

    adapter.scan_pods = []
    second_result = machine.reconcile()

    assert second_result.current[0].lifecycle_state is PodLifecycleState.DISCONNECTED
    assert second_result.current[0].connected is False
    assert second_result.current[0].power_ready is False
    assert second_result.current[0].link_ready is False
    assert second_result.current[0].capabilities == ("lidar", "mapping")
    assert adapter.power_off_calls == [("scout_1", "bay_a")]


def test_rejects_corrupt_eeprom_without_crashing() -> None:
    clock = FakeClock()
    adapter = FakeHardwareAdapter(clock, failure="invalid_eeprom")
    machine = DeviceManagerStateMachine(adapter, clock=clock, enumeration_budget_sec=15.0)

    result = machine.reconcile()

    assert _state_sequence(result) == [
        PodLifecycleState.DETECTED,
        PodLifecycleState.VALIDATING,
        PodLifecycleState.REJECTED,
    ]
    pod = result.current[0]
    assert pod.lifecycle_state is PodLifecycleState.REJECTED
    assert pod.rejection_code == "eeprom_crc_invalid"
    assert "CRC check failed" in pod.rejection_detail
    assert pod.power_ready is False
    assert pod.link_ready is False


def test_rejects_unsupported_pod() -> None:
    clock = FakeClock()
    adapter = FakeHardwareAdapter(clock, failure="unsupported_pod")
    machine = DeviceManagerStateMachine(adapter, clock=clock, enumeration_budget_sec=15.0)

    result = machine.reconcile()

    pod = result.current[0]
    assert pod.lifecycle_state is PodLifecycleState.REJECTED
    assert pod.rejection_code == "unsupported_pod"
    assert "not supported" in pod.rejection_detail


def test_rejects_when_power_budget_is_denied() -> None:
    clock = FakeClock()
    adapter = FakeHardwareAdapter(clock, failure="power_budget")
    machine = DeviceManagerStateMachine(adapter, clock=clock, enumeration_budget_sec=15.0)

    result = machine.reconcile()

    assert _state_sequence(result) == [
        PodLifecycleState.DETECTED,
        PodLifecycleState.VALIDATING,
        PodLifecycleState.POWER_CHECK,
        PodLifecycleState.REJECTED,
    ]
    pod = result.current[0]
    assert pod.lifecycle_state is PodLifecycleState.REJECTED
    assert pod.rejection_code == "power_budget_denied"
    assert "requires 42.0W" in pod.rejection_detail


def test_faults_when_enumeration_exceeds_budget() -> None:
    clock = FakeClock()
    adapter = FakeHardwareAdapter(
        clock,
        stage_durations={
            "eeprom_crc": 4.0,
            "power_budget": 3.0,
            "soft_start": 3.0,
            "link_enumeration": 3.0,
            "capability_registration": 3.0,
        },
    )
    machine = DeviceManagerStateMachine(adapter, clock=clock, enumeration_budget_sec=15.0)

    result = machine.reconcile()

    pod = result.current[0]
    assert pod.lifecycle_state is PodLifecycleState.FAULTED
    assert pod.fault_code == "enumeration_timeout"
    assert pod.enumeration_elapsed_sec == pytest.approx(16.0)
    assert adapter.power_off_calls == [("scout_1", "bay_a")]
