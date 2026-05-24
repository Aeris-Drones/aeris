from aeris_update_manager.adapters import (
    FirmwareUpdateAdapter,
    HealthcheckError,
    FirmwareUpdateError,
    RollbackError,
    SignatureValidationError,
)
from aeris_update_manager.coordinator import FirmwareUpdateCoordinator
from aeris_update_manager.models import (
    FirmwarePartitionState,
    FirmwareUpdateCommand,
    FirmwareUpdateLifecycleState,
    StagedFirmwarePackage,
)


class FakeFirmwareUpdateAdapter(FirmwareUpdateAdapter):
    def __init__(
        self,
        *,
        current_version: str = "2026.04.9",
        active_slot: str = "A",
        inactive_slot: str = "B",
        signature_valid: bool = True,
        healthcheck_passes: bool = True,
        partition_state_failure: FirmwareUpdateError | None = None,
        rollback_failure: RollbackError | None = None,
    ) -> None:
        self.partition_state = FirmwarePartitionState(
            active_slot=active_slot,
            inactive_slot=inactive_slot,
            current_version=current_version,
        )
        self.signature_valid = signature_valid
        self.healthcheck_passes = healthcheck_passes
        self.partition_state_failure = partition_state_failure
        self.rollback_failure = rollback_failure
        self.downloaded_packages: list[str] = []
        self.applied_slots: list[str] = []
        self.boot_switches: list[str] = []
        self.rollbacks: list[str] = []

    def get_partition_state(self, vehicle_id: str) -> FirmwarePartitionState:
        if self.partition_state_failure is not None:
            raise self.partition_state_failure
        return self.partition_state

    def download_package(self, command: FirmwareUpdateCommand) -> StagedFirmwarePackage:
        self.downloaded_packages.append(command.package_id)
        return StagedFirmwarePackage(
            package_id=command.package_id,
            target_version=command.target_version,
            package_uri=command.package_uri,
            signature=command.package_signature,
        )

    def validate_package_signature(self, package: StagedFirmwarePackage) -> None:
        if not self.signature_valid:
            raise SignatureValidationError(
                code="signature_invalid",
                detail=f"{package.package_id} failed detached signature validation",
            )

    def apply_package(
        self,
        vehicle_id: str,
        package: StagedFirmwarePackage,
        target_slot: str,
    ) -> None:
        self.applied_slots.append(target_slot)

    def switch_boot_slot(self, vehicle_id: str, slot: str) -> None:
        self.boot_switches.append(slot)
        self.partition_state = FirmwarePartitionState(
            active_slot=slot,
            inactive_slot="A" if slot == "B" else "B",
            current_version=self.partition_state.current_version,
        )

    def run_healthcheck(self, vehicle_id: str, target_version: str) -> None:
        if not self.healthcheck_passes:
            raise HealthcheckError(
                code="healthcheck_failed",
                detail="Post-update healthcheck failed",
            )
        self.partition_state = FirmwarePartitionState(
            active_slot=self.partition_state.active_slot,
            inactive_slot=self.partition_state.inactive_slot,
            current_version=target_version,
        )

    def rollback_boot_slot(self, vehicle_id: str, slot: str) -> None:
        if self.rollback_failure is not None:
            raise self.rollback_failure
        self.rollbacks.append(slot)
        self.partition_state = FirmwarePartitionState(
            active_slot=slot,
            inactive_slot="A" if slot == "B" else "B",
            current_version="2026.04.9",
        )


def _command() -> FirmwareUpdateCommand:
    return FirmwareUpdateCommand(
        vehicle_id="scout_2",
        package_id="fw-2026.05.23",
        target_version="2026.05.23",
        package_uri="s3://updates/fw-2026.05.23.bin",
        package_signature="signed-manifest",
    )


def test_rejects_invalid_signatures_before_slot_swap() -> None:
    adapter = FakeFirmwareUpdateAdapter(signature_valid=False)
    coordinator = FirmwareUpdateCoordinator(adapter)

    history = coordinator.execute_update(_command())

    assert [snapshot.lifecycle_state for snapshot in history] == [
        FirmwareUpdateLifecycleState.DOWNLOADING,
        FirmwareUpdateLifecycleState.VALIDATING,
        FirmwareUpdateLifecycleState.FAILED,
    ]
    assert history[-1].error_code == "signature_invalid"
    assert history[-1].rollback_performed is False
    assert adapter.applied_slots == []
    assert adapter.boot_switches == []


def test_returns_structured_failed_status_when_partition_state_lookup_fails() -> None:
    adapter = FakeFirmwareUpdateAdapter(
        partition_state_failure=FirmwareUpdateError(
            code="partition_state_unavailable",
            detail="boot metadata unavailable",
        )
    )
    coordinator = FirmwareUpdateCoordinator(adapter)

    history = coordinator.execute_update(_command())

    assert [snapshot.lifecycle_state for snapshot in history] == [
        FirmwareUpdateLifecycleState.FAILED,
    ]
    assert history[-1].active_slot == "unknown"
    assert history[-1].inactive_slot == "unknown"
    assert history[-1].current_version == "unknown"
    assert history[-1].error_code == "partition_state_unavailable"
    assert history[-1].error_detail == "boot metadata unavailable"


def test_progresses_through_ab_update_and_reports_complete() -> None:
    adapter = FakeFirmwareUpdateAdapter()
    coordinator = FirmwareUpdateCoordinator(adapter)

    history = coordinator.execute_update(_command())

    assert [snapshot.lifecycle_state for snapshot in history] == [
        FirmwareUpdateLifecycleState.DOWNLOADING,
        FirmwareUpdateLifecycleState.VALIDATING,
        FirmwareUpdateLifecycleState.APPLYING,
        FirmwareUpdateLifecycleState.VERIFYING,
        FirmwareUpdateLifecycleState.COMPLETE,
    ]
    assert history[-1].active_slot == "B"
    assert history[-1].inactive_slot == "A"
    assert history[-1].current_version == "2026.05.23"
    assert history[-1].progress_percent == 100.0
    assert adapter.applied_slots == ["B"]
    assert adapter.boot_switches == ["B"]


def test_streams_snapshots_to_callback_as_the_update_progresses() -> None:
    adapter = FakeFirmwareUpdateAdapter()
    coordinator = FirmwareUpdateCoordinator(adapter)
    streamed: list = []

    history = coordinator.execute_update(_command(), on_snapshot=streamed.append)

    assert streamed == list(history)
    assert [snapshot.lifecycle_state for snapshot in streamed] == [
        FirmwareUpdateLifecycleState.DOWNLOADING,
        FirmwareUpdateLifecycleState.VALIDATING,
        FirmwareUpdateLifecycleState.APPLYING,
        FirmwareUpdateLifecycleState.VERIFYING,
        FirmwareUpdateLifecycleState.COMPLETE,
    ]


def test_rolls_back_when_healthcheck_fails_after_boot_swap() -> None:
    adapter = FakeFirmwareUpdateAdapter(healthcheck_passes=False)
    coordinator = FirmwareUpdateCoordinator(adapter)

    history = coordinator.execute_update(_command())

    assert [snapshot.lifecycle_state for snapshot in history] == [
        FirmwareUpdateLifecycleState.DOWNLOADING,
        FirmwareUpdateLifecycleState.VALIDATING,
        FirmwareUpdateLifecycleState.APPLYING,
        FirmwareUpdateLifecycleState.VERIFYING,
        FirmwareUpdateLifecycleState.ROLLING_BACK,
        FirmwareUpdateLifecycleState.ROLLED_BACK,
    ]
    assert history[-1].rollback_performed is True
    assert history[-1].active_slot == "A"
    assert history[-1].current_version == "2026.04.9"
    assert history[-1].error_code == "healthcheck_failed"
    assert adapter.rollbacks == ["A"]


def test_reports_last_known_slot_and_version_when_rollback_fails() -> None:
    adapter = FakeFirmwareUpdateAdapter(
        healthcheck_passes=False,
        rollback_failure=RollbackError(
            code="rollback_unavailable",
            detail="bootloader rejected rollback target",
        ),
    )
    coordinator = FirmwareUpdateCoordinator(adapter)

    history = coordinator.execute_update(_command())

    assert [snapshot.lifecycle_state for snapshot in history] == [
        FirmwareUpdateLifecycleState.DOWNLOADING,
        FirmwareUpdateLifecycleState.VALIDATING,
        FirmwareUpdateLifecycleState.APPLYING,
        FirmwareUpdateLifecycleState.VERIFYING,
        FirmwareUpdateLifecycleState.ROLLING_BACK,
        FirmwareUpdateLifecycleState.FAILED,
    ]
    assert history[-1].active_slot == "B"
    assert history[-1].inactive_slot == "A"
    assert history[-1].current_version == "2026.05.23"
    assert history[-1].error_code == "rollback_unavailable"
    assert history[-1].error_detail == "bootloader rejected rollback target"
