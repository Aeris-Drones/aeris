"""Deterministic firmware update coordinator for edge-owned A/B rollouts."""

from collections.abc import Callable

from .adapters import (
    FirmwareUpdateAdapter,
    FirmwareUpdateError,
    HealthcheckError,
    NullFirmwareUpdateAdapter,
)
from .models import (
    FirmwarePartitionState,
    FirmwareUpdateCommand,
    FirmwareUpdateLifecycleState,
    FirmwareUpdateStatusSnapshot,
)


class FirmwareUpdateCoordinator:
    """Runs the edge-controlled firmware update lifecycle for one request."""

    def __init__(self, adapter: FirmwareUpdateAdapter | None = None) -> None:
        self._adapter = adapter or NullFirmwareUpdateAdapter()

    def execute_update(
        self,
        command: FirmwareUpdateCommand,
        *,
        on_snapshot: Callable[[FirmwareUpdateStatusSnapshot], None] | None = None,
    ) -> tuple[FirmwareUpdateStatusSnapshot, ...]:
        history: list[FirmwareUpdateStatusSnapshot] = []
        unknown_state = FirmwarePartitionState(
            active_slot="unknown",
            inactive_slot="unknown",
            current_version="unknown",
        )
        last_known_state = unknown_state
        last_known_version = unknown_state.current_version

        def emit(
            lifecycle_state: FirmwareUpdateLifecycleState,
            progress_percent: float,
            partition_state: FirmwarePartitionState,
            *,
            current_version: str | None = None,
            rollback_performed: bool = False,
            status_detail: str = "",
            error_code: str = "",
            error_detail: str = "",
        ) -> FirmwareUpdateStatusSnapshot:
            nonlocal last_known_state, last_known_version
            snapshot = FirmwareUpdateStatusSnapshot(
                vehicle_id=command.vehicle_id,
                package_id=command.package_id,
                current_version=current_version
                if current_version is not None
                else partition_state.current_version,
                target_version=command.target_version,
                lifecycle_state=lifecycle_state,
                progress_percent=float(progress_percent),
                active_slot=partition_state.active_slot,
                inactive_slot=partition_state.inactive_slot,
                rollback_performed=rollback_performed,
                status_detail=status_detail,
                error_code=error_code,
                error_detail=error_detail,
            )
            history.append(snapshot)
            if on_snapshot is not None:
                on_snapshot(snapshot)
            last_known_state = partition_state
            last_known_version = snapshot.current_version
            return snapshot

        try:
            original_state = self._adapter.get_partition_state(command.vehicle_id)
            last_known_state = original_state
            last_known_version = original_state.current_version
            emit(
                FirmwareUpdateLifecycleState.DOWNLOADING,
                10.0,
                original_state,
                status_detail="Staging signed package",
            )
            package = self._adapter.download_package(command)

            emit(
                FirmwareUpdateLifecycleState.VALIDATING,
                30.0,
                original_state,
                status_detail="Validating detached package signature",
            )
            self._adapter.validate_package_signature(package)

            emit(
                FirmwareUpdateLifecycleState.APPLYING,
                65.0,
                original_state,
                status_detail=f"Applying package to inactive slot {original_state.inactive_slot}",
            )
            self._adapter.apply_package(
                command.vehicle_id,
                package,
                original_state.inactive_slot,
            )
            self._adapter.switch_boot_slot(command.vehicle_id, original_state.inactive_slot)
            last_known_state = FirmwarePartitionState(
                active_slot=original_state.inactive_slot,
                inactive_slot=original_state.active_slot,
                current_version=command.target_version,
            )
            last_known_version = command.target_version

            verifying_state = self._adapter.get_partition_state(command.vehicle_id)
            emit(
                FirmwareUpdateLifecycleState.VERIFYING,
                85.0,
                verifying_state,
                current_version=command.target_version,
                status_detail=f"Booted slot {verifying_state.active_slot}; running healthcheck",
            )

            try:
                self._adapter.run_healthcheck(command.vehicle_id, command.target_version)
            except HealthcheckError as error:
                emit(
                    FirmwareUpdateLifecycleState.ROLLING_BACK,
                    92.0,
                    verifying_state,
                    current_version=command.target_version,
                    status_detail=f"Healthcheck failed; restoring slot {original_state.active_slot}",
                    error_code=error.code,
                    error_detail=error.detail,
                )
                try:
                    self._adapter.rollback_boot_slot(command.vehicle_id, original_state.active_slot)
                except FirmwareUpdateError as rollback_error:
                    emit(
                        FirmwareUpdateLifecycleState.FAILED,
                        history[-1].progress_percent,
                        last_known_state,
                        current_version=last_known_version,
                        status_detail="Healthcheck failed and rollback did not complete",
                        error_code=rollback_error.code,
                        error_detail=(
                            f"{error.detail}; rollback failed: {rollback_error.detail}"
                        ),
                    )
                    return tuple(history)
                rolled_back_state = FirmwarePartitionState(
                    active_slot=original_state.active_slot,
                    inactive_slot=verifying_state.active_slot,
                    current_version=original_state.current_version,
                )
                last_known_state = rolled_back_state
                last_known_version = original_state.current_version
                try:
                    rolled_back_state = self._adapter.get_partition_state(command.vehicle_id)
                except FirmwareUpdateError:
                    emit(
                        FirmwareUpdateLifecycleState.ROLLED_BACK,
                        100.0,
                        rolled_back_state,
                        rollback_performed=True,
                        status_detail=f"Rolled back to slot {rolled_back_state.active_slot} after healthcheck failure",
                        error_code=error.code,
                        error_detail=error.detail,
                    )
                    return tuple(history)
                emit(
                    FirmwareUpdateLifecycleState.ROLLED_BACK,
                    100.0,
                    rolled_back_state,
                    rollback_performed=True,
                    status_detail=f"Rolled back to slot {rolled_back_state.active_slot} after healthcheck failure",
                    error_code=error.code,
                    error_detail=error.detail,
                )
                return tuple(history)

            complete_state = FirmwarePartitionState(
                active_slot=verifying_state.active_slot,
                inactive_slot=verifying_state.inactive_slot,
                current_version=command.target_version,
            )
            try:
                complete_state = self._adapter.get_partition_state(command.vehicle_id)
            except FirmwareUpdateError:
                emit(
                    FirmwareUpdateLifecycleState.COMPLETE,
                    100.0,
                    complete_state,
                    status_detail=(
                        f"Vehicle healthy on slot {complete_state.active_slot}; "
                        "partition state confirmation unavailable"
                    ),
                )
                return tuple(history)
            emit(
                FirmwareUpdateLifecycleState.COMPLETE,
                100.0,
                complete_state,
                status_detail=f"Vehicle healthy on slot {complete_state.active_slot}",
            )
            return tuple(history)

        except FirmwareUpdateError as error:
            emit(
                FirmwareUpdateLifecycleState.FAILED,
                history[-1].progress_percent if history else 0.0,
                last_known_state,
                current_version=last_known_version,
                status_detail="Firmware update failed before completion",
                error_code=error.code,
                error_detail=error.detail,
            )
            return tuple(history)
