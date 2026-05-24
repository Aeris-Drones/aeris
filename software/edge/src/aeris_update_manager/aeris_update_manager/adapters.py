"""Adapter boundaries for Aeris firmware update coordination."""

from typing import Protocol

from .models import (
    FirmwarePartitionState,
    FirmwareUpdateCommand,
    StagedFirmwarePackage,
)


class FirmwareUpdateError(RuntimeError):
    """Structured firmware update failure."""

    def __init__(self, *, code: str, detail: str) -> None:
        super().__init__(detail)
        self.code = code
        self.detail = detail


class PackageDownloadError(FirmwareUpdateError):
    """Raised when the package cannot be staged."""


class SignatureValidationError(FirmwareUpdateError):
    """Raised when the detached signature cannot be trusted."""


class ApplyUpdateError(FirmwareUpdateError):
    """Raised when applying to the inactive slot fails."""


class BootSwitchError(FirmwareUpdateError):
    """Raised when the boot target cannot be switched."""


class HealthcheckError(FirmwareUpdateError):
    """Raised when the post-update healthcheck fails."""


class RollbackError(FirmwareUpdateError):
    """Raised when automatic rollback cannot be completed."""


class FirmwareUpdateAdapter(Protocol):
    """System-specific firmware update seam."""

    def get_partition_state(self, vehicle_id: str) -> FirmwarePartitionState:
        """Return the active/inactive slot state for a vehicle."""

    def download_package(
        self, command: FirmwareUpdateCommand
    ) -> StagedFirmwarePackage:
        """Stage the requested package for validation and apply."""

    def validate_package_signature(self, package: StagedFirmwarePackage) -> None:
        """Raise when the package signature is invalid."""

    def apply_package(
        self,
        vehicle_id: str,
        package: StagedFirmwarePackage,
        target_slot: str,
    ) -> None:
        """Write the staged package to the inactive slot."""

    def switch_boot_slot(self, vehicle_id: str, slot: str) -> None:
        """Point the bootloader at the requested slot."""

    def run_healthcheck(self, vehicle_id: str, target_version: str) -> None:
        """Validate the post-update system state."""

    def rollback_boot_slot(self, vehicle_id: str, slot: str) -> None:
        """Restore the previous boot target after a failed healthcheck."""


class NullFirmwareUpdateAdapter:
    """Default adapter used when no updater integration is configured."""

    def get_partition_state(self, vehicle_id: str) -> FirmwarePartitionState:
        return FirmwarePartitionState(
            active_slot="A",
            inactive_slot="B",
            current_version="unknown",
        )

    def download_package(
        self, command: FirmwareUpdateCommand
    ) -> StagedFirmwarePackage:
        raise PackageDownloadError(
            code="package_download_unavailable",
            detail=f"no firmware package staging adapter configured for {command.vehicle_id}",
        )

    def validate_package_signature(self, package: StagedFirmwarePackage) -> None:
        raise SignatureValidationError(
            code="signature_validator_unavailable",
            detail=f"no signature validator configured for {package.package_id}",
        )

    def apply_package(
        self,
        vehicle_id: str,
        package: StagedFirmwarePackage,
        target_slot: str,
    ) -> None:
        raise ApplyUpdateError(
            code="package_apply_unavailable",
            detail=f"no inactive-slot writer configured for {vehicle_id}",
        )

    def switch_boot_slot(self, vehicle_id: str, slot: str) -> None:
        raise BootSwitchError(
            code="boot_switch_unavailable",
            detail=f"no boot slot switcher configured for {vehicle_id}",
        )

    def run_healthcheck(self, vehicle_id: str, target_version: str) -> None:
        raise HealthcheckError(
            code="healthcheck_unavailable",
            detail=f"no post-update healthcheck configured for {vehicle_id}",
        )

    def rollback_boot_slot(self, vehicle_id: str, slot: str) -> None:
        raise RollbackError(
            code="rollback_unavailable",
            detail=f"no rollback handler configured for {vehicle_id}",
        )
