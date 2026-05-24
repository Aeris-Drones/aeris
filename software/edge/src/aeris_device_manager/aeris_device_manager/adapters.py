"""Hardware adapter boundaries for the Aeris device manager."""

from typing import Protocol

from .models import DetectedPod, PodCalibrationMetadata, PodLinkInfo, PodMetadata


class DeviceManagerError(RuntimeError):
    """Structured device manager failure."""

    def __init__(self, *, code: str, detail: str) -> None:
        super().__init__(detail)
        self.code = code
        self.detail = detail


class InvalidEepromError(DeviceManagerError):
    """Raised when pod EEPROM cannot be trusted."""


class UnsupportedPodError(DeviceManagerError):
    """Raised when a pod reports an unsupported identity or capability set."""


class PowerBudgetDeniedError(DeviceManagerError):
    """Raised when the pod cannot be powered within the current budget."""


class SoftStartError(DeviceManagerError):
    """Raised when pod power ramp-up fails."""


class LinkEnumerationError(DeviceManagerError):
    """Raised when USB or Ethernet enumeration fails."""


class CalibrationWriteError(DeviceManagerError):
    """Raised when calibration metadata cannot be written or verified."""


class PodHardwareAdapter(Protocol):
    """Hardware adapter boundary for the pod lifecycle controller."""

    def scan(self) -> list[DetectedPod]:
        """Return the currently detected pods."""

    def read_eeprom(self, pod: DetectedPod) -> PodMetadata:
        """Read and validate the pod identity EEPROM."""

    def check_power_budget(self, pod: DetectedPod, metadata: PodMetadata) -> None:
        """Raise if the power budget cannot safely accommodate the pod."""

    def soft_start(self, pod: DetectedPod, metadata: PodMetadata) -> None:
        """Apply the pod soft-start sequence."""

    def enumerate_link(self, pod: DetectedPod, metadata: PodMetadata) -> PodLinkInfo:
        """Enumerate the pod data link and return discovered link metadata."""

    def register_capabilities(
        self, pod: DetectedPod, metadata: PodMetadata, link: PodLinkInfo
    ) -> tuple[str, ...]:
        """Return the capabilities to advertise once the pod is online."""

    def write_calibration(
        self, pod: DetectedPod, calibration: PodCalibrationMetadata
    ) -> PodCalibrationMetadata:
        """Persist calibration metadata and return the verified values."""

    def power_off(self, pod: DetectedPod) -> None:
        """Remove pod power after a disconnect or fault."""


class NullPodHardwareAdapter:
    """Default adapter used when no real hardware integration is configured."""

    def scan(self) -> list[DetectedPod]:
        return []

    def read_eeprom(self, pod: DetectedPod) -> PodMetadata:
        raise InvalidEepromError(
            code="eeprom_reader_unavailable",
            detail=f"no EEPROM reader configured for {pod.slot_id}",
        )

    def check_power_budget(self, pod: DetectedPod, metadata: PodMetadata) -> None:
        raise PowerBudgetDeniedError(
            code="power_budget_unavailable",
            detail=f"no power budget adapter configured for {pod.slot_id}",
        )

    def soft_start(self, pod: DetectedPod, metadata: PodMetadata) -> None:
        raise SoftStartError(
            code="soft_start_unavailable",
            detail=f"no soft-start adapter configured for {pod.slot_id}",
        )

    def enumerate_link(self, pod: DetectedPod, metadata: PodMetadata) -> PodLinkInfo:
        raise LinkEnumerationError(
            code="link_enumerator_unavailable",
            detail=f"no link enumerator configured for {pod.slot_id}",
        )

    def register_capabilities(
        self, pod: DetectedPod, metadata: PodMetadata, link: PodLinkInfo
    ) -> tuple[str, ...]:
        return tuple(metadata.capabilities)

    def write_calibration(
        self, pod: DetectedPod, calibration: PodCalibrationMetadata
    ) -> PodCalibrationMetadata:
        raise CalibrationWriteError(
            code="eeprom_writer_unavailable",
            detail=f"no EEPROM calibration writer configured for {pod.slot_id}",
        )

    def power_off(self, pod: DetectedPod) -> None:
        return None
