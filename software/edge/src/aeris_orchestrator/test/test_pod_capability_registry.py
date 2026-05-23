from aeris_orchestrator.pod_capability_registry import (
    PodCapabilityRecord,
    PodCapabilityRegistry,
    PodCapabilityState,
)


def _record(
    *,
    lifecycle_state: PodCapabilityState,
    capabilities: tuple[str, ...],
    vehicle_id: str = "scout_1",
    slot_id: str = "bay_a",
    pod_serial: str = "LDR-001",
    rejection_code: str = "",
    fault_code: str = "",
) -> PodCapabilityRecord:
    return PodCapabilityRecord(
        vehicle_id=vehicle_id,
        slot_id=slot_id,
        pod_serial=pod_serial,
        lifecycle_state=lifecycle_state,
        capabilities=capabilities,
        rejection_code=rejection_code,
        fault_code=fault_code,
    )


def test_registered_pod_capabilities_are_removed_on_disconnect() -> None:
    registry = PodCapabilityRegistry()

    registry.apply_records(
        [_record(lifecycle_state=PodCapabilityState.REGISTERED, capabilities=("lidar",))]
    )

    assert registry.available_capabilities_for_vehicle("scout1") == {"lidar"}
    assert registry.effective_slam_mode("scout1", "liosam") == "liosam"

    registry.apply_records(
        [_record(lifecycle_state=PodCapabilityState.DISCONNECTED, capabilities=("lidar",))]
    )

    assert registry.available_capabilities_for_vehicle("scout1") == set()
    assert registry.unavailable_capabilities_for_vehicle("scout1") == {"lidar"}
    assert registry.unavailable_reason("scout1", "lidar") == "disconnected"
    assert registry.effective_slam_mode("scout1", "liosam") == "vio"


def test_faulted_and_rejected_pods_mark_capabilities_unavailable() -> None:
    registry = PodCapabilityRegistry()

    registry.apply_records(
        [
            _record(
                lifecycle_state=PodCapabilityState.FAULTED,
                capabilities=("thermal",),
                slot_id="bay_b",
                pod_serial="THM-001",
                fault_code="soft_start_failed",
            ),
            _record(
                lifecycle_state=PodCapabilityState.REJECTED,
                capabilities=("gas",),
                slot_id="bay_c",
                pod_serial="GAS-001",
                rejection_code="unsupported_pod",
            ),
        ]
    )

    assert registry.available_capabilities_for_vehicle("scout_1") == set()
    assert registry.unavailable_capabilities_for_vehicle("scout_1") == {"gas", "thermal"}
    assert registry.unavailable_reason("scout_1", "thermal") == "faulted:soft_start_failed"
    assert registry.unavailable_reason("scout_1", "gas") == "rejected:unsupported_pod"
