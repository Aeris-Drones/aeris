import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("aeris_msgs.msg")
pytest.importorskip("aeris_msgs.srv")

from aeris_msgs.msg import PodStatus
from aeris_msgs.srv import LogPodCalibration

from aeris_device_manager.device_manager_node import DeviceManagerNode
from aeris_device_manager.models import DetectedPod, PodCalibrationMetadata, PodMetadata
from aeris_device_manager.state_machine import DeviceManagerStateMachine

from .test_state_machine import FakeClock, FakeHardwareAdapter


def test_device_manager_node_publishes_structured_state_transitions(monkeypatch) -> None:
    rclpy.init()
    node = None
    try:
        clock = FakeClock()
        adapter = FakeHardwareAdapter(
            clock,
            metadata=PodMetadata(
                serial="LDR-001",
                pod_type="lidar",
                capabilities=("lidar", "mapping"),
                nominal_power_watts=42.0,
            ),
            scan_pods=[
                DetectedPod(vehicle_id="scout_1", slot_id="bay_a", one_wire_id="28-0001")
            ],
        )
        machine = DeviceManagerStateMachine(
            adapter, clock=clock, enumeration_budget_sec=15.0
        )
        node = DeviceManagerNode(state_machine=machine, adapter=adapter)
        assert node.PODS_TOPIC == "/device_manager/pods"

        published = []
        monkeypatch.setattr(node._pods_publisher, "publish", published.append)

        node._poll_and_publish()

        assert published
        assert published[-1].pods[0].lifecycle_state == PodStatus.STATE_REGISTERED
        assert published[-1].pods[0].pod_serial == "LDR-001"
        assert list(published[-1].pods[0].capabilities) == ["lidar", "mapping"]

        adapter.scan_pods = []
        node._poll_and_publish()

        assert published[-1].pods[0].lifecycle_state == PodStatus.STATE_DISCONNECTED
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_device_manager_node_requires_explicit_adapter_with_custom_state_machine() -> None:
    rclpy.init()
    try:
        clock = FakeClock()
        adapter = FakeHardwareAdapter(clock)
        machine = DeviceManagerStateMachine(
            adapter, clock=clock, enumeration_budget_sec=15.0
        )
        with pytest.raises(ValueError, match="adapter must be provided"):
            DeviceManagerNode(state_machine=machine)
    finally:
        rclpy.shutdown()


def test_device_manager_node_publishes_inventory_and_persists_calibration_updates(
    tmp_path, monkeypatch
) -> None:
    rclpy.init()
    node = None
    try:
        clock = FakeClock()
        adapter = FakeHardwareAdapter(
            clock,
            metadata=PodMetadata(
                serial="GAS-118",
                pod_type="hazmat",
                capabilities=("gas", "hazmat"),
                nominal_power_watts=18.0,
                calibration=PodCalibrationMetadata(
                    last_calibration_sec=1_700_000_000.0,
                    next_calibration_due_sec=1_700_864_000.0,
                ),
            ),
            scan_pods=[
                DetectedPod(vehicle_id="scout_2", slot_id="belly", one_wire_id="28-0018")
            ],
        )
        machine = DeviceManagerStateMachine(
            adapter, clock=clock, enumeration_budget_sec=15.0
        )
        node = DeviceManagerNode(
            state_machine=machine,
            adapter=adapter,
            inventory_registry_path=tmp_path / "pod-inventory.json",
        )

        published_inventory = []
        monkeypatch.setattr(node._inventory_publisher, "publish", published_inventory.append)

        node._poll_and_publish()

        assert published_inventory
        assert published_inventory[-1].records[0].pod_serial == "GAS-118"
        assert published_inventory[-1].records[0].calibration_state == 2

        request = LogPodCalibration.Request()
        request.pod_serial = "GAS-118"
        request.last_calibration.sec = 1_710_000_000
        request.next_calibration_due.sec = 1_740_000_000

        response = node._handle_calibration_command(
            request, LogPodCalibration.Response()
        )

        assert response.accepted is True
        assert response.failure_code == ""
        assert response.record.pod_serial == "GAS-118"
        assert response.record.calibration_state == response.record.CALIBRATION_CURRENT
        assert adapter.calibration_writes == [
            ("28-0018", 1_710_000_000.0, 1_740_000_000.0)
        ]
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_device_manager_node_returns_structured_calibration_failures(
    tmp_path, monkeypatch
) -> None:
    rclpy.init()
    node = None
    try:
        clock = FakeClock()
        adapter = FakeHardwareAdapter(
            clock,
            metadata=PodMetadata(
                serial="GAS-118",
                pod_type="hazmat",
                capabilities=("gas", "hazmat"),
                nominal_power_watts=18.0,
            ),
            scan_pods=[
                DetectedPod(vehicle_id="scout_2", slot_id="belly", one_wire_id="28-0018")
            ],
        )
        machine = DeviceManagerStateMachine(
            adapter, clock=clock, enumeration_budget_sec=15.0
        )
        node = DeviceManagerNode(
            state_machine=machine,
            adapter=adapter,
            inventory_registry_path=tmp_path / "pod-inventory.json",
        )
        node._poll_and_publish()

        request = LogPodCalibration.Request()
        request.pod_serial = "GAS-118"
        request.last_calibration.sec = 1_710_000_000
        request.next_calibration_due.sec = 1_740_000_000

        monkeypatch.setattr(
            node._inventory_registry,
            "apply_calibration_update",
            lambda **_: (_ for _ in ()).throw(KeyError("missing")),
        )
        missing_response = node._handle_calibration_command(
            request, LogPodCalibration.Response()
        )
        assert missing_response.accepted is False
        assert missing_response.failure_code == "inventory_record_missing"

        monkeypatch.setattr(
            node._adapter,
            "write_calibration",
            lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError("boom")),
        )
        internal_response = node._handle_calibration_command(
            request, LogPodCalibration.Response()
        )
        assert internal_response.accepted is False
        assert internal_response.failure_code == "internal_error"
        assert "Unexpected calibration write failure" in internal_response.message
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_device_manager_node_rejects_calibration_logging_for_detached_or_unverified_pods(
    tmp_path,
) -> None:
    rclpy.init()
    node = None
    try:
        clock = FakeClock()
        adapter = FakeHardwareAdapter(
            clock,
            metadata=PodMetadata(
                serial="LDR-551",
                pod_type="lidar",
                capabilities=("lidar", "mapping"),
                nominal_power_watts=42.0,
            ),
            scan_pods=[
                DetectedPod(vehicle_id="ranger_1", slot_id="rear-bay", one_wire_id="28-0055")
            ],
        )
        machine = DeviceManagerStateMachine(
            adapter, clock=clock, enumeration_budget_sec=15.0
        )
        node = DeviceManagerNode(
            state_machine=machine,
            adapter=adapter,
            inventory_registry_path=tmp_path / "pod-inventory.json",
        )
        node._poll_and_publish()

        adapter.scan_pods = []
        node._poll_and_publish()

        detached_request = LogPodCalibration.Request()
        detached_request.pod_serial = "LDR-551"
        detached_request.last_calibration.sec = 1_710_000_000
        detached_request.next_calibration_due.sec = 1_740_000_000

        detached_response = node._handle_calibration_command(
            detached_request, LogPodCalibration.Response()
        )

        assert detached_response.accepted is False
        assert detached_response.failure_code == "pod_not_connected"

        adapter.scan_pods = [
            DetectedPod(vehicle_id="ranger_1", slot_id="rear-bay", one_wire_id="28-0055")
        ]
        adapter.calibration_write_failure = "verification"
        node._poll_and_publish()

        failed_response = node._handle_calibration_command(
            detached_request, LogPodCalibration.Response()
        )

        assert failed_response.accepted is False
        assert failed_response.failure_code == "eeprom_verification_failed"
        assert "verify" in failed_response.message.lower()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
