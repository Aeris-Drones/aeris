import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("aeris_msgs.msg")

from aeris_msgs.msg import PodStatus

from aeris_device_manager.device_manager_node import DeviceManagerNode
from aeris_device_manager.models import DetectedPod, PodMetadata
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
        node = DeviceManagerNode(state_machine=machine)
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
