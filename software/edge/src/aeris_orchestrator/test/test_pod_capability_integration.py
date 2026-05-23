import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("pymavlink")
pytest.importorskip("aeris_msgs.msg")

from aeris_msgs.msg import PodStatus, PodStatusArray
from rclpy.parameter import Parameter

from aeris_orchestrator.mission_node import MissionNode


def _pod_status(*, lifecycle_state: int) -> PodStatus:
    message = PodStatus()
    message.vehicle_id = "scout_1"
    message.slot_id = "bay_a"
    message.pod_serial = "LDR-001"
    message.capabilities = ["lidar"]
    message.lifecycle_state = lifecycle_state
    return message


def test_mission_node_withdraws_lidar_backed_mode_on_disconnect_or_fault() -> None:
    rclpy.init()
    node = None
    try:
        node = MissionNode(
            parameter_overrides=[Parameter("slam_mode", value="liosam")]
        )

        registered = PodStatusArray()
        registered.pods.append(
            _pod_status(lifecycle_state=PodStatus.STATE_REGISTERED)
        )
        node._handle_pod_status_array(registered)
        assert node._slam_mode_for_vehicle("scout1") == "liosam"

        disconnected = PodStatusArray()
        disconnected.pods.append(
            _pod_status(lifecycle_state=PodStatus.STATE_DISCONNECTED)
        )
        node._handle_pod_status_array(disconnected)
        assert node._slam_mode_for_vehicle("scout1") == "vio"
        assert "scout_1" in node._logged_lidar_downgrades

        faulted = PodStatusArray()
        pod = _pod_status(lifecycle_state=PodStatus.STATE_FAULTED)
        pod.fault_code = "link_enumeration_failed"
        faulted.pods.append(pod)
        node._handle_pod_status_array(faulted)
        node._vehicle_slam_mode_overrides["scout_1"] = "lio_sam"
        assert node._slam_mode_for_vehicle("scout_1") == "vio"
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
