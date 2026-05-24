import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("aeris_msgs.msg")
pytest.importorskip("aeris_msgs.srv")

from aeris_update_manager.node import FirmwareUpdateManagerNode

from .test_update_coordinator import FakeFirmwareUpdateAdapter


def test_update_manager_node_publishes_structured_statuses_for_service_requests(monkeypatch) -> None:
    rclpy.init()
    node = None
    try:
        node = FirmwareUpdateManagerNode(adapter=FakeFirmwareUpdateAdapter())
        assert node.STATUS_TOPIC == "/vehicle/firmware_update_status"
        assert node.COMMAND_SERVICE == "/vehicle/request_firmware_update"

        published = []
        monkeypatch.setattr(node._status_publisher, "publish", published.append)

        request = node._command_type.Request()
        request.vehicle_id = "scout_2"
        request.package_id = "fw-2026.05.23"
        request.target_version = "2026.05.23"
        request.package_uri = "s3://updates/fw-2026.05.23.bin"
        request.package_signature = "signed-manifest"

        response = node._command_type.Response()
        result = node._handle_command(request, response)

        assert result.accepted is True
        assert published
        assert published[-1].updates[0].lifecycle_state_label == "complete"
        assert published[-1].updates[0].target_version == "2026.05.23"
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_update_manager_node_marks_rolled_back_updates_unsuccessful(monkeypatch) -> None:
    rclpy.init()
    node = None
    try:
        node = FirmwareUpdateManagerNode(
            adapter=FakeFirmwareUpdateAdapter(healthcheck_passes=False)
        )
        published = []
        monkeypatch.setattr(node._status_publisher, "publish", published.append)

        request = node._command_type.Request()
        request.vehicle_id = "scout_2"
        request.package_id = "fw-2026.05.23"
        request.target_version = "2026.05.23"
        request.package_uri = "s3://updates/fw-2026.05.23.bin"
        request.package_signature = "signed-manifest"

        response = node._command_type.Response()
        result = node._handle_command(request, response)

        assert result.accepted is False
        assert published[-1].updates[0].lifecycle_state_label == "rolled_back"
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
