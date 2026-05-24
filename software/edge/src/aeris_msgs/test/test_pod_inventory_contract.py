from pathlib import Path


def test_pod_inventory_record_contract_exposes_attachment_and_calibration_fields():
    record_msg = (
        Path(__file__).resolve().parents[1] / "msg" / "PodInventoryRecord.msg"
    ).read_text(encoding="utf-8")

    assert "uint8 CALIBRATION_DUE_SOON=2" in record_msg
    assert "uint8 CALIBRATION_OVERDUE=3" in record_msg
    assert "string pod_serial" in record_msg
    assert "bool attached" in record_msg
    assert "string vehicle_id" in record_msg
    assert "string slot_id" in record_msg
    assert "builtin_interfaces/Time last_calibration" in record_msg
    assert "builtin_interfaces/Time next_calibration_due" in record_msg
    assert "uint8 calibration_state" in record_msg
    assert "string calibration_detail" in record_msg


def test_log_pod_calibration_service_keeps_inventory_mutation_on_the_edge():
    command_srv = (
        Path(__file__).resolve().parents[1] / "srv" / "LogPodCalibration.srv"
    ).read_text(encoding="utf-8")

    assert "string pod_serial" in command_srv
    assert "builtin_interfaces/Time last_calibration" in command_srv
    assert "builtin_interfaces/Time next_calibration_due" in command_srv
    assert "bool accepted" in command_srv
    assert "string failure_code" in command_srv
    assert "aeris_msgs/PodInventoryRecord record" in command_srv
