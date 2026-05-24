from pathlib import Path


def test_firmware_update_status_contract_exposes_lifecycle_and_rollback_fields():
    status_msg = (
        Path(__file__).resolve().parents[1] / "msg" / "FirmwareUpdateStatus.msg"
    ).read_text(encoding="utf-8")

    assert "uint8 STATE_APPLYING=4" in status_msg
    assert "uint8 STATE_ROLLED_BACK=9" in status_msg
    assert "string current_version" in status_msg
    assert "string target_version" in status_msg
    assert "bool rollback_performed" in status_msg
    assert "string error_detail" in status_msg


def test_firmware_update_command_contract_keeps_mutation_out_of_telemetry():
    command_srv = (
        Path(__file__).resolve().parents[1] / "srv" / "FirmwareUpdateCommand.srv"
    ).read_text(encoding="utf-8")

    assert "string vehicle_id" in command_srv
    assert "string package_uri" in command_srv
    assert "string package_signature" in command_srv
    assert "bool accepted" in command_srv
