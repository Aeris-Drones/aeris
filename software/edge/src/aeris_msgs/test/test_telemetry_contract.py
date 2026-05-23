from pathlib import Path


def test_telemetry_contract_exposes_battery_fields_with_explicit_availability():
    telemetry_msg = (
        Path(__file__).resolve().parents[1] / "msg" / "Telemetry.msg"
    ).read_text(encoding="utf-8")

    assert "float32 battery_percent" in telemetry_msg
    assert "float32 remaining_flight_time_sec" in telemetry_msg
    assert "bool remaining_flight_time_available" in telemetry_msg
