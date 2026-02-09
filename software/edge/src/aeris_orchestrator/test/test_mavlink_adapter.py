import time

from pymavlink import mavutil

from aeris_orchestrator.mavlink_adapter import MavlinkAdapter


class _FakeMav:
    def __init__(self) -> None:
        self.mission_count_calls: list[tuple] = []
        self.mission_item_int_calls: list[tuple] = []
        self.set_position_calls: list[tuple] = []
        self.command_long_calls: list[tuple] = []

    def mission_count_send(self, *args) -> None:
        self.mission_count_calls.append(args)

    def mission_item_int_send(self, *args) -> None:
        self.mission_item_int_calls.append(args)

    def set_position_target_local_ned_send(self, *args) -> None:
        self.set_position_calls.append(args)

    def command_long_send(self, *args) -> None:
        self.command_long_calls.append(args)


class _FakeConnection:
    def __init__(self, endpoint: str) -> None:
        self.endpoint = endpoint
        self.mav = _FakeMav()
        self.closed = False

    def close(self) -> None:
        self.closed = True


def test_adapter_sends_mission_item_int_and_local_ned_setpoints(monkeypatch) -> None:
    connections: list[_FakeConnection] = []

    def _fake_connection(endpoint: str, source_system: int, source_component: int):
        del source_system, source_component
        conn = _FakeConnection(endpoint)
        connections.append(conn)
        return conn

    monkeypatch.setattr(
        "aeris_orchestrator.mavlink_adapter.mavutil.mavlink_connection",
        _fake_connection,
    )

    adapter = MavlinkAdapter(host="127.0.0.1", port=14540, stream_hz=20.0)
    waypoints = [
        {"x": 10.5, "z": 3.25, "altitude_m": 20.0},
        {"x": 12.0, "z": 5.0, "altitude_m": 20.0},
    ]
    adapter.upload_mission_items_int("mission-1", waypoints)

    first_conn = connections[0]
    assert len(first_conn.mav.mission_count_calls) == 1
    assert first_conn.mav.mission_count_calls[0][2] == len(waypoints)
    assert len(first_conn.mav.mission_item_int_calls) == len(waypoints)
    assert first_conn.mav.mission_item_int_calls[0][3] == mavutil.mavlink.MAV_FRAME_LOCAL_NED
    assert first_conn.mav.mission_item_int_calls[0][4] == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT

    adapter.start_stream("mission-1", waypoints[0])
    time.sleep(0.1)
    adapter.stop_stream()
    assert len(first_conn.mav.set_position_calls) > 0

    setpoint_call = first_conn.mav.set_position_calls[0]
    assert setpoint_call[3] == mavutil.mavlink.MAV_FRAME_LOCAL_NED
    assert setpoint_call[5] == waypoints[0]["z"]  # north
    assert setpoint_call[6] == waypoints[0]["x"]  # east
    assert setpoint_call[7] == -waypoints[0]["altitude_m"]  # down

    adapter.set_endpoint("127.0.0.1", 14541)
    assert len(connections) == 2
    assert connections[0].closed
    adapter.close()


def test_adapter_sends_explicit_rtl_command_to_current_endpoint(monkeypatch) -> None:
    connections: list[_FakeConnection] = []

    def _fake_connection(endpoint: str, source_system: int, source_component: int):
        del source_system, source_component
        conn = _FakeConnection(endpoint)
        connections.append(conn)
        return conn

    monkeypatch.setattr(
        "aeris_orchestrator.mavlink_adapter.mavutil.mavlink_connection",
        _fake_connection,
    )

    adapter = MavlinkAdapter(host="127.0.0.1", port=14540, stream_hz=20.0)
    assert adapter.send_return_to_launch()

    first_conn = connections[0]
    assert len(first_conn.mav.command_long_calls) == 1
    rtl_call = first_conn.mav.command_long_calls[0]
    assert rtl_call[0] == 1  # target_system default
    assert rtl_call[1] == 1  # target_component default
    assert rtl_call[2] == mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH

    adapter.set_endpoint("127.0.0.1", 14541)
    assert adapter.send_return_to_launch()
    assert len(connections) == 2
    assert len(connections[1].mav.command_long_calls) == 1

    adapter.close()
