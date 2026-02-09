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
        self.messages_by_type: dict[str, list[object]] = {}
        self.recv_match_calls: list[tuple[str, bool, float | None]] = []

    def close(self) -> None:
        self.closed = True

    def recv_match(self, type: str, blocking: bool, timeout: float | None = None):
        self.recv_match_calls.append((type, blocking, timeout))
        queue = self.messages_by_type.get(type, [])
        if queue:
            return queue.pop(0)
        return None


class _FakeAck:
    def __init__(self, command: int, result: int) -> None:
        self.command = command
        self.result = result


class _FakeHeartbeat:
    def __init__(self, custom_mode: int) -> None:
        self.custom_mode = custom_mode


def _px4_rtl_custom_mode() -> int:
    # PX4 MAIN_MODE_AUTO=4, SUB_MODE_AUTO_RTL=5.
    return (5 << 24) | (4 << 16)


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
    connections[0].messages_by_type.setdefault("COMMAND_ACK", []).append(
        _FakeAck(
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            mavutil.mavlink.MAV_RESULT_ACCEPTED,
        )
    )
    connections[0].messages_by_type.setdefault("HEARTBEAT", []).append(
        _FakeHeartbeat(_px4_rtl_custom_mode())
    )
    assert adapter.send_return_to_launch()

    first_conn = connections[0]
    assert len(first_conn.mav.command_long_calls) == 1
    rtl_call = first_conn.mav.command_long_calls[0]
    assert rtl_call[0] == 1  # target_system default
    assert rtl_call[1] == 1  # target_component default
    assert rtl_call[2] == mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
    assert rtl_call[3] == 0  # confirmation on first attempt

    adapter.set_endpoint("127.0.0.1", 14541)
    connections[1].messages_by_type.setdefault("COMMAND_ACK", []).append(
        _FakeAck(
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            mavutil.mavlink.MAV_RESULT_ACCEPTED,
        )
    )
    connections[1].messages_by_type.setdefault("HEARTBEAT", []).append(
        _FakeHeartbeat(_px4_rtl_custom_mode())
    )
    assert adapter.send_return_to_launch()
    assert len(connections) == 2
    assert len(connections[1].mav.command_long_calls) == 1

    adapter.close()


def test_adapter_retries_rtl_when_ack_is_missing(monkeypatch) -> None:
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
    connections[0].messages_by_type.setdefault("COMMAND_ACK", []).extend(
        [
            None,
            _FakeAck(
                mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
            ),
        ]
    )
    connections[0].messages_by_type.setdefault("HEARTBEAT", []).append(
        _FakeHeartbeat(_px4_rtl_custom_mode())
    )

    assert adapter.send_return_to_launch()
    assert len(connections[0].mav.command_long_calls) == 2
    first_attempt = connections[0].mav.command_long_calls[0]
    second_attempt = connections[0].mav.command_long_calls[1]
    assert first_attempt[3] == 0
    assert second_attempt[3] == 1
    adapter.close()


def test_adapter_returns_false_when_heartbeat_never_confirms_rtl(monkeypatch) -> None:
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
    connections[0].messages_by_type.setdefault("COMMAND_ACK", []).extend(
        [
            _FakeAck(
                mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
            ),
            _FakeAck(
                mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
            ),
        ]
    )
    connections[0].messages_by_type.setdefault("HEARTBEAT", []).extend(
        [
            _FakeHeartbeat(0),
            _FakeHeartbeat(0),
        ]
    )

    assert not adapter.send_return_to_launch()
    assert len(connections[0].mav.command_long_calls) == 2
    adapter.close()


def test_adapter_send_rtl_returns_false_on_oserror(monkeypatch) -> None:
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

    def _raise_oserror(*_args):
        raise OSError("simulated failure")

    connections[0].mav.command_long_send = _raise_oserror

    assert not adapter.send_return_to_launch()
    adapter.close()


def test_adapter_sends_hold_and_resume_with_ack(monkeypatch) -> None:
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
    command_id = mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE
    connections[0].messages_by_type.setdefault("COMMAND_ACK", []).extend(
        [
            _FakeAck(command_id, mavutil.mavlink.MAV_RESULT_ACCEPTED),
            _FakeAck(command_id, mavutil.mavlink.MAV_RESULT_ACCEPTED),
        ]
    )

    assert adapter.send_hold_position()
    assert adapter.send_resume_mission()

    sent_calls = connections[0].mav.command_long_calls
    assert len(sent_calls) == 2
    assert sent_calls[0][2] == command_id
    assert sent_calls[0][4] == 0.0
    assert sent_calls[1][2] == command_id
    assert sent_calls[1][4] == 1.0
    adapter.close()


def test_adapter_rejects_hold_when_command_ack_is_rejected(monkeypatch) -> None:
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
    connections[0].messages_by_type.setdefault("COMMAND_ACK", []).append(
        _FakeAck(
            mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE,
            mavutil.mavlink.MAV_RESULT_DENIED,
        )
    )

    assert not adapter.send_hold_position()
    assert len(connections[0].mav.command_long_calls) >= 1
    adapter.close()
