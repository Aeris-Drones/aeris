"""MAVLink adapter abstraction for autonomous mission waypoint streaming."""

from __future__ import annotations

import threading
import time
from typing import Callable

from pymavlink import mavutil

Setpoint = dict[str, float]

_METERS_TO_CENTIMETERS = 100.0
_COMMAND_ACK_TIMEOUT_SEC = 1.0
_COMMAND_ACK_RETRIES = 1
_RTL_ACK_TIMEOUT_SEC = 1.0
_RTL_HEARTBEAT_TIMEOUT_SEC = 2.0
_RTL_ACK_RETRIES = 1
_PARTNER_PRIME_TIMEOUT_SEC = 0.4
_STREAM_PARTNER_POLL_TIMEOUT_SEC = 0.01


class MavlinkAdapter:
    """MAVLink transport adapter over UDP using pymavlink."""

    def __init__(
        self,
        *,
        host: str = "127.0.0.1",
        port: int = 14540,
        command_port: int | None = None,
        stream_hz: float = 10.0,
        target_system: int = 1,
        target_component: int = 1,
        source_system: int = 245,
        source_component: int = 190,
        logger: Callable[[str], None] | None = None,
    ) -> None:
        self._host = host
        self._port = port
        self._command_port = int(command_port) if command_port is not None else int(port)
        self._stream_hz = max(stream_hz, 2.1)
        self._target_system = int(target_system)
        self._target_component = int(target_component)
        self._source_system = source_system
        self._source_component = source_component
        self._logger = logger or (lambda _: None)
        self._partner_ready = self._command_port == self._port

        self._mav_connection = self._new_connection()
        self._command_connection = self._new_command_connection()
        self._connection_lock = threading.Lock()

        self._running = False
        self._stream_thread: threading.Thread | None = None
        self._mission_id = ""
        self._latest_setpoint: Setpoint | None = None
        self._state_lock = threading.Lock()
        self._last_setpoint_sent_monotonic = float("-inf")
        self._setpoint_dispatch_condition = threading.Condition(self._state_lock)

    @property
    def is_streaming(self) -> bool:
        return self._running

    @property
    def endpoint(self) -> tuple[str, int]:
        return self._host, self._port

    @property
    def last_setpoint_send_monotonic(self) -> float:
        with self._state_lock:
            return self._last_setpoint_sent_monotonic

    def command_port(self) -> int:
        return self._command_port

    @property
    def target(self) -> tuple[int, int]:
        return self._target_system, self._target_component

    def set_target(self, target_system: int, target_component: int = 1) -> None:
        self._target_system = int(target_system)
        self._target_component = int(target_component)

    def set_endpoint(self, host: str, port: int, command_port: int | None = None) -> None:
        host_clean = host.strip() or "127.0.0.1"
        port_clean = int(port)
        command_port_clean = (
            int(command_port) if command_port is not None else int(port_clean)
        )
        if (
            host_clean == self._host
            and port_clean == self._port
            and command_port_clean == self._command_port
        ):
            return

        with self._connection_lock:
            previous = self._mav_connection
            previous_command = self._command_connection
            try:
                previous.close()
            except OSError:
                pass
            if previous_command is not None:
                try:
                    previous_command.close()
                except OSError:
                    pass
            self._host = host_clean
            self._port = port_clean
            self._command_port = command_port_clean
            self._partner_ready = self._command_port == self._port
            self._mav_connection = self._new_connection()
            self._command_connection = self._new_command_connection()
        self._logger(
            "MAVLink endpoint set to "
            f"{self._host}:{self._port} (command_port={self._command_port})"
        )

    def upload_mission_items_int(self, mission_id: str, waypoints: list[Setpoint]) -> None:
        if not waypoints:
            return
        try:
            with self._connection_lock:
                mav = self._mav_connection.mav
                mav.mission_count_send(
                    self._target_system,
                    self._target_component,
                    len(waypoints),
                )
                for index, waypoint in enumerate(waypoints):
                    north_m = float(waypoint.get("z", 0.0))
                    east_m = float(waypoint.get("x", 0.0))
                    altitude_m = float(waypoint.get("altitude_m", 0.0))

                    mav.mission_item_int_send(
                        self._target_system,
                        self._target_component,
                        index,
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        1 if index == 0 else 0,
                        1,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        int(round(north_m * _METERS_TO_CENTIMETERS)),
                        int(round(east_m * _METERS_TO_CENTIMETERS)),
                        -altitude_m,
                    )
            self._logger(
                f"Uploaded {len(waypoints)} MISSION_ITEM_INT waypoints for {mission_id}"
            )
        except OSError as error:
            self._logger(f"MAVLink mission upload failed: {error}")

    def start_stream(self, mission_id: str, initial_setpoint: Setpoint) -> None:
        self.stop_stream()
        with self._state_lock:
            self._mission_id = mission_id
            self._latest_setpoint = dict(initial_setpoint)
            self._running = True
            self._last_setpoint_sent_monotonic = float("-inf")

        self._stream_thread = threading.Thread(
            target=self._stream_loop,
            name="aeris-mavlink-stream",
            daemon=True,
        )
        self._stream_thread.start()
        self._logger(
            f"MAVLink stream started to {self._host}:{self._port} at {self._stream_hz:.2f}Hz"
        )

    def update_setpoint(self, setpoint: Setpoint) -> None:
        with self._state_lock:
            self._latest_setpoint = dict(setpoint)

    def stop_stream(self) -> None:
        with self._state_lock:
            was_running = self._running
            self._running = False

        thread = self._stream_thread
        if thread and thread.is_alive():
            thread.join(timeout=1.0)
        self._stream_thread = None

        with self._state_lock:
            self._latest_setpoint = None

        if was_running:
            self._logger("MAVLink stream stopped")

    def close(self) -> None:
        self.stop_stream()
        with self._connection_lock:
            self._mav_connection.close()
            if self._command_connection is not None:
                self._command_connection.close()

    def wait_for_setpoint_dispatch(
        self, *, after_monotonic: float, timeout_sec: float
    ) -> float | None:
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        with self._state_lock:
            while self._last_setpoint_sent_monotonic <= after_monotonic:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._setpoint_dispatch_condition.wait(timeout=remaining)
            return self._last_setpoint_sent_monotonic

    def send_return_to_launch(self) -> bool:
        """Send an explicit RTL command to the currently selected endpoint."""
        return self._send_command_with_ack(
            command_id=mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            params=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            ack_timeout_sec=_RTL_ACK_TIMEOUT_SEC,
            ack_retries=_RTL_ACK_RETRIES,
            require_rtl_heartbeat=True,
            heartbeat_timeout_sec=_RTL_HEARTBEAT_TIMEOUT_SEC,
            label="RTL",
        )

    def send_hold_position(self) -> bool:
        """Put the target vehicle in PX4 AUTO.LOITER (hold) mode."""
        return self._send_command_with_ack(
            command_id=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            params=(
                float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                4.0,  # PX4_CUSTOM_MAIN_MODE_AUTO
                3.0,  # PX4_CUSTOM_SUB_MODE_AUTO_LOITER
                0.0,
                0.0,
                0.0,
                0.0,
            ),
            ack_timeout_sec=_COMMAND_ACK_TIMEOUT_SEC,
            ack_retries=_COMMAND_ACK_RETRIES,
            require_rtl_heartbeat=False,
            heartbeat_timeout_sec=0.0,
            label="HOLD",
        )

    def send_resume_mission(self) -> bool:
        """Put the target vehicle in PX4 AUTO.MISSION (resume) mode."""
        return self._send_command_with_ack(
            command_id=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            params=(
                float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                4.0,  # PX4_CUSTOM_MAIN_MODE_AUTO
                4.0,  # PX4_CUSTOM_SUB_MODE_AUTO_MISSION
                0.0,
                0.0,
                0.0,
                0.0,
            ),
            ack_timeout_sec=_COMMAND_ACK_TIMEOUT_SEC,
            ack_retries=_COMMAND_ACK_RETRIES,
            require_rtl_heartbeat=False,
            heartbeat_timeout_sec=0.0,
            label="RESUME",
        )

    def _send_command_with_ack(
        self,
        *,
        command_id: int,
        params: tuple[float, float, float, float, float, float, float],
        ack_timeout_sec: float,
        ack_retries: int,
        require_rtl_heartbeat: bool,
        heartbeat_timeout_sec: float,
        label: str,
    ) -> bool:
        accepted_results = {
            mavutil.mavlink.MAV_RESULT_ACCEPTED,
            mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
        }
        if self._command_connection is None and not self._ensure_partner_ready(
            timeout_sec=_PARTNER_PRIME_TIMEOUT_SEC
        ):
            self._logger(
                f"{label} command could not run because no MAVLink partner is available "
                f"for {self._host}:{self._port} (command_port={self._command_port})"
            )
            return False

        command_connection = (
            self._command_connection
            if self._command_connection is not None
            else self._mav_connection
        )
        command_log_endpoint = (
            f"{self._host}:{self._command_port}"
            if self._command_connection is not None
            else f"{self._host}:{self._port}"
        )

        for attempt in range(1, ack_retries + 2):
            confirmation = attempt - 1
            try:
                with self._connection_lock:
                    command_connection.mav.command_long_send(
                        self._target_system,
                        self._target_component,
                        command_id,
                        confirmation,
                        params[0],
                        params[1],
                        params[2],
                        params[3],
                        params[4],
                        params[5],
                        params[6],
                    )
                self._logger(
                    f"Sent command {command_id} ({label}) to "
                    f"{command_log_endpoint} (attempt {attempt}, confirmation={confirmation})"
                )
            except OSError as error:
                self._logger(f"MAVLink {label} command send failed: {error}")
                return False

            ack = self._wait_for_command_ack(command_id, ack_timeout_sec)
            if ack is None:
                if attempt <= ack_retries:
                    self._logger(
                        f"No COMMAND_ACK received for {label} command; retrying "
                        f"({attempt}/{ack_retries + 1})"
                    )
                    continue
                self._logger(f"No COMMAND_ACK received for {label} command")
                return False

            result = int(getattr(ack, "result", -1))
            if result in accepted_results:
                if require_rtl_heartbeat and not self._wait_for_heartbeat_rtl(
                    heartbeat_timeout_sec
                ):
                    self._logger(
                        "COMMAND_ACK accepted but RTL mode was not observed via HEARTBEAT "
                        f"for {self._host}:{self._port}"
                    )
                    if attempt <= ack_retries:
                        continue
                    return False
                return True

            self._logger(
                f"{label} command rejected with COMMAND_ACK result="
                f"{result} on {self._host}:{self._port}"
            )
            if attempt <= ack_retries:
                continue
            return False

        return False

    def _wait_for_command_ack(self, command_id: int, timeout_sec: float):
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        while time.monotonic() < deadline:
            remaining = max(deadline - time.monotonic(), 0.0)
            with self._connection_lock:
                ack = self._mav_connection.recv_match(
                    type="COMMAND_ACK",
                    blocking=True,
                    timeout=remaining,
                )
            if ack is None:
                return None
            if not self._message_matches_target(ack):
                continue
            if int(getattr(ack, "command", -1)) != command_id:
                continue
            return ack
        return None

    def _wait_for_heartbeat_rtl(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        while time.monotonic() < deadline:
            remaining = max(deadline - time.monotonic(), 0.0)
            with self._connection_lock:
                heartbeat = self._mav_connection.recv_match(
                    type="HEARTBEAT",
                    blocking=True,
                    timeout=remaining,
                )
            if heartbeat is None:
                return False
            if not self._message_matches_target(heartbeat):
                continue
            if self._heartbeat_indicates_rtl(heartbeat):
                return True
        return False

    def _message_matches_target(self, message) -> bool:
        get_system = getattr(message, "get_srcSystem", None)
        if callable(get_system):
            try:
                if int(self._target_system) > 0 and int(get_system()) != int(
                    self._target_system
                ):
                    return False
            except (TypeError, ValueError):
                return False

        get_component = getattr(message, "get_srcComponent", None)
        if callable(get_component):
            try:
                if int(self._target_component) > 0 and int(get_component()) != int(
                    self._target_component
                ):
                    return False
            except (TypeError, ValueError):
                return False

        return True

    def _heartbeat_indicates_rtl(self, heartbeat) -> bool:
        # PX4 encodes main/sub mode in custom_mode.
        custom_mode = int(getattr(heartbeat, "custom_mode", -1))
        if custom_mode >= 0:
            px4_main_mode = (custom_mode >> 16) & 0xFF
            px4_sub_mode = (custom_mode >> 24) & 0xFF
            if px4_main_mode == 4 and px4_sub_mode == 5:
                return True
            # ArduPilot Copter/Plane RTL custom mode values.
            if custom_mode in {6, 11}:
                return True
        return False

    def _stream_loop(self) -> None:
        period = 1.0 / self._stream_hz
        next_tick = time.perf_counter()

        while True:
            with self._state_lock:
                running = self._running
                setpoint = dict(self._latest_setpoint) if self._latest_setpoint else None

            if not running:
                return

            if setpoint is not None:
                if not self._partner_ready:
                    self._ensure_partner_ready(timeout_sec=_STREAM_PARTNER_POLL_TIMEOUT_SEC)
                self._send_local_ned_setpoint(setpoint)

            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                # Keep timing stable even if a cycle overruns.
                next_tick = time.perf_counter()

    def _new_connection(self):
        if self._command_port != self._port:
            return mavutil.mavlink_connection(
                f"udpin:0.0.0.0:{self._port}",
                source_system=self._source_system,
                source_component=self._source_component,
            )
        return mavutil.mavlink_connection(
            f"udpout:{self._host}:{self._port}",
            source_system=self._source_system,
            source_component=self._source_component,
        )

    def _new_command_connection(self):
        if self._command_port == self._port:
            return None
        return mavutil.mavlink_connection(
            f"udpout:{self._host}:{self._command_port}",
            source_system=self._source_system,
            source_component=self._source_component,
        )

    def _ensure_partner_ready(self, timeout_sec: float) -> bool:
        if self._partner_ready:
            return True

        with self._connection_lock:
            clients = getattr(self._mav_connection, "clients", None)
            if isinstance(clients, set) and clients:
                self._partner_ready = True
                return True

        deadline = time.monotonic() + max(timeout_sec, 0.0)
        while time.monotonic() < deadline:
            remaining = max(deadline - time.monotonic(), 0.0)
            wait_slice = min(0.25, remaining)
            try:
                with self._connection_lock:
                    sock = getattr(self._mav_connection, "port", None)
                    if sock is None:
                        self._partner_ready = True
                        return True
                    sock.sendto(b"\x00", (self._host, self._port))
                    if self._command_port != self._port:
                        sock.sendto(b"\x00", (self._host, self._command_port))
                    message = self._mav_connection.recv_match(
                        blocking=True, timeout=wait_slice
                    )
            except OSError as error:
                self._logger(f"MAVLink endpoint handshake failed: {error}")
                return False

            if message is not None:
                self._partner_ready = True
                return True

        return False

    def _send_local_ned_setpoint(self, setpoint: Setpoint) -> None:
        # Our mission plan uses x/east and z/north in meters. MAVLink local NED expects
        # x=north, y=east, z=down.
        north_m = float(setpoint.get("z", 0.0))
        east_m = float(setpoint.get("x", 0.0))
        altitude_m = float(setpoint.get("altitude_m", 0.0))
        down_m = -altitude_m

        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        time_boot_ms = int(time.time() * 1000.0) & 0xFFFFFFFF
        try:
            with self._connection_lock:
                self._mav_connection.mav.set_position_target_local_ned_send(
                    time_boot_ms,
                    self._target_system,
                    self._target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    type_mask,
                    north_m,
                    east_m,
                    down_m,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                )
            with self._state_lock:
                self._last_setpoint_sent_monotonic = time.monotonic()
                self._setpoint_dispatch_condition.notify_all()
        except OSError as error:
            self._logger(f"MAVLink setpoint send failed: {error}")
