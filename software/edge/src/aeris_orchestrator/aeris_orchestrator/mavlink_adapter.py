"""MAVLink adapter abstraction for autonomous mission waypoint streaming."""

from __future__ import annotations

import threading
import time
from typing import Callable

from pymavlink import mavutil

Setpoint = dict[str, float]

_METERS_TO_CENTIMETERS = 100.0


class MavlinkAdapter:
    """MAVLink transport adapter over UDP using pymavlink."""

    def __init__(
        self,
        *,
        host: str = "127.0.0.1",
        port: int = 14540,
        stream_hz: float = 10.0,
        target_system: int = 1,
        target_component: int = 1,
        source_system: int = 245,
        source_component: int = 190,
        logger: Callable[[str], None] | None = None,
    ) -> None:
        self._host = host
        self._port = port
        self._stream_hz = max(stream_hz, 2.1)
        self._target_system = target_system
        self._target_component = target_component
        self._source_system = source_system
        self._source_component = source_component
        self._logger = logger or (lambda _: None)

        self._mav_connection = self._new_connection()
        self._connection_lock = threading.Lock()

        self._running = False
        self._stream_thread: threading.Thread | None = None
        self._mission_id = ""
        self._latest_setpoint: Setpoint | None = None
        self._state_lock = threading.Lock()

    @property
    def is_streaming(self) -> bool:
        return self._running

    @property
    def endpoint(self) -> tuple[str, int]:
        return self._host, self._port

    def set_endpoint(self, host: str, port: int) -> None:
        host_clean = host.strip() or "127.0.0.1"
        port_clean = int(port)
        if host_clean == self._host and port_clean == self._port:
            return

        with self._connection_lock:
            previous = self._mav_connection
            self._host = host_clean
            self._port = port_clean
            self._mav_connection = self._new_connection()
            try:
                previous.close()
            except OSError:
                pass
        self._logger(f"MAVLink endpoint set to {self._host}:{self._port}")

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

    def send_return_to_launch(self) -> bool:
        """Send an explicit RTL command to the currently selected endpoint."""
        try:
            with self._connection_lock:
                self._mav_connection.mav.command_long_send(
                    self._target_system,
                    self._target_component,
                    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                    0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                )
            self._logger(
                "Sent MAV_CMD_NAV_RETURN_TO_LAUNCH to "
                f"{self._host}:{self._port}"
            )
            return True
        except OSError as error:
            self._logger(f"MAVLink RTL command send failed: {error}")
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
                self._send_local_ned_setpoint(setpoint)

            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                # Keep timing stable even if a cycle overruns.
                next_tick = time.perf_counter()

    def _new_connection(self):
        return mavutil.mavlink_connection(
            f"udpout:{self._host}:{self._port}",
            source_system=self._source_system,
            source_component=self._source_component,
        )

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
        except OSError as error:
            self._logger(f"MAVLink setpoint send failed: {error}")
