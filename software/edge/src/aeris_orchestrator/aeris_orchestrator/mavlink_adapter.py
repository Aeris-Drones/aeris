"""MAVLink adapter abstraction for autonomous mission waypoint streaming."""

from __future__ import annotations

import json
import socket
import threading
import time
from typing import Callable

Setpoint = dict[str, float]


class MavlinkAdapter:
    """Best-effort MAVLink transport adapter over UDP.

    This keeps the orchestrator transport isolated and testable. It provides
    the required continuous proof-of-life stream while a mission is active.
    """

    def __init__(
        self,
        *,
        host: str = "127.0.0.1",
        port: int = 14540,
        stream_hz: float = 10.0,
        logger: Callable[[str], None] | None = None,
    ) -> None:
        self._host = host
        self._port = port
        self._stream_hz = max(stream_hz, 2.1)
        self._logger = logger or (lambda _: None)

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setblocking(False)

        self._running = False
        self._stream_thread: threading.Thread | None = None
        self._mission_id = ""
        self._latest_setpoint: Setpoint | None = None
        self._state_lock = threading.Lock()

    @property
    def is_streaming(self) -> bool:
        return self._running

    def upload_mission_items_int(self, mission_id: str, waypoints: list[Setpoint]) -> None:
        payload = {
            "protocol": "MISSION_ITEM_INT",
            "type": "MISSION_UPLOAD",
            "mission_id": mission_id,
            "count": len(waypoints),
            "waypoints": waypoints,
            "timestamp": time.time(),
        }
        self._send_payload(payload)

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
        self._socket.close()

    def _stream_loop(self) -> None:
        period = 1.0 / self._stream_hz
        next_tick = time.perf_counter()

        while True:
            with self._state_lock:
                running = self._running
                mission_id = self._mission_id
                setpoint = dict(self._latest_setpoint) if self._latest_setpoint else None

            if not running:
                return

            if setpoint is not None:
                payload = {
                    "protocol": "MAVLINK",
                    "type": "SET_POSITION_TARGET_LOCAL_NED",
                    "mission_id": mission_id,
                    "setpoint": setpoint,
                    "timestamp": time.time(),
                }
                self._send_payload(payload)

            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                # Keep timing stable even if a cycle overruns.
                next_tick = time.perf_counter()

    def _send_payload(self, payload: dict[str, object]) -> None:
        try:
            encoded = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self._socket.sendto(encoded, (self._host, self._port))
        except OSError as error:
            self._logger(f"MAVLink UDP send failed: {error}")
