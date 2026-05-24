"""ROS 2 node wrapper for the Aeris firmware update coordinator."""

import threading

import rclpy
from aeris_msgs.msg import FirmwareUpdateStatus, FirmwareUpdateStatusArray
from aeris_msgs.srv import FirmwareUpdateCommand as FirmwareUpdateCommandService
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.parameter import Parameter

from .adapters import NullFirmwareUpdateAdapter
from .coordinator import FirmwareUpdateCoordinator
from .models import (
    FirmwareUpdateCommand,
    FirmwareUpdateLifecycleState,
    FirmwareUpdateStatusSnapshot,
)
from .time_utils import split_nanoseconds


class FirmwareUpdateManagerNode(Node):
    """Publishes typed firmware update state snapshots on the ROS graph."""

    STATUS_TOPIC = "/vehicle/firmware_update_status"
    COMMAND_SERVICE = "/vehicle/request_firmware_update"

    def __init__(
        self,
        *,
        parameter_overrides: list[Parameter] | None = None,
        adapter=None,
        coordinator: FirmwareUpdateCoordinator | None = None,
    ) -> None:
        super().__init__("aeris_firmware_update_manager", parameter_overrides=parameter_overrides)

        queue_depth = int(self.declare_parameter("queue_depth", 10).value)
        self._status_publisher = self.create_publisher(
            FirmwareUpdateStatusArray, self.STATUS_TOPIC, queue_depth
        )
        self._command_type = FirmwareUpdateCommandService
        self._coordinator = coordinator or FirmwareUpdateCoordinator(
            adapter or NullFirmwareUpdateAdapter()
        )
        self._latest_statuses: dict[str, FirmwareUpdateStatusSnapshot] = {}
        self._status_lock = threading.Lock()
        self._active_updates: set[str] = set()
        self._active_updates_lock = threading.Lock()
        self._worker_threads: dict[str, threading.Thread] = {}
        self._worker_threads_lock = threading.Lock()
        self._command_service = self.create_service(
            FirmwareUpdateCommandService,
            self.COMMAND_SERVICE,
            self._handle_command,
        )

        self.get_logger().info(
            "Firmware update manager ready: topic=%s, service=%s"
            % (self.STATUS_TOPIC, self.COMMAND_SERVICE)
        )

    def _handle_command(
        self,
        request: FirmwareUpdateCommandService.Request,
        response: FirmwareUpdateCommandService.Response,
    ) -> FirmwareUpdateCommandService.Response:
        command = FirmwareUpdateCommand(
            vehicle_id=request.vehicle_id,
            package_id=request.package_id,
            target_version=request.target_version,
            package_uri=request.package_uri,
            package_signature=request.package_signature,
        )

        with self._active_updates_lock:
            if command.vehicle_id in self._active_updates:
                response.accepted = False
                response.message = (
                    f"Firmware update already in progress for {command.vehicle_id}"
                )
                return response
            self._active_updates.add(command.vehicle_id)

        try:
            self._start_update_worker(command)
        except Exception as error:
            with self._active_updates_lock:
                self._active_updates.discard(command.vehicle_id)
            self.get_logger().error(
                "Firmware update could not be started: %s" % error
            )
            response.accepted = False
            response.message = str(error)
            return response

        response.accepted = True
        response.message = f"Firmware update started for {command.vehicle_id}"
        return response

    def _publish_snapshot(self, snapshot: FirmwareUpdateStatusSnapshot) -> None:
        with self._status_lock:
            self._latest_statuses[snapshot.vehicle_id] = snapshot
            snapshots = tuple(self._latest_statuses.values())
        self._status_publisher.publish(self._to_status_array(snapshots))

    def _start_update_worker(self, command: FirmwareUpdateCommand) -> None:
        worker = threading.Thread(
            target=self._run_update,
            args=(command,),
            daemon=False,
            name=f"firmware-update-{command.vehicle_id}",
        )
        with self._worker_threads_lock:
            self._worker_threads[command.vehicle_id] = worker
        try:
            worker.start()
        except Exception:
            with self._worker_threads_lock:
                self._worker_threads.pop(command.vehicle_id, None)
            raise

    def _run_update(self, command: FirmwareUpdateCommand) -> None:
        try:
            self._coordinator.execute_update(
                command,
                on_snapshot=self._publish_snapshot,
            )
        except Exception as error:
            self.get_logger().error(
                "Firmware update failed unexpectedly: %s" % error
            )
            self._publish_snapshot(
                self._unexpected_failure_snapshot(command, str(error))
            )
        finally:
            with self._active_updates_lock:
                self._active_updates.discard(command.vehicle_id)
            with self._worker_threads_lock:
                self._worker_threads.pop(command.vehicle_id, None)

    def _unexpected_failure_snapshot(
        self, command: FirmwareUpdateCommand, detail: str
    ) -> FirmwareUpdateStatusSnapshot:
        with self._status_lock:
            latest = self._latest_statuses.get(command.vehicle_id)
        return FirmwareUpdateStatusSnapshot(
            vehicle_id=command.vehicle_id,
            package_id=command.package_id,
            current_version=latest.current_version if latest else "unknown",
            target_version=command.target_version,
            lifecycle_state=FirmwareUpdateLifecycleState.FAILED,
            progress_percent=float(latest.progress_percent if latest else 0.0),
            active_slot=latest.active_slot if latest else "unknown",
            inactive_slot=latest.inactive_slot if latest else "unknown",
            rollback_performed=latest.rollback_performed if latest else False,
            status_detail="Firmware update failed before completion",
            error_code="unexpected_execution_error",
            error_detail=detail,
        )

    def _to_status_array(
        self, snapshots: tuple[FirmwareUpdateStatusSnapshot, ...]
    ) -> FirmwareUpdateStatusArray:
        message = FirmwareUpdateStatusArray()
        sec, nanosec = split_nanoseconds(self.get_clock().now().nanoseconds)
        message.observed_at = self._time_from_parts(sec, nanosec)
        for snapshot in snapshots:
            message.updates.append(self._to_status(snapshot))
        return message

    def _to_status(self, snapshot: FirmwareUpdateStatusSnapshot) -> FirmwareUpdateStatus:
        message = FirmwareUpdateStatus()
        message.vehicle_id = snapshot.vehicle_id
        message.package_id = snapshot.package_id
        message.current_version = snapshot.current_version
        message.target_version = snapshot.target_version
        message.lifecycle_state = self._state_constant(snapshot.lifecycle_state)
        message.lifecycle_state_label = snapshot.lifecycle_state.value
        message.progress_percent = float(snapshot.progress_percent)
        message.active_slot = snapshot.active_slot
        message.inactive_slot = snapshot.inactive_slot
        message.rollback_performed = snapshot.rollback_performed
        message.status_detail = snapshot.status_detail
        message.error_code = snapshot.error_code
        message.error_detail = snapshot.error_detail
        return message

    @staticmethod
    def _state_constant(state: FirmwareUpdateLifecycleState) -> int:
        return {
            FirmwareUpdateLifecycleState.UNKNOWN: FirmwareUpdateStatus.STATE_UNKNOWN,
            FirmwareUpdateLifecycleState.IDLE: FirmwareUpdateStatus.STATE_IDLE,
            FirmwareUpdateLifecycleState.DOWNLOADING: FirmwareUpdateStatus.STATE_DOWNLOADING,
            FirmwareUpdateLifecycleState.VALIDATING: FirmwareUpdateStatus.STATE_VALIDATING,
            FirmwareUpdateLifecycleState.APPLYING: FirmwareUpdateStatus.STATE_APPLYING,
            FirmwareUpdateLifecycleState.VERIFYING: FirmwareUpdateStatus.STATE_VERIFYING,
            FirmwareUpdateLifecycleState.COMPLETE: FirmwareUpdateStatus.STATE_COMPLETE,
            FirmwareUpdateLifecycleState.FAILED: FirmwareUpdateStatus.STATE_FAILED,
            FirmwareUpdateLifecycleState.ROLLING_BACK: FirmwareUpdateStatus.STATE_ROLLING_BACK,
            FirmwareUpdateLifecycleState.ROLLED_BACK: FirmwareUpdateStatus.STATE_ROLLED_BACK,
        }[state]

    @staticmethod
    def _time_from_parts(sec: int, nanosec: int) -> Time:
        time_message = Time()
        time_message.sec = max(0, int(sec))
        time_message.nanosec = max(0, int(nanosec))
        return time_message

    def destroy_node(self):
        current_thread = threading.current_thread()
        with self._worker_threads_lock:
            workers = list(self._worker_threads.values())
        for worker in workers:
            if worker is current_thread:
                continue
            if worker.is_alive():
                self.get_logger().info(
                    "Waiting for firmware update worker to finish before shutdown: %s"
                    % worker.name
                )
            worker.join()
        destroy = getattr(super(), "destroy_node", None)
        if destroy is None:
            return None
        return destroy()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FirmwareUpdateManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Firmware update manager shutdown requested.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
