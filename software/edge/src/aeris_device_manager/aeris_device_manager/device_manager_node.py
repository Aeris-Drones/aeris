"""ROS 2 node wrapper for the Aeris device manager state machine."""

from pathlib import Path
import time

import rclpy
from aeris_msgs.msg import PodInventoryArray, PodInventoryRecord
from aeris_msgs.msg import PodStageTiming as PodStageTimingMessage
from aeris_msgs.msg import PodStatus, PodStatusArray
from aeris_msgs.srv import LogPodCalibration as LogPodCalibrationService
from builtin_interfaces.msg import Time
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.parameter import Parameter

from .adapters import DeviceManagerError, NullPodHardwareAdapter, PodHardwareAdapter
from .inventory import PodCalibrationState, PodInventoryRecord as InventoryRow
from .inventory import PodInventoryRegistry
from .models import DetectedPod, PodCalibrationMetadata, PodLifecycleState, PodStatusSnapshot
from .state_machine import DeviceManagerClock, DeviceManagerStateMachine


class RosDeviceManagerClock(DeviceManagerClock):
    """Bridges ROS timestamps with monotonic duration tracking."""

    def __init__(self, clock: Clock) -> None:
        self._clock = clock

    def monotonic(self) -> float:
        return time.monotonic()

    def wall_time(self) -> float:
        return self._clock.now().nanoseconds / 1e9


class DeviceManagerNode(Node):
    """Publishes typed pod state snapshots on the ROS graph."""

    PODS_TOPIC = "/device_manager/pods"
    INVENTORY_TOPIC = "/device_manager/pod_inventory"
    CALIBRATION_SERVICE = "/device_manager/log_pod_calibration"

    def __init__(
        self,
        *,
        parameter_overrides: list[Parameter] | None = None,
        adapter: PodHardwareAdapter | None = None,
        state_machine: DeviceManagerStateMachine | None = None,
        inventory_registry_path: str | Path | None = None,
    ) -> None:
        super().__init__("aeris_device_manager", parameter_overrides=parameter_overrides)

        queue_depth = int(self.declare_parameter("queue_depth", 10).value)
        poll_period_sec = float(self.declare_parameter("poll_period_sec", 0.5).value)
        enumeration_budget_sec = float(
            self.declare_parameter("enumeration_budget_sec", 15.0).value
        )
        registry_path_param = self.declare_parameter(
            "inventory_registry_path",
            "/tmp/aeris_device_manager_pod_inventory.json",
        ).value
        due_soon_window_days = float(
            self.declare_parameter("calibration_due_soon_window_days", 14.0).value
        )

        effective_adapter = adapter or NullPodHardwareAdapter()
        if state_machine is None:
            state_machine = DeviceManagerStateMachine(
                effective_adapter,
                clock=RosDeviceManagerClock(self.get_clock()),
                enumeration_budget_sec=enumeration_budget_sec,
            )
        self._state_machine = state_machine
        self._adapter = adapter or getattr(state_machine, "_adapter", effective_adapter)
        self._pods_publisher = self.create_publisher(
            PodStatusArray, self.PODS_TOPIC, queue_depth
        )
        self._inventory_publisher = self.create_publisher(
            PodInventoryArray, self.INVENTORY_TOPIC, queue_depth
        )
        self._inventory_registry = PodInventoryRegistry(
            registry_path=inventory_registry_path or registry_path_param,
            due_soon_window_sec=due_soon_window_days * 24 * 60 * 60,
        )
        self._latest_snapshots_by_serial: dict[str, PodStatusSnapshot] = {}
        self._calibration_service = self.create_service(
            LogPodCalibrationService,
            self.CALIBRATION_SERVICE,
            self._handle_calibration_command,
        )
        self._poll_timer = self.create_timer(poll_period_sec, self._poll_and_publish)

        self.get_logger().info(
            "Device manager ready: topic=%s, inventory=%s, service=%s, poll=%.3fs, budget=%.1fs"
            % (
                self.PODS_TOPIC,
                self.INVENTORY_TOPIC,
                self.CALIBRATION_SERVICE,
                poll_period_sec,
                enumeration_budget_sec,
            )
        )

    def _poll_and_publish(self) -> None:
        result = self._state_machine.reconcile()
        if result.transition_snapshots:
            for snapshots in result.transition_snapshots:
                self._pods_publisher.publish(self._to_status_array(snapshots))
        else:
            self._pods_publisher.publish(self._to_status_array(result.current))
        self._sync_inventory(result.current)

    def _sync_inventory(self, snapshots: tuple[PodStatusSnapshot, ...]) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        self._inventory_registry.sync_live_snapshots(snapshots, now_sec=now_sec)
        self._latest_snapshots_by_serial = {
            snapshot.pod_serial: snapshot for snapshot in snapshots if snapshot.pod_serial
        }
        self._inventory_publisher.publish(self._to_inventory_array(now_sec))

    def _to_status_array(
        self, snapshots: tuple[PodStatusSnapshot, ...]
    ) -> PodStatusArray:
        message = PodStatusArray()
        message.observed_at = self._seconds_to_time(self.get_clock().now().nanoseconds / 1e9)
        for snapshot in snapshots:
            message.pods.append(self._to_status(snapshot))
        return message

    def _to_inventory_array(self, observed_at_sec: float) -> PodInventoryArray:
        message = PodInventoryArray()
        message.observed_at = self._seconds_to_time(observed_at_sec)
        for record in self._inventory_registry.snapshot(now_sec=observed_at_sec):
            message.records.append(self._to_inventory_record(record))
        return message

    def _to_status(self, snapshot: PodStatusSnapshot) -> PodStatus:
        message = PodStatus()
        message.vehicle_id = snapshot.vehicle_id
        message.slot_id = snapshot.slot_id
        message.one_wire_id = snapshot.one_wire_id
        message.pod_serial = snapshot.pod_serial
        message.pod_type = snapshot.pod_type
        message.lifecycle_state = self._state_constant(snapshot.lifecycle_state)
        message.lifecycle_state_label = snapshot.lifecycle_state.value
        message.capabilities = list(snapshot.capabilities)
        message.connected = snapshot.connected
        message.power_ready = snapshot.power_ready
        message.link_ready = snapshot.link_ready
        message.first_seen = self._seconds_to_time(snapshot.first_seen_sec)
        message.last_seen = self._seconds_to_time(snapshot.last_seen_sec)
        message.enumeration_elapsed_sec = float(snapshot.enumeration_elapsed_sec)
        message.rejection_code = snapshot.rejection_code
        message.rejection_detail = snapshot.rejection_detail
        message.fault_code = snapshot.fault_code
        message.fault_detail = snapshot.fault_detail
        for timing in snapshot.stage_timings:
            stage_message = PodStageTimingMessage()
            stage_message.stage = timing.stage
            stage_message.elapsed_sec = float(timing.elapsed_sec)
            message.stage_timings.append(stage_message)
        return message

    def _to_inventory_record(self, record: InventoryRow) -> PodInventoryRecord:
        message = PodInventoryRecord()
        message.pod_serial = record.pod_serial
        message.pod_type = record.pod_type
        message.one_wire_id = record.one_wire_id
        message.attached = record.attached
        message.vehicle_id = record.vehicle_id
        message.slot_id = record.slot_id
        message.lifecycle_state_label = record.lifecycle_state_label
        message.connected = record.connected
        message.power_ready = record.power_ready
        message.link_ready = record.link_ready
        message.capabilities = list(record.capabilities)
        message.first_seen = self._seconds_to_time(record.first_seen_sec)
        message.last_seen = self._seconds_to_time(record.last_seen_sec)
        message.last_calibration = self._seconds_to_time(record.last_calibration_sec or 0.0)
        message.next_calibration_due = self._seconds_to_time(
            record.next_calibration_due_sec or 0.0
        )
        message.calibration_state = self._calibration_state_constant(record.calibration_state)
        message.calibration_state_label = record.calibration_state.value
        message.calibration_detail = record.calibration_detail
        return message

    def _handle_calibration_command(
        self,
        request: LogPodCalibrationService.Request,
        response: LogPodCalibrationService.Response,
    ) -> LogPodCalibrationService.Response:
        pod_serial = request.pod_serial.strip()
        if not pod_serial:
            response.accepted = False
            response.failure_code = "invalid_request"
            response.message = "Pod serial is required"
            return response

        last_calibration_sec = self._time_to_seconds(request.last_calibration)
        next_due_sec = self._time_to_seconds(request.next_calibration_due)
        if last_calibration_sec <= 0.0 or next_due_sec <= last_calibration_sec:
            response.accepted = False
            response.failure_code = "invalid_request"
            response.message = "Calibration dates must be ordered and non-empty"
            return response

        snapshot = self._latest_snapshots_by_serial.get(pod_serial)
        if snapshot is None or not snapshot.connected:
            response.accepted = False
            response.failure_code = "pod_not_connected"
            response.message = f"Pod {pod_serial} is not currently connected"
            return response

        try:
            verified = self._adapter.write_calibration(
                self._snapshot_to_detected_pod(snapshot),
                PodCalibrationMetadata(
                    last_calibration_sec=last_calibration_sec,
                    next_calibration_due_sec=next_due_sec,
                ),
            )
        except DeviceManagerError as error:
            response.accepted = False
            response.failure_code = error.code
            response.message = error.detail
            return response

        now_sec = self.get_clock().now().nanoseconds / 1e9
        updated = self._inventory_registry.apply_calibration_update(
            pod_serial=pod_serial,
            calibration=verified,
            now_sec=now_sec,
        )
        self._inventory_publisher.publish(self._to_inventory_array(now_sec))
        response.accepted = True
        response.failure_code = ""
        response.message = f"Calibration logged for {pod_serial}"
        response.record = self._to_inventory_record(updated)
        return response

    @staticmethod
    def _state_constant(state: PodLifecycleState) -> int:
        return {
            PodLifecycleState.DETECTED: PodStatus.STATE_DETECTED,
            PodLifecycleState.VALIDATING: PodStatus.STATE_VALIDATING,
            PodLifecycleState.POWER_CHECK: PodStatus.STATE_POWER_CHECK,
            PodLifecycleState.SOFT_START: PodStatus.STATE_SOFT_START,
            PodLifecycleState.ENUMERATING: PodStatus.STATE_ENUMERATING,
            PodLifecycleState.REGISTERED: PodStatus.STATE_REGISTERED,
            PodLifecycleState.REJECTED: PodStatus.STATE_REJECTED,
            PodLifecycleState.DISCONNECTED: PodStatus.STATE_DISCONNECTED,
            PodLifecycleState.FAULTED: PodStatus.STATE_FAULTED,
        }[state]

    @staticmethod
    def _calibration_state_constant(state: PodCalibrationState) -> int:
        return {
            PodCalibrationState.UNKNOWN: PodInventoryRecord.CALIBRATION_UNKNOWN,
            PodCalibrationState.CURRENT: PodInventoryRecord.CALIBRATION_CURRENT,
            PodCalibrationState.DUE_SOON: PodInventoryRecord.CALIBRATION_DUE_SOON,
            PodCalibrationState.OVERDUE: PodInventoryRecord.CALIBRATION_OVERDUE,
        }[state]

    @staticmethod
    def _seconds_to_time(value: float) -> Time:
        seconds = max(0.0, float(value))
        time_message = Time()
        time_message.sec = int(seconds)
        time_message.nanosec = int((seconds - time_message.sec) * 1_000_000_000)
        return time_message

    @staticmethod
    def _time_to_seconds(value: Time) -> float:
        return float(int(value.sec) + (int(value.nanosec) / 1_000_000_000))

    @staticmethod
    def _snapshot_to_detected_pod(snapshot: PodStatusSnapshot) -> DetectedPod:
        return DetectedPod(
            vehicle_id=snapshot.vehicle_id,
            slot_id=snapshot.slot_id,
            one_wire_id=snapshot.one_wire_id,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DeviceManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Device manager shutdown requested.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
