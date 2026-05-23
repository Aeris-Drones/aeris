"""ROS 2 node wrapper for the Aeris device manager state machine."""

import time

import rclpy
from aeris_msgs.msg import PodStageTiming as PodStageTimingMessage
from aeris_msgs.msg import PodStatus, PodStatusArray
from builtin_interfaces.msg import Time
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.parameter import Parameter

from .adapters import NullPodHardwareAdapter, PodHardwareAdapter
from .models import PodLifecycleState, PodStatusSnapshot
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

    def __init__(
        self,
        *,
        parameter_overrides: list[Parameter] | None = None,
        adapter: PodHardwareAdapter | None = None,
        state_machine: DeviceManagerStateMachine | None = None,
    ) -> None:
        super().__init__("aeris_device_manager", parameter_overrides=parameter_overrides)

        queue_depth = int(self.declare_parameter("queue_depth", 10).value)
        poll_period_sec = float(
            self.declare_parameter("poll_period_sec", 0.5).value
        )
        enumeration_budget_sec = float(
            self.declare_parameter("enumeration_budget_sec", 15.0).value
        )

        if state_machine is None:
            state_machine = DeviceManagerStateMachine(
                adapter or NullPodHardwareAdapter(),
                clock=RosDeviceManagerClock(self.get_clock()),
                enumeration_budget_sec=enumeration_budget_sec,
            )
        self._state_machine = state_machine
        self._pods_publisher = self.create_publisher(
            PodStatusArray, self.PODS_TOPIC, queue_depth
        )
        self._poll_timer = self.create_timer(poll_period_sec, self._poll_and_publish)

        self.get_logger().info(
            "Device manager ready: topic=%s, poll=%.3fs, budget=%.1fs"
            % (self.PODS_TOPIC, poll_period_sec, enumeration_budget_sec)
        )

    def _poll_and_publish(self) -> None:
        result = self._state_machine.reconcile()
        if result.transition_snapshots:
            for snapshots in result.transition_snapshots:
                self._pods_publisher.publish(self._to_status_array(snapshots))
            return
        self._pods_publisher.publish(self._to_status_array(result.current))

    def _to_status_array(
        self, snapshots: tuple[PodStatusSnapshot, ...]
    ) -> PodStatusArray:
        message = PodStatusArray()
        message.observed_at = self._seconds_to_time(self.get_clock().now().nanoseconds / 1e9)
        for snapshot in snapshots:
            message.pods.append(self._to_status(snapshot))
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
    def _seconds_to_time(value: float) -> Time:
        seconds = max(0.0, float(value))
        time_message = Time()
        time_message.sec = int(seconds)
        time_message.nanosec = int((seconds - time_message.sec) * 1_000_000_000)
        return time_message


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
