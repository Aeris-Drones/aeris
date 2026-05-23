"""Deterministic pod lifecycle controller for the Aeris device manager."""

from dataclasses import dataclass, field
from typing import Protocol

from .adapters import (
    DeviceManagerError,
    LinkEnumerationError,
    PodHardwareAdapter,
    SoftStartError,
)
from .models import (
    DetectedPod,
    PodLifecycleState,
    PodLinkInfo,
    PodMetadata,
    PodStageTiming,
    PodStatusSnapshot,
)


class DeviceManagerClock(Protocol):
    """Minimal clock abstraction for deterministic time and timestamp handling."""

    def monotonic(self) -> float:
        ...

    def wall_time(self) -> float:
        ...


@dataclass(frozen=True)
class ReconcileResult:
    current: tuple[PodStatusSnapshot, ...]
    transition_snapshots: tuple[tuple[PodStatusSnapshot, ...], ...]


@dataclass
class _PodRuntime:
    detected_pod: DetectedPod
    first_seen_sec: float
    last_seen_sec: float
    enumeration_started_monotonic: float
    lifecycle_state: PodLifecycleState = PodLifecycleState.DETECTED
    metadata: PodMetadata | None = None
    link: PodLinkInfo | None = None
    capabilities: tuple[str, ...] = ()
    connected: bool = True
    power_ready: bool = False
    link_ready: bool = False
    stage_timings: list[PodStageTiming] = field(default_factory=list)
    rejection_code: str = ""
    rejection_detail: str = ""
    fault_code: str = ""
    fault_detail: str = ""


class DeviceManagerStateMachine:
    """Runs the pod enumeration sequence in the architecture-defined order."""

    _ENUMERATION_STAGES = (
        "detect",
        "eeprom_crc",
        "power_budget",
        "soft_start",
        "link_enumeration",
        "capability_registration",
    )

    def __init__(
        self,
        adapter: PodHardwareAdapter,
        *,
        clock: DeviceManagerClock,
        enumeration_budget_sec: float = 15.0,
    ) -> None:
        self._adapter = adapter
        self._clock = clock
        self._enumeration_budget_sec = float(max(0.1, enumeration_budget_sec))
        self._runtimes: dict[tuple[str, str], _PodRuntime] = {}

    def reconcile(self) -> ReconcileResult:
        transitions: list[tuple[PodStatusSnapshot, ...]] = []
        detected_pods = sorted(self._adapter.scan(), key=self._pod_key)
        detected_keys = {self._pod_key(pod) for pod in detected_pods}

        for key in sorted(self._runtimes.keys()):
            runtime = self._runtimes[key]
            if key not in detected_keys and runtime.connected:
                self._mark_disconnected(runtime, transitions)

        for pod in detected_pods:
            key = self._pod_key(pod)
            runtime = self._runtimes.get(key)
            if runtime is None or runtime.lifecycle_state is PodLifecycleState.DISCONNECTED:
                runtime = self._new_runtime(pod)
                self._runtimes[key] = runtime
                self._emit_transition(runtime, PodLifecycleState.DETECTED, transitions)
                self._run_enumeration(runtime, transitions)
                continue

            runtime.last_seen_sec = self._clock.wall_time()
            runtime.connected = True

        return ReconcileResult(
            current=self._current_snapshots(),
            transition_snapshots=tuple(transitions),
        )

    def _new_runtime(self, pod: DetectedPod) -> _PodRuntime:
        now_monotonic = self._clock.monotonic()
        now_wall = self._clock.wall_time()
        return _PodRuntime(
            detected_pod=pod,
            first_seen_sec=now_wall,
            last_seen_sec=now_wall,
            enumeration_started_monotonic=now_monotonic,
            stage_timings=[PodStageTiming(stage="detect", elapsed_sec=0.0)],
        )

    def _run_enumeration(
        self,
        runtime: _PodRuntime,
        transitions: list[tuple[PodStatusSnapshot, ...]],
    ) -> None:
        pod = runtime.detected_pod
        self._emit_transition(runtime, PodLifecycleState.VALIDATING, transitions)
        try:
            metadata = self._timed_stage(
                runtime, "eeprom_crc", lambda: self._adapter.read_eeprom(pod)
            )
        except DeviceManagerError as error:
            self._reject(runtime, error, transitions)
            return
        runtime.metadata = metadata
        if self._fault_if_budget_exceeded(runtime, "eeprom_crc", transitions):
            return

        self._emit_transition(runtime, PodLifecycleState.POWER_CHECK, transitions)
        try:
            self._timed_stage(
                runtime,
                "power_budget",
                lambda: self._adapter.check_power_budget(pod, metadata),
            )
        except DeviceManagerError as error:
            self._reject(runtime, error, transitions)
            return
        if self._fault_if_budget_exceeded(runtime, "power_budget", transitions):
            return

        self._emit_transition(runtime, PodLifecycleState.SOFT_START, transitions)
        try:
            self._timed_stage(
                runtime,
                "soft_start",
                lambda: self._adapter.soft_start(pod, metadata),
            )
        except DeviceManagerError as error:
            self._fault(runtime, error.code, error.detail, transitions)
            return
        runtime.power_ready = True
        if self._fault_if_budget_exceeded(runtime, "soft_start", transitions):
            return

        self._emit_transition(runtime, PodLifecycleState.ENUMERATING, transitions)
        try:
            link = self._timed_stage(
                runtime,
                "link_enumeration",
                lambda: self._adapter.enumerate_link(pod, metadata),
            )
        except (LinkEnumerationError, SoftStartError, DeviceManagerError) as error:
            self._fault(runtime, error.code, error.detail, transitions)
            return
        runtime.link = link
        runtime.link_ready = True
        if self._fault_if_budget_exceeded(runtime, "link_enumeration", transitions):
            return

        try:
            capabilities = self._timed_stage(
                runtime,
                "capability_registration",
                lambda: self._adapter.register_capabilities(pod, metadata, link),
            )
        except DeviceManagerError as error:
            self._fault(runtime, error.code, error.detail, transitions)
            return
        runtime.capabilities = tuple(self._unique_capabilities(capabilities))
        if self._fault_if_budget_exceeded(runtime, "capability_registration", transitions):
            return

        runtime.rejection_code = ""
        runtime.rejection_detail = ""
        runtime.fault_code = ""
        runtime.fault_detail = ""
        self._emit_transition(runtime, PodLifecycleState.REGISTERED, transitions)

    def _timed_stage(self, runtime: _PodRuntime, stage: str, action):
        started = self._clock.monotonic()
        result = action()
        elapsed = self._clock.monotonic() - started
        runtime.last_seen_sec = self._clock.wall_time()
        runtime.stage_timings.append(
            PodStageTiming(stage=stage, elapsed_sec=float(max(0.0, elapsed)))
        )
        return result

    def _fault_if_budget_exceeded(
        self,
        runtime: _PodRuntime,
        stage: str,
        transitions: list[tuple[PodStatusSnapshot, ...]],
    ) -> bool:
        elapsed = self._enumeration_elapsed(runtime)
        if elapsed <= self._enumeration_budget_sec:
            return False
        self._fault(
            runtime,
            "enumeration_timeout",
            (
                f"enumeration exceeded {self._enumeration_budget_sec:.1f}s budget "
                f"after {stage} ({elapsed:.2f}s)"
            ),
            transitions,
        )
        return True

    def _reject(
        self,
        runtime: _PodRuntime,
        error: DeviceManagerError,
        transitions: list[tuple[PodStatusSnapshot, ...]],
    ) -> None:
        runtime.rejection_code = error.code
        runtime.rejection_detail = error.detail
        runtime.fault_code = ""
        runtime.fault_detail = ""
        self._emit_transition(runtime, PodLifecycleState.REJECTED, transitions)

    def _fault(
        self,
        runtime: _PodRuntime,
        code: str,
        detail: str,
        transitions: list[tuple[PodStatusSnapshot, ...]],
    ) -> None:
        if runtime.power_ready or runtime.link_ready:
            self._adapter.power_off(runtime.detected_pod)
        runtime.power_ready = False
        runtime.link_ready = False
        runtime.fault_code = code
        runtime.fault_detail = detail
        runtime.rejection_code = ""
        runtime.rejection_detail = ""
        self._emit_transition(runtime, PodLifecycleState.FAULTED, transitions)

    def _mark_disconnected(
        self,
        runtime: _PodRuntime,
        transitions: list[tuple[PodStatusSnapshot, ...]],
    ) -> None:
        if runtime.power_ready or runtime.link_ready:
            self._adapter.power_off(runtime.detected_pod)
        runtime.connected = False
        runtime.power_ready = False
        runtime.link_ready = False
        runtime.last_seen_sec = self._clock.wall_time()
        self._emit_transition(runtime, PodLifecycleState.DISCONNECTED, transitions)

    def _emit_transition(
        self,
        runtime: _PodRuntime,
        new_state: PodLifecycleState,
        transitions: list[tuple[PodStatusSnapshot, ...]],
    ) -> None:
        runtime.lifecycle_state = new_state
        runtime.last_seen_sec = self._clock.wall_time()
        transitions.append(self._current_snapshots())

    def _current_snapshots(self) -> tuple[PodStatusSnapshot, ...]:
        snapshots = [
            self._snapshot(runtime)
            for key, runtime in sorted(self._runtimes.items(), key=lambda item: item[0])
        ]
        return tuple(snapshots)

    def _snapshot(self, runtime: _PodRuntime) -> PodStatusSnapshot:
        metadata = runtime.metadata
        return PodStatusSnapshot(
            vehicle_id=runtime.detected_pod.vehicle_id,
            slot_id=runtime.detected_pod.slot_id,
            one_wire_id=runtime.detected_pod.one_wire_id,
            pod_serial=metadata.serial if metadata is not None else "",
            pod_type=metadata.pod_type if metadata is not None else "",
            lifecycle_state=runtime.lifecycle_state,
            capabilities=tuple(runtime.capabilities),
            connected=runtime.connected,
            power_ready=runtime.power_ready,
            link_ready=runtime.link_ready,
            first_seen_sec=runtime.first_seen_sec,
            last_seen_sec=runtime.last_seen_sec,
            enumeration_elapsed_sec=self._enumeration_elapsed(runtime),
            stage_timings=tuple(runtime.stage_timings),
            rejection_code=runtime.rejection_code,
            rejection_detail=runtime.rejection_detail,
            fault_code=runtime.fault_code,
            fault_detail=runtime.fault_detail,
        )

    def _enumeration_elapsed(self, runtime: _PodRuntime) -> float:
        return float(
            max(0.0, self._clock.monotonic() - runtime.enumeration_started_monotonic)
        )

    @staticmethod
    def _pod_key(pod: DetectedPod) -> tuple[str, str]:
        return (pod.vehicle_id.strip(), pod.slot_id.strip())

    @staticmethod
    def _unique_capabilities(capabilities: tuple[str, ...] | list[str]) -> tuple[str, ...]:
        seen: dict[str, None] = {}
        for capability in capabilities:
            value = str(capability).strip()
            if value:
                seen.setdefault(value, None)
        return tuple(seen.keys())
