export const HOLD_TO_ABORT_MS = 1500;
const ACTIVE_ABORT_PHASES = new Set(["SEARCHING", "TRACKING"]);

export function isActiveAbortPhase(missionPhase) {
  return ACTIVE_ABORT_PHASES.has(missionPhase);
}

export function advanceAbortRequestWindow(currentWindow, missionPhase) {
  const previous = currentWindow ?? { id: 0, isActive: false };
  const nextIsActive = isActiveAbortPhase(missionPhase);

  if (nextIsActive && !previous.isActive) {
    return {
      id: previous.id + 1,
      isActive: true,
    };
  }

  if (!nextIsActive && previous.isActive) {
    return {
      id: previous.id,
      isActive: false,
    };
  }

  return {
    id: previous.id,
    isActive: nextIsActive,
  };
}

export function createIdleEmergencyStopHold() {
  return {
    phase: "idle",
    progress: 0,
  };
}

export function beginEmergencyStopHold(now) {
  return {
    phase: "holding",
    startedAt: now,
    progress: 0,
  };
}

export function cancelEmergencyStopHold() {
  return createIdleEmergencyStopHold();
}

export function cancelEmergencyStopHoldState(state) {
  return state.phase === "holding" ? cancelEmergencyStopHold() : state;
}

export function isAbortRequestPending({
  abortWindow,
  pendingWindowId,
  resolvedWindowId,
  abortError,
}) {
  return (
    pendingWindowId !== null &&
    pendingWindowId === abortWindow.id &&
    resolvedWindowId !== pendingWindowId &&
    !abortError &&
    abortWindow.isActive
  );
}

export function resolveAbortRequestWindowId({
  abortWindow,
  pendingWindowId,
  resolvedWindowId,
  abortError,
}) {
  if (pendingWindowId === null || pendingWindowId !== abortWindow.id) {
    return resolvedWindowId;
  }

  if (abortError || !abortWindow.isActive) {
    return pendingWindowId;
  }

  return resolvedWindowId;
}

export function advanceEmergencyStopHold({
  state,
  now,
  canAbort,
  abortPending,
  holdDurationMs = HOLD_TO_ABORT_MS,
}) {
  if (state.phase !== "holding") {
    return {
      nextState: state,
      shouldDispatchAbort: false,
    };
  }

  if (!canAbort || abortPending) {
    return {
      nextState: createIdleEmergencyStopHold(),
      shouldDispatchAbort: false,
    };
  }

  const nextState = tickEmergencyStopHold(state, now, holdDurationMs);
  return {
    nextState:
      nextState.phase === "completed" ? createIdleEmergencyStopHold() : nextState,
    shouldDispatchAbort: nextState.phase === "completed",
  };
}

export function tickEmergencyStopHold(state, now, holdDurationMs = HOLD_TO_ABORT_MS) {
  if (state.phase !== "holding") {
    return state;
  }

  const elapsedMs = Math.max(0, now - state.startedAt);
  const progress = Math.min(elapsedMs / holdDurationMs, 1);

  if (progress >= 1) {
    return {
      phase: "completed",
      progress: 1,
      startedAt: state.startedAt,
      completedAt: now,
    };
  }

  return {
    ...state,
    progress,
  };
}
