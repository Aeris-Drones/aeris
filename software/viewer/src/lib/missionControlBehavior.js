const ACTIVE_PHASES = new Set(["SEARCHING", "TRACKING"]);

export function isMissionPaused(phase, pausedAt) {
  return ACTIVE_PHASES.has(phase) && pausedAt !== undefined;
}

export function getAbortMissionUnavailableReason({
  phase,
  rosConnected,
  missionId,
}) {
  const abortableForPhase = ACTIVE_PHASES.has(phase);

  if (!abortableForPhase) {
    return "Emergency stop is unavailable until a mission is active.";
  }

  if (!rosConnected) {
    return "ROS is disconnected. Reconnect before aborting the mission.";
  }

  if (!missionId || !missionId.trim()) {
    return "Mission abort failed: active mission id is missing.";
  }

  return null;
}

export function computeMissionControlFlags({
  phase,
  pausedAt,
  hasValidStartZone,
  rosConnected,
  missionId,
}) {
  const isActive = ACTIVE_PHASES.has(phase);
  const isPaused = isMissionPaused(phase, pausedAt);
  const isComplete = phase === "COMPLETE";
  const isIdle = phase === "IDLE";
  const abortUnavailableReason = getAbortMissionUnavailableReason({
    phase,
    rosConnected,
    missionId,
  });

  return {
    isActive,
    isPaused,
    isComplete,
    canStart: isIdle && !isPaused && hasValidStartZone && rosConnected,
    canPause: isActive && !isPaused,
    canResume: isPaused,
    canAbort: abortUnavailableReason === null,
  };
}

export function getAbortMissionValidationError({
  phase = "SEARCHING",
  rosConnected,
  missionId,
}) {
  return getAbortMissionUnavailableReason({
    phase,
    rosConnected,
    missionId,
  });
}

export function withServiceTimeout(invoke, timeoutMs, label = "service call") {
  return new Promise((resolve, reject) => {
    let settled = false;

    const finish = (callback, value) => {
      if (settled) {
        return;
      }
      settled = true;
      clearTimeout(timeoutId);
      callback(value);
    };

    const timeoutId = setTimeout(() => {
      finish(
        reject,
        new Error(`${label} timed out after ${timeoutMs}ms`)
      );
    }, timeoutMs);

    invoke(
      value => finish(resolve, value),
      error => finish(reject, error instanceof Error ? error : new Error(String(error)))
    );
  });
}
