const ACTIVE_PHASES = new Set(["SEARCHING", "TRACKING"]);

export function isMissionPaused(phase, pausedAt) {
  return ACTIVE_PHASES.has(phase) && pausedAt !== undefined;
}

export function computeMissionControlFlags({
  phase,
  pausedAt,
  hasValidStartZone,
  rosConnected,
}) {
  const isActive = ACTIVE_PHASES.has(phase);
  const isPaused = isMissionPaused(phase, pausedAt);
  const isComplete = phase === "COMPLETE";
  const isIdle = phase === "IDLE";

  return {
    isActive,
    isPaused,
    isComplete,
    canStart: isIdle && !isPaused && hasValidStartZone && rosConnected,
    canPause: isActive && !isPaused,
    canResume: isPaused,
    canAbort: isActive || isPaused,
  };
}

export function getAbortMissionValidationError({ rosConnected, missionId }) {
  if (!rosConnected) {
    return "ROS is disconnected. Reconnect before aborting the mission.";
  }

  if (!missionId || !missionId.trim()) {
    return "Mission abort failed: active mission id is missing.";
  }

  return null;
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
