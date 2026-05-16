export const DEFAULT_TELEMETRY_STALE_TIMEOUT_MS = 5000;
export const DEFAULT_LAST_KNOWN_RETENTION_MS = 120000;

export function normalizeMissionMetaForVehicle(vehicleId, missionMeta = {}) {
  const normalizedId = String(vehicleId ?? "").trim().toLowerCase().replace(/-/g, "_");
  return {
    assignment:
      missionMeta.assignments?.[normalizedId] ??
      missionMeta.assignments?.[vehicleId],
    assignmentLabel:
      missionMeta.assignmentLabels?.[normalizedId] ??
      missionMeta.assignmentLabels?.[vehicleId],
    progress:
      missionMeta.progress?.[normalizedId] ??
      missionMeta.progress?.[vehicleId],
    online:
      missionMeta.online?.[normalizedId] ??
      missionMeta.online?.[vehicleId],
    slamMode:
      missionMeta.slamModes?.[normalizedId] ??
      missionMeta.slamModes?.[vehicleId],
  };
}

export function deriveVehicleDegradedState({
  lastUpdate,
  missionOnline,
  deliveryMode,
  isRetroactive,
  now = Date.now(),
  staleTimeoutMs = DEFAULT_TELEMETRY_STALE_TIMEOUT_MS,
  retentionMs = DEFAULT_LAST_KNOWN_RETENTION_MS,
} = {}) {
  const ageMs = Number.isFinite(lastUpdate) ? Math.max(0, now - lastUpdate) : Infinity;
  const telemetryStale = ageMs > staleTimeoutMs;
  const missionOffline = missionOnline === false;
  const offline = telemetryStale || missionOffline;
  const retained = ageMs <= retentionMs;
  const replayed = deliveryMode === "replayed" || isRetroactive === true;

  return {
    status: offline ? "offline" : replayed ? "warning" : "active",
    offline,
    retained,
    replayed,
    lastContactAgeMs: Number.isFinite(ageMs) ? ageMs : null,
    staleSinceMs: offline && Number.isFinite(lastUpdate) ? lastUpdate + staleTimeoutMs : null,
  };
}

export function formatLastContactAge(ageMs) {
  if (!Number.isFinite(ageMs) || ageMs < 0) {
    return "--";
  }
  const totalSeconds = Math.floor(ageMs / 1000);
  if (totalSeconds < 60) {
    return `${totalSeconds}s`;
  }
  const minutes = Math.floor(totalSeconds / 60);
  const seconds = totalSeconds % 60;
  return `${minutes}m ${seconds.toString().padStart(2, "0")}s`;
}
