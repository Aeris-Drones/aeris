/**
 * Fleet vehicle presentation helpers used by FleetPanel/VehicleCard rendering.
 */

import { deriveVehicleDegradedState } from "./degradedVehicleState.js";

export function applyVehicleMissionMeta(vehicleInfo, meta = {}) {
  const next = { ...vehicleInfo };
  const degraded = deriveVehicleDegradedState({
    lastUpdate: next.lastUpdate,
    missionOnline: meta?.online,
    deliveryMode: next.deliveryMode,
    isRetroactive: next.isRetroactive,
  });
  const alreadyOffline = next.status === "offline";
  next.status = alreadyOffline ? "offline" : degraded.status;
  next.isOffline = alreadyOffline || degraded.offline;
  next.lastContactAgeMs = degraded.lastContactAgeMs;
  next.staleSinceMs = degraded.staleSinceMs;
  next.isLastKnown = (alreadyOffline || degraded.offline) && degraded.retained;

  const commandStatusHint = meta?.commandStatusHint;
  if (!alreadyOffline && !degraded.offline && commandStatusHint) {
    next.status = commandStatusHint;
  }

  const assignmentLabel =
    typeof meta?.assignmentLabel === "string" && meta.assignmentLabel.trim()
      ? meta.assignmentLabel.trim()
      : typeof meta?.assignment === "string" && meta.assignment.trim()
        ? meta.assignment.trim()
        : "";
  if (assignmentLabel) {
    next.assignment = assignmentLabel;
  }

  if (typeof meta?.progress === "number" && Number.isFinite(meta.progress)) {
    next.missionProgressPercent = meta.progress;
  }

  if (typeof meta?.slamMode === "string" && meta.slamMode.trim()) {
    next.slamMode = meta.slamMode.trim().toLowerCase();
  }

  return next;
}
