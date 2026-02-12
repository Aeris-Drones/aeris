/**
 * Fleet vehicle presentation helpers used by FleetPanel/VehicleCard rendering.
 */

export function applyVehicleMissionMeta(vehicleInfo, meta = {}) {
  const next = { ...vehicleInfo };

  const commandStatusHint = meta?.commandStatusHint;
  if (next.status !== "offline" && commandStatusHint) {
    next.status = commandStatusHint;
  }

  if (meta?.online === false) {
    next.status = "offline";
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
