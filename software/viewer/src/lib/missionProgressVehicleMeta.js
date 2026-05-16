/**
 * Helpers for extracting per-vehicle assignment/progress metadata from
 * /mission/progress legacy JSON payloads.
 */

function createEmptyMeta() {
  return {
    assignments: {},
    assignmentLabels: {},
    progress: {},
    online: {},
    slamModes: {},
  };
}

export function normalizeVehicleId(value) {
  if (typeof value !== "string") {
    return "";
  }
  const normalized = value.trim().toLowerCase().replace(/-/g, "_");
  const match = normalized.match(/^([a-z]+)(\d+)$/);
  if (!match) {
    return normalized;
  }
  return `${match[1]}_${match[2]}`;
}

export function extractVehicleMissionMetaFromProgressPayload(rawDataOrParsed) {
  let parsed;
  if (typeof rawDataOrParsed === "string") {
    if (!rawDataOrParsed.trim().startsWith("{")) {
      return createEmptyMeta();
    }
    try {
      parsed = JSON.parse(rawDataOrParsed);
    } catch {
      return createEmptyMeta();
    }
  } else if (rawDataOrParsed && typeof rawDataOrParsed === "object") {
    parsed = rawDataOrParsed;
  } else {
    return createEmptyMeta();
  }

  const assignments = {};
  const assignmentLabels = {};
  const progress = {};
  const online = {};
  const slamModes = {};

  const rawAssignments = parsed?.vehicleAssignments;
  if (rawAssignments && typeof rawAssignments === "object") {
    for (const [vehicleId, assignment] of Object.entries(rawAssignments)) {
      if (typeof assignment !== "string") {
        continue;
      }
      const normalizedId = normalizeVehicleId(vehicleId);
      if (!normalizedId) {
        continue;
      }
      assignments[normalizedId] = assignment;
    }
  }

  const rawAssignmentLabels = parsed?.vehicleAssignmentLabels;
  if (rawAssignmentLabels && typeof rawAssignmentLabels === "object") {
    for (const [vehicleId, label] of Object.entries(rawAssignmentLabels)) {
      if (typeof label !== "string") {
        continue;
      }
      const normalizedId = normalizeVehicleId(vehicleId);
      if (!normalizedId) {
        continue;
      }
      assignmentLabels[normalizedId] = label;
    }
  }

  const rawProgress = parsed?.vehicleProgress;
  if (rawProgress && typeof rawProgress === "object") {
    for (const [vehicleId, value] of Object.entries(rawProgress)) {
      if (typeof value !== "number" || !Number.isFinite(value)) {
        continue;
      }
      const normalizedId = normalizeVehicleId(vehicleId);
      if (!normalizedId) {
        continue;
      }
      progress[normalizedId] = value;
    }
  }

  const rawOnline = parsed?.vehicleOnline;
  if (rawOnline && typeof rawOnline === "object") {
    for (const [vehicleId, value] of Object.entries(rawOnline)) {
      const normalizedId = normalizeVehicleId(vehicleId);
      if (!normalizedId) {
        continue;
      }
      online[normalizedId] = Boolean(value);
    }
  }

  const rawSlamModes = parsed?.vehicleSlamModes;
  if (rawSlamModes && typeof rawSlamModes === "object") {
    for (const [vehicleId, value] of Object.entries(rawSlamModes)) {
      if (typeof value !== "string") {
        continue;
      }
      const normalizedId = normalizeVehicleId(vehicleId);
      if (!normalizedId) {
        continue;
      }
      slamModes[normalizedId] = value.trim().toLowerCase();
    }
  }

  return {
    assignments,
    assignmentLabels,
    progress,
    online,
    slamModes,
  };
}
