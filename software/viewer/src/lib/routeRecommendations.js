export const DEFAULT_ROUTE_STAGING_AREA = {
  id: "responder-staging",
  label: "Responder staging",
  position: [0, 0, 0],
};

const SURVIVOR_CONFIDENCE_THRESHOLD = 0.75;
const STALE_INPUT_MS = 2 * 60 * 1000;
const ROUTE_CLEARANCE_METERS = 18;

function isFinitePoint(point) {
  return Array.isArray(point) &&
    point.length === 3 &&
    point.every((value) => typeof value === "number" && Number.isFinite(value));
}

function isDismissed(detection) {
  return detection?.status === "dismissed";
}

function modalitySet(detection) {
  return new Set(Array.isArray(detection?.sourceModalities) ? detection.sourceModalities : []);
}

function normalizeBlockerType(detection) {
  const value = typeof detection?.routeBlockerType === "string"
    ? detection.routeBlockerType.trim().toLowerCase()
    : "";
  if (value === "gas" || value === "structural") {
    return value;
  }
  if (detection?.sensorType === "gas") {
    return "gas";
  }
  return null;
}

function isHazardOnlyDetection(detection) {
  const modalities = modalitySet(detection);
  if (detection?.sensorType === "gas") return true;
  if (modalities.size > 0 && [...modalities].every((modality) => modality === "gas")) return true;
  return normalizeBlockerType(detection) !== null;
}

function isHighConfidence(detection) {
  if (detection?.confidenceLevel === "HIGH") return true;
  return typeof detection?.confidence === "number" &&
    Number.isFinite(detection.confidence) &&
    detection.confidence >= SURVIVOR_CONFIDENCE_THRESHOLD;
}

function detectionLabel(detection) {
  if (typeof detection?.signatureType === "string" && detection.signatureType.trim()) {
    return detection.signatureType.trim();
  }
  return `${detection?.sensorType ?? "unknown"} detection`;
}

export function selectSurvivorRouteTargets(detections) {
  if (!Array.isArray(detections)) {
    return [];
  }

  return detections
    .filter((detection) => {
      if (!detection || isDismissed(detection)) return false;
      if (!isFinitePoint(detection.position)) return false;
      if (isHazardOnlyDetection(detection)) return false;
      return isHighConfidence(detection);
    })
    .map((detection) => ({
      id: detection.id,
      label: detectionLabel(detection),
      position: detection.position,
      detection,
    }));
}

function boundsForGeometry(points) {
  const validPoints = points.filter(isFinitePoint);
  if (validPoints.length < 3) return null;

  return validPoints.reduce(
    (bounds, point) => ({
      minX: Math.min(bounds.minX, point[0]),
      maxX: Math.max(bounds.maxX, point[0]),
      minZ: Math.min(bounds.minZ, point[2]),
      maxZ: Math.max(bounds.maxZ, point[2]),
    }),
    { minX: Infinity, maxX: -Infinity, minZ: Infinity, maxZ: -Infinity }
  );
}

export function extractRouteBlockers(detections) {
  if (!Array.isArray(detections)) {
    return [];
  }

  return detections
    .map((detection) => {
      const type = normalizeBlockerType(detection);
      const geometry = Array.isArray(detection?.geometry) ? detection.geometry.filter(isFinitePoint) : [];
      const bounds = boundsForGeometry(geometry);
      if (!type || !bounds) return null;
      return {
        id: detection.id,
        type,
        label: detectionLabel(detection),
        geometry,
        bounds,
        freshness: freshnessForInputs([detection]),
        sourceDetection: detection,
      };
    })
    .filter((blocker) => blocker !== null);
}

function freshnessForInputs(inputs, nowMs = Date.now()) {
  const replayed = inputs.some(
    (input) => input?.deliveryMode === "replayed" || input?.isRetroactive === true
  );
  if (replayed) {
    return { source: "replayed", ageMs: null };
  }

  const timestamps = inputs
    .map((input) => input?.timestamp)
    .filter((timestamp) => typeof timestamp === "number" && Number.isFinite(timestamp));
  if (timestamps.length === 0) {
    return { source: "unknown", ageMs: null };
  }

  const newest = Math.max(...timestamps);
  const ageMs = Math.max(0, nowMs - newest);
  return {
    source: ageMs > STALE_INPUT_MS ? "stale" : "live",
    ageMs,
  };
}

function segmentIntersectsBounds(start, end, bounds) {
  const minX = Math.min(start[0], end[0]);
  const maxX = Math.max(start[0], end[0]);
  const minZ = Math.min(start[2], end[2]);
  const maxZ = Math.max(start[2], end[2]);

  return maxX >= bounds.minX &&
    minX <= bounds.maxX &&
    maxZ >= bounds.minZ &&
    minZ <= bounds.maxZ;
}

function buildPolylineAroundBlockers(start, end, blockers) {
  const blocking = blockers.filter((blocker) => segmentIntersectsBounds(start, end, blocker.bounds));
  if (blocking.length === 0) {
    return { polyline: [start, end], blocking };
  }

  const blockerBounds = blocking.reduce(
    (bounds, blocker) => ({
      minX: Math.min(bounds.minX, blocker.bounds.minX),
      maxX: Math.max(bounds.maxX, blocker.bounds.maxX),
      minZ: Math.min(bounds.minZ, blocker.bounds.minZ),
      maxZ: Math.max(bounds.maxZ, blocker.bounds.maxZ),
    }),
    { minX: Infinity, maxX: -Infinity, minZ: Infinity, maxZ: -Infinity }
  );
  const upperZ = blockerBounds.maxZ + ROUTE_CLEARANCE_METERS;
  const lowerZ = blockerBounds.minZ - ROUTE_CLEARANCE_METERS;
  const startToUpper = Math.abs(start[2] - upperZ) + Math.abs(end[2] - upperZ);
  const startToLower = Math.abs(start[2] - lowerZ) + Math.abs(end[2] - lowerZ);
  const doglegZ = startToUpper <= startToLower ? upperZ : lowerZ;

  return {
    polyline: [
      start,
      [blockerBounds.minX - ROUTE_CLEARANCE_METERS, start[1], doglegZ],
      [blockerBounds.maxX + ROUTE_CLEARANCE_METERS, end[1], doglegZ],
      end,
    ],
    blocking,
  };
}

export function deriveRouteRecommendations({
  detections,
  stagingArea = DEFAULT_ROUTE_STAGING_AREA,
  nowMs = Date.now(),
} = {}) {
  if (!stagingArea || !isFinitePoint(stagingArea.position)) {
    return [];
  }

  const targets = selectSurvivorRouteTargets(detections);
  const blockers = extractRouteBlockers(detections);

  return targets.map((target) => {
    const { polyline, blocking } = buildPolylineAroundBlockers(
      stagingArea.position,
      target.position,
      blockers
    );
    const freshness = freshnessForInputs(
      [target.detection, ...blocking.map((blocker) => blocker.sourceDetection)],
      nowMs
    );
    const status = freshness.source === "live"
      ? "clear"
      : freshness.source === "unknown"
        ? "pending"
        : "stale";

    return {
      id: `route-${stagingArea.id}-${target.id}`,
      sourceLabel: stagingArea.label,
      destinationLabel: target.label,
      targetDetectionId: target.id,
      polyline,
      blockingHazardIds: blocking.map((blocker) => blocker.id),
      freshness,
      status,
    };
  });
}
