import { normalizeModalities } from "../modalities.js";
const MODALITY_PRIORITY = ["gas", "thermal", "acoustic"];

function isFiniteNumber(value) {
  return typeof value === "number" && Number.isFinite(value);
}

function clamp01(value) {
  if (!Number.isFinite(value)) {
    return 0;
  }
  return Math.max(0, Math.min(1, value));
}

function parseConfidence(rawConfidence) {
  const confidence = Number(rawConfidence);
  if (!isFiniteNumber(confidence)) {
    throw new Error("Invalid fused detection message: confidence must be numeric");
  }
  return clamp01(confidence);
}

function normalizeConfidenceLevel(rawLevel, confidence) {
  const level = typeof rawLevel === "string" ? rawLevel.trim().toUpperCase() : "";
  if (level === "HIGH" || level === "MEDIUM" || level === "LOW") {
    return level;
  }
  if (confidence >= 0.85) {
    return "HIGH";
  }
  if (confidence >= 0.6) {
    return "MEDIUM";
  }
  if (confidence >= 0.3) {
    return "LOW";
  }
  return "UNKNOWN";
}

function parseTimestampMs(rawStamp, nowMs) {
  const stamp = rawStamp && typeof rawStamp === "object" ? rawStamp : {};
  const sec = Number(stamp.sec);
  const nanosec = Number(stamp.nanosec);
  if (isFiniteNumber(sec) && sec > 0) {
    const safeNanosec = isFiniteNumber(nanosec) ? nanosec : 0;
    return (sec * 1000) + (safeNanosec / 1_000_000);
  }
  return nowMs;
}

function parseEpochMs(rawValue, fallbackMs = null) {
  if (typeof rawValue === "number" && Number.isFinite(rawValue)) {
    return rawValue > 1_000_000_000_000 ? rawValue : rawValue * 1000;
  }
  if (rawValue && typeof rawValue === "object") {
    const sec = Number(rawValue.sec);
    const nanosec = Number(rawValue.nanosec);
    if (Number.isFinite(sec)) {
      const safeNanosec = Number.isFinite(nanosec) ? nanosec : 0;
      return (sec * 1000) + (safeNanosec / 1_000_000);
    }
  }
  return fallbackMs;
}

function parseReplayMetadata(rawMessage, fallbackOriginalTsMs) {
  const nested = rawMessage?.replay_metadata;
  const source = nested && typeof nested === "object" ? nested : rawMessage;
  const hasReplayFields =
    source &&
    typeof source === "object" &&
    (
      "delivery_mode" in source ||
      "original_event_ts" in source ||
      "replayed_at_ts" in source
    );
  if (!hasReplayFields) {
    return null;
  }

  const deliveryMode = typeof source.delivery_mode === "string" &&
    source.delivery_mode.trim().toLowerCase() === "replayed"
    ? "replayed"
    : "live";
  const originalEventTs = parseEpochMs(source.original_event_ts, fallbackOriginalTsMs);
  const replayedAtTs = deliveryMode === "replayed"
    ? parseEpochMs(source.replayed_at_ts, null)
    : null;

  return {
    deliveryMode,
    originalEventTs,
    replayedAtTs,
    isRetroactive: deliveryMode === "replayed",
  };
}

function parsePoint(rawPoint, pointName) {
  if (!rawPoint || typeof rawPoint !== "object") {
    throw new Error(`Invalid fused detection message: ${pointName} is missing`);
  }

  const point = rawPoint;
  const x = Number(point.x);
  const y = Number(point.y);
  const z = Number(point.z);

  if (!isFiniteNumber(x) || !isFiniteNumber(y) || !isFiniteNumber(z)) {
    throw new Error(`Invalid fused detection message: ${pointName} requires numeric x/y/z`);
  }

  return [x, y, z];
}

function parseGeometryFromLocalGeometry(rawGeometry) {
  if (!Array.isArray(rawGeometry)) {
    return [];
  }

  return rawGeometry
    .map((rawPoint) => {
      try {
        const [x, y, z] = parsePoint(rawPoint, "local_geometry point");
        // ROS Point32 for fused geometry uses (x, local_z, altitude),
        // while viewer map coordinates are [x, up(y), z].
        return [x, z, y];
      } catch {
        return null;
      }
    })
    .filter((point) => point !== null);
}

function parseGeometryFromHazardPayload(rawHazardPayload) {
  if (typeof rawHazardPayload !== "string" || rawHazardPayload.trim() === "") {
    return [];
  }

  try {
    const payload = JSON.parse(rawHazardPayload);
    if (!Array.isArray(payload?.polygons) || payload.polygons.length === 0) {
      return [];
    }

    const polygon = payload.polygons[0];
    if (!Array.isArray(polygon)) {
      return [];
    }

    return polygon
      .map((point) => {
        const x = Number(point?.x);
        const z = Number(point?.z);
        if (!isFiniteNumber(x) || !isFiniteNumber(z)) {
          return null;
        }
        return [x, 0, z];
      })
      .filter((point) => point !== null);
  } catch {
    return [];
  }
}

function pickPrimarySensorType(sourceModalities) {
  for (const modality of MODALITY_PRIORITY) {
    if (sourceModalities.includes(modality)) {
      return modality;
    }
  }
  return "thermal";
}

function buildDetectionId(candidateId, timestampMs, position) {
  const normalizedCandidateId = typeof candidateId === "string" ? candidateId.trim() : "";
  if (normalizedCandidateId) {
    return normalizedCandidateId;
  }

  const [x, , z] = position;
  return `fused-${Math.round(timestampMs)}-${Math.round(x)}-${Math.round(z)}`;
}

function buildSignature(sourceModalities, confidenceLevel) {
  if (sourceModalities.length === 0) {
    return `Fusion alert (${confidenceLevel.toLowerCase()})`;
  }

  if (sourceModalities.length === 1) {
    return `${sourceModalities[0]} detection (${confidenceLevel.toLowerCase()})`;
  }

  return `${sourceModalities.join(' + ')} fusion (${confidenceLevel.toLowerCase()})`;
}

export function normalizeFusedDetectionMessage(rawMessage, options = {}) {
  const nowMs = Number.isFinite(options.nowMs) ? Number(options.nowMs) : Date.now();
  const maxAgeMs = Number.isFinite(options.maxAgeMs) ? Number(options.maxAgeMs) : 5 * 60 * 1000;
  const maxFutureSkewMs = Number.isFinite(options.maxFutureSkewMs) ? Number(options.maxFutureSkewMs) : 5_000;

  if (!rawMessage || typeof rawMessage !== "object") {
    throw new Error("Invalid fused detection message: expected object payload");
  }

  const message = rawMessage;
  const timestamp = parseTimestampMs(message.stamp, nowMs);
  const ageMs = nowMs - timestamp;

  if (ageMs > maxAgeMs) {
    return null;
  }
  if (timestamp > nowMs + maxFutureSkewMs) {
    return null;
  }

  const [targetX, targetY, targetZ] = parsePoint(message.local_target, "local_target");
  const position = [targetX, targetZ, targetY];

  if (!Array.isArray(message.source_modalities)) {
    throw new Error("Invalid fused detection message: source_modalities must be an array");
  }
  const sourceModalities = normalizeModalities(message.source_modalities);
  if (sourceModalities.length === 0) {
    throw new Error("Invalid fused detection message: source_modalities must contain at least one known modality");
  }
  const sensorType = pickPrimarySensorType(sourceModalities);

  const confidence = parseConfidence(message.confidence);
  const confidenceLevel = normalizeConfidenceLevel(message.confidence_level, confidence);
  const replayMetadata = parseReplayMetadata(message, timestamp);
  const geometry = parseGeometryFromLocalGeometry(message.local_geometry);
  const fallbackGeometry = geometry.length > 0
    ? geometry
    : parseGeometryFromHazardPayload(message.hazard_payload_json);

  const fallbackVehicleId = typeof options.fallbackVehicleId === "string"
    ? options.fallbackVehicleId
    : "fusion";
  const fallbackVehicleName = typeof options.fallbackVehicleName === "string"
    ? options.fallbackVehicleName
    : "Fusion";

  return {
    id: buildDetectionId(message.candidate_id, timestamp, position),
    sensorType,
    confidence,
    confidenceLevel,
    timestamp,
    status: "new",
    vehicleId: fallbackVehicleId,
    vehicleName: fallbackVehicleName,
    position,
    sourceModalities,
    geometry: fallbackGeometry.length > 0 ? fallbackGeometry : undefined,
    signatureType: buildSignature(sourceModalities, confidenceLevel),
    deliveryMode: replayMetadata?.deliveryMode,
    originalEventTs: replayMetadata?.originalEventTs ?? timestamp,
    replayedAtTs: replayMetadata?.replayedAtTs ?? undefined,
    isRetroactive: replayMetadata?.isRetroactive ?? false,
  };
}
