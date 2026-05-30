function isFiniteNumber(value) {
  return typeof value === "number" && Number.isFinite(value);
}

function clamp01(value) {
  if (!Number.isFinite(value)) {
    return 0;
  }
  return Math.max(0, Math.min(1, value));
}

function requireObject(rawValue, fieldName) {
  if (!rawValue || typeof rawValue !== "object" || Array.isArray(rawValue)) {
    throw new Error(`Invalid Halo confidence event: ${fieldName} must be an object`);
  }
  return rawValue;
}

function requireString(rawValue, fieldName) {
  if (typeof rawValue !== "string" || rawValue.trim() === "") {
    throw new Error(`Invalid Halo confidence event: ${fieldName} must be a non-empty string`);
  }
  return rawValue.trim();
}

function optionalString(rawValue, fieldName) {
  if (rawValue === undefined || rawValue === null) {
    return undefined;
  }
  if (typeof rawValue !== "string") {
    throw new Error(`Invalid Halo confidence event: ${fieldName} must be a string`);
  }
  const normalized = rawValue.trim();
  return normalized === "" ? undefined : normalized;
}

function requireInt(rawValue, fieldName, minimum = 0) {
  if (typeof rawValue === "boolean") {
    throw new Error(`Invalid Halo confidence event: ${fieldName} must be an integer`);
  }
  const value = Number(rawValue);
  if (!Number.isInteger(value) || value < minimum) {
    throw new Error(`Invalid Halo confidence event: ${fieldName} must be an integer >= ${minimum}`);
  }
  return value;
}

function requireFiniteNumber(rawValue, fieldName) {
  const value = Number(rawValue);
  if (!Number.isFinite(value)) {
    throw new Error(`Invalid Halo confidence event: ${fieldName} must be numeric`);
  }
  return value;
}

function humanizeDetectionType(detectionType) {
  return detectionType
    .split("_")
    .filter(Boolean)
    .map((segment) => segment.charAt(0).toUpperCase() + segment.slice(1))
    .join(" ");
}

function buildEventId({ sourceId, frameId, frameIndex, timestampNs, detectionType, region }) {
  const regionKey = region
    ? `${region.x}:${region.y}:${region.width}:${region.height}`
    : "none";
  return `halo-confidence-${sourceId}:${frameId}:${frameIndex}:${timestampNs}:${detectionType}:${regionKey}`;
}

function normalizeConfidenceLevel(rawLevel, confidence) {
  const level = typeof rawLevel === "string" ? rawLevel.trim().toUpperCase() : "";
  if (level === "HIGH" || level === "MEDIUM" || level === "LOW" || level === "UNKNOWN") {
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

function parseRegion(rawValue) {
  if (rawValue === undefined || rawValue === null) {
    return undefined;
  }

  const region = requireObject(rawValue, "region");
  const normalized = {
    x: requireInt(region.x, "region.x", 0),
    y: requireInt(region.y, "region.y", 0),
    width: requireInt(region.width, "region.width", 1),
    height: requireInt(region.height, "region.height", 1),
  };
  return normalized;
}

function parseLocationHint(rawValue) {
  if (rawValue === undefined || rawValue === null) {
    return undefined;
  }
  if (typeof rawValue === "string") {
    return requireString(rawValue, "location_hint");
  }

  const locationHint = requireObject(rawValue, "location_hint");
  const normalized = {};
  if (locationHint.label !== undefined) {
    normalized.label = requireString(locationHint.label, "location_hint.label");
  }
  for (const axis of ["x", "y", "z"]) {
    if (locationHint[axis] !== undefined) {
      normalized[axis] = requireFiniteNumber(locationHint[axis], `location_hint.${axis}`);
    }
  }
  if (Object.keys(normalized).length === 0) {
    throw new Error("Invalid Halo confidence event: location_hint must include a label or coordinates");
  }
  return normalized;
}

function parseRecognition(rawValue) {
  const recognition = requireObject(rawValue, "recognition");
  const normalized = {
    baseline_name: requireString(recognition.baseline_name, "recognition.baseline_name"),
    baseline_version: requireString(recognition.baseline_version, "recognition.baseline_version"),
  };

  if (recognition.latency_ms !== undefined) {
    normalized.latency_ms = requireFiniteNumber(recognition.latency_ms, "recognition.latency_ms");
  }
  if (recognition.source_timestamp_ns !== undefined) {
    normalized.source_timestamp_ns = requireInt(
      recognition.source_timestamp_ns,
      "recognition.source_timestamp_ns",
      0
    );
  }
  if (recognition.confidence_components !== undefined) {
    const components = requireObject(
      recognition.confidence_components,
      "recognition.confidence_components"
    );
    normalized.confidence_components = Object.fromEntries(
      Object.entries(components).map(([key, value]) => [
        requireString(key, "recognition.confidence_components key"),
        clamp01(requireFiniteNumber(value, `recognition.confidence_components.${key}`)),
      ])
    );
  }
  return normalized;
}

function parsePositionFromLocationHint(locationHint) {
  if (!locationHint || typeof locationHint === "string") {
    return [0, 0, 0];
  }
  const x = isFiniteNumber(locationHint.x) ? locationHint.x : 0;
  const y = isFiniteNumber(locationHint.y) ? locationHint.y : 0;
  const z = isFiniteNumber(locationHint.z) ? locationHint.z : 0;
  return [x, y, z];
}

function deriveSector(locationHint, frameId) {
  if (typeof locationHint === "string") {
    return locationHint;
  }
  if (locationHint && typeof locationHint.label === "string") {
    return locationHint.label;
  }
  return frameId;
}

export function normalizeHaloConfidenceEventMessage(rawMessage) {
  const message = requireObject(rawMessage, "payload");
  const sourceId = requireString(message.source_id, "source_id");
  const sourceName = requireString(message.source_name, "source_name");
  const frameId = requireString(message.frame_id, "frame_id");
  const frameIndex = requireInt(message.frame_index, "frame_index", 0);
  const timestampNs = requireInt(message.timestamp_ns, "timestamp_ns", 0);
  const detectionType = optionalString(message.detection_type, "detection_type");
  const label = optionalString(message.label, "label");
  if (!detectionType && !label) {
    throw new Error("Invalid Halo confidence event: detection_type or label is required");
  }

  const resolvedDetectionType = detectionType ?? requireString(
    label.toLowerCase().replace(/\s+/g, "_"),
    "label"
  );
  const resolvedLabel = label ?? humanizeDetectionType(resolvedDetectionType);
  const confidence = clamp01(requireFiniteNumber(message.confidence, "confidence"));
  const region = parseRegion(message.region);
  const locationHint = parseLocationHint(message.location_hint);
  const recognition = parseRecognition(message.recognition);
  const position = parsePositionFromLocationHint(locationHint);

  return {
    id: optionalString(message.event_id, "event_id") ?? buildEventId({
      sourceId,
      frameId,
      frameIndex,
      timestampNs,
      detectionType: resolvedDetectionType,
      region,
    }),
    sensorType: "rgb",
    confidence,
    confidenceLevel: normalizeConfidenceLevel(message.confidence_level, confidence),
    timestamp: Math.floor(timestampNs / 1_000_000),
    status: "new",
    vehicleId: sourceId,
    vehicleName: sourceName,
    position,
    sourceModalities: ["rgb"],
    sector: deriveSector(locationHint, frameId),
    signatureType: resolvedLabel,
    frameId,
    frameIndex,
    timestampNs,
    sourceUri: optionalString(message.source_uri, "source_uri"),
    label: resolvedLabel,
    detectionType: resolvedDetectionType,
    region,
    locationHint,
    evidenceRef: optionalString(message.evidence_ref, "evidence_ref"),
    evidenceUri: optionalString(message.evidence_uri, "evidence_uri"),
    recognition,
  };
}
