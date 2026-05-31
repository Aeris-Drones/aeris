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

function requireReplayPayloadRecord(rawValue) {
  if (!rawValue || typeof rawValue !== "object" || Array.isArray(rawValue)) {
    throw new Error("Invalid Halo evidence replay payload: record must be an object");
  }
  return rawValue;
}

function parseIntegerText(rawValue, fieldName) {
  if (typeof rawValue === "boolean" || rawValue === null || rawValue === undefined) {
    throw new Error(`Invalid Halo confidence event: ${fieldName} must be an integer`);
  }
  if (typeof rawValue === "bigint") {
    return rawValue.toString();
  }
  if (typeof rawValue === "number") {
    if (!Number.isSafeInteger(rawValue)) {
      throw new Error(`Invalid Halo confidence event: ${fieldName} must be a safe integer or string`);
    }
    return String(rawValue);
  }
  if (typeof rawValue === "string") {
    const normalized = rawValue.trim();
    if (/^[+-]?\d+$/.test(normalized)) {
      return normalized;
    }
  }
  throw new Error(`Invalid Halo confidence event: ${fieldName} must be an integer`);
}

function requireInt(rawValue, fieldName, minimum = 0) {
  const text = parseIntegerText(rawValue, fieldName);
  const value = Number(text);
  if (!Number.isSafeInteger(value) || value < minimum) {
    throw new Error(`Invalid Halo confidence event: ${fieldName} must be a safe integer >= ${minimum}`);
  }
  return value;
}

function requireBigInt(rawValue, fieldName, minimum = 0n) {
  const text = parseIntegerText(rawValue, fieldName);
  const value = BigInt(text);
  if (value < minimum) {
    throw new Error(`Invalid Halo confidence event: ${fieldName} must be an integer >= ${minimum}`);
  }
  return { value, text };
}

function requireFiniteNumber(rawValue, fieldName) {
  if (rawValue === null || rawValue === undefined || typeof rawValue === "boolean") {
    throw new Error(`Invalid Halo confidence event: ${fieldName} must be numeric`);
  }
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

function buildEventId({ sourceId, frameId, frameIndex, timestampNsText, detectionType, region }) {
  const regionKey = region
    ? `${region.x}:${region.y}:${region.width}:${region.height}`
    : "none";
  return `halo-confidence-${sourceId}:${frameId}:${frameIndex}:${timestampNsText}:${detectionType}:${regionKey}`;
}

function isEpochNanoseconds(value) {
  return value >= 946_684_800_000_000_000n;
}

function deriveDisplayTimestampMs(timestampNs, options) {
  if (isEpochNanoseconds(timestampNs)) {
    return Number(timestampNs / 1_000_000n);
  }
  return Number.isFinite(options.nowMs) ? Number(options.nowMs) : Date.now();
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
    normalized.source_timestamp_ns = requireBigInt(
      recognition.source_timestamp_ns,
      "recognition.source_timestamp_ns",
      0n
    ).text;
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

export function normalizeHaloConfidenceEventMessage(rawMessage, options = {}) {
  const message = requireObject(rawMessage, "payload");
  const sourceId = requireString(message.source_id, "source_id");
  const sourceName = requireString(message.source_name, "source_name");
  const frameId = requireString(message.frame_id, "frame_id");
  const frameIndex = requireInt(message.frame_index, "frame_index", 0);
  const { value: timestampNs, text: timestampNsText } = requireBigInt(
    message.timestamp_ns,
    "timestamp_ns",
    0n
  );
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
      timestampNsText,
      detectionType: resolvedDetectionType,
      region,
    }),
    sensorType: "rgb",
    confidence,
    confidenceLevel: normalizeConfidenceLevel(message.confidence_level, confidence),
    timestamp: deriveDisplayTimestampMs(timestampNs, options),
    status: "new",
    vehicleId: sourceId,
    vehicleName: sourceName,
    position,
    sourceModalities: ["rgb"],
    sector: deriveSector(locationHint, frameId),
    signatureType: resolvedLabel,
    frameId,
    frameIndex,
    timestampNs: timestampNsText,
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

export function normalizeHaloEvidenceLogRecord(rawRecord, options = {}) {
  const record = requireReplayPayloadRecord(rawRecord);
  const detection = normalizeHaloConfidenceEventMessage(requireObject(record.event, "event"), options);
  const recordedAtNs = record.recorded_at_ns !== undefined
    ? requireBigInt(record.recorded_at_ns, "recorded_at_ns", 0n).value
    : null;

  return {
    ...detection,
    evidencePath: optionalString(record.evidence_path, "evidence_path"),
    evidenceRef: optionalString(record.evidence_ref, "evidence_ref") ?? detection.evidenceRef,
    evidenceUri: optionalString(record.evidence_uri, "evidence_uri") ?? detection.evidenceUri,
    deliveryMode: "replayed",
    originalEventTs: detection.timestamp,
    replayedAtTs: recordedAtNs === null ? undefined : deriveDisplayTimestampMs(recordedAtNs, options),
    isRetroactive: true,
  };
}

function parseReplayPayloadLines(rawPayload) {
  return rawPayload
    .split(/\r?\n/u)
    .map((line) => line.trim())
    .filter(Boolean)
    .map((line) => {
      try {
        return JSON.parse(line);
      } catch (error) {
        throw new Error("Malformed Halo evidence replay payload: invalid JSONL record", {
          cause: error,
        });
      }
    });
}

export function normalizeHaloEvidenceReplayPayload(rawPayload, options = {}) {
  if (typeof rawPayload === "string") {
    return parseReplayPayloadLines(rawPayload).map((record) =>
      normalizeHaloEvidenceLogRecord(record, options)
    );
  }

  if (Array.isArray(rawPayload)) {
    return rawPayload.map((record) => normalizeHaloEvidenceLogRecord(record, options));
  }

  return [normalizeHaloEvidenceLogRecord(rawPayload, options)];
}
