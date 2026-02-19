const KNOWN_MODALITIES = new Set(["thermal", "acoustic", "gas"]);

function normalizeModalities(modalities, sensorType) {
  if (Array.isArray(modalities)) {
    const normalized = modalities
      .map(modality => (typeof modality === "string" ? modality.trim().toLowerCase() : ""))
      .filter(modality => KNOWN_MODALITIES.has(modality));
    if (normalized.length > 0) {
      return Array.from(new Set(normalized));
    }
  }

  if (typeof sensorType === "string" && KNOWN_MODALITIES.has(sensorType)) {
    return [sensorType];
  }

  return [];
}

export function getConfidenceTextClass(detection) {
  const level = typeof detection?.confidenceLevel === "string"
    ? detection.confidenceLevel.toUpperCase()
    : "";

  if (level === "HIGH") {
    return "text-emerald-400";
  }
  if (level === "MEDIUM") {
    return "text-white";
  }
  if (level === "LOW") {
    return "text-white/60";
  }

  const confidence = Number(detection?.confidence);
  if (!Number.isFinite(confidence)) {
    return "text-white/50";
  }
  const percent = Math.round(confidence * 100);
  if (percent >= 85) {
    return "text-emerald-400";
  }
  if (percent >= 70) {
    return "text-white";
  }
  return "text-white/50";
}

export function computeDetectionCounts(detections) {
  const counts = {
    thermal: 0,
    acoustic: 0,
    gas: 0,
    pending: 0,
    confirmed: 0,
  };

  for (const detection of detections) {
    const modalities = normalizeModalities(detection?.sourceModalities, detection?.sensorType);
    for (const modality of modalities) {
      counts[modality] += 1;
    }

    if (detection?.status === "confirmed") {
      counts.confirmed += 1;
    }
    if (detection?.status === "new" || detection?.status === "reviewing") {
      counts.pending += 1;
    }
  }

  return counts;
}

export function applyDetectionStatusOverrides(detections, overrides) {
  return detections.map((detection) => ({
    ...detection,
    status: overrides[detection.id] ?? detection.status,
  }));
}
