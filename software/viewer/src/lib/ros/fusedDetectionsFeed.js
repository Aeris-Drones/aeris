export function toVehicleName(vehicleId) {
  const normalized = typeof vehicleId === "string" ? vehicleId.trim() : "";
  if (!normalized) {
    return "Unknown Vehicle";
  }
  return normalized.replace(/[_-]/g, " ").toUpperCase();
}

export function findNearestVehicle(vehicles, [x, , z]) {
  if (!Array.isArray(vehicles) || vehicles.length === 0) {
    return undefined;
  }

  let closestVehicle;
  let closestDistance = Number.POSITIVE_INFINITY;

  for (const vehicle of vehicles) {
    const vehicleX = Number(vehicle?.position?.x);
    const vehicleZ = Number(vehicle?.position?.z);
    if (!Number.isFinite(vehicleX) || !Number.isFinite(vehicleZ)) {
      continue;
    }

    const dx = vehicleX - x;
    const dz = vehicleZ - z;
    const distance = Math.hypot(dx, dz);
    if (distance < closestDistance) {
      closestDistance = distance;
      closestVehicle = vehicle;
    }
  }

  return closestVehicle;
}

export function mergeLiveDetections(previous, incomingDetection, options = {}) {
  const nowMs = Number.isFinite(options.nowMs) ? Number(options.nowMs) : Date.now();
  const maxAgeMs = Number.isFinite(options.maxAgeMs) ? Number(options.maxAgeMs) : 5 * 60 * 1000;
  const maxDetections = Number.isFinite(options.maxDetections) ? Number(options.maxDetections) : 250;

  const cutoff = nowMs - maxAgeMs;
  const byId = new Map();

  for (const prior of previous) {
    if (prior?.id && Number(prior.timestamp) >= cutoff) {
      byId.set(prior.id, prior);
    }
  }

  if (incomingDetection?.id) {
    byId.set(incomingDetection.id, incomingDetection);
  }

  return Array.from(byId.values())
    .sort((left, right) => Number(right.timestamp) - Number(left.timestamp))
    .slice(0, maxDetections);
}

export function subscribeToFusedTopic({
  topicFactory,
  ros,
  topicName,
  messageType,
  onMessage,
}) {
  const topic = topicFactory({
    ros,
    name: topicName,
    messageType,
  });

  topic.subscribe(onMessage);

  return () => {
    topic.unsubscribe();
  };
}
