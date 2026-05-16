function parseTileId(value) {
  const tileId = String(value ?? "").trim();
  const match = /^(\d+)\/(\d+)\/(\d+)$/.exec(tileId);
  if (!match) {
    return null;
  }

  const z = Number(match[1]);
  const x = Number(match[2]);
  const y = Number(match[3]);
  if (!Number.isInteger(z) || !Number.isInteger(x) || !Number.isInteger(y)) {
    return null;
  }
  if (z < 0 || z > 22) {
    return null;
  }

  const maxIndex = (2 ** z) - 1;
  if (x < 0 || y < 0 || x > maxIndex || y > maxIndex) {
    return null;
  }

  return tileId;
}

function parseFormat(value) {
  const format = String(value ?? "").trim();
  return format.length > 0 ? format : null;
}

function parseByteSize(value) {
  const byteSize = Number(value);
  if (!Number.isInteger(byteSize) || byteSize < 0) {
    return null;
  }
  return byteSize;
}

function normalizeDeliveryMode(value) {
  if (typeof value !== "string") {
    return undefined;
  }
  return value.trim().toLowerCase() === "replayed" ? "replayed" : "live";
}

export function normalizeMapTileMessage(rawMessage) {
  if (!rawMessage || typeof rawMessage !== "object") {
    return null;
  }

  const tileId = parseTileId(rawMessage.tile_id ?? rawMessage.tileId);
  const format = parseFormat(rawMessage.format);
  const byteSize = parseByteSize(rawMessage.byte_size ?? rawMessage.byteSize);
  if (!tileId || !format || byteSize === null) {
    return null;
  }

  const normalized = {
    ...rawMessage,
    tile_id: tileId,
    format,
    byte_size: byteSize,
  };

  const deliveryMode = normalizeDeliveryMode(rawMessage.delivery_mode ?? rawMessage.deliveryMode);
  if (deliveryMode) {
    normalized.delivery_mode = deliveryMode;
  }

  if ("original_event_ts" in rawMessage || "originalEventTs" in rawMessage) {
    normalized.original_event_ts = rawMessage.original_event_ts ?? rawMessage.originalEventTs;
  }
  if ("replayed_at_ts" in rawMessage || "replayedAtTs" in rawMessage) {
    normalized.replayed_at_ts = rawMessage.replayed_at_ts ?? rawMessage.replayedAtTs;
  }

  return normalized;
}
