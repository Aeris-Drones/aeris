import test from "node:test";
import assert from "node:assert/strict";

import { normalizeMapTileMessage } from "./mapTilePayload.js";

test("normalizeMapTileMessage accepts snake_case tile payloads", () => {
  const normalized = normalizeMapTileMessage({
    tile_id: "16/10342/25311",
    format: "image/png",
    byte_size: 2048,
    data: "Zm9v",
  });

  assert.ok(normalized);
  assert.equal(normalized.tile_id, "16/10342/25311");
  assert.equal(normalized.byte_size, 2048);
  assert.equal(normalized.format, "image/png");
});

test("normalizeMapTileMessage accepts camelCase payloads and replay metadata", () => {
  const normalized = normalizeMapTileMessage({
    tileId: "17/20684/50622",
    format: "image/png",
    byteSize: 1024,
    deliveryMode: "replayed",
    originalEventTs: { sec: 1700000000, nanosec: 0 },
    replayedAtTs: 1700000010,
  });

  assert.ok(normalized);
  assert.equal(normalized.tile_id, "17/20684/50622");
  assert.equal(normalized.byte_size, 1024);
  assert.equal(normalized.delivery_mode, "replayed");
  assert.deepEqual(normalized.original_event_ts, { sec: 1700000000, nanosec: 0 });
  assert.equal(normalized.replayed_at_ts, 1700000010);
});

test("normalizeMapTileMessage rejects malformed payloads", () => {
  assert.equal(normalizeMapTileMessage(null), null);
  assert.equal(
    normalizeMapTileMessage({ tile_id: "", format: "image/png", byte_size: 10 }),
    null
  );
  assert.equal(
    normalizeMapTileMessage({ tile_id: "1/2/3", format: "", byte_size: 10 }),
    null
  );
  assert.equal(
    normalizeMapTileMessage({ tile_id: "1/2/3", format: "image/png", byte_size: -1 }),
    null
  );
});
