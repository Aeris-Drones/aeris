import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";

const PAGE_PATH = path.join(process.cwd(), "src", "app", "page.tsx");

test("PiP live badge stays off for replayed or last-known vehicles", () => {
  const source = fs.readFileSync(PAGE_PATH, "utf8");

  assert.match(
    source,
    /vehicle\.deliveryMode !== 'replayed'/,
    "PiPVideoFeed should not mark replayed telemetry as live video"
  );
  assert.match(
    source,
    /vehicle\.isRetroactive !== true/,
    "PiPVideoFeed should keep retroactive catch-up distinct from live feeds"
  );
  assert.match(
    source,
    /vehicle\.isLastKnown !== true/,
    "PiPVideoFeed should not advertise last-known offline vehicles as live"
  );
});
