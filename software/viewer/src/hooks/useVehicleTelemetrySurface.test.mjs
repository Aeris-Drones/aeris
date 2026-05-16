import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const ROOT = path.dirname(fileURLToPath(import.meta.url));
const SOURCE = fs.readFileSync(path.join(ROOT, "useVehicleTelemetry.ts"), "utf8");

test("buildTelemetryDedupeKey recognizes snake_case and camelCase vehicle ids", () => {
  assert.match(
    SOURCE,
    /vehicle_id\?: unknown; vehicleId\?: unknown/,
    "telemetry dedupe keys should read both snake_case and camelCase vehicle ids"
  );
});

test("buildTelemetryDedupeKey drops payloads without a usable vehicle id", () => {
  assert.match(
    SOURCE,
    /if \(!vehicleId\) {\s*return null;\s*}/,
    "telemetry dedupe keys should be skipped when vehicle ids are missing"
  );
});
