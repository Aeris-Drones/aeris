import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");

test("useMissionControl captures per-vehicle slam modes from mission progress", () => {
  const hookPath = path.join(ROOT, "hooks", "useMissionControl.ts");
  const source = fs.readFileSync(hookPath, "utf8");

  assert.match(
    source,
    /extractVehicleMissionMetaFromProgressPayload/,
    "useMissionControl should reuse the mission progress vehicle-meta parser"
  );
  assert.match(
    source,
    /vehicleSlamModes:\s*Record<string,\s*string>/,
    "useMissionControl should expose normalized vehicle slam modes"
  );
  assert.match(
    source,
    /setVehicleSlamModes\(\s*extractVehicleMissionMetaFromProgressPayload\(rawData\)\.slamModes\s*\)/,
    "useMissionControl should replace the current slam mode snapshot from each progress payload"
  );
});

test("page projects mission-control slam modes into fleet vehicle cards", () => {
  const pagePath = path.join(ROOT, "app", "page.tsx");
  const source = fs.readFileSync(pagePath, "utf8");

  assert.match(
    source,
    /slamMode:\s*vehicleSlamModes\[normalizeVehicleId\(vehicle\.id\)\]/,
    "page.tsx should pass the active slam mode into fleet vehicle card data"
  );
});
