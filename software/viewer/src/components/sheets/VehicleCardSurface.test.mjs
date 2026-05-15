import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const ROOT = path.dirname(fileURLToPath(import.meta.url));

test("VehicleCard renders a slam mode label using the vehicle mission metadata", () => {
  const cardPath = path.join(ROOT, "VehicleCard.tsx");
  const source = fs.readFileSync(cardPath, "utf8");

  assert.match(
    source,
    /SLAM:\s*\{formatSlamModeLabel\(vehicle\.slamMode\)\}/,
    "VehicleCard should render the current slam mode label on the live status card"
  );
  assert.match(
    source,
    /data-testid="vehicle-slam-mode"/,
    "VehicleCard should keep the slam mode label easy to target in follow-up tests"
  );
});

test("VehicleCard slam mode formatter keeps the unknown fallback explicit", () => {
  const cardPath = path.join(ROOT, "VehicleCard.tsx");
  const source = fs.readFileSync(cardPath, "utf8");

  assert.match(
    source,
    /if \(!normalized \|\| normalized === 'unknown'\)\s*{\s*return 'UNKNOWN';\s*}/,
    "VehicleCard should render UNKNOWN when the slam mode is missing or unknown"
  );
  assert.match(source, /return 'VIO';/, "VehicleCard should present the VIO label cleanly");
  assert.match(
    source,
    /return 'LIO-SAM';/,
    "VehicleCard should present the LIO-SAM label cleanly"
  );
});
