import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const HOOKS = [
  "useVehicleTelemetry.ts",
  "useFusedDetections.ts",
  "useMapTiles.ts",
  "useMissionControl.ts",
  "useThermalHotspots.ts",
];

test("parallel ROS hooks consume shared ROS connection surface", () => {
  for (const hookFile of HOOKS) {
    const fullPath = path.join(ROOT, "hooks", hookFile);
    const source = fs.readFileSync(fullPath, "utf8");
    assert.match(
      source,
      /useSharedROSConnection/,
      `${hookFile} should consume shared ROS connection context`
    );
    assert.doesNotMatch(
      source,
      /useROSConnection\(/,
      `${hookFile} should not create an isolated ROS connection`
    );
  }
});

test("viewer ROS socket creation stays centralized in the shared connection hook", () => {
  const hookDir = path.join(ROOT, "hooks");
  const hookFiles = fs.readdirSync(hookDir).filter((entry) => entry.endsWith(".ts"));

  for (const hookFile of hookFiles) {
    const source = fs.readFileSync(path.join(hookDir, hookFile), "utf8");
    if (hookFile === "useROSConnection.ts") {
      assert.match(
        source,
        /new ROSLIB\.Ros/,
        "useROSConnection.ts should remain the single ROS socket creation surface"
      );
      continue;
    }

    assert.doesNotMatch(
      source,
      /new ROSLIB\.Ros/,
      `${hookFile} should not create a direct ROS socket`
    );
  }
});

test("viewer app provides a shared ROS connection provider", () => {
  const pagePath = path.join(ROOT, "app", "page.tsx");
  const source = fs.readFileSync(pagePath, "utf8");
  assert.match(source, /ROSConnectionProvider/, "page.tsx should include ROSConnectionProvider");
});
