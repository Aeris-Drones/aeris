import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";

const FILE_PATH = path.join(process.cwd(), "src", "components", "map", "DroneMarker3D.tsx");

test("DroneMarker3D reuses the shared last-contact formatter", () => {
  const source = fs.readFileSync(FILE_PATH, "utf8");

  assert.match(
    source,
    /import\s*\{\s*formatLastContactAge as formatMarkerAge\s*\}\s*from\s*['"]@\/lib\/degradedVehicleState['"]/,
    "DroneMarker3D should import the shared last-contact formatter"
  );
  assert.doesNotMatch(
    source,
    /function formatMarkerAge\(/,
    "DroneMarker3D should not keep a local duplicate of the age formatter"
  );
});
