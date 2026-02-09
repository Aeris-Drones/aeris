import test from "node:test";
import assert from "node:assert/strict";

import { applyVehicleMissionMeta } from "./fleetVehicleProjection.js";

function baseVehicle(overrides = {}) {
  return {
    id: "scout_1",
    status: "active",
    batteryPercent: 95,
    signalStrength: 88,
    speed: 2.4,
    altitude: 18,
    distanceFromHome: 14,
    isSelected: false,
    ...overrides,
  };
}

test("applyVehicleMissionMeta applies assignment label and progress for UI rendering", () => {
  const projected = applyVehicleMissionMeta(baseVehicle(), {
    assignment: "SEARCHING",
    assignmentLabel: "SEARCHING:zone-a",
    progress: 44.5,
    online: true,
  });

  assert.equal(projected.assignment, "SEARCHING:zone-a");
  assert.equal(projected.missionProgressPercent, 44.5);
  assert.equal(projected.status, "active");
});

test("applyVehicleMissionMeta falls back to assignment when label is missing", () => {
  const projected = applyVehicleMissionMeta(baseVehicle(), {
    assignment: "TRACKING",
    progress: 10,
  });

  assert.equal(projected.assignment, "TRACKING");
  assert.equal(projected.missionProgressPercent, 10);
});

test("applyVehicleMissionMeta applies command status hint only for online vehicles", () => {
  const projected = applyVehicleMissionMeta(baseVehicle(), {
    commandStatusHint: "holding",
    online: true,
  });

  assert.equal(projected.status, "holding");
});

test("applyVehicleMissionMeta forces offline when telemetry marks vehicle offline", () => {
  const projected = applyVehicleMissionMeta(baseVehicle({ status: "active" }), {
    commandStatusHint: "holding",
    online: false,
  });

  assert.equal(projected.status, "offline");
});

test("applyVehicleMissionMeta ignores non-finite progress", () => {
  const projected = applyVehicleMissionMeta(baseVehicle(), {
    progress: Number.NaN,
  });

  assert.equal(projected.missionProgressPercent, undefined);
});
