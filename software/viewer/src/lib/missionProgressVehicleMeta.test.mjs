import test from "node:test";
import assert from "node:assert/strict";

import {
  extractVehicleMissionMetaFromProgressPayload,
  normalizeVehicleId,
} from "./missionProgressVehicleMeta.js";

test("normalizeVehicleId stabilizes mixed formatting", () => {
  assert.equal(normalizeVehicleId("Scout2"), "scout_2");
  assert.equal(normalizeVehicleId("ranger-1"), "ranger_1");
  assert.equal(normalizeVehicleId(" scout_3 "), "scout_3");
});

test("extractVehicleMissionMetaFromProgressPayload parses valid metadata", () => {
  const payload = JSON.stringify({
    vehicleAssignments: {
      scout1: "SEARCHING",
      "scout-2": "TRACKING",
    },
    vehicleAssignmentLabels: {
      scout1: "SEARCHING:zone-a",
      "scout-2": "TRACKING",
      ranger1: "OVERWATCH",
    },
    vehicleProgress: {
      scout1: 44.5,
      "scout-2": 88,
    },
    vehicleOnline: {
      scout1: true,
      "scout-2": false,
      ranger1: 1,
    },
  });
  const meta = extractVehicleMissionMetaFromProgressPayload(payload);

  assert.deepEqual(meta.assignments, {
    scout_1: "SEARCHING",
    scout_2: "TRACKING",
  });
  assert.deepEqual(meta.assignmentLabels, {
    scout_1: "SEARCHING:zone-a",
    scout_2: "TRACKING",
    ranger_1: "OVERWATCH",
  });
  assert.deepEqual(meta.progress, {
    scout_1: 44.5,
    scout_2: 88,
  });
  assert.deepEqual(meta.online, {
    scout_1: true,
    scout_2: false,
    ranger_1: true,
  });
});

test("extractVehicleMissionMetaFromProgressPayload ignores malformed content", () => {
  const payload = JSON.stringify({
    vehicleAssignments: { scout1: 12, scout2: "SEARCHING" },
    vehicleAssignmentLabels: { scout1: ["bad"], scout2: "SEARCHING:zone-2" },
    vehicleProgress: { scout1: "10", scout2: Number.NaN, scout3: 10 },
    vehicleOnline: { scout1: 0 },
  });
  const meta = extractVehicleMissionMetaFromProgressPayload(payload);

  assert.deepEqual(meta.assignments, { scout_2: "SEARCHING" });
  assert.deepEqual(meta.assignmentLabels, { scout_2: "SEARCHING:zone-2" });
  assert.deepEqual(meta.progress, { scout_3: 10 });
  assert.deepEqual(meta.online, { scout_1: false });
});

test("extractVehicleMissionMetaFromProgressPayload returns empty maps for invalid JSON", () => {
  const meta = extractVehicleMissionMetaFromProgressPayload("not-json");
  assert.deepEqual(meta, {
    assignments: {},
    assignmentLabels: {},
    progress: {},
    online: {},
  });
});
