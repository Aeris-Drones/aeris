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
    vehicleSlamModes: {
      scout1: "vio",
      "scout-2": "LioSam",
      ranger1: "unknown",
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
  assert.deepEqual(meta.slamModes, {
    scout_1: "vio",
    scout_2: "liosam",
    ranger_1: "unknown",
  });
});

test("extractVehicleMissionMetaFromProgressPayload ignores malformed content", () => {
  const payload = JSON.stringify({
    vehicleAssignments: { scout1: 12, scout2: "SEARCHING" },
    vehicleAssignmentLabels: { scout1: ["bad"], scout2: "SEARCHING:zone-2" },
    vehicleProgress: { scout1: "10", scout2: Number.NaN, scout3: 10 },
    vehicleOnline: { scout1: 0 },
    vehicleSlamModes: { scout1: 7, scout2: "vio" },
  });
  const meta = extractVehicleMissionMetaFromProgressPayload(payload);

  assert.deepEqual(meta.assignments, { scout_2: "SEARCHING" });
  assert.deepEqual(meta.assignmentLabels, { scout_2: "SEARCHING:zone-2" });
  assert.deepEqual(meta.progress, { scout_3: 10 });
  assert.deepEqual(meta.online, { scout_1: false });
  assert.deepEqual(meta.slamModes, { scout_2: "vio" });
});

test("extractVehicleMissionMetaFromProgressPayload returns empty maps for invalid JSON", () => {
  const meta = extractVehicleMissionMetaFromProgressPayload("not-json");
  assert.deepEqual(meta, {
    assignments: {},
    assignmentLabels: {},
    progress: {},
    online: {},
    slamModes: {},
  });
});

test("extractVehicleMissionMetaFromProgressPayload returns fresh empty maps per call", () => {
  const first = extractVehicleMissionMetaFromProgressPayload("not-json");
  first.assignments.scout_1 = "SEARCHING";
  first.assignmentLabels.scout_1 = "SEARCHING:zone-a";
  first.progress.scout_1 = 42;
  first.online.scout_1 = true;
  first.slamModes.scout_1 = "vio";

  const second = extractVehicleMissionMetaFromProgressPayload("not-json");

  assert.notStrictEqual(first, second);
  assert.notStrictEqual(first.assignments, second.assignments);
  assert.notStrictEqual(first.assignmentLabels, second.assignmentLabels);
  assert.notStrictEqual(first.progress, second.progress);
  assert.notStrictEqual(first.online, second.online);
  assert.notStrictEqual(first.slamModes, second.slamModes);
  assert.deepEqual(second, {
    assignments: {},
    assignmentLabels: {},
    progress: {},
    online: {},
    slamModes: {},
  });
});
