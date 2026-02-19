import test from "node:test";
import assert from "node:assert/strict";

import { normalizeFusedDetectionMessage } from "./fusedDetections.js";

test("normalizeFusedDetectionMessage maps fused payload fields into viewer detection contract", () => {
  const message = {
    stamp: { sec: 1_700_000_000, nanosec: 250_000_000 },
    candidate_id: "cand-00042",
    confidence_level: "HIGH",
    confidence: 0.91,
    source_modalities: ["thermal", "gas"],
    local_target: { x: 12.5, y: -8.75, z: 1.2 },
    local_geometry: [
      { x: 10, y: -10, z: 0 },
      { x: 15, y: -10, z: 0 },
      { x: 15, y: -5, z: 0 },
      { x: 10, y: -5, z: 0 },
    ],
  };

  const detection = normalizeFusedDetectionMessage(message, {
    nowMs: (1_700_000_000 * 1000) + 500,
    maxAgeMs: 60_000,
  });

  assert.ok(detection);
  assert.equal(detection.id, "cand-00042");
  assert.equal(detection.sensorType, "gas");
  assert.equal(detection.confidenceLevel, "HIGH");
  assert.equal(detection.confidence, 0.91);
  assert.deepEqual(detection.sourceModalities, ["thermal", "gas"]);
  assert.deepEqual(detection.position, [12.5, 1.2, -8.75]);
  assert.equal(detection.status, "new");
  assert.equal(detection.vehicleId, "fusion");
  assert.equal(detection.vehicleName, "Fusion");
  assert.deepEqual(detection.geometry, [
    [10, 0, -10],
    [15, 0, -10],
    [15, 0, -5],
    [10, 0, -5],
  ]);
});

test("normalizeFusedDetectionMessage falls back to hazard payload geometry when local geometry is absent", () => {
  const message = {
    stamp: { sec: 1_700_000_050, nanosec: 0 },
    candidate_id: "cand-00043",
    confidence_level: "MEDIUM",
    confidence: 0.65,
    source_modalities: ["acoustic", "gas"],
    local_target: { x: -40, y: 25, z: 0 },
    local_geometry: [],
    hazard_payload_json: JSON.stringify({
      polygons: [[
        { x: -42, z: 24 },
        { x: -38, z: 24 },
        { x: -38, z: 28 },
        { x: -42, z: 28 },
      ]],
    }),
  };

  const detection = normalizeFusedDetectionMessage(message, {
    nowMs: (1_700_000_050 * 1000) + 1000,
    maxAgeMs: 120_000,
  });

  assert.ok(detection);
  assert.deepEqual(detection.geometry, [
    [-42, 0, 24],
    [-38, 0, 24],
    [-38, 0, 28],
    [-42, 0, 28],
  ]);
});

test("normalizeFusedDetectionMessage drops stale detections", () => {
  const message = {
    stamp: { sec: 1_700_000_000, nanosec: 0 },
    candidate_id: "cand-00044",
    confidence_level: "LOW",
    confidence: 0.45,
    source_modalities: ["thermal"],
    local_target: { x: 0, y: 0, z: 0 },
    local_geometry: [],
  };

  const detection = normalizeFusedDetectionMessage(message, {
    nowMs: (1_700_000_000 * 1000) + 61_000,
    maxAgeMs: 60_000,
  });

  assert.equal(detection, null);
});

test("normalizeFusedDetectionMessage rejects malformed payloads", () => {
  assert.throws(
    () => normalizeFusedDetectionMessage({ confidence: 0.4 }, { nowMs: Date.now() }),
    /local_target/
  );
});
