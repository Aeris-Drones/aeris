import test from "node:test";
import assert from "node:assert/strict";

import {
  applyDetectionStatusOverrides,
  computeDetectionCounts,
  getConfidenceTextClass,
} from "./detectionViewState.js";

test("computeDetectionCounts derives modality counters from fused source modalities", () => {
  const counts = computeDetectionCounts([
    {
      id: "cand-1",
      sensorType: "thermal",
      sourceModalities: ["thermal", "acoustic"],
      confidence: 0.9,
      status: "new",
    },
    {
      id: "cand-2",
      sensorType: "gas",
      sourceModalities: ["gas"],
      confidence: 0.8,
      status: "confirmed",
    },
    {
      id: "cand-3",
      sensorType: "acoustic",
      confidence: 0.5,
      status: "reviewing",
    },
  ]);

  assert.deepEqual(counts, {
    thermal: 1,
    acoustic: 2,
    gas: 1,
    pending: 2,
    confirmed: 1,
  });
});

test("applyDetectionStatusOverrides preserves live IDs and merges confirm/dismiss state", () => {
  const detections = [
    { id: "cand-0001", status: "new", sensorType: "thermal", confidence: 0.7 },
    { id: "cand-0002", status: "reviewing", sensorType: "gas", confidence: 0.8 },
  ];

  const updated = applyDetectionStatusOverrides(detections, {
    "cand-0001": "confirmed",
    "cand-0002": "dismissed",
  });

  assert.equal(updated[0].status, "confirmed");
  assert.equal(updated[1].status, "dismissed");
  assert.equal(updated[0].id, "cand-0001");
  assert.equal(updated[1].id, "cand-0002");
});

test("getConfidenceTextClass honors fused confidence levels for card color mapping", () => {
  assert.equal(
    getConfidenceTextClass({ confidence: 0.2, confidenceLevel: "HIGH" }),
    "text-emerald-400"
  );
  assert.equal(
    getConfidenceTextClass({ confidence: 0.95, confidenceLevel: "MEDIUM" }),
    "text-white"
  );
  assert.equal(
    getConfidenceTextClass({ confidence: 0.9, confidenceLevel: "LOW" }),
    "text-white/60"
  );
});
