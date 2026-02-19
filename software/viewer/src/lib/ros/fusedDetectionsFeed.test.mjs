import test from "node:test";
import assert from "node:assert/strict";

import {
  findNearestVehicle,
  mergeLiveDetections,
  subscribeToFusedTopic,
  toVehicleName,
} from "./fusedDetectionsFeed.js";

test("toVehicleName normalizes ROS vehicle IDs for display", () => {
  assert.equal(toVehicleName("scout_1"), "SCOUT 1");
  assert.equal(toVehicleName("ranger-2"), "RANGER 2");
  assert.equal(toVehicleName(""), "Unknown Vehicle");
});

test("findNearestVehicle picks the closest telemetry track in the local frame", () => {
  const nearest = findNearestVehicle(
    [
      { id: "scout_1", position: { x: 10, z: 10 } },
      { id: "scout_2", position: { x: 100, z: 100 } },
    ],
    [12, 0, 11]
  );

  assert.ok(nearest);
  assert.equal(nearest.id, "scout_1");
});

test("mergeLiveDetections keeps latest by id and prunes stale entries", () => {
  const nowMs = 1_700_000_100_000;
  const previous = [
    { id: "cand-1", timestamp: nowMs - 1_000 },
    { id: "cand-2", timestamp: nowMs - 7_000 },
  ];
  const incoming = { id: "cand-1", timestamp: nowMs };

  const merged = mergeLiveDetections(previous, incoming, {
    nowMs,
    maxAgeMs: 5_000,
    maxDetections: 5,
  });

  assert.deepEqual(
    merged.map((detection) => detection.id),
    ["cand-1"]
  );
  assert.equal(merged[0].timestamp, nowMs);
});

test("subscribeToFusedTopic wires subscribe/unsubscribe lifecycle", () => {
  let subscribeCalls = 0;
  let unsubscribeCalls = 0;
  let subscribedCallback = null;

  const cleanup = subscribeToFusedTopic({
    topicFactory: () => ({
      subscribe(callback) {
        subscribeCalls += 1;
        subscribedCallback = callback;
      },
      unsubscribe() {
        unsubscribeCalls += 1;
      },
    }),
    ros: {},
    topicName: "/detections/fused",
    messageType: "aeris_msgs/FusedDetection",
    onMessage: () => {},
  });

  assert.equal(subscribeCalls, 1);
  assert.equal(typeof subscribedCallback, "function");
  cleanup();
  assert.equal(unsubscribeCalls, 1);
});
