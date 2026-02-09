import test from "node:test";
import assert from "node:assert/strict";

import {
  computeMissionControlFlags,
  getAbortMissionValidationError,
  withServiceTimeout,
} from "./missionControlBehavior.js";

test("ABORTED phase clears paused semantics for controls", () => {
  const flags = computeMissionControlFlags({
    phase: "ABORTED",
    pausedAt: Date.now(),
    hasValidStartZone: true,
    rosConnected: true,
  });

  assert.equal(flags.isPaused, false);
  assert.equal(flags.canResume, false);
  assert.equal(flags.canAbort, false);
});

test("SEARCHING phase remains abortable and resumable when paused", () => {
  const flags = computeMissionControlFlags({
    phase: "SEARCHING",
    pausedAt: Date.now(),
    hasValidStartZone: true,
    rosConnected: true,
  });

  assert.equal(flags.isPaused, true);
  assert.equal(flags.canResume, true);
  assert.equal(flags.canAbort, true);
});

test("abort validation enforces connectivity and mission id", () => {
  assert.equal(
    getAbortMissionValidationError({
      rosConnected: false,
      missionId: "mission-1",
    }),
    "ROS is disconnected. Reconnect before aborting the mission."
  );

  assert.equal(
    getAbortMissionValidationError({
      rosConnected: true,
      missionId: "   ",
    }),
    "Mission abort failed: active mission id is missing."
  );
});

test("withServiceTimeout resolves successful responses", async () => {
  const result = await withServiceTimeout(
    resolve => {
      setTimeout(() => resolve({ success: true }), 10);
    },
    100,
    "abort_mission"
  );

  assert.deepEqual(result, { success: true });
});

test("withServiceTimeout rejects when callback never resolves", async () => {
  await assert.rejects(
    withServiceTimeout(
      () => {},
      20,
      "abort_mission"
    ),
    /abort_mission timed out/
  );
});
