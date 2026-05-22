import test from "node:test";
import assert from "node:assert/strict";

import {
  advanceEmergencyStopHold,
  HOLD_TO_ABORT_MS,
  beginEmergencyStopHold,
  cancelEmergencyStopHold,
  createIdleEmergencyStopHold,
  isAbortRequestPending,
  tickEmergencyStopHold,
} from "./emergencyStopHold.js";

test("quick tap never completes the emergency stop hold", () => {
  beginEmergencyStopHold(100);
  const released = cancelEmergencyStopHold();
  const afterRelease = tickEmergencyStopHold(released, 100 + HOLD_TO_ABORT_MS);

  assert.deepEqual(afterRelease, createIdleEmergencyStopHold());
});

test("hold progress completes only after the full dwell threshold", () => {
  const started = beginEmergencyStopHold(500);
  const beforeThreshold = tickEmergencyStopHold(started, 500 + HOLD_TO_ABORT_MS - 1);
  const completed = tickEmergencyStopHold(started, 500 + HOLD_TO_ABORT_MS);

  assert.equal(beforeThreshold.phase, "holding");
  assert.ok(beforeThreshold.progress > 0.99);
  assert.ok(beforeThreshold.progress < 1);
  assert.equal(completed.phase, "completed");
  assert.equal(completed.progress, 1);
});

test("advancing a completed hold emits one abort signal and resets local hold state", () => {
  const started = beginEmergencyStopHold(500);
  const result = advanceEmergencyStopHold({
    state: started,
    now: 500 + HOLD_TO_ABORT_MS,
    canAbort: true,
    abortPending: false,
  });

  assert.deepEqual(result.nextState, createIdleEmergencyStopHold());
  assert.equal(result.shouldDispatchAbort, true);
});

test("cancel events clear hold progress without dispatching completion", () => {
  const started = beginEmergencyStopHold(0);
  const ticking = tickEmergencyStopHold(started, HOLD_TO_ABORT_MS / 2);
  const cancelled = cancelEmergencyStopHold();
  const afterCancel = tickEmergencyStopHold(cancelled, HOLD_TO_ABORT_MS * 2);

  assert.equal(ticking.phase, "holding");
  assert.ok(ticking.progress > 0);
  assert.deepEqual(cancelled, createIdleEmergencyStopHold());
  assert.deepEqual(afterCancel, createIdleEmergencyStopHold());
});

test("abort request state clears once the mission leaves the active abort path", () => {
  assert.equal(
    isAbortRequestPending({
      abortRequested: true,
      abortError: null,
      missionPhase: "SEARCHING",
    }),
    true
  );
  assert.equal(
    isAbortRequestPending({
      abortRequested: true,
      abortError: null,
      missionPhase: "ABORTED",
    }),
    false
  );
  assert.equal(
    isAbortRequestPending({
      abortRequested: true,
      abortError: null,
      missionPhase: "IDLE",
    }),
    false
  );
  assert.equal(
    isAbortRequestPending({
      abortRequested: true,
      abortError: "Mission abort was rejected by orchestrator.",
      missionPhase: "SEARCHING",
    }),
    false
  );
});
