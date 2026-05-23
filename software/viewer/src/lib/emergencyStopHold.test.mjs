import test from "node:test";
import assert from "node:assert/strict";

import {
  advanceAbortRequestWindow,
  advanceEmergencyStopHold,
  HOLD_TO_ABORT_MS,
  beginEmergencyStopHold,
  cancelEmergencyStopHold,
  cancelEmergencyStopHoldState,
  createIdleEmergencyStopHold,
  isAbortRequestPending,
  resolveAbortRequestWindowId,
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

test("cancelling from the current hold state yields idle state immediately", () => {
  const holding = beginEmergencyStopHold(250);
  const idle = createIdleEmergencyStopHold();

  assert.deepEqual(cancelEmergencyStopHoldState(holding), idle);
  assert.equal(cancelEmergencyStopHoldState(idle), idle);
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

test("abort request state stays pending across active abort phases and drops outside the abort window", () => {
  let abortWindow = advanceAbortRequestWindow({ id: 0, isActive: false }, "SEARCHING");
  assert.equal(
    isAbortRequestPending({
      abortWindow,
      pendingWindowId: abortWindow.id,
      resolvedWindowId: null,
      abortError: null,
    }),
    true
  );

  abortWindow = advanceAbortRequestWindow(abortWindow, "TRACKING");
  assert.equal(
    isAbortRequestPending({
      abortWindow,
      pendingWindowId: abortWindow.id,
      resolvedWindowId: null,
      abortError: null,
    }),
    true
  );

  abortWindow = advanceAbortRequestWindow(abortWindow, "ABORTED");
  assert.equal(
    isAbortRequestPending({
      abortWindow,
      pendingWindowId: abortWindow.id,
      resolvedWindowId: null,
      abortError: null,
    }),
    false
  );
});

test("abort request resolution prevents stale lockout when the same phase name returns later", () => {
  let abortWindow = advanceAbortRequestWindow({ id: 0, isActive: false }, "SEARCHING");
  const firstWindowId = abortWindow.id;
  const resolvedWindowId = resolveAbortRequestWindowId({
    abortWindow: advanceAbortRequestWindow(abortWindow, "ABORTED"),
    pendingWindowId: firstWindowId,
    resolvedWindowId: null,
    abortError: null,
  });

  abortWindow = advanceAbortRequestWindow({ id: firstWindowId, isActive: false }, "SEARCHING");
  assert.equal(abortWindow.id, firstWindowId + 1);
  assert.equal(
    isAbortRequestPending({
      abortWindow,
      pendingWindowId: firstWindowId,
      resolvedWindowId,
      abortError: "Mission abort was rejected by orchestrator.",
    }),
    false
  );
});

test("abort request resolution latches failures until a new hold starts", () => {
  const abortWindow = advanceAbortRequestWindow({ id: 0, isActive: false }, "SEARCHING");
  const resolvedWindowId = resolveAbortRequestWindowId({
    abortWindow,
    pendingWindowId: abortWindow.id,
    resolvedWindowId: null,
    abortError: "Mission abort was rejected by orchestrator.",
  });

  assert.equal(
    isAbortRequestPending({
      abortWindow,
      pendingWindowId: abortWindow.id,
      resolvedWindowId,
      abortError: null,
    }),
    false
  );
});
