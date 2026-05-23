import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { pathToFileURL } from "node:url";

import ts from "typescript";

const sourcePath = new URL("./batteryMonitoring.ts", import.meta.url);
const source = fs.readFileSync(sourcePath, "utf8");
const compiled = ts.transpileModule(source, {
  compilerOptions: {
    module: ts.ModuleKind.ES2022,
    target: ts.ScriptTarget.ES2022,
  },
}).outputText;
const tempDir = fs.mkdtempSync(path.join(os.tmpdir(), "aeris-battery-monitoring-test-"));
const modulePath = path.join(tempDir, "batteryMonitoring.mjs");
fs.writeFileSync(modulePath, compiled, "utf8");

const {
  BATTERY_HEALTHY_THRESHOLD_PERCENT,
  LOW_BATTERY_WARNING_THRESHOLD_PERCENT,
  deriveBatteryAlertTransitions,
  formatRemainingFlightTime,
  getBatteryLevel,
  getLowBatteryAlertId,
} = await import(pathToFileURL(modulePath).href);

test("shared battery thresholds stay anchored at the story contract", () => {
  assert.equal(BATTERY_HEALTHY_THRESHOLD_PERCENT, 50);
  assert.equal(LOW_BATTERY_WARNING_THRESHOLD_PERCENT, 25);
  assert.equal(getBatteryLevel(51), "healthy");
  assert.equal(getBatteryLevel(50.1), "healthy");
  assert.equal(getBatteryLevel(50), "warning");
  assert.equal(getBatteryLevel(26), "warning");
  assert.equal(getBatteryLevel(25.4), "warning");
  assert.equal(getBatteryLevel(25), "critical");
  assert.equal(getBatteryLevel(null), "unknown");
});

test("deriveBatteryAlertTransitions only emits one low-battery alert while a vehicle stays below threshold", () => {
  const first = deriveBatteryAlertTransitions(
    new Map(),
    [{ id: "scout_1", name: "SCOUT 1", battery: 24, remainingFlightTimeSec: 420 }]
  );

  assert.equal(first.transitions.length, 1);
  assert.deepEqual(first.transitions[0], {
    type: "entered_low",
    vehicleId: "scout_1",
    vehicleName: "SCOUT 1",
    battery: 24,
    remainingFlightTimeSec: 420,
    alertId: getLowBatteryAlertId("scout_1"),
  });

  const second = deriveBatteryAlertTransitions(
    first.state,
    [{ id: "scout_1", name: "SCOUT 1", battery: 19, remainingFlightTimeSec: 360 }]
  );

  assert.equal(second.transitions.length, 0);
});

test("deriveBatteryAlertTransitions emits a recovery event when the vehicle climbs back above threshold", () => {
  const first = deriveBatteryAlertTransitions(
    new Map(),
    [{ id: "ranger_1", name: "RANGER 1", battery: 20, remainingFlightTimeSec: null }]
  );

  const recovered = deriveBatteryAlertTransitions(
    first.state,
    [{ id: "ranger_1", name: "RANGER 1", battery: 33, remainingFlightTimeSec: null }]
  );

  assert.deepEqual(recovered.transitions, [
    {
      type: "recovered",
      vehicleId: "ranger_1",
      vehicleName: "RANGER 1",
      battery: 33,
      remainingFlightTimeSec: null,
      alertId: getLowBatteryAlertId("ranger_1"),
    },
  ]);
});

test("deriveBatteryAlertTransitions does not emit a duplicate alert when low battery telemetry goes missing temporarily", () => {
  const first = deriveBatteryAlertTransitions(
    new Map(),
    [{ id: "scout_1", name: "SCOUT 1", battery: 24, remainingFlightTimeSec: 420 }]
  );

  const missing = deriveBatteryAlertTransitions(
    first.state,
    [{ id: "scout_1", name: "SCOUT 1", battery: null, remainingFlightTimeSec: null }]
  );

  assert.equal(missing.transitions.length, 0);

  const resumedLow = deriveBatteryAlertTransitions(
    missing.state,
    [{ id: "scout_1", name: "SCOUT 1", battery: 19, remainingFlightTimeSec: 300 }]
  );

  assert.equal(resumedLow.transitions.length, 0);
});

test("deriveBatteryAlertTransitions emits recovery and allows a later low crossing after recovery", () => {
  const first = deriveBatteryAlertTransitions(
    new Map(),
    [{ id: "ranger_1", name: "RANGER 1", battery: 20, remainingFlightTimeSec: 600 }]
  );

  const recovered = deriveBatteryAlertTransitions(
    first.state,
    [{ id: "ranger_1", name: "RANGER 1", battery: 25.4, remainingFlightTimeSec: 540 }]
  );

  assert.deepEqual(recovered.transitions, [
    {
      type: "recovered",
      vehicleId: "ranger_1",
      vehicleName: "RANGER 1",
      battery: 25.4,
      remainingFlightTimeSec: 540,
      alertId: getLowBatteryAlertId("ranger_1"),
    },
  ]);

  const enteredAgain = deriveBatteryAlertTransitions(
    recovered.state,
    [{ id: "ranger_1", name: "RANGER 1", battery: 24.9, remainingFlightTimeSec: 480 }]
  );

  assert.deepEqual(enteredAgain.transitions, [
    {
      type: "entered_low",
      vehicleId: "ranger_1",
      vehicleName: "RANGER 1",
      battery: 24.9,
      remainingFlightTimeSec: 480,
      alertId: getLowBatteryAlertId("ranger_1"),
    },
  ]);
});

test("formatRemainingFlightTime uses an explicit unavailable fallback", () => {
  assert.equal(formatRemainingFlightTime(undefined), "Unavailable");
  assert.equal(formatRemainingFlightTime(null), "Unavailable");
  assert.equal(formatRemainingFlightTime(45), "45s");
  assert.equal(formatRemainingFlightTime(540), "9m");
  assert.equal(formatRemainingFlightTime(3_780), "1h 3m");
});
