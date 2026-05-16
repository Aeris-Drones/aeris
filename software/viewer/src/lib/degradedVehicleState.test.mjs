import test from "node:test";
import assert from "node:assert/strict";

import {
  DEFAULT_LAST_KNOWN_RETENTION_MS,
  DEFAULT_TELEMETRY_STALE_TIMEOUT_MS,
  deriveVehicleDegradedState,
  formatLastContactAge,
  normalizeMissionMetaForVehicle,
} from "./degradedVehicleState.js";

test("deriveVehicleDegradedState separates replay from offline", () => {
  const now = 100_000;
  const replayed = deriveVehicleDegradedState({
    lastUpdate: now - 1000,
    deliveryMode: "replayed",
    now,
  });
  assert.equal(replayed.status, "warning");
  assert.equal(replayed.offline, false);
  assert.equal(replayed.replayed, true);

  const stale = deriveVehicleDegradedState({
    lastUpdate: now - DEFAULT_TELEMETRY_STALE_TIMEOUT_MS - 1,
    deliveryMode: "replayed",
    now,
  });
  assert.equal(stale.status, "offline");
  assert.equal(stale.offline, true);
  assert.equal(stale.replayed, true);
});

test("deriveVehicleDegradedState keeps last-known state through retention window", () => {
  const now = 200_000;
  assert.equal(
    deriveVehicleDegradedState({
      lastUpdate: now - DEFAULT_LAST_KNOWN_RETENTION_MS + 1000,
      now,
    }).retained,
    true
  );
  assert.equal(
    deriveVehicleDegradedState({
      lastUpdate: now - DEFAULT_LAST_KNOWN_RETENTION_MS - 1,
      now,
    }).retained,
    false
  );
});

test("deriveVehicleDegradedState treats fresh telemetry as authoritative over stale mission offline hints", () => {
  const now = 300_000;
  const degraded = deriveVehicleDegradedState({
    lastUpdate: now - 1000,
    missionOnline: false,
    deliveryMode: "live",
    now,
  });

  assert.equal(degraded.offline, false);
  assert.equal(degraded.status, "active");
  assert.equal(degraded.lastContactAgeMs, 1000);
});

test("normalizeMissionMetaForVehicle supports normalized and raw ids", () => {
  const meta = normalizeMissionMetaForVehicle("scout-1", {
    assignments: { scout_1: "SEARCHING" },
    assignmentLabels: { scout_1: "SEARCHING:zone-a" },
    progress: { scout_1: 32 },
    online: { scout_1: false },
    slamModes: { scout_1: "vio" },
  });

  assert.equal(meta.assignment, "SEARCHING");
  assert.equal(meta.assignmentLabel, "SEARCHING:zone-a");
  assert.equal(meta.progress, 32);
  assert.equal(meta.online, false);
  assert.equal(meta.slamMode, "vio");
});

test("formatLastContactAge renders compact operator timer labels", () => {
  assert.equal(formatLastContactAge(9000), "9s");
  assert.equal(formatLastContactAge(65_000), "1m 05s");
  assert.equal(formatLastContactAge(null), "--");
});
