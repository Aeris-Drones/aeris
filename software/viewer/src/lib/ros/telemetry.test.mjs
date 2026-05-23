import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { pathToFileURL } from "node:url";

import ts from "typescript";

const sourcePath = new URL("./telemetry.ts", import.meta.url);
const source = fs.readFileSync(sourcePath, "utf8");
const compiled = ts.transpileModule(source, {
  compilerOptions: {
    module: ts.ModuleKind.ES2022,
    target: ts.ScriptTarget.ES2022,
  },
}).outputText;
const tempDir = fs.mkdtempSync(path.join(os.tmpdir(), "aeris-telemetry-test-"));
const modulePath = path.join(tempDir, "telemetry.mjs");
fs.writeFileSync(modulePath, compiled, "utf8");

const { parseVehicleTelemetry, VehicleType } = await import(pathToFileURL(modulePath).href);

test("parseVehicleTelemetry accepts camelCase compatibility fields", () => {
  const parsed = parseVehicleTelemetry({
    vehicleId: "scout_2",
    vehicleType: "Scout",
    timestamp: { sec: 1_700_000_000, nanosec: 500_000_000 },
    position: {
      latitude: 33.2,
      longitude: -87.5,
      altitudeM: 120,
    },
    orientation: {
      roll: 0.1,
      pitch: 0.2,
      yaw: 0.3,
    },
    velocity: {
      x: 4,
      y: 5,
      z: 6,
    },
    batteryPercent: 82,
    linkQuality: 61,
    coveragePercent: 44,
  });

  assert.equal(parsed.vehicle_id, "scout_2");
  assert.equal(parsed.vehicle_type, VehicleType.SCOUT);
  assert.equal(parsed.position.altitude, 120);
  assert.equal(parsed.battery_percent, 82);
  assert.equal(parsed.link_quality, 61);
  assert.equal(parsed.coverage_percent, 44);
});

test("parseVehicleTelemetry leaves optional percentages unset when telemetry reports null", () => {
  const parsed = parseVehicleTelemetry({
    vehicle_id: "ranger_1",
    vehicle_type: "ranger",
    timestamp: { sec: 1_700_000_001, nanosec: 0 },
    position: {
      latitude: 33.2,
      longitude: -87.5,
      altitude: 140,
    },
    orientation: {
      roll: 0,
      pitch: 0,
      yaw: 0,
    },
    velocity: {
      x: 0,
      y: 0,
      z: 0,
    },
    battery_percent: null,
    link_quality: null,
    coverage_percent: null,
  });

  assert.equal(parsed.battery_percent, undefined);
  assert.equal(parsed.link_quality, undefined);
  assert.equal(parsed.coverage_percent, undefined);
});

test("parseVehicleTelemetry keeps remaining flight time only when telemetry marks it available", () => {
  const parsed = parseVehicleTelemetry({
    vehicle_id: "scout_3",
    vehicle_type: "scout",
    timestamp: { sec: 1_700_000_002, nanosec: 0 },
    position: {
      latitude: 33.2,
      longitude: -87.5,
      altitude: 150,
    },
    orientation: {
      roll: 0,
      pitch: 0,
      yaw: 0.1,
    },
    velocity: {
      x: 1,
      y: 2,
      z: 3,
    },
    battery_percent: 36,
    remaining_flight_time_available: true,
    remaining_flight_time_sec: 540,
  });

  assert.equal(parsed.remaining_flight_time_available, true);
  assert.equal(parsed.remaining_flight_time_sec, 540);
  assert.equal(parsed.remainingFlightTimeAvailable, true);
  assert.equal(parsed.remainingFlightTimeSec, 540);
});

test("parseVehicleTelemetry drops remaining flight time when telemetry marks the estimate unavailable", () => {
  const parsed = parseVehicleTelemetry({
    vehicle_id: "ranger_2",
    vehicle_type: "ranger",
    timestamp: { sec: 1_700_000_003, nanosec: 0 },
    position: {
      latitude: 33.2,
      longitude: -87.5,
      altitude: 160,
    },
    orientation: {
      roll: 0,
      pitch: 0,
      yaw: 0.1,
    },
    velocity: {
      x: 1,
      y: 2,
      z: 3,
    },
    battery_percent: 18,
    remaining_flight_time_available: false,
    remaining_flight_time_sec: 120,
  });

  assert.equal(parsed.remaining_flight_time_available, false);
  assert.equal(parsed.remaining_flight_time_sec, undefined);
  assert.equal(parsed.remainingFlightTimeAvailable, false);
  assert.equal(parsed.remainingFlightTimeSec, undefined);
});
