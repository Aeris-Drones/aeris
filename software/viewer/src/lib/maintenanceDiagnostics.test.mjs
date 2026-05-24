import test, { after } from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { pathToFileURL } from "node:url";
import ts from "typescript";

const sourcePath = new URL("./maintenanceDiagnostics.ts", import.meta.url);
const source = fs.readFileSync(sourcePath, "utf8");
const tempDir = fs.mkdtempSync(path.join(os.tmpdir(), "aeris-maintenance-diagnostics-"));

after(() => {
  fs.rmSync(tempDir, { recursive: true, force: true });
});

fs.writeFileSync(
  path.join(tempDir, "degradedVehicleState.mjs"),
  `
    export function deriveVehicleDegradedState({
      lastUpdate,
      missionOnline,
      deliveryMode,
      isRetroactive,
      now = Date.now(),
      staleTimeoutMs = 5000,
    } = {}) {
      const hasTelemetry = Number.isFinite(lastUpdate);
      const ageMs = hasTelemetry ? Math.max(0, now - lastUpdate) : Infinity;
      const telemetryStale = ageMs > staleTimeoutMs;
      const telemetryFresh = hasTelemetry && ageMs <= staleTimeoutMs;
      const missionOffline = missionOnline === false && !telemetryFresh;
      const offline = telemetryStale || missionOffline;
      const replayed = deliveryMode === "replayed" || isRetroactive === true;
      return {
        status: offline ? "offline" : replayed ? "warning" : "active",
        offline,
        retained: ageMs <= 120000,
        replayed,
        lastContactAgeMs: hasTelemetry ? ageMs : null,
        staleSinceMs: offline && hasTelemetry ? lastUpdate + staleTimeoutMs : null,
      };
    }
  `,
  "utf8"
);

const compiled = ts.transpileModule(
  source.replace("./degradedVehicleState.js", "./degradedVehicleState.mjs"),
  {
    compilerOptions: {
      module: ts.ModuleKind.ES2022,
      target: ts.ScriptTarget.ES2022,
    },
  }
).outputText;
const modulePath = path.join(tempDir, "maintenanceDiagnostics.mjs");
fs.writeFileSync(modulePath, compiled, "utf8");

const {
  createDemoFleetDiagnostics,
  projectFleetDiagnostics,
} = await import(pathToFileURL(modulePath).href);

test("projectFleetDiagnostics merges telemetry, pods, and maintenance diagnostics into readiness buckets", () => {
  const projected = projectFleetDiagnostics({
    nowMs: 20_000,
    telemetry: [
      {
        id: "scout_1",
        name: "SCOUT 1",
        batteryPercent: 81,
        altitudeMeters: 0,
        linkQualityPercent: 88,
        lastUpdate: 18_000,
        missionOnline: true,
      },
      {
        id: "scout_2",
        name: "SCOUT 2",
        batteryPercent: 57,
        altitudeMeters: 0,
        linkQualityPercent: 52,
        lastUpdate: 17_000,
        missionOnline: true,
      },
      {
        id: "ranger_1",
        name: "RANGER 1",
        batteryPercent: 64,
        altitudeMeters: 0,
        linkQualityPercent: 26,
        lastUpdate: 11_000,
        missionOnline: false,
      },
    ],
    diagnostics: [
      {
        vehicleId: "scout_1",
        motorHealthState: "healthy",
        motorHealthDetail: "All motor tests nominal",
        calibrationState: "healthy",
        calibrationDetail: "Calibration current",
        imuState: "healthy",
        imuDetail: "IMU drift nominal",
        compassState: "healthy",
        compassDetail: "Compass offsets current",
        accelerometerState: "healthy",
        accelerometerDetail: "Accelerometer aligned",
      },
      {
        vehicleId: "scout_2",
        motorHealthState: "warning",
        motorHealthDetail: "Motor 3 current ripple above baseline",
        calibrationState: "warning",
        calibrationDetail: "Compass recalibration due soon",
        imuState: "healthy",
        imuDetail: "IMU drift nominal",
        compassState: "warning",
        compassDetail: "Recheck before next sortie",
        accelerometerState: "healthy",
        accelerometerDetail: "Accelerometer aligned",
      },
      {
        vehicleId: "ranger_1",
        motorHealthState: "blocked",
        motorHealthDetail: "Motor 2 ESC fault latched",
        calibrationState: "blocked",
        calibrationDetail: "IMU calibration invalid",
        imuState: "blocked",
        imuDetail: "Bias table missing",
        compassState: "warning",
        compassDetail: "Compass offsets stale",
        accelerometerState: "warning",
        accelerometerDetail: "Tolerance trending high",
      },
    ],
    pods: [
      {
        vehicleId: "scout_1",
        slotId: "front-left",
        podSerial: "THM-1",
        podType: "thermal",
        lifecycleState: "registered",
        connected: true,
        powerReady: true,
        linkReady: true,
        capabilities: ["thermal"],
      },
      {
        vehicleId: "scout_2",
        slotId: "belly",
        podSerial: "GAS-9",
        podType: "hazmat",
        lifecycleState: "registered",
        connected: true,
        powerReady: true,
        linkReady: false,
        capabilities: ["gas", "hazmat"],
      },
      {
        vehicleId: "ranger_1",
        slotId: "rear-bay",
        podSerial: "LDR-4",
        podType: "lidar",
        lifecycleState: "faulted",
        connected: false,
        powerReady: false,
        linkReady: false,
        capabilities: ["lidar"],
      },
    ],
  });

  assert.equal(projected.summary.readyCount, 1);
  assert.equal(projected.summary.warningCount, 1);
  assert.equal(projected.summary.blockedCount, 1);
  assert.equal(projected.summary.connectedCount, 2);
  assert.equal(projected.summary.averageLinkQuality, 70);

  const scoutOne = projected.vehicles.find((vehicle) => vehicle.id === "scout_1");
  assert.equal(scoutOne.readiness, "ready");
  assert.equal(scoutOne.summaryChecks.motor.summary, "All motor tests nominal");
  assert.equal(scoutOne.summaryChecks.sensors.summary, "1/1 pods ready");

  const scoutTwo = projected.vehicles.find((vehicle) => vehicle.id === "scout_2");
  assert.equal(scoutTwo.readiness, "warning");
  assert.equal(scoutTwo.summaryChecks.mesh.state, "warning");
  assert.equal(scoutTwo.summaryChecks.calibration.state, "warning");

  const ranger = projected.vehicles.find((vehicle) => vehicle.id === "ranger_1");
  assert.equal(ranger.readiness, "blocked");
  assert.equal(ranger.isOffline, true);
  assert.equal(ranger.summaryChecks.motor.state, "blocked");
  assert.equal(ranger.detailChecks[0].label, "IMU calibration");
  assert.equal(ranger.pods[0].lifecycleLabel, "Faulted");
  assert.equal(scoutTwo.pods[0].state, "warning");
});

test("createDemoFleetDiagnostics keeps the maintenance route populated during disconnected local development", () => {
  const projected = createDemoFleetDiagnostics();

  assert.equal(projected.vehicles.length, 3);
  assert.equal(projected.summary.readyCount, 1);
  assert.equal(projected.summary.warningCount, 1);
  assert.equal(projected.summary.blockedCount, 1);
});

test("registered pods with explicit readiness failures stay actionable as degraded warnings", () => {
  const projected = projectFleetDiagnostics({
    nowMs: 5_000,
    telemetry: [
      {
        id: "scout_3",
        name: "SCOUT 3",
        batteryPercent: 73,
        altitudeMeters: 0,
        linkQualityPercent: 79,
        lastUpdate: 4_500,
        missionOnline: true,
      },
    ],
    diagnostics: [],
    pods: [
      {
        vehicleId: "scout_3",
        slotId: "top-rail",
        podSerial: "THM-11",
        podType: "thermal",
        lifecycleState: "registered",
        connected: true,
        powerReady: true,
        linkReady: false,
        capabilities: ["thermal"],
      },
    ],
  });

  assert.equal(projected.vehicles[0].pods[0].state, "warning");
  assert.equal(projected.vehicles[0].summaryChecks.sensors.state, "warning");
});

test("attached overdue pod inventory rows escalate calibration readiness without redoing date math in the viewer", () => {
  const projected = projectFleetDiagnostics({
    nowMs: 5_000,
    telemetry: [
      {
        id: "scout_5",
        name: "SCOUT 5",
        batteryPercent: 88,
        altitudeMeters: 0,
        linkQualityPercent: 81,
        lastUpdate: 4_900,
        missionOnline: true,
      },
    ],
    diagnostics: [
      {
        vehicleId: "scout_5",
        motorHealthState: "healthy",
        calibrationState: "healthy",
        imuState: "healthy",
        compassState: "healthy",
        accelerometerState: "healthy",
      },
    ],
    pods: [],
    podInventory: [
      {
        podSerial: "GAS-118",
        podType: "hazmat",
        attached: true,
        vehicleId: "scout_5",
        slotId: "belly",
        lastCalibrationAtMs: 1_700_000_000_000,
        nextCalibrationDueAtMs: 1_700_100_000_000,
        calibrationState: "overdue",
        calibrationDetail: "Overdue by 4 days",
      },
    ],
  });

  assert.equal(projected.vehicles[0].readiness, "blocked");
  assert.equal(projected.vehicles[0].summaryChecks.calibration.state, "blocked");
  assert.match(projected.vehicles[0].summaryChecks.calibration.summary, /Overdue by 4 days/);
});

test("calibration roll-up keeps the worst reported detail state", () => {
  const projected = projectFleetDiagnostics({
    nowMs: 5_000,
    telemetry: [
      {
        id: "scout_4",
        name: "SCOUT 4",
        batteryPercent: 88,
        altitudeMeters: 0,
        linkQualityPercent: 83,
        lastUpdate: 4_900,
        missionOnline: true,
      },
    ],
    diagnostics: [
      {
        vehicleId: "scout_4",
        motorHealthState: "healthy",
        calibrationState: "healthy",
        calibrationDetail: "Calibration current",
        imuState: "blocked",
        imuDetail: "Bias table missing",
        compassState: "healthy",
        compassDetail: "Compass offsets current",
        accelerometerState: "healthy",
        accelerometerDetail: "Accelerometer aligned",
      },
    ],
    pods: [],
  });

  assert.equal(projected.vehicles[0].summaryChecks.calibration.state, "blocked");
  assert.equal(projected.vehicles[0].summaryChecks.calibration.summary, "Calibration blocking readiness");
  assert.equal(projected.vehicles[0].detailChecks[0].state, "blocked");
});

test("fleet mesh average excludes offline vehicles with stale link readings", () => {
  const projected = projectFleetDiagnostics({
    nowMs: 12_000,
    telemetry: [
      {
        id: "scout_5",
        name: "SCOUT 5",
        batteryPercent: 91,
        altitudeMeters: 0,
        linkQualityPercent: 84,
        lastUpdate: 11_500,
        missionOnline: true,
      },
      {
        id: "scout_6",
        name: "SCOUT 6",
        batteryPercent: 77,
        altitudeMeters: 0,
        linkQualityPercent: 56,
        lastUpdate: 11_000,
        missionOnline: true,
      },
      {
        id: "ranger_2",
        name: "RANGER 2",
        batteryPercent: 42,
        altitudeMeters: 0,
        linkQualityPercent: 5,
        lastUpdate: 4_000,
        missionOnline: false,
      },
    ],
    diagnostics: [],
    pods: [],
  });

  assert.equal(projected.summary.connectedCount, 2);
  assert.equal(projected.summary.averageLinkQuality, 70);
});
