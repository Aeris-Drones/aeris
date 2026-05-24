import test, { after } from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { pathToFileURL } from "node:url";
import ts from "typescript";

const sourcePath = new URL("./podInventory.ts", import.meta.url);
const source = fs.readFileSync(sourcePath, "utf8");
const tempDir = fs.mkdtempSync(path.join(os.tmpdir(), "aeris-pod-inventory-"));

after(() => {
  fs.rmSync(tempDir, { recursive: true, force: true });
});

const compiled = ts.transpileModule(source, {
  compilerOptions: {
    module: ts.ModuleKind.ES2022,
    target: ts.ScriptTarget.ES2022,
  },
}).outputText;

const modulePath = path.join(tempDir, "podInventory.mjs");
fs.writeFileSync(modulePath, compiled, "utf8");

const {
  getCalibrationBadgeVariant,
  parseLogPodCalibrationResponse,
  parsePodInventoryArray,
} = await import(pathToFileURL(modulePath).href);

test("parsePodInventoryArray normalizes attached and detached known pod inventory rows", () => {
  const parsed = parsePodInventoryArray({
    observed_at: { sec: 42, nanosec: 250_000_000 },
    records: [
      {
        pod_serial: "GAS-118",
        pod_type: "hazmat",
        attached: true,
        vehicle_id: "scout_2",
        slot_id: "belly",
        last_calibration: { sec: 100, nanosec: 0 },
        next_calibration_due: { sec: 200, nanosec: 0 },
        calibration_state: 2,
        calibration_detail: "Due in 10 days",
      },
      {
        podSerial: "LDR-551",
        podType: "lidar",
        attached: false,
        calibrationState: "overdue",
        calibrationDetail: "Overdue by 4 days",
      },
      {
        pod_serial: "   ",
      },
    ],
  });

  assert.equal(parsed.observedAtMs, 42_250);
  assert.equal(parsed.records.length, 2);
  assert.equal(parsed.records[0].calibrationState, "due_soon");
  assert.equal(parsed.records[0].vehicleId, "scout_2");
  assert.equal(parsed.records[1].attached, false);
  assert.equal(parsed.records[1].calibrationState, "overdue");
});

test("parseLogPodCalibrationResponse keeps typed record payloads on success and failure", () => {
  const success = parseLogPodCalibrationResponse({
    accepted: true,
    message: "Calibration logged for GAS-118",
    failure_code: "",
    record: {
      pod_serial: "GAS-118",
      pod_type: "hazmat",
      attached: true,
      calibration_state: 1,
      calibration_detail: "Current until 2026-11-10",
    },
  });

  assert.equal(success.accepted, true);
  assert.equal(success.record?.podSerial, "GAS-118");
  assert.equal(success.record?.calibrationState, "current");

  const failure = parseLogPodCalibrationResponse({
    accepted: false,
    message: "Pod is detached",
    failureCode: "pod_not_connected",
    record: {},
  });

  assert.equal(failure.accepted, false);
  assert.equal(failure.failureCode, "pod_not_connected");
  assert.equal(failure.record, null);
});

test("getCalibrationBadgeVariant distinguishes current, due soon, overdue, and unknown states", () => {
  assert.equal(getCalibrationBadgeVariant("current"), "success");
  assert.equal(getCalibrationBadgeVariant("due_soon"), "warning");
  assert.equal(getCalibrationBadgeVariant("overdue"), "danger");
  assert.equal(getCalibrationBadgeVariant("unknown"), "outline");
});
