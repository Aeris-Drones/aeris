import test, { after } from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import ts from "typescript";
import { pathToFileURL } from "node:url";

const ROOT = path.resolve(new URL(".", import.meta.url).pathname);
const sourcePath = path.join(ROOT, "usePodCalibrationAction.ts");
const source = fs.readFileSync(sourcePath, "utf8");
const tempDir = fs.mkdtempSync(path.join(os.tmpdir(), "aeris-pod-calibration-action-"));

after(() => {
  fs.rmSync(tempDir, { recursive: true, force: true });
});

fs.writeFileSync(path.join(tempDir, "react.mjs"), `
  export function useCallback(fn) { return fn; }
  export function useEffect() {}
  export function useMemo(fn) { return fn(); }
  export function useState(initial) { return [initial, () => {}]; }
`, "utf8");

fs.writeFileSync(path.join(tempDir, "roslib.mjs"), `export default {};`, "utf8");
fs.writeFileSync(path.join(tempDir, "context.mjs"), `export function useSharedROSConnection() { return { ros: null, isConnected: false }; }`, "utf8");
fs.writeFileSync(path.join(tempDir, "missionControlBehavior.mjs"), `export function withServiceTimeout() { throw new Error("unused in test"); }`, "utf8");
fs.writeFileSync(path.join(tempDir, "podInventory.mjs"), `
  export function parseLogPodCalibrationResponse(value) { return value; }
`, "utf8");

const compiled = ts.transpileModule(
  source
    .replaceAll("react", "./react.mjs")
    .replaceAll("roslib", "./roslib.mjs")
    .replaceAll("@/context/ROSConnectionContext", "./context.mjs")
    .replaceAll("@/lib/missionControlBehavior.js", "./missionControlBehavior.mjs")
    .replaceAll("@/lib/ros/podInventory", "./podInventory.mjs"),
  {
    compilerOptions: {
      module: ts.ModuleKind.ES2022,
      target: ts.ScriptTarget.ES2022,
    },
  }
).outputText;

const modulePath = path.join(tempDir, "usePodCalibrationAction.mjs");
fs.writeFileSync(modulePath, compiled, "utf8");

const {
  POD_CALIBRATION_SERVICE_TIMEOUT_MS,
  reconcilePodCalibrationActionState,
} = await import(pathToFileURL(modulePath).href);

test("pod calibration requests use a short service timeout", () => {
  assert.equal(POD_CALIBRATION_SERVICE_TIMEOUT_MS, 15_000);
});

test("reconcilePodCalibrationActionState keeps submit freshness state when inventory has not advanced yet", () => {
  const reconciled = reconcilePodCalibrationActionState({
    actionStatesBySerial: {
      GAS_118: { kind: "success", message: "Calibration logged for GAS-118" },
      LDR_551: { kind: "error", message: "keep me" },
    },
    inventoryBaselineBySerial: {
      GAS_118: 1_710_000_000_000,
    },
    records: [
      {
        podSerial: "GAS_118",
        lastCalibrationAtMs: 1_710_000_000_000,
      },
    ],
  });

  assert.deepEqual(reconciled.actionStatesBySerial.GAS_118, {
    kind: "success",
    message: "Calibration logged for GAS-118",
  });
  assert.deepEqual(reconciled.actionStatesBySerial.LDR_551, {
    kind: "error",
    message: "keep me",
  });
  assert.equal(reconciled.inventoryBaselineBySerial.GAS_118, 1_710_000_000_000);
});

test("reconcilePodCalibrationActionState clears stale success state after fresher inventory arrives", () => {
  const reconciled = reconcilePodCalibrationActionState({
    actionStatesBySerial: {
      GAS_118: { kind: "success", message: "Calibration logged for GAS-118" },
    },
    inventoryBaselineBySerial: {
      GAS_118: 1_710_000_000_000,
    },
    records: [
      {
        podSerial: "GAS_118",
        lastCalibrationAtMs: 1_720_000_000_000,
      },
    ],
  });

  assert.equal(reconciled.actionStatesBySerial.GAS_118, null);
  assert.equal("GAS_118" in reconciled.inventoryBaselineBySerial, false);
});
