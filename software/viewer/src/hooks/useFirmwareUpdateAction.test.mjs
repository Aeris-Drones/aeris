import test, { after } from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import ts from "typescript";
import { pathToFileURL } from "node:url";

const ROOT = path.resolve(new URL(".", import.meta.url).pathname);
const sourcePath = path.join(ROOT, "useFirmwareUpdateAction.ts");
const source = fs.readFileSync(sourcePath, "utf8");
const tempDir = fs.mkdtempSync(path.join(os.tmpdir(), "aeris-firmware-update-action-"));

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
fs.writeFileSync(path.join(tempDir, "firmwareUpdateStatus.mjs"), `
  export function hasFreshFirmwareUpdateStatus(status) {
    return ["downloading", "validating", "applying", "verifying", "complete", "failed", "rolling_back", "rolled_back"].includes(status?.lifecycleState);
  }
`, "utf8");

const compiled = ts.transpileModule(
  source
    .replaceAll("react", "./react.mjs")
    .replaceAll("roslib", "./roslib.mjs")
    .replaceAll("@/context/ROSConnectionContext", "./context.mjs")
    .replaceAll("@/lib/missionControlBehavior.js", "./missionControlBehavior.mjs")
    .replaceAll("@/lib/ros/firmwareUpdateStatus", "./firmwareUpdateStatus.mjs"),
  {
    compilerOptions: {
      module: ts.ModuleKind.ES2022,
      target: ts.ScriptTarget.ES2022,
    },
  }
).outputText;

const modulePath = path.join(tempDir, "useFirmwareUpdateAction.mjs");
fs.writeFileSync(modulePath, compiled, "utf8");

const { reconcileFirmwareUpdateActionState } = await import(pathToFileURL(modulePath).href);

test("reconcileFirmwareUpdateActionState clears stale errors and pending state when live status arrives", () => {
  const reconciled = reconcileFirmwareUpdateActionState({
    errorsByVehicle: {
      scout_2: "firmware update service call timed out after 8000ms",
      scout_3: "keep me",
    },
    submittingVehicleId: "scout_2",
    statuses: [
      {
        vehicleId: "scout_2",
        lifecycleState: "applying",
      },
    ],
  });

  assert.equal(reconciled.errorsByVehicle.scout_2, null);
  assert.equal(reconciled.errorsByVehicle.scout_3, "keep me");
  assert.equal(reconciled.submittingVehicleId, null);
});

test("reconcileFirmwareUpdateActionState leaves errors alone when no fresh status is available", () => {
  const current = {
    errorsByVehicle: {
      scout_2: "still relevant",
    },
    submittingVehicleId: "scout_2",
    statuses: [
      {
        vehicleId: "scout_2",
        lifecycleState: "idle",
      },
    ],
  };

  const reconciled = reconcileFirmwareUpdateActionState(current);

  assert.deepEqual(reconciled.errorsByVehicle, current.errorsByVehicle);
  assert.equal(reconciled.submittingVehicleId, "scout_2");
});
