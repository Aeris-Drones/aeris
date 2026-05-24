import test, { after } from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { pathToFileURL } from "node:url";
import ts from "typescript";

const sourcePath = new URL("./firmwareUpdateStatus.ts", import.meta.url);
const source = fs.readFileSync(sourcePath, "utf8");
const tempDir = fs.mkdtempSync(path.join(os.tmpdir(), "aeris-firmware-update-status-"));

after(() => {
  fs.rmSync(tempDir, { recursive: true, force: true });
});

const compiled = ts.transpileModule(source, {
  compilerOptions: {
    module: ts.ModuleKind.ES2022,
    target: ts.ScriptTarget.ES2022,
  },
}).outputText;

const modulePath = path.join(tempDir, "firmwareUpdateStatus.mjs");
fs.writeFileSync(modulePath, compiled, "utf8");

const {
  parseFirmwareUpdateStatusArray,
  getFirmwareUpdateBadgeVariant,
} = await import(pathToFileURL(modulePath).href);

test("parseFirmwareUpdateStatusArray normalizes lifecycle, versions, slots, and rollback metadata", () => {
  const parsed = parseFirmwareUpdateStatusArray({
    observed_at: { sec: 42, nanosec: 500_000_000 },
    updates: [
      {
        vehicle_id: "scout_2",
        package_id: "fw-2026.05.23",
        current_version: "2026.04.9",
        target_version: "2026.05.23",
        lifecycle_state: 4,
        progress_percent: 63.5,
        active_slot: "A",
        inactive_slot: "B",
        rollback_performed: false,
        status_detail: "Writing inactive partition",
      },
      {
        vehicleId: "scout_3",
        packageId: "fw-2026.05.24",
        currentVersion: "2026.05.1",
        targetVersion: "2026.05.24",
        lifecycleState: "rolled_back",
        progressPercent: 100,
        activeSlot: "A",
        inactiveSlot: "B",
        rollbackPerformed: true,
        errorDetail: "Post-update healthcheck failed",
      },
      {
        vehicle_id: "   ",
      },
    ],
  });

  assert.equal(parsed.observedAtMs, 42_500);
  assert.equal(parsed.updates.length, 2);

  assert.deepEqual(parsed.updates[0], {
    vehicleId: "scout_2",
    packageId: "fw-2026.05.23",
    currentVersion: "2026.04.9",
    targetVersion: "2026.05.23",
    lifecycleState: "applying",
    lifecycleLabel: "Applying",
    progressPercent: 63.5,
    activeSlot: "A",
    inactiveSlot: "B",
    rollbackPerformed: false,
    statusDetail: "Writing inactive partition",
    errorCode: undefined,
    errorDetail: undefined,
  });

  assert.equal(parsed.updates[1].lifecycleState, "rolled_back");
  assert.equal(parsed.updates[1].rollbackPerformed, true);
  assert.equal(parsed.updates[1].errorDetail, "Post-update healthcheck failed");
});

test("parseFirmwareUpdateStatusArray rejects invalid payload roots", () => {
  assert.throws(
    () => parseFirmwareUpdateStatusArray(null),
    /Invalid firmware update data/
  );
});

test("getFirmwareUpdateBadgeVariant distinguishes active, complete, and failure states", () => {
  assert.equal(getFirmwareUpdateBadgeVariant("downloading"), "info");
  assert.equal(getFirmwareUpdateBadgeVariant("complete"), "success");
  assert.equal(getFirmwareUpdateBadgeVariant("rolled_back"), "warning");
  assert.equal(getFirmwareUpdateBadgeVariant("failed"), "danger");
});
