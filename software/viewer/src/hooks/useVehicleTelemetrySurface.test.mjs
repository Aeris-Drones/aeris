import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import ts from "typescript";
import { fileURLToPath } from "node:url";

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");

function parseSource(filePath, scriptKind) {
  const source = fs.readFileSync(filePath, "utf8");
  const sourceFile = ts.createSourceFile(
    filePath,
    source,
    ts.ScriptTarget.Latest,
    true,
    scriptKind
  );
  return { sourceFile };
}

function walk(node, visitor) {
  visitor(node);
  ts.forEachChild(node, (child) => walk(child, visitor));
}

test("useVehicleTelemetry allows callers to disable redundant subscriptions", () => {
  const hookPath = path.join(ROOT, "hooks", "useVehicleTelemetry.ts");
  const { sourceFile } = parseSource(hookPath, ts.ScriptKind.TS);

  let exposesSubscribeTelemetryOption = false;
  let exposesSubscribeMissionProgressOption = false;
  let guardsTelemetryTopic = false;
  let guardsProgressTopic = false;
  let guardsReplayTopic = false;

  walk(sourceFile, (node) => {
    if (
      ts.isPropertySignature(node) &&
      node.name.getText(sourceFile) === "subscribeTelemetry"
    ) {
      exposesSubscribeTelemetryOption = true;
    }

    if (
      ts.isPropertySignature(node) &&
      node.name.getText(sourceFile) === "subscribeMissionProgress"
    ) {
      exposesSubscribeMissionProgressOption = true;
    }

    if (
      ts.isVariableDeclaration(node) &&
      node.name.getText(sourceFile) === "topic" &&
      node.initializer?.getText(sourceFile).includes("subscribeTelemetry")
    ) {
      guardsTelemetryTopic = true;
    }

    if (
      ts.isVariableDeclaration(node) &&
      node.name.getText(sourceFile) === "progressTopic" &&
      node.initializer?.getText(sourceFile).includes("subscribeMissionProgress")
    ) {
      guardsProgressTopic = true;
    }

    if (
      ts.isVariableDeclaration(node) &&
      node.name.getText(sourceFile) === "replayTopic" &&
      node.initializer?.getText(sourceFile).includes("subscribeTelemetry")
    ) {
      guardsReplayTopic = true;
    }
  });

  assert.ok(
    exposesSubscribeTelemetryOption,
    "useVehicleTelemetry should let callers opt out of live telemetry subscriptions"
  );
  assert.ok(
    exposesSubscribeMissionProgressOption,
    "useVehicleTelemetry should let callers opt out of mission-progress subscriptions"
  );
  assert.ok(
    guardsTelemetryTopic,
    "useVehicleTelemetry should only create the telemetry topic when it is needed"
  );
  assert.ok(
    guardsProgressTopic,
    "useVehicleTelemetry should only create the mission-progress topic when it is needed"
  );
  assert.ok(
    guardsReplayTopic,
    "useVehicleTelemetry should only create the replay annotation topic when live telemetry is enabled"
  );
});

test("MapScene3D disables duplicate subscriptions when parent props already provide data", () => {
  const mapScenePath = path.join(ROOT, "components", "map", "MapScene3D.tsx");
  const { sourceFile } = parseSource(mapScenePath, ts.ScriptKind.TSX);

  let tracksMissingVehicleProps = false;
  let tracksMissingReturnTrajectoryProps = false;
  let passesTelemetryOptOut = false;
  let passesProgressOptOut = false;

  walk(sourceFile, (node) => {
    if (
      ts.isVariableDeclaration(node) &&
      node.name.getText(sourceFile) === "needsLiveVehicles" &&
      node.initializer?.getText(sourceFile) === "vehiclesProp === undefined"
    ) {
      tracksMissingVehicleProps = true;
    }

    if (
      ts.isVariableDeclaration(node) &&
      node.name.getText(sourceFile) === "needsLiveReturnTrajectories" &&
      node.initializer?.getText(sourceFile) === "returnTrajectoriesProp === undefined"
    ) {
      tracksMissingReturnTrajectoryProps = true;
    }

    if (
      ts.isPropertyAssignment(node) &&
      node.name.getText(sourceFile) === "subscribeTelemetry" &&
      node.initializer.getText(sourceFile) === "needsLiveVehicles"
    ) {
      passesTelemetryOptOut = true;
    }

    if (
      ts.isPropertyAssignment(node) &&
      node.name.getText(sourceFile) === "subscribeMissionProgress" &&
      node.initializer.getText(sourceFile) === "needsLiveReturnTrajectories"
    ) {
      passesProgressOptOut = true;
    }
  });

  assert.ok(
    tracksMissingVehicleProps,
    "MapScene3D should detect when the parent has already provided vehicles"
  );
  assert.ok(
    tracksMissingReturnTrajectoryProps,
    "MapScene3D should detect when the parent has already provided return trajectories"
  );
  assert.ok(
    passesTelemetryOptOut,
    "MapScene3D should disable the live telemetry subscription when parent vehicles are present"
  );
  assert.ok(
    passesProgressOptOut,
    "MapScene3D should disable the mission-progress subscription when parent trajectories are present"
  );
});

test("MapScene3D derives offline marker and stale tile ownership from shared state", () => {
  const mapScenePath = path.join(ROOT, "components", "map", "MapScene3D.tsx");
  const source = fs.readFileSync(mapScenePath, "utf8");

  assert.match(
    source,
    /vehicleMissionMeta\?: MissionMetaMaps/,
    "MapScene3D should accept mission metadata from the shared page projection"
  );
  assert.match(
    source,
    /deriveVehicleDegradedState/,
    "MapScene3D should use the canonical degraded-state helper for marker status"
  );
  assert.match(
    source,
    /offlineVehicleIds\.has\(normalizeVehicleId\(tile\.sourceVehicleId\)\)/,
    "MapScene3D should only stale-style tiles owned by offline vehicles"
  );
});

const SOURCE = fs.readFileSync(path.join(ROOT, "hooks", "useVehicleTelemetry.ts"), "utf8");

test("buildTelemetryDedupeKey recognizes snake_case and camelCase vehicle ids", () => {
  assert.match(
    SOURCE,
    /vehicle_id\?: unknown; vehicleId\?: unknown/,
    "telemetry dedupe keys should read both snake_case and camelCase vehicle ids"
  );
});

test("buildTelemetryDedupeKey drops payloads without a usable vehicle id", () => {
  assert.match(
    SOURCE,
    /if \(!vehicleId\) {\s*return null;\s*}/,
    "telemetry dedupe keys should be skipped when vehicle ids are missing"
  );
});
