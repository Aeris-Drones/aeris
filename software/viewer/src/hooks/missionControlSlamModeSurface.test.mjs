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

test("useMissionControl captures and preserves per-vehicle slam modes from mission progress", () => {
  const hookPath = path.join(ROOT, "hooks", "useMissionControl.ts");
  const { sourceFile } = parseSource(hookPath, ts.ScriptKind.TS);

  let vehicleSlamModesTypeMatches = false;
  let reusesMissionMetaParser = false;
  let guardsEmptySlamModeUpdates = false;
  let mergesSlamModesIntoExistingState = false;

  walk(sourceFile, (node) => {
    if (
      ts.isPropertySignature(node) &&
      node.name.getText(sourceFile) === "vehicleSlamModes" &&
      node.type?.getText(sourceFile) === "Record<string, string>"
    ) {
      vehicleSlamModesTypeMatches = true;
    }

    if (
      ts.isCallExpression(node) &&
      node.expression.getText(sourceFile) === "extractVehicleMissionMetaFromProgressPayload"
    ) {
      reusesMissionMetaParser = true;
    }

    if (
      ts.isIfStatement(node) &&
      node.expression.getText(sourceFile) === "Object.keys(slamModes).length > 0"
    ) {
      guardsEmptySlamModeUpdates = true;
    }

    if (
      ts.isCallExpression(node) &&
      node.expression.getText(sourceFile) === "setVehicleSlamModes" &&
      node.arguments.length === 1 &&
      ts.isArrowFunction(node.arguments[0])
    ) {
      const updater = node.arguments[0];
      if (
        updater.parameters[0]?.name.getText(sourceFile) === "previous" &&
        updater.body.getText(sourceFile).includes("...previous") &&
        updater.body.getText(sourceFile).includes("...slamModes")
      ) {
        mergesSlamModesIntoExistingState = true;
      }
    }
  });

  assert.ok(
    reusesMissionMetaParser,
    "useMissionControl should reuse the mission progress vehicle-meta parser"
  );
  assert.ok(
    vehicleSlamModesTypeMatches,
    "useMissionControl should expose normalized vehicle slam modes"
  );
  assert.ok(
    guardsEmptySlamModeUpdates,
    "useMissionControl should avoid clearing known slam modes when progress payloads omit metadata"
  );
  assert.ok(
    mergesSlamModesIntoExistingState,
    "useMissionControl should merge new slam mode updates into the existing state"
  );
});

test("page projects mission-control slam modes into fleet vehicle cards", () => {
  const pagePath = path.join(ROOT, "app", "page.tsx");
  const { sourceFile } = parseSource(pagePath, ts.ScriptKind.TSX);

  let wiresVehicleSlamMode = false;

  walk(sourceFile, (node) => {
    if (
      ts.isPropertyAssignment(node) &&
      node.name.getText(sourceFile) === "slamMode" &&
      node.initializer.getText(sourceFile) ===
        "vehicleSlamModes[normalizeVehicleId(vehicle.id)]"
    ) {
      wiresVehicleSlamMode = true;
    }
  });

  assert.ok(
    wiresVehicleSlamMode,
    "page.tsx should pass the active slam mode into fleet vehicle card data"
  );
});
