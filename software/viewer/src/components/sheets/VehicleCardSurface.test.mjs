import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import ts from "typescript";
import { fileURLToPath } from "node:url";

const ROOT = path.dirname(fileURLToPath(import.meta.url));

function parseTsx(filePath) {
  const source = fs.readFileSync(filePath, "utf8");
  const sourceFile = ts.createSourceFile(
    filePath,
    source,
    ts.ScriptTarget.Latest,
    true,
    ts.ScriptKind.TSX
  );
  return { source, sourceFile };
}

function walk(node, visitor) {
  visitor(node);
  ts.forEachChild(node, (child) => walk(child, visitor));
}

test("VehicleCard keeps a dedicated SLAM mode label bound to vehicle metadata", () => {
  const cardPath = path.join(ROOT, "VehicleCard.tsx");
  const { sourceFile } = parseTsx(cardPath);

  let hasSlamModeTestId = false;
  let formatsVehicleSlamMode = false;

  walk(sourceFile, (node) => {
    if (
      ts.isJsxAttribute(node) &&
      node.name.text === "data-testid" &&
      node.initializer &&
      ts.isStringLiteral(node.initializer) &&
      node.initializer.text === "vehicle-slam-mode"
    ) {
      hasSlamModeTestId = true;
    }

    if (
      ts.isCallExpression(node) &&
      node.expression.getText(sourceFile) === "formatSlamModeLabel" &&
      node.arguments.length === 1 &&
      node.arguments[0].getText(sourceFile) === "vehicle.slamMode"
    ) {
      formatsVehicleSlamMode = true;
    }
  });

  assert.ok(
    hasSlamModeTestId,
    "VehicleCard should keep the slam mode label easy to target in follow-up tests"
  );
  assert.ok(
    formatsVehicleSlamMode,
    "VehicleCard should render the current slam mode label on the live status card"
  );
});

test("VehicleCard slam mode formatter preserves explicit labels and UNKNOWN fallback", () => {
  const cardPath = path.join(ROOT, "VehicleCard.tsx");
  const { sourceFile } = parseTsx(cardPath);

  let formatter = null;
  walk(sourceFile, (node) => {
    if (
      ts.isFunctionDeclaration(node) &&
      node.name?.text === "formatSlamModeLabel"
    ) {
      formatter = node;
    }
  });

  assert.ok(formatter, "VehicleCard should define a dedicated slam mode formatter");

  const returnedLabels = new Set();
  let checksUnknownLiteral = false;
  let checksMissingValue = false;

  walk(formatter, (node) => {
    if (ts.isReturnStatement(node) && node.expression && ts.isStringLiteral(node.expression)) {
      returnedLabels.add(node.expression.text);
    }

    if (
      ts.isBinaryExpression(node) &&
      node.left.getText(sourceFile) === "normalized" &&
      node.operatorToken.kind === ts.SyntaxKind.EqualsEqualsEqualsToken &&
      ts.isStringLiteral(node.right) &&
      node.right.text === "unknown"
    ) {
      checksUnknownLiteral = true;
    }

    if (
      ts.isPrefixUnaryExpression(node) &&
      node.operator === ts.SyntaxKind.ExclamationToken &&
      node.operand.getText(sourceFile) === "normalized"
    ) {
      checksMissingValue = true;
    }
  });

  assert.ok(
    checksMissingValue && checksUnknownLiteral,
    "VehicleCard should treat missing and explicit unknown slam modes as the UNKNOWN fallback"
  );
  assert.ok(returnedLabels.has("UNKNOWN"), "VehicleCard should render UNKNOWN for missing slam modes");
  assert.ok(returnedLabels.has("VIO"), "VehicleCard should present the VIO label cleanly");
  assert.ok(returnedLabels.has("LIO-SAM"), "VehicleCard should present the LIO-SAM label cleanly");
});

test("VehicleCard exposes offline last-known status and timer copy", () => {
  const cardPath = path.join(ROOT, "VehicleCard.tsx");
  const { source, sourceFile } = parseTsx(cardPath);

  assert.match(source, /offline:/, "VehicleCard should define an offline status config");
  assert.match(source, /OFFLINE/, "VehicleCard should render an explicit OFFLINE label");
  assert.match(source, /vehicle-last-known-age/, "VehicleCard should expose a targeted last-contact timer");

  let importsLastContactFormatter = false;
  let formatsLastContactAge = false;
  walk(sourceFile, (node) => {
    if (
      ts.isImportDeclaration(node) &&
      node.moduleSpecifier.getText(sourceFile).includes("degradedVehicleState")
    ) {
      importsLastContactFormatter = true;
    }
    if (
      ts.isCallExpression(node) &&
      node.expression.getText(sourceFile) === "formatLastContactAge" &&
      node.arguments[0]?.getText(sourceFile) === "vehicle.lastContactAgeMs"
    ) {
      formatsLastContactAge = true;
    }
  });

  assert.ok(importsLastContactFormatter);
  assert.ok(formatsLastContactAge);
});
