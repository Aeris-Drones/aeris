import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import ts from "typescript";
import { fileURLToPath } from "node:url";

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");

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

function findFunction(sourceFile, name) {
  let match = null;
  function walk(node) {
    if (match) return;
    if ((ts.isFunctionDeclaration(node) || ts.isVariableDeclaration(node)) && node.name?.getText(sourceFile) === name) {
      match = node;
      return;
    }
    ts.forEachChild(node, walk);
  }
  walk(sourceFile);
  return match;
}

test("IC mode keeps one canonical page-shell flag and propagates it", () => {
  const pagePath = path.join(ROOT, "app", "page.tsx");
  const { source } = parseTsx(pagePath);

  assert.match(source, /useSearchParams/);
  assert.match(source, /isIcViewModeQueryValue/);
  assert.match(source, /setIcViewModeEnabled/);
  assert.match(source, /viewMode=\{icViewModeEnabled \? 'ic' : 'operator'\}/);
  assert.match(source, /mode=\{icViewModeEnabled \? 'ic' : 'operator'\}/);
});

test("IC mode blocks feed launch and operator keyboard shortcuts", () => {
  const pagePath = path.join(ROOT, "app", "page.tsx");
  const { source, sourceFile } = parseTsx(pagePath);

  const handleViewFeed = findFunction(sourceFile, "handleViewFeed");
  assert.ok(handleViewFeed, "page should define a dedicated feed handler");
  const handleViewFeedText = handleViewFeed.getText(sourceFile);
  assert.match(handleViewFeedText, /if \(icViewModeEnabled\) \{\s*return;\s*\}/s);
  assert.match(handleViewFeedText, /setPipVehicleId\(id\)/);

  const keyboardEffect = source.match(/const handleKeyDown = \(e: KeyboardEvent\) => \{([\s\S]*?)\n    \};/);
  assert.ok(keyboardEffect, "page should keep keyboard handling localized");
  const keyboardText = keyboardEffect[1];
  assert.match(keyboardText, /case 'Escape':[\s\S]*if \(icViewModeEnabled\) \{\s*break;\s*\}/);
  assert.match(keyboardText, /case '1':[\s\S]*case '6':[\s\S]*if \(icViewModeEnabled\) \{\s*break;\s*\}/);
  assert.match(keyboardText, /case 'r':[\s\S]*case 'R':[\s\S]*if \(icViewModeEnabled\) \{\s*break;\s*\}/);
});

test("FleetSheet only exposes feed action when the parent passes it", () => {
  const fleetSheetPath = path.join(ROOT, "components", "sheets", "FleetSheet.tsx");
  const { source } = parseTsx(fleetSheetPath);

  assert.match(source, /onViewFeed=\{\(\) => handleViewFeed\(vehicle\.id\)\}/);
});

test("IC mode typography upgrades keep key tactical labels at text-base or larger", () => {
  const files = [
    path.join(ROOT, "app", "page.tsx"),
    path.join(ROOT, "components", "layout", "StatusPill.tsx"),
    path.join(ROOT, "components", "cards", "FleetCard.tsx"),
    path.join(ROOT, "components", "cards", "DetectionsCard.tsx"),
    path.join(ROOT, "components", "sheets", "VehicleCard.tsx"),
    path.join(ROOT, "components", "sheets", "DetectionCard.tsx"),
  ];

  const combined = files.map((file) => fs.readFileSync(file, "utf8")).join("\n");
  assert.match(combined, /text-base text-white\/75">fleet active/);
  assert.match(combined, /text-base text-white\/75">pending alerts/);
  assert.match(combined, /isIcMode \? 'text-lg' : 'text-xs'/);
  assert.match(combined, /isIcMode \? 'text-lg text-white\/85' : 'text-xs text-white\/50'/);
  assert.match(combined, /text-base font-medium tracking-\[0\.12em\] text-white\/55/);
  assert.match(combined, /text-base text-white\/45 uppercase tracking-wide">Alt \(m\)</);
});

test("DetectionCard read-only mode keeps locate while removing triage actions", () => {
  const detectionCardPath = path.join(ROOT, "components", "sheets", "DetectionCard.tsx");
  const { source } = parseTsx(detectionCardPath);

  const actionableBlock = source.match(/\{isActionable && \(([\s\S]*?)\n\s*\)\}/);
  assert.ok(actionableBlock, "DetectionCard should keep a dedicated actionable footer");
  assert.match(actionableBlock[1], /Locate/);

  const readOnlyBranch = source.match(/\{!readOnly && \(([\s\S]*?)\n\s*\)\}/);
  assert.ok(readOnlyBranch, "triage controls should be gated behind readOnly");
  assert.match(readOnlyBranch[1], /Dismiss/);
  assert.match(readOnlyBranch[1], /Confirm/);
});
