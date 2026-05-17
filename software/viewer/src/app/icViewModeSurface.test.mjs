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

function walk(node, visitor) {
  visitor(node);
  ts.forEachChild(node, (child) => walk(child, visitor));
}

test("page owns one URL-aware IC view mode flag", () => {
  const pagePath = path.join(ROOT, "app", "page.tsx");
  const { source, sourceFile } = parseTsx(pagePath);

  assert.match(source, /useSearchParams/, "IC mode should use App Router search params");
  assert.match(source, /isIcViewModeQueryValue/, "IC mode should normalize URL values in one helper");
  assert.match(source, /setIcViewModeEnabled/, "IC mode should have one canonical page-shell state setter");

  let passesModeToLayout = false;
  let passesModeToDock = false;
  walk(sourceFile, (node) => {
    if (
      ts.isJsxAttribute(node) &&
      node.name.text === "viewMode" &&
      node.initializer?.getText(sourceFile).includes("icViewModeEnabled ? 'ic' : 'operator'")
    ) {
      passesModeToLayout = true;
    }
    if (
      ts.isJsxAttribute(node) &&
      node.name.text === "mode" &&
      node.initializer?.getText(sourceFile).includes("icViewModeEnabled ? 'ic' : 'operator'")
    ) {
      passesModeToDock = true;
    }
  });

  assert.ok(passesModeToLayout, "layout should receive the canonical IC mode flag");
  assert.ok(passesModeToDock, "command dock should receive the canonical IC mode flag");
});

test("IC mode guards state-changing mission, detection, zone, and vehicle actions", () => {
  const pagePath = path.join(ROOT, "app", "page.tsx");
  const { source } = parseTsx(pagePath);

  assert.match(source, /if \(icViewModeEnabled\) \{\s*return;\s*\}\s*setDetectionStatusOverrides/s);
  assert.match(source, /onDroneSelect=\{icViewModeEnabled \? undefined : handleDroneSelect\}/);
  assert.match(source, /onDetectionSelect=\{icViewModeEnabled \? handleLocateDetection : handleDetectionSelect\}/);
  assert.match(source, /onZoneSelect=\{icViewModeEnabled \? undefined : selectZone\}/);
  assert.match(source, /onAddZonePoint=\{icViewModeEnabled \? undefined : addPoint\}/);
  assert.match(source, /onRouteStagingAreaSet=\{icViewModeEnabled \? undefined : setRouteStagingAreaPosition\}/);
  assert.doesNotMatch(source, /zoneToolbar=\{<ZoneToolbar \/>}/);
});

test("IC mode keeps tactical summaries visible while removing operator controls", () => {
  const pagePath = path.join(ROOT, "app", "page.tsx");
  const { source } = parseTsx(pagePath);

  assert.match(source, /icTacticalSummary/, "page should compose a dedicated IC tactical summary");
  assert.match(source, /text-base/, "IC tactical text should be distance-readable");
  assert.match(source, /controlsCard=\{icViewModeEnabled \? undefined : \(/);
  assert.match(source, /readOnly=\{icViewModeEnabled\}/);
  assert.match(source, /IC VIEW/, "page should expose a visible mode toggle label");
});
