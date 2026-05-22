import test, { after } from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { createElement } from "react";
import ts from "typescript";
import { fileURLToPath, pathToFileURL } from "node:url";
import { renderToStaticMarkup } from "react-dom/server";

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const RENDER_TEMP_ROOT = fs.mkdtempSync(path.join(os.tmpdir(), "aeris-ic-view-render-"));
const VIEWER_NODE_MODULES = path.resolve(ROOT, "..", "node_modules");

after(() => {
  fs.rmSync(RENDER_TEMP_ROOT, { recursive: true, force: true });
});

fs.symlinkSync(VIEWER_NODE_MODULES, path.join(RENDER_TEMP_ROOT, "node_modules"), "dir");

fs.writeFileSync(path.join(RENDER_TEMP_ROOT, "button.mjs"), `
  import { jsx as _jsx } from "react/jsx-runtime";
  export function Button({ children, className = "", ...props }) {
    return _jsx("button", { className, ...props, children });
  }
`, "utf8");

fs.writeFileSync(path.join(RENDER_TEMP_ROOT, "card.mjs"), `
  import { jsx as _jsx } from "react/jsx-runtime";
  export function Card({ children, className = "", ...props }) {
    return _jsx("div", { className, ...props, children });
  }
`, "utf8");

fs.writeFileSync(path.join(RENDER_TEMP_ROOT, "badge.mjs"), `
  import { jsx as _jsx } from "react/jsx-runtime";
  export function Badge({ children, className = "", ...props }) {
    return _jsx("span", { className, ...props, children });
  }
`, "utf8");

fs.writeFileSync(path.join(RENDER_TEMP_ROOT, "utils.mjs"), `
  export function cn(...values) {
    return values.flatMap((value) => Array.isArray(value) ? value : [value]).filter(Boolean).join(" ");
  }
`, "utf8");

fs.writeFileSync(path.join(RENDER_TEMP_ROOT, "degradedVehicleState.mjs"), `
  export function formatLastContactAge(value) {
    return value == null ? "--" : value + "ms";
  }
`, "utf8");

fs.writeFileSync(path.join(RENDER_TEMP_ROOT, "detectionViewState.mjs"), `
  export function getConfidenceTextClass() {
    return "text-emerald-300";
  }
`, "utf8");

fs.writeFileSync(path.join(RENDER_TEMP_ROOT, "lucide-react.mjs"), `
  import { jsx as _jsx } from "react/jsx-runtime";
  function icon(name) {
    return function Icon({ className = "", ...props }) {
      return _jsx("svg", { "data-icon": name, className, ...props });
    };
  }
  export const AlertTriangle = icon("AlertTriangle");
  export const AudioLines = icon("AudioLines");
  export const Battery = icon("Battery");
  export const BatteryFull = icon("BatteryFull");
  export const BatteryLow = icon("BatteryLow");
  export const BatteryMedium = icon("BatteryMedium");
  export const BatteryWarning = icon("BatteryWarning");
  export const Bell = icon("Bell");
  export const Check = icon("Check");
  export const ChevronRight = icon("ChevronRight");
  export const Crosshair = icon("Crosshair");
  export const Flame = icon("Flame");
  export const Gauge = icon("Gauge");
  export const Home = icon("Home");
  export const MapPin = icon("MapPin");
  export const Radio = icon("Radio");
  export const Signal = icon("Signal");
  export const Video = icon("Video");
  export const Wind = icon("Wind");
  export const X = icon("X");
  export const XCircle = icon("XCircle");
  export const Zap = icon("Zap");
`, "utf8");

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

async function renderComponent(relativePath, exportName, props) {
  const sourcePath = path.join(ROOT, relativePath);
  let source = fs.readFileSync(sourcePath, "utf8");
  source = source
    .replaceAll("@/components/ui/button", "./button.mjs")
    .replaceAll("@/components/ui/card", "./card.mjs")
    .replaceAll("@/components/ui/badge", "./badge.mjs")
    .replaceAll("@/lib/utils", "./utils.mjs")
    .replaceAll("@/lib/degradedVehicleState", "./degradedVehicleState.mjs")
    .replaceAll("@/lib/detectionViewState", "./detectionViewState.mjs")
    .replaceAll("lucide-react", "./lucide-react.mjs");

  const compiled = ts.transpileModule(source, {
    compilerOptions: {
      module: ts.ModuleKind.ES2022,
      target: ts.ScriptTarget.ES2022,
      jsx: ts.JsxEmit.ReactJSX,
    },
  }).outputText;

  const moduleName = `${path.basename(relativePath, ".tsx")}-${Date.now()}-${Math.random().toString(16).slice(2)}.mjs`;
  const modulePath = path.join(RENDER_TEMP_ROOT, moduleName);
  fs.writeFileSync(modulePath, compiled, "utf8");

  const loaded = await import(pathToFileURL(modulePath).href);
  const Component = loaded[exportName];
  assert.ok(Component, `${exportName} should be exported from ${relativePath}`);
  return renderToStaticMarkup(createElement(Component, props));
}

test("IC mode keeps one canonical page-shell flag and propagates it", () => {
  const pagePath = path.join(ROOT, "app", "page.tsx");
  const { source } = parseTsx(pagePath);

  assert.match(source, /useSearchParams/);
  assert.match(source, /EmergencyStopControl/);
  assert.match(source, /isIcViewModeQueryValue/);
  assert.match(source, /setIcViewModeEnabled/);
  assert.match(source, /viewMode=\{icViewModeEnabled \? 'ic' : 'operator'\}/);
  assert.match(source, /mode=\{icViewModeEnabled \? 'ic' : 'operator'\}/);
  assert.match(
    source,
    /topRightOverlay=\{icViewModeEnabled \? undefined : \(/,
    "operator mode should own the persistent emergency-stop surface while IC mode keeps the shell read-only"
  );
  assert.match(
    source,
    /<EmergencyStopControl[\s\S]*onAbort=\{abortMission\}/,
    "page should route the persistent control through the existing abortMission callback"
  );
  assert.match(
    source,
    /<EmergencyStopControl[\s\S]*key=\{missionPhase\}/,
    "mission-phase changes should reset the emergency-stop hold and pending state"
  );
});

test("IC mode blocks feed launch and operator keyboard shortcuts", () => {
  const pagePath = path.join(ROOT, "app", "page.tsx");
  const { source, sourceFile } = parseTsx(pagePath);

  const handleViewFeed = findFunction(sourceFile, "handleViewFeed");
  assert.ok(handleViewFeed, "page should define a dedicated feed handler");
  const handleViewFeedText = handleViewFeed.getText(sourceFile);
  assert.match(handleViewFeedText, /if \(icViewModeEnabled\) \{\s*return;\s*\}/s);
  assert.match(handleViewFeedText, /setPipVehicleId\(id\)/);
  assert.match(
    source,
    /onViewFeed=\{icViewModeEnabled \? undefined : handleViewFeed\}/,
    "page should hide the feed action entirely in IC mode instead of leaving a dead button behind"
  );
  assert.match(
    source,
    /useEffect\(\(\) => \{\s*if \(icViewModeEnabled\) \{\s*setPipVehicleId\(null\);\s*\}\s*\}, \[icViewModeEnabled\]\);/s,
    "entering IC mode should clear any already-open PiP feed"
  );
  assert.match(
    source,
    /const locateVehicleActionRef = useRef\(handleLocateVehicle\);/,
    "mock alert actions should keep a stable reference to the latest locate handler"
  );
  assert.match(
    source,
    /const viewFeedActionRef = useRef\(handleViewFeed\);/,
    "mock alert actions should keep a stable reference to the latest feed handler"
  );
  assert.match(
    source,
    /action:\s*icViewModeEnabled\s*\?\s*undefined\s*:\s*\{\s*label:\s*'VIEW',\s*onClick:\s*\(\) => viewFeedActionRef\.current\('ranger_1'\)\s*\}/s,
    "IC mode should remove feed-launch actions from mock alert surfaces as well"
  );
  assert.match(
    source,
    /\[allowMockFallback, icViewModeEnabled\]\s*\);/,
    "mock alert definitions should not resubscribe on telemetry-driven handler churn"
  );

  const keyboardEffect = source.match(/const handleKeyDown = \(e: KeyboardEvent\) => \{([\s\S]*?)\n    \};/);
  assert.ok(keyboardEffect, "page should keep keyboard handling localized");
  const keyboardText = keyboardEffect[1];
  assert.match(keyboardText, /case 'Escape':[\s\S]*if \(icViewModeEnabled\) \{\s*break;\s*\}/);
  assert.match(keyboardText, /case '1':[\s\S]*case '6':[\s\S]*if \(icViewModeEnabled\) \{\s*break;\s*\}/);
  assert.match(keyboardText, /case 'r':[\s\S]*case 'R':[\s\S]*if \(icViewModeEnabled\) \{\s*break;\s*\}/);
});

test("VehicleCard only renders the feed action when a feed callback is provided", async () => {
  const vehicle = {
    id: "scout-1",
    name: "Scout 1",
    status: "active",
    battery: 82,
    altitude: 46,
    linkQuality: 77,
    coverage: 64,
    slamMode: "vio",
  };

  const icMarkup = await renderComponent("components/sheets/VehicleCard.tsx", "VehicleCard", {
    vehicle,
    isSelected: false,
    onLocate: () => {},
    viewMode: "ic",
  });
  assert.doesNotMatch(icMarkup, />Feed</, "IC-mode vehicle cards should not render the operator video action");

  const operatorMarkup = await renderComponent("components/sheets/VehicleCard.tsx", "VehicleCard", {
    vehicle,
    isSelected: false,
    onLocate: () => {},
    onViewFeed: () => {},
    viewMode: "operator",
  });
  assert.match(operatorMarkup, />Feed</, "operator vehicle cards should still expose the feed action");
});

test("IC mode render branches use large, distance-readable tactical typography", async () => {
  const statusMarkup = await renderComponent("components/layout/StatusPill.tsx", "StatusPill", {
    missionPhase: "SEARCHING",
    elapsedTime: 95,
    progressPercent: 62,
    connectionStatus: "connected",
    alertCount: 3,
    hasUnreadAlerts: true,
    detectionCounts: {
      thermal: 2,
      acoustic: 1,
      gas: 1,
      pending: 3,
      confirmed: 2,
    },
    viewMode: "ic",
  });
  assert.match(statusMarkup, /text-xl[^"]*">AERIS</, "IC status branding should render at large-display size");
  assert.match(statusMarkup, /text-xl text-white\/90[^"]*">Progress</, "IC status labels should render above the old text-base size");
  assert.match(statusMarkup, /text-2xl text-white[^"]*">62%</, "IC progress counters should render above the old text-lg size");

  const fleetMarkup = await renderComponent("components/cards/FleetCard.tsx", "FleetCard", {
    vehicles: [{ id: "scout-1", name: "Scout 1", status: "active", battery: 82, altitude: 46 }],
    activeCount: 1,
    totalCount: 1,
    avgBattery: 82,
    avgAltitude: 46,
    warnings: [],
    viewMode: "ic",
  });
  assert.match(fleetMarkup, /text-xl text-white\/90[^"]*">Fleet</, "IC fleet headings should render above the old text-lg size");
  assert.match(fleetMarkup, /text-2xl text-white\/80[^"]*">\/1</, "IC fleet counters should keep the shared-display total legible");

  const detectionsMarkup = await renderComponent("components/cards/DetectionsCard.tsx", "DetectionsCard", {
    thermalCount: 2,
    acousticCount: 1,
    gasCount: 1,
    pendingCount: 3,
    confirmedCount: 2,
    viewMode: "ic",
  });
  assert.match(detectionsMarkup, /text-xl text-white\/90[^"]*">Detections</, "IC detection headings should render above the old text-lg size");
  assert.match(detectionsMarkup, /text-xl[^"]*">Review</, "IC detection affordances should stay readable at distance");

  const vehicleMarkup = await renderComponent("components/sheets/VehicleCard.tsx", "VehicleCard", {
    vehicle: {
      id: "scout-1",
      name: "Scout 1",
      status: "active",
      battery: 82,
      altitude: 46,
      linkQuality: 77,
      coverage: 64,
      slamMode: "vio",
    },
    isSelected: false,
    onLocate: () => {},
    onViewFeed: () => {},
    viewMode: "ic",
  });
  assert.match(vehicleMarkup, /text-xl font-medium tracking-\[0\.12em\] text-white\/60[\s\S]*?>SLAM:/, "vehicle tactical labels should render above the old text-base size");
  assert.match(vehicleMarkup, /text-xl text-white\/50 uppercase tracking-wide[\s\S]*?>Alt \(m\)</, "vehicle metric captions should render above the old text-base size");
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
