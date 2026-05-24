import test, { after } from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { createElement } from "react";
import { renderToStaticMarkup } from "react-dom/server";
import ts from "typescript";
import { fileURLToPath, pathToFileURL } from "node:url";

const ROOT = path.dirname(fileURLToPath(import.meta.url));
const TEMP_ROOT = fs.mkdtempSync(path.join(os.tmpdir(), "aeris-maintenance-card-render-"));
const VIEWER_NODE_MODULES = path.resolve(ROOT, "..", "..", "..", "node_modules");

after(() => {
  fs.rmSync(TEMP_ROOT, { recursive: true, force: true });
});

fs.symlinkSync(VIEWER_NODE_MODULES, path.join(TEMP_ROOT, "node_modules"), "dir");

fs.writeFileSync(path.join(TEMP_ROOT, "button.mjs"), `
  import { jsx as _jsx } from "react/jsx-runtime";
  export function Button({ children, className = "", ...props }) {
    return _jsx("button", { className, ...props, children });
  }
`, "utf8");

fs.writeFileSync(path.join(TEMP_ROOT, "card.mjs"), `
  import { jsx as _jsx } from "react/jsx-runtime";
  export function Card({ children, className = "", ...props }) {
    return _jsx("div", { className, ...props, children });
  }
`, "utf8");

fs.writeFileSync(path.join(TEMP_ROOT, "badge.mjs"), `
  import { jsx as _jsx } from "react/jsx-runtime";
  export function Badge({ children, className = "", ...props }) {
    return _jsx("span", { className, ...props, children });
  }
`, "utf8");

fs.writeFileSync(path.join(TEMP_ROOT, "utils.mjs"), `
  export function cn(...values) {
    return values.flatMap((value) => Array.isArray(value) ? value : [value]).filter(Boolean).join(" ");
  }
`, "utf8");

fs.writeFileSync(path.join(TEMP_ROOT, "maintenanceDiagnostics.mjs"), `
  export function getDiagnosticBadgeVariant(state) {
    if (state === "healthy") return "success";
    if (state === "blocked") return "danger";
    if (state === "warning") return "warning";
    return "outline";
  }
  export function getReadinessBadgeVariant(readiness) {
    if (readiness === "ready") return "success";
    if (readiness === "blocked") return "danger";
    return "warning";
  }
`, "utf8");

fs.writeFileSync(path.join(TEMP_ROOT, "degradedVehicleState.mjs"), `
  export function formatLastContactAge(value) {
    return value == null ? "--" : Math.floor(value / 1000) + "s";
  }
`, "utf8");

fs.writeFileSync(path.join(TEMP_ROOT, "firmwareUpdatePanel.mjs"), `
  import { jsx as _jsx } from "react/jsx-runtime";
  export function FirmwareUpdatePanel() {
    return _jsx("div", { "data-testid": "firmware-update-panel", children: "Firmware update panel" });
  }
`, "utf8");

fs.writeFileSync(path.join(TEMP_ROOT, "firmwareUpdateStatus.mjs"), `
  export function getFirmwareUpdateBadgeVariant() {
    return "outline";
  }
  export function isFirmwareUpdateActive() {
    return false;
  }
`, "utf8");

fs.writeFileSync(path.join(TEMP_ROOT, "lucide-react.mjs"), `
  import { jsx as _jsx } from "react/jsx-runtime";
  function icon(name) {
    return function Icon({ className = "", ...props }) {
      return _jsx("svg", { "data-icon": name, className, ...props });
    };
  }
  export const Activity = icon("Activity");
  export const Battery = icon("Battery");
  export const ChevronDown = icon("ChevronDown");
  export const ChevronUp = icon("ChevronUp");
  export const Compass = icon("Compass");
  export const Cpu = icon("Cpu");
  export const Gauge = icon("Gauge");
  export const Radio = icon("Radio");
  export const ShieldAlert = icon("ShieldAlert");
`, "utf8");

async function renderCard(props) {
  const sourcePath = path.join(ROOT, "MaintenanceVehicleCard.tsx");
  let source = fs.readFileSync(sourcePath, "utf8");
  source = source
    .replaceAll("@/components/ui/button", "./button.mjs")
    .replaceAll("@/components/ui/card", "./card.mjs")
    .replaceAll("@/components/ui/badge", "./badge.mjs")
    .replaceAll("@/components/maintenance/FirmwareUpdatePanel", "./firmwareUpdatePanel.mjs")
    .replaceAll("@/lib/utils", "./utils.mjs")
    .replaceAll("@/lib/maintenanceDiagnostics", "./maintenanceDiagnostics.mjs")
    .replaceAll("@/lib/degradedVehicleState", "./degradedVehicleState.mjs")
    .replaceAll("@/lib/ros/firmwareUpdateStatus", "./firmwareUpdateStatus.mjs")
    .replaceAll("lucide-react", "./lucide-react.mjs");

  const compiled = ts.transpileModule(source, {
    compilerOptions: {
      module: ts.ModuleKind.ES2022,
      target: ts.ScriptTarget.ES2022,
      jsx: ts.JsxEmit.ReactJSX,
    },
  }).outputText;

  const modulePath = path.join(TEMP_ROOT, `MaintenanceVehicleCard-${Date.now()}-${Math.random().toString(16).slice(2)}.mjs`);
  fs.writeFileSync(modulePath, compiled, "utf8");
  const { MaintenanceVehicleCard } = await import(pathToFileURL(modulePath).href);
  return renderToStaticMarkup(createElement(MaintenanceVehicleCard, props));
}

const vehicle = {
  id: "scout_2",
  name: "SCOUT 2",
  readiness: "warning",
  readinessSummary: "Diagnostics need maintenance review",
  batteryPercent: 54,
  altitudeMeters: 0,
  lastContactAgeMs: 1900,
  isOffline: false,
  summaryChecks: {
    motor: { label: "Motor health", state: "warning", summary: "Motor 3 current ripple above baseline" },
    sensors: { label: "Sensors", state: "warning", summary: "1 pod needs attention" },
    mesh: { label: "Mesh radio", state: "warning", summary: "58% degraded mesh link" },
    calibration: { label: "Calibration", state: "warning", summary: "Compass recalibration due soon" },
  },
  detailChecks: [
    { label: "IMU calibration", state: "healthy", summary: "IMU drift nominal" },
    { label: "Compass calibration", state: "warning", summary: "Recheck before next sortie" },
    { label: "Accelerometer", state: "healthy", summary: "Accelerometer aligned" },
  ],
  pods: [
    {
      slotId: "belly",
      podSerial: "GAS-9",
      podType: "hazmat",
      lifecycleLabel: "Registered",
      state: "warning",
      capabilities: ["gas", "hazmat"],
    },
  ],
};

test("MaintenanceVehicleCard keeps calibration details hidden until expanded", async () => {
  const markup = await renderCard({
    vehicle,
    expanded: false,
    onToggle: () => {},
  });

  assert.match(markup, /Show details/);
  assert.match(markup, /Motor health/);
  assert.doesNotMatch(markup, /IMU calibration/);
  assert.doesNotMatch(markup, /Attached pods/);
});

test("MaintenanceVehicleCard renders expanded calibration and pod detail states", async () => {
  const markup = await renderCard({
    vehicle,
    expanded: true,
    onToggle: () => {},
  });

  assert.match(markup, /Hide details/);
  assert.match(markup, /IMU calibration/);
  assert.match(markup, /Compass calibration/);
  assert.match(markup, /Accelerometer/);
  assert.match(markup, /Attached pods/);
  assert.match(markup, /hazmat/);
  assert.match(markup, /GAS-9/);
  assert.match(markup, /aria-expanded="true"/);
});

test("MaintenanceVehicleCard tolerates missing calibration detail rows", async () => {
  const markup = await renderCard({
    vehicle: {
      ...vehicle,
      detailChecks: [vehicle.detailChecks[0]],
    },
    expanded: true,
    onToggle: () => {},
  });

  assert.match(markup, /IMU calibration/);
  assert.doesNotMatch(markup, /Compass calibration/);
  assert.doesNotMatch(markup, /Accelerometer/);
  assert.match(markup, /Attached pods/);
});

test("MaintenanceVehicleCard renders a safe altitude fallback for non-finite values", async () => {
  const markup = await renderCard({
    vehicle: {
      ...vehicle,
      altitudeMeters: Number.NaN,
    },
    expanded: false,
    onToggle: () => {},
  });

  assert.match(markup, />--</);
  assert.doesNotMatch(markup, /NaNm/);
});
