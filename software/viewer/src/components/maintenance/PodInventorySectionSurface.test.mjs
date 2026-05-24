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
const TEMP_ROOT = fs.mkdtempSync(path.join(os.tmpdir(), "aeris-pod-inventory-section-"));
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

fs.writeFileSync(path.join(TEMP_ROOT, "badge.mjs"), `
  import { jsx as _jsx } from "react/jsx-runtime";
  export function Badge({ children, className = "", ...props }) {
    return _jsx("span", { className, ...props, children });
  }
`, "utf8");

fs.writeFileSync(path.join(TEMP_ROOT, "card.mjs"), `
  import { jsx as _jsx } from "react/jsx-runtime";
  export function Card({ children, className = "", ...props }) {
    return _jsx("section", { className, ...props, children });
  }
`, "utf8");

fs.writeFileSync(path.join(TEMP_ROOT, "utils.mjs"), `
  export function cn(...values) {
    return values.flatMap((value) => Array.isArray(value) ? value : [value]).filter(Boolean).join(" ");
  }
`, "utf8");

fs.writeFileSync(path.join(TEMP_ROOT, "podInventory.mjs"), `
  export function getCalibrationBadgeVariant(state) {
    if (state === "current") return "success";
    if (state === "due_soon") return "warning";
    if (state === "overdue") return "danger";
    return "outline";
  }
  export function formatInventoryTimestamp(value) {
    if (value == null) return "Not recorded";
    return new Date(value).toISOString().slice(0, 10);
  }
`, "utf8");

fs.writeFileSync(path.join(TEMP_ROOT, "lucide-react.mjs"), `
  import { jsx as _jsx } from "react/jsx-runtime";
  function icon(name) {
    return function Icon({ className = "", ...props }) {
      return _jsx("svg", { "data-icon": name, className, ...props });
    };
  }
  export const ClipboardCheck = icon("ClipboardCheck");
  export const PlugZap = icon("PlugZap");
  export const Radio = icon("Radio");
`, "utf8");

async function renderSection(props) {
  const sourcePath = path.join(ROOT, "PodInventorySection.tsx");
  let source = fs.readFileSync(sourcePath, "utf8");
  source = source
    .replaceAll("@/components/ui/button", "./button.mjs")
    .replaceAll("@/components/ui/badge", "./badge.mjs")
    .replaceAll("@/components/ui/card", "./card.mjs")
    .replaceAll("@/lib/utils", "./utils.mjs")
    .replaceAll("@/lib/ros/podInventory", "./podInventory.mjs")
    .replaceAll("lucide-react", "./lucide-react.mjs");

  const compiled = ts.transpileModule(source, {
    compilerOptions: {
      module: ts.ModuleKind.ES2022,
      target: ts.ScriptTarget.ES2022,
      jsx: ts.JsxEmit.ReactJSX,
    },
  }).outputText;

  const modulePath = path.join(TEMP_ROOT, `PodInventorySection-${Date.now()}-${Math.random().toString(16).slice(2)}.mjs`);
  fs.writeFileSync(modulePath, compiled, "utf8");
  const { PodInventorySection } = await import(pathToFileURL(modulePath).href);
  return renderToStaticMarkup(createElement(PodInventorySection, props));
}

const inventory = [
  {
    podSerial: "GAS-118",
    podType: "hazmat",
    attached: true,
    vehicleId: "scout_2",
    slotId: "belly",
    lastCalibrationAtMs: 1_710_000_000_000,
    nextCalibrationDueAtMs: 1_711_000_000_000,
    calibrationState: "due_soon",
    calibrationDetail: "Due in 11 days",
  },
  {
    podSerial: "LDR-551",
    podType: "lidar",
    attached: false,
    vehicleId: "",
    slotId: "",
    lastCalibrationAtMs: 1_700_000_000_000,
    nextCalibrationDueAtMs: 1_700_100_000_000,
    calibrationState: "overdue",
    calibrationDetail: "Overdue by 4 days",
  },
];

test("PodInventorySection renders known pods and calibration urgency badges", async () => {
  const markup = await renderSection({
    inventory,
    submittingPodSerial: null,
    actionStateBySerial: {},
    onSubmitCalibration: () => {},
  });

  assert.match(markup, /Pod inventory/);
  assert.match(markup, /GAS-118/);
  assert.match(markup, /LDR-551/);
  assert.match(markup, /Due in 11 days/);
  assert.match(markup, /Overdue by 4 days/);
});

test("PodInventorySection disables calibration logging for detached pods and surfaces success feedback for connected ones", async () => {
  const markup = await renderSection({
    inventory,
    submittingPodSerial: null,
    actionStateBySerial: {
      "GAS-118": { kind: "success", message: "Calibration logged for GAS-118" },
    },
    onSubmitCalibration: () => {},
  });

  assert.match(markup, /Calibration logged for GAS-118/);
  assert.match(markup, /Connect pod to log calibration/);
  assert.match(markup, /<button[^>]*disabled=""/);
});

test("mergeCalibrationDraft preserves the untouched sibling date field", async () => {
  const sourcePath = path.join(ROOT, "PodInventorySection.tsx");
  let source = fs.readFileSync(sourcePath, "utf8");
  source = source
    .replaceAll("@/components/ui/button", "./button.mjs")
    .replaceAll("@/components/ui/badge", "./badge.mjs")
    .replaceAll("@/components/ui/card", "./card.mjs")
    .replaceAll("@/lib/utils", "./utils.mjs")
    .replaceAll("@/lib/ros/podInventory", "./podInventory.mjs")
    .replaceAll("lucide-react", "./lucide-react.mjs");

  const compiled = ts.transpileModule(source, {
    compilerOptions: {
      module: ts.ModuleKind.ES2022,
      target: ts.ScriptTarget.ES2022,
      jsx: ts.JsxEmit.ReactJSX,
    },
  }).outputText;

  const modulePath = path.join(TEMP_ROOT, `PodInventorySection-helpers-${Date.now()}-${Math.random().toString(16).slice(2)}.mjs`);
  fs.writeFileSync(modulePath, compiled, "utf8");
  const { mergeCalibrationDraft } = await import(pathToFileURL(modulePath).href);

  const merged = mergeCalibrationDraft(
    inventory[0],
    undefined,
    "lastCalibration",
    "2026-05-24"
  );

  assert.equal(merged.lastCalibration, "2026-05-24");
  assert.equal(
    merged.nextCalibrationDue,
    new Date(inventory[0].nextCalibrationDueAtMs).toISOString().slice(0, 10)
  );
});
