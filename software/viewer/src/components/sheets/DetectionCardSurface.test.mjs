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
const TEMP_ROOT = fs.mkdtempSync(path.join(os.tmpdir(), "aeris-detection-card-render-"));
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

fs.writeFileSync(path.join(TEMP_ROOT, "utils.mjs"), `
  export function cn(...values) {
    return values.flatMap((value) => Array.isArray(value) ? value : [value]).filter(Boolean).join(" ");
  }
`, "utf8");

fs.writeFileSync(path.join(TEMP_ROOT, "detectionViewState.mjs"), `
  export function getConfidenceTextClass() {
    return "text-emerald-400";
  }
`, "utf8");

fs.writeFileSync(path.join(TEMP_ROOT, "lucide-react.mjs"), `
  import { jsx as _jsx } from "react/jsx-runtime";
  function icon(name) {
    return function Icon({ className = "", ...props }) {
      return _jsx("svg", { "data-icon": name, className, ...props });
    };
  }
  export const AudioLines = icon("AudioLines");
  export const Camera = icon("Camera");
  export const Check = icon("Check");
  export const Crosshair = icon("Crosshair");
  export const Flame = icon("Flame");
  export const History = icon("History");
  export const MapPin = icon("MapPin");
  export const TimerReset = icon("TimerReset");
  export const Wind = icon("Wind");
  export const X = icon("X");
`, "utf8");

async function renderCard(props) {
  const sourcePath = path.join(ROOT, "DetectionCard.tsx");
  let source = fs.readFileSync(sourcePath, "utf8");
  source = source
    .replaceAll("@/components/ui/button", "./button.mjs")
    .replaceAll("@/lib/utils", "./utils.mjs")
    .replaceAll("@/lib/detectionViewState", "./detectionViewState.mjs")
    .replaceAll("lucide-react", "./lucide-react.mjs");

  const compiled = ts.transpileModule(source, {
    compilerOptions: {
      module: ts.ModuleKind.ES2022,
      target: ts.ScriptTarget.ES2022,
      jsx: ts.JsxEmit.ReactJSX,
    },
  }).outputText;

  const modulePath = path.join(TEMP_ROOT, `DetectionCard-${Date.now()}-${Math.random().toString(16).slice(2)}.mjs`);
  fs.writeFileSync(modulePath, compiled, "utf8");
  const { DetectionCard } = await import(pathToFileURL(modulePath).href);
  return renderToStaticMarkup(createElement(DetectionCard, props));
}

test("DetectionCard surfaces Halo evidence handles and replay/stale provenance", async () => {
  const markup = await renderCard({
    detection: {
      id: "halo-confidence-001",
      sensorType: "rgb",
      confidence: 0.91,
      confidenceLevel: "HIGH",
      timestamp: Date.now() - (3 * 60 * 1000),
      status: "new",
      vehicleId: "camera:halo_front_camera",
      vehicleName: "Halo Front Camera",
      position: [12.5, 0, -6],
      sourceModalities: ["rgb"],
      sector: "Zone E-2",
      signatureType: "Possible Survivor",
      deliveryMode: "replayed",
      isRetroactive: true,
      isStale: true,
      evidenceRef: "evidence-017",
      evidencePath: "/tmp/halo/capture-017.png",
      evidenceUri: "file:///tmp/halo/capture-017.png",
    },
    isNew: true,
    onConfirm: () => {},
    onDismiss: () => {},
    onLocate: () => {},
  });

  assert.match(markup, /Replayed/);
  assert.match(markup, /Stale/);
  assert.match(markup, /evidence-017/);
  assert.match(markup, /\/tmp\/halo\/capture-017\.png/);
  assert.match(markup, /file:\/\/\/tmp\/halo\/capture-017\.png/);
  assert.match(markup, /Possible Survivor/);
});

test("DetectionCard keeps stale-only provenance distinct from replayed detections", async () => {
  const markup = await renderCard({
    detection: {
      id: "halo-confidence-002",
      sensorType: "rgb",
      confidence: 0.64,
      confidenceLevel: "MEDIUM",
      timestamp: Date.now() - (4 * 60 * 1000),
      status: "reviewing",
      vehicleId: "camera:halo_rear_camera",
      vehicleName: "Halo Rear Camera",
      position: [4, 0, 8],
      sourceModalities: ["rgb"],
      sector: "Zone W-1",
      signatureType: "Scene Change",
      isStale: true,
    },
    isNew: false,
    onConfirm: () => {},
    onDismiss: () => {},
    onLocate: () => {},
  });

  assert.match(markup, /Stale/);
  assert.doesNotMatch(markup, /Replayed/);
});
