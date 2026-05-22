import test, { after } from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { createElement } from "react";
import ts from "typescript";
import { fileURLToPath, pathToFileURL } from "node:url";
import { renderToStaticMarkup } from "react-dom/server";

const ROOT = path.dirname(fileURLToPath(import.meta.url));
const RENDER_TEMP_ROOT = fs.mkdtempSync(path.join(os.tmpdir(), "aeris-emergency-stop-render-"));
const VIEWER_NODE_MODULES = path.resolve(ROOT, "..", "..", "..", "node_modules");

after(() => {
  fs.rmSync(RENDER_TEMP_ROOT, { recursive: true, force: true });
});

fs.symlinkSync(VIEWER_NODE_MODULES, path.join(RENDER_TEMP_ROOT, "node_modules"), "dir");

fs.writeFileSync(path.join(RENDER_TEMP_ROOT, "button.mjs"), `
  import { jsx as _jsx } from "react/jsx-runtime";
  export function Button({ children, className = "", disabled = false, ...props }) {
    return _jsx("button", { className, disabled, ...props, children });
  }
`, "utf8");

fs.writeFileSync(path.join(RENDER_TEMP_ROOT, "progress.mjs"), `
  import { jsx as _jsx } from "react/jsx-runtime";
  export function Progress({ value = 0, className = "" }) {
    return _jsx("div", { "data-progress": String(value), className });
  }
`, "utf8");

fs.writeFileSync(path.join(RENDER_TEMP_ROOT, "utils.mjs"), `
  export function cn(...values) {
    return values.flatMap((value) => Array.isArray(value) ? value : [value]).filter(Boolean).join(" ");
  }
`, "utf8");

fs.writeFileSync(path.join(RENDER_TEMP_ROOT, "lucide-react.mjs"), `
  import { jsx as _jsx } from "react/jsx-runtime";
  export function AlertTriangle({ className = "", ...props }) {
    return _jsx("svg", { "data-icon": "AlertTriangle", className, ...props });
  }
`, "utf8");

async function renderEmergencyStop(props) {
  const sourcePath = path.join(ROOT, "EmergencyStopControl.tsx");
  const holdModuleHref = pathToFileURL(path.resolve(ROOT, "..", "..", "lib", "emergencyStopHold.js")).href;
  let source = fs.readFileSync(sourcePath, "utf8");
  source = source
    .replaceAll("@/components/ui/button", "./button.mjs")
    .replaceAll("@/components/ui/progress", "./progress.mjs")
    .replaceAll("@/lib/utils", "./utils.mjs")
    .replaceAll("lucide-react", "./lucide-react.mjs")
    .replaceAll("@/lib/emergencyStopHold", holdModuleHref);

  const compiled = ts.transpileModule(source, {
    compilerOptions: {
      module: ts.ModuleKind.ES2022,
      target: ts.ScriptTarget.ES2022,
      jsx: ts.JsxEmit.ReactJSX,
    },
  }).outputText;

  const modulePath = path.join(RENDER_TEMP_ROOT, `EmergencyStopControl-${Date.now()}.mjs`);
  fs.writeFileSync(modulePath, compiled, "utf8");

  const loaded = await import(pathToFileURL(modulePath).href);
  return renderToStaticMarkup(createElement(loaded.EmergencyStopControl, props));
}

test("EmergencyStopControl renders the operator-only hold affordance", async () => {
  const markup = await renderEmergencyStop({
    missionPhase: "SEARCHING",
    canAbort: true,
    abortUnavailableReason: null,
    abortError: null,
    onAbort: () => {},
  });

  assert.match(markup, />Operator only</, "surface should stay clearly scoped to operators");
  assert.match(markup, />EMERGENCY STOP</, "surface should render the persistent emergency stop button");
  assert.match(
    markup,
    /Press and hold to dispatch the fleet abort command\./,
    "surface should render the deliberate hold instructions in the default state"
  );
});

test("EmergencyStopControl renders honest unavailable and error states", async () => {
  const unavailableMarkup = await renderEmergencyStop({
    missionPhase: "IDLE",
    canAbort: false,
    abortUnavailableReason: "Emergency stop is unavailable until a mission is active.",
    abortError: null,
    onAbort: () => {},
  });
  assert.match(
    unavailableMarkup,
    /Emergency stop is unavailable until a mission is active\./,
    "surface should expose the supplied unavailable reason when abort is disabled"
  );
  assert.match(unavailableMarkup, /disabled/, "surface should disable the button when abort is unavailable");

  const errorMarkup = await renderEmergencyStop({
    missionPhase: "SEARCHING",
    canAbort: true,
    abortUnavailableReason: null,
    abortError: "Mission abort was rejected by orchestrator.",
    onAbort: () => {},
  });
  assert.match(
    errorMarkup,
    /Mission abort was rejected by orchestrator\./,
    "surface should surface abort failures directly instead of masking them behind generic helper text"
  );
});

test("EmergencyStopControl dispatches abort only from nonce changes", () => {
  const source = fs.readFileSync(path.join(ROOT, "EmergencyStopControl.tsx"), "utf8");

  assert.match(
    source,
    /const onAbortRef = useRef\(onAbort\);/,
    "abort callback should be stored in a ref so callback identity changes do not dispatch"
  );
  assert.match(
    source,
    /useEffect\(\(\) => \{\s*onAbortRef\.current = onAbort;\s*\}, \[onAbort\]\);/s,
    "abort callback ref should stay current without driving dispatch"
  );
  assert.match(
    source,
    /onAbortRef\.current\(\);\s*\}, \[abortDispatchNonce\]\);/s,
    "abort dispatch effect should only depend on the dispatch nonce"
  );
  assert.ok(
    source.includes("shouldResetAbortRequest({ abortRequested, missionPhase })"),
    "control should explicitly clear stale abort-request state when the mission leaves the active abort lifecycle"
  );
  assert.match(
    source,
    /useEffect\(\(\) => \{\s*if \(!shouldResetAbortRequest\([\s\S]*?\)\) \{\s*return;\s*\}\s*setAbortRequested\(false\);\s*\}, \[abortRequested, missionPhase\]\);/s,
    "control should reset abortRequested from a phase-change effect instead of waiting for a remount"
  );
});
