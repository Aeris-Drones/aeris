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
const TEMP_ROOT = fs.mkdtempSync(path.join(os.tmpdir(), "aeris-firmware-update-panel-"));
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

fs.writeFileSync(path.join(TEMP_ROOT, "progress.mjs"), `
  import { jsx as _jsx } from "react/jsx-runtime";
  export function Progress({ value = 0, className = "", indicatorColor, ...props }) {
    return _jsx("div", { className, "data-progress": value, "data-color": indicatorColor, ...props });
  }
`, "utf8");

fs.writeFileSync(path.join(TEMP_ROOT, "utils.mjs"), `
  export function cn(...values) {
    return values.flatMap((value) => Array.isArray(value) ? value : [value]).filter(Boolean).join(" ");
  }
`, "utf8");

fs.writeFileSync(path.join(TEMP_ROOT, "firmwareUpdateStatus.mjs"), `
  export function getFirmwareUpdateBadgeVariant(state) {
    if (state === "complete") return "success";
    if (state === "rolled_back") return "warning";
    if (state === "failed") return "danger";
    return "info";
  }
  export function hasFreshFirmwareUpdateStatus(status) {
    return ["applying", "verifying", "complete", "failed", "rolling_back", "rolled_back"].includes(status?.lifecycleState);
  }
  export function isFirmwareUpdateActive(status) {
    return status?.lifecycleState === "applying" || status?.lifecycleState === "verifying";
  }
`, "utf8");

fs.writeFileSync(path.join(TEMP_ROOT, "lucide-react.mjs"), `
  import { jsx as _jsx } from "react/jsx-runtime";
  function icon(name) {
    return function Icon({ className = "", ...props }) {
      return _jsx("svg", { "data-icon": name, className, ...props });
    };
  }
  export const ArrowUpCircle = icon("ArrowUpCircle");
  export const RotateCcw = icon("RotateCcw");
  export const ShieldCheck = icon("ShieldCheck");
  export const ShieldX = icon("ShieldX");
  export const HardDriveDownload = icon("HardDriveDownload");
`, "utf8");

async function renderPanel(props) {
  const sourcePath = path.join(ROOT, "FirmwareUpdatePanel.tsx");
  let source = fs.readFileSync(sourcePath, "utf8");
  source = source
    .replaceAll("@/components/ui/button", "./button.mjs")
    .replaceAll("@/components/ui/badge", "./badge.mjs")
    .replaceAll("@/components/ui/progress", "./progress.mjs")
    .replaceAll("@/lib/utils", "./utils.mjs")
    .replaceAll("@/lib/ros/firmwareUpdateStatus", "./firmwareUpdateStatus.mjs")
    .replaceAll("lucide-react", "./lucide-react.mjs");

  const compiled = ts.transpileModule(source, {
    compilerOptions: {
      module: ts.ModuleKind.ES2022,
      target: ts.ScriptTarget.ES2022,
      jsx: ts.JsxEmit.ReactJSX,
    },
  }).outputText;

  const modulePath = path.join(TEMP_ROOT, `FirmwareUpdatePanel-${Date.now()}-${Math.random().toString(16).slice(2)}.mjs`);
  fs.writeFileSync(modulePath, compiled, "utf8");
  const { FirmwareUpdatePanel } = await import(pathToFileURL(modulePath).href);
  return renderToStaticMarkup(createElement(FirmwareUpdatePanel, props));
}

async function loadPanelModule() {
  const sourcePath = path.join(ROOT, "FirmwareUpdatePanel.tsx");
  let source = fs.readFileSync(sourcePath, "utf8");
  source = source
    .replaceAll("@/components/ui/button", "./button.mjs")
    .replaceAll("@/components/ui/badge", "./badge.mjs")
    .replaceAll("@/components/ui/progress", "./progress.mjs")
    .replaceAll("@/lib/utils", "./utils.mjs")
    .replaceAll("@/lib/ros/firmwareUpdateStatus", "./firmwareUpdateStatus.mjs")
    .replaceAll("lucide-react", "./lucide-react.mjs");

  const compiled = ts.transpileModule(source, {
    compilerOptions: {
      module: ts.ModuleKind.ES2022,
      target: ts.ScriptTarget.ES2022,
      jsx: ts.JsxEmit.ReactJSX,
    },
  }).outputText;

  const modulePath = path.join(TEMP_ROOT, `FirmwareUpdatePanel-helpers-${Date.now()}-${Math.random().toString(16).slice(2)}.mjs`);
  fs.writeFileSync(modulePath, compiled, "utf8");
  return import(pathToFileURL(modulePath).href);
}

const baseStatus = {
  vehicleId: "scout_2",
  packageId: "fw-2026.05.23",
  currentVersion: "2026.04.9",
  targetVersion: "2026.05.23",
  lifecycleState: "applying",
  lifecycleLabel: "Applying",
  progressPercent: 63.5,
  activeSlot: "A",
  inactiveSlot: "B",
  rollbackPerformed: false,
  statusDetail: "Writing inactive partition",
};

test("FirmwareUpdatePanel renders in-flight progress and disables restart while update is active", async () => {
  const markup = await renderPanel({
    vehicleName: "SCOUT 2",
    status: baseStatus,
    isSubmitting: false,
    actionError: null,
    onSubmit: () => {},
  });

  assert.match(markup, /Firmware update/);
  assert.match(markup, /Applying/);
  assert.match(markup, /63.5%/);
  assert.match(markup, /Current firmware/);
  assert.match(markup, /2026.04.9/);
  assert.match(markup, /Target firmware/);
  assert.match(markup, /2026.05.23/);
  assert.match(markup, /disabled=""/);
  assert.match(markup, /Writing inactive partition/);
  assert.match(markup, /Signature token/);
  assert.doesNotMatch(markup, /value="signed-manifest"/);
});

test("FirmwareUpdatePanel surfaces rollback outcomes with terminal detail", async () => {
  const markup = await renderPanel({
    vehicleName: "SCOUT 2",
    status: {
      ...baseStatus,
      lifecycleState: "rolled_back",
      lifecycleLabel: "Rolled back",
      progressPercent: 100,
      rollbackPerformed: true,
      errorDetail: "Post-update healthcheck failed",
      statusDetail: "Vehicle returned to slot A",
    },
    isSubmitting: false,
    actionError: null,
    onSubmit: () => {},
  });

  assert.match(markup, /Rolled back/);
  assert.match(markup, /Vehicle returned to slot A/);
  assert.match(markup, /Post-update healthcheck failed/);
  assert.match(markup, /<button[^>]*disabled=""/);
});

test("FirmwareUpdatePanel suppresses stale action errors when a fresher live status is present", async () => {
  const markup = await renderPanel({
    vehicleName: "SCOUT 2",
    status: {
      ...baseStatus,
      lifecycleState: "verifying",
      lifecycleLabel: "Verifying",
      statusDetail: "Booted slot B; running healthcheck",
    },
    isSubmitting: false,
    actionError: "firmware update service call timed out after 8000ms",
    onSubmit: () => {},
  });

  assert.match(markup, /Booted slot B; running healthcheck/);
  assert.doesNotMatch(markup, /timed out after 8000ms/);
});

test("FirmwareUpdatePanel keeps action errors visible when only terminal status is present", async () => {
  const markup = await renderPanel({
    vehicleName: "SCOUT 2",
    status: {
      ...baseStatus,
      lifecycleState: "complete",
      lifecycleLabel: "Complete",
      statusDetail: "Vehicle healthy on slot B",
    },
    isSubmitting: false,
    actionError: "signature token was rejected",
    onSubmit: () => {},
  });

  assert.match(markup, /signature token was rejected/);
  assert.doesNotMatch(markup, /Vehicle healthy on slot B/);
});

test("FirmwareUpdatePanel requires non-empty command fields before enabling submit", async () => {
  const markup = await renderPanel({
    vehicleName: "SCOUT 2",
    status: null,
    isSubmitting: false,
    actionError: null,
    onSubmit: () => {},
  });

  assert.equal((markup.match(/required=""/g) ?? []).length, 4);
  assert.match(markup, /<button[^>]*disabled=""/);
});

test("resolvePackageUriForTargetVersionChange keeps the default URI aligned until the operator customizes it", async () => {
  const {
    buildDefaultFirmwarePackageUri,
    isFirmwareUpdateCommandReady,
    resolvePackageUriForTargetVersionChange,
  } = await loadPanelModule();

  assert.equal(
    resolvePackageUriForTargetVersionChange({
      seed: "scout-2",
      nextTargetVersion: "2026.06.01",
      currentPackageUri: buildDefaultFirmwarePackageUri("scout-2", "2026.05.23"),
      isPackageUriCustomized: false,
    }),
    "s3://updates/scout-2/2026.06.01.bin"
  );

  assert.equal(
    resolvePackageUriForTargetVersionChange({
      seed: "scout-2",
      nextTargetVersion: "2026.06.01",
      currentPackageUri: "https://signed.example.com/scout-2/fw.bin",
      isPackageUriCustomized: true,
    }),
    "https://signed.example.com/scout-2/fw.bin"
  );

  assert.equal(
    isFirmwareUpdateCommandReady({
      packageId: " fw-2026.05.23 ",
      targetVersion: " 2026.05.23 ",
      packageUri: " s3://updates/scout-2/2026.05.23.bin ",
      packageSignature: " signed-token ",
    }),
    true
  );

  assert.equal(
    isFirmwareUpdateCommandReady({
      packageId: "fw-2026.05.23",
      targetVersion: "2026.05.23",
      packageUri: "s3://updates/scout-2/2026.05.23.bin",
      packageSignature: "   ",
    }),
    false
  );
});
