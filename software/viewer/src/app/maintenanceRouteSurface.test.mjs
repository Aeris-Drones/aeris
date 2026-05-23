import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");

test("operator and maintenance routes both use the shared viewer provider seam", () => {
  const providersSource = fs.readFileSync(path.join(ROOT, "app", "ViewerAppProviders.tsx"), "utf8");
  const operatorSource = fs.readFileSync(path.join(ROOT, "app", "page.tsx"), "utf8");
  const maintenanceSource = fs.readFileSync(path.join(ROOT, "app", "maintenance", "page.tsx"), "utf8");

  assert.match(providersSource, /ROSConnectionProvider/, "shared providers should own the ROS connection surface");
  assert.match(providersSource, /MissionProvider/, "shared providers should include the mission context");
  assert.match(operatorSource, /ViewerAppProviders/, "operator page should reuse the shared viewer providers");
  assert.match(maintenanceSource, /ViewerAppProviders/, "maintenance page should reuse the shared viewer providers");
});

test("viewer route navigation exposes both operations and maintenance destinations", () => {
  const navSource = fs.readFileSync(path.join(ROOT, "components", "layout", "ViewerRouteNav.tsx"), "utf8");
  const operatorSource = fs.readFileSync(path.join(ROOT, "app", "page.tsx"), "utf8");
  const maintenanceSource = fs.readFileSync(path.join(ROOT, "app", "maintenance", "page.tsx"), "utf8");

  assert.match(navSource, /href="\/"/, "route nav should retain the operator landing route");
  assert.match(navSource, /href="\/maintenance"/, "route nav should expose the maintenance route");
  assert.match(operatorSource, /ViewerRouteNav currentRoute="operations"/, "operator page should wire the route nav");
  assert.match(maintenanceSource, /MaintenanceDashboardPage/, "maintenance route should render the dedicated diagnostics dashboard");
});
