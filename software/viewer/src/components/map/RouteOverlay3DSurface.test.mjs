import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";

const ROOT = process.cwd();

test("LayerVisibilityContext exposes a dedicated routes layer", () => {
  const source = fs.readFileSync(
    path.join(ROOT, "src", "context", "LayerVisibilityContext.tsx"),
    "utf8"
  );

  assert.match(source, /routes:\s*boolean/, "layer state should include routes");
  assert.match(source, /routes:\s*true/, "routes should be visible by default");
});

test("LayersPanel renders a route toggle separate from trajectories", () => {
  const source = fs.readFileSync(
    path.join(ROOT, "src", "components", "layers", "LayersPanel.tsx"),
    "utf8"
  );

  assert.match(source, /id:\s*'routes'/, "layers panel should define a route layer item");
  assert.match(source, /label:\s*'Routes'/, "route layer should have its own operator label");
});

test("MapScene3D passes route recommendations through a dedicated overlay component", () => {
  const source = fs.readFileSync(
    path.join(ROOT, "src", "components", "map", "MapScene3D.tsx"),
    "utf8"
  );

  assert.match(source, /RouteOverlay3D/, "MapScene3D should render the dedicated route overlay");
  assert.match(source, /visibility\.routes/, "route overlays should use the dedicated layer toggle");
  assert.doesNotMatch(
    source,
    /visibility\.trajectories\s*&&\s*routeRecommendations/,
    "routes should not piggyback on trajectory visibility"
  );
});

test("RouteOverlay3D distinguishes pending route labels from clear routes", () => {
  const source = fs.readFileSync(
    path.join(ROOT, "src", "components", "map", "RouteOverlay3D.tsx"),
    "utf8"
  );

  assert.match(source, /Pending route/, "pending routes should render a dedicated label");
  assert.match(source, /Stale route/, "stale routes should keep their dedicated label");
  assert.match(source, /Entry route/, "clear routes should keep their normal label");
});

test("page.tsx consumes shared staging and structural hazard state instead of a local default-only staging point", () => {
  const source = fs.readFileSync(
    path.join(ROOT, "src", "app", "page.tsx"),
    "utf8"
  );

  assert.match(source, /structuralHazardZones/, "page should read shared structural hazard zones");
  assert.match(source, /routeStagingArea/, "page should read the shared route staging area");
  assert.match(source, /nowMs:\s*routeNowMs/, "route freshness should be recomputed from a live clock");
  assert.doesNotMatch(
    source,
    /useState<[^>]*RouteStagingArea[^>]*>\(DEFAULT_ROUTE_STAGING_AREA\)/,
    "page should not keep route staging as a local default-only state"
  );
});

test("ZoneToolbar exposes explicit controls for structural hazards and staging placement", () => {
  const source = fs.readFileSync(
    path.join(ROOT, "src", "components", "zones", "ZoneToolbar.tsx"),
    "utf8"
  );

  assert.match(source, /Hazard Zone/, "toolbar should let operators draw structural hazard polygons");
  assert.match(source, /Set Staging/, "toolbar should let operators place the staging area");
});
