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
