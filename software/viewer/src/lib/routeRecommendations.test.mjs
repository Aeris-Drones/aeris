import test from "node:test";
import assert from "node:assert/strict";

import {
  DEFAULT_ROUTE_STAGING_AREA,
  deriveRouteRecommendations,
  extractRouteBlockers,
  selectSurvivorRouteTargets,
} from "./routeRecommendations.js";

const nowMs = 1_800_000_000_000;

function detection(overrides = {}) {
  return {
    id: "det-target",
    sensorType: "thermal",
    confidence: 0.9,
    confidenceLevel: "HIGH",
    timestamp: nowMs - 10_000,
    status: "new",
    vehicleId: "scout-1",
    vehicleName: "Scout 1",
    position: [100, 0, 0],
    sourceModalities: ["thermal"],
    signatureType: "Human signature likely",
    ...overrides,
  };
}

test("selectSurvivorRouteTargets excludes gas-only and hazard-only detections", () => {
  const targets = selectSurvivorRouteTargets([
    detection({ id: "thermal-survivor", sensorType: "thermal" }),
    detection({
      id: "gas-hazard",
      sensorType: "gas",
      sourceModalities: ["gas"],
      signatureType: "Hazardous levels",
      confidence: 0.98,
      confidenceLevel: "HIGH",
    }),
    detection({
      id: "collapse-zone",
      sensorType: "thermal",
      routeBlockerType: "structural",
      signatureType: "Structural collapse area",
      confidence: 0.96,
      confidenceLevel: "HIGH",
    }),
    detection({
      id: "low-confidence",
      sensorType: "acoustic",
      confidence: 0.42,
      confidenceLevel: "LOW",
      signatureType: "Movement sounds",
    }),
  ]);

  assert.deepEqual(targets.map((target) => target.id), ["thermal-survivor"]);
});

test("extractRouteBlockers uses gas geometry and explicit structural blocker metadata", () => {
  const blockers = extractRouteBlockers([
    detection({
      id: "gas-poly",
      sensorType: "gas",
      geometry: [
        [30, 0, -10],
        [60, 0, -10],
        [60, 0, 10],
        [30, 0, 10],
      ],
    }),
    detection({
      id: "structural-poly",
      sensorType: "thermal",
      routeBlockerType: "structural",
      geometry: [
        [-20, 0, -20],
        [-5, 0, -20],
        [-5, 0, -5],
        [-20, 0, -5],
      ],
    }),
    detection({
      id: "thermal-outline-only",
      sensorType: "thermal",
      geometry: [
        [10, 0, 10],
        [15, 0, 10],
        [15, 0, 15],
      ],
    }),
  ]);

  assert.deepEqual(
    blockers.map((blocker) => [blocker.id, blocker.type]),
    [
      ["gas-poly", "gas"],
      ["structural-poly", "structural"],
    ]
  );
});

test("deriveRouteRecommendations bends advisory routes around blocking hazards", () => {
  const routes = deriveRouteRecommendations({
    detections: [
      detection({
        id: "survivor-a",
        position: [100, 0, 0],
      }),
      detection({
        id: "gas-blocker",
        sensorType: "gas",
        sourceModalities: ["gas"],
        signatureType: "Elevated CO levels",
        geometry: [
          [35, 0, -12],
          [65, 0, -12],
          [65, 0, 12],
          [35, 0, 12],
        ],
      }),
    ],
    stagingArea: DEFAULT_ROUTE_STAGING_AREA,
    nowMs,
  });

  assert.equal(routes.length, 1);
  assert.equal(routes[0].status, "clear");
  assert.deepEqual(routes[0].blockingHazardIds, ["gas-blocker"]);
  assert.ok(routes[0].polyline.length > 2, "route should include a dogleg around the blocker");
  assert.deepEqual(routes[0].polyline[0], DEFAULT_ROUTE_STAGING_AREA.position);
  assert.deepEqual(routes[0].polyline.at(-1), [100, 0, 0]);
});

test("deriveRouteRecommendations marks stale/replayed inputs and clears routes when inputs disappear", () => {
  const staleRoutes = deriveRouteRecommendations({
    detections: [
      detection({
        id: "survivor-replayed",
        deliveryMode: "replayed",
        isRetroactive: true,
        timestamp: nowMs - 10_000,
      }),
    ],
    nowMs,
  });

  assert.equal(staleRoutes.length, 1);
  assert.equal(staleRoutes[0].status, "stale");
  assert.equal(staleRoutes[0].freshness.source, "replayed");

  const clearedRoutes = deriveRouteRecommendations({
    detections: [],
    nowMs,
  });

  assert.deepEqual(clearedRoutes, []);
});
