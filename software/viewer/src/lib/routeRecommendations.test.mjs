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
    detection({
      id: "dismissed-gas",
      sensorType: "gas",
      status: "dismissed",
      geometry: [
        [70, 0, -8],
        [88, 0, -8],
        [88, 0, 8],
        [70, 0, 8],
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

test("extractRouteBlockers uses the caller-provided clock for freshness", () => {
  const blockers = extractRouteBlockers([
    detection({
      id: "gas-stale",
      sensorType: "gas",
      timestamp: nowMs - (3 * 60 * 1000),
      geometry: [
        [30, 0, -10],
        [60, 0, -10],
        [60, 0, 10],
        [30, 0, 10],
      ],
    }),
  ], nowMs);

  assert.equal(blockers.length, 1);
  assert.equal(blockers[0].freshness.source, "stale");
  assert.equal(blockers[0].freshness.ageMs, 3 * 60 * 1000);
});

test("deriveRouteRecommendations excludes dismissed hazards and explicit structural hazard zones from the shared planning path", () => {
  const routes = deriveRouteRecommendations({
    detections: [
      detection({
        id: "survivor-b",
        position: [120, 0, 0],
      }),
      detection({
        id: "dismissed-blocker",
        sensorType: "gas",
        status: "dismissed",
        sourceModalities: ["gas"],
        geometry: [
          [20, 0, -10],
          [40, 0, -10],
          [40, 0, 10],
          [20, 0, 10],
        ],
      }),
    ],
    structuralHazards: [
      {
        id: "collapse-zone-1",
        kind: "structural_hazard",
        name: "Collapsed atrium",
        priority: 1,
        status: "active",
        polygon: [
          { x: 55, z: -12 },
          { x: 82, z: -12 },
          { x: 82, z: 12 },
          { x: 55, z: 12 },
        ],
        createdAt: nowMs - 1_000,
      },
    ],
    nowMs,
  });

  assert.equal(routes.length, 1);
  assert.deepEqual(routes[0].blockingHazardIds, ["collapse-zone-1"]);
  assert.ok(
    !routes[0].blockingHazardIds.includes("dismissed-blocker"),
    "dismissed hazards should not continue to block routes"
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

test("deriveRouteRecommendations does not detour around diagonal blockers that do not intersect the path", () => {
  const routes = deriveRouteRecommendations({
    detections: [
      detection({
        id: "survivor-diagonal",
        position: [100, 0, 100],
      }),
      detection({
        id: "gas-off-path",
        sensorType: "gas",
        sourceModalities: ["gas"],
        geometry: [
          [20, 0, 80],
          [30, 0, 80],
          [30, 0, 90],
          [20, 0, 90],
        ],
      }),
    ],
    stagingArea: DEFAULT_ROUTE_STAGING_AREA,
    nowMs,
  });

  assert.equal(routes.length, 1);
  assert.deepEqual(routes[0].blockingHazardIds, []);
  assert.deepEqual(routes[0].polyline, [
    DEFAULT_ROUTE_STAGING_AREA.position,
    [100, 0, 100],
  ]);
});

test("deriveRouteRecommendations keeps the first detour waypoint from retreating behind the staging area", () => {
  const routes = deriveRouteRecommendations({
    detections: [
      detection({
        id: "survivor-forward",
        position: [100, 0, 0],
      }),
      detection({
        id: "gas-overlap-start",
        sensorType: "gas",
        sourceModalities: ["gas"],
        geometry: [
          [-5, 0, -12],
          [20, 0, -12],
          [20, 0, 12],
          [-5, 0, 12],
        ],
      }),
    ],
    stagingArea: DEFAULT_ROUTE_STAGING_AREA,
    nowMs,
  });

  assert.equal(routes.length, 1);
  assert.ok(routes[0].polyline.length > 2);
  assert.ok(
    routes[0].polyline[1][0] >= DEFAULT_ROUTE_STAGING_AREA.position[0],
    "first detour waypoint should not move behind the start position"
  );
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
