import type { Detection } from "@/components/sheets/DetectionCard";

export type RoutePoint = [number, number, number];

export interface RouteStagingArea {
  id: string;
  label: string;
  position: RoutePoint;
}

export interface RouteFreshness {
  source: "live" | "replayed" | "stale" | "unknown";
  ageMs: number | null;
}

export interface RouteTarget {
  id: string;
  label: string;
  position: RoutePoint;
  detection: Detection;
}

export interface RouteBlocker {
  id: string;
  type: "gas" | "structural";
  label: string;
  geometry: RoutePoint[];
  bounds: {
    minX: number;
    maxX: number;
    minZ: number;
    maxZ: number;
  };
  freshness: RouteFreshness;
}

export interface RouteRecommendation {
  id: string;
  sourceLabel: string;
  destinationLabel: string;
  targetDetectionId: string;
  polyline: RoutePoint[];
  blockingHazardIds: string[];
  freshness: RouteFreshness;
  status: "clear" | "stale" | "pending";
}

export const DEFAULT_ROUTE_STAGING_AREA: RouteStagingArea;

export function selectSurvivorRouteTargets(detections: Detection[]): RouteTarget[];

export function extractRouteBlockers(detections: Detection[]): RouteBlocker[];

export function deriveRouteRecommendations(input?: {
  detections?: Detection[];
  stagingArea?: RouteStagingArea;
  nowMs?: number;
}): RouteRecommendation[];
