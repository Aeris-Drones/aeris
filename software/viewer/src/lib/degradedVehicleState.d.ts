export const DEFAULT_TELEMETRY_STALE_TIMEOUT_MS: number;
export const DEFAULT_LAST_KNOWN_RETENTION_MS: number;

export interface MissionMetaMaps {
  assignments?: Record<string, string>;
  assignmentLabels?: Record<string, string>;
  progress?: Record<string, number>;
  online?: Record<string, boolean>;
  slamModes?: Record<string, string>;
}

export interface VehicleMissionMeta {
  assignment?: string;
  assignmentLabel?: string;
  progress?: number;
  online?: boolean;
  slamMode?: string;
}

export interface DegradedVehicleState {
  status: "active" | "warning" | "offline";
  offline: boolean;
  retained: boolean;
  replayed: boolean;
  lastContactAgeMs: number | null;
  staleSinceMs: number | null;
}

export function normalizeMissionMetaForVehicle(
  vehicleId: string,
  missionMeta?: MissionMetaMaps
): VehicleMissionMeta;

export function deriveVehicleDegradedState(input?: {
  lastUpdate?: number;
  missionOnline?: boolean;
  deliveryMode?: "live" | "replayed";
  isRetroactive?: boolean;
  now?: number;
  staleTimeoutMs?: number;
  retentionMs?: number;
}): DegradedVehicleState;

export function formatLastContactAge(ageMs: number | null | undefined): string;
