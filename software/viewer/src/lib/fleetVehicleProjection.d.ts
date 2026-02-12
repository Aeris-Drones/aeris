import type { VehicleInfo, VehicleStatus } from "@/types/vehicle";

/**
 * Metadata overlay for vehicles participating in active missions.
 *
 * This type represents mission-specific state that augments base vehicle
 * information, sourced from mission progress updates rather than direct
 * vehicle telemetry. Used to display mission context in fleet views.
 */
export type VehicleMissionMeta = {
  /** Suggested status indicator based on mission command state */
  commandStatusHint?: VehicleStatus;
  /** Mission assignment identifier (e.g., "search", "track") */
  assignment?: string;
  /** Human-readable label for the assignment */
  assignmentLabel?: string;
  /** Mission completion percentage (0-100) */
  progress?: number;
  /** Whether the vehicle is currently communicating with the mission controller */
  online?: boolean;
  slamMode?: string;
};

/**
 * Merges mission metadata into a vehicle's base information.
 *
 * Used by fleet overview components to enrich vehicle displays with
 * mission context. The original VehicleInfo is preserved; metadata
 * fields are applied as overlays without mutation.
 *
 * @param vehicleInfo - Base vehicle information from telemetry
 * @param meta - Mission-specific metadata from progress updates
 * @returns VehicleInfo with mission context applied
 */
export function applyVehicleMissionMeta(
  vehicleInfo: VehicleInfo,
  meta?: VehicleMissionMeta
): VehicleInfo;
