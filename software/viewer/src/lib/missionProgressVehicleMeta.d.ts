/**
 * Normalizes a vehicle identifier to a canonical format.
 *
 * Handles variations in ID formatting (case sensitivity, whitespace, prefixes)
 * to ensure consistent vehicle lookup across ROS messages and UI components.
 *
 * @param value - Raw vehicle identifier from external sources
 * @returns Normalized vehicle ID suitable for internal use
 */
export function normalizeVehicleId(value: string): string;

/**
 * Parses mission progress payload to extract per-vehicle metadata.
 *
 * The mission controller broadcasts progress updates as a compressed payload
 * string. This function decodes the payload and maps fields to vehicle IDs
 * for display in mission tracking views.
 *
 * Used to populate the mission progress panel with vehicle assignments,
 * completion percentages, and online status without querying individual
 * vehicle telemetry.
 *
 * @param rawData - Compressed progress payload from mission controller
 * @returns Mapped vehicle metadata keyed by normalized vehicle ID
 */
export function extractVehicleMissionMetaFromProgressPayload(
  rawData: string
): {
  assignments: Record<string, string>;
  assignmentLabels: Record<string, string>;
  progress: Record<string, number>;
  online: Record<string, boolean>;
  slamModes: Record<string, string>;
};
