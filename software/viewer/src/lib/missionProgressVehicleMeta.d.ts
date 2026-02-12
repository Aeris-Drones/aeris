export function normalizeVehicleId(value: string): string;

export function extractVehicleMissionMetaFromProgressPayload(
  rawData: string
): {
  assignments: Record<string, string>;
  assignmentLabels: Record<string, string>;
  progress: Record<string, number>;
  online: Record<string, boolean>;
  slamModes: Record<string, string>;
};
