import type { VehicleInfo, VehicleStatus } from "@/types/vehicle";

export type VehicleMissionMeta = {
  commandStatusHint?: VehicleStatus;
  assignment?: string;
  assignmentLabel?: string;
  progress?: number;
  online?: boolean;
};

export function applyVehicleMissionMeta(
  vehicleInfo: VehicleInfo,
  meta?: VehicleMissionMeta
): VehicleInfo;
