import type { VehicleInfo, VehicleStatus } from "@/types/vehicle";

export type VehicleMissionMeta = {
  commandStatusHint?: VehicleStatus;
  assignment?: string;
  assignmentLabel?: string;
  progress?: number;
  online?: boolean;
  slamMode?: string;
};

export function applyVehicleMissionMeta(
  vehicleInfo: VehicleInfo,
  meta?: VehicleMissionMeta
): VehicleInfo;
