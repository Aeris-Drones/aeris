export const VEHICLE_COMMAND_SERVICE_TIMEOUT_MS: number;
export const VEHICLE_COMMAND_SERVICE_TYPE: string;
export const VEHICLE_COMMAND_SERVICE_NAME: string;

export function buildVehicleCommandServiceRequest(args: {
  vehicleId: string;
  command: string;
  missionId?: string;
}): {
  vehicle_id: string;
  command: string;
  mission_id: string;
};

export function normalizeVehicleCommandServiceResponse(response: unknown): {
  success: boolean;
  message: string;
};

export function getVehicleCommandValidationError(args: {
  rosConnected: boolean;
  vehicleId: string | undefined;
}): string | null;

export function formatVehicleCommandFailure(
  command: string,
  vehicleId: string,
  message: string
): string;

export function callVehicleCommandService(args: {
  ros: unknown;
  serviceName?: string;
  serviceType?: string;
  request: unknown;
  timeoutMs?: number;
  createService: (args: {
    ros: unknown;
    name: string;
    serviceType: string;
  }) => {
    callService: (
      request: unknown,
      onSuccess: (response: unknown) => void,
      onError: (error: unknown) => void
    ) => void;
  };
  createServiceRequest: (request: unknown) => unknown;
}): Promise<{
  success: boolean;
  message: string;
}>;
