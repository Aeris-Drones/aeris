import { withServiceTimeout } from "./missionControlBehavior.js";

export const VEHICLE_COMMAND_SERVICE_TIMEOUT_MS = 8000;
export const VEHICLE_COMMAND_SERVICE_TYPE = "aeris_msgs/srv/VehicleCommand";
export const VEHICLE_COMMAND_SERVICE_NAME = "vehicle_command";

export function buildVehicleCommandServiceRequest({
  vehicleId,
  command,
  missionId = "",
}) {
  return {
    vehicle_id: vehicleId,
    command,
    mission_id: missionId,
  };
}

export function normalizeVehicleCommandServiceResponse(response) {
  return {
    success: Boolean(response?.success),
    message: typeof response?.message === "string" ? response.message : "",
  };
}

export function getVehicleCommandValidationError({ rosConnected, vehicleId }) {
  if (!rosConnected) {
    return "ROS is disconnected. Reconnect before sending vehicle commands.";
  }

  if (!vehicleId || !vehicleId.trim()) {
    return "Vehicle command failed: vehicle id is missing.";
  }

  return null;
}

export function formatVehicleCommandFailure(command, vehicleId, message) {
  const details = message && message.trim()
    ? message.trim()
    : "No rejection reason was returned by the orchestrator.";
  return `${command} rejected for ${vehicleId}: ${details}`;
}

export function callVehicleCommandService({
  ros,
  serviceName = VEHICLE_COMMAND_SERVICE_NAME,
  serviceType = VEHICLE_COMMAND_SERVICE_TYPE,
  request,
  timeoutMs = VEHICLE_COMMAND_SERVICE_TIMEOUT_MS,
  createService,
  createServiceRequest,
}) {
  return withServiceTimeout(
    (resolve, reject) => {
      const service = createService({
        ros,
        name: serviceName,
        serviceType,
      });
      const serviceRequest = createServiceRequest(request);
      service.callService(
        serviceRequest,
        response => resolve(normalizeVehicleCommandServiceResponse(response)),
        error => reject(new Error(String(error)))
      );
    },
    timeoutMs,
    serviceName
  );
}
