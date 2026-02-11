/**
 * Timeout for vehicle command service calls in milliseconds.
 * Commands that exceed this duration will be considered failed.
 */
export const VEHICLE_COMMAND_SERVICE_TIMEOUT_MS: number;

/**
 * ROS service type for vehicle commands.
 * Maps to the aeris_msgs/VehicleCommand service definition.
 */
export const VEHICLE_COMMAND_SERVICE_TYPE: string;

/**
 * Default ROS service name for vehicle commands.
 * Published by the mission control node.
 */
export const VEHICLE_COMMAND_SERVICE_NAME: string;

/**
 * Builds a ROS service request payload for vehicle commands.
 *
 * Used to send operational commands (start, pause, resume, abort, etc.)
 * to individual vehicles through the ROS service layer.
 *
 * @param args - Command parameters
 * @returns ROS-compatible request object with snake_case keys
 */
export function buildVehicleCommandServiceRequest(args: {
  vehicleId: string;
  command: string;
  missionId?: string;
}): {
  vehicle_id: string;
  command: string;
  mission_id: string;
};

/**
 * Normalizes the ROS service response into a standard format.
 *
 * Handles variations in response structure from different ROS service versions
 * and ensures consistent success/error handling at the application layer.
 *
 * @param response - Raw ROS service response
 * @returns Normalized response with success flag and message
 */
export function normalizeVehicleCommandServiceResponse(response: unknown): {
  success: boolean;
  message: string;
};

/**
 * Validates preconditions for sending a vehicle command.
 *
 * Checks ROS connection state and vehicle identification before
 * attempting service calls to prevent unnecessary network traffic.
 *
 * @param args - Validation context
 * @returns Error message if validation fails, null if ready to send
 */
export function getVehicleCommandValidationError(args: {
  rosConnected: boolean;
  vehicleId: string | undefined;
}): string | null;

/**
 * Formats a user-facing error message for command failures.
 *
 * Provides consistent error messaging across the fleet command UI,
 * including the command type and vehicle ID for debugging.
 *
 * @param command - The command that failed
 * @param vehicleId - Target vehicle identifier
 * @param message - Error message from the service
 * @returns Formatted error string for display
 */
export function formatVehicleCommandFailure(
  command: string,
  vehicleId: string,
  message: string
): string;

/**
 * Executes a vehicle command through the ROS service layer.
 *
 * Wraps the ROS service call with timeout handling and response normalization.
 * This is the primary integration point between the UI command layer and ROS.
 *
 * @param args - Service call configuration including ROS instance and request
 * @returns Promise resolving to success status and message
 */
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
