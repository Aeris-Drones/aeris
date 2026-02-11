/**
 * ROS Telemetry Message Parsing
 *
 * This module handles validation and type conversion of raw ROS messages
 * into strongly-typed TypeScript interfaces. All numeric fields are validated
 * to ensure they are finite numbers before casting.
 */

export enum VehicleType {
  SCOUT = 'scout',
  RANGER = 'ranger',
  UNKNOWN = 'unknown',
}

/**
 * Standardized vehicle telemetry message structure.
 *
 * Mirrors the ROS aeris_msgs/Telemetry message format with TypeScript types.
 * All angles are in radians, distances in meters, velocities in m/s.
 */
export interface VehicleTelemetryMessage {
  /** Vehicle identifier, e.g., "scout_1", "ranger_1" */
  vehicle_id: string;
  /** Vehicle classification for UI theming */
  vehicle_type: VehicleType;
  /** ROS timestamp (seconds + nanoseconds since epoch) */
  timestamp: { sec: number; nanosec: number };
  /** Geographic position (WGS84) */
  position: {
    latitude: number;
    longitude: number;
    /** Altitude in meters above ground level (AGL) */
    altitude: number;
  };
  /** Orientation in radians (ROS convention: roll-pitch-yaw, extrinsic rotations) */
  orientation: {
    roll: number;
    pitch: number;
    /** Yaw in radians, 0 = North, positive clockwise */
    yaw: number;
  };
  /** Linear velocity in ROS body frame (m/s) */
  velocity: {
    x: number;
    y: number;
    z: number;
  };
}

/**
 * Validates and converts a raw ROS message to a typed VehicleTelemetryMessage.
 *
 * Validation Rules:
 * - vehicle_id: non-empty string
 * - timestamp: sec and nanosec must be finite numbers
 * - position: latitude, longitude, altitude must be finite numbers
 * - orientation: roll, pitch, yaw must be finite numbers
 * - velocity: x, y, z must be finite numbers
 * - vehicle_type: case-insensitive match to 'scout' or 'ranger', defaults to UNKNOWN
 *
 * @param raw - The raw ROS message object from roslib
 * @returns Validated VehicleTelemetryMessage
 * @throws Error with descriptive message if any validation fails
 */
export function parseVehicleTelemetry(raw: unknown): VehicleTelemetryMessage {
  if (!raw || typeof raw !== 'object') {
    throw new Error('Invalid telemetry data: raw input is null or undefined');
  }
  const data = raw as Record<string, unknown>;

  const isNumber = (value: unknown): value is number =>
    typeof value === 'number' && Number.isFinite(value);

  if (!data.vehicle_id || typeof data.vehicle_id !== 'string') {
    throw new Error('Invalid telemetry data: vehicle_id must be a non-empty string');
  }

  const timestamp = data.timestamp as Record<string, unknown> | undefined;
  if (
    !timestamp ||
    !isNumber(timestamp.sec) ||
    !isNumber(timestamp.nanosec)
  ) {
    throw new Error('Invalid telemetry data: timestamp.sec and timestamp.nanosec must be numbers');
  }

  const position = data.position as Record<string, unknown> | undefined;
  if (
    !position ||
    !isNumber(position.latitude) ||
    !isNumber(position.longitude) ||
    !isNumber(position.altitude)
  ) {
    throw new Error('Invalid telemetry data: position must include numeric latitude, longitude, and altitude');
  }

  const orientation = data.orientation as Record<string, unknown> | undefined;
  if (
    !orientation ||
    !isNumber(orientation.roll) ||
    !isNumber(orientation.pitch) ||
    !isNumber(orientation.yaw)
  ) {
    throw new Error('Invalid telemetry data: orientation must include numeric roll, pitch, and yaw');
  }

  const velocity = data.velocity as Record<string, unknown> | undefined;
  if (
    !velocity ||
    !isNumber(velocity.x) ||
    !isNumber(velocity.y) ||
    !isNumber(velocity.z)
  ) {
    throw new Error('Invalid telemetry data: velocity must include numeric x, y, and z');
  }

  // Normalize vehicle type string to enum (case-insensitive)
  const vType = typeof data.vehicle_type === 'string'
    ? data.vehicle_type.trim().toLowerCase()
    : '';

  let vehicleType: VehicleType = VehicleType.UNKNOWN;
  if (vType === 'scout') {
    vehicleType = VehicleType.SCOUT;
  } else if (vType === 'ranger') {
    vehicleType = VehicleType.RANGER;
  }

  return {
    vehicle_id: data.vehicle_id as string,
    vehicle_type: vehicleType,
    timestamp: {
      sec: timestamp.sec as number,
      nanosec: timestamp.nanosec as number,
    },
    position: {
      latitude: position.latitude as number,
      longitude: position.longitude as number,
      altitude: position.altitude as number,
    },
    orientation: {
      roll: orientation.roll as number,
      pitch: orientation.pitch as number,
      yaw: orientation.yaw as number,
    },
    velocity: {
      x: velocity.x as number,
      y: velocity.y as number,
      z: velocity.z as number,
    },
  };
}
