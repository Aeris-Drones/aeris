export enum VehicleType {
  SCOUT = 'scout',
  RANGER = 'ranger',
  UNKNOWN = 'unknown',
}

export interface VehicleTelemetryMessage {
  vehicle_id: string;        // "scout_1", "ranger_1"
  vehicle_type: VehicleType; // SCOUT | RANGER | UNKNOWN
  timestamp: { sec: number; nanosec: number };
  position: {
    latitude: number;
    longitude: number;
    altitude: number;        // meters AGL
  };
  orientation: {
    roll: number;            // radians
    pitch: number;           // radians
    yaw: number;             // radians (heading)
  };
  velocity: {
    x: number;               // m/s
    y: number;
    z: number;
  };
}

/**
 * Converts a raw ROS message (with string vehicle_type) to a typed VehicleTelemetryMessage
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
