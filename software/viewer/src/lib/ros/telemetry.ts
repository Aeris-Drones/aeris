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
export function parseVehicleTelemetry(raw: any): VehicleTelemetryMessage {
  if (!raw) {
    throw new Error('Invalid telemetry data: raw input is null or undefined');
  }

  const isNumber = (value: unknown): value is number =>
    typeof value === 'number' && Number.isFinite(value);

  if (!raw.vehicle_id || typeof raw.vehicle_id !== 'string') {
    throw new Error('Invalid telemetry data: vehicle_id must be a non-empty string');
  }

  if (
    !raw.timestamp ||
    !isNumber(raw.timestamp.sec) ||
    !isNumber(raw.timestamp.nanosec)
  ) {
    throw new Error('Invalid telemetry data: timestamp.sec and timestamp.nanosec must be numbers');
  }

  if (
    !raw.position ||
    !isNumber(raw.position.latitude) ||
    !isNumber(raw.position.longitude) ||
    !isNumber(raw.position.altitude)
  ) {
    throw new Error('Invalid telemetry data: position must include numeric latitude, longitude, and altitude');
  }

  if (
    !raw.orientation ||
    !isNumber(raw.orientation.roll) ||
    !isNumber(raw.orientation.pitch) ||
    !isNumber(raw.orientation.yaw)
  ) {
    throw new Error('Invalid telemetry data: orientation must include numeric roll, pitch, and yaw');
  }

  if (
    !raw.velocity ||
    !isNumber(raw.velocity.x) ||
    !isNumber(raw.velocity.y) ||
    !isNumber(raw.velocity.z)
  ) {
    throw new Error('Invalid telemetry data: velocity must include numeric x, y, and z');
  }

  const vType = typeof raw.vehicle_type === 'string' 
    ? raw.vehicle_type.trim().toLowerCase() 
    : '';
  
  let vehicleType: VehicleType = VehicleType.UNKNOWN;
  if (vType === 'scout') {
    vehicleType = VehicleType.SCOUT;
  } else if (vType === 'ranger') {
    vehicleType = VehicleType.RANGER;
  }
  
  return {
    vehicle_id: raw.vehicle_id,
    vehicle_type: vehicleType,
    timestamp: {
      sec: raw.timestamp.sec,
      nanosec: raw.timestamp.nanosec,
    },
    position: {
      latitude: raw.position.latitude,
      longitude: raw.position.longitude,
      altitude: raw.position.altitude,
    },
    orientation: {
      roll: raw.orientation.roll,
      pitch: raw.orientation.pitch,
      yaw: raw.orientation.yaw,
    },
    velocity: {
      x: raw.velocity.x,
      y: raw.velocity.y,
      z: raw.velocity.z,
    },
  };
}
