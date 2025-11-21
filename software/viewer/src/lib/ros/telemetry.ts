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
  const vType = typeof raw.vehicle_type === 'string' 
    ? raw.vehicle_type.toLowerCase() 
    : '';
  
  let vehicleType: VehicleType = VehicleType.UNKNOWN;
  if (vType.includes('scout')) {
    vehicleType = VehicleType.SCOUT;
  } else if (vType.includes('ranger')) {
    vehicleType = VehicleType.RANGER;
  }
  
  return {
    ...raw,
    vehicle_type: vehicleType,
  };
}
