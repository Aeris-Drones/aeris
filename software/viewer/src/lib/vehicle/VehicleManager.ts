import * as THREE from 'three';
import { Vector3, Quaternion, Color } from 'three';
import { VehicleTelemetryMessage, VehicleType } from '../ros/telemetry';
import { geoToLocal, GeoCoordinates } from '../ros/mapTile';

export interface VehicleState {
  id: string;
  type: VehicleType;
  position: Vector3; // Local position (Three.js coordinates)
  rawPosition: GeoCoordinates; // Keep raw lat/lon for map/UI usage
  velocity: Vector3; // Velocity vector from telemetry (m/s)
  orientation: Quaternion;
  heading: number; // yaw in radians
  trajectory: Vector3[]; // History of positions
  lastUpdate: number; // Timestamp of last update
  color: Color;
}

export class VehicleManager {
  private vehicles: Map<string, VehicleState>;
  private trajectoryBuffers: Map<string, { positions: Vector3[]; timestamps: number[] }>;
  private lastTelemetry: Map<string, { msg: VehicleTelemetryMessage; localPos: Vector3; time: number }>;
  private maxTrajectoryTime: number = 30000; // 30 seconds
  private cleanupThreshold: number = 10000; // 10 seconds (fade out time)

  constructor() {
    this.vehicles = new Map();
    this.trajectoryBuffers = new Map();
    this.lastTelemetry = new Map();
  }

  public processTelemetry(message: VehicleTelemetryMessage, origin: GeoCoordinates | null): GeoCoordinates | null {
    let newOrigin: GeoCoordinates | null = null;
    
    let effectiveOrigin = origin;
    if (!effectiveOrigin) {
        effectiveOrigin = { lat: message.position.latitude, lon: message.position.longitude };
        newOrigin = effectiveOrigin;
    }

    const localPos = geoToLocal(
      { lat: message.position.latitude, lon: message.position.longitude },
      effectiveOrigin
    );

    // Convert to Three.js: x=East, y=altitude, z=North (negative)
    const position = new Vector3(localPos.x, message.position.altitude, localPos.z);

    // Convert ROS (Roll, Pitch, Yaw) to Three.js Quaternion
    const quaternion = new Quaternion();
    quaternion.setFromEuler(new THREE.Euler(message.orientation.pitch, -message.orientation.yaw, message.orientation.roll, 'YXZ'));

    const now = Date.now();
    const vehicleId = message.vehicle_id;

    this.lastTelemetry.set(vehicleId, {
      msg: message,
      localPos: position.clone(),
      time: now
    });

    if (!this.trajectoryBuffers.has(vehicleId)) {
      this.trajectoryBuffers.set(vehicleId, { positions: [], timestamps: [] });
    }
    const buffer = this.trajectoryBuffers.get(vehicleId)!;
    buffer.positions.push(position.clone());
    buffer.timestamps.push(now);

    while (buffer.timestamps.length > 0 && now - buffer.timestamps[0] > this.maxTrajectoryTime) {
      buffer.timestamps.shift();
      buffer.positions.shift();
    }

    let color = new Color('#9CA3AF');
    const typeEnum = message.vehicle_type;
    if (typeEnum === VehicleType.SCOUT) {
        color = new Color('#3B82F6');
    } else if (typeEnum === VehicleType.RANGER) {
        color = new Color('#F97316');
    }

    this.vehicles.set(vehicleId, {
      id: vehicleId,
      type: typeEnum,
      position: position,
      rawPosition: { lat: message.position.latitude, lon: message.position.longitude },
      velocity: new Vector3(message.velocity.x, message.velocity.y, message.velocity.z),
      orientation: quaternion,
      heading: message.orientation.yaw,
      trajectory: [...buffer.positions],
      lastUpdate: now,
      color: color
    });

    return newOrigin;
  }

  public getVehicles(): VehicleState[] {
    const now = Date.now();
    for (const [id, vehicle] of this.vehicles.entries()) {
        if (now - vehicle.lastUpdate > this.cleanupThreshold) {
            this.vehicles.delete(id);
            this.trajectoryBuffers.delete(id);
            this.lastTelemetry.delete(id);
        }
    }
    return Array.from(this.vehicles.values());
  }

  // TODO: implement temporal interpolation using telemetry timestamps if needed
  public getCurrentPosition(vehicleId: string): { position: Vector3, heading: number } | null {
      const vehicle = this.vehicles.get(vehicleId);
      if (!vehicle) return null;

      return { position: vehicle.position, heading: vehicle.heading };
  }
}
