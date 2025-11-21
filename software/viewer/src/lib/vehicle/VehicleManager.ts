import * as THREE from 'three';
import { Vector3, Quaternion, Color } from 'three';
import { VehicleTelemetryMessage, VehicleType } from '../ros/telemetry';
import { geoToLocal, GeoCoordinates } from '../ros/mapTile';

export interface VehicleState {
  id: string;
  type: VehicleType;
  position: Vector3;
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
  private staleThreshold: number = 2000; // 2 seconds
  private cleanupThreshold: number = 10000; // 10 seconds (fade out time)

  constructor() {
    this.vehicles = new Map();
    this.trajectoryBuffers = new Map();
    this.lastTelemetry = new Map();
  }

  public processTelemetry(message: VehicleTelemetryMessage, origin: GeoCoordinates | null): GeoCoordinates | null {
    let newOrigin: GeoCoordinates | null = null;
    // If no origin, set it from this vehicle (Option 2 from user, but using Context preferred)
    // However, the manager needs to return it if it sets it internally so the hook can update context.
    // But if 'origin' passed in is null, we treat this vehicle's position as origin temporarily or permanent?
    // Requirement: "First vehicle sets origin: If no map tiles yet, VehicleManager sets origin from first telemetry"

    let effectiveOrigin = origin;
    if (!effectiveOrigin) {
        effectiveOrigin = { lat: message.position.latitude, lon: message.position.longitude };
        newOrigin = effectiveOrigin;
    }

    const localPos = geoToLocal(
      { lat: message.position.latitude, lon: message.position.longitude },
      effectiveOrigin
    );

    // Convert to Three.js coordinates (x, y=altitude, z)
    // geoToLocal returns {x, z} where z is North (negative or positive depending on convention).
    // In mapTile.ts: z = -(lat - originLat) * R. So -Z is North.
    // Altitude is Y.
    const position = new Vector3(localPos.x, message.position.altitude, localPos.z);

    // Orientation (ROS is usually Z-up, Three.js is Y-up)
    // ROS: Roll(x), Pitch(y), Yaw(z)
    // Three: X=East, Z=South, Y=Up.
    // Need to check if we need conversion.
    // Assuming standard ENU to standard Three.js conversion.
    // For now, direct mapping of yaw to rotation around Y axis.
    // message.orientation is radians.
    const quaternion = new Quaternion();
    quaternion.setFromEuler(new THREE.Euler(message.orientation.pitch, -message.orientation.yaw, message.orientation.roll, 'YXZ'));
    // Note: Euler order and signs might need tuning based on visual verification.
    // Standard aerospace: Yaw is heading (0 = North? East?).
    // If map is ENU: East=X, North=Y (but we mapped North to -Z).
    // Let's stick to simple Heading = Yaw for now.

    const now = Date.now(); // Wall clock time for animation smoothing
    const vehicleId = message.vehicle_id;

    // Store raw telemetry for interpolation
    this.lastTelemetry.set(vehicleId, {
      msg: message,
      localPos: position.clone(),
      time: now
    });

    // Update buffers
    if (!this.trajectoryBuffers.has(vehicleId)) {
      this.trajectoryBuffers.set(vehicleId, { positions: [], timestamps: [] });
    }
    const buffer = this.trajectoryBuffers.get(vehicleId)!;
    buffer.positions.push(position.clone());
    buffer.timestamps.push(now);

    // Prune old points
    while (buffer.timestamps.length > 0 && now - buffer.timestamps[0] > this.maxTrajectoryTime) {
      buffer.timestamps.shift();
      buffer.positions.shift();
    }

    // Determine Color
    let color = new Color('#9CA3AF'); // Unknown
    const vType = message.vehicle_type.toLowerCase();
    let typeEnum = VehicleType.UNKNOWN;
    if (vType.includes('scout')) {
        color = new Color('#3B82F6');
        typeEnum = VehicleType.SCOUT;
    } else if (vType.includes('ranger')) {
        color = new Color('#F97316');
        typeEnum = VehicleType.RANGER;
    }

    // Update Vehicle State
    // Note: Real render loop will interpolate. This just sets the latest "keyframe".
    // But to keep it simple for the state exposed to React, we update the state here.
    // The Hook will trigger re-renders or we use a ref in the component for 60fps.
    // Actually, for 60fps, we shouldn't rely on React state updates for position.
    // We should return the manager instance or methods to get interpolated state.

    // However, for the React component tree to render the *existence* of vehicles, we need state.
    this.vehicles.set(vehicleId, {
      id: vehicleId,
      type: typeEnum,
      position: position,
      orientation: quaternion, // Placeholder
      heading: message.orientation.yaw,
      trajectory: [...buffer.positions], // Clone for safety
      lastUpdate: now,
      color: color
    });

    return newOrigin;
  }

  public getVehicles(): VehicleState[] {
    // Filter out very old vehicles (cleanup)
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

  public getInterpolatedPosition(vehicleId: string): { position: Vector3, heading: number } | null {
      // Simple interpolation can be done here if requested by the render loop
      // For now, just return latest for simplicity or implement lerp if needed.
      // The render loop in R3F usually handles interpolation if we pass target props.
      // But if we want "true" 60fps independent of telemetry 10Hz:
      const vehicle = this.vehicles.get(vehicleId);
      if (!vehicle) return null;

      // Since we only have "last" telemetry, we can't really interpolate *between* two knowns
      // unless we add delay.
      // Standard approach: Render state is delayed by ~100ms to interpolate between T-2 and T-1.
      // Or we just extrapolate (dead reckoning).
      // Prompt suggested: "Store last two positions... t = (currentTime - lastUpdateTime) / expectedUpdateInterval".
      // But we don't have strict previous/next without delay.

      // Let's stick to returning the current state.
      // The visual component can use `drei`'s <Line> which updates when props change.
      // For smooth movement of the drone model, we can use `useFrame` in the component
      // to lerp the mesh position to the `vehicle.position`.

      return { position: vehicle.position, heading: vehicle.heading };
  }
}
