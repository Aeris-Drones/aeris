import * as THREE from 'three';
import { Vector3, Quaternion, Color } from 'three';
import { VehicleTelemetryMessage, VehicleType } from '../ros/telemetry';
import { geoToLocal, GeoCoordinates } from '../ros/mapTile';

export interface VehicleState {
  id: string;
  type: VehicleType;
  /** Local position in Three.js coordinates: x=East, y=altitude, z=North (negative) */
  position: Vector3;
  /** Raw geographic coordinates for map/UI reference */
  rawPosition: GeoCoordinates;
  /** Velocity vector from telemetry in m/s (ROS coordinate frame) */
  velocity: Vector3;
  orientation: Quaternion;
  /** Heading in radians (yaw), 0 = North, positive clockwise */
  heading: number;
  /** Historical positions for trajectory visualization (last 30 seconds) */
  trajectory: Vector3[];
  /** Unix timestamp (ms) of last telemetry update */
  lastUpdate: number;
  /** Vehicle color based on type (SCOUT=blue, RANGER=orange, UNKNOWN=gray) */
  color: Color;
}

/**
 * Manages vehicle state and trajectory history from telemetry data.
 *
 * Responsibilities:
 * - Coordinate transformation from geographic (lat/lon) to local Three.js coordinates
 * - Quaternion conversion from ROS (roll/pitch/yaw) to Three.js convention
 * - Trajectory buffering with time-based eviction (30 second window)
 * - Stale vehicle cleanup (10 second timeout)
 *
 * Coordinate Systems:
 * - Input (ROS): latitude, longitude, altitude; roll, pitch, yaw
 * - Output (Three.js): x=East, y=up (altitude), z=South (negated from North)
 */
export class VehicleManager {
  private vehicles: Map<string, VehicleState>;
  private trajectoryBuffers: Map<string, { positions: Vector3[]; timestamps: number[] }>;
  private lastTelemetry: Map<string, { msg: VehicleTelemetryMessage; localPos: Vector3; time: number }>;

  /** Maximum trajectory history in milliseconds (30 seconds) */
  private maxTrajectoryTime: number = 30000;

  /** Vehicle timeout threshold in milliseconds (10 seconds) */
  private cleanupThreshold: number = 10000;

  constructor() {
    this.vehicles = new Map();
    this.trajectoryBuffers = new Map();
    this.lastTelemetry = new Map();
  }

  /**
   * Processes a telemetry message and updates vehicle state.
   *
   * Coordinate Transformation:
   * 1. If no origin is set, uses this message's position as the new origin
   * 2. Converts geographic (lat/lon) to local meters using equirectangular approximation
   * 3. Maps to Three.js: x=East, y=altitude, z=North (negated)
   *
   * Quaternion Conversion:
   * ROS uses (roll, pitch, yaw) applied in order. Three.js Euler order is 'YXZ'
   * to match aerospace convention (yaw-pitch-roll). The yaw is negated because
   * ROS uses counter-clockwise positive, while Three.js uses clockwise positive
   * when looking down the Y axis.
   *
   * @param message - Parsed telemetry message from ROS
   * @param origin - Current coordinate origin, or null to auto-set from this message
   * @returns New origin if it was auto-set, null otherwise
   */
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

    // Three.js coordinate mapping: x=East, y=altitude, z=North (negative)
    const position = new Vector3(localPos.x, message.position.altitude, localPos.z);

    // Convert ROS (roll, pitch, yaw) to Three.js Quaternion
    // Euler order 'YXZ': yaw (Y), pitch (X), roll (Z)
    // Negative yaw accounts for ROS/Three.js handedness difference
    const quaternion = new Quaternion();
    quaternion.setFromEuler(new THREE.Euler(message.orientation.pitch, -message.orientation.yaw, message.orientation.roll, 'YXZ'));

    const now = Date.now();
    const vehicleId = message.vehicle_id;

    // Store raw telemetry for potential interpolation use
    this.lastTelemetry.set(vehicleId, {
      msg: message,
      localPos: position.clone(),
      time: now
    });

    // Initialize trajectory buffer if needed
    if (!this.trajectoryBuffers.has(vehicleId)) {
      this.trajectoryBuffers.set(vehicleId, { positions: [], timestamps: [] });
    }
    const buffer = this.trajectoryBuffers.get(vehicleId)!;
    buffer.positions.push(position.clone());
    buffer.timestamps.push(now);

    // Evict old trajectory points outside the time window
    while (buffer.timestamps.length > 0 && now - buffer.timestamps[0] > this.maxTrajectoryTime) {
      buffer.timestamps.shift();
      buffer.positions.shift();
    }

    // Assign vehicle color based on type
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

  /**
   * Returns all active vehicles, filtering out stale entries.
   *
   * Side effect: Removes vehicles that haven't received updates within
   * cleanupThreshold (10 seconds) and their associated trajectory data.
   *
   * @returns Array of current vehicle states
   */
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

  /**
   * Gets the current position and heading for a specific vehicle.
   *
   * Note: This returns the last known position without interpolation.
   * For smoother visualization, temporal interpolation using telemetry
   * timestamps could be implemented here.
   *
   * @param vehicleId - The vehicle identifier
   * @returns Current position and heading, or null if not found
   */
  public getCurrentPosition(vehicleId: string): { position: Vector3, heading: number } | null {
      const vehicle = this.vehicles.get(vehicleId);
      if (!vehicle) return null;

      return { position: vehicle.position, heading: vehicle.heading };
  }
}
