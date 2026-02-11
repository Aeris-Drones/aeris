import type { VehicleState } from '@/lib/vehicle/VehicleManager';
import { VehicleType } from '@/lib/ros/telemetry';

/**
 * Operational status of a vehicle in the field.
 *
 * - active: Normal operation, executing mission
 * - returning: Inbound to home/launch point
 * - holding: Stationary at current position
 * - offline: No telemetry received recently
 * - landed: On ground, powered off or idle
 * - emergency: Critical issue requiring immediate attention
 */
export type VehicleStatus =
  | 'active'
  | 'returning'
  | 'holding'
  | 'offline'
  | 'landed'
  | 'emergency';

/**
 * High-level commands that can be issued to vehicles.
 *
 * Published to ROS topic /vehicle/{id}/command for execution
 * by the vehicle's onboard flight controller.
 */
export type VehicleCommand =
  | 'WAYPOINT'
  | 'HOLD'
  | 'RESUME'
  | 'RECALL'
  | 'LAND';

/**
 * Enriched vehicle information for UI display.
 *
 * Extends VehicleState with computed fields derived from telemetry
 * freshness, battery status, and mission context.
 */
export interface VehicleInfo extends VehicleState {
  status: VehicleStatus;
  batteryPercent: number;
  signalStrength: number;
  speed: number;
  altitude: number;
  distanceFromHome: number;
  isSelected: boolean;
  assignment?: string;
  missionProgressPercent?: number;
}

/**
 * Command payload for vehicle control requests.
 *
 * Published to ROS and logged for audit trail. Params are optional
 * and command-specific (e.g., WAYPOINT requires lat/lon/alt).
 */
export interface VehicleCommandRequest {
  vehicleId: string;
  command: VehicleCommand;
  timestamp: number;
  params?: {
    latitude?: number;
    longitude?: number;
    altitude?: number;
  };
}

/**
 * Returns display name for a vehicle type.
 */
export function getVehicleTypeName(type: VehicleType): string {
  switch (type) {
    case VehicleType.SCOUT:
      return 'Scout';
    case VehicleType.RANGER:
      return 'Ranger';
    default:
      return 'Unknown';
  }
}

/**
 * Returns human-readable description of vehicle capabilities.
 */
export function getVehicleTypeDescription(type: VehicleType): string {
  switch (type) {
    case VehicleType.SCOUT:
      return 'Fast reconnaissance drone';
    case VehicleType.RANGER:
      return 'Heavy sensor platform';
    default:
      return 'Unknown vehicle type';
  }
}

/**
 * Returns CSS color value for a vehicle type.
 *
 * Used for consistent visual identification across the UI.
 */
export function getVehicleTypeColor(type: VehicleType): string {
  switch (type) {
    case VehicleType.SCOUT:
      return 'var(--info)';
    case VehicleType.RANGER:
      return '#F97316';
    default:
      return 'var(--muted-foreground)';
  }
}

/**
 * Returns UI styling configuration for a vehicle status.
 *
 * Provides consistent colors, labels, and icon identifiers
 * for status badges and indicators across components.
 */
export function getVehicleStatusConfig(status: VehicleStatus): {
  label: string;
  color: string;
  bgColor: string;
  icon: 'active' | 'returning' | 'holding' | 'offline' | 'landed' | 'emergency';
} {
  switch (status) {
    case 'active':
      return {
        label: 'Active',
        color: 'text-success',
        bgColor: 'bg-success/10',
        icon: 'active',
      };
    case 'returning':
      return {
        label: 'Returning',
        color: 'text-warning',
        bgColor: 'bg-warning/10',
        icon: 'returning',
      };
    case 'holding':
      return {
        label: 'Holding',
        color: 'text-info',
        bgColor: 'bg-info/10',
        icon: 'holding',
      };
    case 'offline':
      return {
        label: 'Offline',
        color: 'text-danger',
        bgColor: 'bg-danger/10',
        icon: 'offline',
      };
    case 'landed':
      return {
        label: 'Landed',
        color: 'text-muted-foreground',
        bgColor: 'bg-surface-3',
        icon: 'landed',
      };
    case 'emergency':
      return {
        label: 'Emergency',
        color: 'text-danger',
        bgColor: 'bg-danger/20',
        icon: 'emergency',
      };
  }
}

/**
 * Returns CSS class for battery level indicator.
 *
 * Thresholds: >50% success, >20% warning, <=20% danger.
 */
export function getBatteryColor(percent: number): string {
  if (percent > 50) return 'text-success';
  if (percent > 20) return 'text-warning';
  return 'text-danger';
}

/**
 * Returns CSS class for signal strength indicator.
 *
 * Thresholds: >70% success, >40% warning, <=40% danger.
 */
export function getSignalColor(strength: number): string {
  if (strength > 70) return 'text-success';
  if (strength > 40) return 'text-warning';
  return 'text-danger';
}

/**
 * Formats altitude in meters for display.
 *
 * Values under 1m are shown as "0m" to avoid fractional noise.
 */
export function formatAltitude(meters: number): string {
  if (meters < 1) return '0m';
  return `${meters.toFixed(0)}m`;
}

/**
 * Formats speed in meters per second for display.
 *
 * Values under 0.1 m/s are shown as "0 m/s" to filter noise.
 */
export function formatSpeed(mps: number): string {
  if (mps < 0.1) return '0 m/s';
  return `${mps.toFixed(1)} m/s`;
}

/**
 * Formats distance with automatic unit selection.
 *
 * Uses meters for <1km, kilometers with 1 decimal for >=1km.
 */
export function formatDistance(meters: number): string {
  if (meters < 1000) {
    return `${meters.toFixed(0)}m`;
  }
  return `${(meters / 1000).toFixed(1)}km`;
}

/**
 * Calculates scalar speed from 3D velocity vector.
 *
 * Uses Euclidean norm (magnitude) of the velocity components.
 */
export function calculateSpeed(velocity: { x: number; y: number; z: number }): number {
  return Math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2);
}

/**
 * Converts raw VehicleState to enriched VehicleInfo for UI consumption.
 *
 * Derives computed fields:
 * - status: based on telemetry age (offline if >5s stale)
 * - signalStrength: decays linearly over 10s window
 * - speed: calculated from velocity vector
 * - altitude: extracted from position.y
 * - distanceFromHome: horizontal distance from origin
 *
 * @param state - Raw vehicle state from VehicleManager
 * @param isSelected - Whether this vehicle is currently selected in UI
 * @returns Enriched vehicle info with computed display fields
 */
export function vehicleStateToInfo(
  state: VehicleState,
  isSelected: boolean = false
): VehicleInfo {
  const now = Date.now();
  const telemetryAgeMs = Math.max(0, now - state.lastUpdate);
  const offlineThresholdMs = 5000;
  const signalDecayWindowMs = 10000;

  const speed = calculateSpeed({
    x: state.velocity.x,
    y: state.velocity.y,
    z: state.velocity.z,
  });
  const freshness = Math.max(0, 1 - telemetryAgeMs / signalDecayWindowMs);
  const signalStrength = Math.round(freshness * 100);
  const status: VehicleStatus =
    telemetryAgeMs > offlineThresholdMs ? 'offline' : 'active';
  const batteryPercent = 100;

  return {
    ...state,
    status,
    batteryPercent,
    signalStrength,
    speed,
    altitude: state.position.y,
    distanceFromHome: Math.sqrt(state.position.x ** 2 + state.position.z ** 2),
    isSelected,
  };
}
