/**
 * AERIS GCS Vehicle Types
 * 
 * Extended vehicle types for fleet management and control.
 * Builds on the existing VehicleState from VehicleManager.
 */

import type { VehicleState } from '@/lib/vehicle/VehicleManager';
import { VehicleType } from '@/lib/ros/telemetry';

// ============================================================================
// Vehicle Status & Commands
// ============================================================================

/**
 * Operational status of a vehicle
 */
export type VehicleStatus = 
  | 'active'      // Normal operation
  | 'returning'   // RTL in progress
  | 'holding'     // Position hold
  | 'offline'     // Lost connection
  | 'landed'      // On ground
  | 'emergency';  // Emergency state

/**
 * Commands that can be sent to vehicles
 */
export type VehicleCommand = 
  | 'WAYPOINT'    // Go to specific coordinates
  | 'HOLD'        // Hold current position
  | 'RESUME'      // Resume autonomous mission
  | 'RECALL'      // Return to launch
  | 'LAND';       // Land immediately

/**
 * Extended vehicle info for UI display
 */
export interface VehicleInfo extends VehicleState {
  status: VehicleStatus;
  batteryPercent: number;
  signalStrength: number;  // 0-100
  speed: number;           // m/s
  altitude: number;        // meters AGL
  distanceFromHome: number; // meters
  isSelected: boolean;
  assignment?: string;
  missionProgressPercent?: number;
  slamMode?: string;
}

/**
 * Command request sent to ROS
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

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Get display name for vehicle type
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
 * Get vehicle type description
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
 * Get color for vehicle type
 */
export function getVehicleTypeColor(type: VehicleType): string {
  switch (type) {
    case VehicleType.SCOUT:
      return 'var(--info)';  // Blue
    case VehicleType.RANGER:
      return '#F97316';      // Orange
    default:
      return 'var(--muted-foreground)';
  }
}

/**
 * Get status display config
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
 * Get battery level color
 */
export function getBatteryColor(percent: number): string {
  if (percent > 50) return 'text-success';
  if (percent > 20) return 'text-warning';
  return 'text-danger';
}

/**
 * Get signal strength color
 */
export function getSignalColor(strength: number): string {
  if (strength > 70) return 'text-success';
  if (strength > 40) return 'text-warning';
  return 'text-danger';
}

/**
 * Format altitude for display
 */
export function formatAltitude(meters: number): string {
  if (meters < 1) return '0m';
  return `${meters.toFixed(0)}m`;
}

/**
 * Format speed for display
 */
export function formatSpeed(mps: number): string {
  if (mps < 0.1) return '0 m/s';
  return `${mps.toFixed(1)} m/s`;
}

/**
 * Format distance for display
 */
export function formatDistance(meters: number): string {
  if (meters < 1000) {
    return `${meters.toFixed(0)}m`;
  }
  return `${(meters / 1000).toFixed(1)}km`;
}

/**
 * Calculate speed from velocity vector
 */
export function calculateSpeed(velocity: { x: number; y: number; z: number }): number {
  return Math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2);
}

/**
 * Convert VehicleState to VehicleInfo with simulated data
 * In production, this would come from actual telemetry
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
