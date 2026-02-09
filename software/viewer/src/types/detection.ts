/**
 * Unified Detection System Types
 * Aggregates thermal, acoustic, and gas sensor detections into a common interface
 */

import type { SensorType, ConfidenceLevel } from '@/lib/design-tokens';

/**
 * Base detection interface - common fields across all sensor types
 */
export interface BaseDetection {
  /** Unique identifier for this detection */
  id: string;
  /** Sensor type that generated this detection */
  sensorType: SensorType;
  /** Detection confidence (0-1) */
  confidence: number;
  /** Timestamp when detection was created */
  timestamp: number;
  /** Last time this detection was updated */
  lastUpdate: number;
  /** Geographic coordinates */
  location: {
    latitude: number;
    longitude: number;
    altitude: number;
  };
  /** Local 3D coordinates (for rendering) */
  localPosition?: {
    x: number;
    y: number;
    z: number;
  };
  /** Operator actions */
  status: DetectionStatus;
  /** Optional operator notes */
  notes?: string;
  /** Which vehicle detected this (if applicable) */
  vehicleId?: string;
}

/**
 * Detection status lifecycle
 */
export type DetectionStatus =
  | 'new'          // Just received, pulsing animation
  | 'reviewing'    // Operator has expanded/viewed
  | 'confirmed'    // Operator confirmed as real survivor
  | 'dismissed'    // Operator marked as false positive
  | 'expired';     // TTL expired, fading out

/**
 * Thermal hotspot detection
 */
export interface ThermalDetection extends BaseDetection {
  sensorType: 'thermal';
  /** Temperature in Celsius */
  temperature: number;
  /** Bounding box in pixels (if available) */
  boundingBox?: {
    x: number;
    y: number;
    width: number;
    height: number;
  };
}

/**
 * Acoustic bearing detection
 */
export interface AcousticDetection extends BaseDetection {
  sensorType: 'acoustic';
  /** Bearing angle in degrees */
  bearing: number;
  /** Signal-to-noise ratio in dB */
  snr: number;
  /** Classification type */
  classification: 'vocal' | 'tapping' | 'mechanical' | 'unknown';
}

/**
 * Gas plume detection
 */
export interface GasDetection extends BaseDetection {
  sensorType: 'gas';
  /** Gas species detected */
  species: string;
  /** Concentration value */
  concentration: number;
  /** Concentration units */
  units: string;
  /** Plume area (if available) */
  area?: number;
  /** Wind direction vector */
  windDirection?: {
    x: number;
    y: number;
    z: number;
  };
  /** Wind speed in m/s */
  windSpeed?: number;
}

/**
 * Union type for all detection types
 */
export type Detection = ThermalDetection | AcousticDetection | GasDetection;

/**
 * Detection filter options
 */
export interface DetectionFilter {
  /** Filter by sensor type */
  sensorTypes?: SensorType[];
  /** Filter by status */
  statuses?: DetectionStatus[];
  /** Minimum confidence threshold (0-1) */
  minConfidence?: number;
  /** Filter by vehicle ID */
  vehicleId?: string;
  /** Time window (only show detections from last N milliseconds) */
  timeWindow?: number;
}

/**
 * Detection sort options
 */
export type DetectionSortBy = 'confidence' | 'time' | 'distance' | 'type';
export type DetectionSortOrder = 'asc' | 'desc';

/**
 * Operator action for detection
 */
export interface DetectionAction {
  detectionId: string;
  action: 'confirm' | 'dismiss' | 'add_note';
  timestamp: number;
  operatorId?: string;
  notes?: string;
}

/**
 * Detection statistics
 */
export interface DetectionStats {
  total: number;
  byType: {
    thermal: number;
    acoustic: number;
    gas: number;
  };
  byStatus: {
    new: number;
    reviewing: number;
    confirmed: number;
    dismissed: number;
    expired: number;
  };
  byConfidence: {
    high: number;
    medium: number;
    low: number;
    unverified: number;
  };
}

/**
 * Helper to determine confidence level
 */
export function getDetectionConfidenceLevel(detection: Detection): ConfidenceLevel {
  const percentage = detection.confidence * 100;
  if (percentage >= 80) return 'high';
  if (percentage >= 50) return 'medium';
  if (percentage >= 20) return 'low';
  return 'unverified';
}

/**
 * Helper to get human-readable sensor name
 */
export function getSensorName(type: SensorType): string {
  switch (type) {
    case 'thermal':
      return 'Thermal';
    case 'acoustic':
      return 'Acoustic';
    case 'gas':
      return 'Gas';
  }
}

/**
 * Helper to get classification display name
 */
export function getClassificationName(classification: AcousticDetection['classification']): string {
  switch (classification) {
    case 'vocal':
      return 'Vocal (Voice)';
    case 'tapping':
      return 'Tapping';
    case 'mechanical':
      return 'Mechanical';
    case 'unknown':
      return 'Unknown';
  }
}

/**
 * Format detection for display
 */
export function formatDetectionSummary(detection: Detection): string {
  switch (detection.sensorType) {
    case 'thermal':
      return `${detection.temperature.toFixed(1)}°C, ${Math.round(detection.confidence * 100)}% conf.`;
    case 'acoustic':
      return `${getClassificationName(detection.classification)}, SNR: ${detection.snr.toFixed(1)}dB`;
    case 'gas':
      return `${detection.species}, ${detection.concentration.toFixed(1)} ${detection.units}`;
  }
}

/**
 * Calculate time since detection
 */
export function getTimeSince(timestamp: number): string {
  const seconds = Math.floor((Date.now() - timestamp) / 1000);

  if (seconds < 60) return `${seconds}s ago`;

  const minutes = Math.floor(seconds / 60);
  if (minutes < 60) return `${minutes}m ago`;

  const hours = Math.floor(minutes / 60);
  if (hours < 24) return `${hours}h ago`;

  const days = Math.floor(hours / 24);
  return `${days}d ago`;
}

/**
 * Calculate distance from a reference point (in meters)
 */
export function getDetectionDistance(
  detection: Detection,
  referencePoint: { x: number; z: number }
): number {
  if (!detection.localPosition) return 0;

  const dx = detection.localPosition.x - referencePoint.x;
  const dz = detection.localPosition.z - referencePoint.z;

  return Math.sqrt(dx * dx + dz * dz);
}

/**
 * Get cardinal direction to detection
 */
export function getDetectionDirection(
  detection: Detection,
  referencePoint: { x: number; z: number }
): string {
  if (!detection.localPosition) return 'Unknown';

  const dx = detection.localPosition.x - referencePoint.x;
  const dz = detection.localPosition.z - referencePoint.z;

  const angle = Math.atan2(dx, -dz) * (180 / Math.PI);
  const normalized = (angle + 360) % 360;

  if (normalized < 22.5 || normalized >= 337.5) return 'N';
  if (normalized < 67.5) return 'NE';
  if (normalized < 112.5) return 'E';
  if (normalized < 157.5) return 'SE';
  if (normalized < 202.5) return 'S';
  if (normalized < 247.5) return 'SW';
  if (normalized < 292.5) return 'W';
  return 'NW';
}
