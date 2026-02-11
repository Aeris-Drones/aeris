import type { SensorType, ConfidenceLevel } from '@/lib/design-tokens';

/**
 * Base detection interface shared across all sensor types.
 *
 * Location uses GPS coordinates (WGS84). localPosition is optional
 * ENU coordinates relative to search origin for local reference.
 *
 * Status lifecycle:
 *   new -> reviewing -> confirmed|dismissed
 *                    -> expired (auto-timeout)
 */
export interface BaseDetection {
  id: string;
  sensorType: SensorType;

  /**
   * Confidence score from 0.0 to 1.0.
   *
   * Interpretation varies by sensor:
   * - thermal: classification confidence
   * - acoustic: signal-to-noise ratio normalized
   * - gas: concentration above baseline
   */
  confidence: number;

  timestamp: number;
  lastUpdate: number;

  location: {
    latitude: number;
    longitude: number;
    altitude: number;
  };

  /** Local ENU coordinates [east, up, north] relative to search origin */
  localPosition?: {
    x: number;
    y: number;
    z: number;
  };

  status: DetectionStatus;
  notes?: string;
  vehicleId?: string;
}

/**
 * Detection review workflow states.
 *
 * - new: Unreviewed, requires operator attention
 * - reviewing: Currently being evaluated
 * - confirmed: Validated as genuine
 * - dismissed: False positive or irrelevant
 * - expired: Auto-closed due to age without review
 */
export type DetectionStatus =
  | 'new'
  | 'reviewing'
  | 'confirmed'
  | 'dismissed'
  | 'expired';

/**
 * Thermal camera detection with temperature reading.
 *
 * boundingBox defines the region of interest in image coordinates
 * (pixels from top-left origin).
 */
export interface ThermalDetection extends BaseDetection {
  sensorType: 'thermal';
  temperature: number;
  boundingBox?: {
    x: number;
    y: number;
    width: number;
    height: number;
  };
}

/**
 * Acoustic sensor detection with bearing and classification.
 *
 * - bearing: Direction to sound source in degrees (0-360, magnetic)
 * - snr: Signal-to-noise ratio in dB
 * - classification: Acoustic signature category
 */
export interface AcousticDetection extends BaseDetection {
  sensorType: 'acoustic';
  bearing: number;
  snr: number;
  classification: 'vocal' | 'tapping' | 'mechanical' | 'unknown';
}

/**
 * Gas sensor detection with concentration and environmental context.
 *
 * - species: Chemical compound detected (e.g., "CO", "CH4")
 * - concentration: Measured value in specified units
 * - wind data: Optional for plume modeling and source localization
 */
export interface GasDetection extends BaseDetection {
  sensorType: 'gas';
  species: string;
  concentration: number;
  units: string;
  area?: number;
  windDirection?: {
    x: number;
    y: number;
    z: number;
  };
  windSpeed?: number;
}

export type Detection = ThermalDetection | AcousticDetection | GasDetection;

/**
 * Filter criteria for detection queries.
 *
 * timeWindow: maximum age in milliseconds (from now)
 */
export interface DetectionFilter {
  sensorTypes?: SensorType[];
  statuses?: DetectionStatus[];
  minConfidence?: number;
  vehicleId?: string;
  timeWindow?: number;
}

export type DetectionSortBy = 'confidence' | 'time' | 'distance' | 'type';
export type DetectionSortOrder = 'asc' | 'desc';

/**
 * Operator action recorded for audit trail.
 */
export interface DetectionAction {
  detectionId: string;
  action: 'confirm' | 'dismiss' | 'add_note';
  timestamp: number;
  operatorId?: string;
  notes?: string;
}

/**
 * Aggregated detection statistics for dashboard display.
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
 * Map confidence score to discrete confidence level.
 *
 * Thresholds:
 * - high: >=80%
 * - medium: 50-79%
 * - low: 20-49%
 * - unverified: <20%
 */
export function getDetectionConfidenceLevel(detection: Detection): ConfidenceLevel {
  const percentage = detection.confidence * 100;
  if (percentage >= 80) return 'high';
  if (percentage >= 50) return 'medium';
  if (percentage >= 20) return 'low';
  return 'unverified';
}

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
 * Get human-readable classification label for acoustic detections.
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
 * Format detection data as a concise summary string.
 *
 * Returns sensor-specific formatted output:
 * - thermal: temperature and confidence
 * - acoustic: classification and SNR
 * - gas: species and concentration
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
 * Format timestamp as relative time string.
 *
 * Returns "Xs ago", "Xm ago", "Xh ago", or "Xd ago" based on elapsed time.
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
 * Calculate 2D horizontal distance from reference point to detection.
 *
 * Uses localPosition if available, otherwise returns 0.
 * Ignores Y (altitude) for ground distance calculation.
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
 * Calculate cardinal direction from reference point to detection.
 *
 * Returns one of: N, NE, E, SE, S, SW, W, NW
 * Returns 'Unknown' if localPosition is unavailable.
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
