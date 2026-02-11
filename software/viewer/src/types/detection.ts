import type { SensorType, ConfidenceLevel } from '@/lib/design-tokens';

export interface BaseDetection {
  id: string;
  sensorType: SensorType;
  confidence: number;
  timestamp: number;
  lastUpdate: number;
  location: {
    latitude: number;
    longitude: number;
    altitude: number;
  };
  localPosition?: {
    x: number;
    y: number;
    z: number;
  };
  status: DetectionStatus;
  notes?: string;
  vehicleId?: string;
}

export type DetectionStatus =
  | 'new'
  | 'reviewing'
  | 'confirmed'
  | 'dismissed'
  | 'expired';

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

export interface AcousticDetection extends BaseDetection {
  sensorType: 'acoustic';
  bearing: number;
  snr: number;
  classification: 'vocal' | 'tapping' | 'mechanical' | 'unknown';
}

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

export interface DetectionFilter {
  sensorTypes?: SensorType[];
  statuses?: DetectionStatus[];
  minConfidence?: number;
  vehicleId?: string;
  timeWindow?: number;
}

export type DetectionSortBy = 'confidence' | 'time' | 'distance' | 'type';
export type DetectionSortOrder = 'asc' | 'desc';

export interface DetectionAction {
  detectionId: string;
  action: 'confirm' | 'dismiss' | 'add_note';
  timestamp: number;
  operatorId?: string;
  notes?: string;
}

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

export function getDetectionDistance(
  detection: Detection,
  referencePoint: { x: number; z: number }
): number {
  if (!detection.localPosition) return 0;

  const dx = detection.localPosition.x - referencePoint.x;
  const dz = detection.localPosition.z - referencePoint.z;

  return Math.sqrt(dx * dx + dz * dz);
}

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
