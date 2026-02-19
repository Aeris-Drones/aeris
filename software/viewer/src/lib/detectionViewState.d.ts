import type { Detection } from '@/components/sheets/DetectionCard';

export interface DetectionCounts {
  thermal: number;
  acoustic: number;
  gas: number;
  pending: number;
  confirmed: number;
}

export function getConfidenceTextClass(detection: Detection): string;
export function computeDetectionCounts(detections: Detection[]): DetectionCounts;
export function applyDetectionStatusOverrides(
  detections: Detection[],
  overrides: Record<string, Detection['status']>
): Detection[];
