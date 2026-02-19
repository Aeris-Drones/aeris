import type { Detection } from '@/components/sheets/DetectionCard';

export interface NormalizeFusedDetectionOptions {
  nowMs?: number;
  maxAgeMs?: number;
  maxFutureSkewMs?: number;
  fallbackVehicleId?: string;
  fallbackVehicleName?: string;
}

export function normalizeFusedDetectionMessage(
  rawMessage: unknown,
  options?: NormalizeFusedDetectionOptions
): Detection | null;
