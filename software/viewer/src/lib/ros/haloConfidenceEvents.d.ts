export interface HaloRecognitionMetadata {
  baseline_name: string;
  baseline_version: string;
  latency_ms?: number;
  source_timestamp_ns?: string;
  confidence_components?: Record<string, number>;
}

export interface HaloEventRegion {
  x: number;
  y: number;
  width: number;
  height: number;
}

export interface HaloEventLocationHint {
  label?: string;
  x?: number;
  y?: number;
  z?: number;
}

export interface NormalizedHaloConfidenceDetection {
  id: string;
  sensorType: "rgb";
  confidence: number;
  confidenceLevel: "HIGH" | "MEDIUM" | "LOW" | "UNKNOWN";
  /** Unix timestamp in milliseconds for viewer ordering and Date APIs. */
  timestamp: number;
  status: "new";
  vehicleId: string;
  vehicleName: string;
  position: [number, number, number];
  sourceModalities: ["rgb"];
  sector: string;
  signatureType: string;
  frameId: string;
  frameIndex: number;
  /** Raw nanosecond timestamp preserved as text to avoid JavaScript precision loss. */
  timestampNs: string;
  sourceUri?: string;
  label: string;
  detectionType: string;
  region?: HaloEventRegion;
  locationHint?: string | HaloEventLocationHint;
  evidenceRef?: string;
  evidenceUri?: string;
  evidencePath?: string;
  deliveryMode?: "live" | "replayed";
  originalEventTs?: number;
  replayedAtTs?: number;
  isRetroactive?: boolean;
  recognition: HaloRecognitionMetadata;
}

export function normalizeHaloConfidenceEventMessage(
  rawMessage: unknown,
  options?: { nowMs?: number }
): NormalizedHaloConfidenceDetection;

export function normalizeHaloEvidenceLogRecord(
  rawRecord: unknown,
  options?: { nowMs?: number }
): NormalizedHaloConfidenceDetection;

export function normalizeHaloEvidenceReplayPayload(
  rawPayload: unknown,
  options?: { nowMs?: number }
): NormalizedHaloConfidenceDetection[];
