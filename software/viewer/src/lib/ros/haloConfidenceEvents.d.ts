export interface HaloRecognitionMetadata {
  baseline_name: string;
  baseline_version: string;
  latency_ms?: number;
  source_timestamp_ns?: number;
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
  timestampNs: number;
  sourceUri?: string;
  label: string;
  detectionType: string;
  region?: HaloEventRegion;
  locationHint?: string | HaloEventLocationHint;
  evidenceRef?: string;
  evidenceUri?: string;
  recognition: HaloRecognitionMetadata;
}

export function normalizeHaloConfidenceEventMessage(
  rawMessage: unknown
): NormalizedHaloConfidenceDetection;
