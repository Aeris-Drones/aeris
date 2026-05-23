import type { MaintenancePodSnapshot } from '../maintenanceDiagnostics';

interface TimeLike {
  sec?: unknown;
  nanosec?: unknown;
}

export interface PodStatusArrayMessage {
  observedAtMs: number | null;
  pods: MaintenancePodSnapshot[];
}

function parseTime(value: unknown): number | null {
  if (!value || typeof value !== 'object') {
    return null;
  }
  const time = value as TimeLike;
  const sec = Number(time.sec);
  const nanosec = Number(time.nanosec);
  if (!Number.isFinite(sec) || !Number.isFinite(nanosec)) {
    return null;
  }
  return sec * 1000 + nanosec / 1_000_000;
}

function parseBoolean(value: unknown): boolean {
  return value === true;
}

function parseString(value: unknown): string {
  return typeof value === 'string' ? value.trim() : '';
}

function parseCapabilities(value: unknown): string[] {
  if (!Array.isArray(value)) {
    return [];
  }
  return value.filter((item): item is string => typeof item === 'string').map((item) => item.trim()).filter(Boolean);
}

export function parsePodStatusArray(raw: unknown): PodStatusArrayMessage {
  if (!raw || typeof raw !== 'object') {
    throw new Error('Invalid pod status data: expected an object');
  }
  const message = raw as { observed_at?: unknown; observedAt?: unknown; pods?: unknown };
  const pods = Array.isArray(message.pods) ? message.pods : [];

  return {
    observedAtMs: parseTime(message.observed_at ?? message.observedAt),
    pods: pods.map((entry) => {
      const pod = (entry ?? {}) as Record<string, unknown>;
      return {
        vehicleId: parseString(pod.vehicle_id ?? pod.vehicleId),
        slotId: parseString(pod.slot_id ?? pod.slotId),
        podSerial: parseString(pod.pod_serial ?? pod.podSerial),
        podType: parseString(pod.pod_type ?? pod.podType),
        lifecycleState: parseString(pod.lifecycle_state_label ?? pod.lifecycleStateLabel ?? pod.lifecycle_state ?? pod.lifecycleState) || 'unknown',
        connected: parseBoolean(pod.connected),
        powerReady: parseBoolean(pod.power_ready ?? pod.powerReady),
        linkReady: parseBoolean(pod.link_ready ?? pod.linkReady),
        capabilities: parseCapabilities(pod.capabilities),
        faultDetail: parseString(pod.fault_detail ?? pod.faultDetail) || undefined,
        rejectionDetail: parseString(pod.rejection_detail ?? pod.rejectionDetail) || undefined,
      };
    }).filter((pod) => pod.vehicleId.length > 0),
  };
}
