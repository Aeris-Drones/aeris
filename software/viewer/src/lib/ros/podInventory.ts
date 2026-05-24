import type {
  CalibrationDueState,
  MaintenancePodInventorySnapshot,
} from '../maintenanceDiagnostics';

interface TimeLike {
  sec?: unknown;
  nanosec?: unknown;
}

export interface PodInventoryArrayMessage {
  observedAtMs: number | null;
  records: MaintenancePodInventorySnapshot[];
}

export interface LogPodCalibrationResponse {
  accepted: boolean;
  message: string;
  failureCode: string;
  record: MaintenancePodInventorySnapshot | null;
}

const CALIBRATION_STATE_MAP: Record<number, CalibrationDueState> = {
  0: 'unknown',
  1: 'current',
  2: 'due_soon',
  3: 'overdue',
};

function parseTime(value: unknown): number | null {
  if (!value || typeof value !== 'object') {
    return null;
  }
  const time = value as TimeLike;
  const sec = Number(time.sec);
  const nanosec = Number(time.nanosec);
  if (!Number.isFinite(sec) || !Number.isFinite(nanosec) || (sec === 0 && nanosec === 0)) {
    return null;
  }
  return sec * 1000 + nanosec / 1_000_000;
}

function parseString(value: unknown): string {
  return typeof value === 'string' ? value.trim() : '';
}

function parseBoolean(value: unknown): boolean {
  if (typeof value === 'boolean') {
    return value;
  }
  if (typeof value === 'string') {
    return value.trim().toLowerCase() === 'true';
  }
  return Boolean(value);
}

function parseCapabilities(value: unknown): string[] {
  if (!Array.isArray(value)) {
    return [];
  }
  return value
    .filter((item): item is string => typeof item === 'string')
    .map((item) => item.trim())
    .filter(Boolean);
}

function parseCalibrationState(value: unknown): CalibrationDueState {
  if (typeof value === 'number' && CALIBRATION_STATE_MAP[value]) {
    return CALIBRATION_STATE_MAP[value];
  }
  if (typeof value === 'string') {
    const normalized = value.trim().toLowerCase();
    if (
      normalized === 'unknown' ||
      normalized === 'current' ||
      normalized === 'due_soon' ||
      normalized === 'overdue'
    ) {
      return normalized;
    }
  }
  return 'unknown';
}

function parseRecord(raw: unknown): MaintenancePodInventorySnapshot | null {
  const record = (raw ?? {}) as Record<string, unknown>;
  const podSerial = parseString(record.pod_serial ?? record.podSerial);
  if (!podSerial) {
    return null;
  }
  return {
    podSerial,
    podType: parseString(record.pod_type ?? record.podType),
    attached: parseBoolean(record.attached),
    vehicleId: parseString(record.vehicle_id ?? record.vehicleId) || undefined,
    slotId: parseString(record.slot_id ?? record.slotId) || undefined,
    oneWireId: parseString(record.one_wire_id ?? record.oneWireId) || undefined,
    lifecycleStateLabel:
      parseString(record.lifecycle_state_label ?? record.lifecycleStateLabel) || undefined,
    connected: parseBoolean(record.connected),
    powerReady: parseBoolean(record.power_ready ?? record.powerReady),
    linkReady: parseBoolean(record.link_ready ?? record.linkReady),
    capabilities: parseCapabilities(record.capabilities),
    lastCalibrationAtMs: parseTime(
      record.last_calibration ?? record.lastCalibration
    ),
    nextCalibrationDueAtMs: parseTime(
      record.next_calibration_due ?? record.nextCalibrationDue
    ),
    calibrationState: parseCalibrationState(
      record.calibration_state ?? record.calibrationState
    ),
    calibrationDetail:
      parseString(record.calibration_detail ?? record.calibrationDetail) || undefined,
  };
}

export function parsePodInventoryArray(raw: unknown): PodInventoryArrayMessage {
  if (!raw || typeof raw !== 'object') {
    throw new Error('Invalid pod inventory data: expected an object');
  }
  const message = raw as {
    observed_at?: unknown;
    observedAt?: unknown;
    records?: unknown;
    pods?: unknown;
  };
  const records = Array.isArray(message.records)
    ? message.records
    : Array.isArray(message.pods)
      ? message.pods
      : [];

  return {
    observedAtMs: parseTime(message.observed_at ?? message.observedAt),
    records: records
      .map((entry) => parseRecord(entry))
      .filter((entry): entry is MaintenancePodInventorySnapshot => entry != null),
  };
}

export function parseLogPodCalibrationResponse(raw: unknown): LogPodCalibrationResponse {
  if (!raw || typeof raw !== 'object') {
    throw new Error('Invalid pod calibration response: expected an object');
  }
  const response = raw as {
    accepted?: unknown;
    message?: unknown;
    failure_code?: unknown;
    failureCode?: unknown;
    record?: unknown;
  };
  return {
    accepted: parseBoolean(response.accepted),
    message: parseString(response.message),
    failureCode: parseString(response.failure_code ?? response.failureCode),
    record: parseRecord(response.record),
  };
}

export function getCalibrationBadgeVariant(
  state: CalibrationDueState
): 'outline' | 'success' | 'warning' | 'danger' {
  if (state === 'current') {
    return 'success';
  }
  if (state === 'due_soon') {
    return 'warning';
  }
  if (state === 'overdue') {
    return 'danger';
  }
  return 'outline';
}

export function formatInventoryTimestamp(value?: number | null): string {
  if (value == null) {
    return 'Not recorded';
  }
  return new Intl.DateTimeFormat('en-US', {
    year: 'numeric',
    month: 'short',
    day: 'numeric',
    timeZone: 'UTC',
  }).format(new Date(value));
}

export function createDemoPodInventoryArray(): PodInventoryArrayMessage {
  return {
    observedAtMs: 10_000,
    records: [
      {
        podSerial: 'THM-204',
        podType: 'thermal',
        attached: true,
        vehicleId: 'scout_1',
        slotId: 'front-left',
        lastCalibrationAtMs: 1_710_000_000_000,
        nextCalibrationDueAtMs: 1_760_000_000_000,
        calibrationState: 'current',
        calibrationDetail: 'Current until 2025-10-18',
      },
      {
        podSerial: 'GAS-118',
        podType: 'hazmat',
        attached: true,
        vehicleId: 'scout_2',
        slotId: 'belly',
        lastCalibrationAtMs: 1_710_000_000_000,
        nextCalibrationDueAtMs: 1_710_950_000_000,
        calibrationState: 'due_soon',
        calibrationDetail: 'Due in 11 days',
      },
      {
        podSerial: 'LDR-551',
        podType: 'lidar',
        attached: false,
        lastCalibrationAtMs: 1_700_000_000_000,
        nextCalibrationDueAtMs: 1_700_100_000_000,
        calibrationState: 'overdue',
        calibrationDetail: 'Overdue by 4 days',
      },
    ],
  };
}
