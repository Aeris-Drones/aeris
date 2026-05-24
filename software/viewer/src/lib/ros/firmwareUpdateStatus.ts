export type FirmwareUpdateLifecycleState =
  | 'unknown'
  | 'idle'
  | 'downloading'
  | 'validating'
  | 'applying'
  | 'verifying'
  | 'complete'
  | 'failed'
  | 'rolling_back'
  | 'rolled_back';

export interface FirmwareUpdateStatusSnapshot {
  vehicleId: string;
  packageId?: string;
  currentVersion?: string;
  targetVersion?: string;
  lifecycleState: FirmwareUpdateLifecycleState;
  lifecycleLabel: string;
  progressPercent: number;
  activeSlot?: string;
  inactiveSlot?: string;
  rollbackPerformed: boolean;
  statusDetail?: string;
  errorCode?: string;
  errorDetail?: string;
}

export interface FirmwareUpdateStatusArrayMessage {
  observedAtMs: number | null;
  updates: FirmwareUpdateStatusSnapshot[];
}

export interface FirmwareUpdateCommandInput {
  vehicleId: string;
  packageId: string;
  targetVersion: string;
  packageUri: string;
  packageSignature: string;
}

interface TimeLike {
  sec?: unknown;
  nanosec?: unknown;
}

const LIFECYCLE_MAP: Record<number, FirmwareUpdateLifecycleState> = {
  0: 'unknown',
  1: 'idle',
  2: 'downloading',
  3: 'validating',
  4: 'applying',
  5: 'verifying',
  6: 'complete',
  7: 'failed',
  8: 'rolling_back',
  9: 'rolled_back',
};

function parseLifecycleState(value: unknown): FirmwareUpdateLifecycleState {
  if (typeof value === 'number' && LIFECYCLE_MAP[value]) {
    return LIFECYCLE_MAP[value];
  }
  if (typeof value === 'string') {
    const normalized = value.trim().toLowerCase();
    if (
      normalized === 'unknown' ||
      normalized === 'idle' ||
      normalized === 'downloading' ||
      normalized === 'validating' ||
      normalized === 'applying' ||
      normalized === 'verifying' ||
      normalized === 'complete' ||
      normalized === 'failed' ||
      normalized === 'rolling_back' ||
      normalized === 'rolled_back'
    ) {
      return normalized;
    }
  }
  return 'unknown';
}

function parseString(value: unknown): string | undefined {
  if (typeof value !== 'string') {
    return undefined;
  }
  const trimmed = value.trim();
  return trimmed.length > 0 ? trimmed : undefined;
}

function parseNumber(value: unknown): number {
  const numeric = Number(value);
  if (!Number.isFinite(numeric)) {
    return 0;
  }
  return Math.min(100, Math.max(0, numeric));
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

function titleCaseLifecycle(input: FirmwareUpdateLifecycleState): string {
  return input
    .split(/[_\s-]+/)
    .filter(Boolean)
    .map((part) => part.charAt(0).toUpperCase() + part.slice(1))
    .join(' ');
}

export function isFirmwareUpdateActive(
  status?: Pick<FirmwareUpdateStatusSnapshot, 'lifecycleState'> | null
): boolean {
  if (!status) {
    return false;
  }
  return (
    status.lifecycleState === 'downloading' ||
    status.lifecycleState === 'validating' ||
    status.lifecycleState === 'applying' ||
    status.lifecycleState === 'verifying' ||
    status.lifecycleState === 'rolling_back'
  );
}

export function hasFreshFirmwareUpdateStatus(
  status?: Pick<FirmwareUpdateStatusSnapshot, 'lifecycleState'> | null
): boolean {
  if (!status) {
    return false;
  }
  return (
    isFirmwareUpdateActive(status) ||
    status.lifecycleState === 'complete' ||
    status.lifecycleState === 'failed' ||
    status.lifecycleState === 'rolling_back' ||
    status.lifecycleState === 'rolled_back'
  );
}

export function getFirmwareUpdateBadgeVariant(
  state: FirmwareUpdateLifecycleState
): 'outline' | 'info' | 'success' | 'warning' | 'danger' {
  if (state === 'complete') {
    return 'success';
  }
  if (state === 'rolled_back') {
    return 'warning';
  }
  if (state === 'failed') {
    return 'danger';
  }
  if (state === 'unknown' || state === 'idle') {
    return 'outline';
  }
  return 'info';
}

export function createDemoFirmwareUpdateStatusArray(): FirmwareUpdateStatusArrayMessage {
  return {
    observedAtMs: 10_000,
    updates: [
      {
        vehicleId: 'scout_1',
        packageId: 'fw-2026.05.20',
        currentVersion: '2026.05.20',
        targetVersion: '2026.05.20',
        lifecycleState: 'complete',
        lifecycleLabel: 'Complete',
        progressPercent: 100,
        activeSlot: 'B',
        inactiveSlot: 'A',
        rollbackPerformed: false,
        statusDetail: 'Vehicle healthy on slot B',
      },
      {
        vehicleId: 'scout_2',
        packageId: 'fw-2026.05.23',
        currentVersion: '2026.04.9',
        targetVersion: '2026.05.23',
        lifecycleState: 'applying',
        lifecycleLabel: 'Applying',
        progressPercent: 63.5,
        activeSlot: 'A',
        inactiveSlot: 'B',
        rollbackPerformed: false,
        statusDetail: 'Writing inactive partition',
      },
      {
        vehicleId: 'ranger_1',
        packageId: 'fw-2026.05.18',
        currentVersion: '2026.04.1',
        targetVersion: '2026.05.18',
        lifecycleState: 'rolled_back',
        lifecycleLabel: 'Rolled back',
        progressPercent: 100,
        activeSlot: 'A',
        inactiveSlot: 'B',
        rollbackPerformed: true,
        statusDetail: 'Vehicle returned to slot A',
        errorCode: 'healthcheck_failed',
        errorDetail: 'Post-update healthcheck failed',
      },
    ],
  };
}

export function parseFirmwareUpdateStatusArray(
  raw: unknown
): FirmwareUpdateStatusArrayMessage {
  if (!raw || typeof raw !== 'object') {
    throw new Error('Invalid firmware update data: expected an object');
  }

  const message = raw as {
    observed_at?: unknown;
    observedAt?: unknown;
    updates?: unknown;
    statuses?: unknown;
  };
  const updates = Array.isArray(message.updates)
    ? message.updates
    : Array.isArray(message.statuses)
      ? message.statuses
      : [];

  return {
    observedAtMs: parseTime(message.observed_at ?? message.observedAt),
    updates: updates
      .map((entry) => {
        const update = (entry ?? {}) as Record<string, unknown>;
        const lifecycleState = parseLifecycleState(
          update.lifecycle_state ?? update.lifecycleState
        );
        return {
          vehicleId: String(update.vehicle_id ?? update.vehicleId ?? '').trim(),
          packageId: parseString(update.package_id ?? update.packageId),
          currentVersion: parseString(update.current_version ?? update.currentVersion),
          targetVersion: parseString(update.target_version ?? update.targetVersion),
          lifecycleState,
          lifecycleLabel:
            parseString(update.lifecycle_state_label ?? update.lifecycleStateLabel) ??
            titleCaseLifecycle(lifecycleState),
          progressPercent: parseNumber(
            update.progress_percent ?? update.progressPercent
          ),
          activeSlot: parseString(update.active_slot ?? update.activeSlot),
          inactiveSlot: parseString(update.inactive_slot ?? update.inactiveSlot),
          rollbackPerformed: parseBoolean(
            update.rollback_performed ?? update.rollbackPerformed
          ),
          statusDetail: parseString(update.status_detail ?? update.statusDetail),
          errorCode: parseString(update.error_code ?? update.errorCode),
          errorDetail: parseString(update.error_detail ?? update.errorDetail),
        } satisfies FirmwareUpdateStatusSnapshot;
      })
      .filter((update) => update.vehicleId.length > 0),
  };
}
