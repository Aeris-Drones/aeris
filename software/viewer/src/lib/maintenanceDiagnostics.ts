import { deriveVehicleDegradedState } from './degradedVehicleState.js';

export type DiagnosticState = 'healthy' | 'warning' | 'blocked' | 'unknown';
export type FleetReadinessState = 'ready' | 'warning' | 'blocked';

export interface MaintenanceTelemetrySnapshot {
  id: string;
  name: string;
  batteryPercent: number | null;
  altitudeMeters: number;
  linkQualityPercent?: number;
  lastUpdate?: number;
  deliveryMode?: 'live' | 'replayed';
  isRetroactive?: boolean;
  missionOnline?: boolean;
}

export interface MaintenancePodSnapshot {
  vehicleId: string;
  slotId: string;
  podSerial: string;
  podType: string;
  lifecycleState: string;
  connected: boolean;
  powerReady: boolean;
  linkReady: boolean;
  capabilities: string[];
  faultDetail?: string;
  rejectionDetail?: string;
}

export interface VehicleMaintenanceSnapshot {
  vehicleId: string;
  motorHealthState: DiagnosticState;
  motorHealthDetail?: string;
  calibrationState: DiagnosticState;
  calibrationDetail?: string;
  imuState: DiagnosticState;
  imuDetail?: string;
  compassState: DiagnosticState;
  compassDetail?: string;
  accelerometerState: DiagnosticState;
  accelerometerDetail?: string;
  observedAtMs?: number | null;
}

export interface MaintenanceCheckViewModel {
  label: string;
  state: DiagnosticState;
  summary: string;
  detail?: string;
}

export interface MaintenancePodViewModel {
  slotId: string;
  podSerial: string;
  podType: string;
  lifecycleLabel: string;
  state: DiagnosticState;
  capabilities: string[];
}

export interface MaintenanceVehicleViewModel {
  id: string;
  name: string;
  readiness: FleetReadinessState;
  readinessSummary: string;
  batteryPercent: number | null;
  altitudeMeters: number;
  lastContactAgeMs: number | null;
  isOffline: boolean;
  summaryChecks: {
    motor: MaintenanceCheckViewModel;
    sensors: MaintenanceCheckViewModel;
    mesh: MaintenanceCheckViewModel;
    calibration: MaintenanceCheckViewModel;
  };
  detailChecks: MaintenanceCheckViewModel[];
  pods: MaintenancePodViewModel[];
}

export interface FleetDiagnosticsViewModel {
  vehicles: MaintenanceVehicleViewModel[];
  summary: {
    readyCount: number;
    warningCount: number;
    blockedCount: number;
    connectedCount: number;
    averageLinkQuality: number | null;
  };
}

const DIAGNOSTIC_RANK: Record<DiagnosticState, number> = {
  healthy: 0,
  unknown: 1,
  warning: 2,
  blocked: 3,
};

function titleCaseLifecycle(input: string): string {
  return input
    .split(/[_\s-]+/)
    .filter(Boolean)
    .map((part) => part.charAt(0).toUpperCase() + part.slice(1).toLowerCase())
    .join(' ');
}

function summarizeDiagnosticState(state: DiagnosticState, fallback: string, detail?: string): string {
  if (detail && detail.trim().length > 0) {
    return detail.trim();
  }
  return fallback;
}

function summarizeLinkQuality(linkQualityPercent?: number): MaintenanceCheckViewModel {
  if (typeof linkQualityPercent !== 'number' || !Number.isFinite(linkQualityPercent)) {
    return {
      label: 'Mesh radio',
      state: 'unknown',
      summary: 'Link quality unavailable',
    };
  }
  const rounded = Math.round(linkQualityPercent);
  if (rounded >= 70) {
    return { label: 'Mesh radio', state: 'healthy', summary: `${rounded}% stable mesh link` };
  }
  if (rounded >= 40) {
    return { label: 'Mesh radio', state: 'warning', summary: `${rounded}% degraded mesh link` };
  }
  return { label: 'Mesh radio', state: 'blocked', summary: `${rounded}% unreliable mesh link` };
}

function classifyPodState(pod: MaintenancePodSnapshot): { state: DiagnosticState; lifecycleLabel: string } {
  const lifecycle = String(pod.lifecycleState ?? 'unknown').trim().toLowerCase();
  if (lifecycle === 'faulted' || lifecycle === 'rejected') {
    return { state: 'blocked', lifecycleLabel: titleCaseLifecycle(lifecycle) };
  }
  if (
    lifecycle === 'disconnected' ||
    lifecycle === 'validating' ||
    lifecycle === 'power_check' ||
    lifecycle === 'soft_start' ||
    lifecycle === 'enumerating'
  ) {
    return { state: 'warning', lifecycleLabel: titleCaseLifecycle(lifecycle) };
  }
  if (lifecycle === 'registered' && pod.connected && pod.powerReady && pod.linkReady) {
    return { state: 'healthy', lifecycleLabel: 'Registered' };
  }
  if (lifecycle === 'detected') {
    return { state: 'warning', lifecycleLabel: 'Detected' };
  }
  return { state: 'unknown', lifecycleLabel: titleCaseLifecycle(lifecycle || 'unknown') || 'Unknown' };
}

function summarizePods(pods: MaintenancePodSnapshot[]): {
  summary: MaintenanceCheckViewModel;
  detail: MaintenancePodViewModel[];
} {
  if (pods.length === 0) {
    return {
      summary: {
        label: 'Sensors',
        state: 'unknown',
        summary: 'No pod diagnostics reported',
      },
      detail: [],
    };
  }

  const detail = pods.map((pod) => {
    const classified = classifyPodState(pod);
    return {
      slotId: pod.slotId,
      podSerial: pod.podSerial,
      podType: pod.podType,
      lifecycleLabel: classified.lifecycleLabel,
      state: classified.state,
      capabilities: pod.capabilities,
    };
  });

  const worstState = detail.reduce<DiagnosticState>(
    (current, pod) => (DIAGNOSTIC_RANK[pod.state] > DIAGNOSTIC_RANK[current] ? pod.state : current),
    'healthy'
  );

  const healthyCount = detail.filter((pod) => pod.state === 'healthy').length;
  const degradedCount = detail.length - healthyCount;

  return {
    summary: {
      label: 'Sensors',
      state: worstState,
      summary:
        degradedCount === 0
          ? `${healthyCount}/${detail.length} pods ready`
          : `${degradedCount} pod${degradedCount === 1 ? '' : 's'} need attention`,
    },
    detail,
  };
}

function summarizeCalibration(diagnostics?: VehicleMaintenanceSnapshot): {
  summary: MaintenanceCheckViewModel;
  details: MaintenanceCheckViewModel[];
} {
  const detailChecks: MaintenanceCheckViewModel[] = [
    {
      label: 'IMU calibration',
      state: diagnostics?.imuState ?? 'unknown',
      summary: summarizeDiagnosticState(diagnostics?.imuState ?? 'unknown', 'IMU calibration unavailable', diagnostics?.imuDetail),
      detail: diagnostics?.imuDetail,
    },
    {
      label: 'Compass calibration',
      state: diagnostics?.compassState ?? 'unknown',
      summary: summarizeDiagnosticState(diagnostics?.compassState ?? 'unknown', 'Compass calibration unavailable', diagnostics?.compassDetail),
      detail: diagnostics?.compassDetail,
    },
    {
      label: 'Accelerometer',
      state: diagnostics?.accelerometerState ?? 'unknown',
      summary: summarizeDiagnosticState(
        diagnostics?.accelerometerState ?? 'unknown',
        'Accelerometer calibration unavailable',
        diagnostics?.accelerometerDetail
      ),
      detail: diagnostics?.accelerometerDetail,
    },
  ];

  const derivedWorstDetail = detailChecks.reduce<DiagnosticState>(
    (current, detail) => (DIAGNOSTIC_RANK[detail.state] > DIAGNOSTIC_RANK[current] ? detail.state : current),
    'healthy'
  );
  const calibrationState = diagnostics?.calibrationState ?? derivedWorstDetail;

  return {
    summary: {
      label: 'Calibration',
      state: calibrationState,
      summary: summarizeDiagnosticState(
        calibrationState,
        calibrationState === 'healthy'
          ? 'Calibration checks passed'
          : calibrationState === 'blocked'
            ? 'Calibration blocking readiness'
            : calibrationState === 'warning'
              ? 'Calibration follow-up required'
              : 'Calibration unavailable',
        diagnostics?.calibrationDetail
      ),
      detail: diagnostics?.calibrationDetail,
    },
    details: detailChecks,
  };
}

function summarizeMotorHealth(diagnostics?: VehicleMaintenanceSnapshot): MaintenanceCheckViewModel {
  const state = diagnostics?.motorHealthState ?? 'unknown';
  return {
    label: 'Motor health',
    state,
    summary: summarizeDiagnosticState(
      state,
      state === 'healthy'
        ? 'Motor checks passed'
        : state === 'blocked'
          ? 'Motor fault blocks dispatch'
          : state === 'warning'
            ? 'Motor test follow-up required'
            : 'Motor diagnostics unavailable',
      diagnostics?.motorHealthDetail
    ),
    detail: diagnostics?.motorHealthDetail,
  };
}

function deriveReadiness(checks: MaintenanceCheckViewModel[], isOffline: boolean): {
  readiness: FleetReadinessState;
  summary: string;
} {
  if (isOffline) {
    return {
      readiness: 'blocked',
      summary: 'Telemetry stale or vehicle offline',
    };
  }

  if (checks.some((check) => check.state === 'blocked')) {
    return {
      readiness: 'blocked',
      summary: 'Blocking diagnostic issue present',
    };
  }
  if (checks.some((check) => check.state === 'warning' || check.state === 'unknown')) {
    return {
      readiness: 'warning',
      summary: 'Diagnostics need maintenance review',
    };
  }
  return {
    readiness: 'ready',
    summary: 'Ready for deployment',
  };
}

export function projectFleetDiagnostics(args: {
  telemetry: MaintenanceTelemetrySnapshot[];
  diagnostics: VehicleMaintenanceSnapshot[];
  pods: MaintenancePodSnapshot[];
  nowMs?: number;
}): FleetDiagnosticsViewModel {
  const { telemetry, diagnostics, pods, nowMs = Date.now() } = args;
  const telemetryById = new Map(telemetry.map((vehicle) => [vehicle.id, vehicle]));
  const diagnosticsById = new Map(diagnostics.map((snapshot) => [snapshot.vehicleId, snapshot]));
  const podGroups = new Map<string, MaintenancePodSnapshot[]>();

  for (const pod of pods) {
    const list = podGroups.get(pod.vehicleId) ?? [];
    list.push(pod);
    podGroups.set(pod.vehicleId, list);
  }

  const vehicleIds = Array.from(
    new Set([
      ...telemetry.map((vehicle) => vehicle.id),
      ...diagnostics.map((snapshot) => snapshot.vehicleId),
      ...pods.map((pod) => pod.vehicleId),
    ])
  ).sort();

  const vehicles = vehicleIds.map<MaintenanceVehicleViewModel>((vehicleId) => {
    const vehicle = telemetryById.get(vehicleId);
    const vehicleDiagnostics = diagnosticsById.get(vehicleId);
    const vehiclePods = podGroups.get(vehicleId) ?? [];
    const degraded = deriveVehicleDegradedState({
      lastUpdate: vehicle?.lastUpdate,
      missionOnline: vehicle?.missionOnline,
      deliveryMode: vehicle?.deliveryMode,
      isRetroactive: vehicle?.isRetroactive,
      now: nowMs,
    });
    const motor = summarizeMotorHealth(vehicleDiagnostics);
    const podSummary = summarizePods(vehiclePods);
    const mesh = summarizeLinkQuality(vehicle?.linkQualityPercent);
    const calibration = summarizeCalibration(vehicleDiagnostics);
    const readiness = deriveReadiness(
      [motor, podSummary.summary, mesh, calibration.summary],
      degraded.offline
    );

    return {
      id: vehicleId,
      name: vehicle?.name ?? vehicleId.replace(/[_-]/g, ' ').toUpperCase(),
      readiness: readiness.readiness,
      readinessSummary: readiness.summary,
      batteryPercent: vehicle?.batteryPercent ?? null,
      altitudeMeters: vehicle?.altitudeMeters ?? 0,
      lastContactAgeMs: degraded.lastContactAgeMs,
      isOffline: degraded.offline,
      summaryChecks: {
        motor,
        sensors: podSummary.summary,
        mesh,
        calibration: calibration.summary,
      },
      detailChecks: calibration.details,
      pods: podSummary.detail,
    };
  });

  const readyCount = vehicles.filter((vehicle) => vehicle.readiness === 'ready').length;
  const blockedCount = vehicles.filter((vehicle) => vehicle.readiness === 'blocked').length;
  const warningCount = vehicles.length - readyCount - blockedCount;
  const linkReadings = telemetry
    .map((vehicle) => vehicle.linkQualityPercent)
    .filter((value): value is number => typeof value === 'number' && Number.isFinite(value));

  return {
    vehicles,
    summary: {
      readyCount,
      warningCount,
      blockedCount,
      connectedCount: telemetry.filter((vehicle) => {
        const degraded = deriveVehicleDegradedState({
          lastUpdate: vehicle.lastUpdate,
          missionOnline: vehicle.missionOnline,
          deliveryMode: vehicle.deliveryMode,
          isRetroactive: vehicle.isRetroactive,
          now: nowMs,
        });
        return !degraded.offline;
      }).length,
      averageLinkQuality:
        linkReadings.length > 0
          ? Math.round(linkReadings.reduce((sum, value) => sum + value, 0) / linkReadings.length)
          : null,
    },
  };
}

export function getReadinessBadgeVariant(readiness: FleetReadinessState): 'success' | 'warning' | 'danger' {
  if (readiness === 'ready') {
    return 'success';
  }
  if (readiness === 'blocked') {
    return 'danger';
  }
  return 'warning';
}

export function getDiagnosticBadgeVariant(state: DiagnosticState): 'success' | 'warning' | 'danger' | 'outline' {
  if (state === 'healthy') {
    return 'success';
  }
  if (state === 'blocked') {
    return 'danger';
  }
  if (state === 'warning') {
    return 'warning';
  }
  return 'outline';
}

export function createDemoFleetDiagnostics(): FleetDiagnosticsViewModel {
  return projectFleetDiagnostics({
    nowMs: 10_000,
    telemetry: [
      {
        id: 'scout_1',
        name: 'SCOUT 1',
        batteryPercent: 82,
        altitudeMeters: 0,
        linkQualityPercent: 87,
        lastUpdate: 9_000,
        missionOnline: true,
      },
      {
        id: 'scout_2',
        name: 'SCOUT 2',
        batteryPercent: 54,
        altitudeMeters: 0,
        linkQualityPercent: 58,
        lastUpdate: 8_500,
        missionOnline: true,
      },
      {
        id: 'ranger_1',
        name: 'RANGER 1',
        batteryPercent: 67,
        altitudeMeters: 0,
        linkQualityPercent: 28,
        lastUpdate: 2_000,
        missionOnline: false,
      },
    ],
    diagnostics: [
      {
        vehicleId: 'scout_1',
        motorHealthState: 'healthy',
        motorHealthDetail: 'ESC and motor vibration checks nominal',
        calibrationState: 'healthy',
        calibrationDetail: 'Bench calibration current',
        imuState: 'healthy',
        imuDetail: 'Bias drift within threshold',
        compassState: 'healthy',
        compassDetail: 'Compass offset validated',
        accelerometerState: 'healthy',
        accelerometerDetail: 'Accelerometer alignment nominal',
      },
      {
        vehicleId: 'scout_2',
        motorHealthState: 'warning',
        motorHealthDetail: 'Motor 3 current ripple above baseline',
        calibrationState: 'warning',
        calibrationDetail: 'Compass calibration expires soon',
        imuState: 'healthy',
        imuDetail: 'Bias drift within threshold',
        compassState: 'warning',
        compassDetail: 'Recheck before next sortie',
        accelerometerState: 'healthy',
        accelerometerDetail: 'Accelerometer alignment nominal',
      },
      {
        vehicleId: 'ranger_1',
        motorHealthState: 'blocked',
        motorHealthDetail: 'Motor 2 ESC fault latched during self-test',
        calibrationState: 'blocked',
        calibrationDetail: 'IMU calibration invalid after hard reset',
        imuState: 'blocked',
        imuDetail: 'Calibration file missing',
        compassState: 'warning',
        compassDetail: 'Compass offset stale',
        accelerometerState: 'warning',
        accelerometerDetail: 'Accelerometer tolerance trending high',
      },
    ],
    pods: [
      {
        vehicleId: 'scout_1',
        slotId: 'front-left',
        podSerial: 'THM-204',
        podType: 'thermal',
        lifecycleState: 'registered',
        connected: true,
        powerReady: true,
        linkReady: true,
        capabilities: ['thermal', 'survivor-detection'],
      },
      {
        vehicleId: 'scout_2',
        slotId: 'belly',
        podSerial: 'GAS-118',
        podType: 'hazmat',
        lifecycleState: 'registered',
        connected: true,
        powerReady: true,
        linkReady: false,
        capabilities: ['gas', 'hazmat'],
      },
      {
        vehicleId: 'ranger_1',
        slotId: 'rear-bay',
        podSerial: 'LDR-551',
        podType: 'lidar',
        lifecycleState: 'faulted',
        connected: false,
        powerReady: false,
        linkReady: false,
        capabilities: ['lidar', 'mapping'],
        faultDetail: 'Power rail brownout detected',
      },
    ],
  });
}
