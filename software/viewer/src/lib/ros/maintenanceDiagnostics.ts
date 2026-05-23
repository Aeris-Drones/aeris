import type {
  DiagnosticState,
  VehicleMaintenanceSnapshot,
} from '../maintenanceDiagnostics';

interface TimeLike {
  sec?: unknown;
  nanosec?: unknown;
}

const STATE_MAP: Record<number, DiagnosticState> = {
  0: 'unknown',
  1: 'healthy',
  2: 'warning',
  3: 'blocked',
};

export interface VehicleMaintenanceDiagnosticsArrayMessage {
  observedAtMs: number | null;
  vehicles: VehicleMaintenanceSnapshot[];
}

function parseState(value: unknown): DiagnosticState {
  if (typeof value === 'number' && STATE_MAP[value]) {
    return STATE_MAP[value];
  }
  if (typeof value === 'string') {
    const normalized = value.trim().toLowerCase();
    if (normalized === 'healthy' || normalized === 'warning' || normalized === 'blocked' || normalized === 'unknown') {
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

export function parseVehicleMaintenanceDiagnosticsArray(
  raw: unknown
): VehicleMaintenanceDiagnosticsArrayMessage {
  if (!raw || typeof raw !== 'object') {
    throw new Error('Invalid maintenance diagnostics data: expected an object');
  }

  const message = raw as {
    observed_at?: unknown;
    observedAt?: unknown;
    vehicles?: unknown;
    diagnostics?: unknown;
  };
  const vehicles = Array.isArray(message.vehicles)
    ? message.vehicles
    : Array.isArray(message.diagnostics)
      ? message.diagnostics
      : [];

  return {
    observedAtMs: parseTime(message.observed_at ?? message.observedAt),
    vehicles: vehicles.map((entry) => {
      const vehicle = (entry ?? {}) as Record<string, unknown>;
      return {
        vehicleId: String(vehicle.vehicle_id ?? vehicle.vehicleId ?? '').trim(),
        motorHealthState: parseState(vehicle.motor_health_state ?? vehicle.motorHealthState),
        motorHealthDetail: parseString(vehicle.motor_health_detail ?? vehicle.motorHealthDetail),
        calibrationState: parseState(vehicle.calibration_state ?? vehicle.calibrationState),
        calibrationDetail: parseString(vehicle.calibration_detail ?? vehicle.calibrationDetail),
        imuState: parseState(vehicle.imu_state ?? vehicle.imuState),
        imuDetail: parseString(vehicle.imu_detail ?? vehicle.imuDetail),
        compassState: parseState(vehicle.compass_state ?? vehicle.compassState),
        compassDetail: parseString(vehicle.compass_detail ?? vehicle.compassDetail),
        accelerometerState: parseState(vehicle.accelerometer_state ?? vehicle.accelerometerState),
        accelerometerDetail: parseString(vehicle.accelerometer_detail ?? vehicle.accelerometerDetail),
        observedAtMs: parseTime(vehicle.observed_at ?? vehicle.observedAt) ?? null,
      };
    }).filter((vehicle) => vehicle.vehicleId.length > 0),
  };
}
