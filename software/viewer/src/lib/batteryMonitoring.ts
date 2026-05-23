export const BATTERY_HEALTHY_THRESHOLD_PERCENT = 50;
export const LOW_BATTERY_WARNING_THRESHOLD_PERCENT = 25;

export type BatteryLevel = "unknown" | "healthy" | "warning" | "critical";

export interface BatteryAlertVehicle {
  id: string;
  name: string;
  battery: number | null;
  remainingFlightTimeSec?: number | null;
}

export interface BatteryAlertTransition {
  type: "entered_low" | "recovered";
  vehicleId: string;
  vehicleName: string;
  battery: number;
  remainingFlightTimeSec: number | null;
  alertId: string;
}

export function getBatteryLevel(
  batteryPercent: number | null | undefined,
  lowBatteryThresholdPercent: number = LOW_BATTERY_WARNING_THRESHOLD_PERCENT
): BatteryLevel {
  if (typeof batteryPercent !== "number" || !Number.isFinite(batteryPercent)) {
    return "unknown";
  }
  if (batteryPercent > BATTERY_HEALTHY_THRESHOLD_PERCENT) {
    return "healthy";
  }
  if (batteryPercent > lowBatteryThresholdPercent) {
    return "warning";
  }
  return "critical";
}

export function getLowBatteryAlertId(vehicleId: string): string {
  return `battery-low-${vehicleId}`;
}

export function formatRemainingFlightTime(
  remainingFlightTimeSec: number | null | undefined
): string {
  if (
    typeof remainingFlightTimeSec !== "number" ||
    !Number.isFinite(remainingFlightTimeSec) ||
    remainingFlightTimeSec < 0
  ) {
    return "Unavailable";
  }

  const rounded = Math.max(0, Math.round(remainingFlightTimeSec));
  if (rounded < 60) {
    return `${rounded}s`;
  }

  const hours = Math.floor(rounded / 3600);
  const minutes = Math.floor((rounded % 3600) / 60);

  if (hours > 0) {
    return minutes > 0 ? `${hours}h ${minutes}m` : `${hours}h`;
  }

  return `${minutes}m`;
}

export function deriveBatteryAlertTransitions(
  previousState: ReadonlyMap<string, boolean>,
  vehicles: BatteryAlertVehicle[],
  lowBatteryThresholdPercent: number = LOW_BATTERY_WARNING_THRESHOLD_PERCENT
): {
  state: Map<string, boolean>;
  transitions: BatteryAlertTransition[];
} {
  const nextState = new Map<string, boolean>();
  const transitions: BatteryAlertTransition[] = [];

  for (const vehicle of vehicles) {
    const battery = vehicle.battery;
    const hasBatteryReading =
      typeof battery === "number" && Number.isFinite(battery);
    const isLowBattery =
      hasBatteryReading &&
      battery <= lowBatteryThresholdPercent;
    const wasLowBattery = previousState.get(vehicle.id) === true;

    nextState.set(vehicle.id, hasBatteryReading ? isLowBattery : wasLowBattery);

    if (!hasBatteryReading) {
      continue;
    }

    if (isLowBattery && !wasLowBattery) {
      transitions.push({
        type: "entered_low",
        vehicleId: vehicle.id,
        vehicleName: vehicle.name,
        battery,
        remainingFlightTimeSec: vehicle.remainingFlightTimeSec ?? null,
        alertId: getLowBatteryAlertId(vehicle.id),
      });
      continue;
    }

    if (!isLowBattery && wasLowBattery) {
      transitions.push({
        type: "recovered",
        vehicleId: vehicle.id,
        vehicleName: vehicle.name,
        battery,
        remainingFlightTimeSec: vehicle.remainingFlightTimeSec ?? null,
        alertId: getLowBatteryAlertId(vehicle.id),
      });
    }
  }

  return {
    state: nextState,
    transitions,
  };
}
