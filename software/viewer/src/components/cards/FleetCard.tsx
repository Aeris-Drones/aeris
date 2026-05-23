'use client';

import { Card } from '@/components/ui/card';
import { Badge } from '@/components/ui/badge';
import { AlertTriangle, BatteryFull, BatteryMedium, BatteryLow, BatteryWarning } from 'lucide-react';
import { cn } from '@/lib/utils';
import { getBatteryLevel } from '@/lib/batteryMonitoring';

/**
 * Vehicle operational states for the fleet monitoring system.
 * - active: Vehicle is flying and executing mission
 * - warning: Non-critical issue detected (e.g., low battery, weak signal)
 * - error: Critical issue requiring immediate attention
 * - returning: Vehicle is executing return-to-launch sequence
 * - idle: Vehicle is on ground and ready
 * - offline: Last-known state; telemetry is stale or mission progress says offline
 */
export type VehicleStatus = 'active' | 'warning' | 'error' | 'returning' | 'idle' | 'offline';

export interface VehicleInfo {
  id: string;
  name: string;
  status: VehicleStatus;
  battery: number | null;
  altitude: number;
}

export interface VehicleWarning {
  vehicleId: string;
  message: string;
  severity: 'warning' | 'critical';
}

export interface FleetCardProps {
  vehicles: VehicleInfo[];
  activeCount: number;
  totalCount: number;
  avgBattery: number | null;
  avgAltitude: number;
  warnings: VehicleWarning[];
  viewMode?: 'operator' | 'ic';
}

/**
 * Status indicator colors mapped to design system semantic tokens.
 * Colors are consistent across all fleet visualization components
 * (FleetCard, FleetSheet, map vehicle markers) for operator familiarity.
 */
const statusColors: Record<VehicleStatus, string> = {
  active: 'bg-[var(--success)]',
  warning: 'bg-[var(--warning)]',
  error: 'bg-[var(--danger)]',
  returning: 'bg-[var(--info)]',
  idle: 'bg-white/30',
  offline: 'bg-zinc-500',
};

/**
 * Returns battery icon based on charge level.
 * Icon changes at 75%, 50%, and 25% thresholds to provide
 * at-a-glance status assessment during high-tempo operations.
 */
function getBatteryIcon(percent: number | null) {
  if (percent === null) return <BatteryMedium className="h-4 w-4 opacity-50" />;
  if (percent > 75) return <BatteryFull className="h-4 w-4" />;
  if (percent > 50) return <BatteryMedium className="h-4 w-4" />;
  if (percent > 25) return <BatteryLow className="h-4 w-4" />;
  return <BatteryWarning className="h-4 w-4" />;
}

/**
 * Returns semantic color class for battery display.
 * Warning/danger colors draw attention to low battery states
 * that may require immediate RTL (Return to Launch) decisions.
 */
function getBatteryColor(percent: number | null) {
  switch (getBatteryLevel(percent)) {
    case 'healthy':
      return 'text-[var(--success)]';
    case 'warning':
      return 'text-[var(--warning)]';
    case 'critical':
      return 'text-[var(--danger)]';
    default:
      return 'text-white/40';
  }
}

/**
 * FleetCard displays a high-level overview of the drone fleet status.
 *
 * UI/UX Decisions:
 * - Compact dot indicators (max 8 visible) prevent visual overload with large fleets
 * - Warning badge uses severity-based color coding (danger for critical warnings)
 * - Battery icon changes based on level for at-a-glance status assessment
 * - Active/total count format (e.g., "3/5") clearly shows fleet utilization
 * - Hover and active states provide tactile feedback on touch devices
 *
 * Accessibility:
 * - Status dots use title attribute for vehicle name and status on hover
 * - Color is not the sole indicator (icons + text reinforce battery state)
 * - Badge includes both icon and numeric count for warnings
 */
export function FleetCard({
  vehicles,
  activeCount,
  totalCount,
  avgBattery,
  warnings,
  viewMode = 'operator',
}: FleetCardProps) {
  const hasWarnings = warnings.length > 0;
  const criticalWarnings = warnings.filter(w => w.severity === 'critical').length;
  const isIcMode = viewMode === 'ic';

  return (
    <Card
      className={cn(
        'flex h-full cursor-pointer flex-col justify-between p-4 transition-all',
        'hover:bg-[var(--surface-3)]',
        'active:scale-[0.98]',
        isIcMode && 'border-white/25 bg-black/75 p-5',
        hasWarnings && 'border-[var(--warning)]/30'
      )}
    >
      <div className="flex items-center justify-between">
        <span className={cn('font-medium uppercase tracking-wider', isIcMode ? 'text-xl text-white/90' : 'text-xs text-white/50')}>
          Fleet
        </span>
        {hasWarnings && (
          <Badge
            variant={criticalWarnings > 0 ? 'danger' : 'warning'}
            className="flex items-center gap-1 px-2 py-0.5"
          >
            <AlertTriangle className="h-3.5 w-3.5" />
            <span className={cn(isIcMode ? 'text-xl' : 'text-xs')}>{warnings.length}</span>
          </Badge>
        )}
      </div>

      {/* Fleet status dots - capped at 8 to prevent layout overflow with large swarms */}
      <div className="flex items-center gap-1.5">
        {vehicles.slice(0, 8).map((vehicle) => (
          <div
            key={vehicle.id}
            className={cn(
              'h-3 w-3 rounded-full transition-all',
              statusColors[vehicle.status]
            )}
            title={`${vehicle.name}: ${vehicle.status}`}
          />
        ))}
        {vehicles.length > 8 && (
          <span className={cn('ml-1 text-white/50', isIcMode ? 'text-lg' : 'text-xs')}>
            +{vehicles.length - 8}
          </span>
        )}
      </div>

      <div className="flex items-center justify-between">
        <div className="flex items-baseline gap-1.5">
          <span className={cn('font-mono font-bold text-white', isIcMode ? 'text-4xl' : 'text-xl')}>
            {activeCount}
          </span>
          <span className={cn(isIcMode ? 'text-2xl text-white/80' : 'text-sm text-white/50')}>
            /{totalCount}
          </span>
        </div>

        <div className={cn('flex items-center gap-1.5', getBatteryColor(avgBattery))}>
          {getBatteryIcon(avgBattery)}
          <span className={cn('font-mono', isIcMode ? 'text-xl' : 'text-sm')}>
            {avgBattery === null ? '--' : `${avgBattery}%`}
          </span>
        </div>
      </div>
    </Card>
  );
}
