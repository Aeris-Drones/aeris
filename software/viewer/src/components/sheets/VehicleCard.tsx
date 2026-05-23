'use client';

import { Button } from '@/components/ui/button';
import { MapPin, Video, Home, Signal, Gauge, Radio, TimerReset } from 'lucide-react';
import { cn } from '@/lib/utils';
import { formatLastContactAge } from '@/lib/degradedVehicleState';
import { formatRemainingFlightTime, getBatteryLevel } from '@/lib/batteryMonitoring';

/**
 * Vehicle operational status for UI display.
 *
 * Status values drive visual styling (colors, glow effects) and available actions
 * in the GCS. Critical statuses (error, returning) receive heightened visual
 * prominence to ensure operator awareness during multi-vehicle operations.
 */
export type VehicleStatus = 'active' | 'warning' | 'error' | 'returning' | 'idle' | 'offline';

/**
 * Core telemetry snapshot for a single vehicle.
 *
 * Data is received via ROS telemetry topics and normalized for UI consumption.
 * Altitude is relative to takeoff (AMSL offset applied upstream). Percentage
 * fields (battery, linkQuality, coverage) are clamped 0-100 by the telemetry
 * adapter to prevent display anomalies.
 */
export interface VehicleInfo {
  id: string;
  name: string;
  status: VehicleStatus;
  battery: number | null;
  altitude: number;
  linkQuality?: number;
  coverage?: number;
  remainingFlightTimeSec?: number | null;
  slamMode?: string;
  lastContactAgeMs?: number | null;
  isLastKnown?: boolean;
  deliveryMode?: 'live' | 'replayed';
  isRetroactive?: boolean;
}

/**
 * Props for the VehicleCard component.
 *
 * Callbacks integrate with the map viewport (onLocate), video streaming
 * service (onViewFeed), and mission commander (onRTH). All callbacks are
 * expected to be memoized by the parent to prevent unnecessary re-renders
 * in the vehicle list.
 */
export interface VehicleCardProps {
  vehicle: VehicleInfo;
  isSelected: boolean;
  onLocate: () => void;
  onViewFeed?: () => void;
  onRTH?: () => void;
  viewMode?: 'operator' | 'ic';
}

/**
 * Visual styling configuration mapped by vehicle status.
 *
 * Glow intensity correlates with operational urgency to draw operator
 * attention during high-stress scenarios (error, returning). Colors align
 * with aviation standards: green (normal), amber (caution), red (warning).
 */
const statusConfig: Record<VehicleStatus, {
  label: string;
  color: string;
  glow: string;
  dot: string;
}> = {
  active: {
    label: 'ACTIVE',
    color: 'text-emerald-400',
    glow: 'shadow-[0_0_30px_rgba(52,211,153,0.15)]',
    dot: 'bg-emerald-400'
  },
  warning: {
    label: 'WARNING',
    color: 'text-amber-400',
    glow: 'shadow-[0_0_30px_rgba(251,191,36,0.15)]',
    dot: 'bg-amber-400'
  },
  error: {
    label: 'ERROR',
    color: 'text-red-400',
    glow: 'shadow-[0_0_30px_rgba(248,113,113,0.2)]',
    dot: 'bg-red-400'
  },
  returning: {
    label: 'RTH',
    color: 'text-cyan-400',
    glow: 'shadow-[0_0_30px_rgba(34,211,238,0.15)]',
    dot: 'bg-cyan-400'
  },
  idle: {
    label: 'STANDBY',
    color: 'text-white/40',
    glow: '',
    dot: 'bg-white/40'
  },
  offline: {
    label: 'OFFLINE',
    color: 'text-zinc-300',
    glow: '',
    dot: 'bg-zinc-500'
  },
};

/**
 * Returns the appropriate color class for a given battery percentage.
 *
 * Colors match the shared getBatteryLevel thresholds so cards, fleet summary,
 * and alerts all classify battery health the same way.
 */
function getBatteryColor(battery: number | null): string {
  switch (getBatteryLevel(battery)) {
    case 'healthy':
      return 'text-emerald-400';
    case 'warning':
      return 'text-amber-400';
    case 'critical':
      return 'text-red-400';
    default:
      return 'text-white/40';
  }
}

/**
 * Returns the SVG stroke color class for the battery arc indicator.
 * Mirrors getBatteryColor logic for visual consistency.
 */
function getBatteryStroke(battery: number | null): string {
  switch (getBatteryLevel(battery)) {
    case 'healthy':
      return 'stroke-emerald-400';
    case 'warning':
      return 'stroke-amber-400';
    case 'critical':
      return 'stroke-red-400';
    default:
      return 'stroke-white/40';
  }
}

function formatSlamModeLabel(slamMode?: string): string {
  const normalized = typeof slamMode === 'string' ? slamMode.trim().toLowerCase() : '';

  if (!normalized || normalized === 'unknown') {
    return 'UNKNOWN';
  }
  if (normalized === 'vio') {
    return 'VIO';
  }
  if (normalized === 'liosam') {
    return 'LIO-SAM';
  }

  return normalized.toUpperCase();
}

/**
 * Circular battery indicator using SVG stroke-dasharray technique.
 *
 * Renders a progress arc that depletes counter-clockwise as battery drains.
 * The -90deg rotation ensures the arc starts at 12 o'clock position.
 */
function BatteryArc({ battery }: { battery: number | null }) {
  const radius = 26;
  const circumference = 2 * Math.PI * radius;
  const value = battery ?? 0;
  const strokeDashoffset = circumference * (1 - value / 100);

  return (
    <svg className="h-16 w-16 -rotate-90" viewBox="0 0 60 60">
      <circle
        cx="30"
        cy="30"
        r={radius}
        fill="none"
        stroke="currentColor"
        strokeWidth="3"
        className="text-white/[0.06]"
      />
      <circle
        cx="30"
        cy="30"
        r={radius}
        fill="none"
        strokeWidth="3"
        strokeLinecap="round"
        strokeDasharray={circumference}
        strokeDashoffset={strokeDashoffset}
        className={getBatteryStroke(battery)}
      />
    </svg>
  );
}

/**
 * Displays a single vehicle's telemetry and provides quick actions.
 *
 * This component is rendered for each vehicle in the fleet sheet. It receives
 * pre-normalized telemetry data from the vehicle store and delegates actions
 * to the parent component to maintain loose coupling with map/video services.
 *
 * Performance: The component is optimized for frequent re-renders as telemetry
 * updates arrive (typically 1-10Hz per vehicle). Avoid adding heavy computations
 * or effects here; preprocess data in the telemetry adapter instead.
 */
export function VehicleCard({
  vehicle,
  isSelected,
  onLocate,
  onViewFeed,
  onRTH,
  viewMode = 'operator',
}: VehicleCardProps) {
  const status = statusConfig[vehicle.status];
  const isOffline = vehicle.status === 'offline';
  const isIcMode = viewMode === 'ic';
  const batteryDisplay = typeof vehicle.battery === 'number' ? Math.round(vehicle.battery) : null;
  const remainingFlightTimeLabel = formatRemainingFlightTime(vehicle.remainingFlightTimeSec);
  const remainingFlightTimeUnavailable = vehicle.remainingFlightTimeSec == null;

  return (
    <div
      className={cn(
        'group relative overflow-hidden rounded-xl transition-all duration-300',
        'bg-white/[0.03] backdrop-blur-md',
        'border border-white/[0.06]',
        status.glow,
        isIcMode && 'border-white/18 bg-black/60',
        isOffline && 'grayscale opacity-75 bg-zinc-950/60 border-zinc-500/20',
        isSelected && 'ring-2 ring-cyan-500/50',
        vehicle.status === 'error' && 'border-red-500/20'
      )}
    >
      <div className="flex items-start justify-between p-3 pb-0">
        <div className="flex flex-col gap-0.5">
          <span className={cn(isIcMode ? 'text-xl font-medium text-white' : 'text-lg font-light text-white')}>{vehicle.name}</span>
          <div className="flex items-center gap-2">
            <span className={cn('h-1.5 w-1.5 rounded-full animate-pulse', status.dot)} />
            <span className={cn(isIcMode ? 'text-xl font-semibold tracking-wider' : 'text-base font-semibold tracking-wider', status.color)}>
              {status.label}
            </span>
          </div>
          <span
            className={cn(
              isIcMode
                ? 'text-xl font-medium tracking-[0.12em] text-white/60'
                : 'text-base font-medium tracking-[0.12em] text-white/55'
            )}
            data-testid="vehicle-slam-mode"
          >
            SLAM: {formatSlamModeLabel(vehicle.slamMode)}
          </span>
          {isOffline && (
            <span
              className={cn(
                isIcMode
                  ? 'text-xl font-semibold tracking-[0.12em] text-zinc-100'
                  : 'text-base font-semibold tracking-[0.12em] text-zinc-200'
              )}
              data-testid="vehicle-last-known-age"
            >
              LAST CONTACT {formatLastContactAge(vehicle.lastContactAgeMs)}
            </span>
          )}
        </div>

        <div className="relative">
          <BatteryArc battery={vehicle.battery} />
          <div className="absolute inset-0 flex flex-col items-center justify-center">
            <span className={cn(isIcMode ? 'font-mono text-xl font-light tabular-nums' : 'font-mono text-base font-light tabular-nums', getBatteryColor(vehicle.battery))}>
              {batteryDisplay ?? '--'}
            </span>
            <span className={cn(isIcMode ? 'text-xl text-white/50' : 'text-base text-white/45')}>{batteryDisplay === null ? '' : '%'}</span>
          </div>
        </div>
      </div>

      {/* Telemetry metrics grid */}
      <div className="grid grid-cols-4 gap-px bg-white/[0.02] mx-3 my-2 rounded-lg overflow-hidden">
        <div className="flex flex-col items-center gap-0.5 bg-white/[0.02] py-2">
          <Gauge className="h-3 w-3 text-white/30" />
          <span className={cn(isIcMode ? 'font-mono text-xl text-white/85' : 'font-mono text-base text-white/80')}>{vehicle.altitude}</span>
          <span className={cn(isIcMode ? 'text-xl text-white/50 uppercase tracking-wide' : 'text-base text-white/45 uppercase tracking-wide')}>Alt (m)</span>
        </div>
        <div className="flex flex-col items-center gap-0.5 bg-white/[0.02] py-2">
          <Radio className="h-3 w-3 text-white/30" />
          <span className={cn(isIcMode ? 'font-mono text-xl text-white/85' : 'font-mono text-base text-white/80')}>{vehicle.linkQuality ?? '--'}</span>
          <span className={cn(isIcMode ? 'text-xl text-white/50 uppercase tracking-wide' : 'text-base text-white/45 uppercase tracking-wide')}>Link %</span>
        </div>
        <div className="flex flex-col items-center gap-0.5 bg-white/[0.02] py-2">
          <Signal className="h-3 w-3 text-white/30" />
          <span className={cn(isIcMode ? 'font-mono text-xl text-white/85' : 'font-mono text-base text-white/80')}>{vehicle.coverage ?? '--'}</span>
          <span className={cn(isIcMode ? 'text-xl text-white/50 uppercase tracking-wide' : 'text-base text-white/45 uppercase tracking-wide')}>Cover %</span>
        </div>
        <div className="flex flex-col items-center gap-0.5 bg-white/[0.02] px-1 py-2">
          <TimerReset className="h-3 w-3 text-white/30" />
          <span
            className={cn(
              'text-center text-white/85',
              remainingFlightTimeUnavailable
                ? (isIcMode ? 'text-base font-medium leading-tight' : 'text-[11px] font-medium leading-tight')
                : (isIcMode ? 'font-mono text-xl' : 'font-mono text-base')
            )}
          >
            {remainingFlightTimeLabel}
          </span>
          <span className={cn(isIcMode ? 'text-xl text-white/50 uppercase tracking-wide' : 'text-base text-white/45 uppercase tracking-wide')}>Flight</span>
        </div>
      </div>

      {/* Action buttons */}
      <div className="flex items-center gap-1.5 p-3 pt-0">
        <Button
          variant="ghost"
          size="sm"
          className={cn(
            'flex-1 rounded-lg',
            isIcMode ? 'h-11 text-lg' : 'h-10 text-base',
            'text-white/50 hover:text-white hover:bg-white/10'
          )}
          onClick={onLocate}
        >
          <MapPin className="mr-1 h-3 w-3" />
          Locate
        </Button>
        {onViewFeed && (
          <Button
            variant="ghost"
            size="sm"
            className={cn(
              'flex-1 rounded-lg',
              isIcMode ? 'h-11 text-lg' : 'h-10 text-base',
              'text-white/50 hover:text-white hover:bg-white/10'
            )}
            onClick={onViewFeed}
          >
            <Video className="mr-1 h-3 w-3" />
            Feed
          </Button>
        )}
        {onRTH && vehicle.status !== 'returning' && vehicle.status !== 'idle' && (
          <Button
            variant="ghost"
            size="sm"
            className={cn(
              'px-3 rounded-lg',
              isIcMode ? 'h-11 text-lg' : 'h-10 text-base',
              'text-amber-400/70 hover:text-amber-400 hover:bg-amber-500/10'
            )}
            onClick={onRTH}
            title="Return to Home"
          >
            <Home className="h-3 w-3" />
          </Button>
        )}
      </div>
    </div>
  );
}
