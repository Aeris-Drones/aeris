'use client';

import { ReactNode } from 'react';
import { cn } from '@/lib/utils';
import { Badge } from '@/components/ui/badge';
import { Zap, AlertTriangle, XCircle, Bell } from 'lucide-react';

/** Mission phase states for the GCS lifecycle */
export type MissionPhase =
  | 'IDLE'
  | 'PLANNING'
  | 'SEARCHING'
  | 'TRACKING'
  | 'COMPLETE'
  | 'ABORTED';

/** Telemetry link quality states - drives connection indicator UI */
export type ConnectionStatus = 'connected' | 'degraded' | 'disconnected';

export interface DetectionStatusCounts {
  thermal: number;
  acoustic: number;
  gas: number;
  pending: number;
  confirmed: number;
}

/** Props for the StatusPill component - centralizes mission-critical information */
export interface StatusPillProps {
  /** Optional custom logo element; defaults to "AERIS" text */
  logo?: ReactNode;
  /** Current mission phase - drives badge color and pulse animation */
  missionPhase: MissionPhase;
  /** Mission elapsed time in seconds */
  elapsedTime: number;
  /** Mission progress percentage (0-100) */
  progressPercent: number;
  /** Telemetry connection quality state */
  connectionStatus: ConnectionStatus;
  /** Number of active alerts to display on bell icon */
  alertCount: number;
  /** Whether there are unread alerts (shows red dot indicator) */
  hasUnreadAlerts: boolean;
  /** Live fused detection counts shown in the status bar */
  detectionCounts?: DetectionStatusCounts;
  /** Callback when alert bell is clicked */
  onAlertClick?: () => void;
}

/** Mission phase UI configuration - drives badge color and pulse animation */
const phaseConfig: Record<MissionPhase, {
  label: string;
  variant: 'default' | 'success' | 'info' | 'danger' | 'secondary';
  pulse: boolean;
  icon?: string;
}> = {
  IDLE: { label: 'IDLE', variant: 'secondary', pulse: false },
  PLANNING: { label: 'PLANNING', variant: 'default', pulse: true },
  SEARCHING: { label: 'SEARCHING', variant: 'success', pulse: true },
  TRACKING: { label: 'TRACKING', variant: 'info', pulse: true },
  COMPLETE: { label: 'COMPLETE', variant: 'info', pulse: false, icon: '✓' },
  ABORTED: { label: 'ABORTED', variant: 'danger', pulse: false },
};

/** Connection status UI configuration - maps state to icon and color */
const connectionConfig: Record<ConnectionStatus, {
  label: string;
  colorClass: string;
  Icon: typeof Zap;
}> = {
  connected: { label: 'Connected', colorClass: 'text-[var(--success)]', Icon: Zap },
  degraded: { label: 'Degraded', colorClass: 'text-[var(--warning)]', Icon: AlertTriangle },
  disconnected: { label: 'Disconnected', colorClass: 'text-[var(--danger)]', Icon: XCircle },
};

/** Formats elapsed seconds into MM:SS display format */
function formatElapsedTime(seconds: number): string {
  const mins = Math.floor(seconds / 60);
  const secs = seconds % 60;
  return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
}

/**
 * Compact mission status indicator for the GCS header.
 *
 * Displays mission phase, elapsed time, progress percentage, connection quality,
 * and alert count in a glassmorphism pill format. Mission phase drives the
 * badge color and pulse animation for active states.
 *
 * @example
 * ```tsx
 * <StatusPill
 *   missionPhase="SEARCHING"
 *   elapsedTime={360}
 *   progressPercent={45}
 *   connectionStatus="connected"
 *   detectionCounts={{ thermal: 3, acoustic: 2, gas: 1, pending: 4, confirmed: 2 }}
 *   alertCount={2}
 *   hasUnreadAlerts={true}
 *   onAlertClick={() => openAlertsPanel()}
 * />
 * ```
 */
export function StatusPill({
  logo,
  missionPhase,
  elapsedTime,
  progressPercent,
  connectionStatus,
  alertCount,
  hasUnreadAlerts,
  detectionCounts,
  onAlertClick,
}: StatusPillProps) {
  const phase = phaseConfig[missionPhase];
  const connection = connectionConfig[connectionStatus];
  const ConnectionIcon = connection.Icon;
  const liveDetections: DetectionStatusCounts = detectionCounts ?? {
    thermal: 0,
    acoustic: 0,
    gas: 0,
    pending: 0,
    confirmed: 0,
  };

  return (
    <div
      className="flex items-center gap-1 rounded-full border border-[var(--glass-border)] bg-[var(--glass-bg)] px-1 py-1 backdrop-blur-xl"
      style={{ boxShadow: 'var(--glass-shadow)' }}
    >
      <div className="flex h-8 items-center justify-center rounded-full bg-[var(--surface-2)] px-3">
        {logo || (
          <span className="font-mono text-xs font-bold tracking-wider text-[var(--foreground)]">
            AERIS
          </span>
        )}
      </div>

      <div className="h-4 w-px bg-[var(--glass-border)]" />

      <Badge variant={phase.variant} className="flex items-center gap-2 rounded-full px-3 py-1.5">
        {phase.pulse ? (
          <span className="relative flex h-2 w-2">
            <span className="absolute inline-flex h-full w-full animate-ping rounded-full bg-current opacity-75" />
            <span className="relative inline-flex h-2 w-2 rounded-full bg-current" />
          </span>
        ) : phase.icon ? (
          <span className="text-xs">{phase.icon}</span>
        ) : (
          <span className="h-2 w-2 rounded-full bg-current opacity-50" />
        )}
        <span className="text-xs font-semibold tracking-wide">{phase.label}</span>
      </Badge>

      <div className="h-4 w-px bg-[var(--glass-border)]" />

      <div className="flex items-center px-3">
        <span className="font-mono text-sm font-medium tabular-nums text-[var(--foreground)]">
          {formatElapsedTime(elapsedTime)}
        </span>
      </div>

      <div className="h-4 w-px bg-[var(--glass-border)]" />

      <div className="flex items-center gap-2 px-3">
        <span className="text-[10px] uppercase tracking-wide text-white/50">Progress</span>
        <div className="relative h-1.5 w-16 overflow-hidden rounded-full bg-white/10">
          <div
            className="absolute inset-y-0 left-0 rounded-full bg-[var(--success)] transition-all duration-300"
            style={{ width: `${progressPercent}%` }}
          />
        </div>
        <span className="font-mono text-xs tabular-nums text-white/70">
          {progressPercent}%
        </span>
      </div>

      <div className="h-4 w-px bg-[var(--glass-border)]" />

      <div className="flex items-center gap-1.5 px-3">
        <span className="text-[10px] uppercase tracking-wide text-white/50">Det</span>
        <span className="font-mono text-xs text-orange-400">T{liveDetections.thermal}</span>
        <span className="font-mono text-xs text-sky-400">A{liveDetections.acoustic}</span>
        <span className="font-mono text-xs text-amber-400">G{liveDetections.gas}</span>
        <span className="text-white/25">|</span>
        <span className="font-mono text-xs text-emerald-400">P{liveDetections.pending}</span>
        <span className="font-mono text-xs text-white/70">C{liveDetections.confirmed}</span>
      </div>

      <div className="h-4 w-px bg-[var(--glass-border)]" />

      <div className={cn('flex items-center gap-1.5 px-3', connection.colorClass)}>
        <ConnectionIcon className="h-3.5 w-3.5" />
        <span className="text-xs font-medium">{connection.label}</span>
      </div>

      <div className="h-4 w-px bg-[var(--glass-border)]" />

      <button
        onClick={onAlertClick}
        className={cn(
          'relative flex h-8 items-center gap-1.5 rounded-full px-3 transition-colors',
          alertCount > 0 ? 'hover:bg-[var(--surface-3)]' : 'opacity-50'
        )}
      >
        <Bell className="h-4 w-4" />
        {alertCount > 0 && (
          <span className="text-xs font-medium text-[var(--foreground)]">
            {alertCount}
          </span>
        )}
        {hasUnreadAlerts && (
          <span className="absolute -right-0.5 -top-0.5 h-2.5 w-2.5 rounded-full bg-[var(--danger)]" />
        )}
      </button>
    </div>
  );
}
