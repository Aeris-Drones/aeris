'use client';

import { ReactNode } from 'react';
import { cn } from '@/lib/utils';
import { Badge } from '@/components/ui/badge';
import { Zap, AlertTriangle, XCircle, Bell } from 'lucide-react';

/**
 * StatusPill - Top center floating status bar
 * 
 * Per spec Section 4.1 - Uses shadcn Badge component
 * Per spec Section 9.1: StatusPill = Badge (custom layout, pulsing dot)
 */

export type MissionPhase =
  | 'IDLE'
  | 'PLANNING'
  | 'SEARCHING'
  | 'TRACKING'
  | 'COMPLETE'
  | 'ABORTED';
export type ConnectionStatus = 'connected' | 'degraded' | 'disconnected';

export interface StatusPillProps {
  logo?: ReactNode;
  missionPhase: MissionPhase;
  elapsedTime: number; // seconds
  progressPercent: number; // 0-100
  connectionStatus: ConnectionStatus;
  alertCount: number;
  hasUnreadAlerts: boolean;
  onAlertClick?: () => void;
}

// Per spec Section 4.1 Visual States
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

const connectionConfig: Record<ConnectionStatus, { 
  label: string; 
  colorClass: string; 
  Icon: typeof Zap;
}> = {
  connected: { label: 'Connected', colorClass: 'text-[var(--success)]', Icon: Zap },
  degraded: { label: 'Degraded', colorClass: 'text-[var(--warning)]', Icon: AlertTriangle },
  disconnected: { label: 'Disconnected', colorClass: 'text-[var(--danger)]', Icon: XCircle },
};

function formatElapsedTime(seconds: number): string {
  const mins = Math.floor(seconds / 60);
  const secs = seconds % 60;
  return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
}

export function StatusPill({
  logo,
  missionPhase,
  elapsedTime,
  progressPercent,
  connectionStatus,
  alertCount,
  hasUnreadAlerts,
  onAlertClick,
}: StatusPillProps) {
  const phase = phaseConfig[missionPhase];
  const connection = connectionConfig[connectionStatus];
  const ConnectionIcon = connection.Icon;

  return (
    <div 
      className="flex items-center gap-1 rounded-full border border-[var(--glass-border)] bg-[var(--glass-bg)] px-1 py-1 backdrop-blur-xl"
      style={{ boxShadow: 'var(--glass-shadow)' }}
    >
      {/* Logo */}
      <div className="flex h-8 items-center justify-center rounded-full bg-[var(--surface-2)] px-3">
        {logo || (
          <span className="font-mono text-xs font-bold tracking-wider text-[var(--foreground)]">
            AERIS
          </span>
        )}
      </div>

      {/* Divider */}
      <div className="h-4 w-px bg-[var(--glass-border)]" />

      {/* Mission Phase Badge - using shadcn Badge */}
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

      {/* Divider */}
      <div className="h-4 w-px bg-[var(--glass-border)]" />

      {/* Elapsed Time */}
      <div className="flex items-center px-3">
        <span className="font-mono text-sm font-medium tabular-nums text-[var(--foreground)]">
          {formatElapsedTime(elapsedTime)}
        </span>
      </div>

      {/* Divider */}
      <div className="h-4 w-px bg-[var(--glass-border)]" />

      {/* Progress Bar */}
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

      {/* Divider */}
      <div className="h-4 w-px bg-[var(--glass-border)]" />

      {/* Connection Status - using Lucide icons */}
      <div className={cn('flex items-center gap-1.5 px-3', connection.colorClass)}>
        <ConnectionIcon className="h-3.5 w-3.5" />
        <span className="text-xs font-medium">{connection.label}</span>
      </div>

      {/* Divider */}
      <div className="h-4 w-px bg-[var(--glass-border)]" />

      {/* Alert Bell - using Lucide icon */}
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
