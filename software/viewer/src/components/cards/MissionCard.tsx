'use client';

import { Card } from '@/components/ui/card';
import { Progress } from '@/components/ui/progress';
import { cn } from '@/lib/utils';

/**
 * MissionCard - Command Dock card showing mission status
 * 
 * Per spec Section 4.3 - Compact view:
 * - Timer (MM:SS)
 * - Progress bar with percentage
 * 
 * Note: Phase badge removed - already shown in StatusPill
 */

export type MissionPhase = 'IDLE' | 'SEARCHING' | 'TRACKING' | 'COMPLETE' | 'ABORTED';

export interface MissionCardProps {
  phase: MissionPhase;
  elapsedSeconds: number;
  progressPercent: number;
  isPaused: boolean;
  onTap?: () => void;
}

function formatTime(seconds: number): string {
  const mins = Math.floor(seconds / 60);
  const secs = seconds % 60;
  return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
}

export function MissionCard({
  elapsedSeconds,
  progressPercent,
  isPaused,
  onTap,
}: MissionCardProps) {
  return (
    <Card
      className={cn(
        'flex h-full cursor-pointer flex-col justify-between p-3 transition-all',
        'hover:bg-[var(--surface-3)]',
        'active:scale-[0.98]'
      )}
      onClick={onTap}
    >
      {/* Header */}
      <div className="flex items-center">
        <span className="text-[10px] font-medium uppercase tracking-wider text-white/50">
          Mission
        </span>
      </div>

      {/* Timer */}
      <div className="flex items-baseline gap-2">
        <span className="font-mono text-2xl font-bold tabular-nums text-white">
          {formatTime(elapsedSeconds)}
        </span>
        {isPaused && (
          <span className="text-xs font-medium text-[var(--warning)]">PAUSED</span>
        )}
      </div>

      {/* Progress bar */}
      <div className="space-y-1">
        <div className="flex items-center justify-between">
          <span className="text-[10px] text-white/50">Progress</span>
          <span className="font-mono text-xs font-medium text-white/70">
            {progressPercent}%
          </span>
        </div>
        <Progress 
          value={progressPercent} 
          className="h-1.5"
        />
      </div>
    </Card>
  );
}
