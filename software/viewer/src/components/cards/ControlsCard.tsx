'use client';

import { useState, useEffect, useCallback } from 'react';
import { Card } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { ShimmerButton } from '@/components/ui/shimmer-button';
import { Play, Pause, Square } from 'lucide-react';
import { cn } from '@/lib/utils';

/**
 * ControlsCard - Command Dock card with mission controls
 * 
 * Per spec Section 4.3:
 * - START: Green gradient, disabled during mission (uses ShimmerButton)
 * - PAUSE/RESUME: Yellow, toggles based on state
 * - ABORT: Red, 5-second countdown confirmation
 */

export type MissionPhase = 'IDLE' | 'SEARCHING' | 'TRACKING' | 'COMPLETE' | 'ABORTED';

export interface ControlsCardProps {
  missionPhase: MissionPhase;
  isPaused: boolean;
  canStart: boolean;
  canPause: boolean;
  canAbort: boolean;
  onStart: () => void;
  onPause: () => void;
  onResume: () => void;
  onAbort: () => void;
}

export function ControlsCard({
  missionPhase,
  isPaused,
  canStart,
  canPause,
  canAbort,
  onStart,
  onPause,
  onResume,
  onAbort,
}: ControlsCardProps) {
  const [abortCountdown, setAbortCountdown] = useState<number | null>(null);
  
  const isIdle = missionPhase === 'IDLE';
  const isActive = missionPhase === 'SEARCHING' || missionPhase === 'TRACKING';

  // Handle abort countdown
  useEffect(() => {
    if (abortCountdown === null) return;
    
    if (abortCountdown === 0) {
      onAbort();
      setAbortCountdown(null);
      return;
    }

    const timer = setTimeout(() => {
      setAbortCountdown(abortCountdown - 1);
    }, 1000);

    return () => clearTimeout(timer);
  }, [abortCountdown, onAbort]);

  // Reset countdown if mission changes
  useEffect(() => {
    if (!isActive) {
      setAbortCountdown(null);
    }
  }, [isActive]);

  const handleAbortClick = useCallback(() => {
    if (abortCountdown !== null) {
      // Cancel if clicked again during countdown
      setAbortCountdown(null);
    } else {
      // Start 5-second countdown
      setAbortCountdown(5);
    }
  }, [abortCountdown]);

  return (
    <Card className="flex h-full flex-col justify-between p-4">
      {/* Header */}
      <div className="flex items-center justify-between">
        <span className="text-xs font-medium uppercase tracking-wider text-white/50">
          Controls
        </span>
      </div>

      {/* Button row */}
      <div className="flex items-stretch gap-3">
        {/* START button - only show when idle */}
        {isIdle && (
          <ShimmerButton
            className="flex-1 px-5 py-2.5 text-sm font-semibold"
            shimmerColor="rgba(34, 197, 94, 0.5)"
            background="rgba(22, 101, 52, 0.8)"
            borderRadius="8px"
            disabled={!canStart}
            onClick={onStart}
          >
            <Play className="mr-2 h-4 w-4 fill-current" />
            START
          </ShimmerButton>
        )}

        {/* PAUSE/RESUME button - only show when active */}
        {isActive && (
          <Button
            variant="outline"
            className={cn(
              'flex-1 h-auto py-2.5 border-[var(--warning)]/50 text-[var(--warning)]',
              'hover:bg-[var(--warning)]/10 hover:text-[var(--warning)]',
              !canPause && 'opacity-50'
            )}
            disabled={!canPause}
            onClick={isPaused ? onResume : onPause}
          >
            {isPaused ? (
              <>
                <Play className="mr-2 h-4 w-4" />
                RESUME
              </>
            ) : (
              <>
                <Pause className="mr-2 h-4 w-4" />
                PAUSE
              </>
            )}
          </Button>
        )}

        {/* ABORT button - show when active */}
        {isActive && (
          <Button
            variant="outline"
            className={cn(
              'flex-1 h-auto py-2.5 transition-all',
              abortCountdown !== null
                ? 'border-[var(--danger)] bg-[var(--danger)]/20 text-[var(--danger)]'
                : 'border-[var(--danger)]/50 text-[var(--danger)] hover:bg-[var(--danger)]/10 hover:text-[var(--danger)]',
              !canAbort && 'opacity-50'
            )}
            disabled={!canAbort}
            onClick={handleAbortClick}
          >
            <Square className="mr-2 h-4 w-4 fill-current" />
            {abortCountdown !== null ? (
              <span className="font-mono">{abortCountdown}s</span>
            ) : (
              'ABORT'
            )}
          </Button>
        )}

        {/* Completed/Aborted state */}
        {(missionPhase === 'COMPLETE' || missionPhase === 'ABORTED') && (
          <ShimmerButton
            className="flex-1 px-5 py-2.5 text-sm font-semibold"
            shimmerColor="rgba(34, 197, 94, 0.5)"
            background="rgba(22, 101, 52, 0.8)"
            borderRadius="8px"
            disabled={!canStart}
            onClick={onStart}
          >
            <Play className="mr-2 h-4 w-4 fill-current" />
            NEW
          </ShimmerButton>
        )}
      </div>

      {/* Status hint */}
      <div className="text-center">
        <span className="text-[10px] text-white/40">
          {isIdle && 'Ready to start mission'}
          {isActive && !isPaused && abortCountdown === null && 'Mission in progress'}
          {isActive && isPaused && 'Mission paused'}
          {isActive && abortCountdown !== null && 'Click again to cancel abort'}
          {missionPhase === 'COMPLETE' && 'Mission completed'}
          {missionPhase === 'ABORTED' && 'Mission aborted'}
        </span>
      </div>
    </Card>
  );
}
