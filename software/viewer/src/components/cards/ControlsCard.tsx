'use client';

import { Card } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { ShimmerButton } from '@/components/ui/shimmer-button';
import { Play, Pause } from 'lucide-react';
import { cn } from '@/lib/utils';

/**
 * Mission lifecycle phases for search and rescue operations.
 * Phases progress from IDLE -> PLANNING -> SEARCHING -> TRACKING -> COMPLETE
 * or can transition to ABORTED at any active phase.
 */
export type MissionPhase =
  | 'IDLE'
  | 'PLANNING'
  | 'SEARCHING'
  | 'TRACKING'
  | 'COMPLETE'
  | 'ABORTED';

export type SearchPattern = 'lawnmower' | 'spiral';

export interface ControlsCardProps {
  missionPhase: MissionPhase;
  isPaused: boolean;
  canStart: boolean;
  canPause: boolean;
  hasValidStartZone?: boolean;
  selectedPattern?: SearchPattern;
  setSelectedPattern?: (pattern: SearchPattern) => void;
  startMissionError?: string | null;
  onStart: () => void;
  onPause: () => void;
  onResume: () => void;
}

/**
 * ControlsCard provides mission control interface with safety features.
 *
 * UI/UX Decisions:
 * - ShimmerButton for START/NEW creates visual prominence for primary actions
 * - Mission abort lives in the persistent shell emergency stop surface
 * - Pattern selector only visible in IDLE phase to prevent mid-mission changes
 * - Status message area provides contextual feedback on current state
 * - Warning/error messages appear inline to guide user corrections
 *
 * Safety Features:
 * - Disabled states prevent invalid actions for current phase
 * - Visual distinction between paused vs running mission states
 *
 * Accessibility:
 * - Buttons have clear labels with icons
 * - Status messages are descriptive and action-oriented
 * - Error messages use warning/danger colors for visibility
 */
export function ControlsCard({
  missionPhase,
  isPaused,
  canStart,
  canPause,
  hasValidStartZone = true,
  selectedPattern = 'lawnmower',
  setSelectedPattern,
  startMissionError = null,
  onStart,
  onPause,
  onResume,
}: ControlsCardProps) {
  const isIdle = missionPhase === 'IDLE';
  const isActive =
    missionPhase === 'PLANNING' ||
    missionPhase === 'SEARCHING' ||
    missionPhase === 'TRACKING';

  return (
    <Card className="flex h-full flex-col justify-between p-4">
      <div className="flex items-center justify-between">
        <span className="text-xs font-medium uppercase tracking-wider text-white/50">
          Controls
        </span>
      </div>

      <div className="space-y-2">
        {/* Pattern selector only available pre-mission to prevent mid-flight changes */}
        {isIdle && (
          <div className="space-y-1">
            <div className="flex items-center gap-2">
              <label className="text-[10px] uppercase tracking-wide text-white/50" htmlFor="mission-pattern-inline">
                Pattern
              </label>
              <select
                id="mission-pattern-inline"
                value={selectedPattern}
                onChange={(event) => setSelectedPattern?.(event.target.value as SearchPattern)}
                className="h-7 rounded-md border border-white/15 bg-white/5 px-2 text-xs text-white/90 outline-none"
              >
                <option value="lawnmower">Lawnmower</option>
                <option value="spiral">Spiral</option>
              </select>
            </div>
            {!hasValidStartZone && (
              <p className="text-[10px] text-[var(--warning)]">
                Select an active zone with at least 3 points.
              </p>
            )}
            {startMissionError && (
              <p className="text-[10px] text-[var(--danger)]">{startMissionError}</p>
            )}
          </div>
        )}
        <div className="flex items-stretch gap-3">
        {/* START button - Primary action with shimmer treatment for visibility */}
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

        {/* PAUSE/RESUME button - Mission phase control with state-aware labeling */}
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

        {/* NEW button - Reset for next mission after completion or abort */}
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
      </div>

      {/* Contextual status message based on current phase and state */}
      <div className="text-center">
        <span className="text-[10px] text-white/40">
          {isIdle && 'Ready to start mission'}
          {isActive && !isPaused && 'Mission in progress'}
          {isActive && isPaused && 'Mission paused'}
          {missionPhase === 'COMPLETE' && 'Mission completed'}
          {missionPhase === 'ABORTED' && 'Mission aborted'}
        </span>
      </div>
    </Card>
  );
}
