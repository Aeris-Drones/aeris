'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { AlertTriangle } from 'lucide-react';
import { Progress } from '@/components/ui/progress';
import { Button } from '@/components/ui/button';
import { cn } from '@/lib/utils';
import type { MissionPhase } from '@/components/layout/StatusPill';
import {
  advanceEmergencyStopHold,
  beginEmergencyStopHold,
  cancelEmergencyStopHoldState,
  createIdleEmergencyStopHold,
  isAbortRequestPending,
} from '@/lib/emergencyStopHold';

interface EmergencyStopControlProps {
  missionPhase: MissionPhase;
  canAbort: boolean;
  abortUnavailableReason: string | null;
  abortError?: string | null;
  onAbort: () => void;
}

export function EmergencyStopControl({
  missionPhase,
  canAbort,
  abortUnavailableReason,
  abortError = null,
  onAbort,
}: EmergencyStopControlProps) {
  const idleHoldState = useMemo(() => createIdleEmergencyStopHold(), []);
  const [holdState, setHoldState] = useState(createIdleEmergencyStopHold);
  const [abortRequested, setAbortRequested] = useState(false);
  const [abortDispatchNonce, setAbortDispatchNonce] = useState(0);
  const holdStateRef = useRef(holdState);
  const abortPending = isAbortRequestPending({
    abortRequested,
    abortError,
    missionPhase,
  });
  const displayHoldState =
    canAbort && !abortPending ? holdState : idleHoldState;

  const cancelHold = useCallback(() => {
    const nextState = cancelEmergencyStopHoldState(holdStateRef.current);
    holdStateRef.current = nextState;
    setHoldState(nextState);
  }, []);

  useEffect(() => {
    holdStateRef.current = holdState;
  }, [holdState]);

  useEffect(() => {
    if (holdState.phase !== 'holding') {
      return;
    }

    const tick = () => {
      const { nextState, shouldDispatchAbort } = advanceEmergencyStopHold({
        state: holdStateRef.current,
        now: Date.now(),
        canAbort,
        abortPending,
      });
      holdStateRef.current = nextState;
      setHoldState(nextState);

      if (shouldDispatchAbort) {
        setAbortRequested(true);
        setAbortDispatchNonce((value) => value + 1);
      }
    };

    tick();
    const intervalId = window.setInterval(tick, 50);
    return () => window.clearInterval(intervalId);
  }, [abortPending, canAbort, holdState.phase]);

  useEffect(() => {
    if (abortDispatchNonce === 0) {
      return;
    }

    onAbort();
  }, [abortDispatchNonce, onAbort]);

  useEffect(() => {
    if (holdState.phase !== 'holding') {
      return;
    }

    const handleWindowBlur = () => cancelHold();
    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.key === 'Escape') {
        cancelHold();
      }
    };

    window.addEventListener('blur', handleWindowBlur);
    window.addEventListener('keydown', handleKeyDown);
    return () => {
      window.removeEventListener('blur', handleWindowBlur);
      window.removeEventListener('keydown', handleKeyDown);
    };
  }, [cancelHold, holdState.phase]);

  const helperText = useMemo(() => {
    if (abortPending) {
      return 'Abort requested. Awaiting mission-state update.';
    }
    if (abortError) {
      return abortError;
    }
    if (!canAbort) {
      return abortUnavailableReason ?? 'Emergency stop is currently unavailable.';
    }
    if (displayHoldState.phase === 'holding') {
      return 'Keep holding until the progress bar fills.';
    }
    return 'Press and hold to dispatch the fleet abort command.';
  }, [abortError, abortPending, abortUnavailableReason, canAbort, displayHoldState.phase]);

  const buttonLabel = abortPending
    ? 'ABORT REQUESTED'
    : displayHoldState.phase === 'holding'
      ? 'KEEP HOLDING'
      : 'EMERGENCY STOP';

  return (
    <div className="pointer-events-auto flex w-[248px] flex-col gap-2 rounded-lg border border-[var(--danger)]/60 bg-black/80 p-3 text-white shadow-[0_0_30px_rgba(0,0,0,0.45)]">
      <div className="flex items-center gap-2 text-[11px] font-semibold uppercase tracking-[0.24em] text-[var(--danger)]">
        <AlertTriangle className="h-4 w-4" />
        <span>Operator only</span>
      </div>

      <Button
        type="button"
        variant="outline"
        className={cn(
          'relative h-16 w-full overflow-hidden border-[var(--danger)] bg-[var(--danger)]/12 px-4 text-left text-white hover:bg-[var(--danger)]/18 hover:text-white',
          'focus-visible:ring-[var(--danger)]/80',
          (canAbort && !abortPending) || 'cursor-not-allowed opacity-70'
        )}
        disabled={!canAbort || abortPending}
        aria-label="Press and hold emergency stop"
        onPointerDown={(event) => {
          if (event.pointerType === 'mouse' && event.button !== 0) {
            return;
          }
          if (!canAbort || abortPending) {
            return;
          }
          setAbortRequested(false);
          setHoldState(beginEmergencyStopHold(Date.now()));
        }}
        onPointerUp={cancelHold}
        onPointerLeave={cancelHold}
        onPointerCancel={cancelHold}
        onBlur={cancelHold}
      >
        <div className="absolute inset-x-0 bottom-0 px-4 pb-2">
          <Progress
            value={displayHoldState.progress * 100}
            className="h-1.5 bg-white/10"
            indicatorColor="danger"
            aria-hidden="true"
          />
        </div>

        <div className="flex items-center gap-3">
          <div className="flex h-10 w-10 items-center justify-center rounded-md border border-[var(--danger)]/50 bg-[var(--danger)]/18">
            <AlertTriangle className="h-5 w-5" />
          </div>
          <div className="flex flex-col">
            <span className="text-base font-semibold tracking-wide">{buttonLabel}</span>
            <span className="text-xs uppercase tracking-[0.2em] text-white/70">
              Press and hold
            </span>
          </div>
        </div>
      </Button>

      <p className="min-h-[2.5rem] text-sm leading-5 text-white/85">{helperText}</p>
    </div>
  );
}
