'use client';

/**
 * AERIS GCS Mission Control Panel
 * 
 * Floating glass panel with mission control buttons (Start, Pause, Resume, Abort).
 * Includes mission phase indicator, timer, and confirmation dialogs for
 * destructive actions.
 */

import React, { useState, useCallback, useEffect } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { GlassPanel } from '@/components/ui/GlassPanel';
import { MissionTimer } from './MissionTimer';
import { useMissionControl } from '@/hooks/useMissionControl';
import { getMissionPhaseConfig, type MissionPhase } from '@/types/mission';
import { transitions, buttonVariants, panelVariants } from '@/lib/animations';
import { cn } from '@/lib/utils';
import {
  Play,
  Pause,
  Square,
  AlertTriangle,
  RotateCcw,
  CheckCircle2,
  Wifi,
  WifiOff,
} from 'lucide-react';

// ============================================================================
// Phase Badge Component
// ============================================================================

interface MissionPhaseBadgeProps {
  phase: MissionPhase;
  className?: string;
}

function MissionPhaseBadge({ phase, className }: MissionPhaseBadgeProps) {
  const config = getMissionPhaseConfig(phase);
  
  return (
    <motion.span
      key={phase}
      initial={{ opacity: 0, scale: 0.9 }}
      animate={{ opacity: 1, scale: 1 }}
      className={cn(
        'inline-flex items-center gap-1.5 px-2.5 py-1 rounded-md',
        'text-xs font-semibold uppercase tracking-wider',
        config.bgColor,
        config.color,
        className
      )}
    >
      {/* Animated dot for active states */}
      {(phase === 'SEARCHING' || phase === 'TRACKING') && (
        <span className="relative flex h-2 w-2">
          <span className="animate-ping absolute inline-flex h-full w-full rounded-full opacity-75 bg-current" />
          <span className="relative inline-flex rounded-full h-2 w-2 bg-current" />
        </span>
      )}
      {phase === 'COMPLETE' && <CheckCircle2 className="w-3 h-3" />}
      {config.label}
    </motion.span>
  );
}

// ============================================================================
// Control Button Component
// ============================================================================

interface ControlButtonProps {
  onClick: () => void;
  disabled?: boolean;
  variant: 'start' | 'pause' | 'resume' | 'abort' | 'abort-confirm';
  children: React.ReactNode;
  className?: string;
}

function ControlButton({
  onClick,
  disabled = false,
  variant,
  children,
  className,
}: ControlButtonProps) {
  const variantStyles = {
    start: 'bg-success/10 hover:bg-success/20 border-success/30 text-success',
    resume: 'bg-success/10 hover:bg-success/20 border-success/30 text-success',
    pause: 'bg-warning/10 hover:bg-warning/20 border-warning/30 text-warning',
    abort: 'bg-danger/10 hover:bg-danger/20 border-danger/30 text-danger',
    'abort-confirm': 'bg-danger/20 hover:bg-danger/30 border-danger/50 text-danger ring-2 ring-danger/50 animate-pulse',
  }[variant];
  
  return (
    <motion.button
      variants={buttonVariants}
      whileHover={disabled ? undefined : 'hover'}
      whileTap={disabled ? undefined : 'tap'}
      onClick={onClick}
      disabled={disabled}
      className={cn(
        'flex items-center justify-center gap-2 px-4 py-3',
        'border font-medium text-sm rounded-lg',
        'transition-colors duration-150',
        'min-h-[var(--touch-min)]',
        'disabled:opacity-40 disabled:cursor-not-allowed',
        variantStyles,
        className
      )}
    >
      {children}
    </motion.button>
  );
}

// ============================================================================
// Main Component
// ============================================================================

interface MissionControlPanelProps {
  /** Custom class name */
  className?: string;
  /** Whether panel is collapsible */
  collapsible?: boolean;
}

export function MissionControlPanel({
  className,
  collapsible = false,
}: MissionControlPanelProps) {
  const {
    phase,
    isPaused,
    canStart,
    canPause,
    canResume,
    canAbort,
    hasValidStartZone,
    selectedPattern,
    setSelectedPattern,
    startMissionError,
    startMission,
    pauseMission,
    resumeMission,
    abortMission,
    rosConnected,
  } = useMissionControl();
  
  // Abort confirmation state
  const [showAbortConfirm, setShowAbortConfirm] = useState(false);
  const [abortCountdown, setAbortCountdown] = useState(0);
  
  // Reset abort confirmation after timeout
  useEffect(() => {
    if (!showAbortConfirm) return;

    const countdownInterval = setInterval(() => {
      setAbortCountdown(prev => {
        if (prev <= 1) {
          setShowAbortConfirm(false);
          return 0;
        }
        return prev - 1;
      });
    }, 1000);
    
    return () => clearInterval(countdownInterval);
  }, [showAbortConfirm]);
  
  // Handle abort with confirmation
  const handleAbort = useCallback(() => {
    if (!showAbortConfirm) {
      setShowAbortConfirm(true);
      setAbortCountdown(5);
    } else {
      abortMission();
      setShowAbortConfirm(false);
    }
  }, [showAbortConfirm, abortMission]);
  
  // Cancel abort confirmation
  const cancelAbort = useCallback(() => {
    setShowAbortConfirm(false);
  }, []);
  
  return (
    <motion.div
      variants={panelVariants}
      initial="hidden"
      animate="visible"
      exit="exit"
      className={className}
    >
      <GlassPanel
        title="Mission Control"
        collapsible={collapsible}
        className="w-[280px] pointer-events-auto"
        headerClassName="pb-2"
        headerRight={
          <div className="flex items-center gap-1.5">
            {rosConnected ? (
              <Wifi className="w-3.5 h-3.5 text-success" />
            ) : (
              <WifiOff className="w-3.5 h-3.5 text-muted-foreground" />
            )}
          </div>
        }
      >
        <div className="space-y-4">
          {/* Pattern Selection */}
          <div className="space-y-2">
            <label
              htmlFor="mission-pattern"
              className="text-xs text-muted-foreground uppercase tracking-wider"
            >
              Pattern
            </label>
            <select
              id="mission-pattern"
              value={selectedPattern}
              onChange={event => setSelectedPattern(event.target.value as 'lawnmower' | 'spiral')}
              className={cn(
                'w-full rounded-lg border border-glass-border bg-surface-1/60',
                'px-3 py-2 text-sm text-foreground outline-none',
                'focus:ring-2 focus:ring-primary/40'
              )}
            >
              <option value="lawnmower">Lawnmower</option>
              <option value="spiral">Spiral</option>
            </select>
            {!hasValidStartZone && (
              <p className="text-xs text-warning">
                Select an active zone with at least 3 points to enable mission start.
              </p>
            )}
            {startMissionError && (
              <p className="text-xs text-danger">{startMissionError}</p>
            )}
          </div>

          {/* Status Section */}
          <div className="space-y-2">
            <div className="flex items-center justify-between">
              <span className="text-xs text-muted-foreground uppercase tracking-wider">
                Status
              </span>
              <MissionPhaseBadge phase={phase} />
            </div>
            
            {/* Timer (only show when mission has started) */}
            {phase !== 'IDLE' && (
              <div className="flex items-center justify-between py-2 px-3 rounded-lg bg-surface-1/50">
                <span className="text-sm text-muted-foreground">Elapsed</span>
                <MissionTimer size="md" />
              </div>
            )}
          </div>
          
          {/* Control Buttons */}
          <div className="space-y-2">
            {/* Start / Resume / Pause row */}
            <div className="flex gap-2">
              <AnimatePresence mode="wait">
                {phase === 'IDLE' && (
                  <motion.div
                    key="start"
                    initial={{ opacity: 0, x: -10 }}
                    animate={{ opacity: 1, x: 0 }}
                    exit={{ opacity: 0, x: -10 }}
                    transition={transitions.fast}
                    className="flex-1"
                  >
                    <ControlButton
                      variant="start"
                      onClick={startMission}
                      disabled={!canStart}
                      className="w-full"
                    >
                      <Play className="w-4 h-4" />
                      Start Mission
                    </ControlButton>
                  </motion.div>
                )}
                
                {canResume && (
                  <motion.div
                    key="resume"
                    initial={{ opacity: 0, x: -10 }}
                    animate={{ opacity: 1, x: 0 }}
                    exit={{ opacity: 0, x: -10 }}
                    transition={transitions.fast}
                    className="flex-1"
                  >
                    <ControlButton
                      variant="resume"
                      onClick={resumeMission}
                      className="w-full"
                    >
                      <RotateCcw className="w-4 h-4" />
                      Resume
                    </ControlButton>
                  </motion.div>
                )}
                
                {canPause && (
                  <motion.div
                    key="pause"
                    initial={{ opacity: 0, x: -10 }}
                    animate={{ opacity: 1, x: 0 }}
                    exit={{ opacity: 0, x: -10 }}
                    transition={transitions.fast}
                    className="flex-1"
                  >
                    <ControlButton
                      variant="pause"
                      onClick={pauseMission}
                      className="w-full"
                    >
                      <Pause className="w-4 h-4" />
                      Pause
                    </ControlButton>
                  </motion.div>
                )}
              </AnimatePresence>
            </div>
            
            {/* Abort button (always visible when mission is active) */}
            <AnimatePresence>
              {canAbort && (
                <motion.div
                  initial={{ opacity: 0, height: 0 }}
                  animate={{ opacity: 1, height: 'auto' }}
                  exit={{ opacity: 0, height: 0 }}
                  transition={transitions.normal}
                >
                  <ControlButton
                    variant={showAbortConfirm ? 'abort-confirm' : 'abort'}
                    onClick={handleAbort}
                    className="w-full"
                  >
                    {showAbortConfirm ? (
                      <>
                        <AlertTriangle className="w-4 h-4" />
                        Confirm Abort ({abortCountdown}s)
                      </>
                    ) : (
                      <>
                        <Square className="w-4 h-4" />
                        Abort Mission
                      </>
                    )}
                  </ControlButton>
                  
                  {/* Cancel abort hint */}
                  {showAbortConfirm && (
                    <motion.button
                      initial={{ opacity: 0 }}
                      animate={{ opacity: 1 }}
                      className="w-full text-xs text-muted-foreground text-center mt-2 hover:text-foreground transition-colors"
                      onClick={cancelAbort}
                    >
                      Click here or wait to cancel
                    </motion.button>
                  )}
                </motion.div>
              )}
            </AnimatePresence>
          </div>
          
          {/* Paused indicator */}
          <AnimatePresence>
            {isPaused && (
              <motion.div
                initial={{ opacity: 0, y: 10 }}
                animate={{ opacity: 1, y: 0 }}
                exit={{ opacity: 0, y: 10 }}
                className="flex items-center justify-center gap-2 py-2 px-3 rounded-lg bg-warning/10 border border-warning/20"
              >
                <Pause className="w-4 h-4 text-warning" />
                <span className="text-sm font-medium text-warning">
                  Mission Paused
                </span>
              </motion.div>
            )}
          </AnimatePresence>
        </div>
      </GlassPanel>
    </motion.div>
  );
}

export default MissionControlPanel;
