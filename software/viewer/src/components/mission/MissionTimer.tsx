'use client';

/**
 * AERIS GCS Mission Timer
 * 
 * Real-time mission elapsed time display with pause awareness.
 * Updates every second when mission is active, freezes when paused.
 */

import React, { useEffect, useState } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { useMissionContext } from '@/context/MissionContext';
import { formatMissionTime, calculateElapsedSeconds } from '@/types/mission';
import { transitions } from '@/lib/animations';
import { cn } from '@/lib/utils';
import { Clock, Pause } from 'lucide-react';

// ============================================================================
// Types
// ============================================================================

interface MissionTimerProps {
  /** Show icon */
  showIcon?: boolean;
  /** Size variant */
  size?: 'sm' | 'md' | 'lg';
  /** Custom class name */
  className?: string;
}

// ============================================================================
// Component
// ============================================================================

export function MissionTimer({
  showIcon = true,
  size = 'md',
  className,
}: MissionTimerProps) {
  const { state } = useMissionContext();
  const [elapsedSeconds, setElapsedSeconds] = useState(0);
  
  // Calculate and update elapsed time
  useEffect(() => {
    // If mission hasn't started, show 00:00
    if (state.phase === 'IDLE' || !state.startTime) {
      setElapsedSeconds(0);
      return;
    }
    
    // Calculate current elapsed time
    const calculateElapsed = () => {
      return calculateElapsedSeconds(state);
    };
    
    // Set initial value
    setElapsedSeconds(calculateElapsed());
    
    // If paused, don't update
    if (state.pausedAt) {
      return;
    }
    
    // Update every second while active
    const interval = setInterval(() => {
      setElapsedSeconds(calculateElapsed());
    }, 1000);
    
    return () => clearInterval(interval);
  }, [state]);
  
  // Size configurations
  const sizeConfig = {
    sm: {
      icon: 'w-3 h-3',
      text: 'text-xs',
      gap: 'gap-1',
    },
    md: {
      icon: 'w-4 h-4',
      text: 'text-sm',
      gap: 'gap-1.5',
    },
    lg: {
      icon: 'w-5 h-5',
      text: 'text-lg',
      gap: 'gap-2',
    },
  }[size];
  
  const isPaused = state.pausedAt !== undefined;
  const isActive = state.phase === 'SEARCHING' || state.phase === 'TRACKING';
  
  return (
    <div
      className={cn(
        'flex items-center',
        sizeConfig.gap,
        className
      )}
    >
      {showIcon && (
        <AnimatePresence mode="wait">
          {isPaused ? (
            <motion.div
              key="paused"
              initial={{ opacity: 0, scale: 0.8 }}
              animate={{ opacity: 1, scale: 1 }}
              exit={{ opacity: 0, scale: 0.8 }}
              transition={transitions.fast}
            >
              <Pause className={cn(sizeConfig.icon, 'text-warning')} />
            </motion.div>
          ) : (
            <motion.div
              key="clock"
              initial={{ opacity: 0, scale: 0.8 }}
              animate={{ opacity: 1, scale: 1 }}
              exit={{ opacity: 0, scale: 0.8 }}
              transition={transitions.fast}
            >
              <Clock
                className={cn(
                  sizeConfig.icon,
                  isActive ? 'text-info' : 'text-muted-foreground'
                )}
              />
            </motion.div>
          )}
        </AnimatePresence>
      )}
      
      <span
        className={cn(
          'font-mono font-semibold tabular-nums',
          sizeConfig.text,
          isPaused && 'text-warning animate-pulse',
          !isPaused && isActive && 'text-foreground',
          !isPaused && !isActive && 'text-muted-foreground'
        )}
      >
        {formatMissionTime(elapsedSeconds)}
      </span>
    </div>
  );
}

// ============================================================================
// Compact Timer (for StatusBar)
// ============================================================================

interface CompactTimerProps {
  className?: string;
}

export function CompactMissionTimer({ className }: CompactTimerProps) {
  return (
    <MissionTimer
      size="sm"
      showIcon={false}
      className={className}
    />
  );
}

export default MissionTimer;
