/**
 * Mission lifecycle phases for search and rescue operations.
 *
 * State machine transitions:
 * IDLE -> PLANNING -> SEARCHING <-> TRACKING -> COMPLETE
n * Any state can transition to ABORTED.
 */
export type MissionPhase =
  | 'IDLE'
  | 'PLANNING'
  | 'SEARCHING'
  | 'TRACKING'
  | 'COMPLETE'
  | 'ABORTED';

/**
 * Core mission state with timing tracking.
 *
 * totalPausedTime accumulates all paused durations to calculate
 * actual mission elapsed time excluding pauses.
 */
export interface MissionState {
  phase: MissionPhase;
  startTime?: number;
  endTime?: number;
  pausedAt?: number;
  totalPausedTime: number;
  missionId?: string;
}

/**
 * Progress metrics for ongoing search operations.
 *
 * Grid progress tracks the search pattern completion when using
 * systematic grid-based search algorithms.
 */
export interface MissionProgress {
  coveragePercent: number;
  searchAreaKm2: number;
  coveredAreaKm2: number;
  activeDrones: number;
  totalDrones: number;
  estimatedTimeRemaining?: number;
  gridProgress?: {
    completed: number;
    total: number;
  };
}

/**
 * Aggregated statistics for mission reporting and analytics.
 *
 * detectionRate is detections per hour. timeToFirstDetection
 * measures search efficiency from mission start.
 */
export interface MissionStats {
  elapsedSeconds: number;
  detectionCounts: {
    thermal: number;
    acoustic: number;
    gas: number;
    total: number;
  };
  confirmedSurvivors: number;
  dismissedDetections: number;
  pendingDetections: number;
  timeToFirstDetection?: number;
  detectionRate?: number;
}

/**
 * High-level mission commands for state machine control.
 */
export type MissionCommand =
  | 'START'
  | 'PAUSE'
  | 'RESUME'
  | 'ABORT'
  | 'COMPLETE';

/**
 * Mission command message for ROS publishing.
 */
export interface MissionCommandMessage {
  command: MissionCommand;
  timestamp: number;
  missionId?: string;
  metadata?: Record<string, unknown>;
}

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Format seconds into MM:SS format for timer display.
 */
export function formatMissionTime(seconds: number): string {
  const absSeconds = Math.abs(Math.floor(seconds));
  const mins = Math.floor(absSeconds / 60);
  const secs = absSeconds % 60;
  return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
}

/**
 * Format seconds into human-readable duration (e.g., "5m", "1h 30m").
 */
export function formatDuration(seconds: number): string {
  const absSeconds = Math.abs(Math.floor(seconds));

  if (absSeconds < 60) {
    return `${absSeconds}s`;
  }

  const mins = Math.floor(absSeconds / 60);
  if (mins < 60) {
    return `${mins}m`;
  }

  const hours = Math.floor(mins / 60);
  const remainingMins = mins % 60;

  if (remainingMins === 0) {
    return `${hours}h`;
  }

  return `${hours}h ${remainingMins}m`;
}

/**
 * Format area with automatic unit selection:
 * - <0.01 km²: display in m²
 * - <1 km²: display in hectares
 * - >=1 km²: display in km²
 */
export function formatArea(km2: number): string {
  if (km2 < 0.01) {
    const m2 = km2 * 1_000_000;
    return `${m2.toFixed(0)} m²`;
  }
  if (km2 < 1) {
    return `${(km2 * 1000).toFixed(1)} ha`;
  }
  return `${km2.toFixed(2)} km²`;
}

export function generateMissionId(): string {
  const timestamp = Date.now().toString(36);
  const random = Math.random().toString(36).substring(2, 6);
  return `MSN-${timestamp}-${random}`.toUpperCase();
}

/**
 * Get UI configuration for a mission phase.
 */
export function getMissionPhaseConfig(phase: MissionPhase): {
  label: string;
  color: string;
  bgColor: string;
  icon: 'idle' | 'planning' | 'searching' | 'tracking' | 'complete' | 'aborted';
} {
  switch (phase) {
    case 'IDLE':
      return {
        label: 'Idle',
        color: 'text-muted-foreground',
        bgColor: 'bg-surface-3',
        icon: 'idle',
      };
    case 'PLANNING':
      return {
        label: 'Planning',
        color: 'text-primary',
        bgColor: 'bg-primary/10',
        icon: 'planning',
      };
    case 'SEARCHING':
      return {
        label: 'Searching',
        color: 'text-info',
        bgColor: 'bg-info/10',
        icon: 'searching',
      };
    case 'TRACKING':
      return {
        label: 'Tracking',
        color: 'text-warning',
        bgColor: 'bg-warning/10',
        icon: 'tracking',
      };
    case 'COMPLETE':
      return {
        label: 'Complete',
        color: 'text-success',
        bgColor: 'bg-success/10',
        icon: 'complete',
      };
    case 'ABORTED':
      return {
        label: 'Aborted',
        color: 'text-danger',
        bgColor: 'bg-danger/10',
        icon: 'aborted',
      };
  }
}

/**
 * Calculate elapsed mission time in seconds, accounting for pauses.
 *
 * Returns 0 if mission hasn't started (IDLE phase or no startTime).
 * Uses endTime if mission is complete, otherwise uses current time.
 */
export function calculateElapsedSeconds(state: MissionState): number {
  if (state.phase === 'IDLE' || !state.startTime) {
    return 0;
  }

  const now = state.endTime ?? Date.now();
  const endPoint = state.pausedAt ?? now;
  const elapsed = endPoint - state.startTime - state.totalPausedTime;

  return Math.max(0, Math.floor(elapsed / 1000));
}

export function getInitialMissionState(): MissionState {
  return {
    phase: 'IDLE',
    totalPausedTime: 0,
  };
}

export function getInitialMissionProgress(): MissionProgress {
  return {
    coveragePercent: 0,
    searchAreaKm2: 0,
    coveredAreaKm2: 0,
    activeDrones: 0,
    totalDrones: 0,
  };
}

export function getInitialMissionStats(): MissionStats {
  return {
    elapsedSeconds: 0,
    detectionCounts: {
      thermal: 0,
      acoustic: 0,
      gas: 0,
      total: 0,
    },
    confirmedSurvivors: 0,
    dismissedDetections: 0,
    pendingDetections: 0,
  };
}
