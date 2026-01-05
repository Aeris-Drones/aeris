/**
 * AERIS GCS Mission Types
 * 
 * Core types for mission state management, progress tracking,
 * and statistics for disaster response drone swarm operations.
 */

// ============================================================================
// Mission Phase & State
// ============================================================================

/**
 * Mission lifecycle phases
 */
export type MissionPhase = 'IDLE' | 'SEARCHING' | 'TRACKING' | 'COMPLETE';

/**
 * Core mission state for persistence and control
 */
export interface MissionState {
  /** Current phase of the mission */
  phase: MissionPhase;
  /** Unix timestamp when mission started */
  startTime?: number;
  /** Unix timestamp when mission ended */
  endTime?: number;
  /** Unix timestamp when mission was paused (undefined if not paused) */
  pausedAt?: number;
  /** Total time spent paused in milliseconds */
  totalPausedTime: number;
  /** Unique mission ID for tracking */
  missionId?: string;
}

/**
 * Real-time mission progress data
 */
export interface MissionProgress {
  /** Coverage percentage (0-100) */
  coveragePercent: number;
  /** Total search area in square kilometers */
  searchAreaKm2: number;
  /** Covered area in square kilometers */
  coveredAreaKm2: number;
  /** Number of active drones in the swarm */
  activeDrones: number;
  /** Total drones assigned to mission */
  totalDrones: number;
  /** Estimated time remaining in seconds */
  estimatedTimeRemaining?: number;
  /** Grid cells completed / total */
  gridProgress?: {
    completed: number;
    total: number;
  };
}

/**
 * Aggregated mission statistics
 */
export interface MissionStats {
  /** Total elapsed time in seconds (excluding paused time) */
  elapsedSeconds: number;
  /** Detection counts by sensor type */
  detectionCounts: {
    thermal: number;
    acoustic: number;
    gas: number;
    total: number;
  };
  /** Number of confirmed survivors */
  confirmedSurvivors: number;
  /** Number of dismissed false positives */
  dismissedDetections: number;
  /** Number of pending detections awaiting review */
  pendingDetections: number;
  /** Average time to first detection in seconds */
  timeToFirstDetection?: number;
  /** Detection rate (detections per minute) */
  detectionRate?: number;
}

// ============================================================================
// Mission Commands
// ============================================================================

/**
 * Commands that can be issued to the mission control system
 */
export type MissionCommand = 
  | 'START'
  | 'PAUSE'
  | 'RESUME'
  | 'ABORT'
  | 'COMPLETE';

/**
 * Mission command message for ROS publishing
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
 * Format seconds into MM:SS format for timer display
 */
export function formatMissionTime(seconds: number): string {
  const absSeconds = Math.abs(Math.floor(seconds));
  const mins = Math.floor(absSeconds / 60);
  const secs = absSeconds % 60;
  return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
}

/**
 * Format seconds into human-readable duration (e.g., "5m", "1h 30m")
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
 * Format area with appropriate units
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

/**
 * Generate a unique mission ID
 */
export function generateMissionId(): string {
  const timestamp = Date.now().toString(36);
  const random = Math.random().toString(36).substring(2, 6);
  return `MSN-${timestamp}-${random}`.toUpperCase();
}

/**
 * Get phase display configuration
 */
export function getMissionPhaseConfig(phase: MissionPhase): {
  label: string;
  color: string;
  bgColor: string;
  icon: 'idle' | 'searching' | 'tracking' | 'complete';
} {
  switch (phase) {
    case 'IDLE':
      return {
        label: 'Idle',
        color: 'text-muted-foreground',
        bgColor: 'bg-surface-3',
        icon: 'idle',
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
  }
}

/**
 * Calculate elapsed seconds from mission state
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

/**
 * Default initial mission state
 */
export function getInitialMissionState(): MissionState {
  return {
    phase: 'IDLE',
    totalPausedTime: 0,
  };
}

/**
 * Default initial mission progress
 */
export function getInitialMissionProgress(): MissionProgress {
  return {
    coveragePercent: 0,
    searchAreaKm2: 0,
    coveredAreaKm2: 0,
    activeDrones: 0,
    totalDrones: 0,
  };
}

/**
 * Default initial mission stats
 */
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
