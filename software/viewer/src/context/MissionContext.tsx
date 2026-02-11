'use client';

import React, {
  createContext,
  useContext,
  useState,
  useCallback,
  useEffect,
  useRef,
  type ReactNode,
} from 'react';
import type {
  MissionPhase,
  MissionState,
  MissionProgress,
  MissionStats,
  MissionCommand,
} from '@/types/mission';
import {
  getInitialMissionState,
  getInitialMissionProgress,
  getInitialMissionStats,
  generateMissionId,
  calculateElapsedSeconds,
} from '@/types/mission';

interface MissionContextValue {
  state: MissionState;
  progress: MissionProgress;
  stats: MissionStats;
  startMission: (missionId?: string) => void;
  pauseMission: () => void;
  resumeMission: () => void;
  abortMission: () => void;
  completeMission: () => void;
  updateProgress: (progress: Partial<MissionProgress>) => void;
  updateStats: (stats: Partial<MissionStats>) => void;
  setPhase: (phase: MissionPhase) => void;
  markExternalUpdate: () => void;
  lastCommand?: MissionCommand;
  commandHistory: Array<{ command: MissionCommand; timestamp: number }>;
}

const MissionContext = createContext<MissionContextValue | null>(null);

const STORAGE_KEY_STATE = 'aeris-mission-state';
const STORAGE_KEY_PROGRESS = 'aeris-mission-progress';

/**
 * Restore mission state from localStorage on initial load.
 * Only resumes active missions (non-IDLE phases) to prevent
 * accidental restarts of stale sessions after browser refresh.
 */
function loadInitialMissionState(): MissionState {
  const initial = getInitialMissionState();
  if (typeof window === 'undefined') return initial;
  try {
    const savedState = localStorage.getItem(STORAGE_KEY_STATE);
    if (!savedState) return initial;
    const parsed = JSON.parse(savedState) as MissionState;
    if (parsed.phase !== 'IDLE' && parsed.startTime) return parsed;
  } catch (error) {
    console.warn('Failed to restore mission state:', error);
  }
  return initial;
}

/**
 * Restore mission progress metrics from localStorage.
 * Merges with defaults to handle schema evolution between deployments.
 */
function loadInitialMissionProgress(initialProgress?: Partial<MissionProgress>): MissionProgress {
  const initial = { ...getInitialMissionProgress(), ...initialProgress };
  if (typeof window === 'undefined') return initial;
  try {
    const savedProgress = localStorage.getItem(STORAGE_KEY_PROGRESS);
    if (!savedProgress) return initial;
    return {
      ...initial,
      ...(JSON.parse(savedProgress) as Partial<MissionProgress>),
    };
  } catch (error) {
    console.warn('Failed to restore mission progress:', error);
    return initial;
  }
}

interface MissionProviderProps {
  children: ReactNode;
  demoMode?: boolean;
  initialProgress?: Partial<MissionProgress>;
}

/**
 * Provides mission lifecycle state management and persistence.
 *
 * State persistence strategy:
 * - State and progress are persisted to localStorage on every change
 * - Only non-IDLE missions are restored on page load
 * - Stats (transient counters) are not persisted
 *
 * Demo mode simulates mission progress when ROS is unavailable,
 * generating synthetic coverage updates and random detections.
 * External updates (from ROS) take precedence and disable demo mode
 * for 5 seconds via the markExternalUpdate mechanism.
 */
export function MissionProvider({
  children,
  demoMode = false,
  initialProgress,
}: MissionProviderProps) {
  const [state, setState] = useState<MissionState>(loadInitialMissionState);
  const [progress, setProgress] = useState<MissionProgress>(() =>
    loadInitialMissionProgress(initialProgress)
  );
  const [stats, setStats] = useState<MissionStats>(getInitialMissionStats);
  const [lastCommand, setLastCommand] = useState<MissionCommand>();
  const [commandHistory, setCommandHistory] = useState<
    Array<{ command: MissionCommand; timestamp: number }>
  >([]);
  const [externalUpdatesActive, setExternalUpdatesActive] = useState(false);
  const demoIntervalRef = useRef<NodeJS.Timeout | null>(null);
  const externalUpdateTimeoutRef = useRef<NodeJS.Timeout | null>(null);

  // Persist state and progress to localStorage for crash recovery.
  useEffect(() => {
    if (typeof window === 'undefined') return;

    try {
      localStorage.setItem(STORAGE_KEY_STATE, JSON.stringify(state));
      localStorage.setItem(STORAGE_KEY_PROGRESS, JSON.stringify(progress));
    } catch (error) {
      console.warn('Failed to persist mission state:', error);
    }
  }, [state, progress]);

  // Update elapsed time every second while mission is active and not paused.
  useEffect(() => {
    if (state.phase === 'IDLE' || state.pausedAt) {
      return;
    }

    const interval = setInterval(() => {
      setStats(prev => ({
        ...prev,
        elapsedSeconds: calculateElapsedSeconds(state),
      }));
    }, 1000);

    return () => clearInterval(interval);
  }, [state]);

  // Demo mode: simulate mission progress and detections when no ROS connection.
  // Disabled when external updates are active to prevent state conflicts.
  useEffect(() => {
    if (!demoMode || externalUpdatesActive || state.phase === 'IDLE' || state.pausedAt) {
      if (demoIntervalRef.current) {
        clearInterval(demoIntervalRef.current);
        demoIntervalRef.current = null;
      }
      return;
    }

    demoIntervalRef.current = setInterval(() => {
      setProgress(prev => {
        const newCoverage = Math.min(100, prev.coveragePercent + 0.5);
        const newCovered = (newCoverage / 100) * prev.searchAreaKm2;
        const remainingPercent = 100 - newCoverage;
        const estimatedTimeRemaining = remainingPercent > 0
          ? Math.floor((remainingPercent / 0.5) * 2)
          : 0;

        return {
          ...prev,
          coveragePercent: newCoverage,
          coveredAreaKm2: newCovered,
          estimatedTimeRemaining,
        };
      });

      // 5% chance per tick to generate a synthetic detection
      if (Math.random() < 0.05) {
        const sensorTypes = ['thermal', 'acoustic', 'gas'] as const;
        const sensor = sensorTypes[Math.floor(Math.random() * 3)];

        setStats(prev => ({
          ...prev,
          detectionCounts: {
            ...prev.detectionCounts,
            [sensor]: prev.detectionCounts[sensor] + 1,
            total: prev.detectionCounts.total + 1,
          },
          pendingDetections: prev.pendingDetections + 1,
        }));
      }
    }, 2000);

    return () => {
      if (demoIntervalRef.current) {
        clearInterval(demoIntervalRef.current);
        demoIntervalRef.current = null;
      }
    };
  }, [demoMode, externalUpdatesActive, state.phase, state.pausedAt]);

  useEffect(() => {
    return () => {
      if (externalUpdateTimeoutRef.current) {
        clearTimeout(externalUpdateTimeoutRef.current);
        externalUpdateTimeoutRef.current = null;
      }
    };
  }, []);

  // Maintain rolling command history (last 20 commands) for debugging.
  const recordCommand = useCallback((command: MissionCommand) => {
    setLastCommand(command);
    setCommandHistory(prev => [
      ...prev.slice(-19),
      { command, timestamp: Date.now() },
    ]);
  }, []);

  const startMission = useCallback((missionIdOverride?: string) => {
    const missionId = missionIdOverride?.trim() || generateMissionId();

    setState({
      phase: 'SEARCHING',
      startTime: Date.now(),
      totalPausedTime: 0,
      missionId,
    });

    setProgress(prev => ({
      ...prev,
      coveragePercent: 0,
      coveredAreaKm2: 0,
    }));

    setStats({
      ...getInitialMissionStats(),
      elapsedSeconds: 0,
    });

    recordCommand('START');
  }, [recordCommand]);

  const pauseMission = useCallback(() => {
    if (state.phase === 'IDLE' || state.pausedAt) return;

    setState(prev => ({
      ...prev,
      pausedAt: Date.now(),
    }));

    recordCommand('PAUSE');
  }, [state.phase, state.pausedAt, recordCommand]);

  const resumeMission = useCallback(() => {
    if (!state.pausedAt) return;

    const pauseDuration = Date.now() - state.pausedAt;

    setState(prev => ({
      ...prev,
      pausedAt: undefined,
      totalPausedTime: prev.totalPausedTime + pauseDuration,
    }));

    recordCommand('RESUME');
  }, [state.pausedAt, recordCommand]);

  const abortMission = useCallback(() => {
    setState(prev => ({
      ...getInitialMissionState(),
      missionId: prev.missionId,
    }));

    recordCommand('ABORT');
  }, [recordCommand]);

  const completeMission = useCallback(() => {
    setState(prev => ({
      ...prev,
      phase: 'COMPLETE',
      endTime: Date.now(),
      pausedAt: undefined,
    }));

    recordCommand('COMPLETE');
  }, [recordCommand]);

  const updateProgress = useCallback((updates: Partial<MissionProgress>) => {
    setProgress(prev => ({
      ...prev,
      ...updates,
    }));
  }, []);

  const updateStats = useCallback((updates: Partial<MissionStats>) => {
    setStats(prev => ({
      ...prev,
      ...updates,
    }));
  }, []);

  const setPhase = useCallback((phase: MissionPhase) => {
    setState(prev => ({
      ...prev,
      phase,
      pausedAt:
        phase === 'SEARCHING' || phase === 'TRACKING'
          ? prev.pausedAt
          : undefined,
    }));
  }, []);

  /**
   * Signals that ROS has provided external state updates.
   * Temporarily disables demo mode for 5 seconds to prevent
   * conflicting synthetic updates from overwriting real telemetry.
   */
  const markExternalUpdate = useCallback(() => {
    setExternalUpdatesActive(true);
    if (externalUpdateTimeoutRef.current) {
      clearTimeout(externalUpdateTimeoutRef.current);
    }
    externalUpdateTimeoutRef.current = setTimeout(() => {
      setExternalUpdatesActive(false);
      externalUpdateTimeoutRef.current = null;
    }, 5000);
  }, []);

  const value: MissionContextValue = {
    state,
    progress,
    stats,
    startMission,
    pauseMission,
    resumeMission,
    abortMission,
    completeMission,
    updateProgress,
    updateStats,
    setPhase,
    markExternalUpdate,
    lastCommand,
    commandHistory,
  };

  return (
    <MissionContext.Provider value={value}>
      {children}
    </MissionContext.Provider>
  );
}

export function useMissionContext(): MissionContextValue {
  const context = useContext(MissionContext);

  if (!context) {
    throw new Error('useMissionContext must be used within a MissionProvider');
  }

  return context;
}

export function useMissionState(): MissionState {
  return useMissionContext().state;
}

export function useMissionProgress(): MissionProgress {
  return useMissionContext().progress;
}

export function useMissionStats(): MissionStats {
  return useMissionContext().stats;
}

export function useMissionPhase(): MissionPhase {
  return useMissionContext().state.phase;
}

export function useIsMissionActive(): boolean {
  const phase = useMissionPhase();
  return phase === 'SEARCHING' || phase === 'TRACKING';
}

export function useIsMissionPaused(): boolean {
  const state = useMissionState();
  return state.pausedAt !== undefined;
}
