'use client';

/**
 * AERIS GCS Mission Context
 * 
 * Global state management for mission control, progress tracking,
 * and statistics. Provides mission lifecycle commands and real-time
 * data aggregation for the dashboard.
 */

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

// ============================================================================
// Context Types
// ============================================================================

interface MissionContextValue {
  // State
  state: MissionState;
  progress: MissionProgress;
  stats: MissionStats;
  
  // Actions
  startMission: (missionId?: string) => void;
  pauseMission: () => void;
  resumeMission: () => void;
  abortMission: () => void;
  completeMission: () => void;
  
  // Progress updates (from ROS or simulation)
  updateProgress: (progress: Partial<MissionProgress>) => void;
  updateStats: (stats: Partial<MissionStats>) => void;
  setPhase: (phase: MissionPhase) => void;
  markExternalUpdate: () => void;
  
  // Command history for debugging
  lastCommand?: MissionCommand;
  commandHistory: Array<{ command: MissionCommand; timestamp: number }>;
}

// ============================================================================
// Context Creation
// ============================================================================

const MissionContext = createContext<MissionContextValue | null>(null);

// ============================================================================
// Storage Keys
// ============================================================================

const STORAGE_KEY_STATE = 'aeris-mission-state';
const STORAGE_KEY_PROGRESS = 'aeris-mission-progress';

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

// ============================================================================
// Provider Component
// ============================================================================

interface MissionProviderProps {
  children: ReactNode;
  /** Enable demo mode with simulated progress */
  demoMode?: boolean;
  /** Initial progress for demo/testing */
  initialProgress?: Partial<MissionProgress>;
}

export function MissionProvider({
  children,
  demoMode = false,
  initialProgress,
}: MissionProviderProps) {
  // Core state
  const [state, setState] = useState<MissionState>(loadInitialMissionState);
  const [progress, setProgress] = useState<MissionProgress>(() =>
    loadInitialMissionProgress(initialProgress)
  );
  const [stats, setStats] = useState<MissionStats>(getInitialMissionStats);
  
  // Command tracking
  const [lastCommand, setLastCommand] = useState<MissionCommand>();
  const [commandHistory, setCommandHistory] = useState<
    Array<{ command: MissionCommand; timestamp: number }>
  >([]);
  const [externalUpdatesActive, setExternalUpdatesActive] = useState(false);
  
  // Demo mode interval ref
  const demoIntervalRef = useRef<NodeJS.Timeout | null>(null);
  const externalUpdateTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  
  // ============================================================================
  // Persistence
  // ============================================================================
  
  // Persist state changes
  useEffect(() => {
    if (typeof window === 'undefined') return;
    
    try {
      localStorage.setItem(STORAGE_KEY_STATE, JSON.stringify(state));
      localStorage.setItem(STORAGE_KEY_PROGRESS, JSON.stringify(progress));
    } catch (error) {
      console.warn('Failed to persist mission state:', error);
    }
  }, [state, progress]);
  
  // ============================================================================
  // Stats Timer
  // ============================================================================
  
  // Update elapsed seconds every second when mission is active
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
  
  // ============================================================================
  // Demo Mode Simulation
  // ============================================================================
  
  useEffect(() => {
    if (!demoMode || externalUpdatesActive || state.phase === 'IDLE' || state.pausedAt) {
      if (demoIntervalRef.current) {
        clearInterval(demoIntervalRef.current);
        demoIntervalRef.current = null;
      }
      return;
    }
    
    // Simulate progress updates in demo mode
    demoIntervalRef.current = setInterval(() => {
      setProgress(prev => {
        const newCoverage = Math.min(100, prev.coveragePercent + 0.5);
        const newCovered = (newCoverage / 100) * prev.searchAreaKm2;
        
        // Calculate ETA based on current rate
        const remainingPercent = 100 - newCoverage;
        const estimatedTimeRemaining = remainingPercent > 0
          ? Math.floor((remainingPercent / 0.5) * 2) // seconds
          : 0;
        
        return {
          ...prev,
          coveragePercent: newCoverage,
          coveredAreaKm2: newCovered,
          estimatedTimeRemaining,
        };
      });
      
      // Random detection simulation
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
  
  // ============================================================================
  // Command Helpers
  // ============================================================================
  
  const recordCommand = useCallback((command: MissionCommand) => {
    setLastCommand(command);
    setCommandHistory(prev => [
      ...prev.slice(-19), // Keep last 20 commands
      { command, timestamp: Date.now() },
    ]);
  }, []);
  
  // ============================================================================
  // Mission Actions
  // ============================================================================
  
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
      // Keep searchAreaKm2, activeDrones, totalDrones from config
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
      missionId: prev.missionId, // Keep for history
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
  
  // ============================================================================
  // Update Functions
  // ============================================================================
  
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

  const markExternalUpdate = useCallback(() => {
    setExternalUpdatesActive(true);
    if (externalUpdateTimeoutRef.current) {
      clearTimeout(externalUpdateTimeoutRef.current);
    }
    // Resume demo mode automatically if external updates stop arriving.
    externalUpdateTimeoutRef.current = setTimeout(() => {
      setExternalUpdatesActive(false);
      externalUpdateTimeoutRef.current = null;
    }, 5000);
  }, []);
  
  // ============================================================================
  // Context Value
  // ============================================================================
  
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

// ============================================================================
// Hook
// ============================================================================

export function useMissionContext(): MissionContextValue {
  const context = useContext(MissionContext);
  
  if (!context) {
    throw new Error('useMissionContext must be used within a MissionProvider');
  }
  
  return context;
}

// ============================================================================
// Selector Hooks
// ============================================================================

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
