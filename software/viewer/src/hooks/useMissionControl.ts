'use client';

/**
 * AERIS GCS Mission Control Hook
 * 
 * Provides mission control capabilities with ROS integration,
 * computed state flags, and real-time detection statistics.
 */

import { useEffect, useMemo, useCallback, useContext } from 'react';
import { useMissionContext } from '@/context/MissionContext';
import { DetectionContext } from '@/context/DetectionContext';
import { useROSConnection } from './useROSConnection';
import type { MissionPhase, MissionCommand } from '@/types/mission';
import ROSLIB from 'roslib';

// ============================================================================
// Hook Return Type
// ============================================================================

export interface MissionControlState {
  // State from context
  phase: MissionPhase;
  isActive: boolean;
  isPaused: boolean;
  isComplete: boolean;
  missionId?: string;
  
  // Progress data
  coveragePercent: number;
  searchAreaKm2: number;
  coveredAreaKm2: number;
  activeDrones: number;
  totalDrones: number;
  estimatedTimeRemaining?: number;
  
  // Statistics
  elapsedSeconds: number;
  detectionCounts: {
    thermal: number;
    acoustic: number;
    gas: number;
    total: number;
  };
  confirmedSurvivors: number;
  pendingDetections: number;
  
  // Computed flags for UI
  canStart: boolean;
  canPause: boolean;
  canResume: boolean;
  canAbort: boolean;
  
  // Actions
  startMission: () => void;
  pauseMission: () => void;
  resumeMission: () => void;
  abortMission: () => void;
  completeMission: () => void;
  
  // ROS status
  rosConnected: boolean;
}

// ============================================================================
// Hook Implementation
// ============================================================================

export function useMissionControl(): MissionControlState {
  const {
    state,
    progress,
    stats,
    startMission: contextStart,
    pauseMission: contextPause,
    resumeMission: contextResume,
    abortMission: contextAbort,
    completeMission: contextComplete,
    updateStats,
  } = useMissionContext();
  
  const { ros, isConnected: rosConnected } = useROSConnection();
  
  // Get detection stats from DetectionContext if available
  let detectionStats = stats.detectionCounts;
  let confirmedCount = stats.confirmedSurvivors;
  let pendingCount = stats.pendingDetections;

  const detectionContext = useContext(DetectionContext);
  if (detectionContext) {
    const detectionContextStats = detectionContext.stats;
    detectionStats = {
      thermal: detectionContextStats.byType.thermal ?? 0,
      acoustic: detectionContextStats.byType.acoustic ?? 0,
      gas: detectionContextStats.byType.gas ?? 0,
      total: detectionContextStats.total,
    };
    confirmedCount = detectionContextStats.byStatus.confirmed ?? 0;
    pendingCount = (detectionContextStats.byStatus.new ?? 0) +
                   (detectionContextStats.byStatus.reviewing ?? 0);
  }
  
  // ============================================================================
  // ROS Integration
  // ============================================================================
  
  // Publish mission commands to ROS
  const publishCommand = useCallback((command: MissionCommand) => {
    if (!ros || !rosConnected) {
      console.warn('[MissionControl] ROS not connected, command not published');
      return;
    }

    const topic = new ROSLIB.Topic({
      ros: ros,
      name: '/mission/command',
      messageType: 'std_msgs/String',
    });

    const message = new ROSLIB.Message({
      data: JSON.stringify({
        command,
        timestamp: Date.now(),
        missionId: state.missionId,
      }),
    });

    topic.publish(message);
  }, [ros, rosConnected, state.missionId]);
  
  // Subscribe to mission state updates from ROS
  useEffect(() => {
    if (!ros || !rosConnected) return;
    
    const stateTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/mission/state',
      messageType: 'std_msgs/String',
    });
    
    const progressTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/mission/progress',
      messageType: 'std_msgs/String',
    });
    
    const handleStateMessage = (message: ROSLIB.Message) => {
      try {
        const data = JSON.parse((message as { data: string }).data);
        // Could update phase from external source
        void data;
      } catch (error) {
        console.warn('[MissionControl] Failed to parse state message:', error);
      }
    };

    const handleProgressMessage = (message: ROSLIB.Message) => {
      try {
        const data = JSON.parse((message as { data: string }).data);
        // Could update progress from external source
        void data;
      } catch (error) {
        console.warn('[MissionControl] Failed to parse progress message:', error);
      }
    };
    
    stateTopic.subscribe(handleStateMessage);
    progressTopic.subscribe(handleProgressMessage);
    
    return () => {
      stateTopic.unsubscribe();
      progressTopic.unsubscribe();
    };
  }, [ros, rosConnected]);
  
  // ============================================================================
  // Actions with ROS publishing
  // ============================================================================
  
  const startMission = useCallback(() => {
    contextStart();
    publishCommand('START');
  }, [contextStart, publishCommand]);
  
  const pauseMission = useCallback(() => {
    contextPause();
    publishCommand('PAUSE');
  }, [contextPause, publishCommand]);
  
  const resumeMission = useCallback(() => {
    contextResume();
    publishCommand('RESUME');
  }, [contextResume, publishCommand]);
  
  const abortMission = useCallback(() => {
    contextAbort();
    publishCommand('ABORT');
  }, [contextAbort, publishCommand]);
  
  const completeMission = useCallback(() => {
    contextComplete();
    publishCommand('COMPLETE');
  }, [contextComplete, publishCommand]);
  
  // ============================================================================
  // Sync detection stats to mission stats
  // ============================================================================
  
  useEffect(() => {
    updateStats({
      detectionCounts: detectionStats,
      confirmedSurvivors: confirmedCount,
      pendingDetections: pendingCount,
    });
  }, [detectionStats, confirmedCount, pendingCount, updateStats]);
  
  // ============================================================================
  // Computed State
  // ============================================================================
  
  const computedState = useMemo(() => {
    const isActive = state.phase === 'SEARCHING' || state.phase === 'TRACKING';
    const isPaused = state.pausedAt !== undefined;
    const isComplete = state.phase === 'COMPLETE';
    const isIdle = state.phase === 'IDLE';
    
    return {
      phase: state.phase,
      isActive,
      isPaused,
      isComplete,
      missionId: state.missionId,
      
      // Control flags
      canStart: isIdle && !isPaused,
      canPause: isActive && !isPaused,
      canResume: isPaused,
      canAbort: isActive || isPaused,
    };
  }, [state.phase, state.pausedAt, state.missionId]);
  
  // ============================================================================
  // Return Value
  // ============================================================================
  
  return {
    // From computed state
    ...computedState,
    
    // Progress
    coveragePercent: progress.coveragePercent,
    searchAreaKm2: progress.searchAreaKm2,
    coveredAreaKm2: progress.coveredAreaKm2,
    activeDrones: progress.activeDrones,
    totalDrones: progress.totalDrones,
    estimatedTimeRemaining: progress.estimatedTimeRemaining,
    
    // Stats
    elapsedSeconds: stats.elapsedSeconds,
    detectionCounts: detectionStats,
    confirmedSurvivors: confirmedCount,
    pendingDetections: pendingCount,
    
    // Actions
    startMission,
    pauseMission,
    resumeMission,
    abortMission,
    completeMission,
    
    // ROS status
    rosConnected,
  };
}

// ============================================================================
// Selector Hooks
// ============================================================================

/**
 * Get just the mission phase
 */
export function useMissionPhase(): MissionPhase {
  const { phase } = useMissionControl();
  return phase;
}

/**
 * Get just the coverage percentage
 */
export function useMissionCoverage(): number {
  const { coveragePercent } = useMissionControl();
  return coveragePercent;
}

/**
 * Check if mission can be controlled (not complete)
 */
export function useCanControlMission(): boolean {
  const { phase } = useMissionControl();
  return phase !== 'COMPLETE';
}
