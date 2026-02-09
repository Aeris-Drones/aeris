'use client';

/**
 * AERIS GCS Mission Control Hook
 * 
 * Provides mission control capabilities with ROS integration,
 * computed state flags, and real-time detection statistics.
 */

import { useEffect, useMemo, useCallback, useContext, useState } from 'react';
import { useMissionContext } from '@/context/MissionContext';
import { DetectionContext } from '@/context/DetectionContext';
import { useZoneContext } from '@/context/ZoneContext';
import { useROSConnection } from './useROSConnection';
import {
  generateMissionId,
  type MissionPhase,
  type MissionCommand,
  type MissionProgress,
} from '@/types/mission';
import {
  computeMissionControlFlags,
  getAbortMissionValidationError,
  withServiceTimeout,
} from '@/lib/missionControlBehavior';
import ROSLIB from 'roslib';

type SearchPattern = 'lawnmower' | 'spiral';
const MISSION_SERVICE_TIMEOUT_MS = 8000;
const INVALID_START_ZONE_ERROR =
  'Select an active zone with at least 3 points before starting.';

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
  hasValidStartZone: boolean;
  selectedPattern: SearchPattern;
  setSelectedPattern: (pattern: SearchPattern) => void;
  startMissionError: string | null;
  abortMissionError: string | null;
  
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
    completeMission: contextComplete,
    setPhase,
    updateProgress,
    updateStats,
    markExternalUpdate,
  } = useMissionContext();
  const { selectedZone } = useZoneContext();
  
  const { ros, isConnected: rosConnected } = useROSConnection();
  const [selectedPattern, setSelectedPattern] = useState<SearchPattern>('lawnmower');
  const [startMissionError, setStartMissionError] = useState<string | null>(null);
  const [abortMissionError, setAbortMissionError] = useState<string | null>(null);
  const hasValidStartZone =
    !!selectedZone && selectedZone.status === 'active' && selectedZone.polygon.length >= 3;
  const effectiveStartMissionError =
    hasValidStartZone && startMissionError === INVALID_START_ZONE_ERROR
      ? null
      : startMissionError;
  const updateSelectedPattern = useCallback((pattern: SearchPattern) => {
    setSelectedPattern(pattern);
    setStartMissionError(null);
  }, []);

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

  const callMissionService = useCallback(
    (
      serviceName: 'start_mission' | 'abort_mission',
      request: {
        command: MissionCommand;
        mission_id: string;
        zone_geometry: string;
      }
    ): Promise<{ success: boolean; message: string }> => {
      return withServiceTimeout(
        (resolve, reject) => {
          if (!ros || !rosConnected) {
            reject(new Error('ROS is not connected'));
            return;
          }

          const service = new ROSLIB.Service({
            ros,
            name: serviceName,
            serviceType: 'aeris_msgs/srv/MissionCommand',
          });
          const serviceRequest = new ROSLIB.ServiceRequest(request);
          service.callService(
            serviceRequest,
            response => {
              const typed = response as { success?: boolean; message?: string };
              resolve({
                success: Boolean(typed.success),
                message: typed.message ?? '',
              });
            },
            error => {
              reject(new Error(String(error)));
            }
          );
        },
        MISSION_SERVICE_TIMEOUT_MS,
        serviceName
      );
    },
    [ros, rosConnected]
  );
  
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

    const missionPhases: MissionPhase[] = [
      'IDLE',
      'PLANNING',
      'SEARCHING',
      'TRACKING',
      'COMPLETE',
      'ABORTED',
    ];

    const isMissionPhase = (value: string): value is MissionPhase =>
      missionPhases.includes(value as MissionPhase);
    
    const handleStateMessage = (message: ROSLIB.Message) => {
      try {
        const rawData = (message as { data: string }).data;
        const rawState = rawData.trim();
        let stateFromPayload: string | undefined;

        if (rawState.startsWith('{')) {
          const payload = JSON.parse(rawState) as { phase?: string; state?: string };
          stateFromPayload = payload.phase ?? payload.state;
        } else {
          stateFromPayload = rawState;
        }

        if (stateFromPayload && isMissionPhase(stateFromPayload)) {
          markExternalUpdate();
          setPhase(stateFromPayload);
          if (stateFromPayload === 'ABORTED' || stateFromPayload === 'IDLE') {
            setAbortMissionError(null);
          }
        } else {
          console.warn('[MissionControl] Ignoring unknown mission phase:', rawState);
        }
      } catch (error) {
        console.warn('[MissionControl] Failed to parse state message:', error);
      }
    };

    const handleProgressMessage = (message: ROSLIB.Message) => {
      try {
        const data = JSON.parse((message as { data: string }).data) as {
          coveragePercent?: number;
          coverage_percent?: number;
          searchAreaKm2?: number;
          search_area_km2?: number;
          coveredAreaKm2?: number;
          covered_area_km2?: number;
          activeDrones?: number;
          active_drones?: number;
          totalDrones?: number;
          total_drones?: number;
          estimatedTimeRemaining?: number;
          estimated_time_remaining?: number;
          gridProgress?: { completed?: number; total?: number };
          grid_completed?: number;
          grid_total?: number;
        };
        const progressPayload: Partial<MissionProgress> = {};

        const coveragePercent = data.coveragePercent ?? data.coverage_percent;
        if (coveragePercent !== undefined) {
          progressPayload.coveragePercent = coveragePercent;
        }

        const searchAreaKm2 = data.searchAreaKm2 ?? data.search_area_km2;
        if (searchAreaKm2 !== undefined) {
          progressPayload.searchAreaKm2 = searchAreaKm2;
        }

        const coveredAreaKm2 = data.coveredAreaKm2 ?? data.covered_area_km2;
        if (coveredAreaKm2 !== undefined) {
          progressPayload.coveredAreaKm2 = coveredAreaKm2;
        }

        const activeDrones = data.activeDrones ?? data.active_drones;
        if (activeDrones !== undefined) {
          progressPayload.activeDrones = activeDrones;
        }

        const totalDrones = data.totalDrones ?? data.total_drones;
        if (totalDrones !== undefined) {
          progressPayload.totalDrones = totalDrones;
        }

        const estimatedTimeRemaining =
          data.estimatedTimeRemaining ?? data.estimated_time_remaining;
        if (estimatedTimeRemaining !== undefined) {
          progressPayload.estimatedTimeRemaining = estimatedTimeRemaining;
        }

        if (data.gridProgress) {
          const completed = data.gridProgress.completed;
          const total = data.gridProgress.total;
          if (completed !== undefined || total !== undefined) {
            progressPayload.gridProgress = {
              completed: completed ?? 0,
              total: total ?? 0,
            };
          }
        } else if (data.grid_completed !== undefined || data.grid_total !== undefined) {
          progressPayload.gridProgress = {
            completed: data.grid_completed ?? 0,
            total: data.grid_total ?? 0,
          };
        }

        updateProgress(progressPayload);
        markExternalUpdate();
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
  }, [ros, rosConnected, setPhase, updateProgress, markExternalUpdate]);
  
  // ============================================================================
  // Actions with ROS publishing
  // ============================================================================
  
  const startMission = useCallback(() => {
    const zone = selectedZone;
    if (!zone || !hasValidStartZone) {
      setStartMissionError(INVALID_START_ZONE_ERROR);
      return;
    }
    const missionId = state.missionId?.trim() || generateMissionId();

    const payload = JSON.stringify({
      pattern: selectedPattern,
      zone: {
        id: zone.id,
        polygon: zone.polygon.map(point => ({ x: point.x, z: point.z })),
      },
    });
    callMissionService('start_mission', {
      command: 'START',
      mission_id: missionId,
      zone_geometry: payload,
    })
      .then(response => {
        if (!response.success) {
          setStartMissionError(response.message || 'Mission start was rejected by orchestrator.');
          return;
        }
        setStartMissionError(null);
        contextStart(missionId);
        setPhase('PLANNING');
        markExternalUpdate();
      })
      .catch(error => {
        console.error('[MissionControl] Failed to call start_mission:', error);
        setStartMissionError(
          error instanceof Error ? error.message : 'Failed to call start_mission'
        );
      });
  }, [
    callMissionService,
    contextStart,
    state.missionId,
    markExternalUpdate,
    selectedPattern,
    selectedZone,
    hasValidStartZone,
    setPhase,
  ]);
  
  const pauseMission = useCallback(() => {
    contextPause();
    publishCommand('PAUSE');
  }, [contextPause, publishCommand]);
  
  const resumeMission = useCallback(() => {
    contextResume();
    publishCommand('RESUME');
  }, [contextResume, publishCommand]);
  
  const abortMission = useCallback(() => {
    setAbortMissionError(null);

    const missionId = state.missionId?.trim() ?? '';
    const validationError = getAbortMissionValidationError({
      rosConnected: rosConnected && !!ros,
      missionId,
    });
    if (validationError) {
      if (!missionId) {
        console.warn('[MissionControl] abort_mission called without a missionId');
      }
      setAbortMissionError(validationError);
      return;
    }

    callMissionService('abort_mission', {
      command: 'ABORT',
      mission_id: missionId,
      zone_geometry: '',
    })
      .then(response => {
        if (!response.success) {
          console.warn('[MissionControl] abort_mission rejected:', response.message);
          setAbortMissionError(
            response.message || 'Mission abort was rejected by orchestrator.'
          );
          return;
        }
        setAbortMissionError(null);
      })
      .catch(error => {
        console.error('[MissionControl] Failed to call abort_mission:', error);
        setAbortMissionError(
          error instanceof Error ? error.message : 'Failed to call abort_mission'
        );
      });
  }, [
    callMissionService,
    ros,
    rosConnected,
    state.missionId,
  ]);

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
    const controlFlags = computeMissionControlFlags({
      phase: state.phase,
      pausedAt: state.pausedAt,
      hasValidStartZone,
      rosConnected,
    });
    
    return {
      phase: state.phase,
      isActive: controlFlags.isActive,
      isPaused: controlFlags.isPaused,
      isComplete: controlFlags.isComplete,
      missionId: state.missionId,
      
      // Control flags
      canStart: controlFlags.canStart,
      canPause: controlFlags.canPause,
      canResume: controlFlags.canResume,
      canAbort: controlFlags.canAbort,
      hasValidStartZone,
    };
  }, [rosConnected, hasValidStartZone, state.phase, state.pausedAt, state.missionId]);
  
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
    
    selectedPattern,
    setSelectedPattern: updateSelectedPattern,
    startMissionError: effectiveStartMissionError,
    abortMissionError,

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
