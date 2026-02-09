'use client';

/**
 * AERIS GCS Fleet Context
 * 
 * Global state management for fleet control, vehicle selection,
 * and command dispatch.
 */

import React, {
  createContext,
  useContext,
  useState,
  useCallback,
  useMemo,
  type ReactNode,
} from 'react';
import type { VehicleInfo, VehicleCommand, VehicleCommandRequest } from '@/types/vehicle';
import { useVehicleTelemetry } from '@/hooks/useVehicleTelemetry';
import { vehicleStateToInfo } from '@/types/vehicle';
import { useROSConnection } from '@/hooks/useROSConnection';
import {
  buildVehicleCommandServiceRequest,
  callVehicleCommandService,
  formatVehicleCommandFailure,
  getVehicleCommandValidationError,
} from '@/lib/fleetCommandBehavior';
import ROSLIB from 'roslib';

type FleetCommandResult = {
  success: boolean;
  message: string;
};

// ============================================================================
// Context Types
// ============================================================================

interface FleetContextValue {
  // Fleet state
  vehicles: VehicleInfo[];
  selectedVehicleId: string | null;
  selectedVehicle: VehicleInfo | null;
  
  // Selection
  selectVehicle: (id: string | null) => void;
  toggleVehicleSelection: (id: string) => void;
  
  // Commands
  sendCommand: (
    vehicleId: string,
    command: VehicleCommand,
    params?: { latitude?: number; longitude?: number; altitude?: number }
  ) => Promise<FleetCommandResult>;
  recallVehicle: (vehicleId: string) => Promise<FleetCommandResult>;
  holdVehicle: (vehicleId: string) => Promise<FleetCommandResult>;
  resumeVehicle: (vehicleId: string) => Promise<FleetCommandResult>;
  recallAll: () => void;
  holdAll: () => void;
  resumeAll: () => void;
  
  // Stats
  fleetStats: {
    total: number;
    active: number;
    returning: number;
    offline: number;
    avgBattery: number;
  };
  
  // Command history
  lastCommand?: VehicleCommandRequest;
  commandErrors: Record<string, string>;
}

// ============================================================================
// Context Creation
// ============================================================================

const FleetContext = createContext<FleetContextValue | null>(null);

// ============================================================================
// Provider Component
// ============================================================================

interface FleetProviderProps {
  children: ReactNode;
}

export function FleetProvider({ children }: FleetProviderProps) {
  const { vehicles: rawVehicles } = useVehicleTelemetry();
  const { ros, isConnected } = useROSConnection();
  
  const [selectedVehicleId, setSelectedVehicleId] = useState<string | null>(null);
  const [lastCommand, setLastCommand] = useState<VehicleCommandRequest>();
  const [commandErrors, setCommandErrors] = useState<Record<string, string>>({});
  
  // Convert raw vehicle states to VehicleInfo
  const vehicles = useMemo(() => {
    return rawVehicles.map(v => vehicleStateToInfo(v, v.id === selectedVehicleId));
  }, [rawVehicles, selectedVehicleId]);
  
  // Selected vehicle
  const selectedVehicle = useMemo(() => {
    return vehicles.find(v => v.id === selectedVehicleId) ?? null;
  }, [vehicles, selectedVehicleId]);
  
  // Fleet statistics
  const fleetStats = useMemo(() => {
    const total = vehicles.length;
    const active = vehicles.filter(v => v.status === 'active').length;
    const returning = vehicles.filter(v => v.status === 'returning').length;
    const offline = vehicles.filter(v => v.status === 'offline').length;
    const avgBattery = total > 0
      ? vehicles.reduce((sum, v) => sum + v.batteryPercent, 0) / total
      : 0;
    
    return { total, active, returning, offline, avgBattery };
  }, [vehicles]);
  
  // ============================================================================
  // Selection
  // ============================================================================
  
  const selectVehicle = useCallback((id: string | null) => {
    setSelectedVehicleId(id);
  }, []);
  
  const toggleVehicleSelection = useCallback((id: string) => {
    setSelectedVehicleId(prev => prev === id ? null : id);
  }, []);
  
  // ============================================================================
  // Commands
  // ============================================================================
  
  const publishLegacyCommand = useCallback((request: VehicleCommandRequest) => {
    if (!ros || !isConnected) {
      return;
    }

    const topic = new ROSLIB.Topic({
      ros,
      name: '/fleet/command',
      messageType: 'std_msgs/String',
    });

    const message = new ROSLIB.Message({
      data: JSON.stringify(request),
    });
    topic.publish(message);
  }, [ros, isConnected]);

  const sendCommand = useCallback(async (
    vehicleId: string,
    command: VehicleCommand,
    params?: { latitude?: number; longitude?: number; altitude?: number }
  ): Promise<FleetCommandResult> => {
    const request: VehicleCommandRequest = {
      vehicleId,
      command,
      timestamp: Date.now(),
      params,
    };
    
    setLastCommand(request);

    const validationError = getVehicleCommandValidationError({
      rosConnected: !!ros && isConnected,
      vehicleId,
    });
    if (validationError) {
      setCommandErrors(prev => ({ ...prev, [vehicleId]: validationError }));
      return { success: false, message: validationError };
    }

    const serviceRequest = buildVehicleCommandServiceRequest({
      vehicleId,
      command,
    });

    try {
      const response = await callVehicleCommandService({
        ros,
        request: serviceRequest,
        createService: args => new ROSLIB.Service(args),
        createServiceRequest: payload => new ROSLIB.ServiceRequest(payload as object),
      });

      if (!response.success) {
        const failure = formatVehicleCommandFailure(command, vehicleId, response.message);
        setCommandErrors(prev => ({ ...prev, [vehicleId]: failure }));
        console.warn(`[FleetContext] ${failure}`);
        return { success: false, message: failure };
      }

      setCommandErrors(prev => {
        if (!(vehicleId in prev)) {
          return prev;
        }
        const next = { ...prev };
        delete next[vehicleId];
        return next;
      });

      // Keep backward compatibility with legacy listeners until they are removed.
      publishLegacyCommand(request);
      return response;
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : String(error);
      const failure = formatVehicleCommandFailure(command, vehicleId, errorMessage);
      setCommandErrors(prev => ({ ...prev, [vehicleId]: failure }));
      console.error('[FleetContext] Failed to send vehicle command:', error);
      return { success: false, message: failure };
    }
  }, [isConnected, publishLegacyCommand, ros]);
  
  const recallVehicle = useCallback((vehicleId: string) => {
    return sendCommand(vehicleId, 'RECALL');
  }, [sendCommand]);
  
  const holdVehicle = useCallback((vehicleId: string) => {
    return sendCommand(vehicleId, 'HOLD');
  }, [sendCommand]);
  
  const resumeVehicle = useCallback((vehicleId: string) => {
    return sendCommand(vehicleId, 'RESUME');
  }, [sendCommand]);
  
  const recallAll = useCallback(() => {
    void Promise.all(vehicles.map(v => sendCommand(v.id, 'RECALL')));
  }, [vehicles, sendCommand]);
  
  const holdAll = useCallback(() => {
    void Promise.all(vehicles.map(v => sendCommand(v.id, 'HOLD')));
  }, [vehicles, sendCommand]);
  
  const resumeAll = useCallback(() => {
    void Promise.all(vehicles.map(v => sendCommand(v.id, 'RESUME')));
  }, [vehicles, sendCommand]);
  
  // ============================================================================
  // Context Value
  // ============================================================================
  
  const value: FleetContextValue = {
    vehicles,
    selectedVehicleId,
    selectedVehicle,
    selectVehicle,
    toggleVehicleSelection,
    sendCommand,
    recallVehicle,
    holdVehicle,
    resumeVehicle,
    recallAll,
    holdAll,
    resumeAll,
    fleetStats,
    lastCommand,
    commandErrors,
  };
  
  return (
    <FleetContext.Provider value={value}>
      {children}
    </FleetContext.Provider>
  );
}

// ============================================================================
// Hook
// ============================================================================

export function useFleetContext(): FleetContextValue {
  const context = useContext(FleetContext);
  
  if (!context) {
    throw new Error('useFleetContext must be used within a FleetProvider');
  }
  
  return context;
}

// ============================================================================
// Selector Hooks
// ============================================================================

export function useFleetVehicles(): VehicleInfo[] {
  return useFleetContext().vehicles;
}

export function useSelectedVehicle(): VehicleInfo | null {
  return useFleetContext().selectedVehicle;
}

export function useFleetStats() {
  return useFleetContext().fleetStats;
}
