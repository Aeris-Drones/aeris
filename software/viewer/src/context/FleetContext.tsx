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
import ROSLIB from 'roslib';

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
  sendCommand: (vehicleId: string, command: VehicleCommand, params?: { latitude?: number; longitude?: number; altitude?: number }) => void;
  recallVehicle: (vehicleId: string) => void;
  holdVehicle: (vehicleId: string) => void;
  resumeVehicle: (vehicleId: string) => void;
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
  
  const sendCommand = useCallback((
    vehicleId: string,
    command: VehicleCommand,
    params?: { latitude?: number; longitude?: number; altitude?: number }
  ) => {
    const request: VehicleCommandRequest = {
      vehicleId,
      command,
      timestamp: Date.now(),
      params,
    };
    
    setLastCommand(request);
    
    // Publish to ROS if connected
    if (ros && isConnected) {
      const topic = new ROSLIB.Topic({
        ros: ros,
        name: '/fleet/command',
        messageType: 'std_msgs/String',
      });
      
      const message = new ROSLIB.Message({
        data: JSON.stringify(request),
      });
      
      topic.publish(message);
    }
  }, [ros, isConnected]);
  
  const recallVehicle = useCallback((vehicleId: string) => {
    sendCommand(vehicleId, 'RECALL');
  }, [sendCommand]);
  
  const holdVehicle = useCallback((vehicleId: string) => {
    sendCommand(vehicleId, 'HOLD');
  }, [sendCommand]);
  
  const resumeVehicle = useCallback((vehicleId: string) => {
    sendCommand(vehicleId, 'RESUME');
  }, [sendCommand]);
  
  const recallAll = useCallback(() => {
    vehicles.forEach(v => sendCommand(v.id, 'RECALL'));
  }, [vehicles, sendCommand]);
  
  const holdAll = useCallback(() => {
    vehicles.forEach(v => sendCommand(v.id, 'HOLD'));
  }, [vehicles, sendCommand]);
  
  const resumeAll = useCallback(() => {
    vehicles.forEach(v => sendCommand(v.id, 'RESUME'));
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
