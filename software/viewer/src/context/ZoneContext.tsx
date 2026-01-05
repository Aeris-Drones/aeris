'use client';

/**
 * AERIS GCS Zone Context
 * 
 * Global state management for priority search zones.
 */

import React, {
  createContext,
  useContext,
  useState,
  useCallback,
  useMemo,
  type ReactNode,
} from 'react';
import type { PriorityZone, ZoneInput, ZonePriority, ZonePoint } from '@/types/zone';
import { createZone, generateZoneId } from '@/types/zone';

// ============================================================================
// Drawing State
// ============================================================================

export type DrawingMode = 'none' | 'drawing' | 'editing';

export interface DrawingState {
  mode: DrawingMode;
  currentPriority: ZonePriority;
  points: ZonePoint[];
  editingZoneId?: string;
}

// ============================================================================
// Context Types
// ============================================================================

interface ZoneContextValue {
  // Zones
  zones: PriorityZone[];
  activeZones: PriorityZone[];
  
  // Drawing state
  drawing: DrawingState;
  isDrawing: boolean;
  
  // Zone CRUD
  addZone: (input: ZoneInput) => PriorityZone;
  updateZone: (id: string, updates: Partial<PriorityZone>) => void;
  deleteZone: (id: string) => void;
  completeZone: (id: string) => void;
  skipZone: (id: string) => void;
  reactivateZone: (id: string) => void;
  
  // Drawing actions
  startDrawing: (priority: ZonePriority) => void;
  addPoint: (point: ZonePoint) => void;
  undoLastPoint: () => void;
  cancelDrawing: () => void;
  finishDrawing: (name?: string, notes?: string) => PriorityZone | null;
  setPriority: (priority: ZonePriority) => void;
  
  // Editing
  startEditing: (zoneId: string) => void;
  stopEditing: () => void;
  
  // Selection
  selectedZoneId: string | null;
  selectedZone: PriorityZone | null;
  selectZone: (id: string | null) => void;
}

// ============================================================================
// Context Creation
// ============================================================================

const ZoneContext = createContext<ZoneContextValue | null>(null);

// ============================================================================
// Provider Component
// ============================================================================

interface ZoneProviderProps {
  children: ReactNode;
}

export function ZoneProvider({ children }: ZoneProviderProps) {
  const [zones, setZones] = useState<PriorityZone[]>([]);
  const [selectedZoneId, setSelectedZoneId] = useState<string | null>(null);
  const [drawing, setDrawing] = useState<DrawingState>({
    mode: 'none',
    currentPriority: 1,
    points: [],
  });
  
  // ============================================================================
  // Computed Values
  // ============================================================================
  
  const activeZones = useMemo(() => {
    return zones.filter(z => z.status === 'active');
  }, [zones]);
  
  const selectedZone = useMemo(() => {
    return zones.find(z => z.id === selectedZoneId) ?? null;
  }, [zones, selectedZoneId]);
  
  const isDrawing = drawing.mode === 'drawing';
  
  // ============================================================================
  // Zone CRUD
  // ============================================================================
  
  const addZone = useCallback((input: ZoneInput): PriorityZone => {
    const zone = createZone(input);
    setZones(prev => [...prev, zone]);
    console.log(`[Zone] Created zone: ${zone.name} (Priority ${zone.priority})`);
    return zone;
  }, []);
  
  const updateZone = useCallback((id: string, updates: Partial<PriorityZone>) => {
    setZones(prev => prev.map(z => 
      z.id === id ? { ...z, ...updates } : z
    ));
  }, []);
  
  const deleteZone = useCallback((id: string) => {
    setZones(prev => prev.filter(z => z.id !== id));
    if (selectedZoneId === id) {
      setSelectedZoneId(null);
    }
    console.log(`[Zone] Deleted zone: ${id}`);
  }, [selectedZoneId]);
  
  const completeZone = useCallback((id: string) => {
    updateZone(id, { status: 'completed', completedAt: Date.now() });
  }, [updateZone]);
  
  const skipZone = useCallback((id: string) => {
    updateZone(id, { status: 'skipped' });
  }, [updateZone]);
  
  const reactivateZone = useCallback((id: string) => {
    updateZone(id, { status: 'active', completedAt: undefined });
  }, [updateZone]);
  
  // ============================================================================
  // Drawing Actions
  // ============================================================================
  
  const startDrawing = useCallback((priority: ZonePriority) => {
    setDrawing({
      mode: 'drawing',
      currentPriority: priority,
      points: [],
    });
    console.log(`[Zone] Started drawing priority ${priority} zone`);
  }, []);
  
  const addPoint = useCallback((point: ZonePoint) => {
    setDrawing(prev => ({
      ...prev,
      points: [...prev.points, point],
    }));
  }, []);
  
  const undoLastPoint = useCallback(() => {
    setDrawing(prev => ({
      ...prev,
      points: prev.points.slice(0, -1),
    }));
  }, []);
  
  const cancelDrawing = useCallback(() => {
    setDrawing({
      mode: 'none',
      currentPriority: 1,
      points: [],
    });
    console.log('[Zone] Drawing cancelled');
  }, []);
  
  const finishDrawing = useCallback((name?: string, notes?: string): PriorityZone | null => {
    if (drawing.points.length < 3) {
      console.warn('[Zone] Need at least 3 points to create a zone');
      cancelDrawing();
      return null;
    }
    
    const zone = addZone({
      name,
      priority: drawing.currentPriority,
      polygon: drawing.points,
      notes,
    });
    
    setDrawing({
      mode: 'none',
      currentPriority: 1,
      points: [],
    });
    
    return zone;
  }, [drawing, addZone, cancelDrawing]);
  
  const setPriority = useCallback((priority: ZonePriority) => {
    setDrawing(prev => ({
      ...prev,
      currentPriority: priority,
    }));
  }, []);
  
  // ============================================================================
  // Editing
  // ============================================================================
  
  const startEditing = useCallback((zoneId: string) => {
    const zone = zones.find(z => z.id === zoneId);
    if (!zone) return;
    
    setDrawing({
      mode: 'editing',
      currentPriority: zone.priority,
      points: [...zone.polygon],
      editingZoneId: zoneId,
    });
  }, [zones]);
  
  const stopEditing = useCallback(() => {
    if (drawing.mode === 'editing' && drawing.editingZoneId) {
      updateZone(drawing.editingZoneId, {
        polygon: drawing.points,
        priority: drawing.currentPriority,
      });
    }
    
    setDrawing({
      mode: 'none',
      currentPriority: 1,
      points: [],
    });
  }, [drawing, updateZone]);
  
  // ============================================================================
  // Selection
  // ============================================================================
  
  const selectZone = useCallback((id: string | null) => {
    setSelectedZoneId(id);
  }, []);
  
  // ============================================================================
  // Context Value
  // ============================================================================
  
  const value: ZoneContextValue = {
    zones,
    activeZones,
    drawing,
    isDrawing,
    addZone,
    updateZone,
    deleteZone,
    completeZone,
    skipZone,
    reactivateZone,
    startDrawing,
    addPoint,
    undoLastPoint,
    cancelDrawing,
    finishDrawing,
    setPriority,
    startEditing,
    stopEditing,
    selectedZoneId,
    selectedZone,
    selectZone,
  };
  
  return (
    <ZoneContext.Provider value={value}>
      {children}
    </ZoneContext.Provider>
  );
}

// ============================================================================
// Hook
// ============================================================================

export function useZoneContext(): ZoneContextValue {
  const context = useContext(ZoneContext);
  
  if (!context) {
    throw new Error('useZoneContext must be used within a ZoneProvider');
  }
  
  return context;
}

// ============================================================================
// Selector Hooks
// ============================================================================

export function useZones(): PriorityZone[] {
  return useZoneContext().zones;
}

export function useActiveZones(): PriorityZone[] {
  return useZoneContext().activeZones;
}

export function useDrawingState(): DrawingState {
  return useZoneContext().drawing;
}

export function useIsDrawing(): boolean {
  return useZoneContext().isDrawing;
}
