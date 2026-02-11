'use client';

import React, {
  createContext,
  useContext,
  useState,
  useCallback,
  useMemo,
  type ReactNode,
} from 'react';
import type { PriorityZone, ZoneInput, ZonePriority, ZonePoint } from '@/types/zone';
import { createZone } from '@/types/zone';

export type DrawingMode = 'none' | 'drawing' | 'editing';

export interface DrawingState {
  mode: DrawingMode;
  currentPriority: ZonePriority;
  points: ZonePoint[];
  editingZoneId?: string;
}

interface ZoneContextValue {
  zones: PriorityZone[];
  activeZones: PriorityZone[];
  drawing: DrawingState;
  isDrawing: boolean;
  addZone: (input: ZoneInput) => PriorityZone;
  updateZone: (id: string, updates: Partial<PriorityZone>) => void;
  deleteZone: (id: string) => void;
  completeZone: (id: string) => void;
  skipZone: (id: string) => void;
  reactivateZone: (id: string) => void;
  startDrawing: (priority: ZonePriority) => void;
  addPoint: (point: ZonePoint) => void;
  undoLastPoint: () => void;
  cancelDrawing: () => void;
  finishDrawing: (name?: string, notes?: string) => PriorityZone | null;
  setPriority: (priority: ZonePriority) => void;
  startEditing: (zoneId: string) => void;
  stopEditing: () => void;
  selectedZoneId: string | null;
  selectedZone: PriorityZone | null;
  selectZone: (id: string | null) => void;
}

const ZoneContext = createContext<ZoneContextValue | null>(null);

interface ZoneProviderProps {
  children: ReactNode;
}

/**
 * Provides search zone management and polygon drawing capabilities.
 *
 * Zones define search areas for the mission orchestrator. Each zone has:
 * - A priority level (1-5) affecting search order
 * - A polygon boundary (minimum 3 points)
 * - A lifecycle status (active, completed, skipped)
 *
 * Drawing workflow:
 * 1. Call startDrawing(priority) to enter drawing mode
 * 2. Map interactions call addPoint() for each vertex
 * 3. finishDrawing() validates (>=3 points) and creates the zone
 * 4. Editing mode allows modifying existing zone geometry
 */
export function ZoneProvider({ children }: ZoneProviderProps) {
  const [zones, setZones] = useState<PriorityZone[]>([]);
  const [selectedZoneId, setSelectedZoneId] = useState<string | null>(null);
  const [drawing, setDrawing] = useState<DrawingState>({
    mode: 'none',
    currentPriority: 1,
    points: [],
  });

  const activeZones = useMemo(() => {
    return zones.filter(z => z.status === 'active');
  }, [zones]);

  const selectedZone = useMemo(() => {
    return zones.find(z => z.id === selectedZoneId) ?? null;
  }, [zones, selectedZoneId]);

  const isDrawing = drawing.mode === 'drawing';

  const addZone = useCallback((input: ZoneInput): PriorityZone => {
    const zone = createZone(input);
    setZones(prev => [...prev, zone]);
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

  const startDrawing = useCallback((priority: ZonePriority) => {
    setDrawing({
      mode: 'drawing',
      currentPriority: priority,
      points: [],
    });
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
  }, []);

  /**
   * Finalizes the current drawing into a search zone.
   * Requires at least 3 points to form a valid polygon.
   * Returns null and cancels if validation fails.
   */
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

  /**
   * Enters editing mode for an existing zone.
   * Loads the zone's polygon and priority into the drawing state.
   */
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

  /**
   * Exits editing mode and persists changes to the zone.
   * No-op if not currently in editing mode.
   */
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

  const selectZone = useCallback((id: string | null) => {
    setSelectedZoneId(id);
  }, []);

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

export function useZoneContext(): ZoneContextValue {
  const context = useContext(ZoneContext);

  if (!context) {
    throw new Error('useZoneContext must be used within a ZoneProvider');
  }

  return context;
}

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
