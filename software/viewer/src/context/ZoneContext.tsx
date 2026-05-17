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
import {
  DEFAULT_ROUTE_STAGING_AREA,
  type RouteStagingArea,
} from '@/lib/routeRecommendations';

export type DrawingMode = 'none' | 'drawing' | 'editing';
export type DrawingTarget = 'search-zone' | 'structural-hazard';

export interface DrawingState {
  mode: DrawingMode;
  target: DrawingTarget;
  currentPriority: ZonePriority;
  points: ZonePoint[];
  editingZoneId?: string;
}

interface ZoneContextValue {
  zones: PriorityZone[];
  activeZones: PriorityZone[];
  structuralHazardZones: PriorityZone[];
  drawing: DrawingState;
  isDrawing: boolean;
  routeStagingArea: RouteStagingArea;
  isPlacingRouteStagingArea: boolean;
  addZone: (input: ZoneInput) => PriorityZone;
  updateZone: (id: string, updates: Partial<PriorityZone>) => void;
  deleteZone: (id: string) => void;
  completeZone: (id: string) => void;
  skipZone: (id: string) => void;
  reactivateZone: (id: string) => void;
  startDrawing: (priority: ZonePriority, target?: DrawingTarget) => void;
  addPoint: (point: ZonePoint) => void;
  undoLastPoint: () => void;
  cancelDrawing: () => void;
  finishDrawing: (name?: string, notes?: string) => PriorityZone | null;
  setPriority: (priority: ZonePriority) => void;
  startPlacingRouteStagingArea: () => void;
  setRouteStagingAreaPosition: (point: ZonePoint) => void;
  cancelRouteStagingAreaPlacement: () => void;
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
 * Provides search zone management with polygon drawing and editing capabilities.
 *
 * Zones define search areas for the mission orchestrator, with priority levels
 * affecting search order. The drawing workflow supports both creating new zones
 * and editing existing zone geometry.
 */
export function ZoneProvider({ children }: ZoneProviderProps) {
  const [zones, setZones] = useState<PriorityZone[]>([]);
  const [selectedZoneId, setSelectedZoneId] = useState<string | null>(null);
  const [drawing, setDrawing] = useState<DrawingState>({
    mode: 'none',
    target: 'search-zone',
    currentPriority: 1,
    points: [],
  });
  const [routeStagingArea, setRouteStagingArea] = useState<RouteStagingArea>(
    DEFAULT_ROUTE_STAGING_AREA
  );
  const [isPlacingRouteStagingArea, setIsPlacingRouteStagingArea] = useState(false);

  const activeZones = useMemo(() => {
    return zones.filter(z => z.kind === 'search' && z.status === 'active');
  }, [zones]);

  const structuralHazardZones = useMemo(() => {
    return zones.filter(z => z.kind === 'structural_hazard' && z.status === 'active');
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

  const startDrawing = useCallback((priority: ZonePriority, target: DrawingTarget = 'search-zone') => {
    setIsPlacingRouteStagingArea(false);
    setDrawing({
      mode: 'drawing',
      target,
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
      target: 'search-zone',
      currentPriority: 1,
      points: [],
    });
  }, []);

  const finishDrawing = useCallback((name?: string, notes?: string): PriorityZone | null => {
    if (drawing.points.length < 3) {
      console.warn('[Zone] Need at least 3 points to create a zone');
      cancelDrawing();
      return null;
    }

    const zone = addZone({
      name,
      kind: drawing.target === 'structural-hazard' ? 'structural_hazard' : 'search',
      priority: drawing.currentPriority,
      polygon: drawing.points,
      notes,
    });

    setDrawing({
      mode: 'none',
      target: 'search-zone',
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

  const startPlacingRouteStagingArea = useCallback(() => {
    cancelDrawing();
    setIsPlacingRouteStagingArea(true);
  }, [cancelDrawing]);

  const setRouteStagingAreaPosition = useCallback((point: ZonePoint) => {
    setRouteStagingArea((previous) => ({
      ...previous,
      position: [point.x, 0, point.z],
    }));
    setIsPlacingRouteStagingArea(false);
  }, []);

  const cancelRouteStagingAreaPlacement = useCallback(() => {
    setIsPlacingRouteStagingArea(false);
  }, []);

  const startEditing = useCallback((zoneId: string) => {
    const zone = zones.find(z => z.id === zoneId);
    if (!zone) return;

    setDrawing({
      mode: 'editing',
      target: zone.kind === 'structural_hazard' ? 'structural-hazard' : 'search-zone',
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
        kind: drawing.target === 'structural-hazard' ? 'structural_hazard' : 'search',
      });
    }

    setDrawing({
      mode: 'none',
      target: 'search-zone',
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
    structuralHazardZones,
    drawing,
    isDrawing,
    routeStagingArea,
    isPlacingRouteStagingArea,
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
    startPlacingRouteStagingArea,
    setRouteStagingAreaPosition,
    cancelRouteStagingAreaPlacement,
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

/**
 * Accesses the zone context. Must be used within a ZoneProvider.
 *
 * @throws Error if used outside ZoneProvider
 */
export function useZoneContext(): ZoneContextValue {
  const context = useContext(ZoneContext);
  if (!context) {
    throw new Error('useZoneContext must be used within a ZoneProvider');
  }
  return context;
}

/** Returns all search zones. */
export function useZones(): PriorityZone[] {
  return useZoneContext().zones;
}

/** Returns zones with active status (not completed or skipped). */
export function useActiveZones(): PriorityZone[] {
  return useZoneContext().activeZones;
}

/** Returns the current drawing state for zone creation/editing. */
export function useDrawingState(): DrawingState {
  return useZoneContext().drawing;
}

/** Returns true when user is actively drawing a zone polygon. */
export function useIsDrawing(): boolean {
  return useZoneContext().isDrawing;
}
