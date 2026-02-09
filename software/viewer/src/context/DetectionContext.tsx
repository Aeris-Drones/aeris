'use client';

import React, { createContext, useContext, useState, useCallback, useMemo, useEffect } from 'react';
import type {
  Detection,
  DetectionFilter,
  DetectionSortBy,
  DetectionSortOrder,
  DetectionStats,
  DetectionStatus,
} from '@/types/detection';
import { getDetectionConfidenceLevel } from '@/types/detection';

interface DetectionContextValue {
  /** All detections (raw list) */
  detections: Detection[];
  /** Filtered and sorted detections */
  filteredDetections: Detection[];
  /** Current filter settings */
  filter: DetectionFilter;
  /** Current sort settings */
  sortBy: DetectionSortBy;
  sortOrder: DetectionSortOrder;
  /** Detection statistics */
  stats: DetectionStats;
  /** Update detections list (called by aggregation hook) */
  setDetections: (detections: Detection[]) => void;
  /** Confirm a detection */
  confirmDetection: (id: string, notes?: string) => void;
  /** Dismiss a detection */
  dismissDetection: (id: string, notes?: string) => void;
  /** Add notes to a detection */
  addNotes: (id: string, notes: string) => void;
  /** Update filter */
  setFilter: (filter: DetectionFilter) => void;
  /** Update sort */
  setSort: (sortBy: DetectionSortBy, order?: DetectionSortOrder) => void;
  /** Clear all filters */
  clearFilters: () => void;
  /** Get detection by ID */
  getDetection: (id: string) => Detection | undefined;
  /** Undo last action (5 second window) */
  undoLastAction: () => void;
  /** Can undo */
  canUndo: boolean;
}

export const DetectionContext = createContext<DetectionContextValue | undefined>(undefined);

const DEFAULT_FILTER: DetectionFilter = {
  sensorTypes: undefined,
  statuses: undefined,
  minConfidence: undefined,
  vehicleId: undefined,
  timeWindow: undefined,
};

interface UndoState {
  detectionId: string;
  previousStatus: DetectionStatus;
  previousNotes?: string;
  timestamp: number;
}

const UNDO_TIMEOUT = 5000; // 5 seconds

export function DetectionProvider({ children }: { children: React.ReactNode }) {
  const [detections, setDetections] = useState<Detection[]>([]);
  const [filter, setFilter] = useState<DetectionFilter>(DEFAULT_FILTER);
  const [sortBy, setSortBy] = useState<DetectionSortBy>('time');
  const [sortOrder, setSortOrder] = useState<DetectionSortOrder>('desc');
  const [undoState, setUndoState] = useState<UndoState | null>(null);
  const [now, setNow] = useState(() => Date.now());

  useEffect(() => {
    const timer = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(timer);
  }, []);

  // Update detection status
  const updateDetectionStatus = useCallback(
    (id: string, status: DetectionStatus, notes?: string) => {
      setDetections((prev) => {
        const detection = prev.find((d) => d.id === id);
        if (!detection) return prev;

        // Save undo state
        setUndoState({
          detectionId: id,
          previousStatus: detection.status,
          previousNotes: detection.notes,
          timestamp: Date.now(),
        });

        // Clear undo after timeout
        setTimeout(() => {
          setUndoState((current) => {
            if (current?.detectionId === id && Date.now() - current.timestamp >= UNDO_TIMEOUT) {
              return null;
            }
            return current;
          });
        }, UNDO_TIMEOUT);

        return prev.map((d) =>
          d.id === id
            ? {
                ...d,
                status,
                notes: notes !== undefined ? notes : d.notes,
                lastUpdate: Date.now(),
              }
            : d
        );
      });
    },
    []
  );

  // Confirm detection
  const confirmDetection = useCallback(
    (id: string, notes?: string) => {
      updateDetectionStatus(id, 'confirmed', notes);
    },
    [updateDetectionStatus]
  );

  // Dismiss detection
  const dismissDetection = useCallback(
    (id: string, notes?: string) => {
      updateDetectionStatus(id, 'dismissed', notes);
    },
    [updateDetectionStatus]
  );

  // Add notes
  const addNotes = useCallback((id: string, notes: string) => {
    setDetections((prev) =>
      prev.map((d) =>
        d.id === id
          ? {
              ...d,
              notes,
              lastUpdate: Date.now(),
            }
          : d
      )
    );
  }, []);

  // Undo last action
  const undoLastAction = useCallback(() => {
    if (!undoState) return;
    if (Date.now() - undoState.timestamp >= UNDO_TIMEOUT) {
      setUndoState(null);
      return;
    }

    setDetections((prev) =>
      prev.map((d) =>
        d.id === undoState.detectionId
          ? {
              ...d,
              status: undoState.previousStatus,
              notes: undoState.previousNotes,
              lastUpdate: Date.now(),
            }
          : d
      )
    );

    setUndoState(null);
  }, [undoState]);

  // Get detection by ID
  const getDetection = useCallback(
    (id: string) => {
      return detections.find((d) => d.id === id);
    },
    [detections]
  );

  // Clear filters
  const clearFilters = useCallback(() => {
    setFilter(DEFAULT_FILTER);
  }, []);

  // Update sort
  const setSort = useCallback((newSortBy: DetectionSortBy, order?: DetectionSortOrder) => {
    setSortBy(newSortBy);
    if (order) {
      setSortOrder(order);
    }
  }, []);

  // Filtered detections
  const filteredDetections = useMemo(() => {
    let filtered = [...detections];

    // Apply sensor type filter
    if (filter.sensorTypes && filter.sensorTypes.length > 0) {
      filtered = filtered.filter((d) => filter.sensorTypes!.includes(d.sensorType));
    }

    // Apply status filter
    if (filter.statuses && filter.statuses.length > 0) {
      filtered = filtered.filter((d) => filter.statuses!.includes(d.status));
    }

    // Apply confidence filter
    if (filter.minConfidence !== undefined) {
      filtered = filtered.filter((d) => d.confidence >= filter.minConfidence!);
    }

    // Apply vehicle filter
    if (filter.vehicleId) {
      filtered = filtered.filter((d) => d.vehicleId === filter.vehicleId);
    }

    // Apply time window filter
    if (filter.timeWindow) {
      filtered = filtered.filter((d) => now - d.timestamp <= filter.timeWindow!);
    }

    // Sort
    filtered.sort((a, b) => {
      let comparison = 0;

      switch (sortBy) {
        case 'confidence':
          comparison = a.confidence - b.confidence;
          break;
        case 'time':
          comparison = a.timestamp - b.timestamp;
          break;
        case 'type':
          comparison = a.sensorType.localeCompare(b.sensorType);
          break;
        case 'distance':
          // Distance sorting would need reference point - skip for now
          comparison = 0;
          break;
      }

      return sortOrder === 'asc' ? comparison : -comparison;
    });

    return filtered;
  }, [detections, filter, sortBy, sortOrder, now]);

  // Statistics
  const stats = useMemo((): DetectionStats => {
    const byType = { thermal: 0, acoustic: 0, gas: 0 };
    const byStatus = { new: 0, reviewing: 0, confirmed: 0, dismissed: 0, expired: 0 };
    const byConfidence = { high: 0, medium: 0, low: 0, unverified: 0 };

    detections.forEach((d) => {
      byType[d.sensorType]++;
      byStatus[d.status]++;
      byConfidence[getDetectionConfidenceLevel(d)]++;
    });

    return {
      total: detections.length,
      byType,
      byStatus,
      byConfidence,
    };
  }, [detections]);

  const canUndo = useMemo(() => {
    if (!undoState) return false;
    return now - undoState.timestamp < UNDO_TIMEOUT;
  }, [undoState, now]);

  const value: DetectionContextValue = {
    detections,
    filteredDetections,
    filter,
    sortBy,
    sortOrder,
    stats,
    setDetections,
    confirmDetection,
    dismissDetection,
    addNotes,
    setFilter,
    setSort,
    clearFilters,
    getDetection,
    undoLastAction,
    canUndo,
  };

  return <DetectionContext.Provider value={value}>{children}</DetectionContext.Provider>;
}

export function useDetectionContext() {
  const context = useContext(DetectionContext);
  if (!context) {
    throw new Error('useDetectionContext must be used within DetectionProvider');
  }
  return context;
}
