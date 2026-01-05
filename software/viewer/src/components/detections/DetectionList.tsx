'use client';

import React, { useMemo, useState } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { cn } from '@/lib/utils';
import { listContainerVariants, listItemVariants } from '@/lib/animations';
import { DetectionCard, DetectionCardCompact } from './DetectionCard';
import { useDetectionContext } from '@/context/DetectionContext';
import type { Detection, DetectionSortBy, DetectionSortOrder } from '@/types/detection';
import type { SensorType } from '@/lib/design-tokens';
import {
  Thermometer,
  Radio,
  Wind,
  ArrowUpDown,
  Filter,
  X,
  AlertCircle,
} from 'lucide-react';
import { Badge } from '@/components/ui/badge';

interface DetectionListProps {
  /** Use compact card variant */
  compact?: boolean;
  /** Maximum height of scrollable area */
  maxHeight?: string;
  /** Show filter controls */
  showFilters?: boolean;
  /** Show sort controls */
  showSort?: boolean;
  /** On detection focus (pan camera) */
  onDetectionFocus?: (detection: Detection) => void;
  /** Custom filter (additional to context filter) */
  customFilter?: (detection: Detection) => boolean;
  className?: string;
}

const sensorFilterOptions: { type: SensorType; label: string; icon: React.ReactNode }[] = [
  { type: 'thermal', label: 'Thermal', icon: <Thermometer className="w-3.5 h-3.5" /> },
  { type: 'acoustic', label: 'Acoustic', icon: <Radio className="w-3.5 h-3.5" /> },
  { type: 'gas', label: 'Gas', icon: <Wind className="w-3.5 h-3.5" /> },
];

const sortOptions: { value: DetectionSortBy; label: string }[] = [
  { value: 'time', label: 'Time' },
  { value: 'confidence', label: 'Confidence' },
  { value: 'type', label: 'Type' },
];

export function DetectionList({
  compact = false,
  maxHeight = '100%',
  showFilters = true,
  showSort = true,
  onDetectionFocus,
  customFilter,
  className,
}: DetectionListProps) {
  const {
    filteredDetections,
    filter,
    setFilter,
    sortBy,
    sortOrder,
    setSort,
    confirmDetection,
    dismissDetection,
    clearFilters,
  } = useDetectionContext();

  const [expandedId, setExpandedId] = useState<string | null>(null);

  // Apply custom filter if provided
  const displayDetections = useMemo(() => {
    if (!customFilter) return filteredDetections;
    return filteredDetections.filter(customFilter);
  }, [filteredDetections, customFilter]);

  // Toggle sensor type filter
  const toggleSensorFilter = (sensorType: SensorType) => {
    const current = filter.sensorTypes || [];
    const newTypes = current.includes(sensorType)
      ? current.filter((t) => t !== sensorType)
      : [...current, sensorType];

    setFilter({
      ...filter,
      sensorTypes: newTypes.length > 0 ? newTypes : undefined,
    });
  };

  // Toggle sort order
  const toggleSortOrder = () => {
    setSort(sortBy, sortOrder === 'asc' ? 'desc' : 'asc');
  };

  // Change sort field
  const handleSortChange = (newSortBy: DetectionSortBy) => {
    setSort(newSortBy, sortOrder);
  };

  const hasActiveFilters =
    (filter.sensorTypes && filter.sensorTypes.length > 0) ||
    filter.minConfidence !== undefined ||
    filter.vehicleId !== undefined;

  return (
    <div className={cn('flex flex-col h-full', className)}>
      {/* Filters */}
      {showFilters && (
        <div className="flex-none px-3 py-2 border-b border-glass-border bg-surface-1/50">
          <div className="flex items-center justify-between mb-2">
            <div className="flex items-center gap-1.5 text-xs font-medium text-muted-foreground">
              <Filter className="w-3.5 h-3.5" />
              Filters
            </div>
            {hasActiveFilters && (
              <button
                onClick={clearFilters}
                className="text-xs text-muted-foreground hover:text-foreground transition-colors"
              >
                Clear
              </button>
            )}
          </div>

          {/* Sensor type filters */}
          <div className="flex flex-wrap gap-1.5">
            {sensorFilterOptions.map((option) => {
              const isActive = filter.sensorTypes?.includes(option.type) ?? false;
              return (
                <button
                  key={option.type}
                  onClick={() => toggleSensorFilter(option.type)}
                  className={cn(
                    'flex items-center gap-1 px-2 py-1 rounded-md text-xs font-medium transition-all',
                    'border',
                    isActive
                      ? option.type === 'thermal'
                        ? 'bg-sensor-thermal/20 border-sensor-thermal/40 text-sensor-thermal'
                        : option.type === 'acoustic'
                        ? 'bg-sensor-acoustic/20 border-sensor-acoustic/40 text-sensor-acoustic'
                        : 'bg-sensor-gas/20 border-sensor-gas/40 text-sensor-gas'
                      : 'bg-surface-2 border-glass-border text-muted-foreground hover:text-foreground'
                  )}
                >
                  {option.icon}
                  {option.label}
                </button>
              );
            })}
          </div>
        </div>
      )}

      {/* Sort */}
      {showSort && (
        <div className="flex-none px-3 py-2 border-b border-glass-border bg-surface-1/50">
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-2">
              <span className="text-xs font-medium text-muted-foreground">Sort by:</span>
              <select
                value={sortBy}
                onChange={(e) => handleSortChange(e.target.value as DetectionSortBy)}
                className={cn(
                  'text-xs font-medium px-2 py-1 rounded-md',
                  'bg-surface-2 border border-glass-border',
                  'text-foreground',
                  'focus:outline-none focus:ring-2 focus:ring-ring'
                )}
              >
                {sortOptions.map((option) => (
                  <option key={option.value} value={option.value}>
                    {option.label}
                  </option>
                ))}
              </select>
            </div>

            <button
              onClick={toggleSortOrder}
              className={cn(
                'flex items-center gap-1 px-2 py-1 rounded-md text-xs font-medium',
                'bg-surface-2 hover:bg-surface-3 transition-colors'
              )}
            >
              <ArrowUpDown className="w-3.5 h-3.5" />
              {sortOrder === 'asc' ? 'Asc' : 'Desc'}
            </button>
          </div>
        </div>
      )}

      {/* Detection count */}
      <div className="flex-none px-3 py-1.5 text-xs text-muted-foreground bg-surface-1/30">
        {displayDetections.length} detection{displayDetections.length !== 1 ? 's' : ''}
      </div>

      {/* Detection list */}
      <div
        className="flex-1 overflow-y-auto overflow-x-hidden px-3 py-2"
        style={{ maxHeight }}
      >
        <AnimatePresence mode="popLayout">
          {displayDetections.length === 0 ? (
            <motion.div
              initial={{ opacity: 0 }}
              animate={{ opacity: 1 }}
              exit={{ opacity: 0 }}
              className="flex flex-col items-center justify-center h-full text-center py-8"
            >
              <AlertCircle className="w-12 h-12 text-muted-foreground mb-3 opacity-50" />
              <p className="text-sm font-medium text-foreground mb-1">No detections</p>
              <p className="text-xs text-muted-foreground">
                {hasActiveFilters
                  ? 'Try adjusting your filters'
                  : 'Waiting for sensor data...'}
              </p>
            </motion.div>
          ) : (
            <motion.div
              variants={listContainerVariants}
              initial="hidden"
              animate="visible"
              exit="exit"
              className="space-y-2"
            >
              {displayDetections.map((detection) => {
                const CardComponent = compact ? DetectionCardCompact : DetectionCard;

                return (
                  <motion.div key={detection.id} variants={listItemVariants}>
                    <CardComponent
                      detection={detection}
                      expanded={expandedId === detection.id}
                      onExpand={(expanded) => setExpandedId(expanded ? detection.id : null)}
                      onConfirm={() => confirmDetection(detection.id)}
                      onDismiss={() => dismissDetection(detection.id)}
                      onFocus={() => onDetectionFocus?.(detection)}
                    />
                  </motion.div>
                );
              })}
            </motion.div>
          )}
        </AnimatePresence>
      </div>
    </div>
  );
}
