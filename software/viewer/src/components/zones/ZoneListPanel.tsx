'use client';

/**
 * AERIS GCS Zone List
 * 
 * List of priority zones with status and actions.
 */

import React from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { cn } from '@/lib/utils';
import { listContainerVariants, listItemVariants } from '@/lib/animations';
import { GlassPanel } from '@/components/ui/GlassPanel';
import { useZoneContext } from '@/context/ZoneContext';
import type { PriorityZone } from '@/types/zone';
import {
  getPriorityConfig,
  getZoneStatusConfig,
  calculateZoneArea,
} from '@/types/zone';
import {
  MapPin,
  Check,
  SkipForward,
  RotateCcw,
  Trash2,
  Target,
} from 'lucide-react';

// ============================================================================
// Zone Card
// ============================================================================

interface ZoneCardProps {
  zone: PriorityZone;
  isSelected: boolean;
  onSelect: () => void;
  onFocus?: () => void;
}

function ZoneCard({ zone, isSelected, onSelect, onFocus }: ZoneCardProps) {
  const { completeZone, skipZone, reactivateZone, deleteZone } = useZoneContext();
  
  const priorityConfig = getPriorityConfig(zone.priority);
  const statusConfig = getZoneStatusConfig(zone.status);
  const area = calculateZoneArea(zone.polygon);
  
  return (
    <motion.div
      variants={listItemVariants}
      layout
      className={cn(
        'rounded-lg overflow-hidden',
        'bg-surface-2/50 hover:bg-surface-2',
        'border border-transparent transition-all',
        isSelected && 'border-info/50 bg-info/5'
      )}
    >
      {/* Header */}
      <div
        onClick={onSelect}
        className="flex items-center gap-3 p-3 cursor-pointer"
      >
        {/* Priority indicator */}
        <div
          className={cn(
            'w-8 h-8 rounded-lg flex items-center justify-center',
            priorityConfig.bgColor
          )}
        >
          <MapPin className={cn('w-4 h-4', priorityConfig.color)} />
        </div>
        
        {/* Info */}
        <div className="flex-1 min-w-0">
          <div className="flex items-center gap-2">
            <span className="font-medium text-sm text-foreground truncate">
              {zone.name}
            </span>
            <span className={cn(
              'text-xs px-1.5 py-0.5 rounded',
              priorityConfig.bgColor,
              priorityConfig.color
            )}>
              P{zone.priority}
            </span>
          </div>
          <div className="flex items-center gap-2 mt-0.5 text-xs text-muted-foreground">
            <span className={cn(statusConfig.color)}>{statusConfig.label}</span>
            <span>•</span>
            <span>{area.toFixed(0)} m²</span>
          </div>
        </div>
        
        {/* Focus button */}
        {onFocus && (
          <button
            onClick={(e) => {
              e.stopPropagation();
              onFocus();
            }}
            className={cn(
              'p-2 rounded-md',
              'text-muted-foreground hover:text-foreground hover:bg-surface-3',
              'transition-colors'
            )}
          >
            <Target className="w-4 h-4" />
          </button>
        )}
      </div>
      
      {/* Actions (visible when selected) */}
      <AnimatePresence>
        {isSelected && (
          <motion.div
            initial={{ height: 0, opacity: 0 }}
            animate={{ height: 'auto', opacity: 1 }}
            exit={{ height: 0, opacity: 0 }}
            className="overflow-hidden"
          >
            <div className="flex gap-2 px-3 pb-3">
              {zone.status === 'active' && (
                <>
                  <button
                    onClick={() => completeZone(zone.id)}
                    className={cn(
                      'flex-1 flex items-center justify-center gap-1.5 px-2 py-2',
                      'bg-success/10 hover:bg-success/20 border border-success/30',
                      'text-success text-xs font-medium rounded-lg',
                      'transition-colors min-h-[var(--touch-min)]'
                    )}
                  >
                    <Check className="w-3.5 h-3.5" />
                    Complete
                  </button>
                  <button
                    onClick={() => skipZone(zone.id)}
                    className={cn(
                      'flex-1 flex items-center justify-center gap-1.5 px-2 py-2',
                      'bg-surface-3 hover:bg-surface-4',
                      'text-muted-foreground text-xs font-medium rounded-lg',
                      'transition-colors min-h-[var(--touch-min)]'
                    )}
                  >
                    <SkipForward className="w-3.5 h-3.5" />
                    Skip
                  </button>
                </>
              )}
              
              {(zone.status === 'completed' || zone.status === 'skipped') && (
                <button
                  onClick={() => reactivateZone(zone.id)}
                  className={cn(
                    'flex-1 flex items-center justify-center gap-1.5 px-2 py-2',
                    'bg-info/10 hover:bg-info/20 border border-info/30',
                    'text-info text-xs font-medium rounded-lg',
                    'transition-colors min-h-[var(--touch-min)]'
                  )}
                >
                  <RotateCcw className="w-3.5 h-3.5" />
                  Reactivate
                </button>
              )}
              
              <button
                onClick={() => deleteZone(zone.id)}
                className={cn(
                  'flex items-center justify-center gap-1.5 px-2 py-2',
                  'bg-danger/10 hover:bg-danger/20 border border-danger/30',
                  'text-danger text-xs font-medium rounded-lg',
                  'transition-colors min-h-[var(--touch-min)]'
                )}
              >
                <Trash2 className="w-3.5 h-3.5" />
              </button>
            </div>
          </motion.div>
        )}
      </AnimatePresence>
    </motion.div>
  );
}

// ============================================================================
// Zone List Panel
// ============================================================================

interface ZoneListPanelProps {
  onZoneFocus?: (zone: PriorityZone) => void;
  className?: string;
}

export function ZoneListPanel({ onZoneFocus, className }: ZoneListPanelProps) {
  const { zones, activeZones, selectedZoneId, selectZone } = useZoneContext();
  
  const completedCount = zones.filter(z => z.status === 'completed').length;
  
  if (zones.length === 0) {
    return (
      <GlassPanel
        title="Priority Zones"
        icon={<MapPin className="w-4 h-4" />}
        collapsible
        defaultCollapsed={false}
        className={cn('w-[280px] pointer-events-auto', className)}
      >
        <div className="flex flex-col items-center justify-center py-6 text-center">
          <MapPin className="w-8 h-8 text-muted-foreground/30 mb-2" />
          <p className="text-sm text-muted-foreground">No zones defined</p>
          <p className="text-xs text-muted-foreground/70 mt-1">
            Use the drawing tool to create zones
          </p>
        </div>
      </GlassPanel>
    );
  }
  
  return (
    <GlassPanel
      title="Priority Zones"
      icon={<MapPin className="w-4 h-4" />}
      collapsible
      defaultCollapsed={false}
      className={cn('w-[280px] pointer-events-auto', className)}
      headerRight={
        <span className="text-xs font-mono text-muted-foreground">
          {activeZones.length}/{zones.length}
        </span>
      }
    >
      <div className="space-y-3">
        {/* Stats */}
        <div className="flex items-center gap-3 text-xs">
          <span className="text-muted-foreground">
            Active: <span className="text-info font-medium">{activeZones.length}</span>
          </span>
          <span className="text-muted-foreground">
            Completed: <span className="text-success font-medium">{completedCount}</span>
          </span>
        </div>
        
        {/* Zone list */}
        <motion.div
          variants={listContainerVariants}
          initial="hidden"
          animate="visible"
          className="space-y-2 max-h-[300px] overflow-y-auto -mx-1 px-1"
        >
          <AnimatePresence mode="popLayout">
            {zones.map((zone) => (
              <ZoneCard
                key={zone.id}
                zone={zone}
                isSelected={selectedZoneId === zone.id}
                onSelect={() => selectZone(selectedZoneId === zone.id ? null : zone.id)}
                onFocus={onZoneFocus ? () => onZoneFocus(zone) : undefined}
              />
            ))}
          </AnimatePresence>
        </motion.div>
      </div>
    </GlassPanel>
  );
}

export default ZoneListPanel;
